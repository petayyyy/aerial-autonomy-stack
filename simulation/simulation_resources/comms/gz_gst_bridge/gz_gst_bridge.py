"""
Alternative Python implementation of a Gazebo to GStreamer image bridge.

Use as in simulation.yml.erb as:
    python3 /aas/simulation_resources/comms/gz_gst_bridge/gz_gst_bridge.py 
        --gz_topic /world/<%= world %>/model/<%= model_name %>/<%= sensor_path %>model/simple_camera/link/mono_cam/base_link/sensor/imager/image
        --ip <%= "#{sim_subnet}.90.#{drone_id}" %> --port 5600
"""
import argparse
import gi
import signal
gi.require_version('Gst', '1.0')

from gi.repository import Gst, GLib
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

pipeline = None
appsrc = None
main_loop = None

def on_new_gz_frame(msg: Image):
    global appsrc, pipeline

    if appsrc is None:
        return

    if appsrc.get_property('caps') is None:
        print(f"First frame received. Configuring GStreamer pipeline for {msg.width}x{msg.height}...")
        caps_str = f"video/x-raw,format=RGB,width={msg.width},height={msg.height},framerate={args.framerate}/1"
        caps = Gst.Caps.from_string(caps_str)
        appsrc.set_property('caps', caps)

    buffer = Gst.Buffer.new_wrapped(msg.data)
    
    retval = appsrc.emit('push-buffer', buffer)
    if retval != Gst.FlowReturn.OK:
        print("Error pushing buffer to GStreamer")

def check_nvidia_encoder():
    element = Gst.ElementFactory.make('nvh264enc', None)
    if element is not None:
        element.set_state(Gst.State.NULL)
        return True
    return False

def main():
    global pipeline, appsrc, main_loop, args

    parser = argparse.ArgumentParser(description="Bridge a Gazebo camera topic to a GStreamer UDP stream.")
    parser.add_argument('--gz_topic', help="Gazebo Image topic to subscribe to.")
    parser.add_argument('--framerate', type=int, default=10, help="Framerate of the camera sensor (default: 10).")
    parser.add_argument('--ip', help="Destination host for the GStreamer stream.")
    parser.add_argument('--port', type=int, help="Destination port for the GStreamer stream (see pipelines in yolo_inference_node.py).")
    args = parser.parse_args()

    Gst.init(None)

    use_gpu = check_nvidia_encoder()
    if use_gpu:
        pipeline_str = (
            "appsrc name=py_source ! "
            "videoconvert ! "
            "nvh264enc preset=low-latency-hq zerolatency=true gop-size=60 ! " # Use the NVIDIA H.264 encoder, re-send full frame every 60 frames
            "rtph264pay config-interval=1 ! "
            f"udpsink sync=false host={args.ip} port={args.port}"
        )
        print("Using GPU-accelerated GStreamer pipeline")
    else:
        pipeline_str = (
            "appsrc name=py_source ! "
            "videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=500 key-int-max=60 ! " # Optimize CPU use, re-send full frame every 60 frames
            "rtph264pay config-interval=1 ! "
            f"udpsink sync=false host={args.ip} port={args.port}"
        )
        print("[WARNING] Falling back to CPU-based GStreamer pipeline")

    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name('py_source')
    appsrc.set_property('max-bytes', 0) # Disable byte limit
    appsrc.set_property('max-buffers', 2) # Keep only 2 frames max
    appsrc.set_property('leaky-type', 2) # Drop old frames

    node = Node()
    if not node.subscribe(Image, args.gz_topic, on_new_gz_frame):
        print(f"Error subscribing to topic [{args.gz_topic}]. Ensure Gazebo is running and the topic exists.")
        return

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("ERROR: Unable to set pipeline to PLAYING state")
        return
    print(f"GStreamer pipeline started. Streaming to udp://{args.ip}:{args.port}...")
    print(f"Subscribed to Gazebo topic [{args.gz_topic}]. Waiting for frames...")

    main_loop = GLib.MainLoop() 
    signal.signal(signal.SIGINT, lambda s, f: main_loop.quit()) # Handle Ctrl + c
    main_loop.run()

    print("Shutting down...")
    pipeline.set_state(Gst.State.NULL) # Clean up

if __name__ == '__main__':
    main()
