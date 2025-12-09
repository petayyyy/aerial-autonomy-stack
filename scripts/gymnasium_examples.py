import numpy as np
import gymnasium as gym
import argparse
import time
import itertools

from gymnasium.utils.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env as sb3_check_env

from aas_gym.aas_env import AASEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="learn", choices=["step", "speed", "learn"])
    args = parser.parse_args()

    # Register the environment so we can create it with gym.make()
    gym.register(
        id="AASEnv-v0",
        entry_point=AASEnv,
    )

    if args.mode == "step":
        env = gym.make("AASEnv-v0", gym_freq_hz=1, render_mode="human")
        obs, info = env.reset()
        print(f"Reset result -- Obs: {obs}")
        for i in itertools.count():
            user_input = input("Press Enter to step, 'r' then Enter to reset, 'q' then Enter to exit...")
            stripped_input = user_input.strip().lower()
            if stripped_input and stripped_input in ('q', 'quit'):
                break
            if stripped_input and stripped_input in ('r', 'reset'):
                obs, info = env.reset()
                print(f"\nReset result -- Obs: {obs}")
            else:
                rnd_action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(rnd_action)
                print(f"\nStep {i} -- action: {rnd_action} result -- Obs: {obs}, Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}")
        print("\nClosing environment.")
        env.close()

    elif args.mode == "speed":
        CTRL_FREQ_HZ = 50
        env = gym.make("AASEnv-v0", instance=1, gym_freq_hz=CTRL_FREQ_HZ, render_mode="ansi")
        TIME_TO_SIMULATE_SEC = 200
        STEPS = TIME_TO_SIMULATE_SEC * CTRL_FREQ_HZ
        print(f"Starting Speed Test ({STEPS} steps)")    
        obs, info = env.reset()
        start_time = time.time()        
        for _ in range(STEPS):
            action = env.action_space.sample()            
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                obs, info = env.reset()
        total_time = time.time() - start_time
        print(f"\nTest completed in: {(total_time):.2f} sec")
        print(f"Average Step Time: {(total_time / STEPS) * 1000:.3f} ms")
        print(f"Speedup: {(TIME_TO_SIMULATE_SEC / total_time):.2f}x wall-clock")
        print(f"Throughput: {(STEPS / total_time):.2f} steps/second")
        env.close()

    elif args.mode == "learn":
        print(f"TODO")
        # env = gym.make("AASEnv-v0")
        # try:
        #     # check_env(env) # Throws warning
        #     # check_env(env.unwrapped)
        #     sb3_check_env(env)
        #     print("\nEnvironment passes all checks!")
        # except Exception as e:
        #     print(f"\nEnvironment has issues: {e}")

        # env.reset()
        # env.step(env.action_space.sample())
        # env.reset()

        # # Instantiate the agent
        # model = PPO("MlpPolicy", env, verbose=1, device='cpu')

        # # Train the agent
        # print("Training agent...")
        # model.learn(total_timesteps=40000)
        # print("Training complete.")

        # # Save the agent
        # model_path = "ppo_agent.zip"
        # model.save(model_path)
        # print(f"Model saved to {model_path}")

        # # Load and test the trained agent
        # del model # remove to demonstrate loading
        # model = PPO.load(model_path, device='cpu')

        # print("\nTesting trained agent...")
        # obs, info = env.reset()
        # for _ in range(800): # Run for 800 steps
        #     action, _states = model.predict(obs, deterministic=True)
        #     obs, reward, terminated, truncated, info = env.step(action)
            
        #     if terminated or truncated:
        #         print("Episode finished. Resetting.")
        #         obs, info = env.reset()
        
        # env.close()

    else:
        print(f"Unknown mode: {args.mode}")

if __name__ == '__main__':
    main()
