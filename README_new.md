Qground control

```bash
cd ~/qgroundcontrol
export DISPLAY=host.docker.internal:1
./AppRun
```

Запуск симуляции
```bash
sudo chown -R arduuser:arduuser /aas
nano launch_sim.sh
chmod +x launch_sim.sh
./launch_sim.sh
```

Build
```bash
docker-compose build
```
Start
```bash
docker-compose up
```