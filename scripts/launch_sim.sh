#!/bin/bash

export DISPLAY=host.docker.internal:0
export HEADLESS=false

# Исправление формата файлов
sed -i 's/\r$//' /aas/simulation_resources/aircraft_models/_create_ardupilot_models.sh 2>/dev/null
sed -i 's/\r$//' /aas/simulation_resources/simulation_worlds/_create_ardupilot_world.sh 2>/dev/null

echo "=== Создание моделей для VTOL... ==="
/aas/simulation_resources/aircraft_models/_create_ardupilot_models.sh 0 1

echo "=== Создание мира Gazebo... ==="
/aas/simulation_resources/simulation_worlds/_create_ardupilot_world.sh 0 1 impalpable_greyness.sdf

echo "=== Проверка созданных файлов... ==="
echo "Модель VTOL существует:" && ls -la /aas/simulation_resources/aircraft_models/alti_transition_quad_1/
echo "SDF мир создан:" && ls -la /aas/simulation_resources/simulation_worlds/populated_ardupilot.sdf

echo "=== Запуск Gazebo с GUI... ==="
# ВАЖНО: добавили путь к aircraft_models!
export GZ_SIM_SYSTEM_PLUGIN_PATH=/aas/github_apps/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH="/aas/simulation_resources/simulation_worlds:/aas/github_apps/ardupilot_gazebo/models:/aas/simulation_resources/aircraft_models"

# Проверьте, что в populated_ardupilot.sdf правильные пути
# Может потребоваться исправить пути в SDF файле
echo "Исправление ссылок в SDF файле..."
sed -i 's|model://alti_transition_quad_1|/aas/simulation_resources/aircraft_models/alti_transition_quad_1|g' /aas/simulation_resources/simulation_worlds/populated_ardupilot.sdf

# Запуск Gazebo БЕЗ флага -s (с GUI)
gz sim -v 4 -r /aas/simulation_resources/simulation_worlds/populated_ardupilot.sdf &
GZ_PID=$!
sleep 15

echo "=== Запуск ArduPilot SITL... ==="
cd /aas/github_apps/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane \
    --model JSON \
    -I 0 \
    --sysid 1 \
    --speedup 10 \
    --add-param-file=/aas/simulation_resources/aircraft_models/alti_transition_quad_1/ardupilot-4.6.params \
    --out=udp:127.0.0.1:14550 \
    --console \
    --map

wait $GZ_PID