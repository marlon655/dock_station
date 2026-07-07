# dock_station

Repositório com os pacotes de docking por ArUco/LiDAR do Palmares.

## Pacotes

- `aruco_docking` (`aruco_docking_sim/`): pacote para simulação, com launch de Gazebo, modelo do dock e parâmetros simulados.
- `aruco_docking_real`: pacote separado para o robô real, sem Gazebo e sem dependência do `sim_bot`.

## Simulação

```bash
cd ~/sim_ws
colcon build --packages-select aruco_docking
source install/setup.bash
ros2 launch aruco_docking docking_sim.launch.py
```

## Robô Real

Copie este repositório para o workspace do NUC e compile:

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_docking_real
source install/setup.bash
ros2 launch aruco_docking_real docking_real.launch.py
```

Se a câmera real usar outros tópicos:

```bash
ros2 launch aruco_docking_real docking_real.launch.py \
  camera_image_topic:=/color/image_raw \
  camera_info_topic:=/color/camera_info \
  scan_topic:=/scan \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel
```
