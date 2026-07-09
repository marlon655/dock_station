# dock_lidar_ws

Workspace ROS 2 para testar autodocking por LiDAR no mundo `aceleradora` com o robô `sim_bot`.

Este workspace integra:

- `lidar_auto_docking`: pacote de autodocking por LiDAR.
- `lidar_auto_docking_messages`: mensagens/actions usadas pelo autodocking.
- `sim_bot`: simulação Gazebo do robô e mundo.
- `nav_hub`: mapa e localização usados pela simulação.

Referência do pacote base:

- https://github.com/SynapseProgramming/lidar_auto_docking

## Build

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Sequência de uso atual

Use terminais separados.

### 1. Subir simulação

Sobe Gazebo, RViz, mapa e localização, sem navegação:

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sim_bot sim_map_only.launch.py
```

Para nascer perto do dock:

```bash
ros2 launch sim_bot sim_map_only.launch.py spawn_x:=-6.39 spawn_y:=12.98
```

### 2. Subir servidor de autodocking

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_auto_docking autodock_launch.py
```

### 3. Enviar goal de docking

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_auto_docking dock_ws_send_goal.launch.py
```

### 4. Teleop manual

Use apenas para posicionar o robô antes do docking:

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Durante o autodocking, não use teleop ao mesmo tempo, porque ambos publicam em `/cmd_vel`.

## Saver da pose do dock

Para visualizar/salvar a pose estimada do dock:

```bash
cd /home/ros_estudo/dock_lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_auto_docking dockpose_saver_launch.py
```

O arquivo de pose usado pelos launches customizados é:

```text
src/lidar_auto_docking/initial_dock_pose/dock_ws_dock.json
```

## Ajustes atuais de docking

Arquivo:

```text
src/lidar_auto_docking/config/autodock_params.yaml
```

Parâmetros principais:

```yaml
docked_distance_threshold: 0.30
dock_lateral_offset: -0.05
dock_yaw_offset: 0.03
retries: 10
docking_timeout_sec: 300.0
```

- `docked_distance_threshold`: distância final em que o docking considera sucesso.
- `dock_lateral_offset`: correção lateral fina no frame do dock.
- `dock_yaw_offset`: correção angular fina no alvo final.
- `docking_timeout_sec`: tempo máximo da action de docking.

## Observações

O dock trapezoidal já está inserido no mundo:

```text
src/sim_bot/worlds/aceleradora.world
```

Pose do dock no mundo:

```xml
<pose>-7.57 13.21 0 1.57 0 4.5162</pose>
```
