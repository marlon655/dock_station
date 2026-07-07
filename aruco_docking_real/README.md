# aruco_docking_real

Pacote separado para usar o docking por ArUco/LiDAR no robô real, sem Gazebo e sem dependência do `sim_bot`.

## Como usar no NUC

Copie esta pasta para o workspace do robô, por exemplo:

```bash
~/ros2_ws/src/aruco_docking_real
```

Depois compile:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select aruco_docking_real
source install/setup.bash
```

Suba primeiro o bringup real do robô e a navegação (`nav_hub`/Nav2). Depois lance:

```bash
ros2 launch aruco_docking_real docking_real.launch.py
```

Para iniciar com outro valor de bateria:

```bash
ros2 launch aruco_docking_real docking_real.launch.py battery:=35.0
```

Para usar tópicos reais com nomes diferentes:

```bash
ros2 launch aruco_docking_real docking_real.launch.py   camera_image_topic:=/color/image_raw   camera_info_topic:=/color/camera_info   scan_topic:=/scan   odom_topic:=/odom   cmd_vel_topic:=/cmd_vel
```

## O que precisa existir no robô

- `/camera/image` ou remap equivalente
- `/camera/camera_info` ou remap equivalente
- `/scan` ou remap equivalente
- `/tf` e `/tf_static`
- `/odom`
- `/cmd_vel`
- action `/navigate_to_pose` do Nav2
- action `/dock_robot` fornecida pelo `opennav_docking`

Também precisa existir TF correto entre:

```text
base_link -> camera_link -> camera_link_optical
base_link -> laser_frame
odom/map -> base_link
```

## Arquivos principais

- `launch/docking_real.launch.py`: launch principal do robô real.
- `config/docking_params_real.yaml`: parâmetros de pose do dock, ArUco, controlador e bateria.
- `nodes/dock_pose_estimator.py`: detecta ArUco/LiDAR e publica `/detected_dock_pose`.
- `nodes/charging_manager.py`: decide retorno ao dock e chama a action `/dock_robot`.

## Teste manual

Com o launch rodando, force retorno para a dock:

```bash
ros2 topic pub /go_charge std_msgs/msg/Empty "{}" --once
```

Verifique se o detector está publicando pose:

```bash
ros2 topic echo /detected_dock_pose
```

## Câmera USB opcional

Se o robô real usa `usb_cam`, este pacote também instala um launch auxiliar:

```bash
ros2 launch aruco_docking_real camera_real.launch.py
```

Ele publica/remapeia a câmera para os tópicos esperados pelo detector:

```text
/camera/image
/camera/camera_info
```
