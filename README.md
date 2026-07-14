# dock_robot_ws

Workspace ROS 2 minimo para executar o docking por LiDAR no robo real.

## Pacotes incluidos

- `lidar_auto_docking`: servidor das actions `Dock` e `Undock`, percepcao do
  dock pelo `/scan` e controle final pelo `/cmd_vel`.
- `lidar_auto_docking_messages`: mensagens e actions usadas pelo servidor e
  pelos clientes.

`nav_hub` e `sim_bot` nao fazem parte deste workspace.

## Dependencias esperadas do robo

- ROS 2 e `colcon` instalados;
- LiDAR publicando `sensor_msgs/msg/LaserScan` em `/scan`;
- TF disponivel entre `map`, `odom` e `base_link`;
- odometria em `/odom`;
- base aceitando `geometry_msgs/msg/Twist` em `/cmd_vel`;
- Nav2 somente se for usada a navegacao ate uma pose antes do docking.

## Instalar dependencias e compilar

```bash
cd ~/dock_robot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Executar o docking manual

Primeiro inicie os drivers, a odometria, a arvore TF e a localizacao do robo.
Depois, em um terminal:

```bash
cd ~/dock_robot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch lidar_auto_docking autodock_launch.py
```

Quando o robo ja estiver proximo e alinhado com o dock, envie o objetivo em
outro terminal:

```bash
cd ~/dock_robot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch lidar_auto_docking dock_ws_send_goal.launch.py
```

A pose real do dock deve ser configurada em
`src/lidar_auto_docking/initial_dock_pose/dock_ws_dock.json`.

## Integracao automatica

`dock_ws_auto_start.launch.py` espera `/destination=99` e
`/has_reached=true`, cancela a action `navigate_to_pose`, publica velocidade
zero e inicia o docking. Esses topicos precisam ser publicados pelo sistema de
navegacao do robo; eles nao sao fornecidos por este workspace minimo.

> O controlador de docking publica diretamente em `/cmd_vel`. Use um mux ou
> outra arbitragem de comandos no robo real para impedir que Nav2, teleoperacao
> e docking comandem a base simultaneamente.
