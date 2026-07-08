# lidar_docking

Pacote de docking proprio usando somente LiDAR 2D para a aproximacao final em
um painel plano de carregamento de 40 x 40 cm.

Fluxo:

1. O `lidar_docking_manager` publica a sequencia do ponto de staging em
   `/destination`.
2. O `sim_main_route_graph` do `nav_hub` transforma essa sequencia em
   `/goal_pose` para o Nav2.
3. Quando `/has_reached` fica `true`, o manager assume o controle fino.
4. O `lidar_dock_detector` detecta a face plana do dock no `/scan`.
5. O manager centraliza o robo e avanca via `/cmd_vel` ate a distancia final.

Nao ha guia fisico nem dependencia de ArUco/camera. O grafo so precisa deixar o
robo aproximadamente na frente do painel; a partir dai o controle final fica em
malha fechada no LiDAR. Se o erro lateral ou angular estiver alto, o robo gira
parado. Quando fica alinhado o suficiente, ele avanca bem devagar e continua
corrigindo a cada leitura.

## Uso

Compile e carregue o overlay:

```bash
colcon build --packages-select lidar_docking
source install/setup.bash
```

Suba a navegacao/grafo normalmente e depois:

```bash
ros2 launch lidar_docking lidar_docking.launch.py dock_destination_sequence:=16
```

Para iniciar o retorno ao dock manualmente:

```bash
ros2 topic pub --once /go_lidar_dock std_msgs/msg/Empty "{}"
```

## Parametros principais

- `dock_destination_sequence`: ponto do grafo que deixa o robo de frente para o
  dock, idealmente entre 0.8 m e 1.2 m do painel.
- `panel_width`: largura esperada do painel visto pelo LiDAR. Default: `0.40`.
- `robot_length`: comprimento do robo. Default: `0.50`.
- `contact_clearance`: folga entre a frente do robo e o painel no fim do
  docking. Default: `0.03`.
- `drive_lateral_tolerance`: erro lateral maximo para permitir avanco.
- `drive_yaw_tolerance_deg`: erro angular maximo para permitir avanco.
- `max_linear_speed`: velocidade maxima na aproximacao final. Default: `0.05`.
- `sector_half_deg`: abertura frontal usada para procurar o painel.

O detector publica `detected_dock_pose` no frame `base_link`, codificando:

- `pose.position.x`: distancia ate a face do painel.
- `pose.position.y`: erro lateral do centro do painel.
- `pose.orientation`: yaw da face plana detectada.
