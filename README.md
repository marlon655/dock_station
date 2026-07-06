# aruco_docking

Pacote separado para docking/carregamento automático com ArUco + LiDAR.

Ele deve ser iniciado depois da simulação (`sim_bot`) e do nó operacional de
route graph (`nav_hub`). O valor de bateria é manual: o pacote não simula
descarga/carga automática. Você informa a porcentagem por parâmetro.

## Compilar

Na raiz do workspace:

```bash
cd ~/sim_ws
colcon build --packages-select aruco_docking
source install/setup.bash
```

Se ainda não compilou os outros pacotes:

```bash
cd ~/sim_ws
colcon build
source install/setup.bash
```

## Iniciar Na Simulação

A simulação completa usa três launches, em três terminais. Em todos eles:

```bash
cd ~/sim_ws
source install/setup.bash
```

Terminal 1: sobe Gazebo, robô, sensores, RViz e Nav2 com route_server:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

Terminal 2: sobe a camada operacional do route graph. Esse nó escuta
`/destination` e publica o destino em `/goal_pose`:

```bash
ros2 launch nav_hub sim_main_route_graph.launch.py
```

Terminal 3: sobe o docking/ArUco separado e insere o dock no Gazebo:

```bash
ros2 launch aruco_docking docking_sim.launch.py battery:=100.0
```

O `route_server` do Nav2 é iniciado pelo primeiro launch (`sim_bot`) quando
`route:=true`. O segundo launch não substitui o Nav2; ele é a camada de destino
operacional por ID.

Exemplo iniciando com bateria em 30%:

```bash
ros2 launch aruco_docking docking_sim.launch.py battery:=30.0
```

A launch de simulação pode inserir o modelo `charging_dock` no Gazebo já em
execução. Se o dock já estiver no mundo, desative o spawn:

```bash
ros2 launch aruco_docking docking_sim.launch.py spawn_gazebo_dock:=false
```


## Onde Fica A Tag ArUco?

A tag ArUco **nao fica no mapa 2D** do Nav2. O mapa `pgm/yaml` usado por
AMCL/Nav2 serve para localizacao e planejamento. A camera nao le esse mapa.

Na simulacao, a tag vem do modelo Gazebo `charging_dock` instalado neste pacote:

```text
aruco_docking/models/charging_dock/model.sdf
aruco_docking/models/charging_dock/materials/textures/aruco_marker_771.png
```

Quando voce roda:

```bash
ros2 launch aruco_docking docking_sim.launch.py
```

a launch tenta inserir esse modelo no Gazebo ja aberto pelo `sim_bot`. O modelo
possui uma textura ArUco ID `771` na face frontal. A camera simulada publica
`/camera/image`, o `dock_pose_estimator` detecta a tag nessa imagem e publica
`/detected_dock_pose`.

No robo real/NUC, nao existe Gazebo nem textura simulada: a tag precisa estar
fisicamente impressa e fixada no carregador, com o mesmo `marker_id` e tamanho
informado por `marker_size`.

Se o modelo do dock ja estiver no mundo `.world`, rode com:

```bash
ros2 launch aruco_docking docking_sim.launch.py spawn_gazebo_dock:=false
```


### Inserir O Dock Em Uma Pose Especifica

Na simulacao, a pose do dock fica no arquivo:

```text
src/aruco_docking/config/docking_params_sim.yaml
```

Altere este bloco:

```yaml
base_carregamento:
  type: 'charging_dock'
  frame: 'map'
  pose: [4.30, -24.57, -1.849560]
```

A ordem e `[x, y, w]`, onde `w` e o yaw/orientacao em radianos. Na simulacao,
essa pose deve estar no mesmo frame global usado pelo Nav2/route graph (`map`).
A launch usa a mesma pose para inserir o modelo `charging_dock` no Gazebo e para
configurar o `docking_server`.

## Ajustar Bateria Com O Nó Rodando

Para mudar a porcentagem em runtime:

```bash
ros2 param set /charging_manager battery 30.0
```

Para ler o valor publicado:

```bash
ros2 topic echo /battery_level --once
```

Parâmetros de bateria/decisão disponíveis:

```bash
battery:=30.0          # porcentagem atual/inicial
min_for_task:=40.0     # mínimo para aceitar tarefa
low_after:=20.0        # quando battery <= low_after, volta para carregar
emergency:=5.0         # bateria crítica durante tarefa
```

## Robô Real / NUC

No robô real, primeiro suba a navegação/hardware e depois o docking:

```bash
ros2 launch nav_hub essentials.launch.py mode:=hardware
```

```bash
ros2 launch aruco_docking docking_real.launch.py marker_size:=0.15 stop_distance:=0.30 battery:=100.0
```

No hardware real, `spawn_gazebo_dock` fica `false`. Ajuste a pose real do
carregador em `config/docking_params_real.yaml`, no frame fixo usado pelo Nav2.

## Tópicos Esperados

- `/camera/image`
- `/camera/camera_info`
- `/scan`
- `/odom`
- TF conectando `map -> odom -> base_link/base_footprint -> camera_link_optical` e `laser_frame`

## Tópicos Publicados

- `/detected_dock_pose`
- `/charging_manager/state`
- `/battery_level` — replica o valor do parâmetro `battery`

## Forçar Retorno Ao Carregador

```bash
ros2 topic pub --once /go_charge std_msgs/msg/Empty '{}'
```
