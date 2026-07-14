# Dock Station — robô real

Workspace ROS 2 para detectar uma dock trapezoidal com o LiDAR, salvar sua
pose no mapa e executar o docking no robô real.

## Pacotes

- `lidar_auto_docking`: percepção, servidor das actions `Dock` e `Undock`,
  salvamento da pose e controle de aproximação.
- `lidar_auto_docking_messages`: mensagens e actions utilizadas pelo sistema.

Este workspace não inclui `nav_hub` nem `sim_bot`.

## 1. Requisitos do robô

Antes de usar o docking, o sistema principal do robô deve fornecer:

- LaserScan no tópico `/scan`;
- odometria em `/odom`;
- TF válido entre `map`, `odom`, `base_link` e o frame do LiDAR;
- localização do robô no mapa;
- uma entrada de velocidade para o docking (`/cmd_vel` ou um mux configurado);
- o trapézio físico na altura do plano de leitura do LiDAR.

Os comandos abaixo consideram ROS 2 Jazzy. Para outra distribuição instalada,
substitua `jazzy` pelo nome correspondente.

## 2. Instalação em um NUC novo

Instale o ROS 2, `colcon`, `rosdep` e Git. Depois clone somente a branch do
robô:

```bash
git clone --branch robot_dev --single-branch \
  https://github.com/marlon655/dock_station.git ~/dock_robot_ws

cd ~/dock_robot_ws
source /opt/ros/jazzy/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/local_setup.bash
```

Em cada terminal novo, carregue o ambiente:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dock_robot_ws/install/local_setup.bash
```

## 3. Iniciar o sistema principal do robô

Inicie os drivers, LiDAR, odometria, TF, localização/Nav2 e o controle da base.
Antes de continuar, confira:

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map base_link
```

O TF `map -> base_link` deve aparecer continuamente e sem erros.

## 4. Salvar a pose da dock

Esta etapa deve ser executada ao instalar uma dock nova, trocar o mapa ou
mover fisicamente a dock.

Posicione o robô de frente, alinhado e aproximadamente entre `0.5 m` e `1.0 m`
do trapézio.

### Terminal 1 — detector e serviço de salvamento

```bash
source /opt/ros/jazzy/setup.bash
source ~/dock_robot_ws/install/local_setup.bash

ros2 launch lidar_auto_docking dockpose_saver_launch.py
```

### Terminal 2 — interface de salvamento

```bash
source /opt/ros/jazzy/setup.bash
source ~/dock_robot_ws/install/local_setup.bash

ros2 run lidar_auto_docking dock_pose_saver_cli
```

Na interface:

1. Pressione `R` para carregar os dados mais recentes.
2. Confirme que aparece `Dock detectado: SIM` e que a pose no frame `map` está
   disponível.
3. Pressione `S` para salvar.
4. Digite `s` para confirmar.
5. Aguarde as mensagens `[OK]` e pressione `C` para sair.
6. Encerre o Terminal 1 com `Ctrl+C`.

A pose é gravada em:

```text
~/dock_robot_ws/src/lidar_auto_docking/initial_dock_pose/dock_ws_dock.json
```

Um novo salvamento substitui o JSON anterior de forma atômica. Se a detecção,
o TF ou a gravação falhar, o arquivo anterior é preservado.

Para conferir o arquivo:

```bash
cat ~/dock_robot_ws/src/lidar_auto_docking/initial_dock_pose/dock_ws_dock.json
```

## 5. Executar o docking

Antes de enviar o docking, use Nav2 ou conduza o robô até ficar de frente para
a dock, aproximadamente entre `1.0 m` e `1.5 m` dela.

### Iniciar o servidor e enviar o goal

```bash
source /opt/ros/jazzy/setup.bash
source ~/dock_robot_ws/install/local_setup.bash

ros2 launch lidar_auto_docking dockrobot_launch.py
```

Esse launch inicia o servidor `auto_dock`, lê `dock_ws_dock.json` e envia
automaticamente a action `/Dock`. O controlador publica diretamente em
`/cmd_vel`.

Mantenha esse terminal aberto durante toda a operação. O resultado esperado
termina com o goal concluído e `DOCK REACHED`.

### Executar o undocking

Depois que o docking terminar, mantenha `dockrobot_launch.py` aberto e execute
em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dock_robot_ws/install/local_setup.bash

ros2 run lidar_auto_docking undock_robot.py
```

Não inicie outro servidor enquanto `dockrobot_launch.py` estiver rodando.

## 6. Alterar parâmetros

Os parâmetros principais estão em:

```text
src/lidar_auto_docking/config/autodock_params.yaml
```

Como o workspace é compilado com `--symlink-install`, normalmente basta salvar
o YAML e reiniciar o launch. Alterações em C++, `CMakeLists.txt`, mensagens ou
actions exigem um novo build:

```bash
cd ~/dock_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

## Segurança

Não permita que Nav2, teleoperação e docking comandem a base simultaneamente.
Use um mux ou outro mecanismo de arbitragem de velocidade e mantenha uma forma
de parada de emergência disponível durante os testes.
