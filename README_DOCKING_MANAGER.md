# Uso do `nav2_dock.py` como referência para o nó gerenciador do robô

## 1. Objetivo

O arquivo `nav2_dock.py`, presente no pacote `lidar_auto_docking`, pode ser utilizado como referência para implementar o nó responsável por gerenciar o ciclo completo de entrada e saída da estação de carregamento.

Esse arquivo demonstra como coordenar três operações principais:

1. Navegar até uma pose próxima da estação usando o Nav2.
2. Executar o alinhamento e a aproximação final usando a Action de docking.
3. Executar a saída da estação usando a Action de undocking.

A estrutura geral é:

```text
NavigateToPose
      ↓
Dock
      ↓
Charging
      ↓
Undock
```

O `nav2_dock.py` não é o responsável direto pelo controle dos motores durante o docking. Ele funciona como um coordenador que chama os servidores de Action apropriados.

---

# 2. Actions utilizadas

O arquivo utiliza três clientes de Action.

## 2.1 Navegação até a pré-dock

Action do Nav2:

```text
/navigate_to_pose
```

Tipo:

```text
nav2_msgs/action/NavigateToPose
```

Essa Action leva o robô até uma pose aproximada próxima da estação.

Essa posição é chamada de pré-dock e deve deixar o robô em uma região na qual o LiDAR consiga detectar o trapézio da estação.

Exemplo de fluxo:

```text
posição atual do robô
        ↓
Nav2 calcula a rota
        ↓
robô chega próximo da estação
        ↓
inicia a aproximação fina
```

---

## 2.2 Entrada na estação

Action:

```text
/Dock
```

Tipo:

```text
lidar_auto_docking_messages/action/Dock
```

Essa Action é responsável por:

* detectar o trapézio pelo LiDAR;
* refinar a pose da estação;
* alinhar o robô;
* aproximar lentamente;
* concluir o docking.

O gerenciador deve chamar `/Dock` somente depois que o Nav2 tiver chegado corretamente à pose de pré-dock.

---

## 2.3 Saída da estação

Action:

```text
/Undock
```

Tipo:

```text
lidar_auto_docking_messages/action/Undock
```

Essa Action é responsável por:

* recuar o robô;
* afastá-lo da estação;
* opcionalmente girá-lo no próprio eixo;
* liberar o robô para iniciar uma nova missão.

Dependendo da definição da Action, o goal pode conter:

```text
rotate_in_place: true
```

Os campos exatos devem ser confirmados com:

```bash
ros2 interface show \
  lidar_auto_docking_messages/action/Undock
```

---

# 3. Responsabilidade do nó gerenciador

O nó gerenciador do robô deve decidir quando chamar cada Action.

Ele pode usar informações como:

* nível da bateria;
* missão atual;
* presença de uma missão pendente;
* estado de carregamento;
* resultado da navegação;
* resultado do docking;
* resultado do undocking;
* comandos recebidos de um servidor externo.

Exemplo:

```text
bateria abaixo de 20%
        ↓
cancelar ou finalizar missão atual
        ↓
navegar até a pré-dock
        ↓
executar docking
        ↓
confirmar carregamento
        ↓
aguardar bateria suficiente
        ↓
executar undocking
        ↓
retomar missão
```

---

# 4. Máquina de estados recomendada

O `nav2_dock.py` utiliza estados numéricos simples. Para o nó definitivo, é melhor utilizar nomes claros.

Exemplo em Python:

```python
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    EXECUTING_MISSION = 1
    NAVIGATING_TO_DOCK = 2
    DOCKING = 3
    CHARGING = 4
    UNDOCKING = 5
    RECOVERY = 6
    ERROR = 7
```

Fluxo principal:

```text
IDLE
  ↓
EXECUTING_MISSION
  ↓ bateria baixa
NAVIGATING_TO_DOCK
  ↓ Nav2 chegou
DOCKING
  ↓ docking concluído
CHARGING
  ↓ bateria suficiente
UNDOCKING
  ↓ saída concluída
IDLE ou EXECUTING_MISSION
```

Em caso de falha:

```text
NAVIGATING_TO_DOCK
DOCKING
UNDOCKING
        ↓
     RECOVERY
        ↓
nova tentativa ou ERROR
```

---

# 5. Estrutura dos clientes de Action

O nó gerenciador deve possuir clientes para as três Actions.

Exemplo conceitual:

```python
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

from lidar_auto_docking_messages.action import Dock
from lidar_auto_docking_messages.action import Undock


class RobotManager(Node):

    def __init__(self):
        super().__init__('robot_manager')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.dock_client = ActionClient(
            self,
            Dock,
            '/Dock'
        )

        self.undock_client = ActionClient(
            self,
            Undock,
            '/Undock'
        )
```

Os nomes e tipos devem ser confirmados no sistema:

```bash
ros2 action list -t
```

---

# 6. Sequência de navegação até o dock

Quando o gerenciador decidir enviar o robô para carregar, ele deve primeiro chamar o Nav2.

Fluxo:

```text
start_docking_sequence()
        ↓
carregar pose da pré-dock
        ↓
enviar NavigateToPose
        ↓
aguardar resultado
```

Exemplo conceitual:

```python
def navigate_to_pre_dock(self, pre_dock_pose):
    if not self.nav_client.wait_for_server(timeout_sec=2.0):
        self.get_logger().error(
            'Action /navigate_to_pose indisponível'
        )
        self.state = RobotState.ERROR
        return

    goal = NavigateToPose.Goal()
    goal.pose = pre_dock_pose

    self.state = RobotState.NAVIGATING_TO_DOCK

    future = self.nav_client.send_goal_async(
        goal,
        feedback_callback=self.nav_feedback_callback
    )

    future.add_done_callback(
        self.nav_goal_response_callback
    )
```

Quando o goal for aceito:

```python
def nav_goal_response_callback(self, future):
    goal_handle = future.result()

    if not goal_handle.accepted:
        self.get_logger().error(
            'Goal de navegação rejeitado'
        )
        self.state = RobotState.ERROR
        return

    self.nav_goal_handle = goal_handle

    result_future = goal_handle.get_result_async()

    result_future.add_done_callback(
        self.nav_result_callback
    )
```

Quando o Nav2 terminar:

```python
def nav_result_callback(self, future):
    result = future.result()

    if result.status == 4:
        self.get_logger().info(
            'Pré-dock alcançada. Iniciando docking.'
        )

        self.start_dock_action()
    else:
        self.get_logger().error(
            'Falha ao alcançar a pré-dock'
        )

        self.state = RobotState.RECOVERY
```

O valor exato dos status deve ser tratado usando as constantes adequadas do ROS 2, em vez de depender diretamente de números fixos.

---

# 7. Sequência da Action de docking

Depois que o Nav2 chegar à pré-dock, o gerenciador envia um goal para `/Dock`.

Exemplo conceitual:

```python
def start_dock_action(self):
    if not self.dock_client.wait_for_server(
        timeout_sec=2.0
    ):
        self.get_logger().error(
            'Action /Dock indisponível'
        )

        self.state = RobotState.ERROR
        return

    goal = Dock.Goal()

    goal.dock_pose = self.saved_dock_pose

    self.state = RobotState.DOCKING

    future = self.dock_client.send_goal_async(
        goal,
        feedback_callback=self.dock_feedback_callback
    )

    future.add_done_callback(
        self.dock_goal_response_callback
    )
```

Os campos reais do goal devem ser confirmados com:

```bash
ros2 interface show \
  lidar_auto_docking_messages/action/Dock
```

Quando o docking terminar com sucesso:

```python
def dock_result_callback(self, future):
    result = future.result()

    if result.status == 4:
        self.get_logger().info(
            'Docking concluído'
        )

        self.state = RobotState.CHARGING
    else:
        self.get_logger().error(
            'Docking falhou'
        )

        self.state = RobotState.RECOVERY
```

---

# 8. Estado de carregamento

O término da Action `/Dock` não deve ser a única confirmação de que o carregamento começou.

O nó gerenciador também deve verificar algum sinal físico ou elétrico, por exemplo:

* corrente entrando na bateria;
* tensão do carregador;
* sinal digital de contato;
* estado informado pelo BMS;
* tópico de carregamento;
* aumento progressivo da porcentagem da bateria.

Exemplo:

```text
Action /Dock terminou
        ↓
aguardar confirmação elétrica
        ↓
carregamento confirmado
        ↓
estado CHARGING
```

Caso o carregamento não seja confirmado:

```text
dock concluído
        ↓
sem corrente de carga
        ↓
recuar alguns centímetros
        ↓
tentar docking novamente
```

---

# 9. Sequência de undocking

Quando a bateria atingir um nível configurado ou uma nova missão estiver disponível, o gerenciador deve chamar `/Undock`.

Exemplo conceitual:

```python
def start_undock_action(self):
    if not self.undock_client.wait_for_server(
        timeout_sec=2.0
    ):
        self.get_logger().error(
            'Action /Undock indisponível'
        )

        self.state = RobotState.ERROR
        return

    goal = Undock.Goal()
    goal.rotate_in_place = True

    self.state = RobotState.UNDOCKING

    future = self.undock_client.send_goal_async(
        goal,
        feedback_callback=self.undock_feedback_callback
    )

    future.add_done_callback(
        self.undock_goal_response_callback
    )
```

Quando o undocking for concluído:

```python
def undock_result_callback(self, future):
    result = future.result()

    if result.status == 4:
        self.get_logger().info(
            'Undocking concluído'
        )

        self.state = RobotState.IDLE
    else:
        self.get_logger().error(
            'Undocking falhou'
        )

        self.state = RobotState.RECOVERY
```

Se houver uma missão pendente:

```text
UNDOCKING
    ↓ sucesso
EXECUTING_MISSION
```

Caso contrário:

```text
UNDOCKING
    ↓ sucesso
IDLE
```

---

# 10. Uso das poses salvas

O `nav2_dock.py` mostra como utilizar duas poses diferentes.

O arquivo utilizado pelo `nav2_dock.launch.py` é:

```text
src/lidar_auto_docking/initial_dock_pose/dock_ws_dock.json
```

Esse é o mesmo arquivo criado por:

```bash
ros2 launch lidar_auto_docking dockpose_saver_launch.py
ros2 run lidar_auto_docking dock_pose_saver_cli
```

Não deve ser utilizado o nome antigo `init_dock.json`, pois esse arquivo não
existe neste workspace. Antes de executar o gerenciador, salve a pose da dock
com o robô corretamente localizado e posicionado na pré-dock.

## Pose de pré-dock

É a pose utilizada pelo Nav2 para levar o robô até próximo da estação.

Representa normalmente:

```text
pose do base_link antes da aproximação fina
```

## Pose do dock

É a pose absoluta da estação no mapa.

Representa:

```text
posição e orientação da dock_station
```

O arquivo salvo deve conter exatamente estas oito chaves:

```yaml
x: 2.438
y: -1.274
z: 0.713
w: 0.701

bx: 1.850
by: -1.280
bz: 0.705
bw: 0.709
```

Conceitualmente:

```text
x, y, z, w
→ pose do dock

bx, by, bz, bw
→ pose de pré-dock do robô
```

O `nav2_dock.py` utiliza `x/y/z/w` para enviar a Action `/Dock` e
`bx/by/bz/bw` para enviar o goal de pré-dock ao Nav2. Se alguma dessas chaves
estiver ausente, o exemplo não deve iniciar a sequência.

---

# 11. Evitar goals simultâneos

O gerenciador não deve enviar dois goals ao mesmo tempo.

Por exemplo, não deve aceitar um comando de undock enquanto ainda está executando docking.

Exemplo:

```python
def request_docking(self):
    if self.state not in {
        RobotState.IDLE,
        RobotState.EXECUTING_MISSION,
    }:
        self.get_logger().warn(
            f'Não é possível iniciar docking no estado '
            f'{self.state.name}'
        )
        return

    self.navigate_to_pre_dock(
        self.saved_pre_dock_pose
    )
```

Para undock:

```python
def request_undocking(self):
    if self.state != RobotState.CHARGING:
        self.get_logger().warn(
            'Undocking permitido somente quando dockado'
        )
        return

    self.start_undock_action()
```

---

# 12. Cancelamento

O nó gerenciador deve guardar o `goal_handle` de cada Action.

Exemplo:

```python
self.nav_goal_handle = None
self.dock_goal_handle = None
self.undock_goal_handle = None
```

Assim poderá cancelar uma operação:

```python
def cancel_docking(self):
    if self.dock_goal_handle is None:
        return

    cancel_future = (
        self.dock_goal_handle.cancel_goal_async()
    )

    cancel_future.add_done_callback(
        self.dock_cancel_callback
    )
```

Isso pode ser utilizado em situações como:

* botão de emergência;
* obstáculo inesperado;
* perda do LiDAR;
* perda de comunicação;
* comando remoto de cancelamento;
* falha de segurança.

---

# 13. Timeout

Cada etapa deve possuir timeout.

Exemplo:

```text
Nav2 até pré-dock: 120 segundos
detecção do dock: 10 segundos
docking completo: 60 segundos
confirmação de carga: 10 segundos
undocking: 30 segundos
```

Esses valores devem ser parâmetros ROS 2.

Exemplo YAML:

```yaml
robot_manager:
  ros__parameters:
    battery_dock_threshold: 20.0
    battery_resume_threshold: 90.0

    nav_to_dock_timeout_sec: 120.0
    docking_timeout_sec: 60.0
    charging_confirm_timeout_sec: 10.0
    undocking_timeout_sec: 30.0

    max_docking_attempts: 3
```

---

# 14. Tentativas de recuperação

O gerenciador pode executar tentativas automáticas.

Exemplo:

```text
Dock falhou
    ↓
recuar 20 cm
    ↓
reposicionar na pré-dock
    ↓
detectar novamente
    ↓
tentar /Dock outra vez
```

Controle:

```python
self.docking_attempt = 0
self.max_docking_attempts = 3
```

Exemplo:

```python
def handle_docking_failure(self):
    self.docking_attempt += 1

    if (
        self.docking_attempt
        < self.max_docking_attempts
    ):
        self.get_logger().warn(
            'Tentando docking novamente'
        )

        self.state = RobotState.RECOVERY
        self.start_docking_recovery()
    else:
        self.get_logger().error(
            'Número máximo de tentativas atingido'
        )

        self.state = RobotState.ERROR
```

---

# 15. Estrutura recomendada do nó

Uma estrutura organizada pode ser:

```text
robot_manager/
├── robot_manager/
│   ├── robot_manager_node.py
│   ├── docking_manager.py
│   ├── mission_manager.py
│   ├── battery_manager.py
│   ├── state_machine.py
│   └── pose_loader.py
├── config/
│   └── robot_manager.yaml
├── launch/
│   └── robot_manager.launch.py
├── package.xml
└── setup.py
```

Responsabilidades:

```text
robot_manager_node.py
→ integração geral

docking_manager.py
→ clientes /Dock e /Undock

mission_manager.py
→ missões Nav2

battery_manager.py
→ nível e estado de carga

state_machine.py
→ estados do robô

pose_loader.py
→ leitura das poses salvas
```

Para uma primeira versão, essas funções podem estar em um único arquivo. Depois podem ser separadas conforme o projeto crescer.

---

# 16. Fluxo completo recomendado

```text
Gerenciador detecta bateria baixa
              ↓
cancela ou finaliza missão atual
              ↓
carrega pose de pré-dock
              ↓
chama /navigate_to_pose
              ↓
Nav2 chega à pré-dock
              ↓
chama /Dock
              ↓
LiDAR detecta o trapézio
              ↓
robô alinha e aproxima
              ↓
Action /Dock retorna sucesso
              ↓
confirma corrente de carga
              ↓
estado CHARGING
              ↓
bateria atinge nível configurado
              ↓
chama /Undock
              ↓
robô recua e gira
              ↓
Action /Undock retorna sucesso
              ↓
retoma missão ou entra em IDLE
```

---

# 17. Papel do `nav2_dock.py`

O arquivo `nav2_dock.py` deve ser usado como referência para entender:

* como criar os clientes de Action;
* como montar os goals;
* como carregar as poses salvas;
* como sequenciar Nav2 e docking;
* como iniciar undocking;
* como receber resultados;
* como manter um estado simples da operação.

Ele pode ser utilizado para testes rápidos, mas o nó gerenciador definitivo deve acrescentar:

* estados com nomes claros;
* timeouts;
* cancelamento;
* tentativas;
* confirmação elétrica de carga;
* integração com a bateria;
* integração com missões;
* tratamento de falhas;
* proteção contra goals simultâneos;
* publicação do estado atual do robô.

Resumo:

```text
nav2_dock.py
→ exemplo funcional e referência

robot_manager
→ implementação definitiva do robô
```

---

# 18. Comandos úteis para análise

Listar Actions:

```bash
ros2 action list -t
```

Ver a Action de docking:

```bash
ros2 interface show \
  lidar_auto_docking_messages/action/Dock
```

Ver a Action de undocking:

```bash
ros2 interface show \
  lidar_auto_docking_messages/action/Undock
```

Ver o Nav2:

```bash
ros2 interface show \
  nav2_msgs/action/NavigateToPose
```

Ver informações do servidor:

```bash
ros2 action info /Dock
```

```bash
ros2 action info /Undock
```

Acompanhar estado do gerenciador:

```bash
ros2 topic echo /robot_state
```

---

# Conclusão

O `nav2_dock.py` já contém a sequência básica necessária para navegar até uma estação, executar docking e posteriormente executar undocking.

Ele deve ser utilizado como guia para implementar o nó gerenciador principal do robô.

A responsabilidade final deve ser dividida assim:

```text
Nav2
→ navegação global até a pré-dock

lidar_auto_docking
→ alinhamento e aproximação fina

robot_manager
→ decisão, sequência, bateria, missão e falhas
```

Essa separação evita duplicar funções e mantém cada parte do sistema com uma responsabilidade clara.
