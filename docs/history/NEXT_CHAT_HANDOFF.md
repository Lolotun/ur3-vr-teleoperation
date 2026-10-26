# UR3 VR Workshop handoff для следующего чата

Дата: 2026-09-02.

## Репозитории

```text
/home/demon/dev/QS2026/ur3-vr-workshop
```

ROS / Docker backend: ROS-TCP endpoint, safety monitor, MoveIt Servo, direct URScript `speedl`, Robotiq bridge.

```text
/home/demon/dev/QS2026/unity_ros_teleoperation
```

Unity / Quest frontend: VR scene, controller input, visual UR3, gripper command publishing, simulation pickup.

Связь Unity <-> ROS идет через ROS-TCP endpoint:

```text
ROS_TCP_PORT=10000
ROS_DOMAIN_ID=11
topics namespace: /team_1
```

## Текущее главное достижение

Удалось получить рабочий pipeline Unity -> ROS -> реальный UR3 без External Control:

```text
Unity ControllerTwistPublisher
  -> /team_1/twist_command
  -> twist_command_monitor
  -> /team_1/twist_command_safe
  -> ur_speedl_wifi_bridge
  -> UR controller TCP port 30002, speedl(...)
```

И отдельно:

```text
Unity ControllerTwistPublisher
  -> /team_1/gripper_command: "open" / "close"
  -> robotiq_gripper_bridge
  -> Robotiq socket port 63352
```

External Control / URCap для текущего demo path не используется. На teach pendant не надо нажимать External Control Play для `wifi-speedl`.

## Последние известные commit-ы

ROS repo:

```text
366a9ee stable version
26028b9 add robotiq gripper support to wifi speedl
26eabe7 add wifi speedl fallback
```

Unity repo:

```text
e8575e3 Save WorkshopScene layout and teleop tuning from editor.
4128018 Add sim gripper pickup, rigid fingers, and practice cube tower.
955b07f Stable teleop for sim and real: angular mode toggle and gripper fix.
0e2ad6a wire gripper commands in Unity teleop
```

## Важный текущий git status

На момент создания этого файла:

ROS repo имеет незакоммиченные изменения:

```text
M ros2_ws/src/teleop_core/config/ur3_servo.yaml
M ros2_ws/src/teleop_core/launch/servo_sim.launch.py
```

Эти изменения включают rotational Servo в sim:

```text
ur3_servo.yaml:
  scale.rotational: 0.0 -> 0.2

servo_sim.launch.py:
  twist_command_monitor max_angular_speed: 0.0 -> 0.2
```

Также после `26028b9` были сделаны правки для real `wifi-speedl`:

```text
wifi-speedl real angular cap по умолчанию: 0.120 -> 0.0
GRIPPER_FORCE по умолчанию: 20
```

Unity repo имеет только untracked recovery-файлы от Editor:

```text
Assets/_Recovery/0 (1).unity
Assets/_Recovery/0 (1).unity.meta
Assets/_Recovery/0 (2).unity
Assets/_Recovery/0 (2).unity.meta
```

Их не надо коммитить. Можно игнорировать.

## Как запускать реального робота сейчас

Цель: только движение по XYZ, без поворотов TCP в реальности, gripper включен.

```bash
cd /home/demon/dev/QS2026/ur3-vr-workshop

docker rm -f ur3-vr-workshop-wifi-speedl

ROBOT_IP=192.168.50.83 \
SPEEDL_SEND_HZ=8 \
SPEEDL_MAX_LINEAR_SPEED=0.020 \
ENABLE_GRIPPER=true \
./scripts/run.sh wifi-speedl
```

Если нужен другой робот, заменить:

```bash
ROBOT_IP=<IP_UR_CONTROLLER>
```

В логах должно быть:

```text
angular_cap=0.0 rad/s
Robotiq gripper bridge: true on port 63352
```

Смотреть логи:

```bash
docker logs -f ur3-vr-workshop-wifi-speedl
```

Проверка gripper из ROS без Unity:

```bash
docker compose run --rm workshop bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic pub --once /team_1/gripper_command std_msgs/msg/String '{data: close}'"
```

```bash
docker compose run --rm workshop bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic pub --once /team_1/gripper_command std_msgs/msg/String '{data: open}'"
```

Если в логах есть:

```text
Robotiq position sent: 255
Robotiq position sent: 0
```

значит ROS -> Robotiq socket работает. Если этих строк нет при нажатии Grip в Unity, значит Unity не публикует `/team_1/gripper_command`.

## Управление в VR

```text
Right Trigger hold + move controller = движение робота по XYZ
Right Grip press/hold = gripper close
Right Grip release = gripper open
```

В текущем real mode повороты отключены на ROS side:

```text
SPEEDL_MAX_ANGULAR_SPEED=0.0
```

Unity может публиковать angular, но `twist_command_monitor` / `ur_speedl_wifi_bridge` обрежут его до нуля перед реальным роботом.

Если когда-то нужно вернуть вращение TCP в реальности, запускать явно:

```bash
SPEEDL_MAX_ANGULAR_SPEED=0.120
```

Но для демо лучше оставить `0.0`: так управление стабильнее и понятнее.

## Как запускать симуляцию

```bash
cd /home/demon/dev/QS2026/ur3-vr-workshop

docker rm -f ur3-vr-workshop-servo
./scripts/run.sh servo
```

Unity:

```text
Open unity_ros_teleoperation
Open Assets/Scenes/WorkshopScene.unity
Robotics -> ROS Settings:
  ROS 2
  IP: 127.0.0.1 для Editor на том же ноутбуке
  Port: 10000
ROSConnection:
  Listen For TF Messages: off
Play
```

Для Quest standalone использовать IP ноутбука:

```bash
hostname -I | awk '{print $1}'
```

В palm menu / app settings поставить этот IP и port `10000`.

## Что есть в Unity сейчас

Основная сцена:

```text
Assets/Scenes/WorkshopScene.unity
```

Главные scripts:

```text
Assets/Scripts/ControllerTwistPublisher.cs
```

Публикует:

```text
/team_1/twist_command
/team_1/gripper_command
```

Важные поля:

```text
publishAngularVelocity
angularRosSigns = (1, 1, -1)
publishGripperCommands = true
gripperTopic = /team_1/gripper_command
gripCloseThreshold = 0.65
animateGripperVisual = 0 / false
```

Gripper visual:

```text
robotiq_arg2f_base_link
```

Сейчас gripper визуально жесткий. Пальцы специально не анимируются, потому что попытка двигать URDF-звенья выглядела плохо и могла путать визуальную позу.

Для simulation pickup есть отдельные scripts:

```text
Assets/Scripts/SimGripperPickup.cs
Assets/Scripts/SimPickableObject.cs
Assets/Scripts/Ur3GripperRigidizer.cs
Assets/Scripts/WorkshopCubeTowerSpawner.cs
```

Идея sim pickup: если `Grip` зажат рядом с pickable object, предмет parent-ится к gripper, при отпускании Grip отцепляется.

## Что важно не ломать в Unity

Можно двигать/поворачивать корневой объект:

```text
ur3
```

Нельзя менять scale URDF-робота или внутренних link-ов:

```text
base_link
tool0
wrist_*
robotiq_* если не понимаешь offset
```

Если оси управления снова стали странными, первым делом проверить:

```text
ControllerTwistPublisher.robotBase -> активный ur3/world/base_link
```

Если `robotBase` слетел в `None`, движение будет считаться в неправильной системе координат.

## Почему не External Control

Изначально пробовали UR ROS driver + External Control / reverse interface.

Симптом:

```text
Robot requested program
Sent program to robot
Robot connected to reverse interface. Ready to receive control commands.
Connection to reverse interface dropped.
```

После этого:

```text
robot_program_running: false
speed_scaling иногда 100 -> 0
Background reading is not running, cannot get data package
```

Pipeline Unity/ROS/MoveIt Servo при этом считал команды, но реальный robot controller session разваливался. Поэтому для срочного демо перешли на `wifi-speedl`, который отправляет URScript `speedl(...)` напрямую на port `30002` и не требует External Control program.

## Текущая архитектура real mode

```text
Unity
  ControllerTwistPublisher
    Trigger -> TwistStamped /team_1/twist_command
    Grip    -> String /team_1/gripper_command

Docker wifi-speedl
  ros_tcp_endpoint.default_server_endpoint :10000
  twist_command_monitor:
    clamps linear
    clamps angular to 0.0 by default in real
  ur_speedl_wifi_bridge:
    sends speedl([vx, vy, vz, 0, 0, 0], a, t) to UR :30002
  robotiq_gripper_bridge:
    sends SET ACT/GTO/SPE/FOR/POS to Robotiq :63352
  ur_robot_driver state feedback:
    publishes /joint_states for Unity visual feedback
```

Expected pendant state in `wifi-speedl`:

```text
Program: Stopped
```

Это нормально, потому что External Control не используется.

## Диагностика

Список topics:

```bash
docker exec -it ur3-vr-workshop-wifi-speedl bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic list"
```

Twist из Unity:

```bash
docker exec -it ur3-vr-workshop-wifi-speedl bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /team_1/twist_command"
```

Twist после safety:

```bash
docker exec -it ur3-vr-workshop-wifi-speedl bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /team_1/twist_command_safe"
```

Gripper command из Unity:

```bash
docker exec -it ur3-vr-workshop-wifi-speedl bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /team_1/gripper_command"
```

Joint states для Unity visual feedback:

```bash
docker exec -it ur3-vr-workshop-wifi-speedl bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /joint_states"
```

## Частые проблемы

### Робот едет, но Unity-поза не совпадает

Проверить:

```text
запущен ли state feedback в wifi-speedl
есть ли /joint_states
ControllerTwistPublisher.robotBase указывает на активный base_link
Unity не слушает TF
не изменен scale у ur3/base_link/tool0
```

### Gripper не двигается

Проверить по порядку:

1. Unity Console пишет:

```text
[ControllerTwistPublisher] gripper close
[ControllerTwistPublisher] gripper open
```

2. ROS видит topic:

```bash
ros2 topic echo /team_1/gripper_command
```

3. Bridge пишет:

```text
Robotiq position sent: 255
Robotiq position sent: 0
```

4. Если bridge пишет `Robotiq command failed`, проверить:

```text
robot IP
Robotiq socket port 63352
наличие Robotiq URCap/socket server
не занят ли gripper другой программой на pendant
```

### Управление стало дерганым

Для demo real mode использовать:

```text
SPEEDL_SEND_HZ=8
SPEEDL_MAX_LINEAR_SPEED=0.020
SPEEDL_MAX_ANGULAR_SPEED=0.0
SPEEDL_COMMAND_DURATION=0.14
```

Не открывать тяжелые browser/downloads на том же Wi-Fi, потому что ранее ping проседал и робот дергался.

## Что сделать дальше

1. Не трогать рабочий `wifi-speedl` path перед демо.
2. Для Unity object manipulation лучше использовать `SimGripperPickup`, а не анимировать URDF-пальцы.
3. Если нужен красивый gripper visual, сделать отдельную простую визуальную модель из двух губок поверх URDF, а не двигать реальные URDF link-и Robotiq.
4. После стабилизации закоммитить текущий real angular-off diff в ROS, если он подтвержден на роботе.

