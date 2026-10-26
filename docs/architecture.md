# Текущий рабочий путь

```text
Quest / Unity ControllerTwistPublisher
  /team_1/twist_command (geometry_msgs/TwistStamped)
    → twist_command_monitor
  /team_1/twist_command_safe
    → ur_speedl_wifi_bridge
    → TCP socket на UR3:30002, URScript speedl([vx,vy,vz,wx,wy,wz], a, t)

  /team_1/gripper_command (std_msgs/String: open / close)
    → robotiq_gripper_bridge
    → Robotiq socket на IP робота:63352

ur_robot_driver (state feedback включён, joint controller не активируется)
  → /joint_states (sensor_msgs/JointState)
  → Unity Ur3ArticulationJointStateSubscriber
```

ROS-TCP Endpoint слушает порт 10000 компьютера. Unity отправляет и принимает
ROS-сообщения через этот TCP-мост; ROS_DOMAIN_ID задаёт область ROS 2 discovery,
но не выбирает робота. Адрес робота задаёт ROBOT_IP.

Вычисление движения суставов для speedl выполняется контроллером UR3.
MoveIt Servo, ikpy и External Control в этом пути не используются.
Код трёх узлов и wifi_speedl.launch.py перенесён без изменения поведения.
Прежние оговорки остаются: ограничитель Twist не является полноценной системой
безопасности/проверки столкновений, частота отправки мостом 8 Гц не означает
высокочастотный контур слежения. На роботе остаются его штатные ограничения.

На стороне Unity сохранена WorkshopScene: в ней есть также компоненты
симуляционного захвата и отладки. Они сохранены вместе с исходниками сцены,
чтобы избежать Missing Script. Наличие этих компонентов не означает,
что backend рассчитывает контакты и физику объектов.

В коде есть угловая составляющая Twist, но реальный angular cap равен нулю.
Позиционная цель, перепривязка при вмешательстве, переключение политики и
запись демонстраций относятся к дальнейшей разработке.

## Виртуальный режим (`./scripts/run.sh servo`)

```text
Unity Twist → ROS-TCP Endpoint → twist_command_monitor → MoveIt Servo
  → /team_1/servo_joint_trajectory → servo_joint_state_adapter
  → /joint_states → Unity
robot_state_publisher → TF → tcp_actual_publisher → /team_1/tcp_actual
```

Запускается servo_sim.launch.py. UR Driver и оба аппаратных моста не запускаются;
конфигурация не требует ROBOT_IP. В sim включено вращение, линейный cap 0.12 м/с,
угловой cap 0.3 рад/с. Параметры взяты из текущего локального workshop,
включая незакоммиченную правку servo_sim.
Симуляционный адаптер использует порядок суставов и seed из ur3_model.py;
исторический ikpy solver для этого не требуется.
Compose использует один сервис: переключение меняет command и пересоздаёт сервис.
Проверка столкновений Servo в перенесённом конфиге выключена.
