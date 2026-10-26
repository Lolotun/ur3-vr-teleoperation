# UR3 VR Teleoperation

Управление виртуальным UR3 через MoveIt Servo и реальным UR3 через URScript
`speedl` из Unity/Meta Quest через ROS 2. Работа на физической установке после
переноса пока не подтверждена.

```text
Unity / Quest → ROS-TCP Endpoint :10000 → ограничитель Twist
             → URScript speedl → UR3 :30002
             → команды захвата → Robotiq :63352
UR Driver → /joint_states → модель UR3 в Unity
```

## Состав

- `ros/` — Docker, узлы `wifi-speedl` и виртуальный режим MoveIt Servo.
- `unity/` — Unity-проект с `Assets/Scenes/WorkshopScene.unity`, ресурсами и `.meta`.
- `scripts/run.sh` — единый запуск backend из корня репозитория.
- `docs/` — [установка и запуск](docs/setup.md), [архитектура](docs/architecture.md),
  [происхождение и границы переноса](docs/provenance.md), [проверки](docs/validation.md).

Старые учебные launch-файлы day1/day2/day3, real Servo / External Control,
датасеты и Docker-архивы в этот перенос не включены. Unity-сцена и её компоненты
сохранены: это позволяет не ломать сериализованные ссылки при очистке backend.

## Быстрый старт

Linux, Docker Engine + Compose v2; Unity Editor **6000.2.15f1**.
Для сборки на Quest нужен Android Build Support. Python 3 нужен для вспомогательных скриптов.

Для виртуального робота (ROBOT_IP не нужен):

```bash
./scripts/run.sh build
./scripts/run.sh servo
./scripts/run.sh logs
```

В Unity открыть WorkshopScene, выбрать ROS 2, IP `127.0.0.1` (Editor на этом
компьютере), порт `10000` и нажать Play. На Quest использовать Wi-Fi IP компьютера.
Виртуальный режим рассчитывает суставы через MoveIt Servo и публикует `/joint_states`.
`sim` — синоним `servo`. `servo` и `up` переключают один Compose-сервис;
`logs`, `topics` и `down` работают для обоих режимов. После изменений ROS-кода
снова выполнить `build` перед запуском выбранного режима.

Для реального робота:

```bash
cp .env.example .env
# Указать ROBOT_IP в .env.
./scripts/run.sh build
./scripts/run.sh up
./scripts/run.sh logs
```

`up` запускает управление реальным роботом. По умолчанию сохранены параметры
последнего демо: 0.020 м/с, 8 Гц, вращение TCP отключено, захват включён.
External Control Play для этого режима не требуется.

Если уже загружен старый образ `ur3-vr-workshop:humble`, вместо сборки можно
установить в `.env` `TELEOP_IMAGE=ur3-vr-workshop:humble`. Это запускает код,
запечённый в старом образе; изменения `ros/` попадут только в новую сборку.

В Unity Hub открыть `unity/`, затем `WorkshopScene`. Выбрать ROS 2, порт `10000`:
для Editor на том же компьютере IP `127.0.0.1`, для Quest — доступный по Wi-Fi IP компьютера.
При ошибке Meta XR на Linux см. [инструкцию восстановления прежнего исправления](docs/setup.md#unity).

Остановка:

```bash
./scripts/run.sh down
```

Это **скоростное** управление: удержание Right Trigger и отклонение контроллера
задают скорость TCP; Right Grip управляет захватом. Относительное позиционное
управление, DAgger и обучение пока не реализованы.

## Происхождение

Unity основан на ETH Zürich / leggedrobotics `unity_ros_teleoperation`;
его BSD-3-Clause лицензия сохранена в [unity/LICENSE](unity/LICENSE).
Модели и учебные ресурсы имеют отдельное происхождение, описанное в
[docs/provenance.md](docs/provenance.md). Общая новая лицензия на весь набор не назначалась.
