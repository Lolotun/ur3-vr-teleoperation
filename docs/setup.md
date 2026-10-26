# Установка и запуск

## Окружение

Рабочая основа — ROS 2 Humble / Ubuntu Jammy внутри Docker и Unity 6000.2.15f1
снаружи Docker. Этот Compose рассчитан на Linux с host networking.
Нативную установку ROS на компьютере и uv для этого backend делать не нужно.

После клонирования репозитория перейти в его корень. Нужны Docker Engine,
Compose v2, Git и Unity Hub. Для Git-зависимостей Unity может понадобиться Git LFS.
Git-зависимости перенесённого Unity-проекта переведены на HTTPS и закреплены
по хешам из прежнего packages-lock.json. Доступность всех внешних репозиториев
на чистом компьютере пока не проверена.

## Docker

### Только виртуальный UR3

```bash
./scripts/run.sh build
./scripts/run.sh servo
./scripts/run.sh logs
```

ROBOT_IP не нужен. `sim` — синоним `servo`. UR Driver, speedl и мост Robotiq
в этом режиме не запускаются. MoveIt Servo выдаёт траектории, виртуальный
адаптер публикует положения суставов, Unity отображает их.
Вращение включено как в последнем локальном servo_sim: angular cap 0.3 рад/с.
Это кинематическая модель, а не физическая симуляция контактов.

В Unity: ROS 2, IP `127.0.0.1`, порт 10000, WorkshopScene → Play.
На Quest — Wi-Fi IP компьютера. Остановка: `./scripts/run.sh down`.
`up` переключает тот же сервис на реальный режим, `servo` — на виртуальный.
Перед переключением отпустить Trigger, затем переподключить Unity при необходимости.
Не запускать параллельно старые workshop-контейнеры с теми же портом/topics.

### Реальный UR3

Создать локальную конфигурацию:

```bash
cp .env.example .env
```

Указать `ROBOT_IP` — адрес контроллера UR3. IP робота и IP компьютера для
подключения Quest — разные настройки.

Вариант A: новая сборка из перенесённых исходников:

```bash
./scripts/run.sh build
./scripts/run.sh up
```

После изменений ROS-кода повторить build и up. Исходники не подключаются
томом: контейнер исполняет код своего образа.

Вариант B: прежний workshop-образ. В `.env` установить:

```dotenv
TELEOP_IMAGE=ur3-vr-workshop:humble
```

Если образ уже есть в `docker images`, достаточно `./scripts/run.sh up`.
Если есть только архив, загрузить его из места хранения:

```bash
docker load -i /path/to/ur3-vr-workshop-humble.docker.tar.gz
./scripts/run.sh up
```

Не выполнять `build` с выбранным старым тегом, если требуется сохранить старый образ:
сборка переназначает выбранный тег. Архивы образов хранятся отдельно от Git.
Наличие архивов не доказывает их идентичность локальному образу; сравнение digest
архивов не выполнялось.

Перед запуском остановить прежний backend, если он работает: оба используют
порт 10000 и одинаковые topics. `down` нового проекта останавливает только новый
Compose-проект, а не старые контейнеры workshop.

```bash
./scripts/run.sh ps
./scripts/run.sh logs
./scripts/run.sh topics
./scripts/run.sh down
```

## Unity

1. Unity Hub → Add project from disk → папка `unity/`.
2. Editor **6000.2.15f1**, дождаться загрузки пакетов.
3. Открыть `Assets/Scenes/WorkshopScene.unity`.
4. Robotics → ROS Settings → **ROS 2**.
5. Подключение: IP компьютера с backend, порт **10000**. В Editor на том же
   компьютере использовать `127.0.0.1`; в Quest — Wi-Fi IP компьютера.
6. Сохранить прежнее `Listen For TF Messages: off`: в старых проверках отмечена
   ошибка десериализации TF. Визуальная поза руки приходит через `/joint_states`.

В рабочем локальном кеше и раздаче был исправлен Linux compilation bug Meta XR
Core SDK 85.0.0: `downloadedInstallerPath` не определялся вне Windows/macOS.
Кеш `Library/` не переносится. Если на Linux после первой загрузки пакетов
возникнет эта ошибка, из корня выполнить:

```bash
python3 scripts/patch-meta-xr.py
```

Вернуться в Unity и дать Editor перекомпилировать код. Скрипт восстанавливает
тот же `#else` для 85.0.0 и допускает повторный запуск. При пересоздании кеша
исправление потребуется повторить. Полный SDK размером 222 МБ не включён в Git.

Для Quest установить Android Build Support (SDK/NDK/OpenJDK) через Unity Hub,
выбрать Android и собрать WorkshopScene. Готовый APK — отдельный артефакт,
в этот репозиторий не скопирован.

## Управление и диагностика

- Right Trigger удерживать: отклонение руки задаёт скорость TCP.
- Right Grip нажать/удерживать: close; отпустить: open.
- Реальное вращение TCP по умолчанию выключено: `SPEEDL_MAX_ANGULAR_SPEED=0.0`.
- `ControllerTwistPublisher.robotBase` должен указывать на base_link активного UR3.
- Не менять масштаб внутренних URDF-звеньев для визуальной подгонки.

Проверка из контейнера:

```bash
./scripts/run.sh shell
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /team_1/twist_command
# Для других проверок завершить echo через Ctrl-C и выбрать нужный topic:
ros2 topic echo /team_1/twist_command_safe
ros2 topic echo /joint_states
ros2 topic echo /team_1/gripper_command
```

Для последнего демо настройки: send_hz 8, linear cap 0.020 м/с,
angular cap 0, duration 0.14 с, force 20. Сохранены и прежние поправки осей:
linear (-1,-1,+1), angular (+1,+1,+1). Это параметры старой установки,
а не результат новой калибровки.

## Позиционный RTDE-режим

Отдельная команда `./scripts/run.sh rtde`, по умолчанию без движения.
Настройка Unity, привязки и проверка: [rtde.md](rtde.md).
