# Позиционная телеоперация через ur_rtde

Режим ветки `ur_rtde`. Старые `servo` (виртуальный) и `up` / `wifi-speedl` сохранены.
Здесь Servo из MoveIt не используется: `ur_rtde.servoL` исполняет целевые позы
через программу на контроллере UR, с внутренней кинематикой робота.

```text
Unity ControllerPosePublisher: pose + Trigger heartbeat
    → relative_pose_controller: anchors + target_pose
    → ur_rtde_bridge: ограниченные шаги pose → servoL
UR RTDE Receive → /teleop/tcp_actual + /joint_states + /teleop/rtde_status
```

## Первый запуск: без отправки движения

1. Собрать новый образ. Старый workshop-образ не содержит новых узлов/ur_rtde.
   В `.env` выбрать `TELEOP_IMAGE=ur3-vr-teleoperation:humble`.
2. Сохранить существующий ROBOT_IP в `.env`. Новые параметры можно добавить
   из раздела RTDE в `.env.example`. Значения по умолчанию уже есть в Compose.
3. Для первого запуска оставить:

   ```dotenv
   RTDE_ENABLE_MOTION=false
   RTDE_ENABLE_GRIPPER=false
   RTDE_FREQUENCY=125.0
   ```

4. Из корня репозитория:

   ```bash
   ./scripts/run.sh build
   ./scripts/run.sh rtde
   ./scripts/run.sh logs
   ```

5. В Unity открыть WorkshopScene. Найти объект с ControllerTwistPublisher
   (по GUID сцены это объект с Transform 320328463). Рядом уже добавлен
   **ControllerPosePublisher** с теми же rightController / robotBase.
   Включить **Pose Mode** в Inspector до Play. По умолчанию он выключен,
   чтобы не менять работу прежних режимов. Для APK сохранить выбор и пересобрать приложение.
6. Подключить Unity к Endpoint (ROS 2, IP компьютера, порт 10000).
   Отпустить Trigger после старта, затем нажать и двигать контроллер.

Preview создаёт только RTDEReceiveInterface: не загружает управляющий скрипт,
не создаёт RTDEControlInterface и не вызывает servoL. Обратная связь читается
с реального робота. Мост захвата выключен независимо от ENABLE_GRIPPER старого режима.
Тесты с поддельным роботом описаны ниже; это не отдельный пользовательский симулятор.

Смотреть данные:

```bash
./scripts/run.sh shell
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /teleop/tcp_actual
# Ctrl-C, затем другие точки наблюдения:
ros2 topic echo /vr/right_hand/pose
ros2 topic echo /teleop/target_pose
ros2 topic echo /teleop/rtde_status
ros2 topic echo /teleop/pose_status
```

## Координаты, опорная поза и вращение

Unity публикует позу руки в `vr_robot_base`: координаты относительно **фиксированного
основания визуального UR3**, преобразованные Unity ROSGeometry в FLU. ROS берёт
фактический TCP из RTDE, а не из модели Unity, и фиксирует две опорные позы при Trigger.
Относительное движение руки преобразуется в целевую позу TCP. Опорная поза робота
не обновляется за движущимся роботом. Отпускание и новое нажатие дают новую привязку.

Выходная цель и RTDE feedback используют frame `base` — основание реального UR.
`RTDE_MAPPING_YAW_DEG=180` задаёт обычный поворот между URDF base_link и UR base;
на конкретной сцене и установке направления обязательно проверить в preview.
Для другой калибровки доступен yaw в градусах. Произвольная полная extrinsic-калибровка
и компенсация изменения основания VR пока не реализованы. Старые покомпонентные
angularRosSigns и SPEEDL_LINEAR_* здесь **не применяются**.

Ориентация считается кватернионами: относительное вращение руки переводится
в оси робота и умножается на ориентацию TCP при привязке. Вектор вращения UR
получается только на границе RTDE. TCP — именно активный TCP контроллера;
нужно понимать, какой инструмент/TCP настроен на pendant, он может отличаться от tool0 Unity.

Часы Quest и компьютера должны быть согласованы: сообщения руки содержат UTC,
старые (более 1 с) и идущие назад метки отклоняются. Неизменный heartbeat без
новых поз не считается свежим вводом. Тайм-аут приёма позы и Trigger — 0.25 с.

## Включение движения после проверки цели

После проверки направлений и ориентации установить в `.env`:

```dotenv
RTDE_ENABLE_MOTION=true
# Только если нужен реальный захват:
RTDE_ENABLE_GRIPPER=true
```

Применить `./scripts/run.sh rtde`. Изменение .env не требует build.
Это создаст ControlInterface и загрузит его управляющий скрипт на робота.
Не оставлять другие источники команд движения (старый backend, программа политики,
другой RTDEControl/External Control) активными на этом роботе.
Требования remote control зависят от поколения контроллера и PolyScope.
Для CB3 UR3 начальная частота 125 Гц; e-Series может использовать 500 Гц,
но менять её следует после проверки модели и времени цикла.

### Локальная проверка позы

`RTDE_POSE_CHECK_MODE=local` заменяет запрос `isPoseWithinSafetyLimits` в каждом
servo-цикле локальной проверкой конечности поз, смещения/вращения цели от TCP при
начале удержания и размера servo-шага. Пределы берутся из
`RTDE_MAX_DISPLACEMENT`, `RTDE_MAX_ROTATION`, `RTDE_MAX_LINEAR_SPEED` и
`RTDE_MAX_ANGULAR_SPEED`. При отпускании опора сбрасывается. Проверки свежести,
состояния робота, отставания и пропуска периода остаются в мосте.

Это проверка декартовых ограничений, а не эквивалент `isPoseWithinSafetyLimits`:
нет локального IK, проверки суставных ограничений, столкновений и safety planes
из установки UR. Встроенные защиты контроллера UR не перенастраиваются.
Режим по умолчанию — `robot`, он сохраняет удалённую проверку перед каждым шагом.
В измерениях на установке чередование проверки и servoL занимало около 40 мс;
непрерывный servoL с фиксированной целью укладывался в период 8 мс. Это не
гарантирует такую же частоту при меняющейся цели.

Никакого автоматического moveL к стартовой точке нет. Первый servo-setpoint
берётся из actual TCP; последующие шаги ограничены 0.020 м/с и 0.12 рад/с.
Параметры speed/acceleration у servoL в ur_rtde 1.6.2 не используются библиотекой:
ограничения реализованы шагами позы, а не этими аргументами.

Ограничения и остановки:

- Максимальное относительное смещение за один clutch: 0.25 м, вращение 1 рад.
- Перестали обновляться команда/heartbeat или RTDE timestamp — остановка.
- Не normal safety mode / не running robot mode — остановка движения.
- Отставание actual от исполняемой команды >0.05 м или >0.35 рад — остановка.
- Пропуск периода управления >4 периодов — остановка.
- Перед servoL выполняется выбранная проверка (`robot` или `local`);
  отклонение/ошибка servoL — остановка.
- После тайм-аута/ошибки продолжение удержания Trigger не возобновляет движение:
  требуется отпускание и новая привязка. Ошибка соединения завершает worker;
  после восстановления требуется перезапустить режим.

Это программные ограничения прототипа, не сертифицированная система безопасности.
Проверки столкновений со сценой, планирование обхода препятствий и ограничение jerk
не реализованы. Состояния выводятся в rtde_status; ошибки не скрываются автопереподключением.

## Реализация

- `unity/Assets/Scripts/ControllerPosePublisher.cs` — ввод, tracking, heartbeat и выбор режима.
- `ControllerTwistPublisher.cs` — сохранён; в Pose Mode перестаёт публиковать Twist,
  но продолжает обслуживать прежний ввод захвата.
- `pose_control.py` — математика, clutch, ограничение шага и ServoSession без ROS.
- `relative_pose_controller.py` — подписки, freshness и публикация цели.
- `ur_rtde_bridge.py` — один поток владеет Receive/Control; ROS callbacks только
  обновляют последние данные под блокировкой. Очередь целей не накапливается.
- `rtde_real.launch.py` — только Endpoint, относительный контроллер, RTDE bridge,
  опциональный Robotiq. Официальный UR Driver и speedl-мост не запускаются.
- `ros/requirements.txt` — точная версия ur_rtde 1.6.2, устанавливается Dockerfile.

Переход назад: отпустить Trigger, выключить Pose Mode в Unity, затем
`./scripts/run.sh servo` или `./scripts/run.sh up`.

## Проверки без оборудования

```bash
PYTHONPATH=ros/ros2_ws/src/teleop_core python3 -m unittest discover -s tests -v
# В окружении с ROS, без подключения к реальному роботу:
python3 tests/rtde_ros_smoke.py -v
```

Второй тест подменяет rtde_receive/rtde_control до импорта bridge. Он проверяет
preview, командный путь, остановку при пропаже поз и RTDE feedback и запрет
самопроизвольного возобновления. Его нельзя использовать как подтверждение
сетевых задержек или динамики реального робота.

Источники API: [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/),
[пример servoL](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/high_frequency_servoing/servol_example.html).
Логика относительной привязки сопоставлена с локальным Quest2ROS2
`q2r2_bringup/robot_arm_controller_base.py`; код написан отдельно для текущих topics
и RTDE. Политика/DAgger и переключение автономного управления ещё не реализованы.
