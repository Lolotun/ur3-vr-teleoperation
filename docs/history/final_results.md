# UR3 Unity ↔ ROS: результаты проверки

Дата: 2026-09-01

## Что проверено и работает

### ROS / MoveIt Servo pipeline

Полный ROS-путь подтверждён рабочим:

```text
Unity VR controller
→ /team_1/twist_command
→ /team_1/twist_command_safe
→ MoveIt Servo
→ /team_1/servo_joint_trajectory
→ /joint_states
```

- При движении контроллера Unity публикует ненулевой `TwistStamped` в frame `base_link`.
- Safety layer пропускает команду.
- MoveIt Servo публикует новую `JointTrajectory`.
- `/joint_states` содержит шесть ожидаемых имён UR3 в правильном порядке.
- ROS FK topic `/team_1/tcp_actual` изменяется при команде. Например, при команде «вправо» TCP изменился преимущественно по ROS `-Y`, а не по `Z`.

### Unity Articulation model

- В `WorkshopScene` старый `Ur3JointStateSubscriber` отключён.
- Новый `Ur3ArticulationJointStateSubscriber` включён на `ur3_with_gripper`.
- `base_link` и шесть arm `ArticulationBody` включаются компонентом во время Play; gravity отключается, base делается immovable.
- Debug test `0 → +10° → 0` подтвердил, что каждый из шести суставов двигает ожидаемое звено модели.
- Новый Unity read-only topic `/team_1/unity_joint_states` успешно опубликован.

### Численная проверка суставов

При исходной конфигурации `sign = +1` были сравнены:

```text
ROS /joint_states
Unity /team_1/unity_joint_states
```

Внутренние координаты Articulation совпали с входными ROS значениями с погрешностью порядка `10^-7 rad`.

Это доказывает, что Unity получает raw позиции из `/joint_states`, а не delta от стартовой позы, и Articulation drive физически достигает заданной координаты.

## Что найдено

### Несовпадение визуального UR3 и ROS TCP пока не объяснено

Визуальная кисть Unity не выглядит совпадающей с actual TCP marker: при некоторых Cartesian-командах marker двигался ожидаемо, а визуальная рука выглядела как движущаяся в другом направлении.

Поворот основания сам по себе **не является** критерием ошибки: Servo управляет Cartesian позой `tool0`, а не направлением вращения отдельного joint. Для проверки нужно сравнивать только `tool0` относительно `base_link`.

### Проверка tool0 ещё не завершена

Добавлен read-only publisher:

```text
Unity tool0 relative to Unity base_link
→ /team_1/unity_tool0
```

Его нужно сравнивать с:

```text
ROS tool0 relative to ROS base_link
→ /team_1/tcp_actual
```

Во время разработки publisher была допущена ошибка порядка преобразования координат, затем исправлена обратно на правильный вариант: сначала `tool0` вычисляется локально относительно Unity `base_link`, затем применяется Unity→FLU преобразование.

Поэтому все замеры `/team_1/unity_tool0`, сделанные **до последней перекомпиляции Unity после исправления**, нельзя использовать для выводов о знаках или URDF.

## Текущее состояние signs

В сцене сейчас сохранены значения `-1` для всех шести `Joint Signs` как временный эксперимент. Они **ещё не подтверждены** корректным сравнением `tool0` после последнего исправления publisher.

Не следует добавлять offsets или подгонять модель под текущую позу. Допустимое изменение для кинематического соответствия — только `sign` конкретного joint, если это подтвердит тест `tool0`.

## Отдельная проблема TF

Unity Console показала ошибку deserialization для `/tf`:

```text
ArgumentOutOfRangeException in TFMessageMsg.Deserialize
```

После неё `TcpActualSubscriber` пытался применить `NaN` к объекту `TCP_Actual`.

Для текущих проверок `/tf` не нужен. В `ROSConnection` следует временно выключить `Listen For TF Messages` и убрать `/tf`, `/tf_static` из списка TF topics. `/team_1/tcp_actual` остаётся отдельным `PoseStamped` topic и для этой проверки необходим.

## Следующий точный тест

1. Остановить Play, дождаться компиляции Unity, снова включить Play.
2. Оставить `Debug Mode = false`; fallback `Ur3JointStateSubscriber` должен быть выключен.
3. Отключить TF listener, чтобы исключить `NaN`.
4. В двух терминалах слушать:

```bash
ros2 topic echo /team_1/tcp_actual
ros2 topic echo /team_1/unity_tool0
```

5. Снять по одному сообщению в неподвижной позе и после короткой Cartesian-команды.
6. Сравнить изменения `position` двух topic. Только после этого принимать решение о `sign` каждого joint или проверять origin `xyz/rpy` двух URDF.
