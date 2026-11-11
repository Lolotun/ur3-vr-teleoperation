# UR3: запись демонстраций, обучение и запуск политики

Команды выполняются на хосте. Используются два проекта:

- `~/Kosenkov/ur3-vr-teleoperation` — UR3, Robotiq, VR и выбор источника управления.
- `~/Kosenkov/vr-dagger` — камеры, запись bag, датасеты, обучение и инференс.

Контейнер `teleop` управляет роботом. Capture-контейнеры обслуживают камеры,
запись и ROS-bridge политики. Обучение и LeRobot работают на хосте в окружении
`uv`; bridge связывает их с ROS через `data/ipc/ur3_policy.sock`.
Устанавливать `rclpy` в окружение LeRobot не требуется.

## 1. Подготовка

В `ur3-vr-teleoperation/.env` должен быть указан правильный `ROBOT_IP`.
Робот, TCP, сцена и расположение камер должны соответствовать задаче.

```bash
cd ~/Kosenkov/vr-dagger
uv sync
```

Образы должны быть собраны. После изменений ROS-кода или зависимостей:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
./scripts/run.sh build

cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh build
```

Скрипты capture подключены через volume. После изменения их настроек
перезапустить соответствующий сервис.

## 2. Запуск робота для демонстраций

Включить реальное управление UR3 и Robotiq, затем выбрать VR:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
TELEOP_CONTROL_MODE=STOP RTDE_ENABLE_MOTION=true RTDE_ENABLE_GRIPPER=true \
  RTDE_ENABLE_GRIPPER_FEEDBACK=true ./scripts/run.sh rtde
./scripts/run.sh mode VR
```

Запустить Unity/Quest. После выбора VR отпустить Trigger и нажать заново
для привязки руки. Grip задаёт закрытие захвата. Отпускание Trigger позволяет
перехватить контроллер без завершения эпизода.

Проверка контейнера и логов:

```bash
./scripts/run.sh ps
./scripts/run.sh logs
```

Ctrl+C в просмотре логов завершает только просмотр.

Для остановки управления:

```bash
./scripts/run.sh mode STOP
```

STOP прекращает управление и не отправляет команду открытия захвата.
Для проверки связи без исполнения вместо реального запуска использовать:

```bash
TELEOP_CONTROL_MODE=STOP RTDE_ENABLE_MOTION=false RTDE_ENABLE_GRIPPER=false \
  RTDE_ENABLE_GRIPPER_FEEDBACK=true ./scripts/run.sh rtde
```

Для демонстраций с выполненными движениями нужен реальный режим:
конвертер не принимает статус RTDE `PREVIEW`.

## 3. Камеры и готовность записи

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh cameras
bash scripts/capture.sh check
```

`check` проверяет живые данные. ROS_DOMAIN_ID и ROS_LOCALHOST_ONLY подхватываются
из работающего teleop, если не переопределены переменными окружения.

После запуска камер один раз изменить Grip при отпущенном Trigger, чтобы
наблюдатель запомнил текущую цель захвата. `check` должен показывать известную
цель, а не `UNKNOWN`. Повторять между эпизодами не требуется.
Для обучения также нужна действующая измеренная обратная связь Robotiq.

Частота и ошибки камер:

```bash
bash scripts/capture.sh logs
```

Ожидается около 30 FPS с обеих камер. Ctrl+C закрывает просмотр.

## 4. Запись нескольких эпизодов кнопкой B

Указать одну задачу для серии в `vr-dagger/config/recording.json`:

```bash
cd ~/Kosenkov/vr-dagger
nano config/recording.json
```

Поле `task`, например: `pick blue ball and place on the tube`.
Одинаковую задачу описывать одинаковой строкой во всех эпизодах.

Запустить рекордер:

```bash
bash scripts/capture.sh buttons
bash scripts/capture.sh button-logs
```

- Дождаться готовности рекордера.
- Нажать B — начинается запись в очередную свободную папку `demoN`.
- Выполнить задачу. Паузы, отпускание Trigger и перехваты входят в эпизод.
- Нажать B снова — запись останавливается и bag финализируется.
- Дождаться завершения сохранения, вернуть сцену в исходное состояние и повторить.

Начало эпизода — первый полный пригодный набор данных после старта записи,
а не первое нажатие Trigger. Конец задаётся остановкой записи.

Результат на хосте:

```text
data/bags/demoN/episode.json
data/bags/demoN/bag/metadata.yaml
data/bags/demoN/bag/*.db3
```

Остановить сервис B после завершения эпизода:

```bash
bash scripts/capture.sh buttons-stop
```

После изменения `recording.json` также выполнить `buttons-stop` → `buttons`.
Не запускать ручную запись одновременно с записью по B.

### Ручная запись одного эпизода

При остановленном сервисе B:

```bash
bash scripts/capture.sh record \
  --name demo50 \
  --task 'pick blue ball and place on the tube'
```

Имя папки должно быть свободным. Дождаться `All requested topics are subscribed`,
выполнить демонстрацию и нажать Ctrl+C для остановки и сохранения.

## 5. Проверка и конвертация в LeRobot

Проверить отдельную запись, подставив её имя:

```bash
cd ~/Kosenkov/vr-dagger
uv run --frozen python scripts/bag_dataset.py inspect data/bags/demo50 \
  --include-gripper-state
```

Нужны `conversion_ready: true`, `episode_count: 1`, пустой `invalid_after_start`.
Отсутствующие или устаревшие измерения, ошибки RTDE и незавершённая запись
требуют проверки. Не увеличивать допустимый возраст данных для обхода ошибок.

Конвертировать всю серию:

```bash
uv run --frozen python scripts/convert_all_demos.py \
  --output-dir data/lerobot_v3 \
  --include-gripper-state
```

Результаты: `data/lerobot_v3/demoN`. Отчёт: `data/lerobot_v3/_batch_report.json`,
логи: `data/lerobot_v3/_conversion_logs/demoN.log`.
Повторный запуск добавляет новые эпизоды и пропускает готовые совместимые.
Несовместимые или неполные выходные папки отмечаются ошибкой и не перезаписываются.
При ошибке одного эпизода остальные продолжают конвертироваться.

При необходимости конвертация одной записи:

```bash
uv run --frozen python scripts/bag_dataset.py convert data/bags/demo50 \
  --root data/lerobot_v3/demo50 \
  --repo-id local/ur3-demo \
  --include-gripper-state
```

Выходная папка должна отсутствовать. Исходные bag сохраняются.
На Hugging Face данные автоматически не загружаются.

### Что получает модель

`observation.state` — 16 чисел:

| Компоненты | Значение |
| --- | --- |
| 6 | Измеренные углы суставов |
| 3 | Фактическая позиция TCP в метрах, система `base` |
| 6 | Фактическая ориентация TCP: первые две колонки матрицы вращения |
| 1 | Измеренное закрытие Robotiq, от 0 до 1 |

Дополнительно подаются RGB-изображения `wrist` и `external`.

`action` — 10 чисел: измеренная TCP-поза **следующего кадра** (3 + 6)
и удерживаемая **целевая команда** закрытия Robotiq в текущий момент (1).
При 30 FPS сдвиг TCP составляет 33,3 мс; в конце эпизода удерживается последняя
измеренная поза. Команда захвата и наблюдения не сдвигаются.

Измеренное закрытие в state и целевое закрытие в action различаются:
при захвате предмет может остановить пальцы раньше заданной позиции.
POS/PRE/OBJ/STA сохраняются дополнительно в `gripper_feedback.jsonl`.

Просмотр видео после конвертации, по одному:

```bash
ffplay data/lerobot_v3/demo50/videos/observation.images.wrist/chunk-000/file-000.mp4
ffplay data/lerobot_v3/demo50/videos/observation.images.external/chunk-000/file-000.mp4
```

Q закрывает проигрыватель.

## 6. Объединение эпизодов для обучения

Создать отдельный объединённый набор:

```bash
cd ~/Kosenkov/vr-dagger
uv run --frozen python scripts/merge_gripper_v3.py \
  --source data/lerobot_v3 \
  --root data/lerobot/ur3_gripper_49
```

Скрипт проверяет схему, метки TCP следующего кадра и отсутствие ошибок внутри
эпизодов. Неполные эпизоды пропускаются. Существующий результат не перезаписывается.
В итоговом выводе проверить `total_episodes`: для примера ниже должно быть **49**.

После сбора новых демонстраций повторить конвертацию, затем объединить в новую
папку, например `data/lerobot/ur3_gripper_60`. Готовый объединённый набор
самостоятельно не пополняется.

## 7. Обучение Diffusion Policy

Пример: 49 эпизодов одной задачи, **44 train / 5 validation**, 4000 шагов:

```bash
cd ~/Kosenkov/vr-dagger
ACTUAL_DATASET_ROOT="$PWD/data/lerobot/ur3_gripper_49" \
ACTUAL_TRAIN_OUTPUT="$PWD/outputs/train/diffusion_ur3_gripper_49_run1" \
ACTUAL_TRAIN_STEPS=4000 \
bash scripts/train_gripper_v3.sh \
  --dataset.eval_split=0.1 \
  --save_freq=1000 \
  --job_name=diffusion_ur3_gripper_49_run1
```

| Параметр | Что менять |
| --- | --- |
| `ACTUAL_DATASET_ROOT` | Папка объединённого датасета из раздела 6 |
| `ACTUAL_TRAIN_OUTPUT` | Новая папка для модели и конфигурации; должна отсутствовать |
| `ACTUAL_TRAIN_STEPS` | Количество шагов обучения |
| `--dataset.eval_split` | Доля эпизодов для валидации |
| `--save_freq` | Интервал сохранения чекпоинтов в шагах |
| `--job_name` | Название запуска на графиках |

Валидация берёт последние `ceil(N × eval_split)` эпизодов **каждой задачи**,
целыми эпизодами, без случайного перемешивания между train и validation.
При 49 эпизодах одной задачи и `0.1` это ровно 5 эпизодов; при другом количестве
или нескольких задачах число изменится. В начале лога проверить
`Train/eval split: 44 train, 5 eval`.

Скрипт запускает обучение с нуля. Параметры: batch 16, два наблюдения,
горизонт 16, исполнение по 8 действий. Train loss пишется каждые 25 шагов,
validation loss оценивается каждые 250 шагов, максимум на 128 примерах.
При отсутствии переопределений скрипт использует 10 000 шагов, split 0.2
и сохранение каждые 2000 шагов — поэтому для указанного примера передавать
все параметры из команды выше.

Для сохранения консольного лога можно добавить в конец команды:

```bash
2>&1 | tee train_ur3_gripper_49_run1.log
```

Это продолжение команды обучения на той же строке; само по себе обучение
не запускает. В другом терминале открыть графики:

```bash
cd ~/Kosenkov/vr-dagger
uv run --frozen trackio show --project ur3-diffusion
```

Чекпоинты находятся в:

```text
outputs/train/diffusion_ur3_gripper_49_run1/checkpoints/
```

`last/pretrained_model` — последний сохранённый чекпоинт.
Для конкретного шага выбрать папку вида `001000/pretrained_model`.
Для следующего запуска изменить выходную папку и `job_name`, например на `run2`.

## 8. Проверка политики без исполнения

Сначала запустить teleop без разрешения движения:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
TELEOP_CONTROL_MODE=STOP RTDE_ENABLE_MOTION=false RTDE_ENABLE_GRIPPER=false \
  RTDE_ENABLE_GRIPPER_FEEDBACK=true ./scripts/run.sh rtde

cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh policy-bridge
```

Этот bridge запускает камеры и принимает наблюдения робота. Выбрать модель
**явно**, чтобы все проверки использовали нужный чекпоинт:

```bash
export POLICY_PATH="$PWD/outputs/train/diffusion_ur3_gripper_49_run1/checkpoints/last/pretrained_model"
test -f "$POLICY_PATH/config.json"

uv run --frozen python scripts/diagnose_policy.py \
  --policy "$POLICY_PATH" --scheduler DDIM --steps 20 --frames 64
```

Диагностика не отправляет действия. Она показывает расстояние и угол от
фактического TCP до цели, ограниченную цель, `would_STOP` и диапазон команды `g`.
Проверить соответствие сцены демонстрациям, ориентацию TCP и поведение захвата.
`would_STOP=0` означает прохождение перечисленных проверок цели; успешность
задачи оценивается отдельно.

Dry-run полного rollout на 30 секунд:

```bash
DIFFUSION_SCHEDULER=DDIM DIFFUSION_STEPS=20 \
  bash scripts/rollout.sh --duration=30
```

`POLICY_PATH` должен быть экспортирован в этом терминале. Bridge публикует
предсказания, но не отправляет команды исполнения. Скрипт автоматически
включает измеренное состояние захвата по конфигурации выбранной модели.

Логи bridge в другом терминале:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh policy-logs
```

## 9. Исполнение политики на роботе

Подготовить сцену и положение робота как в демонстрациях. Завершить dry-run.

**Терминал 1 — разрешить реальное управление, оставив источник STOP:**

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
TELEOP_CONTROL_MODE=STOP RTDE_ENABLE_MOTION=true RTDE_ENABLE_GRIPPER=true \
  RTDE_ENABLE_GRIPPER_FEEDBACK=true ./scripts/run.sh rtde
```

**Терминал 2 — подготовить исполняющий bridge и выбрать веса:**

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh buttons-stop
bash scripts/capture.sh policy-bridge-execute
export POLICY_PATH="$PWD/outputs/train/diffusion_ur3_gripper_49_run1/checkpoints/005000/pretrained_model"
```

**Терминал 1 — выбрать источник команд:**

```bash
./scripts/run.sh mode POLICY
```

**Терминал 2 — запустить исполнение:**

```bash
DIFFUSION_SCHEDULER=DDIM DIFFUSION_STEPS=20 \
  bash scripts/rollout.sh --execute --duration=30
```

`--duration` задаёт длительность rollout в секундах. `POLICY_PATH` выбирает
чекпоинт; для другого сохранённого шага заменить часть пути после `checkpoints/`.
DDIM20 задаёт способ и число шагов денойзинга при инференсе, переобучать для
выбора этого sampler не требуется.

Политика выдаёт блок из 8 действий, которые синхронный rollout отправляет
последовательно с целевой частотой 30 Гц. На расчёте нового блока возможна задержка;
итоговую частоту смотреть в `effective cadence`. Первые 8 действий адаптера —
прогрев без исполнения. Bridge переводит 6D-ориентацию в кватернион,
далее штатный контроллер позы и Servo формируют суставные команды.

### Остановка и возврат к VR

Для прекращения управления выбрать STOP из терминала 1, затем завершить rollout
через Ctrl+C в терминале 2:

```bash
./scripts/run.sh mode STOP
```

После завершения rollout для ручного управления:

```bash
./scripts/run.sh mode VR
```

Отпустить Trigger и нажать заново. После смены режима для возврата к политике
снова явно выбрать POLICY и запустить **новый** rollout.

## 10. Если исполнение остановилось

`service "teleop" is not running` означает, что отсутствует контейнер управления
роботом. Capture-контейнеры его не заменяют: сначала запустить `run.sh rtde`.

`No such file or directory` при подключении к `ur3_policy.sock` означает,
что bridge не запущен или недоступен. Запустить `policy-bridge` для проверки
либо `policy-bridge-execute` для исполнения.

Ошибка `Select POLICY explicitly … observation epoch must match` требует
проверки первичной причины в статусе управления:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
docker compose exec -T teleop bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic echo /teleop/control_status std_msgs/msg/String --once'
```

Проверить `mode`, `reason`, RTDE и эпоху управления. Исполнение требует одновременно
реального RTDE, исполняющего bridge, режима POLICY и `rollout.sh --execute`.
После отказа устранить указанную причину, явно выбрать POLICY и запустить новый
rollout. Очередь действий из предыдущей эпохи управления не переиспользуется.

Пределы настраиваются в `ur3-vr-teleoperation/.env`. Значения по умолчанию:

| Параметр | Значение |
| --- | --- |
| `POLICY_TARGET_DISTANCE` | 0.03 м — ограничение расстояния отправляемой цели от TCP |
| `POLICY_TARGET_ROTATION` | 0.17453 рад — ограничение угла отправляемой цели |
| `POLICY_MAX_DISTANCE` | 0.20 м — предел исходного предсказания, превышение даёт отказ |
| `POLICY_MAX_ROTATION` | 0.5 рад — предел угла исходного предсказания |
| `POLICY_MAX_LINEAR_SPEED` | 0.020 м/с |
| `POLICY_MAX_ANGULAR_SPEED` | 0.12 рад/с |

Рабочая область по умолчанию: x/y от −0.6 до 0.6 м, z от −0.01 до 1 м,
в системе `base`. Фактические лимиты показывает диагностика `Target limits`.
Также действуют Servo, collision guard и watchdog. `clipped=True` означает
ограничение цели; `would_STOP` показывает причину отклонения.
При выходе за workspace или отказе collision guard проверить предсказания,
сцену и исходную позу. Не расширять пределы для обхода причины отказа.

### Измеренная обратная связь Robotiq

```bash
docker compose exec -T teleop bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic echo /teleop/gripper_state std_msgs/msg/String --once'
```

Нужны `valid=true`, активированный захват (`activation_status=3`) и свежие данные.
`position_norm` — измеренное закрытие 0..1; `requested_norm` — цель контроллера.
`object_status`: 0 — движение, 1/2 — контакт, 3 — достижение цели.
Обратная связь читается примерно на 10 Гц. Измеренная позиция для модели должна
быть не старше 0.25 с; команда или ноль вместо пропавшего измерения не подставляются.

### Запись и разбор rollout

Перед запуском политики, в `vr-dagger`:

```bash
bash scripts/capture.sh trace-start --duration 180
bash scripts/capture.sh trace-logs
```

После теста:

```bash
bash scripts/capture.sh trace-stop
uv run --frozen python scripts/policy_trace.py analyze data/traces/policy_YYYYMMDD_HHMMSS
```

Подставить путь из лога. Trace сохраняет предсказания, команды, статусы,
измеренные состояния и выборочные кадры камер; итог — `summary.json`.
Остановка trace не останавливает управление роботом.

## 11. Смена или переподключение камер

Камеры выбираются из `vr-dagger/config/cameras.env`, автоматически новая камера
не подбирается. Предпочтительны стабильные ссылки `/dev/v4l/by-id/...`:
номера `/dev/videoN` могут измениться после переподключения.

Завершить активный эпизод или rollout, затем:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh down
ls -l /dev/v4l/by-id/
uv run --no-project --script scripts/camera_preview.py --list
```

Подставить найденные устройства для preview: сначала wrist, затем external:

```bash
uv run --no-project --script scripts/camera_preview.py \
  --devices /dev/video6 /dev/video4
```

Проверить изображения, закрыть окно через Q или Esc. Для RealSense нужен RGB-поток:
metadata, IR и depth — другие V4L2-узлы. Затем прописать пути:

```bash
nano config/cameras.env
```

`WRIST_DEVICE` — камера на кисти, `EXTERNAL_DEVICE` — внешняя камера.
Переменные окружения имеют приоритет над файлом. Перезапуск:

```bash
unset WRIST_DEVICE EXTERNAL_DEVICE
bash scripts/capture.sh cameras
bash scripts/capture.sh check
```

После USB-переподключения использовать `down` → запуск сервиса, чтобы Docker
заново пробросил устройства. Пересборка из-за изменения пути камеры не нужна.
Если снова нужен режим политики, запустить соответствующий bridge по разделу 8 или 9.

## 12. Место на диске и завершение работы

Две raw RGB-камеры 640×480 при 30 FPS занимают около 50–55 МБ/с в bag.
Для записи предусмотрен резерв 2 ГиБ и общая блокировка ручного/B-рекордера.
При нехватке места запись финализируется с `stop_reason=low_disk_space`;
такой эпизод требует проверки и автоматически не конвертируется.
Отказы RTDE/collision guard также не включаются в обучающий набор автоматически.

Свободное место:

```bash
df -h ~/Kosenkov/vr-dagger
```

Сначала завершить активный эпизод и дождаться сохранения. Для политики выбрать STOP
и завершить rollout. Затем остановить сервисы:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh down

cd ~/Kosenkov/ur3-vr-teleoperation
./scripts/run.sh down
```

Bag, датасеты и веса остаются на хосте.

## 13. Обучение на отобранных демонстрациях с заданной валидацией

Для набора до `demo70` включительно исключены сомнительные демонстрации:
`2 4 6 7 11 15 16 17 19 21 22 23 24 26 27 29 31`.
Исходные записи и отдельные конвертированные эпизоды сохраняются.
Валидация: `demo5`, `demo14`, `demo55`, `demo59`, `demo63`, `demo65`.
Первые две — из ранних записей, четыре остальные — из последних двадцати.
Итог: **43 train / 6 validation**, всего 49 эпизодов и 28 164 кадра.

Воспроизводимое объединение (папка результата должна отсутствовать):

```bash
cd ~/Kosenkov/vr-dagger
uv run --frozen python scripts/merge_gripper_v3.py \
  --source data/lerobot_v3 \
  --root data/lerobot/ur3_gripper_upto70_curated \
  --max-demo 70 \
  --exclude 2 4 6 7 11 15 16 17 19 21 22 23 24 26 27 29 31 \
  --validation-demos 5 14 55 59 63 65
```

`--max-demo` включает указанный номер. `--exclude` исключает номера из объединения.
`--validation-demos` задаёт конкретные эпизоды проверки. Скрипт помещает их
в конец объединённого набора и сохраняет соответствие в `merge_provenance.json`.
Такой явный holdout поддерживается для одной задачи. Если заданный эпизод
недоступен или одновременно исключён, объединение завершится ошибкой.

`train_gripper_v3.sh` автоматически читает holdout из manifest и вычисляет
долю для штатного разделения LeRobot. **Не передавать `--dataset.eval_split`
для набора с явно заданными `--validation-demos`**, чтобы не изменить выбор.
Если явного holdout нет, доля по умолчанию 0.2; её можно переопределить,
как в разделе 7.

Запуск отдельной модели с нуля, без изменения предыдущих весов:

```bash
cd ~/Kosenkov/vr-dagger
ACTUAL_DATASET_ROOT="$PWD/data/lerobot/ur3_gripper_upto70_curated" \
ACTUAL_TRAIN_OUTPUT="$PWD/outputs/train/diffusion_ur3_gripper_upto70_curated_run1" \
ACTUAL_TRAIN_STEPS=10000 \
bash scripts/train_gripper_v3.sh \
  --save_freq=1000 \
  --job_name=diffusion_ur3_gripper_upto70_curated_run1
```

В начале лога проверить `Train/eval split: 43 train, 6 eval`.
Сохранение каждые 1000 шагов, validation loss — каждые 250, train loss — каждые 25.
Лог запуска, выполненного агентом:

```bash
tail -f outputs/train/logs/diffusion_ur3_gripper_upto70_curated_run1.log
```

Для проверки/инференса после сохранения чекпоинта выбрать его явно:

```bash
export POLICY_PATH="$PWD/outputs/train/diffusion_ur3_gripper_upto70_curated_run1/checkpoints/last/pretrained_model"
```

Далее использовать разделы 8–9. Выбор переменной сам по себе не запускает робота.
