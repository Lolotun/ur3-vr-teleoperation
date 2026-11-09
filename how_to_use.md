
# Управление UR3 и сбор демонстраций

Все команды выполняются на хосте. Заходить внутрь контейнеров не нужно.

- `~/Kosenkov/ur3-vr-teleoperation` — управление UR3 и Robotiq.
- `~/Kosenkov/vr-dagger` — камеры, запись rosbag, проверка и конвертация в LeRobot.

## 1. Запуск робота

Реальное управление UR3 и захватом из Unity/Quest:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
RTDE_ENABLE_MOTION=true RTDE_ENABLE_GRIPPER=true ./scripts/run.sh rtde
```

Для проверки состояния без движения и управления захватом — вместо предыдущей команды:

```bash
RTDE_ENABLE_MOTION=false RTDE_ENABLE_GRIPPER=false ./scripts/run.sh rtde
```

Preview нужен для проверки связи. Для сбора демонстраций с выполненными движениями
используется реальный режим; конвертер не принимает RTDE-статус PREVIEW.

Состояние контейнера, логи и список ROS-топиков:

```bash
./scripts/run.sh ps
./scripts/run.sh logs
./scripts/run.sh topics
```

Ctrl+C в просмотре логов закрывает только просмотр, контейнер продолжает работать.
Команды, идущие после `logs`, выполнять после выхода из просмотра или в другом терминале.

## 2. Предпросмотр камер

Для отдельного preview камеры должны быть освобождены от контейнера записи.
Сначала завершить активную запись через Ctrl+C, затем:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh down

# Показать доступные устройства
uv run --no-project --script scripts/camera_preview.py --list

# Подставить актуальные устройства: сначала wrist, затем external
uv run --no-project --script scripts/camera_preview.py \
  --devices /dev/video6 /dev/video4
```

На момент настройки `/dev/video6` — wrist Logitech C930e, `/dev/video4` — RGB
RealSense D435. После переподключения номера могут измениться: см. раздел о смене камер.

Q или Esc закрывает preview. Закрыть его перед запуском контейнера камер.

## 3. Камеры для записи

Сначала запустить teleop, затем камеры:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh cameras
bash scripts/capture.sh check
```

`check` проверяет живые изображения и обратную связь робота. ROS_DOMAIN_ID и
ROS_LOCALHOST_ONLY подхватываются из контейнера teleop, если они не переопределены
переменными окружения.

Посмотреть частоту кадров и ошибки:

```bash
bash scripts/capture.sh logs
```

Ожидаем около 30 FPS с каждой камеры. Ctrl+C закрывает только просмотр логов.

После запуска или перезапуска камер один раз изменить Grip при отпущенном Trigger,
чтобы пассивный наблюдатель запомнил текущую цель захвата. Между следующими
записями повторять это не нужно. `check` показывает известную цель или `UNKNOWN`.
Запись не ждёт Grip, но неизвестная начальная цель помешает полной конвертации:
конвертер не подменяет её нулём и не обрезает начало до первого захвата.

## 4. Запись одного эпизода

Каждый раз использовать новое имя:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh record \
  --name demo07 \
  --task 'pick blue ball and place on the tube'
```

Дождаться `All requested topics are subscribed`, затем выполнять демонстрацию.

- Начальная пауза сохраняется с первого полного набора данных камер и робота.
- Отпускания Trigger, паузы и перехваты входят в тот же эпизод.
- В паузе действие — фактическая поза TCP и удерживаемая цель захвата.
- **Ctrl+C в терминале записи — конец эпизода.** Пауза до Ctrl+C тоже сохраняется.
- Контейнеры камер и teleop после остановки записи продолжают работать.

После остановки выводятся количества записанных сообщений. Данные находятся здесь:

```text
~/Kosenkov/vr-dagger/data/bags/demo07/episode.json
~/Kosenkov/vr-dagger/data/bags/demo07/bag/metadata.yaml
~/Kosenkov/vr-dagger/data/bags/demo07/bag/*.db3
```

Для следующей попытки снова выполнить `record`, например с `--name demo08`.
Не нужно перезапускать камеры и teleop между эпизодами.

### Запись кнопкой B с автоматической нумерацией

В обновлённом Unity-приложении **B на правом контроллере** отправляет событие
`/vr/dataset/toggle`. APK с этой функцией собран:
`unity/build/UR3Workshop_dataset_buttons.apk` в репозитории ur3-vr-teleoperation.
Его нужно установить на Quest; старый установленный APK кнопку записи не отправляет. Компонент уже находится
в WorkshopScene, добавлять его вручную не нужно.

При настроенном adb и подключённом Quest установка с хоста:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
adb install -r unity/build/UR3Workshop_dataset_buttons.apk
```

Один раз задать общую задачу в конфиге:

```bash
cd ~/Kosenkov/vr-dagger
nano config/recording.json
```

```json
{
  "task": "pick blue ball and place on the tube"
}
```

При запущенном teleop включить приёмник B; камеры также запустятся, если были остановлены:

```bash
bash scripts/capture.sh buttons
bash scripts/capture.sh button-logs
```

Дождаться `DATASET READY: press right B`. После запуска камер начальная цель
захвата должна быть известна наблюдателю, как описано в разделе 3.

- Первое нажатие B — начало записи; в логах появятся STARTING и затем RECORDING.
  Начинать движение после RECORDING. Удержание B не вызывает повторных переключений.
- Второе нажатие B — STOPPING, сохранение bag и затем IDLE.
- Дождаться IDLE перед следующим B: следующий эпизод получит новый номер.
- Паузы и перехваты Trigger остаются внутри эпизода. A продолжает управлять
  фиксацией захвата, Trigger — движением.
- Повторно доставленное событие с тем же номером не переключает запись ещё раз.

Папки создаются в `data/bags/demo1`, `demo2` и далее. Нумерация продолжается после
максимального существующего номера, включая старые имена `demo01`, `demo06`.
Например, при существующих demo01…demo06 следующая запись будет `demo7`.
Существующие записи не перезаписываются. Общая задача копируется в каждый episode.json.

Кнопка управляет записью исходного rosbag. Проверка и конвертация в LeRobot остаются
отдельными командами, например для автоматически созданного demo7:

```bash
uv run python scripts/bag_dataset.py inspect data/bags/demo7
uv run python scripts/bag_dataset.py convert data/bags/demo7 \
  --root data/lerobot/demo7 --repo-id local/ur3-demo
```

Ctrl+C в `button-logs` закрывает только логи. Остановить приёмник B:

```bash
bash scripts/capture.sh buttons-stop
```

Если запись активна, сервис корректно завершит её. Камеры и teleop продолжат
работать. `capture.sh down` завершает и приёмник, и камеры. Для смены общей задачи
остановить приёмник, отредактировать recording.json и снова выполнить `buttons`.
Одновременно использовать ручной `record` и приёмник B не нужно.

Обратный статус приходит в Unity через `/dataset/recording_status` и отображается
в логах Unity как `[Dataset] ...`.

## 5. Проверка и конвертация

```bash
cd ~/Kosenkov/vr-dagger
uv run python scripts/bag_dataset.py inspect data/bags/demo07
```

Ожидаем `conversion_ready: true`, `episode_count: 1` и пустой `invalid_after_start`.
При ошибке смотреть причины в отчёте; не увеличивать `--max-age` просто для обхода проверки.

Конвертация в новую выходную папку:

```bash
uv run python scripts/bag_dataset.py convert data/bags/demo07 \
  --root data/lerobot/demo07 \
  --repo-id local/ur3-demo
```

Выходная папка должна отсутствовать. Если нужна повторная конвертация, выбрать новое
имя, например `--root data/lerobot/demo07_full`. На Hugging Face ничего не загружается.

Если в старом bag начальное закрытие неизвестно, но оно было независимо известно
оператору, его можно явно указать через `--initial-gripper 0.0` в `inspect` и
`convert`. Это утверждение о начале записи, а не команда открыть захват.

### Конвертация всех demo сразу

```bash
cd ~/Kosenkov/vr-dagger
uv run python scripts/convert_all_demos.py
```

Каждая папка `data/bags/demoN` превращается в отдельный dataset
`data/lerobot/demoN`. Готовые пропускаются; ошибки не останавливают обработку
следующих папок. Неполные существующие результаты не перезаписываются.
Логи: `data/lerobot/_conversion_logs/`, итог: `data/lerobot/_batch_report.json`.
Общий код завершения 1 означает, что хотя бы одна конвертация не прошла.

Показать список действий без конвертации:

```bash
uv run python scripts/convert_all_demos.py --dry-run
```

## 6. Просмотр записанных видео


После конвертации, по одному:

```bash
cd ~/Kosenkov/vr-dagger
ffplay data/lerobot/demo07/videos/observation.images.wrist/chunk-000/file-000.mp4
```

```bash
ffplay data/lerobot/demo07/videos/observation.images.external/chunk-000/file-000.mp4
```

Q закрывает проигрыватель. Это просмотр видео без исполнения действий на роботе.

## 7. Смена или переподключение камер

### Камеры находятся автоматически?

**Нет: скрипт использует устройства из `~/Kosenkov/vr-dagger/config/cameras.env`.**
Linux создаёт ссылки `/dev/v4l/by-id/...`, и они обычно сохраняются при изменении
номеров `/dev/videoN`. Поэтому прежняя камера может снова заработать без правки
конфига — если её USB-имя осталось прежним. Скрипт не выбирает новую камеру сам.

Текущие настройки:

```dotenv
WRIST_DEVICE=/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_CD2C619E-video-index0
EXTERNAL_DEVICE=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435-video-index0
```

У RealSense USB-имя уже менялось после переподключения: раньше оно содержало
серийный номер `944523029596`. Такие изменения требуют обновить конфиг.

### Как заменить камеру или исправить путь

1. Завершить текущую запись через Ctrl+C и остановить контейнер камер:

   ```bash
   cd ~/Kosenkov/vr-dagger
   bash scripts/capture.sh down
   ```

2. Подключить камеры и посмотреть актуальные имена и номера:

   ```bash
   ls -l /dev/v4l/by-id/
   uv run --no-project --script scripts/camera_preview.py --list
   ```

3. Проверить нужные видеопотоки через preview, подставив найденные устройства:

   ```bash
   uv run --no-project --script scripts/camera_preview.py \
     --devices /dev/video6 /dev/video4
   ```

   Проверить, где wrist, а где внешняя камера, затем закрыть preview.
   У одной физической камеры может быть несколько V4L2-узлов: depth, IR, RGB и
   metadata. Metadata-узел изображения не выдаёт. Для RealSense нужно выбрать
   именно RGB; один только номер или суффикс `index0` не доказывает тип потока.

4. Открыть конфиг и заменить соответствующие пути:

   ```bash
   nano config/cameras.env
   ```

   `WRIST_DEVICE` — камера на кисти; `EXTERNAL_DEVICE` — внешняя камера.
   Предпочтительны проверенные `/dev/v4l/by-id/...` ссылки. Путь `/dev/videoN`
   тоже допустим, но после переподключения номер может измениться.

5. Убрать старые переопределения окружения и запустить камеры заново:

   ```bash
   unset WRIST_DEVICE EXTERNAL_DEVICE
   bash scripts/capture.sh cameras
   bash scripts/capture.sh check
   bash scripts/capture.sh logs
   ```

   Переменные окружения имеют приоритет над `config/cameras.env`. Если после
   редактирования используется старый путь, проверить их командой:

   ```bash
   printenv WRIST_DEVICE EXTERNAL_DEVICE
   ```

После физического переподключения USB использовать именно `down` → `cameras`,
чтобы Docker заново пробросил устройства. Простой `docker restart` может сохранить
старую привязку. Пересобирать образ из-за смены пути камеры не нужно.
Остановка `capture.sh down` не останавливает teleop.

## 8. Остановка после работы

Сначала завершить активную запись через Ctrl+C, затем:

```bash
cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh down

cd ~/Kosenkov/ur3-vr-teleoperation
./scripts/run.sh down
```

Bag и датасеты на хосте сохраняются.

## 9. Сборка образов

Образы уже подготовлены. После изменения Dockerfile или состава зависимостей:

```bash
cd ~/Kosenkov/ur3-vr-teleoperation
./scripts/run.sh build

cd ~/Kosenkov/vr-dagger
bash scripts/capture.sh build
```

После изменения ROS-кода teleop также нужна новая сборка: он находится внутри
образа. Скрипты камер и записи vr-dagger подключены через volume; после их изменения
достаточно перезапустить соответствующий процесс или контейнер.
