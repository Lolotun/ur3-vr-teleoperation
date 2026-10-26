# Происхождение и перенос — 2026-10-24

Исходные папки сохранены. Это новый снимок исходников, а не импорт полной Git-истории.
Истории остаются в исходных репозиториях. Подробный список скопированных файлов,
исходных путей и SHA-256 до правок упаковки: [migration-manifest.json](migration-manifest.json).

| Источник | Зафиксированная версия | Что взято |
|---|---|---|
| ur3-vr-workshop | d3e8b68ac12d6d531788a941d1ee4830de73d7f0 | wifi_speedl.launch.py, три узла, служебные файлы пакета |
| unity_ros_teleoperation | c53b24e1217ae1f5a09d0b716e8fe318c5ac50da | отслеживаемые Assets, Packages, ProjectSettings, LICENSE, .gitignore |
| Robotics-Object-Pose-Estimation | 4515647 (upstream) | текст лицензии для уже имеющихся учебных Unity-ресурсов |

Unity upstream: https://github.com/leggedrobotics/unity_ros_teleoperation
ROS-TCP Endpoint: https://github.com/leggedrobotics/ROS-TCP-Endpoint
Учебные ресурсы: https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation

Твои интеграционные скрипты: ROS teleop_core, Unity Assets/Scripts и WorkshopScene.
Библиотеки ETH/RSL, модели UR3/Robotiq, Meta SDK и учебные материалы не следует
приписывать авторству интеграционного проекта. BSD-текст ETH сохранён в unity/LICENSE;
лицензия учебного проекта в licenses/. Вложенные notices сохранены с Assets.
package.xml исходного teleop_core декларирует BSD-3-Clause, но отдельного текста
лицензии backend в исходнике не было. Новое авторское уведомление не придумано.

## Что проверено против раздачи

В dist/ur3_vr_workshop_participant_2026-09-02 побайтово совпадают с основными
папками: ur_speedl_wifi_bridge.py, wifi_speedl.launch.py,
ControllerTwistPublisher.cs и WorkshopScene.unity.
В раздаче Unity Git URL переведены на HTTPS и вложен Meta SDK с исправлением Linux.
Здесь URL также HTTPS, а Git revisions закреплены по существующему lock-файлу.
Исправление SDK вынесено в воспроизводимый вспомогательный скрипт.

## Изменения упаковки

- setup.py регистрирует только три перенесённых узла и один launch.
- package.xml описывает зависимости выбранного пути.
- Новый entrypoint оставляет прежний блок wifi_speedl с теми же параметрами.
- Compose и root run.sh запускают один сервис; logs/topics/down обращаются к нему.
- Новая сборка имеет отдельный тег ur3-vr-teleoperation:humble.
- Старый workshop-образ доступен через TELEOP_IMAGE без пересборки.
- Dockerfile фиксирует ROS-TCP Endpoint revision 05a3b25a11d9639549ae128e471bd696430b91f7,
  прочитанный из локального рабочего образа. Apt-пакеты и базовый образ пока не
  закреплены по точным версиям/digest: битовая воспроизводимость не заявляется.

## Что исключено

Старые ROS day1/day2/day3, MoveIt Servo sim/real/dry и их адаптеры; незакоммиченная
правка servo_sim не входит в выбранный реальный путь и остаётся в старой папке.
Не перенесены pose-estimation проект целиком, URTeleoperation, dist, APK,
Docker tar, Library/Temp/Logs, рекламные GIF upstream и утилита ovr-platform-util.
Unity Assets сохранены консервативно: удаление неиспользуемых ресурсов и компонентов
требует отдельной проверки в Editor. Это не минимизированная до последнего файла сцена.

Исторические заметки сохранены в docs/history. Они описывают прежний workspace,
могут ссылаться на исключённые режимы и не заменяют текущую инструкцию setup.md.

## Дополнение 2026-10-26: возвращён виртуальный режим

По запросу перенесены servo_sim.launch.py, servo_joint_state_adapter,
servo_twist_enable, tcp_actual_publisher и четыре config-файла из исходного
ur3-vr-workshop. Предыдущий список исключений выше описывает первый этап переноса:
servo sim теперь включён. Исходный launch перенесён с локальной незакоммиченной
правкой вращения (rotational scale 0.2, angular cap 0.3), без изменения поведения.
В адаптере только import констант заменён на ur3_model.py, чтобы не тащить
неиспользуемый ikpy solver. Исходные хеши добавлены в migration-manifest.json.
Добавлены зависимости MoveIt, регистрация трёх узлов и config в setup.py,
режим entrypoint servo_sim и команды run.sh servo/sim.
