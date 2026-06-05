# robot_ros2_RTK

ROS 2 Jazzy workspace для робота с Hydra, RTK/одометрией, RealSense, YOLO
сегментацией и удаленной визуализацией через RViz.

Репозиторий сейчас оформлен как monorepo: все пакеты, которые нужны этому
workspace, лежат внутри `src/`. Поэтому после клонирования не нужно отдельно
восстанавливать вложенные git-репозитории.

## Быстрый запуск

На компьютере робота:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py
```

На ноутбуке в той же локальной сети:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hydra_visualizer laptop_rviz.launch.yaml
```

Ноутбучный RViz подписывается на граф сцены и на YOLO-картинку с частотой
примерно 1 Гц. Полный поток камеры на ноутбук не передается.

## Первая сборка на новой машине

Установить базовые зависимости:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-vcstool ros-dev-tools
sudo apt install -y ros-jazzy-gtsam nlohmann-json3-dev pybind11-dev
sudo apt install -y libgoogle-glog-dev libgtest-dev libyaml-cpp-dev libzmqpp-dev
```

Если `rosdep` еще не инициализирован:

```bash
sudo rosdep init
rosdep update
```

Установить зависимости пакетов и собрать workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Пакетам `kimera_rpgo`, `kimera_pgmo` и `hydra` нужен GTSAM. На Ubuntu 24.04 с
ROS Jazzy он ставится пакетом `ros-jazzy-gtsam`.

## Внешний YOLO workspace

Launch-файлы ожидают, что YOLO workspace находится здесь:

```text
~/ros2_yolo_ws
```

Используемый скрипт сегментации:

```text
~/ros2_yolo_ws/src/yolo_segmentation/yolo_segmentation/segmentation_node_floor.py
```

Если YOLO workspace лежит в другом месте, передай аргумент:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py yolo_ws:=/path/to/ros2_yolo_ws
```

То же самое работает для `bag_yolo_orangepi.launch.py`.

## Launch-файлы

### `hydra_ros camera_yolo_orangepi.launch.py`

Основной launch для живого запуска на роботе. Он поднимает:

- Intel RealSense с профилем `424x240x15`.
- IMU RealSense и `imu_filter_madgwick`.
- `rtabmap_odom/rgbd_odometry` с ожиданием инициализации по IMU.
- Статические TF `camera_link -> base_link_gt` и `map -> odom`.
- YOLO сегментацию из `~/ros2_yolo_ws`.
- `hydra.launch.yaml` после задержки, чтобы камера, одометрия, TF и YOLO успели
  стартовать.

Команда:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py
```

### `hydra_ros bag_yolo_orangepi.launch.py`

Launch для отладки на rosbag. Он запускает воспроизведение bag-файла,
одометрию, статические TF, YOLO и Hydra в той же схеме топиков, что и живой
запуск с камерой.

Путь к bag по умолчанию:

```text
proezd15_05V4
```

Пример:

```bash
ros2 launch hydra_ros bag_yolo_orangepi.launch.py bag_path:=/path/to/bag
```

### `hydra_ros hydra.launch.yaml`

Основной launch Hydra. Он запускает:

- узел Hydra;
- Hydra visualizer;
- локальный RViz на компьютере робота;
- два republisher-а для RViz-картинок с низкой частотой.

Основные выходные топики:

```text
/hydra_visualizer/graph
/hydra_visualizer/agent_poses
/rviz/camera/color/image_raw
/rviz/yolo_segmentation/result
```

Hydra продолжает использовать полный локальный поток:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/yolo_segmentation/labels
```

Только RViz-картинки ограничиваются до 1 Гц:

```text
/camera/camera/color/image_raw    -> /rviz/camera/color/image_raw
/yolo_segmentation/result         -> /rviz/yolo_segmentation/result
```

Полезные аргументы:

```text
start_visualizer:=true|false
start_rviz:=true|false
start_rviz_image_throttle:=true|false
rviz_image_rate_hz:=1.0
start_rviz_yolo_image_throttle:=true|false
rviz_yolo_image_rate_hz:=1.0
```

### `hydra_visualizer laptop_rviz.launch.yaml`

Легкий RViz launch для ноутбука. Он открывает конфиг:

```text
src/hydra_ros/hydra_visualizer/rviz/laptop_streaming_visualizer.rviz
```

Ноутбучный RViz подписан на:

```text
/hydra_visualizer/graph
/hydra_visualizer/agent_poses
/rviz/yolo_segmentation/result
```

Рекомендуемая команда на ноутбуке:

```bash
ros2 launch hydra_visualizer laptop_rviz.launch.yaml
```

### `hydra_visualizer streaming_visualizer.launch.yaml`

Полный локальный visualizer, который используется на компьютере робота. Он
сохраняет обычное поведение RViz, включая mesh и локальные image display.
Обычно его не нужно запускать напрямую: он подключается через `hydra.launch.yaml`.

## Структура проекта

```text
.
├── colcon_defaults.yaml
├── README.md
├── README.pdf
├── README_RU.md
├── README_RU.pdf
├── src/
│   ├── hydra/                       # Базовая библиотека Hydra и конфиги
│   ├── hydra_ros/
│   │   ├── hydra_ros/               # ROS wrapper, robot/bag launch, throttler
│   │   ├── hydra_visualizer/        # Visualizer, RViz-конфиги, laptop launch
│   │   └── hydra_msgs/              # Сообщения и сервисы Hydra
│   ├── kimera_pgmo/                 # Kimera-PGMO и ROS/RViz интеграция
│   ├── kimera_rpgo/                 # Оптимизация pose graph
│   ├── pose_graph_tools/            # Инструменты и сообщения pose graph
│   ├── spark_dsg/                   # Dynamic Scene Graph library
│   ├── spatial_hash/                # Spatial hashing dependency
│   ├── config_utilities/            # Утилиты конфигурации
│   ├── semantic_inference/          # Semantic inference пакеты
│   ├── realsense-ros/               # RealSense ROS пакеты
│   ├── realsense2_ros_mqtt_bridge/  # RealSense bridge
│   ├── security_system/             # Дополнительные camera/YOLO utilities
│   ├── kursant1_driver/             # Драйвер робота
│   ├── kursant1_task_processor/     # Обработка задач робота
│   ├── ianvs/                       # Дополнительная зависимость
│   └── teaser_plusplus/             # TEASER++ dependency
└── install/, build/, log/            # Генерируются colcon, не исходники
```

YOLO workspace лежит отдельно:

```text
~/ros2_yolo_ws
```

## Схема удаленной визуализации

Главная идея текущей схемы:

- Hydra, одометрия и построение графа работают с полным локальным потоком на
  компьютере робота.
- Ноутбук получает граф постоянно.
- Ноутбук получает только одну YOLO-картинку в секунду.

Так старый ноутбучный RViz не может случайно начать тянуть raw camera stream и
забить локальную сеть.

Проверка частот:

```bash
ros2 topic hz /hydra_visualizer/graph
ros2 topic hz /rviz/yolo_segmentation/result
ros2 topic hz /rviz/camera/color/image_raw
```

Топики `/rviz/...image...` должны быть около `1 Hz`.

## Что изменилось относительно init-коммита

Относительно первого снимка workspace (`21ad9eb`) сделано:

- Workspace переведен в monorepo: вложенные git-репозитории стали обычными
  директориями внутри `src/`.
- Добавлена корневая документация по сборке и запуску.
- Добавлен `camera_yolo_orangepi.launch.py` для живого запуска RealSense + IMU
  + RTAB-Map odometry + YOLO + Hydra.
- Добавлен `bag_yolo_orangepi.launch.py` для запуска той же цепочки на rosbag.
- Добавлен `throttled_image_republisher.py`, который публикует последнюю
  картинку с фиксированной низкой частотой для RViz.
- Добавлены `laptop_rviz.launch.yaml` и `laptop_streaming_visualizer.rviz` для
  удаленного RViz на ноутбуке.
- В `hydra.launch.yaml` добавлены 1 Гц RViz-топики:
  - `/rviz/camera/color/image_raw`
  - `/rviz/yolo_segmentation/result`
- Возвращена IMU-инициализация для camera odometry, потому что без нее могли
  пропадать свежие pose/TF и граф переставал обновляться.

## Диагностика

Если граф пустой в RViz, сначала проверить одометрию и TF:

```bash
ros2 topic echo /rtabmap/odom --once
ros2 run tf2_ros tf2_echo odom camera_link
ros2 topic hz /hydra_visualizer/graph
```

Если на ноутбуке нет картинки или она идет слишком часто:

```bash
ros2 topic hz /rviz/yolo_segmentation/result
ros2 topic info /rviz/yolo_segmentation/result --verbose
```

Если сборка на ноутбуке падает с отсутствием `GTSAMConfig.cmake`:

```bash
sudo apt install -y ros-jazzy-gtsam
```
