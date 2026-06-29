# Методичка по проекту `robot_ros2_RTK`

Документ описывает установку окружения, сборку проекта и запуск только двух рабочих launch-файлов:

- `hydra_ros camera_yolo_orangepi.launch.py` - запуск с живой камерой RealSense;
- `hydra_ros bag_yolo_orangepi.launch.py` - запуск Hydra по записанному rosbag, где уже есть YOLO-топики.

Команды рассчитаны на Ubuntu 24.04 и ROS 2 Jazzy.

## 1. Назначение проекта

Проект запускает ROS 2 pipeline для робота:

```text
RealSense RGB-D + IMU
  -> imu_filter_madgwick
  -> rtabmap_odom
  -> YOLO segmentation
  -> Hydra
  -> RViz / hydra_visualizer
```

Дополнительно есть пакет `robot_data_capture`, который может писать bag-файлы и собирать данные с тепловизора и микрофона.

## 2. Структура проекта

Типовая структура после клонирования и сборки:

```text
/home/<username>/ros2_ws/
├── build/                         # Артефакты сборки colcon
├── install/                       # Установленные пакеты и setup.bash
├── log/                           # Логи сборки
├── 2026_0616_small_circle_backward_run/
│   └── ...                        # Пример rosbag для bag launch
└── robot_ros2_RTK/
    ├── README.md
    ├── README_RU.md
    ├── METODICHKA_LAUNCH.md
    ├── colcon_defaults.yaml
    ├── ros2_yolo_ws/
    │   └── src/yolo_segmentation/ # YOLO workspace для camera launch
    └── src/
        ├── hydra/                 # Базовая библиотека Hydra
        ├── hydra_ros/
        │   ├── hydra_ros/         # ROS wrapper и launch-файлы
        │   ├── hydra_visualizer/  # RViz/visualizer
        │   └── hydra_msgs/        # Сообщения Hydra
        ├── robot_data_capture/    # Тепловизор, микрофон, запись bag
        ├── realsense-ros/         # Драйвер Intel RealSense
        ├── semantic_inference/    # Семантическая сегментация, зависимости Hydra
        ├── kimera_pgmo/           # Оптимизация mesh/pose graph
        ├── kimera_rpgo/           # Robust pose graph optimization
        ├── pose_graph_tools/      # Pose graph сообщения и утилиты
        ├── spark_dsg/             # Dynamic Scene Graph
        ├── config_utilities/      # Утилиты конфигов
        ├── spatial_hash/          # Зависимость Hydra
        ├── kursant1_driver/
        ├── kursant1_task_processor/
        └── security_system/
```

В этой методичке используются только эти launch-файлы:

```text
robot_ros2_RTK/src/hydra_ros/hydra_ros/launch/
├── camera_yolo_orangepi.launch.py
└── bag_yolo_orangepi.launch.py
```

Файл `camera_yolo_orangepi.launch.py` внутри себя также подключает:

```text
robot_ros2_RTK/src/robot_data_capture/launch/
├── capture.launch.py
└── record_bag.launch.py
```

## 3. Установка ROS 2 Jazzy

Если ROS 2 Jazzy уже установлен, этот раздел можно пропустить.

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo apt update
```

Добавить репозиторий ROS 2:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

Установить ROS 2 и инструменты разработки:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Проверить:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

## 4. Системные зависимости

Установить базовые инструменты:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-venv \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  ninja-build \
  v4l-utils \
  alsa-utils
```

Установить библиотеки, которые нужны Hydra/Kimera и пакетам проекта:

```bash
sudo apt install -y \
  ros-jazzy-gtsam \
  nlohmann-json3-dev \
  pybind11-dev \
  libgoogle-glog-dev \
  libgtest-dev \
  libyaml-cpp-dev \
  libzmqpp-dev
```

Установить runtime ROS-пакеты, которые прямо используются в `camera` и `bag` launch:

```bash
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  ros-jazzy-imu-filter-madgwick \
  ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-message-filters \
  ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-transport \
  ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-rviz2
```

Назначение основных пакетов:

```text
ros-jazzy-realsense2-camera          # драйвер Intel RealSense, rs_launch.py
ros-jazzy-imu-filter-madgwick        # imu_filter_madgwick_node
ros-jazzy-rtabmap-odom               # rgbd_odometry
ros-jazzy-tf2-ros                    # static_transform_publisher, tf2_echo
ros-jazzy-cv-bridge                  # конвертация OpenCV <-> sensor_msgs/Image
ros-jazzy-message-filters            # синхронизация RGB и thermal image
ros-jazzy-rosbag2                    # ros2 bag play/record
ros-jazzy-rviz2                      # визуализация
v4l-utils                            # v4l2-ctl для поиска thermal camera
alsa-utils                           # arecord для записи микрофона
ros-jazzy-gtsam                      # зависимость Hydra/Kimera
```

Инициализировать `rosdep`, если он еще не был настроен:

```bash
sudo rosdep init
rosdep update
```

Если `sudo rosdep init` пишет, что файл уже существует, это нормально. Выполните только:

```bash
rosdep update
```

## 5. Клонирование проекта

Создать workspace и клонировать репозиторий:

```bash
mkdir -p /home/<username>/ros2_ws
cd /home/<username>/ros2_ws
git clone git@github.com:nikitosikis/robot_ros2_RTK.git
```

Если используется HTTPS:

```bash
git clone https://github.com/nikitosikis/robot_ros2_RTK.git
```

Репозиторий оформлен как monorepo: основные пакеты уже лежат внутри `robot_ros2_RTK/src`.

## 6. Установка зависимостей через rosdep

Из корня workspace:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths robot_ros2_RTK/src --ignore-src -r -y --rosdistro jazzy
```

Если нужен также YOLO workspace внутри репозитория:

```bash
rosdep install --from-paths robot_ros2_RTK/ros2_yolo_ws/src --ignore-src -r -y --rosdistro jazzy
```

## 7. YOLO workspace для `camera_yolo_orangepi`

`camera_yolo_orangepi.launch.py` запускает YOLO так:

```text
python3 src/yolo_segmentation/yolo_segmentation/segmentation_node_floor.py
```

Рабочий каталог задается аргументом `yolo_ws`.

По умолчанию:

```text
$HOME/ros2_yolo_ws
```

В этом репозитории YOLO workspace лежит здесь:

```text
/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws
```

Поэтому при запуске лучше передавать путь явно:

```bash
yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws
```

Важный момент: текущий `segmentation_node_floor.py` использует `rknn.api.RKNN` и модель для RK3588:

```text
yolov8n-seg-rk3588.rknn
```

Для работы на Orange Pi/RK3588 нужен установленный RKNN runtime/toolkit, совместимый с вашей платой. Минимально проверьте импорт:

```bash
cd /home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws
python3 -c "from rknn.api import RKNN; print('RKNN OK')"
```

Также нужны Python-библиотеки:

```bash
python3 -c "import cv2, numpy; print('OpenCV/NumPy OK')"
```

Если импорт `cv2` не работает:

```bash
sudo apt install -y python3-opencv python3-numpy
```

## 8. Сборка проекта

Собрать основной workspace:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths robot_ros2_RTK/src
source install/setup.bash
```

Если нужно собрать YOLO workspace как отдельный ROS workspace:

```bash
cd /home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

После сборки в новом терминале всегда выполнять:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Проверить, что пакеты доступны:

```bash
ros2 pkg prefix hydra_ros
ros2 pkg prefix robot_data_capture
ros2 pkg prefix realsense2_camera
```

Проверить аргументы launch-файлов:

```bash
ros2 launch hydra_ros --show-args camera_yolo_orangepi.launch.py
ros2 launch hydra_ros --show-args bag_yolo_orangepi.launch.py
```

## 9. Запуск `camera_yolo_orangepi.launch.py`

Этот launch используется для живого запуска с подключенной Intel RealSense.

Он запускает:

- `realsense2_camera/rs_launch.py`;
- `imu_filter_madgwick_node`;
- `rtabmap_odom/rgbd_odometry`;
- статические TF `camera_link -> base_link_gt` и `map -> odom`;
- YOLO `segmentation_node_floor.py` из `yolo_ws`;
- `robot_data_capture/capture.launch.py`;
- `hydra_ros/hydra.launch.yaml`;
- опционально `robot_data_capture/record_bag.launch.py`.

Базовый запуск:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws
```

Запуск без тепловизора и микрофона:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws \
  start_data_capture:=false
```

Запуск с записью полного bag:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws \
  record_full_bag:=true \
  full_bag_output:=robot_full_bag
```

Запуск с записью только data bag:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws \
  record_data_bag:=true \
  data_bag_output:=robot_capture_bag
```

Нельзя одновременно включать:

```text
record_full_bag:=true
record_data_bag:=true
```

### Аргументы `camera_yolo_orangepi.launch.py`

```text
yolo_ws:=...                       # Путь к YOLO workspace
start_data_capture:=true           # Запуск thermal camera, microphone, sync
record_full_bag:=false             # Запись полного bag
full_bag_output:=robot_full_bag
record_data_bag:=false             # Запись data bag
data_bag_output:=robot_capture_bag
bag_storage:=sqlite3               # Storage для ros2 bag
thermal_device:=...                # Например /dev/video2
thermal_device_name:=USB2.0 PC CAMERA
microphone_device:=pipewire
audio_sample_rate:=48000
audio_channels:=1
audio_gain:=1.0
```

### Основные топики в camera launch

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/imu
/rtabmap/imu
/rtabmap/odom
/yolo_segmentation/labels
/yolo_segmentation/result
/data_capture/thermal/image_raw
/thermal/camera_info
/microphone/audio
/hydra_visualizer/graph
/hydra_visualizer/agent_poses
/hydra_visualizer/mesh
/tf
/tf_static
```

## 10. Запуск `bag_yolo_orangepi.launch.py`

Этот launch используется для воспроизведения rosbag. Он подходит для bag-файлов, где уже записаны YOLO-топики:

```text
/yolo_segmentation/labels
/yolo_segmentation/result
```

Launch не запускает YOLO заново. Он берет YOLO-результаты из bag.

Он запускает:

- `imu_filter_madgwick_node`;
- `rtabmap_odom/rgbd_odometry`;
- статические TF `camera_link -> base_link_gt` и `map -> odom`;
- `hydra_ros/hydra.launch.yaml`;
- `ros2 bag play --clock`.

Базовый запуск с bag по умолчанию:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hydra_ros bag_yolo_orangepi.launch.py
```

Запуск с явным путем к bag:

```bash
ros2 launch hydra_ros bag_yolo_orangepi.launch.py \
  bag_path:=/home/<username>/ros2_ws/2026_0616_small_circle_backward_run
```

Запуск медленнее реального времени:

```bash
ros2 launch hydra_ros bag_yolo_orangepi.launch.py \
  bag_path:=/home/<username>/ros2_ws/2026_0616_small_circle_backward_run \
  bag_play_rate:=0.15
```

Запуск без RViz:

```bash
ros2 launch hydra_ros bag_yolo_orangepi.launch.py \
  bag_path:=/home/<username>/ros2_ws/2026_0616_small_circle_backward_run \
  start_rviz:=false
```

### Аргументы `bag_yolo_orangepi.launch.py`

```text
bag_path:=2026_0616_small_circle_backward_run
bag_start_delay:=35.0
bag_play_rate:=0.15
hydra_start_delay:=2.0
use_sim_time:=true
start_rviz:=true
```

`use_sim_time:=true` нужен, потому что bag запускается с `--clock`.

### Топики, которые воспроизводятся из bag

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/imu
/yolo_segmentation/labels
/yolo_segmentation/result
/data_capture/thermal/image_raw
/thermal/camera_info
/tf_static
```

## 11. Проверка работы

Список узлов:

```bash
ros2 node list
```

Список топиков:

```bash
ros2 topic list
```

Проверить RGB:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Проверить depth:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

Проверить IMU после фильтра Madgwick:

```bash
ros2 topic echo /rtabmap/imu --once
```

Проверить одометрию:

```bash
ros2 topic echo /rtabmap/odom --once
```

Проверить YOLO:

```bash
ros2 topic hz /yolo_segmentation/labels
ros2 topic hz /yolo_segmentation/result
```

Проверить Hydra:

```bash
ros2 topic hz /hydra_visualizer/graph
ros2 topic hz /hydra_visualizer/agent_poses
```

Проверить TF:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo camera_link base_link_gt
```

Проверить содержимое bag:

```bash
ros2 bag info /home/<username>/ros2_ws/2026_0616_small_circle_backward_run
```

## 12. Частые проблемы

### `Package 'hydra_ros' not found`

Не выполнен `source install/setup.bash` или проект не собран:

```bash
cd /home/<username>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths robot_ros2_RTK/src
source install/setup.bash
```

### `Package 'imu_filter_madgwick' not found`

Установить пакет:

```bash
sudo apt install -y ros-jazzy-imu-filter-madgwick
```

### `Package 'rtabmap_odom' not found`

Установить RTAB-Map пакеты:

```bash
sudo apt install -y ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-ros
```

### `Package 'realsense2_camera' not found`

Установить RealSense ROS:

```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```

### Тепловизор не находится

Проверить устройства:

```bash
v4l2-ctl --list-devices
```

Передать устройство явно:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws \
  thermal_device:=/dev/video2
```

### Микрофон не пишется

Проверить ALSA-устройства:

```bash
arecord -L
```

Передать устройство явно:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  yolo_ws:=/home/<username>/ros2_ws/robot_ros2_RTK/ros2_yolo_ws \
  microphone_device:=default
```

### В bag-сценарии нет времени

Проверьте, что используется именно `bag_yolo_orangepi.launch.py`. В нем bag запускается с `--clock`, а `use_sim_time` по умолчанию равен `true`.

### В bag-сценарии нет YOLO

`bag_yolo_orangepi.launch.py` не строит YOLO заново. В bag должны быть топики:

```text
/yolo_segmentation/labels
/yolo_segmentation/result
```

Проверить:

```bash
ros2 bag info /path/to/bag
```

### Hydra стартует раньше данных

Увеличить задержку запуска bag:

```bash
ros2 launch hydra_ros bag_yolo_orangepi.launch.py \
  bag_path:=/home/<username>/ros2_ws/2026_0616_small_circle_backward_run \
  bag_start_delay:=45.0
```

## 13. Минимальная последовательность команд с нуля

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-vcstool ros-dev-tools
sudo apt install -y ros-jazzy-imu-filter-madgwick
sudo apt install -y ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-ros
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-gtsam \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  ros-jazzy-imu-filter-madgwick \
  ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-transport \
  ros-jazzy-rosbag2-storage-default-plugins \
  v4l-utils \
  alsa-utils \
  nlohmann-json3-dev \
  pybind11-dev \
  libgoogle-glog-dev \
  libgtest-dev \
  libyaml-cpp-dev \
  libzmqpp-dev

mkdir -p /home/<username>/ros2_ws
cd /home/<username>/ros2_ws
git clone git@github.com:nikitosikis/robot_ros2_RTK.git

source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths robot_ros2_RTK/src --ignore-src -r -y --rosdistro jazzy

colcon build --symlink-install --base-paths robot_ros2_RTK/src
source install/setup.bash

ros2 launch hydra_ros bag_yolo_orangepi.launch.py \
  bag_path:=/home/<username>/ros2_ws/2026_0616_small_circle_backward_run
```
