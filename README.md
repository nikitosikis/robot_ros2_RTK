# robot_ros2_RTK

ROS 2 Jazzy workspace for Hydra/RTK robot experiments.

## First build on a new machine

Install ROS dependencies before running `colcon build`:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-vcstool ros-dev-tools
sudo apt install -y ros-jazzy-gtsam nlohmann-json3-dev pybind11-dev
sudo apt install -y libgoogle-glog-dev libgtest-dev libyaml-cpp-dev libzmqpp-dev
```

If `rosdep` is not initialized yet:

```bash
sudo rosdep init
rosdep update
```

Then install package dependencies and build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
```

The `kimera_rpgo`, `kimera_pgmo`, and `hydra` packages require GTSAM. On
Ubuntu 24.04 with ROS Jazzy this is provided by `ros-jazzy-gtsam`.

## Laptop RViz

For remote visualization on a laptop, use the lightweight RViz launch:

```bash
source install/setup.bash
ros2 launch hydra_visualizer laptop_rviz.launch.yaml
```

It subscribes to:

```text
/hydra_visualizer/graph
/hydra_visualizer/agent_poses
/rviz/camera/color/image_raw
```

The camera image topic above is throttled to 1 Hz by `hydra.launch.yaml`; Hydra
continues to process the full-rate camera stream locally.
