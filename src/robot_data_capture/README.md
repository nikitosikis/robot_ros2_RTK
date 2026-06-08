# robot_data_capture

ROS 2 package for recording extra robot sensors into bag files.

Published topics:

```text
/thermal/image_raw
/thermal/camera_info
/microphone/audio
/data_capture/camera/image_raw
/data_capture/thermal/image_raw
```

The thermal camera is read as MJPG `640x480@15` and published as
`sensor_msgs/Image`. By default the node resolves the camera by the V4L2 device
name `USB2.0 PC CAMERA`, so it does not depend on whether the device becomes
`/dev/video0`, `/dev/video2`, etc. The microphone is streamed through `arecord`
and published as `robot_data_capture/msg/AudioFrame`.

`/data_capture/camera/image_raw` and `/data_capture/thermal/image_raw` are
approximate-synchronized pairs. Each pair is republished with a shared
`header.stamp` for bag analysis.

Start only capture nodes:

```bash
ros2 launch robot_data_capture capture.launch.py
```

If a stable udev path exists, pass it explicitly:

```bash
ros2 launch robot_data_capture capture.launch.py \
  thermal_device:=/dev/v4l/by-id/YOUR_THERMAL_CAMERA_ID
```

Otherwise the default auto-resolver uses:

```bash
v4l2-ctl --list-devices
```

and selects the first `/dev/video*` under:

```text
USB2.0 PC CAMERA
```

Start capture nodes and record a bag:

```bash
ros2 launch robot_data_capture record_bag.launch.py bag_output:=robot_capture_bag
```

Use `record_mode:=data` for a compact sensor-test bag:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  record_data_bag:=true \
  data_bag_output:=robot_capture_bag
```

Use `record_mode:=full` or the main launch flag for one common experiment bag:

```bash
ros2 launch hydra_ros camera_yolo_orangepi.launch.py \
  record_full_bag:=true \
  full_bag_output:=robot_full_bag
```

The `data` bag records synchronized RealSense/thermal image pairs, microphone
audio, full-rate RealSense depth, camera info, the 1 Hz RViz image topics,
including `/rviz/thermal/image_raw`, `/tf`, and `/tf_static`.

The `full` bag records the data bag topics plus raw color, IMU, odometry,
YOLO labels/results, Hydra DSG, RViz graph, poses, and mesh.
