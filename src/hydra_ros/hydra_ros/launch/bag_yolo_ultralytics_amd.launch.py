from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    bag_play_rate = LaunchConfiguration("bag_play_rate")
    yolo_start_delay = LaunchConfiguration("yolo_start_delay")
    hydra_start_delay = LaunchConfiguration("hydra_start_delay")
    yolo_setup = LaunchConfiguration("yolo_setup")
    yolo_venv = LaunchConfiguration("yolo_venv")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    model_path = LaunchConfiguration("model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    yolo_iou = LaunchConfiguration("yolo_iou")
    yolo_image_size = LaunchConfiguration("yolo_image_size")
    use_depth_floor = LaunchConfiguration("use_depth_floor")
    h_floor_m = LaunchConfiguration("h_floor_m")
    h_tolerance = LaunchConfiguration("h_tolerance")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="2026_0616_small_circle_backward_run",
            description="Path to the input bag with camera, depth, and IMU topics.",
        ),
        DeclareLaunchArgument(
            "bag_start_delay",
            default_value="5.0",
            description="Seconds to wait before starting bag playback.",
        ),
        DeclareLaunchArgument(
            "bag_play_rate",
            default_value="1.0",
            description="Playback rate for ros2 bag play.",
        ),
        DeclareLaunchArgument(
            "yolo_start_delay",
            default_value="2.0",
            description="Seconds to wait before starting Ultralytics YOLO.",
        ),
        DeclareLaunchArgument(
            "hydra_start_delay",
            default_value="6.0",
            description="Seconds to wait before starting Hydra.",
        ),
        DeclareLaunchArgument(
            "yolo_setup",
            default_value="/home/nick/ros2_ws/robot_ros2_RTK/ros2_yolo_ws/install/setup.bash",
            description="Setup file for the workspace that provides yolo_segmentation.",
        ),
        DeclareLaunchArgument(
            "yolo_venv",
            default_value="/home/nick/ros2_ws/robot_ros2_RTK/.venv-ultralytics",
            description="Python venv containing Ultralytics and a ROS-compatible NumPy.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/nick/ros2_ws/robot_ros2_RTK/ros2_yolo_ws/src/yolo_segmentation/models/yolov8n-oiv7.pt",
            description="Path to Ultralytics segmentation model.",
        ),
        DeclareLaunchArgument(
            "yolo_device",
            default_value="cpu",
            description="Ultralytics device, e.g. cpu, cuda:0, 0.",
        ),
        DeclareLaunchArgument("yolo_confidence", default_value="0.2"),
        DeclareLaunchArgument("yolo_iou", default_value="0.35"),
        DeclareLaunchArgument("yolo_image_size", default_value="640"),
        DeclareLaunchArgument("use_depth_floor", default_value="true"),
        DeclareLaunchArgument("h_floor_m", default_value="0.23"),
        DeclareLaunchArgument("h_tolerance", default_value="0.15"),

        TimerAction(
            period=0.0,
            actions=[
                Node(
                    package="imu_filter_madgwick",
                    executable="imu_filter_madgwick_node",
                    name="imu_filter_madgwick",
                    output="screen",
                    parameters=[{
                        "use_mag": False,
                        "publish_tf": False,
                        "world_frame": "enu",
                        "use_sim_time": use_sim_time,
                    }],
                    remappings=[
                        ("imu/data_raw", "/camera/camera/imu"),
                        ("imu/data", "/rtabmap/imu"),
                    ],
                ),
            ],
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry",
                    output="screen",
                    parameters=[{
                        "frame_id": "camera_link",
                        "odom_frame_id": "odom",
                        "publish_tf": True,
                        "wait_imu_to_init": True,
                        "approx_sync": False,
                        "approx_sync_max_interval": 0.1,
                        "sync_queue_size": 5,
                        "Vis/MinInliers": "8",
                        "Vis/MaxFeatures": "1000",
                        "Kp/MaxFeatures": "1000",
                        "GFTT/MinDistance": "7",
                        "GFTT/QualityLevel": "0.0001",
                        "OdomF2M/MaxSize": "800",
                        "OdomF2M/BundleAdjustment": "false",
                        "use_sim_time": use_sim_time,
                    }],
                    remappings=[
                        ("/rgb/image", "/camera/camera/color/image_raw"),
                        ("/depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                        ("/rgb/camera_info", "/camera/camera/color/camera_info"),
                        ("/imu", "/rtabmap/imu"),
                    ],
                ),
            ],
        ),

        TimerAction(
            period=0.5,
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_camera_link_to_base_link_gt",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "--x", "0",
                        "--y", "0",
                        "--z", "0",
                        "--yaw", "0",
                        "--pitch", "0",
                        "--roll", "0",
                        "--frame-id", "camera_link",
                        "--child-frame-id", "base_link_gt",
                    ],
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_map_to_odom",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "--x", "0",
                        "--y", "0",
                        "--z", "0",
                        "--yaw", "0",
                        "--pitch", "0",
                        "--roll", "0",
                        "--frame-id", "map",
                        "--child-frame-id", "odom",
                    ],
                ),
            ],
        ),

        TimerAction(
            period=yolo_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        [
                            "source ",
                            yolo_setup,
                            " && source ",
                            yolo_venv,
                            "/bin/activate",
                            " && export PYTHONNOUSERSITE=1",
                            " && exec python -m yolo_segmentation.ultralytics_segmentation_node",
                            " --ros-args",
                            " -r __node:=ultralytics_segmentation_node",
                            " -p model_path:=",
                            model_path,
                            " -p input_topic:=/camera/camera/color/image_raw",
                            " -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw",
                            " -p depth_info_topic:=/camera/camera/aligned_depth_to_color/camera_info",
                            " -p result_topic:=/yolo_segmentation/result",
                            " -p label_topic:=/yolo_segmentation/labels",
                            " -p confidence_threshold:=",
                            yolo_confidence,
                            " -p iou_threshold:=",
                            yolo_iou,
                            " -p image_size:=",
                            yolo_image_size,
                            " -p device:=",
                            yolo_device,
                            " -p use_depth_floor:=",
                            use_depth_floor,
                            " -p h_floor_m:=",
                            h_floor_m,
                            " -p h_tolerance:=",
                            h_tolerance,
                            " -p use_sim_time:=",
                            use_sim_time,
                        ],
                    ],
                    output="screen",
                ),
            ],
        ),

        TimerAction(
            period=hydra_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "launch",
                        "hydra_ros",
                        "hydra.launch.yaml",
                        "dataset:=uhumans2",
                        "labelspace:=ade20k_full",
                        "map_frame:=map",
                        "odom_frame:=odom",
                        "sensor_frame:=camera_color_optical_frame",
                        "robot_frame:=camera_link",
                        "enable_zmq:=false",
                        "start_rviz_image_throttle:=false",
                        "start_rviz_yolo_image_throttle:=false",
                        "start_rviz_thermal_image_throttle:=false",
                        PythonExpression(["'start_rviz:=' + '", start_rviz, "'"]),
                        PythonExpression(["'use_sim_time:=' + '", use_sim_time, "'"]),
                    ],
                    output="screen",
                ),
            ],
        ),

        TimerAction(
            period=bag_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "bag",
                        "play",
                        bag_path,
                        "--clock",
                        "--rate",
                        bag_play_rate,
                        "--disable-keyboard-controls",
                        "--read-ahead-queue-size",
                        "2000",
                        "--exclude-topics",
                        "/clock",
                        "/yolo_segmentation/labels",
                        "/yolo_segmentation/result",
                        "/rviz/yolo_segmentation/result",
                        "/hydra/backend/dsg",
                        "/hydra/backend/dsg_mesh",
                        "/hydra/backend/pose_graph",
                        "/hydra/backend/mesh_graph",
                        "/hydra/backend/deformation_graph_mesh_mesh",
                        "/hydra/backend/deformation_graph_pose_mesh",
                        "/hydra/frontend/mesh_graph_incremental",
                        "/hydra/frontend/full_mesh_update",
                        "/hydra/active_window/mesh",
                        "/hydra/active_window/pose",
                        "/hydra_visualizer/graph",
                        "/hydra_visualizer/agent_poses",
                        "/hydra_visualizer/mesh",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
