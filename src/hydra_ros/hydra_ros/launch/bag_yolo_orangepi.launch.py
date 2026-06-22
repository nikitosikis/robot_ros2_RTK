from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    bag_play_rate = LaunchConfiguration("bag_play_rate")
    hydra_start_delay = LaunchConfiguration("hydra_start_delay")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="2026_0616_small_circle_backward_run",
            description="Path to the input bag with camera, depth, and IMU topics.",
        ),
        DeclareLaunchArgument(
            "bag_start_delay",
            default_value="35.0",
            description="Seconds to wait before starting bag playback so Hydra is fully ready.",
        ),
        DeclareLaunchArgument(
            "bag_play_rate",
            default_value="0.15",
            description="Playback rate for the bag. Lower values give odometry and Hydra more time.",
        ),
        DeclareLaunchArgument(
            "hydra_start_delay",
            default_value="2.0",
            description="Seconds to wait before starting Hydra.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from ros2 bag play.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz from hydra.launch.yaml.",
        ),
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
                        "--topics",
                        "/camera/camera/color/image_raw",
                        "/camera/camera/color/camera_info",
                        "/camera/camera/aligned_depth_to_color/image_raw",
                        "/camera/camera/aligned_depth_to_color/camera_info",
                        "/camera/camera/imu",
                        "/yolo_segmentation/labels",
                        "/yolo_segmentation/result",
                        "/data_capture/thermal/image_raw",
                        "/thermal/camera_info",
                        "/tf_static",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
