from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    bag_play_rate = LaunchConfiguration("bag_play_rate")
    semantic_start_delay = LaunchConfiguration("semantic_start_delay")
    hydra_start_delay = LaunchConfiguration("hydra_start_delay")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    model_name = LaunchConfiguration("model_name")
    model_config = LaunchConfiguration("model_config")
    labelspace_name = LaunchConfiguration("labelspace_name")
    force_rebuild = LaunchConfiguration("force_rebuild")
    max_image_queue_size = LaunchConfiguration("max_image_queue_size")
    min_separation_s = LaunchConfiguration("min_separation_s")
    rotation_type = LaunchConfiguration("rotation_type")
    show_configs = LaunchConfiguration("show_configs")

    semantic_label_grouping = PathJoinSubstitution([
        FindPackageShare("semantic_inference_ros"),
        "config",
        "label_groupings",
        PythonExpression(["'", labelspace_name, ".yaml@output/recolor'"]),
    ])
    semantic_colormap = PathJoinSubstitution([
        FindPackageShare("semantic_inference_ros"),
        "config",
        "distinct_150_colors.csv",
    ])

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
            "semantic_start_delay",
            default_value="2.0",
            description="Seconds to wait before starting semantic_inference.",
        ),
        DeclareLaunchArgument(
            "hydra_start_delay",
            default_value="6.0",
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
        DeclareLaunchArgument(
            "model_name",
            default_value="ade20k-efficientvit_seg_l2",
            description="semantic_inference closed-set model name without .onnx.",
        ),
        DeclareLaunchArgument(
            "model_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("semantic_inference_ros"),
                "config",
                "models",
                PythonExpression(["'", model_name, ".yaml'"]),
            ]),
            description="semantic_inference model config.",
        ),
        DeclareLaunchArgument(
            "labelspace_name",
            default_value="ade20k_full",
            description="Labelspace used by semantic_inference and Hydra.",
        ),
        DeclareLaunchArgument(
            "force_rebuild",
            default_value="false",
            description="Force TensorRT engine rebuild for semantic_inference.",
        ),
        DeclareLaunchArgument(
            "max_image_queue_size",
            default_value="1",
            description="Max queued images for semantic_inference.",
        ),
        DeclareLaunchArgument(
            "min_separation_s",
            default_value="0.0",
            description="Minimum separation between segmented images.",
        ),
        DeclareLaunchArgument(
            "rotation_type",
            default_value="none",
            description="Input camera rotation for semantic_inference.",
        ),
        DeclareLaunchArgument(
            "show_configs",
            default_value="true",
            description="Print semantic_inference configs.",
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
            period=semantic_start_delay,
            actions=[
                Node(
                    package="semantic_inference_ros",
                    executable="closed_set_node",
                    name="semantic_inference",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    remappings=[
                        ("color/image_raw", "/camera/camera/color/image_raw"),
                        ("semantic/image_raw", "/yolo_segmentation/labels"),
                        ("semantic_overlay/image_raw", "/yolo_segmentation/result"),
                        ("semantic_color/image_raw", "/semantic_color/image_raw"),
                    ],
                    arguments=[
                        "--config-utilities-file",
                        model_config,
                        "--config-utilities-file",
                        semantic_label_grouping,
                        "--config-utilities-yaml",
                        PythonExpression([
                            "'{segmenter: {model: {model_file: ",
                            model_name,
                            ".onnx, force_rebuild: ",
                            force_rebuild,
                            "}}}'",
                        ]),
                        "--config-utilities-yaml",
                        ["{output: {recolor: {colormap_path: ", semantic_colormap, "}}}"],
                        "--config-utilities-yaml",
                        PythonExpression([
                            "'{worker: {max_queue_size: ",
                            max_image_queue_size,
                            ", image_separation_s: ",
                            min_separation_s,
                            "}}'",
                        ]),
                        "--config-utilities-yaml",
                        PythonExpression([
                            "'{image_rotator: {rotation: ",
                            rotation_type,
                            "}}'",
                        ]),
                        "--config-utilities-yaml",
                        PythonExpression(["'{show_config: ", show_configs, "}'"]),
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
                        PythonExpression(["'labelspace:=' + '", labelspace_name, "'"]),
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
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
