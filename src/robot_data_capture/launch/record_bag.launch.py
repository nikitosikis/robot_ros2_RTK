from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_output = LaunchConfiguration("bag_output")
    record_mode = LaunchConfiguration("record_mode")
    start_capture = LaunchConfiguration("start_capture")
    storage = LaunchConfiguration("storage")
    thermal_device = LaunchConfiguration("thermal_device")
    thermal_device_name = LaunchConfiguration("thermal_device_name")
    thermal_fps = LaunchConfiguration("thermal_fps")
    microphone_device = LaunchConfiguration("microphone_device")
    audio_sample_rate = LaunchConfiguration("audio_sample_rate")
    audio_channels = LaunchConfiguration("audio_channels")

    data_topics = [
        "/data_capture/camera/image_raw",
        "/data_capture/thermal/image_raw",
        "/thermal/camera_info",
        "/microphone/audio",
        "/camera/camera/color/camera_info",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/aligned_depth_to_color/camera_info",
        "/camera/camera/depth/camera_info",
        "/rviz/camera/color/image_raw",
        "/rviz/yolo_segmentation/result",
        "/rviz/thermal/image_raw",
        "/tf",
        "/tf_static",
    ]

    full_topics = [
        "/camera/camera/color/image_raw",
        "/camera/camera/color/camera_info",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/aligned_depth_to_color/camera_info",
        "/camera/camera/depth/camera_info",
        "/camera/camera/imu",
        "/rtabmap/imu",
        "/rtabmap/odom",
        "/yolo_segmentation/labels",
        "/yolo_segmentation/result",
        "/rviz/camera/color/image_raw",
        "/rviz/yolo_segmentation/result",
        "/rviz/thermal/image_raw",
        "/data_capture/camera/image_raw",
        "/data_capture/thermal/image_raw",
        "/thermal/camera_info",
        "/microphone/audio",
        "/hydra/backend/dsg",
        "/hydra_visualizer/graph",
        "/hydra_visualizer/agent_poses",
        "/hydra_visualizer/mesh",
        "/tf",
        "/tf_static",
    ]

    def launch_recorder(context):
        mode = record_mode.perform(context)
        output_path = bag_output.perform(context)
        if mode == "data":
            topics = data_topics
        elif mode == "full":
            topics = full_topics
        else:
            raise RuntimeError(
                f"Unsupported record_mode '{mode}'. Expected 'data' or 'full'."
            )

        if Path(output_path).exists():
            output_path = f"{output_path}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        return [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    output_path,
                    "-s",
                    storage,
                    *topics,
                ],
                output="screen",
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument("bag_output", default_value="robot_capture_bag"),
        DeclareLaunchArgument("record_mode", default_value="full"),
        DeclareLaunchArgument("start_capture", default_value="true"),
        DeclareLaunchArgument("storage", default_value="sqlite3"),
        DeclareLaunchArgument("thermal_device", default_value=""),
        DeclareLaunchArgument("thermal_device_name", default_value="USB2.0 PC CAMERA"),
        DeclareLaunchArgument("thermal_fps", default_value="15.0"),
        DeclareLaunchArgument("microphone_device", default_value="default"),
        DeclareLaunchArgument("audio_sample_rate", default_value="16000"),
        DeclareLaunchArgument("audio_channels", default_value="1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("robot_data_capture"),
                    "launch",
                    "capture.launch.py",
                ])
            ),
            condition=IfCondition(start_capture),
            launch_arguments={
                "thermal_device": thermal_device,
                "thermal_device_name": thermal_device_name,
                "thermal_fps": thermal_fps,
                "microphone_device": microphone_device,
                "audio_sample_rate": audio_sample_rate,
                "audio_channels": audio_channels,
            }.items(),
        ),
        OpaqueFunction(function=launch_recorder),
    ])
