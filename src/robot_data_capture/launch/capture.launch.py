from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    thermal_device = LaunchConfiguration("thermal_device")
    thermal_device_name = LaunchConfiguration("thermal_device_name")
    thermal_fps = LaunchConfiguration("thermal_fps")
    microphone_device = LaunchConfiguration("microphone_device")
    audio_sample_rate = LaunchConfiguration("audio_sample_rate")
    audio_channels = LaunchConfiguration("audio_channels")

    return LaunchDescription([
        DeclareLaunchArgument("thermal_device", default_value=""),
        DeclareLaunchArgument("thermal_device_name", default_value="USB2.0 PC CAMERA"),
        DeclareLaunchArgument("thermal_fps", default_value="15.0"),
        DeclareLaunchArgument("microphone_device", default_value="default"),
        DeclareLaunchArgument("audio_sample_rate", default_value="16000"),
        DeclareLaunchArgument("audio_channels", default_value="1"),
        Node(
            package="robot_data_capture",
            executable="thermal_camera_node.py",
            name="thermal_camera",
            output="screen",
            parameters=[{
                "device": thermal_device,
                "device_name": thermal_device_name,
                "width": 640,
                "height": 480,
                "fps": thermal_fps,
                "frame_id": "thermal_camera",
                "image_topic": "/thermal/image_raw",
                "camera_info_topic": "/thermal/camera_info",
                "encoding": "bgr8",
            }],
        ),
        Node(
            package="robot_data_capture",
            executable="microphone_node.py",
            name="microphone",
            output="screen",
            parameters=[{
                "device": microphone_device,
                "topic": "/microphone/audio",
                "frame_id": "microphone",
                "sample_rate": audio_sample_rate,
                "channels": audio_channels,
                "sample_format": "S16_LE",
                "encoding": "pcm_s16le",
                "chunk_ms": 100,
            }],
        ),
        Node(
            package="robot_data_capture",
            executable="image_pair_sync_node.py",
            name="image_pair_sync",
            output="screen",
            parameters=[{
                "camera_topic": "/camera/camera/color/image_raw",
                "thermal_topic": "/thermal/image_raw",
                "synced_camera_topic": "/data_capture/camera/image_raw",
                "synced_thermal_topic": "/data_capture/thermal/image_raw",
                "queue_size": 15,
                "slop_sec": 0.08,
            }],
        ),
    ])
