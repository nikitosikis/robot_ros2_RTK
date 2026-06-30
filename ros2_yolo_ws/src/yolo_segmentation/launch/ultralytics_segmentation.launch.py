from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("input_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_info_topic",
            default_value="/camera/camera/aligned_depth_to_color/camera_info",
        ),
        DeclareLaunchArgument("result_topic", default_value="/yolo_segmentation/result"),
        DeclareLaunchArgument("label_topic", default_value="/yolo_segmentation/labels"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.2"),
        DeclareLaunchArgument("iou_threshold", default_value="0.35"),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("use_depth_floor", default_value="true"),
        DeclareLaunchArgument("h_floor_m", default_value="0.23"),
        DeclareLaunchArgument("h_tolerance", default_value="0.15"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="yolo_segmentation",
            executable="ultralytics_segmentation_node",
            name="ultralytics_segmentation_node",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "input_topic": LaunchConfiguration("input_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "depth_info_topic": LaunchConfiguration("depth_info_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "label_topic": LaunchConfiguration("label_topic"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "image_size": LaunchConfiguration("image_size"),
                "device": LaunchConfiguration("device"),
                "use_depth_floor": LaunchConfiguration("use_depth_floor"),
                "h_floor_m": LaunchConfiguration("h_floor_m"),
                "h_tolerance": LaunchConfiguration("h_tolerance"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
