from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='security_system',
            executable='camera_node',
        ),
        Node(
            package='security_system',
            executable='camera_visualization',
        ),
        Node(
            package='security_system',
            executable='yolo_detection_node'
        ),
#        Node(
#            package='rviz2',
#            executable='rviz2',
#            output='screen',
#            parameters=[
#                    {'use_sim_time': True}
#            ],
#            arguments=['-d', '/home/ubuntu24/ros2_ws/src/security_system/security_system/rviz2/rviz2.rviz']
#        ),
    ])
