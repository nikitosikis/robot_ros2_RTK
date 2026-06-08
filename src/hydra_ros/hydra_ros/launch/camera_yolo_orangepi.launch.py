from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    yolo_ws = LaunchConfiguration('yolo_ws')
    start_data_capture = LaunchConfiguration('start_data_capture')
    record_full_bag = LaunchConfiguration('record_full_bag')
    full_bag_output = LaunchConfiguration('full_bag_output')
    record_data_bag = LaunchConfiguration('record_data_bag')
    data_bag_output = LaunchConfiguration('data_bag_output')
    bag_storage = LaunchConfiguration('bag_storage')
    thermal_device = LaunchConfiguration('thermal_device')
    thermal_device_name = LaunchConfiguration('thermal_device_name')
    microphone_device = LaunchConfiguration('microphone_device')

    def launch_bag_recorder(context):
        full_enabled = record_full_bag.perform(context).lower() == 'true'
        data_enabled = record_data_bag.perform(context).lower() == 'true'
        if full_enabled and data_enabled:
            raise RuntimeError(
                'record_full_bag and record_data_bag cannot both be true'
            )
        if not full_enabled and not data_enabled:
            return []

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('robot_data_capture'),
                        'launch',
                        'record_bag.launch.py'
                    ])
                ),
                launch_arguments={
                    'bag_output': full_bag_output if full_enabled else data_bag_output,
                    'record_mode': 'full' if full_enabled else 'data',
                    'start_capture': 'false',
                    'storage': bag_storage,
                    'thermal_device': thermal_device,
                    'thermal_device_name': thermal_device_name,
                    'microphone_device': microphone_device,
                }.items(),
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'yolo_ws',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'ros2_yolo_ws'
            ])
        ),
        DeclareLaunchArgument('start_data_capture', default_value='true'),
        DeclareLaunchArgument('record_full_bag', default_value='false'),
        DeclareLaunchArgument('full_bag_output', default_value='robot_full_bag'),
        DeclareLaunchArgument('record_data_bag', default_value='false'),
        DeclareLaunchArgument('data_bag_output', default_value='robot_capture_bag'),
        DeclareLaunchArgument('bag_storage', default_value='sqlite3'),
        DeclareLaunchArgument('thermal_device', default_value=''),
        DeclareLaunchArgument('thermal_device_name', default_value='USB2.0 PC CAMERA'),
        DeclareLaunchArgument('microphone_device', default_value='default'),

        ExecuteProcess(
            cmd=[
                'ros2', 'launch',
                'realsense2_camera',
                'rs_launch.py',
                'depth_module.depth_profile:=424x240x15',
                'rgb_camera.color_profile:=424x240x15',
                'depth_max:=4.0',
                'clip_distance:=4.0',
                'spatial_filter.enable:=true',
                'pointcloud.enable:=false',
                'align_depth.enable:=true',
                'enable_gyro:=true',
                'enable_accel:=true',
                'gyro_fps:=200',
                'accel_fps:=100',
                'unite_imu_method:=1'
            ],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter_madgwick',
                    output='screen',
                    parameters=[
                        {
                            'use_mag': False,
                            'publish_tf': False,
                            'world_frame': 'enu'
                        }
                    ],
                    remappings=[
                        ('imu/data_raw', '/camera/camera/imu'),
                        ('imu/data', '/rtabmap/imu')
                    ]
                )
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    output='screen',
                    parameters=[
                        {
                            'frame_id': 'camera_link',
                            'odom_frame_id': 'odom',
                            'publish_tf': True,
                            'wait_imu_to_init': True,
                            'approx_sync': True,
                            'approx_sync_max_interval': 0.1,
                            'sync_queue_size': 5,
                            'Vis/MinInliers': '10',
                            'Vis/MaxFeatures': '1000'
                        }
                    ],
                    remappings=[
                        ('/rgb/image', '/camera/camera/color/image_raw'),
                        ('/depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                        ('/rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('/imu', '/rtabmap/imu')
                    ]
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_camera_link_to_base_link_gt',
                    output='screen',
                    arguments=[
                        '--x', '0',
                        '--y', '0',
                        '--z', '0',
                        '--yaw', '0',
                        '--pitch', '0',
                        '--roll', '0',
                        '--frame-id', 'camera_link',
                        '--child-frame-id', 'base_link_gt'
                    ]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_map_to_odom',
                    output='screen',
                    arguments=[
                        '--x', '0',
                        '--y', '0',
                        '--z', '0',
                        '--yaw', '0',
                        '--pitch', '0',
                        '--roll', '0',
                        '--frame-id', 'map',
                        '--child-frame-id', 'odom'
                    ]
                )
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'python3',
                        'src/yolo_segmentation/yolo_segmentation/segmentation_node_floor.py'
                    ],
                    cwd=yolo_ws,
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('robot_data_capture'),
                            'launch',
                            'capture.launch.py'
                        ])
                    ),
                    condition=IfCondition(start_data_capture),
                    launch_arguments={
                        'thermal_device': thermal_device,
                        'thermal_device_name': thermal_device_name,
                        'microphone_device': microphone_device,
                    }.items(),
                )
            ]
        ),

        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch',
                        'hydra_ros',
                        'hydra.launch.yaml',
                        'dataset:=uhumans2',
                        'labelspace:=ade20k_full',
                        'map_frame:=map',
                        'odom_frame:=odom',
                        'sensor_frame:=camera_color_optical_frame',
                        'robot_frame:=camera_link',
                        'enable_zmq:=false'
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=30.0,
            actions=[
                OpaqueFunction(function=launch_bag_recorder)
            ]
        )
    ])
