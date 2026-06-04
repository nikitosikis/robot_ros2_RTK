from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Получаем путь к директории пакета
    #pkg_dir = get_package_share_directory('my_package')
    
    # Путь к YAML файлу с параметрами
    #params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Первая нода
    node1 = Node(
        package='kursant1_driver',
        executable='driver',
        name='driver',
        parameters=[
            # Параметры напрямую
            {
                'device_name': '/dev/ttyUSB0',
                'linear_multiplier': 100,
                'angular_multiplier': 50,
                'debug_mode': False # True 
            },
        ],
        remappings=[
        ],
        output='screen'  # Вывод в консоль
    )
    
    node2 = Node(
        package='kursant1_task_processor',
        executable='task_processor',
        name='task_processor',
        parameters=[
            # Параметры напрямую
            #{
            #    'device_name': '/dev/ttyUSB0',
            #    'linear_multiplier': 120,
            #    'angular_multiplier': 50,
            #    'debug_mode': False
            #},
        ],
        remappings=[
        ],
        output='screen'  # Вывод в консоль
    )
    
    return LaunchDescription([
        node1,
        node2
    ])
