from setuptools import find_packages, setup

package_name = 'security_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agent',
    maintainer_email='agent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = security_system.camera_node:main',
            'yolo_detection_node = security_system.yolo_detection_node:main',
            'camera_visualization = security_system.camera_visualization:main',
        ],
    },
)
