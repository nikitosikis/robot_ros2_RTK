from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOv8 segmentation with ONNX Runtime on ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation_node = yolo_segmentation.segmentation_node:main',
            'ultralytics_segmentation_node = yolo_segmentation.ultralytics_segmentation_node:main',
        ],
    },
)
