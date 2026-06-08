cmake_minimum_required(VERSION 3.8)
project(robot_data_capture)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioFrame.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

install(PROGRAMS
  robot_data_capture/image_pair_sync_node.py
  robot_data_capture/microphone_node.py
  robot_data_capture/thermal_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
