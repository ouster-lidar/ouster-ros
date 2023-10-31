cmake_minimum_required(VERSION 3.5)

project(ouster_sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PacketMsg.msg"
  "srv/GetConfig.srv"
  "srv/SetConfig.srv"
  "srv/GetMetadata.srv"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
