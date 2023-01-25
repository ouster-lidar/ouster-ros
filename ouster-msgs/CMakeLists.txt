cmake_minimum_required(VERSION 3.5)

project(ouster_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PacketMsg.msg"
)

ament_package()
