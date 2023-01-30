cmake_minimum_required(VERSION 3.5)

project(ouster_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetConfig.srv"
  "srv/SetConfig.srv"
  "srv/GetMetadata.srv")

ament_package()
