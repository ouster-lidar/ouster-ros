cmake_minimum_required(VERSION 3.10.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../ouster-sdk/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ouster_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(tf2_eigen REQUIRED)
find_package(CURL REQUIRED)
find_package(Boost REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# for gdb add_compile_options(-Wall -Wextra -g -O0)
add_compile_options(-Wall -Wextra)
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ==== Catkin ====
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetConfig.srv"
  "srv/SetConfig.srv"
  "srv/GetMetadata.srv")

set(_ouster_ros_INCLUDE_DIRS
  "include;../ouster-sdk/ouster_client/include;../ouster-sdk/ouster_client/include/optional-lite")

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

option(BUILD_VIZ "Enabled for Python build" OFF)
option(BUILD_PCAP "Enabled for Python build" OFF)
find_package(OusterSDK REQUIRED)

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_ros_library src/os_ros.cpp)
target_link_libraries(ouster_ros_library
  # PUBLIC
    ouster_build
    pcl_common
  # PRIVATE
  -Wl,--whole-archive ouster_client -Wl,--no-whole-archive)
ament_target_dependencies(ouster_ros_library "rclcpp" "sensor_msgs" "geometry_msgs" "ouster_msgs")

# ==== os_sensor_compnent ====
add_library(os_sensor_component SHARED
  src/os_client_base_node.cpp
  src/os_sensor_node.cpp
)
target_compile_definitions(os_sensor_component PRIVATE
  "OUSTER_ROS_BUILDING_DLL")
ament_target_dependencies(os_sensor_component
  "rclcpp"
  "class_loader"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(os_sensor_component ouster_ros_library "${cpp_typesupport_target}")
rclcpp_components_register_nodes(os_sensor_component "nodelets_os::OusterSensor")

# ==== os_cloud_compnent ====
add_library(os_cloud_component SHARED
  src/os_cloud_node.cpp
)
target_compile_definitions(os_cloud_component PRIVATE
  "OUSTER_ROS_BUILDING_DLL")
ament_target_dependencies(os_cloud_component
  "rclcpp"
  "class_loader"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(os_cloud_component ouster_ros_library "${cpp_typesupport_target}")
rclcpp_components_register_nodes(os_cloud_component "nodelets_os::OusterCloud")

# ==== os_cloud_compnent ====
add_library(os_image_component SHARED
  src/os_image_node.cpp
)
target_compile_definitions(os_image_component PRIVATE
  "OUSTER_ROS_BUILDING_DLL")
ament_target_dependencies(os_image_component
  "rclcpp"
  "class_loader"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(os_image_component ouster_ros_library "${cpp_typesupport_target}")
rclcpp_components_register_nodes(os_image_component "nodelets_os::OusterImage")

# ==== Install ====
install(
  TARGETS
    ouster_ros_library
    os_sensor_component
    os_cloud_component
    os_image_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    ${_ouster_ros_INCLUDE_DIRS}
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  FILES
    ../LICENSE
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()