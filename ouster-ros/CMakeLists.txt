cmake_minimum_required(VERSION 3.10.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ouster_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

# ==== Options ====
add_compile_options(-Wall -Wextra)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

set(_ouster_ros_INCLUDE_DIRS
  include
  ouster-sdk/ouster_client/include
  ouster-sdk/ouster_client/include/optional-lite
)

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

option(BUILD_VIZ "Turn off building VIZ" OFF)
option(BUILD_PCAP "Turn off building PCAP" OFF)
find_package(OusterSDK REQUIRED)

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_ros_library SHARED
  src/os_ros.cpp
)

set(ouster_ros_library_deps
  rclcpp
  sensor_msgs
  geometry_msgs
  ouster_sensor_msgs
  pcl_conversions
  tf2
  tf2_eigen
)

ament_target_dependencies(ouster_ros_library
  ${ouster_ros_library_deps}
)

target_link_libraries(ouster_ros_library
  # PUBLIC (unsupported)
    ouster_build
    pcl_common
  # PRIVATE (unsupported)
  -Wl,--whole-archive ouster_client -Wl,--no-whole-archive
)

# helper method to construct ouster-ros components
function(create_ros2_component
  component_lib_name
  src_list
  additonal_dependencies
)

  add_library(${component_lib_name} SHARED
    ${src_list})
  target_compile_definitions(${component_lib_name}
    PRIVATE
      "OUSTER_ROS_BUILDING_DLL"
  )

  ament_target_dependencies(${component_lib_name}
    rclcpp
    class_loader
    rclcpp_components
    rclcpp_lifecycle
    std_msgs
    sensor_msgs
    geometry_msgs
    ${additonal_dependencies}
  )

  target_link_libraries(${component_lib_name}
    ouster_ros_library
    ${cpp_typesupport_target}
  )

endfunction()


# ==== os_sensor_component ====
create_ros2_component(os_sensor_component
  "src/os_sensor_node_base.cpp;src/os_sensor_node.cpp"
  "std_srvs"
)
rclcpp_components_register_node(os_sensor_component
  PLUGIN "ouster_ros::OusterSensor"
  EXECUTABLE os_sensor
)

# ==== os_replay_component ====
create_ros2_component(os_replay_component
  "src/os_sensor_node_base.cpp;src/os_replay_node.cpp"
  ""
)
rclcpp_components_register_node(os_replay_component
  PLUGIN "ouster_ros::OusterReplay"
  EXECUTABLE os_replay
)

# ==== os_cloud_component ====
create_ros2_component(os_cloud_component
  "src/os_processing_node_base.cpp;src/os_cloud_node.cpp"
  "tf2_ros"
)
rclcpp_components_register_node(os_cloud_component
  PLUGIN "ouster_ros::OusterCloud"
  EXECUTABLE os_cloud
)

# ==== os_image_component ====
create_ros2_component(os_image_component
  "src/os_processing_node_base.cpp;src/os_image_node.cpp"
  ""
)
rclcpp_components_register_node(os_image_component
  PLUGIN "ouster_ros::OusterImage"
  EXECUTABLE os_image
)

# ==== os_driver_component ====
create_ros2_component(os_driver_component
  "src/os_sensor_node_base.cpp;src/os_sensor_node.cpp;src/os_driver_node.cpp"
  "std_srvs"
)
rclcpp_components_register_node(os_driver_component
  PLUGIN "ouster_ros::OusterDriver"
  EXECUTABLE os_driver
)


# ==== Test ====
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    src/os_ros.cpp
    test/test_main.cpp
    test/ring_buffer_test.cpp
    test/lock_free_ring_buffer_test.cpp
    test/point_accessor_test.cpp
    test/point_transform_test.cpp
    test/point_cloud_compose_test.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    ouster_sensor_msgs
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ${_ouster_ros_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test ouster_ros_library)
endif()


# ==== Install ====
install(
  TARGETS
    ouster_ros_library
    os_sensor_component
    os_replay_component
    os_cloud_component
    os_image_component
    os_driver_component
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

ament_export_include_directories(include) 
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(ouster_ros_library)
ament_export_dependencies(${ouster_ros_library_deps})
ament_package()
