cmake_minimum_required(VERSION 3.10.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ouster_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
# Use OpenCV libs variable (OpenCV::OpenCV target may not resolve on all systems)
if(NOT OpenCV_LIBS)
  set(OpenCV_LIBS ${OpenCV_LIBRARIES})
endif()

# Link an ament package as a dependency. ament_cmake exposes packages in
# three shapes across ROS 2 distros; we try each in turn:
#   1. ${pkg}_TARGETS         - rosidl pkgs (msgs/srvs); library pkgs on
#                               Lyrical+ (e.g. tf2).
#   2. pkg::pkg imported tgt  - library pkgs on Humble/Jazzy/Kilted.
#   3. ${pkg}_INCLUDE_DIRS /
#      ${pkg}_LIBRARIES       - legacy header-only pkgs (pcl_conversions
#                               on Humble/Jazzy/Kilted).
function(_ouster_link_ament_deps target visibility)
  foreach(_pkg IN LISTS ARGN)
    if(DEFINED ${_pkg}_TARGETS)
      target_link_libraries(${target} ${visibility} ${${_pkg}_TARGETS})
    elseif(TARGET ${_pkg}::${_pkg})
      target_link_libraries(${target} ${visibility} ${_pkg}::${_pkg})
    elseif(DEFINED ${_pkg}_INCLUDE_DIRS OR DEFINED ${_pkg}_LIBRARIES)
      if(${_pkg}_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM ${visibility}
          ${${_pkg}_INCLUDE_DIRS})
      endif()
      if(${_pkg}_LIBRARIES)
        target_link_libraries(${target} ${visibility} ${${_pkg}_LIBRARIES})
      endif()
    else()
      message(FATAL_ERROR
        "_ouster_link_ament_deps: cannot resolve dependency '${_pkg}' "
        "(none of ${_pkg}_TARGETS, target ${_pkg}::${_pkg}, or legacy "
        "${_pkg}_INCLUDE_DIRS/${_pkg}_LIBRARIES are defined)")
    endif()
  endforeach()
endfunction()

# ==== Options ====
add_compile_options(-Wall -Wextra)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

set(_ouster_ros_INCLUDE_DIRS
  include
  ouster-sdk/ouster_sensor/include
  ouster-sdk/ouster_client/include
  ouster-sdk/ouster_pcap/include
  ouster-sdk/ouster_client/include/optional-lite
)

# ==== Libraries ====
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

option(BUILD_VIZ "Turn off building VIZ" OFF)
option(BUILD_PCAP "Turn off building PCAP" OFF)
option(BUILD_OSF "Turn off building OSF" OFF)
option(BUILD_MAPPING "Turn off building MAPPING" OFF)
find_package(OusterSDK REQUIRED)

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(
  ${_ouster_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_compile_definitions(EIGEN_MPL2_ONLY)

set(OUSTER_TARGET_LINKS ouster_client)
if (BUILD_PCAP)
  list(APPEND OUSTER_TARGET_LINKS ouster_pcap)
endif()

add_library(ouster_ros_library SHARED
  src/impl/file_util.cpp
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

_ouster_link_ament_deps(ouster_ros_library PUBLIC
  ${ouster_ros_library_deps}
)

target_link_libraries(ouster_ros_library
  PUBLIC
    ouster_build
    OusterSDK::ouster_sensor
    pcl_common
    # OpenCV must stay PUBLIC: impl::load_mask in the installed public header
    # os_ros.h is a template calling cv:: functions, instantiated inside the
    # component libraries that link only ouster_ros_library.
    ${OpenCV_LIBS}
  PRIVATE
    ${OUSTER_TARGET_LINKS}
)

# helper method to construct ouster-ros components
function(create_ros2_component
  component_lib_name
  src_list
  additional_dependencies
)

  add_library(${component_lib_name} SHARED
    ${src_list})
  target_compile_definitions(${component_lib_name}
    PRIVATE
      "OUSTER_ROS_BUILDING_DLL"
  )

  _ouster_link_ament_deps(${component_lib_name} PUBLIC
    rclcpp
    class_loader
    rclcpp_components
    rclcpp_lifecycle
    std_msgs
    sensor_msgs
    geometry_msgs
    ${additional_dependencies}
  )

  target_link_libraries(${component_lib_name}
    PUBLIC
      ouster_ros_library
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


if (BUILD_PCAP)
# ==== os_pcap_component ====
create_ros2_component(os_pcap_component
  "src/os_sensor_node_base.cpp;src/os_pcap_node.cpp"
  ""
)
rclcpp_components_register_node(os_pcap_component
  PLUGIN "ouster_ros::OusterPcap"
  EXECUTABLE os_pcap
)
endif()

# ==== Test ====
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    src/os_ros.cpp
    test/test_main.cpp
    test/lock_free_ring_buffer_test.cpp
    test/point_accessor_test.cpp
    test/point_transform_test.cpp
    test/point_cloud_compose_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    target_include_directories(${PROJECT_NAME}_test PRIVATE
      ${_ouster_ros_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    # ament_add_gtest links gtest into the target with the plain
    # target_link_libraries signature on Humble/Jazzy/Kilted and the keyword
    # signature on Lyrical; CMake forbids mixing the two on one target.
    # Appending to LINK_LIBRARIES directly is signature-neutral and, for an
    # executable, equivalent to PRIVATE linkage. The test's remaining deps
    # (rclcpp, ouster_sensor_msgs, PCL, ...) propagate through
    # ouster_ros_library's PUBLIC usage requirements.
    set_property(TARGET ${PROJECT_NAME}_test APPEND PROPERTY
      LINK_LIBRARIES ouster_ros_library)
  endif()
endif()


# ==== Install ====
set(OUSTER_INSTALL_TARGETS
  ouster_ros_library
  os_sensor_component
  os_replay_component
  os_cloud_component
  os_image_component
  os_driver_component
)
if (BUILD_PCAP)
  list(APPEND OUSTER_INSTALL_TARGETS os_pcap_component)
endif()

install(
  TARGETS
    ${OUSTER_INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}/
)

install(
  DIRECTORY
    ouster-sdk/ouster_client/include/ouster
    ouster-sdk/ouster_client/include/optional-lite/nonstd
  DESTINATION include
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
ament_export_libraries(ouster_ros_library)
ament_export_dependencies(${ouster_ros_library_deps})
ament_package()
