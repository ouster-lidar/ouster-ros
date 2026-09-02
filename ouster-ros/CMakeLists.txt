cmake_minimum_required(VERSION 3.14.0)

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
find_package(lifecycle_msgs REQUIRED)
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

# Prefer exported targets, while preserving ament's legacy dependency handling
# on older ROS 2 distributions.
function(_ouster_link_ament_deps target visibility)
  foreach(_pkg IN LISTS ARGN)
    if(TARGET ${_pkg}::${_pkg})
      target_link_libraries(${target} ${visibility} ${_pkg}::${_pkg})
      continue()
    endif()

    set(_ouster_pkg_targets "")
    foreach(_candidate IN LISTS ${_pkg}_TARGETS ${_pkg}_INTERFACES)
      if(TARGET ${_candidate})
        list(APPEND _ouster_pkg_targets ${_candidate})
      endif()
    endforeach()
    if(_ouster_pkg_targets)
      target_link_libraries(${target} ${visibility} ${_ouster_pkg_targets})
    elseif(COMMAND ament_target_dependencies)
      ament_target_dependencies(${target} ${visibility} ${_pkg})
    elseif(DEFINED ${_pkg}_INCLUDE_DIRS OR
           DEFINED ${_pkg}_LIBRARIES OR
           DEFINED ${_pkg}_DEFINITIONS OR
           DEFINED ${_pkg}_LIBRARY_DIRS OR
           DEFINED ${_pkg}_LINK_FLAGS)
      if(${_pkg}_DEFINITIONS)
        target_compile_definitions(${target} ${visibility}
          ${${_pkg}_DEFINITIONS})
      endif()
      if(${_pkg}_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM ${visibility}
          ${${_pkg}_INCLUDE_DIRS})
      endif()
      if(${_pkg}_LIBRARY_DIRS)
        target_link_directories(${target} ${visibility}
          ${${_pkg}_LIBRARY_DIRS})
      endif()
      if(${_pkg}_LIBRARIES)
        target_link_libraries(${target} ${visibility} ${${_pkg}_LIBRARIES})
      endif()
      if(${_pkg}_LINK_FLAGS)
        target_link_options(${target} ${visibility} ${${_pkg}_LINK_FLAGS})
      endif()
    else()
      message(FATAL_ERROR
        "No CMake linkage metadata found for dependency '${_pkg}'")
    endif()
  endforeach()
endfunction()

# ==== Options ====
if(MSVC)
  add_compile_options(/W2)
else()
  add_compile_options(-Wall -Wextra)
endif()

if(WIN32)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
  if(NOT DEFINED CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON CACHE BOOL "Export all symbols on Windows by default")
  endif()
endif()

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

option(BUILD_VIZ "Build SDK visualizer" OFF)
option(BUILD_PCAP "Build PCAP replay support" OFF)
option(BUILD_OSF "Build SDK OSF support" OFF)
option(BUILD_MAPPING "Build SDK mapping support" OFF)
find_package(OusterSDK REQUIRED)

# Keep warnings enabled for the ROS wrapper while suppressing known warnings
# from vendored SDK sources.
foreach(_sdk_target ouster_client ouster_sensor ouster_pcap ouster_osf ouster_viz)
  if(TARGET ${_sdk_target} AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${_sdk_target} PRIVATE
      -Wno-pedantic
      -Wno-deprecated-declarations
    )
  endif()
endforeach()

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(
  ${_ouster_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_compile_definitions(EIGEN_MPL2_ONLY)

set(OUSTER_TARGET_LINKS ouster_client)

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
    # Installed os_ros.h templates call OpenCV.
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
  "std_srvs;lifecycle_msgs"
)
rclcpp_components_register_node(os_sensor_component
  PLUGIN "ouster_ros::OusterSensor"
  EXECUTABLE os_sensor
)

# ==== os_replay_component ====
create_ros2_component(os_replay_component
  "src/os_sensor_node_base.cpp;src/os_replay_node.cpp"
  "lifecycle_msgs"
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
  "std_srvs;lifecycle_msgs"
)
rclcpp_components_register_node(os_driver_component
  PLUGIN "ouster_ros::OusterDriver"
  EXECUTABLE os_driver
)


if (BUILD_PCAP)
# ==== os_pcap_component ====
create_ros2_component(os_pcap_component
  "src/os_sensor_node_base.cpp;src/os_pcap_node.cpp"
  "lifecycle_msgs"
)
# os_pcap_node.cpp calls PcapReader directly.
target_link_libraries(os_pcap_component PRIVATE OusterSDK::ouster_pcap)
rclcpp_components_register_node(os_pcap_component
  PLUGIN "ouster_ros::OusterPcap"
  EXECUTABLE os_pcap
)
endif()

# ==== Test ====
if(BUILD_TESTING)
  # OusterSDK's FindGTest can prevent ament_add_gtest from creating the target.
  # Use CMake's finder here, then restore the module path.
  set(_ouster_ros_saved_cmake_module_path "${CMAKE_MODULE_PATH}")
  list(REMOVE_ITEM CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk/cmake")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
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
    # Avoid mixing the plain and keyword link signatures used across distros.
    set_property(TARGET ${PROJECT_NAME}_test APPEND PROPERTY
      LINK_LIBRARIES ouster_ros_library)
  endif()
  set(CMAKE_MODULE_PATH "${_ouster_ros_saved_cmake_module_path}")
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
    LICENSE
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
ament_export_dependencies(
  ${ouster_ros_library_deps}
  rclcpp_lifecycle
  lifecycle_msgs
  Eigen3
  PCL
  OpenCV
)
ament_package()
