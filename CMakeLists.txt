cmake_minimum_required(VERSION 3.10.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(tf2_eigen REQUIRED)
find_package(CURL REQUIRED)
find_package(Boost REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             pcl_conversions
             roscpp
             tf2
             tf2_ros
             nodelet)

# ==== Options ====
add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra)
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg Telemetry.msg)
add_service_files(FILES GetConfig.srv SetConfig.srv GetMetadata.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

set(_ouster_ros_INCLUDE_DIRS
  "include;"
  "ouster-sdk/ouster_client/include;"
  "ouster-sdk/ouster_client/include/optional-lite;"
  "ouster-sdk/ouster_pcap/include")

catkin_package(
  INCLUDE_DIRS
    ${_ouster_ros_INCLUDE_DIRS}
  LIBRARIES
    ouster_ros
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS
    EIGEN3
)

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

option(BUILD_VIZ "Enabled for Python build" OFF)
option(BUILD_PCAP "Enabled for Python build" OFF)
option(BUILD_OSF "Build OSF library." OFF)
find_package(OusterSDK REQUIRED)

set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

set(NODELET_SRC
  src/os_sensor_nodelet_base.cpp
  src/os_sensor_nodelet.cpp
  src/os_replay_nodelet.cpp
  src/os_cloud_nodelet.cpp
  src/os_image_nodelet.cpp
  src/os_driver_nodelet.cpp
)

set(OUSTER_TARGET_LINKS ouster_client)

if (BUILD_PCAP)
  list(APPEND OUSTER_TARGET_LINKS ouster_pcap)
  list(APPEND NODELET_SRC src/os_pcap_nodelet.cpp)
endif()

add_library(ouster_ros src/os_ros.cpp)
target_link_libraries(ouster_ros
  PUBLIC ${catkin_LIBRARIES} ouster_build pcl_common
  PRIVATE -Wl,--whole-archive ${OUSTER_TARGET_LINKS} -Wl,--no-whole-archive)
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_library(${PROJECT_NAME}_nodelets ${NODELET_SRC})
target_link_libraries(${PROJECT_NAME}_nodelets ouster_ros ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gencpp)

# ==== Test ====
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    src/os_ros.cpp
    tests/test_main.cpp
    tests/lock_free_ring_buffer_test.cpp
    tests/point_accessor_test.cpp
    tests/point_transform_test.cpp
    tests/point_cloud_compose_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES}
    ouster_build pcl_common -Wl,--whole-archive ${OUSTER_TARGET_LINKS} -Wl,--no-whole-archive)
endif()

# ==== Install ====
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/ouster_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY ouster-sdk/ouster_client/include/ouster/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../ouster
)

install(
  DIRECTORY ouster-sdk/ouster_client/include/optional-lite/nonstd/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../ouster/nonstd/
)
install(
  FILES
    LICENSE
    ${PROJECT_NAME}_nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
