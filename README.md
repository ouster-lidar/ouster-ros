# Official ROS driver for Ouster sensors

[Overview](#overview) |
[Getting Started](#getting-started) |
[Usage](#usage)


## Overview
<p align="center"><img width="20%" src="doc/images/logo.png" /></p>

## Requirements
- ROS Melodic or ROS Noetic


## Getting Started
To build the driver using ROS you need to clone the project in the `src` folder of a catkin workspace
To do so use the following steps:

```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'melodic' or 'noetic'
```

Finally, invoke `catkin_make` command from within the catkin workspace as shown below:
```bash
cd catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Specifying `Release` as the build type is important to have a reasonable performance of the driver.


## Usage

### Sensor Mode
```bash
roslaunch ouster_ros sensor.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>
```

### Replay Mode
```bash
roslaunch ouster_ros replay.launch      \
    metadata:=<json file name>          \
    bag_file:=<path to rosbag file>
```

### Recording Mode
```bash
roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>          \
    bag_file:=<optional bag file name>
```

## Services
The ROS driver currently advertises three services `/ouster/get_metadata`,
`/ouster/get_config`, and `/ouster/set_config`. The first one is available
in all three modes of operation: Sensor, Replay, and Recording. The latter two,
however, are only available in Sensor and Recording modes. i.e. when connected
to a sensor.

The usage of the three services is described below:
* `/ouster/get_metadata`: This service takes no parameters and returns the
current sensor metadata, you may use as follow:
```bash
rosservice call /ouster/get_metadata "{}"
```
This will return a json string that contains the sensor metadata

* `/ouster/get_config`: This service takes no parameters and returns the
current sensor configuration, you may use as follow:
```bash
rosservice call /ouster/get_config "{}"
```
This will return a json string represting the current configuration

* `/ouster/set_config`: Takes a single parameter and also returns the updated
sensor configuration. You may use as follows:
```bash
rosservice call /ouster/set_config "config_file: '<path to sensor config>'"
```
It is not guranteed that all requested configuration are applied to the sensor,
thus it is the caller responsibilty to examine the returned json object and
check which of the sensor configuration parameters were successfully applied.