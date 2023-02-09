# Official ROS1/ROS2 drivers for Ouster sensors

[Requirements](#requirements) | [Getting Started](#getting-started) | [Usage](#usage) | [License](#license)


<p style="float: right;"><img width="20%" src="docs/images/logo.png" /></p>

This ROS package provide support for all Ouster sensors with FW v2.0 or later. Upon launch the driver
will configure and connect to the selected sensor device, once connected the driver will handle
incoming IMU and lidar packets, decode lidar frames and publish corresponding ROS messages on the
topics of `/ouster/imu` and `/ouster/points`. In the case the sensor supports dual return and it was
configured to use this capability, then another topic will published named `/ouster/points2` which
corresponds to the second point cloud.

## Requirements
This driver only supports Melodic and Noetic ROS distros.

In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y                     \
    ros-$ROS_DISTRO-pcl-ros             \
    ros-$ROS_DISTRO-rviz
```

where `$ROS-DISTRO` is either ``melodic`` or ``noetic``.

Additional dependenices:
```bash
sudo apt install -y \
    build-essential \
    libeigen3-dev   \
    libjsoncpp-dev  \
    libspdlog-dev   \
    cmake
```

## Getting Started
To build the driver using ROS you need to clone the project into the `src` folder of a catkin workspace
as shown below:

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
The package supports three modes of interaction, you can connect to a live senosr, replay a recorded bag or record a new
bag file using the corresponding launch files. The commands are listed below

### Sensor Mode
```bash
roslaunch ouster_ros sensor.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>              # metadata is optional
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

For further detailed instructions refer to the [main guide](./docs/index.rst)


## License
[License File](./LICENSE)
