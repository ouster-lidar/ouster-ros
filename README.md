# Official ROS driver for Ouster sensors

[Requirements](#requirements) | [Getting Started](#getting-started) | [Usage](#usage) | [License](#license)


<p style="float: right;"><img width="20%" src="docs/images/logo.png" /></p>

This ROS package provide support for all Ouster sensors with FW v2.0 or later. Upon launch the driver
will configure and connect to the selected sensor device, once connected the driver will handle
incoming IMU and lidar packets, decode lidar frames and publish corresponding ROS messages on the
topics of `/ouster/imu` and `/ouster/points`. In the case the sensor supports dual return and it was
configured to use this capability, then another topic will published named `/ouster/points2` which
corresponds to the second point cloud.

## Requirements
This driver only supports **Rolling** and **Humble** ROS 2 distros.

> **Info**  
> If you have _rosdep_ tool installed on your system you can then use the following command to get all
    required dependencies:  
    ```
    rosdep install -y --from-paths $OUSTER_ROS_PATH --ignore-src -r
    ```

### Linux

The following packages are required
```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    cmake                   \
    libcurl4-openssl-dev    \
    python3-colcon-common-extensions
```
> **Note**  
> You may choose a different ssl backend for the curl library such as `libcurl4-gnutls-dev`
  or `libcurl4-nss-dev`


In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y 
# TODO...
```

where `$ROS-DISTRO` is either ``rolling`` or ``humble``.


## Getting Started
To build the driver using ROS2 you need to clone the project into the `src` folder of a ros2
workspace as shown below:

```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'rolling' or 'humble'
```

Finally, invoke `colcon build` command from within the catkin workspace as shown below:
```bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
> **Note**  
> Specifying `Release` as the build type is important to have a reasonable performance of the driver.

Once the build succeeds, you must source the _install_ folder of your ros2 workspace to add launch
commands to your environment:
```bash
source ros2_ws/install/setup.bash # replace ros-distro with 'rolling' or 'humble'
```

## Usage
The package supports three modes of interaction, you can connect to a live senosr, replay a recorded
bag or record a new bag file using the corresponding launch files. The commands are listed below:

### Sensor Mode
```bash
ros2 launch ouster_ros sensor.launch.xml    \
    sensor_hostname:=<sensor host name>
```

### Recording Mode
```bash
ros2 launch ouster_ros record.launch.xml    \
    sensor_hostname:=<sensor host name>     \
    metadata:=<json file name>              \
    bag_file:=<optional bag file name>
```

### Replay Mode
```bash
ros2 launch ouster_ros replay.launch.xml    \
    metadata:=<json file name>              \
    bag_file:=<path to rosbag file>
```

For further detailed instructions refer to the [main guide](./docs/index.rst)


## License
[License File](./LICENSE)
