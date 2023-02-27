# Official ROS2 driver for Ouster sensors (BETA)

[Requirements](#requirements) | [Getting Started](#getting-started) | [Usage](#usage) | [License](#license)


<p style="float: right;"><img width="20%" src="docs/images/logo.png" /></p>

This ROS package provide support for all Ouster sensors with FW v2.0 or later targeting ros2 distros.
Upon launch the driver will configure and connect to the selected sensor device, once connected the 
driver will handle incoming IMU and lidar packets, decode lidar frames and publish corresponding ROS
messages on the topics of `/ouster/imu` and `/ouster/points`. In the case the used sensor supports
dual return and it was configured to use this capability, then another topic will published under the
name `/ouster/points2` which corresponds to the second point cloud.

## Requirements
This driver only supports **Foxy** ROS 2 distro. Please refer to ROS 2 online documentation on how to
setup ros on your machine before proceeding with the remainder of this guide.

> **Note**  
> If you have _rosdep_ tool installed on your system you can then use the following command to get all
    required dependencies:  
    ```
    rosdep install -y --from-paths $OUSTER_ROS_PATH --ignore-src -r
    ```

### Linux

In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2
```
where `$ROS_DISTRO` is ``foxy``.

> **Note**  
> Installing `ros-$ROS_DISTRO-rviz` package is optional in case you didn't need to visualize the
> point cloud using rviz but remember to always set `viz` launch arg to `false`.
  
The following packages are also required
```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```
> **Note**  
> You may choose a different _ssl_ backend for the _curl_ library such as `libcurl4-gnutls-dev` or
> `libcurl4-nss-dev`



### Windows
TBD


### Mac
TBD


## Getting Started
To build the driver using ROS2 you need to clone the project into the `src` folder of a ros2
workspace as shown below:

```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone -b ros2-foxy --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'foxy'
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
source ros2_ws/install/setup.bash
```

## Usage

### Launching Nodes
The package supports three modes of interaction, you can connect to a live sensor, replay a recorded
bag or record a new bag file using the corresponding launch files. The commands are listed below:

#### Sensor Mode
```bash
ros2 launch ouster_ros sensor.launch.xml    \
    sensor_hostname:=<sensor host name>
```

#### Recording Mode
```bash
ros2 launch ouster_ros record.launch.xml    \
    sensor_hostname:=<sensor host name>     \
    metadata:=<json file name>              \
    bag_file:=<optional bag file name>
```

#### Replay Mode
```bash
ros2 launch ouster_ros replay.launch.xml    \
    metadata:=<json file name>              \
    bag_file:=<path to rosbag file>
```

### Invoking Services
To execute any of the following service, first you need to open a new terminal
and source the ros2 workspace again by running the command
`source ros2_ws/install/setup.bash` 
#### GetMetadata
To get metadata while connected to a live sensor or during a replay session invoke
the following command:
```bash
ros2 service call /ouster/get_metadata ouster_srvs/srv/GetMetadata
```

#### GetConfig
To get the current config of a live sensor, invoke the command:
```bash
ros2 service call /ouster/get_config ouster_srvs/srv/GetConfig
```

#### SetConfig (experimental)
To change config via a file while connected to a live sensor, invoke the command:
```bash
ros2 service call /ouster/set_config ouster_srvs/srv/SetConfig \
    "{config_file: 'some_config.json'}"
```

#### Reset (experimental)
To reset the new reset service, execute the following commnad:
```bash
ros2 service call /ouster/reset std_srvs/srv/Empty
```
When this service is invoked the client should stop streaming, dispose current
connection, reset the sensor and reconnect again. 

> **Note**
> Changing settings is not yet fully support during a reset operation (more on this)

TBD: For further detailed instructions refer to the [main guide](./docs/index.rst)


## License
[License File](./LICENSE)
