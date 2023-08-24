=========
Changelog
=========

[unreleased]
============
ouster_ros(1)
-------------
* breaking: publish PCL point clouds destaggered.
* introduced a new launch file parameter ``ptp_utc_tai_offset`` which represent offset in seconds
  to be applied to all ROS messages the driver generates when ``TIME_FROM_PTP_1588`` timestamp mode
  is used.
* fix: destagger columns timestamp when generating destaggered point clouds.


ouster_ros v0.10.0
==================

ouster_ros(1)
-------------
* breaking change: update to ouster_client release 20230403
* EOL notice: ouster-ros driver will drop support for ``ROS melodic`` by May 2023.
* bugfix: Address an issue causing the driver to warn about missing non-legacy fields even they exist
  in the original metadata file.
* added a new launch file ``sensor_mtp.launch`` for multicast use case (experimental).
* added a technique to estimate the the value of the lidar scan timestamp when it is missing packets
  at the beginning
* add frame_id to image topics
* fixed a potential issue of time values within generated point clouds that could result in a value
  overflow
* added a new ``/ouster/metadata`` topic that is consumed by os_cloud and os_image nodelets and
  save it to the bag file on record.
* make specifying metadata file optional during record and replay modes as of package version 8.1
* added a no-bond option to the ``sensor.launch`` file
* reduce the publish rate of imu tf transforms
* implemented a new node named ``os_driver`` which combines the functionality of ``os_sensor``,
  ``os_cloud`` and ``os_image`` into a single node. The new node can be launch via the new 
  ``driver.launch`` file.
* introduced a new topic ``/ouster/scan`` which publishes ``sensor_msgs::LaserScan`` messages, the
  user can pick which beam to be used for the message through the ``scan_ring`` launch argument.
* added ability to pick which messsages to process and through the new ``proc_mask`` launch file
  argument.
* introduced a new parameter ``point_cloud_frame`` to allow users to select which frame to use when
  publishing the point cloud (choose between ``sensor`` and ``lidar``). The default publishing frame
  the sensor one which is in line with the current behavior.
* added the ability to change the names of ``sensor_frame``, ``lidar_frame`` and ``imu_frame``
* added a placeholder for the ``/ouster/reset`` (not implemented for ROS1).
* breaking: switched back to using static transforms broadcast but with ability to select the frames
  to be updated dynamically and at what rate through the two new launch file arguments
  ``dynamic_transforms_broadcast`` and  ``dynamic_transforms_broadcast_rate``.
* updated RVIZ color scheme for point clouds to match with the ROS2 version of the driver.

ouster_ros(2)
-------------
* MVP ouster-ros targeting ros2 distros
* introduced a ``reset`` service to the ``os_sensor`` node
* implemented a new node named ``os_driver`` which combines the functionality of ``os_sensor``,
  ``os_cloud`` and ``os_image`` into a single node.
* added support to parse the same parameters provided by the ``ros2_ouster_driver``, the parameters
  are ``lidar_ip``, ``computer_ip``, ``proc_mask`` and ``use_system_default_qos``; the parameters
  are fully functional and similar to what the ``ros2_ouster_driver`` provides.
* for convenience introduced a new launch file ``driver_launch.py`` that is compatible with the 
  ``ros2_ouster_driver`` in terms of parameters it accepts and the name of published topics.
* introduced a new parameter ``point_cloud_frame`` to allow users to select which frame to use when
  publishing the point cloud (choose between ``sensor`` and ``lidar``).
* breaking: ``lidar`` frame is the default frame used when publishing point clouds.
* added the ability to choose between ``SensorDataQoS`` or ``SystemDefaultQoS`` across all published
  topics with ``SensorDataQoS`` selected by default for live sensor mode and ``SystemDefaultQoS``
  enabled for record and replay modes.
* introduced a new topic ``/ouster/scan`` which publishes ``sensor_msgs::msg::LaserScan`` messages


ouster_client
-------------
* added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* breaking change: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* breaking change: remove deprecations on LidarScan


[20230114]
==========

ouster_ros(2)
-------------
* MVP ouster-ros targeting ros2 distros (beta release)
* introduced a ``reset`` service to the ``os_sensor`` node
* breaking change: updated to ouster sdk release 20230403
* EOL notice: ouster-ros driver will drop support for ``ROS foxy`` by May 2023.
* bugfix: Address an issue causing the driver to warn about missing non-legacy fields even they exist
  in the original metadata file.
* added a new launch file ``sensor_mtp.launch.xml`` for multicast use case (experimental).
* added a technique to estimate the the value of the lidar scan timestamp when it is missing packets
  at the beginning
* add frame_id to image topics
* fixed a potential issue of time values within generated point clouds that could result in a value
  overflow
* added a new ``/ouster/metadata`` topic that is consumed by os_cloud and os_image nodes and save it
  to the bag file on record
* make specifying metadata file optional during record and replay modes as of package version 8.1
* replace ``tf_prefix`` from ``os_cloud`` with ``sensor_frame``, ``lidar_frame`` and ``imu_frame``
  launch parameters.
* bugfix: fixed an issue that prevents running multiple instances of the sensor and cloud components
  in the same process.
* switch to using static transform publisher for the ros2 driver.

ouster_client
-------------
* added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* breaking change: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* breaking change: remove deprecations on LidarScan
