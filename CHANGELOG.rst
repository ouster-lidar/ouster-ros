=========
Changelog
=========

[unreleased]
============

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
