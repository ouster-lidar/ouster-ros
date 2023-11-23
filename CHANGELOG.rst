=========
Changelog
=========

[unreleased]
============
* [BREAKING] ROS1 driver code now requires C++17 (required for point cloud customization feature).
* added the ability to customize the published point clouds(s) to velodyne point cloud format and
  other common pcl point types.
* ouster_image_nodelet can operate independently from ouster_cloud_nodelet.
* install ouster-ros and ouster_client include directories in separate folders.
* [BUGFIX]: LaserScan is not properly aligned with generated point cloud
  * address an issue where LaserScan appeared different on FW prior to 2.4
* [BUGFIX]: LaserScan does not work when using dual mode


ouster_ros v0.10.0
==================

ouster_ros(1)
-------------
* [BREAKING]: updated ouster_client to the release of ``20231031`` [v0.10.0]; changes listed below.
* [BREAKING]: with this release the ouster-ros driver is no longer compatible with ROS melodic
* [BREAKING]: publish PCL point clouds destaggered.
* introduced a new launch file parameter ``ptp_utc_tai_offset`` which represent offset in seconds
  to be applied to all ROS messages the driver generates when ``TIME_FROM_PTP_1588`` timestamp mode
  is used.
  * [BREAKING]: the default value of ``ptp_utc_tai_offset`` is set to ``-37.0``. To retain the same
    time offset for an existing system, users need to set ``ptp_utc_tai_offset`` to ``0.0``.
* [BUGFIX]: destagger columns timestamp when generating destaggered point clouds.
* [BUGFIX]: gracefully stop the driver when shutdown is requested.

ouster_client
-------------
* [BREAKING] Updates to ``sensor_info`` include:
    * new fields added: ``build_date``, ``image_rev``, ``prod_pn``, ``status``, ``cal`` (representing
      the value stored in the ``calibration_status`` metadata JSON key), ``config`` (representing the
      value of the ``sensor_config`` metadata JSON key)
    * the original JSON string is accessible via the ``original_string()`` method
    * The ``updated_metadata_string()`` now returns a JSON string reflecting any modifications to
      ``sensor_info``
    * ``to_string`` is now marked as deprecated
* [BREAKING] The RANGE field defined in `parsing.cpp`, for the low data rate profile, is now 32 bits
  wide (originally 16 bits).
    * Please note this fixes a SDK bug. The underlying UDP format is unchanged.
* [BREAKING] The NEAR_IR field defined in `parsing.cpp`, for the low data rate profile, is now 16
  bits wide (originally 8 bits).
    * Plase note this fixes a SDK bug. The underlying UDP format is unchanged.
* [BREAKING] changed frame_id return size to 32 bits from 16 bits
* An array of per-packet timestamps (called ``packet_timestamp``) is added to ``LidarScan``
* The client now retries failed requests to an Ouster sensor's HTTP API
* Increased the default timeout for HTTP requests to 40s
* Added FuSA UDP profile to support Ouster FW 3.1+
* Improved ``ScanBatcher`` performance by roughly 3x (depending on hardware)
* Receive buffer size increased from 256KB to 1MB
* [bugfix] Fixed an issue that caused incorrect Cartesian point computation in the ``viz.Cloud``
  Python class
* [bugfix] Fixed an issue that resulted in some ``packet_format`` methods returning an uninitialized
  value
* [bugfix] Fixed a libpcap-related linking issue
* [bugfix] Fixed an eigen 3.3-related linking issue
* [bugfix] Fixed a zero beam angle calculation issue
* [bugfix] Fixed dropped columns issue with 4096x5 and 2048x10


ouster_ros v0.9.0
==================

ouster_ros(1)
-------------
* EOL notice: ouster-ros driver will drop support for ``ROS melodic`` by May 2023.
* [BREAKING]: update to ouster_client release 20230403
* [BUGFIX]: Address an issue causing the driver to warn about missing non-legacy fields even they exist
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
* [BREAKING]: switched back to using static transforms broadcast but with ability to select the
  frames to be updated dynamically and at what rate through the two new launch file arguments
  ``dynamic_transforms_broadcast`` and  ``dynamic_transforms_broadcast_rate``.
* updated RVIZ color scheme for point clouds to match with the ROS2 version of the driver.

ouster_client
-------------
* added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* [BREAKING]: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* [BREAKING]: remove deprecations on LidarScan
