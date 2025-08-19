=========
Changelog
=========

[unreleased]
============
* [BUGFIX]: correctly align timestamps to the generated point cloud.
* [BUGFIX]: NEAR_IR data is not populated with data for organized point clouds that have no range.
* Add support to enable **loop** for pcap replay + other replay config.
* Add a new launch file parameter ``pub_static_tf`` that allows users to turn off the braodcast
  of sensor TF transforms.
* Introduce a new topic ``/ouster/telemetry`` that publishes ``ouster_ros::Telemetry`` messages,
  the topic can be turned on/off by including the token ``TLM`` in the flag ``proc_mask`` launch arg.
* Add a new launch file parameter ``min_scan_valid_columns_ratio`` to allow users to set the minimum
  ratio of valid columns in a scan for it to be processed. Default value is ``0.0``.
* Update where ouster-ros and ouster_client include directories get installed so that those headers
  can be included externally.
* Add ``storage`` launch parameter to ``record.launch.xml``
* Add a padding-free point type of ``PointXYZI`` under ``ouster_ros`` namespace contrary to the pcl
  version ``pcl::PointXYZI`` for bandwith sensitive applications.
* [BUGFIX]: Use the node clock to ensure messages report sim time in replay mode.
* Introduce a new param ``v_reduction`` that allows reducing the number of beams count of the published
  point cloud
* Allow users to use ``Zenoh`` with the supplied Dockerfile and add it to the CI pipeline.
* Introduce a new capability to suppress certain range measurements of the point cloud by providing
  a mask image to the driver through the ``mask_path`` launch file argument.
* [BUGFIX]: Correct the computation of ``pointcloud.is_dense`` flag.


ouster_ros v0.13.2
==================
* [BUGFIX]: Make sure to initialize the sensor with launch file parameters.
* [BUGFIX]: ``os_driver`` failed when RAW option is used.


ouster_ros v0.13.0
==================
* [BUGFIX]: LaserScan is not properly aligned with generated point cloud
  * address an issue where LaserScan appeared different on FW prior to 2.4
* [BUGFIX]: LaserScan does not work when using dual mode
* [BUGFIX]: ROS2 crashes when standby mode is set and then set to normal
* [BUGFIX]: Implement lock free ring buffer with throttling to reduce partial frames
* add support for FUSA udp profile ``FUSA_RNG15_RFL8_NIR8_DUAL``.
* [BREAKING]: Set xyz values of individual points in the PointCloud to NaNs when range is zero.
* Added support to replay pcap format direclty from ouster-ros. The feature needs to be enabled
  explicitly by turning on the ``BUILD_PCAP`` cmake option and having ``libpcap-dev`` installed.
* [BREAKING] Added new launch files args ``azimuth_window_start`` and ``azimuth_window_end`` to
  allow users to set LIDAR FOV on startup. The new options will reset the current azimuth window
  to the default [0, 360] azimuth if not configured.
* Added a new launch ``persist_config`` option to request the sensor persist the current config
* Added a new ``loop`` option to the ``replay.launch.xml`` file.
* Added support for automatic sensor reconnection. Consult ``attempt_reconnect`` launch file arg
  documentation and the associated params to enable. Known Issues:
  - Doesn't handle detect and handle invalid configurations
* Added an automatic start mode to make it easier to start the node without using time actions.
  - To disable set ``auto_start`` to ``false`` during launch
* Added a new parameter ``organized`` to request publishing unorganized point cloud
* Added a new parameter ``destagger`` to request publishing staggered point cloud
* Added two parameters ``min_range``, ``max_range`` to limit the lidar effective range
* Updated ouster_client to the release of ``20240425`` [v0.11.1]; changes listed below.

ouster_client
-------------
* Added a new buffered UDP source implementation BufferedUDPSource.
* The method version_of_string is marked as deprecated, use version_from_string
instead.
* Added a new method firmware_version_from_metadata which works across firmwares.
* Added support for return order configuration parameter.
* Added support for gyro and accelerometer FSR configuration parameters.
* [BUGFIX] mtp_init_client throws a bad optional access.
* [BUGFIX] properly handle 32-bit frame IDs from the
* FUSA_RNG15_RFL8_NIR8_DUAL sensor UDP profile.


ouster_ros v0.12.0
==================
* [BREAKING]: updated ouster_client to the release of ``20231031`` [v0.10.0]; changes listed below.
* [BREAKING]: publish PCL point clouds destaggered.
* introduced a new launch file parameter ``ptp_utc_tai_offset`` which represent offset in seconds
  to be applied to all ROS messages the driver generates when ``TIME_FROM_PTP_1588`` timestamp mode
  is used.
  * [BREAKING]: the default value of ``ptp_utc_tai_offset`` is set to ``-37.0``. To retain the same
    time offset for an existing system, users need to set ``ptp_utc_tai_offset`` to ``0.0``.
* fix: destagger columns timestamp when generating destaggered point clouds.
* shutdown the driver when unable to connect to the sensor on startup
* breaking: rename ouster_msgs to ouster_sensor_msgs
* added the ability to customize the published point clouds(s) to velodyne point cloud format and
  other common pcl point types.
* ouster_image_compoenent can operate separately from ouster_cloud_component.
* fix: gracefully stop the driver when shutdown is requested.

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

ouster_ros v0.10.0
==================

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
* fix: on dual returns the 2nd point cloud replaces the 1st one.
* breaking: merge ``ouster-srvs`` package into ``ouster-msgs``.

ouster_client
-------------
* added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* breaking change: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* breaking change: remove deprecations on LidarScan
