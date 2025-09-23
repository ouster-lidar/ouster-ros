^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ouster_sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.14 (2025-09-22)
--------------------
* SW-6906: publish sensor telemetry in ouster ros (#422)
  * Port changes from ROS1 to ROS2
  * Mention TLM as an option in yaml configs
* Use timeout when waiting for packets to be proceed in case they don't come (#293)
* SW-5623: Bump up ouster_client to 20231031 release (#262)
  * Bump ouster-client to 2023103 release
  * fix: gracefully stop the driver when shutdown is requested.
* SW-5466: Support Velodyne point type in the ROS driver amendments (#254)
  * Add support to control point_type through launch.xml files +
  * Add a note to CHANGELOG about the breaking change for ptp/utc time offset
* SW-5466: Support Velodyne and other point types in ouster-ros driver (#216)
  * Quick protoype of Velodyne point type
  * Add PointXYZIR point type + other major pcl point types
  * Include point meta functions and point transform by the ouster_ros namesapce
  * Wrap point meta functions with a namespace and use shorter names for the functions +
  * Add a seed test module for the point_cloud_compose methods +
  Add description for the point_cloud_compose methods + refactor code and add compile time checks.
  * Propagate error state, warn about potential incompatible profile, propagate error state
  * Add minimal documentation about the new `point_type` parameter.
* SW-5607: rename package ouster_msgs to avoid package name conflict in ros index (#244)
  * rename package ouster_msgs to avoid conflict name conflict in ros index
  * set ouster_sensor_msgs version number to match with ouster_ros package
* Contributors: Ussama Naal
