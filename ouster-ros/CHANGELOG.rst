^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ouster_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.14 (2025-09-22)
--------------------
* Add ouster-sdk directly
* Drop whole archive linkage (#489)
* fix-rolling build (#483)
* Correct pointcloud.is_dense flag (#473)
* Port the pointcloud mask feature to ROS2 (#462)
  * Port the pointcloud mask feature to ROS2
  * Base Image fixed
  * Update README and exclude iron from the build
* [ROS2] Expose the vertical beam reduction param (#444)
  * Expose the vertical beam reduction value + rename arg + Better handling of deprecated topic names
* ROS-317: Fix simulation in replay mode (#440)
  * Use node clock to properly populate the timestamp in replay modes
  * Update changelog and package number
* Implement a padding-free version of pcl::PointXYZI (#439)
* Add a new storage param to record.launch.xml (#437)
  * Add storage param to record
* [ros2] Fix install directories (#433)
  * Cmakelists: Fix install directories
  * Bump package.xml patch version
  * Update changelog
  ---------
  Co-authored-by: Michael Wiznitzer <mwiznitzer@neyarobotics.com>
* Port the changes to ROS2 (#430)
  * Port the changes to ROS2
  * Update sensor.*.launch
* SW-6906: publish sensor telemetry in ouster ros (#422)
  * Port changes from ROS1 to ROS2
  * Mention TLM as an option in yaml configs
* Enable auto start on replay node (#421)
* ROS-389 [rolling/humble/iron/jazzy]: replay-improvments-and-fixes (#393)
  Remap metadata topic + Support loop capability in pcap replay + Add play_delay & play_rate for replay
  Added a launch file parameter pub_static_tf to disable sensor transforms broadcast
  ---------
  Co-authored-by: Guillaume Doisy <guillaume@dexory.com>
* ROS-350[HUMBLE/IRON/JAZZY]: 't' timestamp field content is not plausible (#387)
  Align timestamps based on staggered/destaggered option
* HOTFIX/ROS-382: os_driver fails when raw option is enabled (#384)
  * Fix os_driver fails when RAW option is enabled
  * Replace lifecycle_publisher with regular publisher for the os_pcap
* HOTFIX/ROS-376-initialize-the-sensor-with-launch-config-params (#380)
  * Invoke parse_config_from_ros_parameters on node init
  * Fix a typo + Add a note regarding the recommendation
* [HUMBLE|IRON|JAZZY] Port ROS-363 to ROS2 (#369)
  * Port ROS-363 to ROS2
  * Turn off OSF
  * Update package version and CHANGELOG.rst + up  the default max range  to 10000.0 meters
* ROS-368[HUMBLE|IRON|JAZZY]: Unable to use the replay mode due to unknown substitution arg (#370)
  * Fix replay unknown substitution arg
  * Fix the definition of _loop variable
  * Move remap verb to the node
* ROS-119: ouster-ros driver automatic reconnection [HUMBLE/IRON/JAZZY] (#362)
  * Port sensor reconnection logic to ROS 2
  * Add Jazzy to the build!
  * Update README and checkout
  * Add a note about not being able to properly handle invalid configuration
  * Implement automatic start for sensor/record modes
* ROS-227: Set LIDAR FOV on startup and add an option to persist the config [HUMBLE/IRON] (#357)
  * Port azimuth window and persist config changes to ROS2
* ROS2[HUMBLE/IRON] add pcap reader (#355)
  * Port the pcap replay to ros2-foxy
  * Add time update
* Support FUSA dual returns udp profile [HUMBLE/IRON] (#335)
  * Add support for FUSA profile + set xyz to NaNs on zero range
* Implement lock free ring buffer with throttling [HUMBLE/IRON] (#321)
  * Implement lock free ring buffer with throttling
  - (cherry picked from commit ade5822aba552f3839cf077daea44dc26869026b)
  * Update CHANGELOG and package version
* docs: fix spelling mistakes (#296)
* Use timeout when waiting for packets to be proceed in case they don't come (#293)
* ROS-196: laser scan from ros driver is not properly aligned with point cloud [humble] (#203)
  * Apply destagger to laser scan + Add laser to RVIZ
  * Align LaserScan with the PointCloud
  * Apply proper pixel shift
  * Resolve the issue of zeroed laserscan on dual mode
  * Address an issue where LaserScan appeared different on FW prior to 2.4
  * Fix the issue for odd numbers
  * List selected sensors on the main page + Update RVIZ config to highlight the 2D LaserScan.
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
* Shutdown when can not connect to sensor on startup in ROS2 (Humble/Iron) (#211)
  * Shutdown when can not connect to sensor on startup
  * Apply same behavior to independent launch file
  * Update changelog and package version
* SW-5459: add a parameter for utc/tai time offset (#195)
  * Implemented UTC/TAI offset for the PTP timestamp mode
  * Make sure all the timestamp values of LidarScan are utc corrected
  * Modify timestamp values before producing a PointCloud
  * Remove no longer needed comment from the Dockerfile
  * Destagger timestamp when generating destagger point clouds
* SW-5396: publish point cloud in destaggered form (#182)
  * Quick implementtion of cloud destaggering
  * Perform destaggering of point clouds during the copy + add a viz-reliable file + wrap all classes in ouster_ros
  * Update changelog and version
  * Reformat changelog
* SW-5345: merge-ouster-srvs into ouster-msgs (#176)
  * Merge ouster-srvs into ouster-msgs package + Update dockerfile + Other fixes and code improvements
  * Update launch file name within the dockerfile
  * Update CHANGELOG.txt
  * Update ouster_ros package version to highlight the breadking change due obseleting ouster_srvs
  * Use angle brackets for external headers
  * Remove unused library include
  * Remove unused launch params in sensor_mtp.launch
  * Fix the table of contents
* SW-5167: follow ups from SW-5167-fix-black-columns-when-recording-under-ros-foxy (#163)
  * Initialize point_cloud_frame with an empty string + and expose point_cloud_frame through launch xml files +
  Remove unused/duplicate param definitions + Update descriptions
  * Correct the name of the node to be activated for merged node
* ROS compatibility mode dual returns fix (#156)
  * Separately initialize vector elements
  * Update changelog and package version
  * Properly check for the write_text_to_file success
* ROS2 compatibility mode (#146)
  * Factoring out Imu and Lidar packets handling
  * Added os_driver which combines os_sensor and os_cloud +
  Added required launch files +
  Better abstraction of classes +
  Simplified threading logic +
  Added thread safe implemention of ring buffer (not hooked yet)
  * Move down pragma once in the handlers
  * Adding unit tests for the ThreadSafeRingBuffer
  * Add one more case to the unit test of ThreadSafeRingBuffer
  * Quick hook up of the ThreadSafeRingBuffer for os_sensor and os_driver
  * Add an option to select the point_cloud frame
  * Keep transforms in Lidar Frame by default with option to switch
  * Formatting os_sensor and os_driver
  * Provide support for parsing the community driver params file with approprite launch file
  * Factor out tf transforms broadcast
  * Formatting imu and lidar packet handlers
  * Fix build issue
  * Incorporate LaserScan message composition
  * Refactor a bit and add the ability to process and publish point clouds and laser scans
  * Restor os_cloud_node ability to process point clouds
  * Parse proc_mask and hook to launch files and config
  * Add support for the selecting IMU + create topics/subs when their respective flags enabled
  * Reduce sync operations + restore sensor reset/reactivation
  * Add the ability to override current qos settings
  * Add minor note
  * More detailed explanation about the IMG node
  * Rename the file os_sensor_cloud_image_params to os_sensor_cloud_image_params.yaml and update corresponding launch files
  * Expose use_system_default_qos parameter to xml launch file and use proper defaults
  * Update minimal readme file and utilize os_driver by default +
  add proc_mask to xml file +
  Remove experminal marker from set_config and reset ros services
  * Update CHANGELOG.rst and package version
  * Apply && to accepted method of ThreadSafeRingBuffer +
  nits and code formatting
  * Add missing parameter declaration
  * Re-formatting CHANGELOG a bit
  * Added ImageProcessor to support IMG flag +
  Define new argument scan_ring
  * Expose scan_ring param and update relevant params description
  * Added notes to size(), empty(), full()
  * remove deprecated methods + naming nits
  * Update ChangeLog + Fix load_metadata_from_file
  * Try out building against Iron + revise sensor_mtp.launch
  * Correct the params file name + document params
  * Carry over fixes from foxy branch
  * Update README.md to mention compatibilty mode
* SW-4997: Switch from using ROS timers to thread for polling lidar data (#140)
  * Switch from using ROS timers to thread for polling
  * Specify param defaults for non-required params
* SW-4972: merge switching to static transform publisher contribution (#124)
  * use static tf broadcaster for ros2 (#112)
  * use separate params for tf frames
  * send static transforms once
  * Disable static transform publishers and update changelog and package version
  * Disable rviz static transform publisher
  * Remove rviz static transform publisher hack
  * Remove left out variables
  ---------
  Co-authored-by: Adam Aposhian <adam.l.aposhian@gmail.com>
* SW-4859: enable having multiple components of same-type under same process (#108)
  * Remove the use of static vars within components
  * Resolve conflicts and update changelog and version
  * Fix a typo 'instance'
* Drop service_msgs dep (#117)
* SW-4924: Replace tf_prefix by sensor_frame lidar_frame and imu_frame parameters (#115)
  * deprecate tf_prefix from os_cloud (#96)
  Co-authored-by: Guillaume Doisy <guillaume@dexory.com>
  * Squashed commit of the following:
  commit 6280bfa1178bdee4fe695cb4752efd5ff15279db
  Author: Ussama Naal <ussama.naal@ouster.io>
  Date:   Fri Apr 28 07:54:34 2023 -0700
  Merge branch 'deprecate_tf_prefix'
  commit 35f2fd2ba50eaf3e4b65909269eb9609bff7a010
  Author: Guillaume Doisy <guillaume@dexory.com>
  Date:   Mon Apr 3 18:12:44 2023 +0100
  deprecate tf_prefix from os_cloud
  * Update ChangeLog and package version
  * Propagate the parameters to launch files
  * Add a TODO note
  ---------
  Co-authored-by: Guillaume Doisy <guillaume@dexory.com>
* SW-4837: replace the use of ros service to retrieve sensor metadata with latched topics (#102)
  * Working port of latched metadata topic on ros2
  * Update replay and record launch files to providing metadata file an optional parameter
  * Remove extra white space in replay record command
  * Undo changes to the metadata-qos-override
  * minor code syntax improvements
  * Add missing metadata topic when bag file isn't specified
  * Use concise syntax and formatting
  * Reverse logic for easier read
  * Apply node transition if it exists
* Explicity set cxx compile standard if the env isn't (#99)
* SW-4747: update the ros 2 driver(s) to the 20230403 sdk release (#94)
  * Update to the latest ouster sdk
  * Forward multicast funcitonality + Other improvements and fixes
  * Add service_msgs dependency to package.xml
  * Correct sensor_mtp.launch for ros2 launch file format
  * Move to most recent SDK update
  * Declare and fill defaults for mtp paramters + fix uninitialzed compute_to_scan
  * Launch file rename and README corrections
* Remove the duplicate sensor_info object
* Merge pull request #51 from ouster-lidar/SW-4342-prototype-ros-2-driver
  ROS2 driver MVP (beta release)
* more sensor configuration change handling
* Correct the logic around the detection of init_id change
* Wire set_config service call into node lifecycle
* Address typos and as to install rviz2 instead
* Address potential vulnerability when saving metadata to file
* Restructure reset operation on init_id change
* Use initialization list when constructing std::atomic
* When non legacy lidar profile is in use sniff lidar packes and perform self reset on init_id change event
* Add a reset service to sensor node and cycle the node upon invocation
* Uodate readme title and cleanup parameters yaml
* Add specific ros2 installation and usage instructions +
  other refactor and corrections
* code formatting
* Fix a bug caused by the type of point_cloud msg
* correct lidar_scan rename
* Code refactor and formatting
* Implement node lifecycle management for sensor node +
  refactoring launch files +
  rivz launch arg for overriding default config
* move ouster-sdk to a subfolder of ouster-ros
* More build fixes
* Add libtins as a dependency and flush out changelog
* Completely stripout topic_tools
* fix docker build and target supported distros
* launch file cleanup
* Remove the extra '
* Add a note about missing support of parameterize ros namespace when using launch.py
* Restore scoping ouster_ros nodes and topics to a configurable namesapce when using xml launch format
* Address the issue of missing sensor frame and/or old tf data when launching rviz2 from same launch file +
  correct sensor name in rviz
* Add some level of robustness around invoking the get_metadata service from processing nodes
* Re-enable replay functionality +
  address an issue where the os_replay node lose fields when load the metadata from file.
* Restore recording functionality
* Formulate the launch files in xml format +
  Rename separate to independent
* Drop setting cxx stanard in ouster_ros + refactor
* Enable running rviz from same launch file conditionally
* Base point cloud color scheme on range values
* Factor out parameters into a shared parameters.yaml file
* Utilize SensorDataQoS and add RVIZ launch file +
  code refactor
* Make connecting to get_metadata service robust +
  Drop TimerAction from the launch file
  Add ProcessingNode abstract class for os_cloud and os_image +
  Rename files to reflect the new changes
  Drop deprecated scan_to_cloud method +
  Code refactor and formatting
* Code refactor and formatting, correct msg index of 2nd cloud
* Auto generate standalone nodes from components
* Move service definition into a separate ros2 package
* Correct replay component name, replace bind with lambda
* Rename namesapce and enable replay mode
* Fix component discovery
* More refactoring and code readabilty
* Tidy up the cmake file
* Drop std_msgs
* Drop std_msgs
* ROS2 driver prototype
* Contributors: Andre Nguyen, Michael Wiznitzer, Ussama Naal, ralwing
