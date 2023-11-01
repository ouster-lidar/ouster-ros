^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ouster_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.1 (2023-11-01)
-------------------
* breaking: rename ouster_msgs to ouster_sensor_msgs
* shutdown the driver when unable to connect to the sensor on startup


0.10.4 (2023-08-31)
-------------------
* breaking: publish PCL point clouds destaggered.
* introduced a new launch file parameter ``ptp_utc_tai_offset`` which represent offset in seconds
  to be applied to all ROS messages the driver generates when ``TIME_FROM_PTP_1588`` timestamp mode
  is used.
* fix: destagger columns timestamp when generating destaggered point clouds.
* Use the local copy of the LICENSE file during install
* Contributors: Ussama Naal

0.10.3 (2023-08-15)
-------------------
* Add per package LICENSE file
* manifest symbolic links as files
* Contributors: Ussama Naal

0.10.2 (2023-08-15)
-------------------
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
* Contributors: Ussama Naal
