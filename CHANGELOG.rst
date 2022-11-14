=========
Changelog
=========

[unreleased]
============
* correct LICENSE file installation path.
* update code files copyrights period.
* bug fix: ros driver doesn't use correct udp_dest given by user during launch
* update published TF transforms time with senosr or ros time based on the
  active timestamp mode.
* breaking change: renamed ``ouster_ros/ros.h`` to ``ouster_ros/os_ros.h`` and
  ``ouster_ros/point.h`` to ``ouster_ros/os_point.h``.

[20221004]
==========

ouster_ros
----------
* Moved ouster-ros into separate repo
* Refresh the docker file

ouster_sdk
----------
* Removed ouster_ros
