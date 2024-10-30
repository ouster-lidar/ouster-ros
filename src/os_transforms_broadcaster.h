/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_tf_publisher.h
 * @brief ...
 */

#pragma once

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nodelet/nodelet.h>

namespace ouster_ros {

class OusterTransformsBroadcaster {
   public:
    OusterTransformsBroadcaster(const std::string& parent_name)
        : node_name(parent_name) {}

   private:
    bool is_arg_set(const std::string& arg) const {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    const std::string& getName() const { return node_name; }

   public:
    void parse_parameters(ros::NodeHandle& pnh) {
        sensor_frame = pnh.param("sensor_frame", std::string{"os_sensor"});
        lidar_frame = pnh.param("lidar_frame", std::string{"os_lidar"});
        imu_frame = pnh.param("imu_frame", std::string{"os_imu"});
        point_cloud_frame = pnh.param("point_cloud_frame", std::string{});
        pub_static_tf = pnh.param("pub_static_tf", true);

        if (!is_arg_set(sensor_frame) || !is_arg_set(lidar_frame) ||
            !is_arg_set(imu_frame)) {
            NODELET_ERROR(
                "sensor_frame, lidar_frame, imu_frame parameters can not be "
                "empty");
        }

        // validate point_cloud_frame
        if (!is_arg_set(point_cloud_frame)) {
            // for ROS1 we'd still default to sensor_frame
            point_cloud_frame = sensor_frame;
        } else if (point_cloud_frame != sensor_frame &&
                   point_cloud_frame != lidar_frame) {
            NODELET_WARN(
                "point_cloud_frame value needs to match the value of either"
                "sensor_frame or lidar_frame but a different value was supplied"
                ", using lidar_frame's value as the value for "
                "point_cloud_frame");
            point_cloud_frame = lidar_frame;
        }

        if (point_cloud_frame == sensor_frame) {
            NODELET_WARN(
                "point_cloud_frame is set to sensor_frame. If you need  to "
                "reproject or raytrace the points then use the lidar_frame "
                "instead. Refer to: https://github.com/ouster-lidar/ouster-ros"
                "/issues/33 for more details.");
        }

        // Finally prepend tf_prefix if it was set
        auto tf_prefix = pnh.param("tf_prefix", std::string{});
        if (is_arg_set(tf_prefix)) {
            if (tf_prefix.back() != '/') tf_prefix.append("/");
            sensor_frame = tf_prefix + sensor_frame;
            lidar_frame = tf_prefix + lidar_frame;
            imu_frame = tf_prefix + imu_frame;
            point_cloud_frame = tf_prefix + point_cloud_frame;
        }
    }

    void broadcast_transforms(const sensor::sensor_info& info) {
        auto now = ros::Time::now();
        static_tf_bcast.sendTransform(transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now));
        static_tf_bcast.sendTransform(transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now));
    }

    void broadcast_transforms(const sensor::sensor_info& info,
                              const ros::Time& ts) {
        tf_bcast.sendTransform(transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, ts));
        tf_bcast.sendTransform(transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, ts));
    }

    const std::string& imu_frame_id() const { return imu_frame; }
    const std::string& lidar_frame_id() const { return lidar_frame; }
    const std::string& sensor_frame_id() const { return sensor_frame; }

    const std::string& point_cloud_frame_id() const {
        return point_cloud_frame;
    }
    bool apply_lidar_to_sensor_transform() const {
        return point_cloud_frame == sensor_frame;
    }

    bool publish_static_tf() const { return pub_static_tf; }

   private:
    std::string node_name;
    tf2_ros::StaticTransformBroadcaster static_tf_bcast;
    tf2_ros::TransformBroadcaster tf_bcast;  // non-static
    std::string imu_frame;
    std::string lidar_frame;
    std::string sensor_frame;
    std::string point_cloud_frame;
    bool pub_static_tf;
};

}  // namespace ouster_ros
