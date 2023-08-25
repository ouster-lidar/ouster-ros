/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_tf_publisher.h
 * @brief ...
 */

#pragma once

#include <tf2_ros/static_transform_broadcaster.h>

namespace ouster_ros {

template <typename NodeT>
class OusterStaticTransformsBroadcaster {
   public:
    OusterStaticTransformsBroadcaster(NodeT* parent)
        : node(parent), tf_bcast(parent) {}

    void declare_parameters() {
        node->declare_parameter("sensor_frame", "os_sensor");
        node->declare_parameter("lidar_frame", "os_lidar");
        node->declare_parameter("imu_frame", "os_imu");
        node->declare_parameter("point_cloud_frame", "");
    }

    void parse_parameters() {
        sensor_frame = node->get_parameter("sensor_frame").as_string();
        lidar_frame = node->get_parameter("lidar_frame").as_string();
        imu_frame = node->get_parameter("imu_frame").as_string();
        point_cloud_frame =
            node->get_parameter("point_cloud_frame").as_string();

        // validate point_cloud_frame
        if (point_cloud_frame.empty()) {
            point_cloud_frame =
                lidar_frame;  // for ROS1 we'd still use sensor_frame
        } else if (point_cloud_frame != sensor_frame &&
                   point_cloud_frame != lidar_frame) {
            RCLCPP_WARN(node->get_logger(),
                        "point_cloud_frame value needs to match the value of "
                        "either sensor_frame or lidar_frame but a different "
                        "value was supplied, using lidar_frame's value as the "
                        "value for point_cloud_frame");
            point_cloud_frame = lidar_frame;
        }
    }

    void broadcast_transforms(const sensor::sensor_info& info) {
        auto now = node->get_clock()->now();
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now));
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now));
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

   private:
    NodeT* node;
    tf2_ros::StaticTransformBroadcaster tf_bcast;
    std::string imu_frame;
    std::string lidar_frame;
    std::string sensor_frame;
    std::string point_cloud_frame;
};

}  // namespace ouster_ros
