/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_static_transforms_broadcaster.h
 * @brief ...
 */

#pragma once

#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif

#if __has_include(<tf2_ros/version.h>)
#include <tf2_ros/version.h>
#define OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES \
    TF2_ROS_VERSION_GTE(0, 45, 0)
#else
#define OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES 0
#endif

namespace ouster_ros {

template <typename NodeT>
class OusterStaticTransformsBroadcaster {
   public:
    // tf2_ros 0.45 added RequiredInterfaces and deprecated the NodeT overload.
    explicit OusterStaticTransformsBroadcaster(NodeT& parent)
#if OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES
        : node(parent),
          tf_bcast(tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(
              parent.get_node_parameters_interface(),
              parent.get_node_topics_interface())) {}
#else
        : node(parent), tf_bcast(parent) {}
#endif

    void declare_parameters() {
        node.declare_parameter("sensor_frame", "os_sensor");
        node.declare_parameter("lidar_frame", "os_lidar");
        node.declare_parameter("imu_frame", "os_imu");
        node.declare_parameter("point_cloud_frame", "");
        node.declare_parameter("pub_static_tf", true);
    }

    void parse_parameters() {
        sensor_frame = node.get_parameter("sensor_frame").as_string();
        lidar_frame = node.get_parameter("lidar_frame").as_string();
        imu_frame = node.get_parameter("imu_frame").as_string();
        point_cloud_frame =
            node.get_parameter("point_cloud_frame").as_string();
        pub_static_tf = node.get_parameter("pub_static_tf").as_bool();

        // validate point_cloud_frame
        if (point_cloud_frame.empty()) {
            point_cloud_frame =
                lidar_frame;  // for ROS1 we'd still use sensor_frame
        } else if (point_cloud_frame != sensor_frame &&
                   point_cloud_frame != lidar_frame) {
            RCLCPP_WARN(node.get_logger(),
                        "point_cloud_frame value needs to match the value of "
                        "either sensor_frame or lidar_frame but a different "
                        "value was supplied, using lidar_frame's value as the "
                        "value for point_cloud_frame");
            point_cloud_frame = lidar_frame;
        }
    }

    void broadcast_transforms(const ouster::sdk::core::SensorInfo& info) {
        auto now = node.get_clock()->now();
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
    bool publish_static_tf() const { return pub_static_tf; }

   private:
    NodeT& node;
    tf2_ros::StaticTransformBroadcaster tf_bcast;
    std::string imu_frame;
    std::string lidar_frame;
    std::string sensor_frame;
    std::string point_cloud_frame;
    bool pub_static_tf;
};

}  // namespace ouster_ros

#undef OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES
