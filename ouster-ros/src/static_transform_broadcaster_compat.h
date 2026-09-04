// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>

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
std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
make_static_transform_broadcaster(NodeT* node) {
#if OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES
    return std::make_shared<tf2_ros::StaticTransformBroadcaster>(
        tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(
            node->get_node_parameters_interface(),
            node->get_node_topics_interface()));
#else
    return std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
#endif
}

}  // namespace ouster_ros

#undef OUSTER_ROS_TF2_HAS_REQUIRED_INTERFACES
