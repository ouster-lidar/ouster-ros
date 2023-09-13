/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_point.h
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once

#include <pcl/point_types.h>

#include <Eigen/Core>
#include <chrono>

#include <ouster/lidar_scan.h>

namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
)

// clang-format on
