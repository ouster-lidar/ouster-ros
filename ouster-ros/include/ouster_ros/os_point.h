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

// DEFAULT[LEGACY]
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;        // equivalent signal
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;          // equivalent to channel
    uint16_t ambient;       // equivalent to near_ir
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*
 * The following are one-to-one mapping of pcl point representatios that could
 * fit the data sent by the sensor (excluding LEGACY):
 */
// auto=RNG15_RFL8_NIR8 aka LOW_DATA profile
struct EIGEN_ALIGN16 Point_RNG15_RFL8_NIR8 {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp relative to frame
    uint16_t ring;          // equivalent to channel
    uint32_t range;         // decoded_size=uint32
    uint16_t reflectivity;  // decoded_size=uint16
    uint16_t near_ir;       // equivalent to ambient; decoded_size=uint16
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// auto=RNG19_RFL8_SIG16_NIR16
struct EIGEN_ALIGN16 Point_RNG19_RFL8_SIG16_NIR16 {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp relative to frame start time
    uint16_t ring;          // equivalent channel
    uint32_t range;         // decoded_size=uint32
    uint16_t signal;        // equivalent to intensity
    uint16_t reflectivity;  // decoded_size=uint16
    uint16_t near_ir;       // equivalent to ambient; decoded_size=uint16
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// auto=RNG19_RFL8_SIG16_NIR16_DUAL
struct EIGEN_ALIGN16 Point_RNG19_RFL8_SIG16_NIR16_DUAL {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp relative to frame start time
    uint16_t ring;          // equivalent channel
    uint32_t range;         // size(decoded(range))=uint32
    uint16_t signal;        // equivalent to intensity
    uint8_t reflectivity;   // size(decoded(reflectivity))=uint8 << only in DUAL
    uint16_t near_ir;       // equivalent to ambient; decoded_size=uint16
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/* The following are pcl point representations that are common/standard point
   representation that we readily support.
 */

/*
 * Same as Velodyne point cloud type
 * @remark XYZIR point type is not compatible with RNG15_RFL8_NIR8/LOW_DATA
 * udp lidar profile.
 */
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

// Default=RNG15_RFL8_NIR8 aka LOW_DATA profile
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG15_RFL8_NIR8,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG19_RFL8_SIG16_NIR16,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint16_t, signal, signal)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_DUAL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint16_t, signal, signal)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
)

/* common point types */
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
)

// clang-format on
