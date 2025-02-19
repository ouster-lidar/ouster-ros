/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file common_point_types.h
 * @brief common PCL point datatype for use with ouster sensors
 */

#pragma once

#include <pcl/point_types.h>

namespace ouster_ros {

/* The following are pcl point representations that are common/standard point
   representation that we readily support.
 */

/*
 * Same as Apollo point cloud type
 * @remark XYZIT point type is not compatible with RNG15_RFL8_NIR8/LOW_DATA
 * udp lidar profile.
 */
struct EIGEN_ALIGN16 _Ouster_PointXYZI {
    union EIGEN_ALIGN16 {
        float data[4];
        struct {
          float x;
          float y;
          float z;
          float intensity;
        };
   };
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZI : public _Ouster_PointXYZI {

    inline PointXYZI(const _Ouster_PointXYZI& pt)
    {
      x = pt.x; y = pt.y; z = pt.z;
      intensity = pt.intensity;
    }

    inline PointXYZI()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      intensity = 0.0f;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, intensity);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, intensity);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

/*
 * Same as Velodyne point cloud type
 * @remark XYZIR point type is not compatible with RNG15_RFL8_NIR8/LOW_DATA
 * udp lidar profile.
 */
struct EIGEN_ALIGN16 _PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZIR : public _PointXYZIR {

    inline PointXYZIR(const _PointXYZIR& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      intensity = pt.intensity; ring = pt.ring;
    }

    inline PointXYZIR()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      intensity = 0.0f; ring = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, intensity, ring);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, intensity, ring);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::PointXYZI,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
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
