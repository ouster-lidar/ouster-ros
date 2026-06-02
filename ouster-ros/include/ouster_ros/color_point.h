/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file color_point.h
 * @brief PCL point datatype matching ouster_ros::Point with RGB color.
 */

#pragma once

#include <pcl/point_types.h>

#include <tuple>

namespace ouster_ros {

// Same field layout as _Point in os_point.h, extended with PCL's packed RGB
// union. The r/g/b fields are populated by point::transform() when the source
// native point type carries RGB channels; otherwise they remain zero.
struct EIGEN_ALIGN16 _ColorPoint {
    PCL_ADD_POINT4D;
    float intensity;        // equivalent to signal
    uint32_t t;
    uint8_t reflectivity;
    uint16_t ring;          // equivalent to channel
    uint16_t ambient;       // equivalent to near_ir
    uint32_t range;
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ColorPoint : public _ColorPoint {

    inline ColorPoint(const _ColorPoint& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      intensity = pt.intensity;
      t = pt.t;
      reflectivity = pt.reflectivity;
      ring = pt.ring;
      ambient = pt.ambient;
      range = pt.range;
      r = pt.r; g = pt.g; b = pt.b;
    }

    inline ColorPoint()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      intensity = 0.0f;
      t = 0;
      reflectivity = 0;
      ring = 0;
      ambient = 0;
      range = 0;
      rgb = 0;
    }

    inline auto as_tuple() const {
        return std::tie(x, y, z, intensity, t, reflectivity, ring, ambient,
                        range, r, g, b);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, intensity, t, reflectivity, ring, ambient,
                        range, r, g, b);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// COLOR_POINT: same fields as ouster_ros::Point plus packed RGB color.
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::ColorPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
    (std::uint32_t, rgb, rgb)
)

// clang-format on
