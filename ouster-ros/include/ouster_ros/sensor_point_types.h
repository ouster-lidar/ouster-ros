/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_point_types.h
 * @brief native sensor point types
 * The following are one-to-one mapping of pcl point representatios that could
 * fit the data sent by the sensor (with the addition of t and ring fields),
 * All of these representation follow the same order of LaserScan fields:
 * 1. range
 * 2. signal
 * 3. refelctivity
 * 4. near_ir
 * With the exception Point_RNG15_RFL8_NIR8 aka LOW_DATA which does not include
 * the signal field
**/

#pragma once

#include <pcl/point_types.h>
#include <ouster/lidar_scan.h>

namespace ouster_ros {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

namespace ChanField = ouster::sdk::core::ChanField;
using ouster::sdk::core::ChanFieldType;

template <size_t N>
using ChanFieldTable = Table<const char*, ChanFieldType, N>;

}


namespace ouster_ros {

// Profile_LEGACY
static constexpr ChanFieldTable<5> Profile_LEGACY{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8}
}};

// auto=LEGACY
struct EIGEN_ALIGN16 _Point_LEGACY {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint16_t signal;        // equivalent to intensity
    uint8_t reflectivity;
    uint16_t near_ir;       // equivalent to ambient
    uint8_t flags;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point_LEGACY : public _Point_LEGACY {

    inline Point_LEGACY(const _Point_LEGACY& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range; signal = pt.signal;
      reflectivity = pt.reflectivity; near_ir = pt.near_ir;
      flags = pt.flags;
    }

    inline Point_LEGACY()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0; signal = 0;
      reflectivity = 0; near_ir = 0;
      flags = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, near_ir, flags);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, near_ir, flags);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_LEGACY,
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

namespace ouster_ros {

static const Table<std::string, ChanFieldType, 10> DUAL_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

// Profile_RNG19_RFL8_SIG16_NIR16_DUAL: aka dual returns
// This profile is definied differently from RNG19_RFL8_SIG16_NIR16_DUAL of how
// the sensor actually sends the data. The actual RNG19_RFL8_SIG16_NIR16_DUAL
// has 7 fields not 4, but this profile is defined differently in ROS because
// we build and publish a point cloud for each return separately. However, it
// might be desireable to some of the users to choose a point cloud
// representation which combines parts of the the two or more returns. This isn't
// something that the current framework could deal with as of now.
static constexpr ChanFieldTable<6> Profile_RNG19_RFL8_SIG16_NIR16_DUAL {{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

// Note: this is one way to implement the processing of 2nd return
// This should be an exact copy of Profile_RNG19_RFL8_SIG16_NIR16_DUAL with the
// exception of ChanField values for the first three fields. NEAR_IR is same for both
static constexpr ChanFieldTable<6> Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN {{
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

// auto=RNG19_RFL8_SIG16_NIR16_DUAL
struct EIGEN_ALIGN16 _Point_RNG19_RFL8_SIG16_NIR16_DUAL {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent channel
    uint32_t range;
    uint16_t signal;        // equivalent to intensity
    uint8_t reflectivity;
    uint16_t near_ir;       // equivalent to ambient
    uint8_t flags;
    uint8_t window;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point_RNG19_RFL8_SIG16_NIR16_DUAL : public _Point_RNG19_RFL8_SIG16_NIR16_DUAL {

    inline Point_RNG19_RFL8_SIG16_NIR16_DUAL(const _Point_RNG19_RFL8_SIG16_NIR16_DUAL& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range; signal = pt.signal;
      reflectivity = pt.reflectivity; near_ir = pt.near_ir;
      flags = pt.flags; window = pt.window;
    }

    inline Point_RNG19_RFL8_SIG16_NIR16_DUAL()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0; signal = 0;
      reflectivity = 0; near_ir = 0;
      flags = 0; window = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, near_ir, flags, window);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, near_ir, flags, window);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespce ouster_ros

// clang-format off

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
    (std::uint8_t, flags, flags)
    (std::uint8_t, window, window)
)

// clang-format on

namespace ouster_ros {

// Profile_RNG19_RFL8_SIG16_NIR16 aka single return
static constexpr ChanFieldTable<6> Profile_RNG19_RFL8_SIG16_NIR16{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

// auto=RNG19_RFL8_SIG16_NIR16
struct EIGEN_ALIGN16 _Point_RNG19_RFL8_SIG16_NIR16 {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent channel
    uint32_t range;
    uint16_t signal;        // equivalent to intensity
    uint8_t reflectivity;
    uint8_t flags;
    uint16_t near_ir;       // equivalent to ambient
    uint8_t window;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point_RNG19_RFL8_SIG16_NIR16 : public _Point_RNG19_RFL8_SIG16_NIR16 {

    inline Point_RNG19_RFL8_SIG16_NIR16(const _Point_RNG19_RFL8_SIG16_NIR16& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range; signal = pt.signal;
      reflectivity = pt.reflectivity; near_ir = pt.near_ir;
      flags = pt.flags; window = pt.window;
    }

    inline Point_RNG19_RFL8_SIG16_NIR16()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0; signal = 0;
      reflectivity = 0; near_ir = 0;
      flags = 0; window = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, flags, near_ir, window);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, flags, near_ir, window);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG19_RFL8_SIG16_NIR16,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint16_t, signal, signal)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint8_t, flags, flags)
    (std::uint16_t, near_ir, near_ir)
    (std::uint8_t, window, window)
)

// clang-format on

namespace ouster_ros {

// Profile_RNG15_RFL8_NIR8 aka LOW_DATA
static constexpr ChanFieldTable<4> Profile_RNG15_RFL8_NIR8{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8}
}};

// auto=RNG15_RFL8_NIR8 aka LOW_DATA profile
struct EIGEN_ALIGN16 _Point_RNG15_RFL8_NIR8 {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint8_t reflectivity;
    uint16_t near_ir;       // equivalent to ambient
    uint8_t flags;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Point_RNG15_RFL8_NIR8 : public _Point_RNG15_RFL8_NIR8 {

    inline Point_RNG15_RFL8_NIR8(const _Point_RNG15_RFL8_NIR8& pt) {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range;
      reflectivity = pt.reflectivity; near_ir = pt.near_ir;
      flags = pt.flags;
    }

    inline Point_RNG15_RFL8_NIR8()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0;
      reflectivity = 0; near_ir = 0;
      flags = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, flags);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, flags);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// Default=RNG15_RFL8_NIR8 aka LOW_DATA profile
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG15_RFL8_NIR8,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
    (std::uint8_t, flags, flags)
)

// clang-format on


namespace ouster_ros {

// Profile_FUSA_RNG15_RFL8_NIR8_DUAL: aka fusa dual returns
// This profile is definied differently from PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL of how
// the sensor actually sends the data. The actual PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL
// has 8 fields not 5, but this profile is defined differently in ROS because
// we build and publish a point cloud for each return separately. However, it
// might be desireable to some of the users to choose a point cloud
// representation which combines parts of the the two or more returns. This isn't
// something that the current framework could deal with as of now.
static constexpr ChanFieldTable<5> Profile_FUSA_RNG15_RFL8_NIR8_DUAL {{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8}
}};

// Note: this is one way to implement the processing of 2nd return
// This should be an exact copy of Profile_FUSA_RNG15_RFL8_NIR8_DUAL with the
// exception of ChanField values for the first three fields. NEAR_IR is same for both
static constexpr ChanFieldTable<5> Profile_FUSA_RNG15_RFL8_NIR8_DUAL_2ND_RETURN {{
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8}
}};

// auto=RNG19_RFL8_SIG16_NIR16_DUAL
struct EIGEN_ALIGN16 _Point_FUSA_RNG15_RFL8_NIR8_DUAL {
    PCL_ADD_POINT4D;
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint8_t reflectivity;
    uint16_t near_ir;       // equivalent to ambient
    uint8_t flags;
    uint8_t window;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point_FUSA_RNG15_RFL8_NIR8_DUAL : public _Point_FUSA_RNG15_RFL8_NIR8_DUAL {

    inline Point_FUSA_RNG15_RFL8_NIR8_DUAL(const _Point_FUSA_RNG15_RFL8_NIR8_DUAL& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range;
      reflectivity = pt.reflectivity;
      near_ir = pt.near_ir;
      flags = pt.flags;
      window = pt.window;
    }

    inline Point_FUSA_RNG15_RFL8_NIR8_DUAL()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0;
      reflectivity = 0;
      near_ir = 0;
      flags = 0;
      window = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, flags, window);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, flags, window);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespce ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_FUSA_RNG15_RFL8_NIR8_DUAL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
    (std::uint8_t, flags, flags)
    (std::uint8_t, window, window)
)

// clang-format on

namespace ouster_ros {

// Profile_RNG15_RFL8_WIN8
static constexpr ChanFieldTable<4> Profile_RNG15_RFL8_WIN8{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
}};

// auto=RNG15_RFL8_WIN8 aka LOW_DATA profile
struct EIGEN_ALIGN16 _Point_RNG15_RFL8_WIN8 {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint8_t reflectivity;
    uint8_t window;
    uint8_t flags;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Point_RNG15_RFL8_WIN8 : public _Point_RNG15_RFL8_WIN8 {

    inline Point_RNG15_RFL8_WIN8(const _Point_RNG15_RFL8_WIN8& pt) {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range;
      reflectivity = pt.reflectivity; window = pt.window;
      flags = pt.flags;
    }

    inline Point_RNG15_RFL8_WIN8()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0;
      reflectivity = 0; window = 0;
      flags = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, reflectivity, window, flags);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, reflectivity, window, flags);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// Default=RNG15_RFL8_WIN8 aka LOW_DATA profile
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG15_RFL8_WIN8,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint8_t, window, window)
    (std::uint8_t, flags, flags)
)

// clang-format on


namespace ouster_ros {

// Profile_FIVE_WORD_PIXEL
static constexpr ChanFieldTable<5> Profile_FIVE_WORD_PIXEL{{
    {ChanField::RAW32_WORD1, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD2, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD3, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD4, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD5, ChanFieldType::UINT32},
}};

// auto=FIVE_WORD_PIXEL
struct EIGEN_ALIGN16 _Point_FIVE_WORD_PIXEL {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
    uint32_t word4;
    uint32_t word5;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Point_FIVE_WORD_PIXEL : public _Point_FIVE_WORD_PIXEL {

    inline Point_FIVE_WORD_PIXEL(const _Point_FIVE_WORD_PIXEL& pt) {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      word1 = pt.word1;
      word2 = pt.word2;
      word3 = pt.word3;
      word4 = pt.word4;
      word5 = pt.word5;
    }

    inline Point_FIVE_WORD_PIXEL()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      word1 = 0; word2 = 0;
      word3 = 0; word4 = 0;
      word5 = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, word1, word2, word3, word4, word5);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, word1, word2, word3, word4, word5);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// Default=FIVE_WORD_PIXEL aka LOW_DATA profile
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_FIVE_WORD_PIXEL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, word1, word1)
    (std::uint32_t, word2, word2)
    (std::uint32_t, word3, word3)
    (std::uint32_t, word4, word4)
    (std::uint32_t, word5, word5)
)

// clang-format on

namespace ouster_ros {

// Profile_RNG15_RFL8_NIR8_ZONE16
static constexpr ChanFieldTable<4> Profile_RNG15_RFL8_NIR8_ZONE16{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
}};


// auto=RNG15_RFL8_NIR8_ZONE16
struct EIGEN_ALIGN16 _Point_RNG15_RFL8_NIR8_ZONE16 {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint8_t reflectivity;
    uint16_t near_ir;       // equivalent to ambient
    uint16_t zone_mask;     // bitmask of which zones the point falls into
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Point_RNG15_RFL8_NIR8_ZONE16 : public _Point_RNG15_RFL8_NIR8_ZONE16 {

    inline Point_RNG15_RFL8_NIR8_ZONE16(const _Point_RNG15_RFL8_NIR8_ZONE16& pt) {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range;
      reflectivity = pt.reflectivity;
      near_ir = pt.near_ir;
      zone_mask = pt.zone_mask;
    }

    inline Point_RNG15_RFL8_NIR8_ZONE16()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0;
      reflectivity = 0; near_ir = 0;
      zone_mask = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, zone_mask);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, reflectivity, near_ir, zone_mask);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG15_RFL8_NIR8_ZONE16,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint16_t, near_ir, near_ir)
    (std::uint16_t, zone_mask, zone_mask)
)


namespace ouster_ros {

// Profile_RNG15_RFL8_NIR8_ZONE16
static constexpr ChanFieldTable<6> Profile_RNG19_RFL8_SIG16_NIR16_ZONE16{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
}};


// auto=RNG19_RFL8_SIG16_NIR16_ZONE16
struct EIGEN_ALIGN16 _Point_RNG19_RFL8_SIG16_NIR16_ZONE16 {
    PCL_ADD_POINT4D;
    // No signal/intensity in low data mode
    uint32_t t;             // timestamp in nanoseconds relative to frame start
    uint16_t ring;          // equivalent to channel
    uint32_t range;
    uint16_t signal;
    uint8_t reflectivity;
    uint8_t flags;
    uint16_t near_ir;       // equivalent to ambient
    uint16_t zone_mask;     // bitmask of which zones the point falls into
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Point_RNG19_RFL8_SIG16_NIR16_ZONE16 : public _Point_RNG19_RFL8_SIG16_NIR16_ZONE16 {

    inline Point_RNG19_RFL8_SIG16_NIR16_ZONE16(const _Point_RNG19_RFL8_SIG16_NIR16_ZONE16& pt) {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      t = pt.t; ring = pt.ring;
      range = pt.range;
      signal = pt.signal;
      reflectivity = pt.reflectivity;
      flags = pt.flags;
      near_ir = pt.near_ir;
      zone_mask = pt.zone_mask;
    }

    inline Point_RNG19_RFL8_SIG16_NIR16_ZONE16()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      t = 0; ring = 0;
      range = 0;
      signal = 0; reflectivity = 0; flags = 0; near_ir = 0;
      zone_mask = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, flags, near_ir, zone_mask);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, t, ring, range, signal, reflectivity, flags, near_ir, zone_mask);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_ZONE16,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint32_t, t, t)
    (std::uint16_t, ring, ring)
    (std::uint32_t, range, range)
    (std::uint16_t, signal, signal)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint8_t, flags, flags)
    (std::uint16_t, near_ir, near_ir)
    (std::uint16_t, zone_mask, zone_mask)
)

// clang-format on