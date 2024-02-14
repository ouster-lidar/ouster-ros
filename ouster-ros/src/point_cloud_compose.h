#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_ros/os_point.h"
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"

#include "point_meta_helpers.h"
#include "point_transform.h"

namespace ouster_ros {

using ouster::sensor::ChanFieldType;

template <ChanFieldType T>
struct TypeSelector { /*undefined*/
};

template <>
struct TypeSelector<ChanFieldType::UINT8> {
    typedef uint8_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT16> {
    typedef uint16_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT32> {
    typedef uint32_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT64> {
    typedef uint64_t type;
};

/**
 * @brief constructs a suitable tuple at compile time that can store a reference
 * to all the fields of a specific LidarScan object (without conversion)
 * according to the information specified by the ChanFieldTable.
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table>
constexpr auto make_lidar_scan_tuple() {
    if constexpr (Index < N) {
        using ElementType = typename TypeSelector<Table[Index].second>::type;
        return std::tuple_cat(
            std::make_tuple(static_cast<const ElementType*>(0)),
            std::move(make_lidar_scan_tuple<Index + 1, N, Table>()));
    } else {
        return std::make_tuple();
    }
}

/**
 * @brief maps the fields of a LidarScan object to the elements of the supplied
 * tuple in the same order.
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table,
          typename Tuple>
void map_lidar_scan_fields_to_tuple(Tuple& tp, const ouster::LidarScan& ls) {
    static_assert(
        std::tuple_size_v<Tuple> == N,
        "target tuple size has a different size from the channel field table");
    if constexpr (Index < N) {
        using FieldType = typename TypeSelector<Table[Index].second>::type;
        using ElementType = std::remove_const_t<
            std::remove_pointer_t<std::tuple_element_t<Index, Tuple>>>;
        static_assert(std::is_same_v<ElementType, FieldType>,
                      "tuple element, field element types mismatch!");
        std::get<Index>(tp) = ls.field<FieldType>(Table[Index].first).data();
        map_lidar_scan_fields_to_tuple<Index + 1, N, Table>(tp, ls);
    }
}

/**
 * @brief constructs a suitable tuple at compile time that can store a reference
 * to all the fields of a specific LidarScan object (without conversion)
 * according to the information specified by the ChanFieldTable and directly
 * maps the fields of the supplied LidarScan to the constructed tuple before
 * returning.
 * @param[in] ls LidarScan
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table>
constexpr auto make_lidar_scan_tuple(const ouster::LidarScan& ls) {
    auto tp = make_lidar_scan_tuple<0, N, Table>();
    map_lidar_scan_fields_to_tuple<0, N, Table>(tp, ls);
    return tp;
}

/**
 * @brief copies field values from LidarScan fields combined as a tuple into the
 * the corresponding elements of the input point pt.
 * @param[out] pt point to copy values into.
 * @param[in] tp tuple containing arrays to copy LidarScan field values from.
 * @param[in] idx index of the point to be copied.
 * @remark this method is to be used mainly with sensor native point types.
 */
template <std::size_t Index, typename PointT, typename Tuple>
void copy_lidar_scan_fields_to_point(PointT& pt, const Tuple& tp, int idx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        point::get<5 + Index>(pt) = std::get<Index>(tp)[idx];
        copy_lidar_scan_fields_to_point<Index + 1>(pt, tp, idx);
    } else {
        unused_variable(pt);
        unused_variable(tp);
        unused_variable(idx);
    }
}

template <class T>
using Cloud = pcl::PointCloud<T>;

template <std::size_t N, const ChanFieldTable<N>& PROFILE, typename PointT,
          typename PointS>
void scan_to_cloud_f_destaggered(ouster_ros::Cloud<PointT>& cloud,
                                 PointS& staging_point,
                                 const ouster::PointsF& points,
                                 uint64_t scan_ts, const ouster::LidarScan& ls,
                                 const std::vector<int>& pixel_shift_by_row) {
    auto ls_tuple = make_lidar_scan_tuple<0, N, PROFILE>(ls);
    auto timestamp = ls.timestamp();

    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto v_shift = (v + ls.w - pixel_shift_by_row[u]) % ls.w;
            auto ts = timestamp[v_shift];
            ts = ts > scan_ts ? ts - scan_ts : 0UL;
            const auto src_idx = u * ls.w + v_shift;
            const auto tgt_idx = u * ls.w + v;
            const auto xyz = points.row(src_idx);
            // if target point and staging point has matching type bind the
            // target directly and avoid performing transform_point at the end
            auto& pt = CondBinaryBind<std::is_same_v<PointT, PointS>>::run(
                cloud.points[tgt_idx], staging_point);
            // all native point types have x, y, z, t and ring values
            pt.x = static_cast<decltype(pt.x)>(xyz(0));
            pt.y = static_cast<decltype(pt.y)>(xyz(1));
            pt.z = static_cast<decltype(pt.z)>(xyz(2));
            // TODO: in the future we could probably skip copying t and ring
            // values if knowing before hand that the target point cloud does
            // not have a field to hold the timestamp or a ring for example the
            // case of pcl::PointXYZ or pcl::PointXYZI.
            pt.t = static_cast<uint32_t>(ts);
            pt.ring = static_cast<uint16_t>(u);
            copy_lidar_scan_fields_to_point<0>(pt, ls_tuple, src_idx);
            // only perform point transform operation when PointT, and PointS
            // don't match
            CondBinaryOp<!std::is_same_v<PointT, PointS>>::run(
                cloud.points[tgt_idx], staging_point,
                [](auto& tgt_pt, const auto& src_pt) {
                    point::transform(tgt_pt, src_pt);
                });
        }
    }
}

}  // namespace ouster_ros