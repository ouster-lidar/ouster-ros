#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_ros/os_point.h"
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"

#include "point_meta_helpers.h"
#include "point_transform.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;

template <sensor::ChanFieldType T>
struct TypeSelector { /*undefined*/ };

template <>
struct TypeSelector<sensor::ChanFieldType::UINT8> { typedef uint8_t ElementType; };

template <>
struct TypeSelector<sensor::ChanFieldType::UINT16> { typedef uint16_t ElementType; };

template <>
struct TypeSelector<sensor::ChanFieldType::UINT32> { typedef uint32_t ElementType; };

template <>
struct TypeSelector<sensor::ChanFieldType::UINT64> { typedef uint64_t ElementType; };

template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table>
constexpr auto compose_tuple_table() {
    if constexpr (Index < N) {
        using ElementType = typename TypeSelector<Table[Index].second>::ElementType;
        return std::tuple_cat(
          std::make_tuple(static_cast<const ElementType*>(0)),
          std::move(compose_tuple_table<Index + 1, N, Table>()));
    } else {
       return std::make_tuple();
    }
}

template <std::size_t Index,  std::size_t N, const ChanFieldTable<N>& Table, typename Tuple>
void map_scan_fields_to_tuple(Tuple& tuple, const ouster::LidarScan& ls) {
    if constexpr (Index < std::tuple_size<Tuple>::value) {
        using ElementType = typename TypeSelector<Table[Index].second>::ElementType;
        std::get<Index>(tuple) = ls.field<ElementType>(Table[Index].first).data();
        map_scan_fields_to_tuple<Index + 1, N, Table>(tuple, ls);
    }
}

template <std::size_t Index,  std::size_t N, const ChanFieldTable<N>& Table, typename PointT, typename Tuple>
void tuple_copy_values(PointT& point, Tuple& tuple, int src_idx) {
    if constexpr (Index < N) {
        point.template get<Index + 5>() = std::get<Index>(tuple)[src_idx];
        tuple_copy_values<Index + 1, N, Table>(point, tuple, src_idx);
    } else {
        unused_variable(src_idx);
    }
}

template <class T>
using Cloud = pcl::PointCloud<T>;

template <std::size_t N, const ChanFieldTable<N>& PROFILE, typename PointT, typename PointS>
void scan_to_cloud_f_destaggered(
    ouster_ros::Cloud<PointT>& cloud,
    PointS& staging_point,
    const ouster::PointsF& points,
    uint64_t scan_ts, const ouster::LidarScan& ls,
    const std::vector<int>& pixel_shift_by_row) {

    auto profile_tuple = compose_tuple_table<0, N, PROFILE>();
    map_scan_fields_to_tuple<0, N, PROFILE>(profile_tuple, ls);
    auto timestamp = ls.timestamp();

    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto v_shift = (v + ls.w - pixel_shift_by_row[u]) % ls.w;
            auto ts = timestamp[v_shift]; ts = ts > scan_ts ? ts - scan_ts : 0UL;
            const auto src_idx = u * ls.w + v_shift;
            const auto tgt_idx = u * ls.w + v;
            const auto xyz = points.row(src_idx);
            // if target point and staging point has matching type bind the target directly
            // and avoid performing transform_point at the end
            auto& pt = CondBinaryBind<std::is_same_v<PointT, PointS>>::run(
                cloud.points[tgt_idx], staging_point);
            // all native point types have x, y, z, t and ring values
            pt.x = static_cast<float>(xyz(0));
            pt.y = static_cast<float>(xyz(1));
            pt.z = static_cast<float>(xyz(2));
            pt.t = static_cast<uint32_t>(ts);
            pt.ring = static_cast<uint16_t>(u);
            tuple_copy_values<0, N, PROFILE>(pt, profile_tuple, src_idx);
            // only perform point transform operation when PointT, and PointS don't match
            CondBinaryOp<!std::is_same_v<PointT, PointS>>::run(cloud.points[tgt_idx],  staging_point,
                [](auto& tgt_pt, const auto& src_pt) {
                    transform_point(tgt_pt, src_pt);
                }
            );
        }
    }
}

}   // namespace ouster_ros