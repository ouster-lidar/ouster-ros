/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_transform.h
 * @brief Implements the main transform_point method used to convert point from
 * a source pcl point format usually sensor native point representation to other
 * pcl point formats such as Velodyne XYZIR or pcl::XYZ, pcl::XYZI, ... 
 */

#pragma once

#include "point_meta_helpers.h"

namespace ouster_ros {
namespace point {

DEFINE_MEMBER_CHECKER(x);
DEFINE_MEMBER_CHECKER(y);
DEFINE_MEMBER_CHECKER(z);
DEFINE_MEMBER_CHECKER(t);
DEFINE_MEMBER_CHECKER(ring);
DEFINE_MEMBER_CHECKER(intensity);
DEFINE_MEMBER_CHECKER(ambient);
DEFINE_MEMBER_CHECKER(range);
DEFINE_MEMBER_CHECKER(signal);
DEFINE_MEMBER_CHECKER(reflectivity);
DEFINE_MEMBER_CHECKER(near_ir);

template <typename PointTGT, typename PointSRC>
void transform(PointTGT& tgt_pt, const PointSRC& src_pt) {
    // NOTE: for now we assume all points have xyz component
    tgt_pt.x = src_pt.x; tgt_pt.y = src_pt.y; tgt_pt.z = src_pt.z;

    // t: timestamp
    CondBinaryOp<has_t_v<PointTGT> && has_t_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) { tgt_pt.t = src_pt.t; }
    );

    CondBinaryOp<has_t_v<PointTGT> && !has_t_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) { tgt_pt.t = 0U; }
    );

    // ring
    CondBinaryOp<has_ring_v<PointTGT> && has_ring_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) { tgt_pt.ring = src_pt.ring; }
    );

    CondBinaryOp<has_ring_v<PointTGT> && !has_ring_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) {
            tgt_pt.ring = static_cast<decltype(tgt_pt.ring)>(0);
        }
    );

    // range
    CondBinaryOp<has_range_v<PointTGT> && has_range_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) { tgt_pt.range = src_pt.range; }
    );

    CondBinaryOp<has_range_v<PointTGT> && !has_range_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) { tgt_pt.range = 0U; }
    );

    // signal
    CondBinaryOp<has_signal_v<PointTGT> && has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) { tgt_pt.signal = src_pt.signal; }
    );

    CondBinaryOp<has_signal_v<PointTGT> && !has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) {
            tgt_pt.signal = static_cast<decltype(tgt_pt.signal)>(0);
        }
    );

    // intensity <- signal
    // PointTGT should not have signal and intensity at the same time [normally]
    CondBinaryOp<has_intensity_v<PointTGT> && has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) {
            tgt_pt.intensity = static_cast<decltype(tgt_pt.intensity)>(src_pt.signal);
        }
    );

    CondBinaryOp<has_intensity_v<PointTGT> && !has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) {
            tgt_pt.intensity = static_cast<decltype(tgt_pt.intensity)>(0);
        }
    );

    // reflectivity
    CondBinaryOp<has_reflectivity_v<PointTGT> && has_reflectivity_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) {
            tgt_pt.reflectivity = src_pt.reflectivity;
        }
    );

    CondBinaryOp<has_reflectivity_v<PointTGT> && !has_reflectivity_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) {
            tgt_pt.reflectivity = static_cast<decltype(tgt_pt.reflectivity)>(0);
        }
    );

    // near_ir
    CondBinaryOp<has_near_ir_v<PointTGT> && has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto& src_pt) { tgt_pt.near_ir = src_pt.near_ir; }
    );

    CondBinaryOp<has_near_ir_v<PointTGT> && !has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](auto& tgt_pt, const auto&) {
            tgt_pt.near_ir = static_cast<decltype(tgt_pt.near_ir)>(0); }
    );

    // ambient <- near_ir
    CondBinaryOp<has_ambient_v<PointTGT> && has_near_ir_v<PointSRC>>::run(tgt_pt, src_pt,
        [](auto& tgt_pt, const auto& src_pt) {
            tgt_pt.ambient = static_cast<decltype(tgt_pt.ambient)>(src_pt.near_ir);
        }
    );

    CondBinaryOp<has_ambient_v<PointTGT> && !has_near_ir_v<PointSRC>>::run(tgt_pt, src_pt,
        [](auto& tgt_pt, const auto&) {
            tgt_pt.ambient = static_cast<decltype(tgt_pt.ambient)>(0);
        }
    );
}

}   // point
}   // ouster_ros