#include <gtest/gtest.h>

#include <random>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster_ros/os_point.h"
#include "../src/point_cloud_compose.h"
#include "../src/point_meta_helpers.h"

using namespace ouster_ros;
using namespace ouster_ros::point;

class PointTransformTest : public ::testing::Test {
    template <std::size_t N, typename PointT>
    void initialize_point_elements_with_randoms(PointT& pt) {
        std::default_random_engine g;
        std::uniform_real_distribution<double> d(0.0, 128.0);
        point::apply<0, N>(pt, [&g, &d](auto& value) {
            using value_type = std::remove_reference_t<decltype(value)>;
            value = static_cast<value_type>(d(g));
        });
    }

   protected:
    void SetUp() override {
        // pcl + velodyne point types
        initialize_point_elements_with_randoms<point::size(pt_xyz)>(pt_xyz);
        initialize_point_elements_with_randoms<point::size(pt_xyzi)>(pt_xyzi);
        initialize_point_elements_with_randoms<point::size(pt_xyzir)>(pt_xyzir);
        // native sensor point types
        initialize_point_elements_with_randoms<point::size(pt_legacy)>(
            pt_legacy);
        initialize_point_elements_with_randoms<point::size(
            pt_rg19_rf8_sg16_nr16_dual)>(pt_rg19_rf8_sg16_nr16_dual);
        initialize_point_elements_with_randoms<point::size(
            pt_rg19_rf8_sg16_nr16)>(pt_rg19_rf8_sg16_nr16);
        initialize_point_elements_with_randoms<point::size(pt_rg15_rfl8_nr8)>(
            pt_rg15_rfl8_nr8);
        // ouster_ros original/legacy point type
        initialize_point_elements_with_randoms<point::size(pt_os_point)>(
            pt_os_point);
    }

    void TearDown() override {}

    // pcl & velodyne point types
    static pcl::PointXYZ pt_xyz;
    static pcl::PointXYZI pt_xyzi;
    static PointXYZIR pt_xyzir;
    // native point types
    static Point_LEGACY pt_legacy;
    static Point_RNG19_RFL8_SIG16_NIR16_DUAL pt_rg19_rf8_sg16_nr16_dual;
    static Point_RNG19_RFL8_SIG16_NIR16 pt_rg19_rf8_sg16_nr16;
    static Point_RNG15_RFL8_NIR8 pt_rg15_rfl8_nr8;
    // ouster_ros original/legacy point (not to be confused with Point_LEGACY)
    static ouster_ros::Point pt_os_point;
};

// pcl & velodyne point types
pcl::PointXYZ PointTransformTest::pt_xyz;
pcl::PointXYZI PointTransformTest::pt_xyzi;
PointXYZIR PointTransformTest::pt_xyzir;
// native point types
Point_LEGACY PointTransformTest::pt_legacy;
Point_RNG19_RFL8_SIG16_NIR16_DUAL
    PointTransformTest::pt_rg19_rf8_sg16_nr16_dual;
Point_RNG19_RFL8_SIG16_NIR16 PointTransformTest::pt_rg19_rf8_sg16_nr16;
Point_RNG15_RFL8_NIR8 PointTransformTest::pt_rg15_rfl8_nr8;
// ouster_ros original/legacy point (not to be confused with Point_LEGACY)
ouster_ros::Point PointTransformTest::pt_os_point;

template <typename PointT, typename PointU>
void expect_points_xyz_equal(PointT& point1, PointU& point2) {
    EXPECT_EQ(point1.x, point2.x);
    EXPECT_EQ(point1.y, point2.y);
    EXPECT_EQ(point1.z, point2.z);
}

template <typename PointTGT, typename PointSRC>
void verify_point_transform(PointTGT& tgt_pt, const PointSRC& src_pt) {
    // t: timestamp
    CondBinaryOp<has_t_v<PointTGT> && has_t_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.t, src_pt.t);
        });

    CondBinaryOp<has_t_v<PointTGT> && !has_t_v<PointSRC>>::run(
        tgt_pt, src_pt,
        [](const auto& tgt_pt, const auto&) { EXPECT_EQ(tgt_pt.t, 0U); });

    // ring
    CondBinaryOp<has_ring_v<PointTGT> && has_ring_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.ring, src_pt.ring);
        });

    CondBinaryOp<has_ring_v<PointTGT> && !has_ring_v<PointSRC>>::run(
        tgt_pt, src_pt,
        [](const auto& tgt_pt, const auto&) { EXPECT_EQ(tgt_pt.ring, 0U); });

    // range
    CondBinaryOp<has_range_v<PointTGT> && has_range_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.range, src_pt.range);
        });

    CondBinaryOp<has_range_v<PointTGT> && !has_range_v<PointSRC>>::run(
        tgt_pt, src_pt,
        [](const auto& tgt_pt, const auto&) { EXPECT_EQ(tgt_pt.range, 0U); });

    // signal
    CondBinaryOp<has_signal_v<PointTGT> && has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.signal, src_pt.signal);
        });

    CondBinaryOp<has_signal_v<PointTGT> && !has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto&) {
            EXPECT_EQ(tgt_pt.signal, static_cast<decltype(tgt_pt.signal)>(0));
        });

    // intensity <- signal
    CondBinaryOp<has_intensity_v<PointTGT> && has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.intensity,
                      static_cast<decltype(tgt_pt.intensity)>(src_pt.signal));
        });

    CondBinaryOp<has_intensity_v<PointTGT> && !has_signal_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto&) {
            EXPECT_EQ(tgt_pt.intensity,
                      static_cast<decltype(tgt_pt.intensity)>(0));
        });

    // reflectivity
    CondBinaryOp<has_reflectivity_v<PointTGT> && has_reflectivity_v<PointSRC>>::
        run(tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.reflectivity, src_pt.reflectivity);
        });

    CondBinaryOp<has_reflectivity_v<PointTGT> &&
                 !has_reflectivity_v<PointSRC>>::
        run(tgt_pt, src_pt, [](const auto& tgt_pt, const auto&) {
            EXPECT_EQ(tgt_pt.reflectivity,
                      static_cast<decltype(tgt_pt.reflectivity)>(0));
        });

    // near_ir
    CondBinaryOp<has_near_ir_v<PointTGT> && has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.near_ir, src_pt.near_ir);
        });

    CondBinaryOp<has_near_ir_v<PointTGT> && has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto&) {
            EXPECT_EQ(tgt_pt.near_ir, static_cast<decltype(tgt_pt.near_ir)>(0));
        });

    // ambient <- near_ir
    CondBinaryOp<has_ambient_v<PointTGT> && has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto& src_pt) {
            EXPECT_EQ(tgt_pt.ambient,
                      static_cast<decltype(tgt_pt.ambient)>(src_pt.near_ir));
        });

    CondBinaryOp<has_ambient_v<PointTGT> && !has_near_ir_v<PointSRC>>::run(
        tgt_pt, src_pt, [](const auto& tgt_pt, const auto&) {
            EXPECT_EQ(tgt_pt.ambient, static_cast<decltype(tgt_pt.ambient)>(0));
        });
}

// lambda function to check that point elements other than xyz have been zeroed
template <std::size_t N, typename PointT>
void expect_point_fields_zeros(PointT& pt) {
    // starting at 3 to skip xyz
    point::apply<3, N>(pt, [](auto value) {
        EXPECT_EQ(value, static_cast<decltype(value)>(0));
    });
};

TEST_F(PointTransformTest, ExpectPointFieldZeroed) {
    point::transform(pt_xyzi, pt_xyz);
    expect_points_xyz_equal(pt_xyzi, pt_xyz);
    expect_point_fields_zeros<point::size(pt_xyzi)>(pt_xyzi);

    point::transform(pt_xyzir, pt_xyz);
    expect_points_xyz_equal(pt_xyzir, pt_xyz);
    expect_point_fields_zeros<point::size(pt_xyzir)>(pt_xyzir);

    point::transform(pt_legacy, pt_xyz);
    expect_points_xyz_equal(pt_legacy, pt_xyz);
    expect_point_fields_zeros<point::size(pt_legacy)>(pt_legacy);

    point::transform(pt_rg19_rf8_sg16_nr16_dual, pt_xyz);
    expect_points_xyz_equal(pt_rg19_rf8_sg16_nr16_dual, pt_xyz);
    expect_point_fields_zeros<point::size(pt_rg19_rf8_sg16_nr16_dual)>(
        pt_rg19_rf8_sg16_nr16_dual);

    point::transform(pt_rg19_rf8_sg16_nr16, pt_xyz);
    expect_points_xyz_equal(pt_rg19_rf8_sg16_nr16, pt_xyz);
    expect_point_fields_zeros<point::size(pt_rg19_rf8_sg16_nr16)>(
        pt_rg19_rf8_sg16_nr16);

    point::transform(pt_rg15_rfl8_nr8, pt_xyz);
    expect_points_xyz_equal(pt_rg15_rfl8_nr8, pt_xyz);
    expect_point_fields_zeros<point::size(pt_rg15_rfl8_nr8)>(pt_rg15_rfl8_nr8);

    point::transform(pt_os_point, pt_xyz);
    expect_points_xyz_equal(pt_os_point, pt_xyz);
    expect_point_fields_zeros<point::size(pt_os_point)>(pt_os_point);
}

TEST_F(PointTransformTest, TestTransformReduce_LEGACY) {
    point::transform(pt_xyz, pt_legacy);
    expect_points_xyz_equal(pt_xyz, pt_legacy);
    verify_point_transform(pt_xyz, pt_legacy);

    point::transform(pt_xyzi, pt_legacy);
    expect_points_xyz_equal(pt_xyzi, pt_legacy);
    verify_point_transform(pt_xyzi, pt_legacy);

    point::transform(pt_xyzir, pt_legacy);
    expect_points_xyz_equal(pt_xyzir, pt_legacy);
    verify_point_transform(pt_xyzir, pt_legacy);
}

TEST_F(PointTransformTest, TestTransformReduce_RNG19_RFL8_SIG16_NIR16_DUAL) {
    point::transform(pt_xyz, pt_rg19_rf8_sg16_nr16_dual);
    expect_points_xyz_equal(pt_xyz, pt_rg19_rf8_sg16_nr16_dual);
    verify_point_transform(pt_xyz, pt_rg19_rf8_sg16_nr16_dual);

    point::transform(pt_xyzi, pt_rg19_rf8_sg16_nr16_dual);
    expect_points_xyz_equal(pt_xyzi, pt_rg19_rf8_sg16_nr16_dual);
    verify_point_transform(pt_xyzi, pt_rg19_rf8_sg16_nr16_dual);

    point::transform(pt_xyzir, pt_rg19_rf8_sg16_nr16_dual);
    expect_points_xyz_equal(pt_xyzir, pt_rg19_rf8_sg16_nr16_dual);
    verify_point_transform(pt_xyzir, pt_rg19_rf8_sg16_nr16_dual);
}

TEST_F(PointTransformTest, TestTransformReduce_RNG19_RFL8_SIG16_NIR16) {
    point::transform(pt_xyz, pt_rg19_rf8_sg16_nr16);
    expect_points_xyz_equal(pt_xyz, pt_rg19_rf8_sg16_nr16);
    verify_point_transform(pt_xyz, pt_rg19_rf8_sg16_nr16);

    point::transform(pt_xyzi, pt_rg19_rf8_sg16_nr16);
    expect_points_xyz_equal(pt_xyzi, pt_rg19_rf8_sg16_nr16);
    verify_point_transform(pt_xyzi, pt_rg19_rf8_sg16_nr16);

    point::transform(pt_xyzir, pt_rg19_rf8_sg16_nr16);
    expect_points_xyz_equal(pt_xyzir, pt_rg19_rf8_sg16_nr16);
    verify_point_transform(pt_xyzir, pt_rg19_rf8_sg16_nr16);
}

TEST_F(PointTransformTest, TestTransformReduce_RNG15_RFL8_NIR8) {
    point::transform(pt_xyz, pt_rg15_rfl8_nr8);
    expect_points_xyz_equal(pt_xyz, pt_rg15_rfl8_nr8);
    verify_point_transform(pt_xyz, pt_rg15_rfl8_nr8);

    point::transform(pt_xyzi, pt_rg15_rfl8_nr8);
    expect_points_xyz_equal(pt_xyzi, pt_rg15_rfl8_nr8);
    verify_point_transform(pt_xyzi, pt_rg15_rfl8_nr8);

    point::transform(pt_xyzir, pt_rg15_rfl8_nr8);
    expect_points_xyz_equal(pt_xyzir, pt_rg15_rfl8_nr8);
    verify_point_transform(pt_xyzir, pt_rg15_rfl8_nr8);
}

TEST_F(PointTransformTest,
       TestTransform_SensorNativePoints_2_ouster_ros_Point) {
    point::transform(pt_os_point, pt_legacy);
    expect_points_xyz_equal(pt_os_point, pt_legacy);
    verify_point_transform(pt_os_point, pt_legacy);

    point::transform(pt_os_point, pt_rg19_rf8_sg16_nr16_dual);
    expect_points_xyz_equal(pt_os_point, pt_rg19_rf8_sg16_nr16_dual);
    verify_point_transform(pt_os_point, pt_rg19_rf8_sg16_nr16_dual);

    point::transform(pt_os_point, pt_rg19_rf8_sg16_nr16);
    expect_points_xyz_equal(pt_os_point, pt_rg19_rf8_sg16_nr16);
    verify_point_transform(pt_os_point, pt_rg19_rf8_sg16_nr16);

    point::transform(pt_os_point, pt_rg15_rfl8_nr8);
    expect_points_xyz_equal(pt_os_point, pt_rg15_rfl8_nr8);
    verify_point_transform(pt_os_point, pt_rg15_rfl8_nr8);
}