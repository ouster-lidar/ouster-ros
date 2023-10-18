#include <gtest/gtest.h>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster_ros/os_point.h"
#include "../src/point_meta_helpers.h"

using namespace std;
using namespace ouster_ros;

class PointAccessorTest : public ::testing::Test {
   protected:
    void SetUp() override {
        pt_xyz = pcl::_PointXYZ{0.0f, 1.0f, 2.0f, 1.0f};
        pt_xyzi = pcl::_PointXYZI{{0.0f, 1.0f, 2.0f, 1.0}, 3.0f};
        pt_xyzir = ouster_ros::_PointXYZIR{{0.0f, 1.0f, 2.0f, 1.0f}, 3.0f, 4};

        pt_legacy = ouster_ros::_Point_LEGACY{
            {0.0f, 1.0f, 2.0f, 1.0f},   // x, y, z, w
            3, 4,                       // t, ring
            5, 6, 7, 8                  // range, signal, reflectivity, near_ir
        };

        pt_rg19_rf8_sg16_nr16_dual = ouster_ros::_Point_RNG19_RFL8_SIG16_NIR16_DUAL{
            {0.0f, 1.0f, 2.0f, 1.0f},   // x, y, z, w
            3, 4,                       // t, ring
            5, 6, 7, 8                  // range, signal, reflectivity, near_ir
        };

        pt_rg19_rf8_sg16_nr16 = ouster_ros::_Point_RNG19_RFL8_SIG16_NIR16{
            {0.0f, 1.0f, 2.0f, 1.0f},   // x, y, z, w
            3, 4,                       // t, ring
            5, 6, 7, 8                  // range, signal, reflectivity, near_ir
        };

        pt_rg15_rfl8_nr8 = ouster_ros::_Point_RNG15_RFL8_NIR8{
            {0.0f, 1.0f, 2.0f, 1.0f},   // x, y, z, w
            3, 4,                       // t, ring
            5, 6, 7,                    // range, reflectivity, near_ir
        };

        pt_os_point = ouster_ros::_Point{
            {0.0f, 1.0f, 2.0f, 1.0f},   // x, y, z, w
            3.0f, 4,                    // intensity, t,
            5, 6, 7, 8                  // reflectivity, ring, ambient, range
        };
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
pcl::PointXYZ PointAccessorTest::pt_xyz;
pcl::PointXYZI PointAccessorTest::pt_xyzi;
PointXYZIR PointAccessorTest::pt_xyzir;
// native point types
Point_LEGACY PointAccessorTest::pt_legacy;
Point_RNG19_RFL8_SIG16_NIR16_DUAL PointAccessorTest::pt_rg19_rf8_sg16_nr16_dual;
Point_RNG19_RFL8_SIG16_NIR16 PointAccessorTest::pt_rg19_rf8_sg16_nr16;
Point_RNG15_RFL8_NIR8 PointAccessorTest::pt_rg15_rfl8_nr8;
// ouster_ros original/legacy point (not to be confused with Point_LEGACY)
ouster_ros::Point PointAccessorTest::pt_os_point;

TEST_F(PointAccessorTest, ElementCount) {
    // pcl & velodyne point types
    EXPECT_EQ(point::size(pt_xyz), 3U);
    EXPECT_EQ(point::size(pt_xyzi), 4U);
    EXPECT_EQ(point::size(pt_xyzir), 5U);
    // all native sensor point types has {x, y, z, t and ring} fields
    EXPECT_EQ(point::size(pt_legacy), 5 + Profile_LEGACY.size());
    EXPECT_EQ(point::size(pt_rg19_rf8_sg16_nr16_dual),
              5 + Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size());
    EXPECT_EQ(point::size(pt_rg19_rf8_sg16_nr16),
              5 + Profile_RNG19_RFL8_SIG16_NIR16.size());
    EXPECT_EQ(point::size(pt_rg15_rfl8_nr8),
              5 + Profile_RNG15_RFL8_NIR8.size());
    // ouster_ros original/legacy point type
    EXPECT_EQ(point::size(pt_os_point), 9U);
}

// This test is only concerned with pcl & velodyne point types to verify that
// point::get<I> performs as expected. Subsequent tests does the same test on
// the remaining point types but uses point::apply and point::enumerate rather
// than accessing each element individually/directly as this test case does.
TEST_F(PointAccessorTest, ElementGetSet) {
    // pcl::PointXYZ
    EXPECT_EQ(point::get<0>(pt_xyz), 0.0f);
    EXPECT_EQ(point::get<1>(pt_xyz), 1.0f);
    EXPECT_EQ(point::get<2>(pt_xyz), 2.0f);
    // pcl::PointXYZI
    EXPECT_EQ(point::get<0>(pt_xyzi), 0.0f);
    EXPECT_EQ(point::get<1>(pt_xyzi), 1.0f);
    EXPECT_EQ(point::get<2>(pt_xyzi), 2.0f);
    EXPECT_EQ(point::get<3>(pt_xyzi), 3.0f);
    // ouster_ros::PointXYZIR
    EXPECT_EQ(point::get<0>(pt_xyzir), 0.0f);
    EXPECT_EQ(point::get<1>(pt_xyzir), 1.0f);
    EXPECT_EQ(point::get<2>(pt_xyzir), 2.0f);
    EXPECT_EQ(point::get<3>(pt_xyzir), 3.0f);
    EXPECT_EQ(point::get<4>(pt_xyzir), 4);

    // pcl::PointXYZ
    point::get<0>(pt_xyz) = 10.0f;
    point::get<1>(pt_xyz) = 11.0f;
    point::get<2>(pt_xyz) = 12.0f;
    // pcl::PointXYZI
    point::get<0>(pt_xyzi) = 10.0f;
    point::get<1>(pt_xyzi) = 11.0f;
    point::get<2>(pt_xyzi) = 12.0f;
    point::get<3>(pt_xyzi) = 13.0f;
    // ouster_ros::PointXYZIR
    point::get<0>(pt_xyzir) = 10.0f;
    point::get<1>(pt_xyzir) = 11.0f;
    point::get<2>(pt_xyzir) = 12.0f;
    point::get<3>(pt_xyzir) = 13.0f;
    point::get<4>(pt_xyzir) = 14;

    // pcl::PointXYZ
    EXPECT_EQ(point::get<0>(pt_xyz), 10.0f);
    EXPECT_EQ(point::get<1>(pt_xyz), 11.0f);
    EXPECT_EQ(point::get<2>(pt_xyz), 12.0f);
    // pcl::PointXYZI
    EXPECT_EQ(point::get<0>(pt_xyzi), 10.0f);
    EXPECT_EQ(point::get<1>(pt_xyzi), 11.0f);
    EXPECT_EQ(point::get<2>(pt_xyzi), 12.0f);
    EXPECT_EQ(point::get<3>(pt_xyzi), 13.0f);
    // ouster_ros::PointXYZIR
    EXPECT_EQ(point::get<0>(pt_xyzir), 10.0f);
    EXPECT_EQ(point::get<1>(pt_xyzir), 11.0f);
    EXPECT_EQ(point::get<2>(pt_xyzir), 12.0f);
    EXPECT_EQ(point::get<3>(pt_xyzir), 13.0f);
    EXPECT_EQ(point::get<4>(pt_xyzir), 14);
}

template <std::size_t N, typename PointT>
void expect_element_equals_index(PointT& pt) {
    point::enumerate<0, N>(pt, [](auto index, auto value) {
        EXPECT_EQ(value, static_cast<decltype(value)>(index));
    });
}

TEST_F(PointAccessorTest, ExpectElementValueSameAsIndex) {
    // pcl + velodyne point types
    expect_element_equals_index<point::size(pt_xyz)>(pt_xyz);
    expect_element_equals_index<point::size(pt_xyzi)>(pt_xyzi);
    expect_element_equals_index<point::size(pt_xyzir)>(pt_xyzir);
    // native sensor point types
    expect_element_equals_index<point::size(pt_legacy)>(pt_legacy);
    expect_element_equals_index<point::size(pt_rg19_rf8_sg16_nr16_dual)>(
        pt_rg19_rf8_sg16_nr16_dual);
    expect_element_equals_index<point::size(pt_rg19_rf8_sg16_nr16)>(
        pt_rg19_rf8_sg16_nr16);
    expect_element_equals_index<point::size(pt_rg15_rfl8_nr8)>(
        pt_rg15_rfl8_nr8);
    // ouster_ros original/legacy point type
    expect_element_equals_index<point::size(pt_os_point)>(pt_os_point);
}

template <std::size_t N, typename PointT>
void increment_by_value(PointT& pt, int increment) {
    point::apply<0, N>(pt, [increment](auto& value) { value += increment; });
}

template <std::size_t N, typename PointT>
void expect_value_increased_by_value(PointT& pt, int increment) {
    point::enumerate<0, N>(pt, [increment](auto index, auto value) {
        EXPECT_EQ(value, static_cast<decltype(value)>(index + increment));
    });
};

TEST_F(PointAccessorTest, ExpectPointElementValueIncrementedByValue) {
    auto increment = 1;

    // pcl + velodyne point types
    increment_by_value<point::size(pt_xyz)>(pt_xyz, increment);
    expect_value_increased_by_value<point::size(pt_xyz)>(pt_xyz, increment);
    increment_by_value<point::size(pt_xyzi)>(pt_xyzi, increment);
    expect_value_increased_by_value<point::size(pt_xyzi)>(pt_xyzi, increment);
    increment_by_value<point::size(pt_xyzir)>(pt_xyzir, increment);
    expect_value_increased_by_value<point::size(pt_xyzir)>(pt_xyzir, increment);
    // // native sensor point types
    increment_by_value<point::size(pt_legacy)>(pt_legacy, increment);
    expect_value_increased_by_value<point::size(pt_legacy)>(pt_legacy,
                                                            increment);
    increment_by_value<point::size(pt_rg19_rf8_sg16_nr16_dual)>(
        pt_rg19_rf8_sg16_nr16_dual, increment);
    expect_value_increased_by_value<point::size(pt_rg19_rf8_sg16_nr16_dual)>(
        pt_rg19_rf8_sg16_nr16_dual, increment);
    increment_by_value<point::size(pt_rg19_rf8_sg16_nr16)>(
        pt_rg19_rf8_sg16_nr16, increment);
    expect_value_increased_by_value<point::size(pt_rg19_rf8_sg16_nr16)>(
        pt_rg19_rf8_sg16_nr16, increment);
    increment_by_value<point::size(pt_rg15_rfl8_nr8)>(pt_rg15_rfl8_nr8,
                                                      increment);
    expect_value_increased_by_value<point::size(pt_rg15_rfl8_nr8)>(
        pt_rg15_rfl8_nr8, increment);
    // // ouster_ros original/legacy point type
    increment_by_value<point::size(pt_os_point)>(pt_os_point, increment);
    expect_value_increased_by_value<point::size(pt_os_point)>(pt_os_point,
                                                              increment);
}
