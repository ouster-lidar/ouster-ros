#include <gtest/gtest.h>

#include "ouster_ros/os_ros.h"
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster_ros/os_point.h"
#include "../src/point_meta_helpers.h"

using namespace ouster_ros;
using namespace std;

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

    void TearDown() override {
    }

    // pcl & velodyne point types
    pcl::PointXYZ pt_xyz;
    pcl::PointXYZI pt_xyzi;
    PointXYZIR pt_xyzir;
    // native point types
    Point_LEGACY pt_legacy;
    Point_RNG19_RFL8_SIG16_NIR16_DUAL pt_rg19_rf8_sg16_nr16_dual;
    Point_RNG19_RFL8_SIG16_NIR16 pt_rg19_rf8_sg16_nr16;
    Point_RNG15_RFL8_NIR8 pt_rg15_rfl8_nr8;
    // ouster_ros original/legacy point (not to be confused with Point_LEGACY)
    ouster_ros::Point pt_os_point;
};

TEST_F(PointAccessorTest, ElementCount) {
    // pcl & velodyne point types
    EXPECT_EQ(point_element_count(pt_xyz), 3U);
    EXPECT_EQ(point_element_count(pt_xyzi), 4U);
    EXPECT_EQ(point_element_count(pt_xyzir), 5U);
    // all native sensor point types has {x, y, z, t and ring} fields
    EXPECT_EQ(point_element_count(pt_legacy), 5 + Profile_LEGACY.size());
    EXPECT_EQ(point_element_count(pt_rg19_rf8_sg16_nr16_dual), 5 + Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size());
    EXPECT_EQ(point_element_count(pt_rg19_rf8_sg16_nr16), 5 + Profile_RNG19_RFL8_SIG16_NIR16.size());
    EXPECT_EQ(point_element_count(pt_rg15_rfl8_nr8), 5 + Profile_RNG15_RFL8_NIR8.size());
    // ouster_ros original/legacy point type
    EXPECT_EQ(point_element_count(pt_os_point), 9U);
}

TEST_F(PointAccessorTest, ExpectElementValueSameAsIndex) {

    auto expect_element_equals_index = [](auto& pt) {
        enumerate_point<0, point_element_count(pt)>(pt, [](auto index, auto value) {
            EXPECT_EQ(value, static_cast<decltype(value)>(index));
        });
    };

    // pcl + velodyne point types
    expect_element_equals_index(pt_xyz);
    expect_element_equals_index(pt_xyzi);
    expect_element_equals_index(pt_xyzir);
    // native sensor point types
    expect_element_equals_index(pt_legacy);
    expect_element_equals_index(pt_rg19_rf8_sg16_nr16_dual);
    expect_element_equals_index(pt_rg19_rf8_sg16_nr16);
    expect_element_equals_index(pt_rg15_rfl8_nr8);
    // ouster_ros original/legacy point type
    expect_element_equals_index(pt_os_point);
}

TEST_F(PointAccessorTest, ExpectPointElementValueIncrementedByValue) {

    auto increment_by_value = [](auto& pt, auto increment) {
        iterate_point<0, point_element_count(pt)>(pt, [increment](auto& value) {
            value += increment;
        });
    };

    auto expect_value_increased_by_value = [](auto& pt, auto increment) {
        enumerate_point<0, point_element_count(pt)>(pt, [increment](auto index, auto value) {
            EXPECT_EQ(value, static_cast<decltype(value)>(index + increment));
        });
    };

    auto inc = 1;

    // pcl + velodyne point types
    increment_by_value(pt_xyz, inc);
    expect_value_increased_by_value(pt_xyz, inc);
    increment_by_value(pt_xyzi, inc);
    expect_value_increased_by_value(pt_xyzi, inc);
    increment_by_value(pt_xyzir, inc);
    expect_value_increased_by_value(pt_xyzir, inc);

    // native sensor point types
    increment_by_value(pt_legacy, inc);
    expect_value_increased_by_value(pt_legacy, inc);
    increment_by_value(pt_rg19_rf8_sg16_nr16_dual, inc);
    expect_value_increased_by_value(pt_rg19_rf8_sg16_nr16_dual, inc);
    increment_by_value(pt_rg19_rf8_sg16_nr16, inc);
    expect_value_increased_by_value(pt_rg19_rf8_sg16_nr16, inc);
    increment_by_value(pt_rg15_rfl8_nr8, inc);
    expect_value_increased_by_value(pt_rg15_rfl8_nr8, inc);

    // ouster_ros original/legacy point type
    increment_by_value(pt_os_point, inc);
    expect_value_increased_by_value(pt_os_point, inc);
}
