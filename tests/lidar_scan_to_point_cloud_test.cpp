/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_scan_to_point_cloud_test.cpp
 * @brief Unit tests for the LidarScan + LidarInfo -> PointCloud2 conversion.
 */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "ouster_ros/ChannelField.h"
#include "ouster_ros/LidarInfo.h"
#include "ouster_ros/LidarScan.h"
#include "ouster_ros/lidar_scan_to_point_cloud.h"

namespace {

/**
 * @brief Helper to create a minimal LidarInfo for a simple sensor with
 * h beams and w columns, all beams at 0 elevation and 0 azimuth offset,
 * identity transforms, and 1mm range unit.
 */
ouster_ros::LidarInfo makeSimpleLidarInfo(uint32_t h, uint32_t w) {
    ouster_ros::LidarInfo info;
    info.pixels_per_column = h;
    info.columns_per_frame = w;
    info.fps = 10;
    info.range_unit = 0.001;  // 1mm per count
    info.product_line = "TEST-SENSOR";
    info.serial_number = 12345;
    info.firmware_revision = "v1.0.0";
    info.udp_profile_lidar = "LEGACY";

    // All beams at 0 elevation, 0 azimuth offset -> uniform horizontal ring
    info.beam_azimuth_angles.resize(h, 0.0);
    info.beam_altitude_angles.resize(h, 0.0);

    // No pixel shifts
    info.pixel_shift_by_row.resize(h, 0);

    // Identity transforms (row-major 4x4)
    // beam_to_lidar: identity (no beam origin offset)
    info.beam_to_lidar_transform.fill(0.0);
    info.beam_to_lidar_transform[0] = 1.0;
    info.beam_to_lidar_transform[5] = 1.0;
    info.beam_to_lidar_transform[10] = 1.0;
    info.beam_to_lidar_transform[15] = 1.0;

    // lidar_to_sensor: identity
    info.lidar_to_sensor_transform.fill(0.0);
    info.lidar_to_sensor_transform[0] = 1.0;
    info.lidar_to_sensor_transform[5] = 1.0;
    info.lidar_to_sensor_transform[10] = 1.0;
    info.lidar_to_sensor_transform[15] = 1.0;

    // imu_to_sensor: identity
    info.imu_to_sensor_transform.fill(0.0);
    info.imu_to_sensor_transform[0] = 1.0;
    info.imu_to_sensor_transform[5] = 1.0;
    info.imu_to_sensor_transform[10] = 1.0;
    info.imu_to_sensor_transform[15] = 1.0;

    // extrinsic: identity
    info.extrinsic.fill(0.0);
    info.extrinsic[0] = 1.0;
    info.extrinsic[5] = 1.0;
    info.extrinsic[10] = 1.0;
    info.extrinsic[15] = 1.0;

    return info;
}

/**
 * @brief Helper to add a uint32 channel to a LidarScan.
 */
void addUint32Channel(ouster_ros::LidarScan& scan, const std::string& name,
                      const std::vector<uint32_t>& values) {
    ouster_ros::ChannelField ch;
    ch.name = name;
    ch.datatype = ouster_ros::ChannelField::UINT32;
    ch.element_size = 4;
    ch.data.resize(values.size() * sizeof(uint32_t));
    std::memcpy(ch.data.data(), values.data(),
                values.size() * sizeof(uint32_t));
    scan.channels.push_back(ch);
}

/**
 * @brief Helper to add a uint16 channel to a LidarScan.
 */
void addUint16Channel(ouster_ros::LidarScan& scan, const std::string& name,
                      const std::vector<uint16_t>& values) {
    ouster_ros::ChannelField ch;
    ch.name = name;
    ch.datatype = ouster_ros::ChannelField::UINT16;
    ch.element_size = 2;
    ch.data.resize(values.size() * sizeof(uint16_t));
    std::memcpy(ch.data.data(), values.data(),
                values.size() * sizeof(uint16_t));
    scan.channels.push_back(ch);
}

/**
 * @brief Read a float value from raw point cloud data at a given byte offset.
 */
float readFloat(const sensor_msgs::PointCloud2& cloud, size_t point_idx,
                uint32_t field_offset) {
    float val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(float));
    return val;
}

/**
 * @brief Read a uint32 value from raw point cloud data.
 */
uint32_t readUint32(const sensor_msgs::PointCloud2& cloud, size_t point_idx,
                    uint32_t field_offset) {
    uint32_t val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(uint32_t));
    return val;
}

/**
 * @brief Read a uint16 value from raw point cloud data.
 */
uint16_t readUint16(const sensor_msgs::PointCloud2& cloud, size_t point_idx,
                    uint32_t field_offset) {
    uint16_t val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(uint16_t));
    return val;
}

}  // anonymous namespace

class LidarScanToPointCloudTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Create a minimal 4-beam, 8-column sensor
        h = 4;
        w = 8;
        info = makeSimpleLidarInfo(h, w);
    }

    uint32_t h, w;
    ouster_ros::LidarInfo info;
};

TEST_F(LidarScanToPointCloudTest, BasicDimensions) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);  // 1 second in ns

    // All ranges = 1000 (1 meter at range_unit=0.001)
    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    EXPECT_EQ(cloud.height, h);
    EXPECT_EQ(cloud.width, w);
    EXPECT_EQ(cloud.fields.size(), 9u);  // x, y, z, range, signal, reflectivity, near_ir, t, ring
    EXPECT_FALSE(cloud.is_bigendian);
}

TEST_F(LidarScanToPointCloudTest, ZeroRangeProducesNaN) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    // All ranges = 0 (invalid)
    std::vector<uint32_t> range_data(h * w, 0);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    // All points should be NaN
    for (size_t i = 0; i < static_cast<size_t>(h * w); ++i) {
        float x = readFloat(cloud, i, 0);
        float y = readFloat(cloud, i, 4);
        float z = readFloat(cloud, i, 8);
        EXPECT_TRUE(std::isnan(x)) << "Point " << i << " x should be NaN";
        EXPECT_TRUE(std::isnan(y)) << "Point " << i << " y should be NaN";
        EXPECT_TRUE(std::isnan(z)) << "Point " << i << " z should be NaN";
    }

    EXPECT_FALSE(cloud.is_dense);
}

TEST_F(LidarScanToPointCloudTest, NonZeroRangeProducesValidXYZ) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    // Set range to 1000mm = 1.0m for all points
    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    // With identity transforms, 0 elevation, and 0 azimuth offsets,
    // points should lie at unit distance from origin with z=0.
    // The encoder angle varies with column:
    //   encoder = 2*pi - col * (2*pi/w)
    // xyz = range * direction, direction = (cos(encoder), sin(encoder), 0)
    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            size_t idx = row * w + col;
            float x = readFloat(cloud, idx, 0);
            float y = readFloat(cloud, idx, 4);
            float z = readFloat(cloud, idx, 8);

            EXPECT_FALSE(std::isnan(x));
            EXPECT_FALSE(std::isnan(y));

            // Distance from origin should be ~1.0m
            float dist = std::sqrt(x * x + y * y + z * z);
            EXPECT_NEAR(dist, 1.0f, 0.01f)
                << "Point (" << row << "," << col << ") dist=" << dist;
        }
    }
}

TEST_F(LidarScanToPointCloudTest, ChannelDataIsCopied) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    size_t n = h * w;
    std::vector<uint32_t> range_data(n, 1000);
    std::vector<uint16_t> signal_data(n, 42);
    std::vector<uint16_t> reflectivity_data(n, 100);
    std::vector<uint16_t> near_ir_data(n, 200);

    addUint32Channel(scan, "range", range_data);
    addUint16Channel(scan, "signal", signal_data);
    addUint16Channel(scan, "reflectivity", reflectivity_data);
    addUint16Channel(scan, "near_ir", near_ir_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    // Verify channel data for the first point
    uint32_t range_val = readUint32(cloud, 0, 12);
    uint16_t signal_val = readUint16(cloud, 0, 16);
    uint16_t refl_val = readUint16(cloud, 0, 18);
    uint16_t nir_val = readUint16(cloud, 0, 20);

    EXPECT_EQ(range_val, 1000u);
    EXPECT_EQ(signal_val, 42u);
    EXPECT_EQ(refl_val, 100u);
    EXPECT_EQ(nir_val, 200u);
}

TEST_F(LidarScanToPointCloudTest, RingFieldIsSet) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    // ring field is at offset 26 (after t at 22)
    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            size_t idx = row * w + col;
            uint16_t ring = readUint16(cloud, idx, 26);
            EXPECT_EQ(ring, static_cast<uint16_t>(row))
                << "ring mismatch at (" << row << "," << col << ")";
        }
    }
}

TEST_F(LidarScanToPointCloudTest, TimestampOffsets) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;

    // Set timestamps: column 0 starts at 1e9, each column increments by 1e6
    scan.timestamp.resize(w);
    for (uint32_t col = 0; col < w; ++col) {
        scan.timestamp[col] = 1000000000ULL + col * 1000000ULL;
    }

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    // t field is at offset 22
    // Column 0 should have t=0 (relative to scan start)
    // Column 1 should have t=1000000 (1ms)
    uint32_t t0 = readUint32(cloud, 0, 22);
    EXPECT_EQ(t0, 0u);

    // Second column of first row
    uint32_t t1 = readUint32(cloud, 1, 22);
    EXPECT_EQ(t1, 1000000u);  // 1ms
}

TEST_F(LidarScanToPointCloudTest, MissingOptionalChannels) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    // Only range, no signal/reflectivity/near_ir
    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    EXPECT_NO_THROW(
        ouster_ros::lidarScanToPointCloud(scan, info, cloud, false));

    // Optional channels should default to 0
    uint16_t signal_val = readUint16(cloud, 0, 16);
    uint16_t refl_val = readUint16(cloud, 0, 18);
    uint16_t nir_val = readUint16(cloud, 0, 20);

    EXPECT_EQ(signal_val, 0u);
    EXPECT_EQ(refl_val, 0u);
    EXPECT_EQ(nir_val, 0u);
}

TEST_F(LidarScanToPointCloudTest, MissingRangeChannelThrows) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    // No channels at all
    sensor_msgs::PointCloud2 cloud;
    EXPECT_THROW(ouster_ros::lidarScanToPointCloud(scan, info, cloud, false),
                 std::invalid_argument);
}

TEST_F(LidarScanToPointCloudTest, DimensionMismatchThrows) {
    ouster_ros::LidarScan scan;
    scan.height = h + 1;  // Mismatch!
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data((h + 1) * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    EXPECT_THROW(ouster_ros::lidarScanToPointCloud(scan, info, cloud, false),
                 std::invalid_argument);
}

TEST_F(LidarScanToPointCloudTest, HeaderIsCopied) {
    ouster_ros::LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.header.frame_id = "os_lidar";
    scan.header.stamp.sec = 100;
    scan.header.stamp.nsec = 500;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    EXPECT_EQ(cloud.header.frame_id, "os_lidar");
    EXPECT_EQ(cloud.header.stamp.sec, 100u);
    EXPECT_EQ(cloud.header.stamp.nsec, 500u);
}
