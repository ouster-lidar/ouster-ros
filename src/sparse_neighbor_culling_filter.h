/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sparse_neighbor_culling_filter.h
 * @brief Sparse-neighbor culling for organized ROS PointCloud2 messages.
 */

#pragma once

#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace ouster_ros {

struct SparseNeighborCullingConfig {
    bool enabled;
    int spatial_min_neighbors;
    int spatial_kernel_height;
    int spatial_kernel_width;
    double max_culling_range_m;
    double neighbor_distance_m;
    bool use_filter_field;
    std::string filter_field;
    double filter_threshold;
    bool keep_organized;
};

/**
 * Byte-layout-preserving port of the Python sparse noise culler. It changes
 * only XYZ values for organized output, or removes complete point records for
 * compact output.
 */
class SparseNeighborCullingFilter {
   public:
    explicit SparseNeighborCullingFilter(SparseNeighborCullingConfig config)
        : config_{std::move(config)} {}

    void filter(sensor_msgs::PointCloud2& cloud) const {
        if (!config_.enabled) return;

        if (cloud.height <= 1) {
            ROS_WARN_THROTTLE(
                5.0,
                "Input PointCloud2 is not organized (height <= 1). Sparse "
                "image-based culling needs an organized cloud.");
            return;
        }

        const auto* x_field = find_field(cloud, "x");
        const auto* y_field = find_field(cloud, "y");
        const auto* z_field = find_field(cloud, "z");
        if (!usable_xyz_field(cloud, x_field) ||
            !usable_xyz_field(cloud, y_field) ||
            !usable_xyz_field(cloud, z_field)) {
            ROS_WARN_THROTTLE(
                5.0,
                "Input PointCloud2 must contain usable scalar floating-point "
                "x, y, z fields. Publishing unmodified cloud.");
            return;
        }

        const uint64_t required_size =
            static_cast<uint64_t>(cloud.row_step) * cloud.height;
        const uint64_t minimum_row_size =
            static_cast<uint64_t>(cloud.point_step) * cloud.width;
        if (cloud.data.size() < required_size ||
            cloud.row_step < minimum_row_size) {
            ROS_WARN_THROTTLE(
                5.0,
                "PointCloud2 data buffer or row_step is too small. Publishing "
                "unmodified cloud.");
            return;
        }

        const sensor_msgs::PointField* filter_field = nullptr;
        if (config_.use_filter_field) {
            filter_field = find_field(cloud, config_.filter_field);
            if (!usable_scalar_field(cloud, filter_field)) {
                ROS_WARN_THROTTLE(
                    5.0,
                    "Input PointCloud2 does not contain a usable '%s' field. "
                    "Publishing unmodified cloud.",
                    config_.filter_field.c_str());
                return;
            }
        }

        const size_t point_count =
            static_cast<size_t>(cloud.height) * cloud.width;
        std::vector<float> ranges(point_count, 0.0f);
        std::vector<uint8_t> candidates(point_count, 0);
        bool any_candidate = false;

        for (uint32_t row = 0; row < cloud.height; ++row) {
            for (uint32_t col = 0; col < cloud.width; ++col) {
                const size_t index = linear_index(cloud, row, col);
                const uint8_t* point = point_data(cloud, row, col);
                const double x = read_scalar(point + x_field->offset,
                                             x_field->datatype,
                                             cloud.is_bigendian);
                const double y = read_scalar(point + y_field->offset,
                                             y_field->datatype,
                                             cloud.is_bigendian);
                const double z = read_scalar(point + z_field->offset,
                                             z_field->datatype,
                                             cloud.is_bigendian);
                if (!std::isfinite(x) || !std::isfinite(y) ||
                    !std::isfinite(z)) {
                    continue;
                }

                const float range = static_cast<float>(
                    std::sqrt(x * x + y * y + z * z));
                ranges[index] = range;
                bool candidate =
                    range > 0.0f && range < config_.max_culling_range_m;
                if (candidate && filter_field) {
                    const double value =
                        read_scalar(point + filter_field->offset,
                                    filter_field->datatype,
                                    cloud.is_bigendian);
                    candidate = std::isfinite(value) &&
                                value < config_.filter_threshold;
                }
                candidates[index] = candidate;
                any_candidate = any_candidate || candidate;
            }
        }

        if (!any_candidate) return;

        std::vector<uint8_t> cull(point_count, 0);
        bool any_culled = false;
        const int half_width = config_.spatial_kernel_width / 2;
        const int half_height = config_.spatial_kernel_height / 2;

        for (uint32_t row = 0; row < cloud.height; ++row) {
            for (uint32_t col = 0; col < cloud.width; ++col) {
                const size_t index = linear_index(cloud, row, col);
                if (!candidates[index]) continue;

                uint8_t neighbor_count = 0;
                for (int shift_x = -half_width; shift_x <= half_width;
                     ++shift_x) {
                    for (int shift_y = -half_height;
                         shift_y <= half_height; ++shift_y) {
                        const int neighbor_row =
                            static_cast<int>(row) - shift_y;
                        if (neighbor_row < 0 ||
                            neighbor_row >= static_cast<int>(cloud.height)) {
                            continue;
                        }
                        int neighbor_col =
                            (static_cast<int>(col) - shift_x) %
                            static_cast<int>(cloud.width);
                        if (neighbor_col < 0) neighbor_col += cloud.width;

                        const float neighbor_range = ranges[linear_index(
                            cloud, static_cast<uint32_t>(neighbor_row),
                            static_cast<uint32_t>(neighbor_col))];
                        if (neighbor_range > 0.0f &&
                            std::abs(neighbor_range - ranges[index]) <=
                                config_.neighbor_distance_m) {
                            ++neighbor_count;
                        }
                    }
                }

                if (neighbor_count < config_.spatial_min_neighbors) {
                    cull[index] = 1;
                    any_culled = true;
                }
            }
        }

        if (!any_culled) return;

        if (config_.keep_organized) {
            for (uint32_t row = 0; row < cloud.height; ++row) {
                for (uint32_t col = 0; col < cloud.width; ++col) {
                    if (!cull[linear_index(cloud, row, col)]) continue;
                    uint8_t* point = point_data(cloud, row, col);
                    write_nan(point + x_field->offset, x_field->datatype,
                              cloud.is_bigendian);
                    write_nan(point + y_field->offset, y_field->datatype,
                              cloud.is_bigendian);
                    write_nan(point + z_field->offset, z_field->datatype,
                              cloud.is_bigendian);
                }
            }
        } else {
            std::vector<uint8_t> compacted;
            compacted.reserve(point_count * cloud.point_step);
            for (uint32_t row = 0; row < cloud.height; ++row) {
                for (uint32_t col = 0; col < cloud.width; ++col) {
                    if (cull[linear_index(cloud, row, col)]) continue;
                    const uint8_t* point = point_data(cloud, row, col);
                    compacted.insert(compacted.end(), point,
                                     point + cloud.point_step);
                }
            }
            cloud.height = 1;
            cloud.width =
                static_cast<uint32_t>(compacted.size() / cloud.point_step);
            cloud.row_step = cloud.width * cloud.point_step;
            cloud.data = std::move(compacted);
        }
        cloud.is_dense = false;
    }

   private:
    static const sensor_msgs::PointField* find_field(
        const sensor_msgs::PointCloud2& cloud, const std::string& name) {
        const auto it = std::find_if(
            cloud.fields.begin(), cloud.fields.end(),
            [&name](const sensor_msgs::PointField& field) {
                return field.name == name;
            });
        return it == cloud.fields.end() ? nullptr : &*it;
    }

    static size_t datatype_size(uint8_t datatype) {
        switch (datatype) {
            case sensor_msgs::PointField::INT8:
            case sensor_msgs::PointField::UINT8:
                return 1;
            case sensor_msgs::PointField::INT16:
            case sensor_msgs::PointField::UINT16:
                return 2;
            case sensor_msgs::PointField::INT32:
            case sensor_msgs::PointField::UINT32:
            case sensor_msgs::PointField::FLOAT32:
                return 4;
            case sensor_msgs::PointField::FLOAT64:
                return 8;
            default:
                return 0;
        }
    }

    static bool usable_scalar_field(
        const sensor_msgs::PointCloud2& cloud,
        const sensor_msgs::PointField* field) {
        if (!field || field->count != 1) return false;
        const size_t size = datatype_size(field->datatype);
        return size != 0 &&
               static_cast<uint64_t>(field->offset) + size <= cloud.point_step;
    }

    static bool usable_xyz_field(const sensor_msgs::PointCloud2& cloud,
                                 const sensor_msgs::PointField* field) {
        return usable_scalar_field(cloud, field) &&
               (field->datatype == sensor_msgs::PointField::FLOAT32 ||
                field->datatype == sensor_msgs::PointField::FLOAT64);
    }

    static size_t linear_index(const sensor_msgs::PointCloud2& cloud,
                               uint32_t row, uint32_t col) {
        return static_cast<size_t>(row) * cloud.width + col;
    }

    static const uint8_t* point_data(const sensor_msgs::PointCloud2& cloud,
                                     uint32_t row, uint32_t col) {
        return cloud.data.data() + static_cast<size_t>(row) * cloud.row_step +
               static_cast<size_t>(col) * cloud.point_step;
    }

    static uint8_t* point_data(sensor_msgs::PointCloud2& cloud, uint32_t row,
                               uint32_t col) {
        return cloud.data.data() + static_cast<size_t>(row) * cloud.row_step +
               static_cast<size_t>(col) * cloud.point_step;
    }

    static bool host_is_bigendian() {
        const uint16_t value = 0x0102;
        return *reinterpret_cast<const uint8_t*>(&value) == 0x01;
    }

    template <typename T>
    static T load_value(const uint8_t* data, bool is_bigendian) {
        T value;
        std::memcpy(&value, data, sizeof(T));
        if (is_bigendian != host_is_bigendian()) {
            auto* bytes = reinterpret_cast<uint8_t*>(&value);
            std::reverse(bytes, bytes + sizeof(T));
        }
        return value;
    }

    template <typename T>
    static void store_value(uint8_t* data, T value, bool is_bigendian) {
        if (is_bigendian != host_is_bigendian()) {
            auto* bytes = reinterpret_cast<uint8_t*>(&value);
            std::reverse(bytes, bytes + sizeof(T));
        }
        std::memcpy(data, &value, sizeof(T));
    }

    static double read_scalar(const uint8_t* data, uint8_t datatype,
                              bool is_bigendian) {
        switch (datatype) {
            case sensor_msgs::PointField::INT8:
                return load_value<int8_t>(data, is_bigendian);
            case sensor_msgs::PointField::UINT8:
                return load_value<uint8_t>(data, is_bigendian);
            case sensor_msgs::PointField::INT16:
                return load_value<int16_t>(data, is_bigendian);
            case sensor_msgs::PointField::UINT16:
                return load_value<uint16_t>(data, is_bigendian);
            case sensor_msgs::PointField::INT32:
                return load_value<int32_t>(data, is_bigendian);
            case sensor_msgs::PointField::UINT32:
                return load_value<uint32_t>(data, is_bigendian);
            case sensor_msgs::PointField::FLOAT32:
                return load_value<float>(data, is_bigendian);
            case sensor_msgs::PointField::FLOAT64:
                return load_value<double>(data, is_bigendian);
            default:
                return std::numeric_limits<double>::quiet_NaN();
        }
    }

    static void write_nan(uint8_t* data, uint8_t datatype,
                          bool is_bigendian) {
        if (datatype == sensor_msgs::PointField::FLOAT32) {
            store_value(data, std::numeric_limits<float>::quiet_NaN(),
                        is_bigendian);
        } else {
            store_value(data, std::numeric_limits<double>::quiet_NaN(),
                        is_bigendian);
        }
    }

    SparseNeighborCullingConfig config_;
};

}  // namespace ouster_ros
