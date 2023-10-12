#pragma once

#include "point_cloud_processor.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using sensor::UDPProfileLidar;

class PointCloudProcessorFactory {

    template <typename PointT>
    static typename PointCloudProcessor<PointT>::ScanToCloudFn make_scan_to_cloud_fn(const sensor::sensor_info& info) {

        switch (info.format.udp_profile_lidar) {
            case UDPProfileLidar::PROFILE_LIDAR_LEGACY:
                return [](ouster_ros::Cloud<PointT>& cloud,
                            const ouster::PointsF& points,
                            uint64_t scan_ts, const ouster::LidarScan& ls,
                            const std::vector<int>& pixel_shift_by_row,
                            int return_index) {
                    unused_variable(return_index);
                    static Point_LEGACY staging_pt; // TODO: temporary, will remove
                    scan_to_cloud_f_destaggered_tuple<Profile_LEGACY.size(), Profile_LEGACY>(
                        cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                };

            case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL:
                return [](ouster_ros::Cloud<PointT>& cloud,
                            const ouster::PointsF& points,
                            uint64_t scan_ts, const ouster::LidarScan& ls,
                            const std::vector<int>& pixel_shift_by_row,
                            int return_index) {
                    static Point_RNG19_RFL8_SIG16_NIR16_DUAL staging_pt; // TODO: temporary, will remove
                    if (return_index == 0) {
                        scan_to_cloud_f_destaggered_tuple<
                            Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(), Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(
                            cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                    }
                    else {
                        scan_to_cloud_f_destaggered_tuple<
                            Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN.size(), Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN>(
                            cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                    }
                };

            case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16:
                return [](ouster_ros::Cloud<PointT>& cloud,
                            const ouster::PointsF& points,
                            uint64_t scan_ts, const ouster::LidarScan& ls,
                            const std::vector<int>& pixel_shift_by_row,
                            int return_index) {
                    unused_variable(return_index);
                    static Point_RNG19_RFL8_SIG16_NIR16 staging_pt; // TODO: temporary, will remove
                    scan_to_cloud_f_destaggered_tuple<
                        Profile_RNG19_RFL8_SIG16_NIR16.size(), Profile_RNG19_RFL8_SIG16_NIR16>(
                        cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                };

            case UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8:
                return [](ouster_ros::Cloud<PointT>& cloud,
                            const ouster::PointsF& points,
                            uint64_t scan_ts, const ouster::LidarScan& ls,
                            const std::vector<int>& pixel_shift_by_row,
                            int return_index) {
                    unused_variable(return_index);
                    static Point_RNG15_RFL8_NIR8 staging_pt; // TODO: temporary, will remove
                    scan_to_cloud_f_destaggered_tuple<
                        Profile_RNG15_RFL8_NIR8.size(), Profile_RNG15_RFL8_NIR8>(
                        cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                };

            default:
                // RCLCPP_WARN_STREAM(get_logger(),
                //     "point_type is set to auto but current udp_profile_lidar is unsupported "
                //     "! falling back to the driver default/legacy pcl point format");
                return [](ouster_ros::Cloud<PointT>& cloud,
                            const ouster::PointsF& points,
                            uint64_t scan_ts, const ouster::LidarScan& ls,
                            const std::vector<int>& pixel_shift_by_row,
                            int return_index) {
                    unused_variable(return_index);
                    static Point_LEGACY staging_pt; // TODO: temporary, will remove
                    scan_to_cloud_f_destaggered_tuple<Profile_LEGACY.size(), Profile_LEGACY>(
                        cloud, staging_pt, points, scan_ts, ls, pixel_shift_by_row);
                };
        }
    }

    template <typename PointT>
    static LidarScanProcessor make_point_cloud_procssor(
        const sensor::sensor_info& info, const std::string& frame,
        bool apply_lidar_to_sensor_transform, PointCloudProcessor_PostProcessingFn post_processing_fn) {
        auto scan_to_cloud_fn = make_scan_to_cloud_fn<PointT>(info);
        return PointCloudProcessor<PointT>::create(
            info, frame, apply_lidar_to_sensor_transform,
            scan_to_cloud_fn, post_processing_fn);
    }

    public:
    static LidarScanProcessor create_point_cloud_processor(const std::string& point_type,
                                                    const sensor::sensor_info& info,
                                                    const std::string& frame,
                                                    bool apply_lidar_to_sensor_transform,
                                                    PointCloudProcessor_PostProcessingFn post_processing_fn) {
        if (point_type == "xyz") {
            return make_point_cloud_procssor<pcl::PointXYZ>(
                info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
        } else if (point_type == "xyzi") {
            if (info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8)
                // RCLCPP_WARN_STREAM(get_logger(),
                //     "selected point type 'xyzi' is not compatible with the current udp profile: RNG15_RFL8_NIR8");
            return make_point_cloud_procssor<pcl::PointXYZI>(
                info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
        } else if (point_type == "xyzir") {
            if (info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8)
                // RCLCPP_WARN_STREAM(get_logger(),
                //     "selected point type 'xyzir' is not compatible with the current udp profile: RNG15_RFL8_NIR8");
            return make_point_cloud_procssor<PointXYZIR>(
                info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
        } else if (point_type == "native") {
            switch (info.format.udp_profile_lidar) {
                case UDPProfileLidar::PROFILE_LIDAR_LEGACY:
                    return make_point_cloud_procssor<Point_LEGACY>(
                        info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
                case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL:
                    return make_point_cloud_procssor<Point_RNG19_RFL8_SIG16_NIR16_DUAL>(
                        info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
                case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16:
                    return make_point_cloud_procssor<Point_RNG19_RFL8_SIG16_NIR16>(
                        info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
                case UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8:
                    return make_point_cloud_procssor<Point_RNG15_RFL8_NIR8>(
                        info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
                default:
                    // TODO: revize fallback
                    throw std::runtime_error("point_type is set to auto but current udp_profile_lidar is unknown");
            }
        } else if (point_type != "original") {
            // RCLCPP_WARN_STREAM(get_logger(),
            //     "Un-supported point type used: " << point_type <<
            //     "! falling back to driver original/legacy pcl point format");
        }

        return make_point_cloud_procssor<ouster_ros::Point>(
            info, frame, apply_lidar_to_sensor_transform, post_processing_fn);
    }

};

}