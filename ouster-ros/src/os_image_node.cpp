/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_image_node.cpp
 * @brief A node to decode range, near ir and signal images from ouster
 * point cloud
 *
 * Publishes ~/range_image, ~/nearir_image, and ~/signal_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ouster/image_processing.h"
#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_srvs/srv/get_metadata.hpp"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;
using ouster_srvs::srv::GetMetadata;

using pixel_type = uint16_t;
const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

namespace ouster_ros {

class OusterImage : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterImage(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_image", options) {
        on_init();
    }

   private:
    void on_init() {
        auto metadata = get_metadata();
        info = sensor::parse_metadata(metadata);
        create_cloud_object();
        const int n_returns = get_n_returns();
        create_publishers(n_returns);
        create_subscriptions(n_returns);
    }

    void create_cloud_object() {
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        cloud = ouster_ros::Cloud{W, H};
    }

    void create_publishers(int n_returns) {
        rclcpp::SensorDataQoS qos;
        nearir_image_pub =
            create_publisher<sensor_msgs::msg::Image>("nearir_image", qos);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr a_pub;
        for (int i = 0; i < n_returns; i++) {
            a_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_for_return("range_image", i), qos);
            range_image_pubs.push_back(a_pub);

            a_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_for_return("signal_image", i), qos);
            signal_image_pubs.push_back(a_pub);

            a_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_for_return("reflec_image", i), qos);
            reflec_image_pubs.push_back(a_pub);
        }
    }

    void create_subscriptions(int n_returns) {
        pc_subs.resize(n_returns);
        rclcpp::SensorDataQoS qos;
        for (int i = 0; i < n_returns; ++i) {
            pc_subs[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_for_return("points", i), qos,
                [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    point_cloud_handler(msg, i);
                });
        }
    }

    void point_cloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr m,
                             int return_index) {
        pcl::fromROSMsg(*m, cloud);

        const bool first = (return_index == 0);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;

        auto range_image = make_image_msg(H, W, m->header.stamp);
        auto signal_image = make_image_msg(H, W, m->header.stamp);
        auto reflec_image = make_image_msg(H, W, m->header.stamp);
        auto nearir_image = make_image_msg(H, W, m->header.stamp);

        ouster::img_t<float> nearir_image_eigen(H, W);
        ouster::img_t<float> signal_image_eigen(H, W);
        ouster::img_t<float> reflec_image_eigen(H, W);

        // views into message data
        auto range_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)range_image->data.data(), H, W);
        auto signal_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)signal_image->data.data(), H, W);
        auto reflec_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)reflec_image->data.data(), H, W);

        auto nearir_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)nearir_image->data.data(), H, W);

        const auto& px_offset = info.format.pixel_shift_by_row;

        // copy data out of Cloud message, with destaggering
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const auto& pt = cloud[u * W + vv];

                // 16 bit img: use 4mm resolution and throw out returns >
                // 260m
                auto r = (pt.range + 0b10) >> 2;
                range_image_map(u, v) = r > pixel_value_max ? 0 : r;

                signal_image_eigen(u, v) = pt.intensity;
                reflec_image_eigen(u, v) = pt.reflectivity;
                nearir_image_eigen(u, v) = pt.ambient;
            }
        }

        signal_ae(signal_image_eigen, first);
        reflec_ae(reflec_image_eigen, first);
        nearir_buc(nearir_image_eigen);
        nearir_ae(nearir_image_eigen, first);
        nearir_image_eigen = nearir_image_eigen.sqrt();
        signal_image_eigen = signal_image_eigen.sqrt();

        // copy data into image messages
        signal_image_map =
            (signal_image_eigen * pixel_value_max).cast<pixel_type>();
        reflec_image_map =
            (reflec_image_eigen * pixel_value_max).cast<pixel_type>();
        if (first) {
            nearir_image_map =
                (nearir_image_eigen * pixel_value_max).cast<pixel_type>();
            nearir_image_pub->publish(std::move(nearir_image));
        }

        // publish at return index
        range_image_pubs[return_index]->publish(std::move(range_image));
        signal_image_pubs[return_index]->publish(std::move(signal_image));
        reflec_image_pubs[return_index]->publish(std::move(reflec_image));
    }

    static sensor_msgs::msg::Image::UniquePtr make_image_msg(
        size_t H, size_t W, const rclcpp::Time& stamp) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->width = W;
        msg->height = H;
        msg->step = W * sizeof(pixel_type);
        msg->encoding = sensor_msgs::image_encodings::MONO16;
        msg->data.resize(W * H * sizeof(pixel_type));
        msg->header.stamp = stamp;
        return msg;
    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr nearir_image_pub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        range_image_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        signal_image_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        reflec_image_pubs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
        pc_subs;

    ouster_ros::Cloud cloud;
    viz::AutoExposure nearir_ae, signal_ae, reflec_ae;
    viz::BeamUniformityCorrector nearir_buc;
};
}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterImage)
