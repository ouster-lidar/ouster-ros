/**
 * @file os_pinhole_node.cpp
 * @brief Publishes N pinhole rectified panels (range/signal/reflec/nearir +
 *        CameraInfo) re-sampled from the destaggered Ouster panorama. Each
 *        panel has its own optical-convention frame, broadcast via
 *        StaticTransformBroadcaster, so Foxglove/Trillium and standard
 *        image pipelines see proper pinhole cameras.
 *
 * Built on the same LidarPacketHandler + LidarScan path as os_image_node;
 * the resampling is implemented in PinholeProcessor.
 */

// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_processing_node_base.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lidar_packet_handler.h"
#include "pinhole_processor.h"

namespace ouster_ros {

using ouster_sensor_msgs::msg::PacketMsg;
using ouster::sdk::core::LidarPacket;

class OusterPinhole : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterPinhole(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_pinhole", options) {
        on_init();
    }

   private:
    void on_init() {
        // QoS / replay config — same shape as os_image.
        declare_parameter("timestamp_mode", "");
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("use_system_default_qos", false);
        declare_parameter("min_scan_valid_columns_ratio", 0.0);

        // Panel config (parallel arrays). panel_names is the source of
        // truth for the panel count; the rest are sized to match (or
        // broadcast a single scalar to all panels).
        declare_parameter<std::vector<std::string>>(
            "panel_names",
            std::vector<std::string>{"front", "left", "rear", "right"});
        declare_parameter<std::vector<double>>(
            "panel_yaws_deg", std::vector<double>{0.0, 90.0, 180.0, 270.0});
        declare_parameter<std::vector<double>>(
            "panel_pitches_deg", std::vector<double>{0.0});
        declare_parameter<std::vector<double>>(
            "panel_hfovs_deg", std::vector<double>{90.0});
        declare_parameter<std::vector<int64_t>>("panel_widths",
                                                 std::vector<int64_t>{256});
        declare_parameter<std::vector<int64_t>>("panel_heights",
                                                 std::vector<int64_t>{0});
        declare_parameter<std::vector<double>>("panel_vfovs_deg",
                                                std::vector<double>{0.0});

        // Frame configuration.
        declare_parameter("parent_frame", "os_lidar");
        declare_parameter("lidar_namespace", "lidar0");
        declare_parameter("optical_frame_template",
                          std::string("{ns}/panels/{name}_optical_frame"));

        // Constant azimuth offset (degrees) of the destaggered image's
        // column 0 relative to lidar_frame +X. If column 0 actually
        // contains content from azimuth=φ in lidar_frame, set this to φ
        // (positive = CCW). Per-platform empirical tune.
        declare_parameter("azimuth_offset_deg", 0.0);

        static_tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });

        RCLCPP_INFO(get_logger(), "OusterPinhole: node initialized!");
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterPinhole: retrieved new sensor metadata!");
        info = ouster::sdk::core::SensorInfo(metadata_msg->data);
        packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(
            ouster::sdk::core::get_format(info));
        create_publishers_subscribers();
        broadcast_static_transforms();
    }

    std::vector<PinholeProcessor::PanelConfig> read_panel_configs() {
        const auto names =
            get_parameter("panel_names").as_string_array();
        const auto yaws =
            get_parameter("panel_yaws_deg").as_double_array();
        const auto pitches =
            get_parameter("panel_pitches_deg").as_double_array();
        const auto hfovs =
            get_parameter("panel_hfovs_deg").as_double_array();
        const auto widths =
            get_parameter("panel_widths").as_integer_array();
        const auto heights =
            get_parameter("panel_heights").as_integer_array();
        const auto vfovs =
            get_parameter("panel_vfovs_deg").as_double_array();

        if (names.empty()) {
            RCLCPP_FATAL(get_logger(),
                         "OusterPinhole: panel_names is empty.");
            throw std::runtime_error("panel_names is empty");
        }
        if (yaws.size() != names.size()) {
            RCLCPP_FATAL(get_logger(),
                "OusterPinhole: panel_yaws_deg length (%zu) must match "
                "panel_names length (%zu).",
                yaws.size(), names.size());
            throw std::runtime_error("panel_yaws_deg length mismatch");
        }

        auto pick = [&](const auto& vec, size_t i, auto fallback) {
            if (vec.empty()) return fallback;
            // Singleton broadcasts to all panels; partial-length arrays
            // wrap (caller should pass length 1 or length names.size()).
            const size_t idx = (vec.size() == 1) ? 0 : (i % vec.size());
            return static_cast<decltype(fallback)>(vec[idx]);
        };

        std::vector<PinholeProcessor::PanelConfig> out;
        out.reserve(names.size());
        for (size_t i = 0; i < names.size(); ++i) {
            PinholeProcessor::PanelConfig cfg;
            cfg.name = names[i];
            cfg.yaw_rad = yaws[i] * M_PI / 180.0;
            cfg.pitch_rad = pick(pitches, i, 0.0) * M_PI / 180.0;
            cfg.hfov_rad = pick(hfovs, i, 90.0) * M_PI / 180.0;
            cfg.width = static_cast<uint32_t>(pick(widths, i, int64_t{256}));
            cfg.height = static_cast<uint32_t>(pick(heights, i, int64_t{0}));
            cfg.vfov_rad = pick(vfovs, i, 0.0) * M_PI / 180.0;
            out.push_back(cfg);
        }
        return out;
    }

    void create_publishers_subscribers() {
        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();
        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS selected_qos = use_system_default_qos
                                       ? rclcpp::QoS(rclcpp::SystemDefaultsQoS())
                                       : rclcpp::QoS(rclcpp::SensorDataQoS());

        const auto min_ratio =
            get_parameter("min_scan_valid_columns_ratio").as_double();
        if (min_ratio < 0.0 || min_ratio > 1.0) {
            RCLCPP_FATAL(get_logger(),
                         "min_scan_valid_columns_ratio out of [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio");
        }

        const auto lidar_namespace =
            get_parameter("lidar_namespace").as_string();
        const auto optical_frame_template =
            get_parameter("optical_frame_template").as_string();
        const auto azimuth_offset_deg =
            get_parameter("azimuth_offset_deg").as_double();
        const double W = static_cast<double>(info.format.columns_per_frame);
        const double azimuth_offset_columns =
            azimuth_offset_deg * W / 360.0;

        const auto panel_configs = read_panel_configs();

        // Build the processor + its panels.
        std::vector<LidarScanProcessor> processors {
            PinholeProcessor::create(
                info, panel_configs,
                optical_frame_template, lidar_namespace,
                azimuth_offset_columns,
                [this](PinholeProcessor::OutputType& panels) {
                    publish_panels(panels);
                })
        };

        // Build publishers from the constructed panel list. The processor
        // already created PanelOutput entries with image buffers + ci, so
        // we mirror those here as publishers.
        // We need to query the processor for its panels — but
        // PinholeProcessor::create returns just the lambda. Re-build a
        // separate helper instance just to reach the panel list, OR
        // build publishers from panel_configs directly + the channel set.
        const auto channel_topics = pinhole_channel_topics(info.num_returns());
        for (const auto& cfg : panel_configs) {
            PanelPublishers pp;
            pp.name = cfg.name;
            for (const auto& kv : channel_topics) {
                const std::string image_topic =
                    "panels/" + cfg.name + "/" + kv.second;
                pp.image_pubs[kv.first] =
                    create_publisher<sensor_msgs::msg::Image>(
                        image_topic, selected_qos);
            }
            const std::string ci_topic =
                "panels/" + cfg.name + "/camera_info";
            pp.camera_info_pub =
                create_publisher<sensor_msgs::msg::CameraInfo>(ci_topic,
                                                                selected_qos);
            panel_pubs_.push_back(std::move(pp));
        }

        lidar_packet_handler_ = LidarPacketHandler::create(
            info, processors, timestamp_mode,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            static_cast<float>(min_ratio));

        lidar_packet_sub_ = create_subscription<PacketMsg>(
            "lidar_packets", selected_qos,
            [this](const PacketMsg::ConstSharedPtr msg) {
                if (!lidar_packet_handler_) return;
                LidarPacket packet(msg->buf.size());
                packet.format = packet_format;
                packet.host_timestamp =
                    static_cast<uint64_t>(now().nanoseconds());
                std::memcpy(packet.buf.data(), msg->buf.data(),
                            msg->buf.size());
                lidar_packet_handler_(packet);
            });

        // Cache panel configs for the static-TF broadcast (yaw → rpy).
        panel_configs_for_tf_ = panel_configs;
        lidar_namespace_for_tf_ = lidar_namespace;
        optical_frame_template_for_tf_ = optical_frame_template;
    }

    void publish_panels(PinholeProcessor::OutputType& panels) {
        for (size_t i = 0; i < panels.size() && i < panel_pubs_.size(); ++i) {
            const auto& panel = panels[i];
            auto& pp = panel_pubs_[i];
            for (auto& kv : panel->images) {
                auto it = pp.image_pubs.find(kv.first);
                if (it != pp.image_pubs.end() && kv.second) {
                    it->second->publish(*kv.second);
                }
            }
            if (pp.camera_info_pub) {
                pp.camera_info_pub->publish(panel->camera_info);
            }
        }
    }

    void broadcast_static_transforms() {
        const auto parent_frame = get_parameter("parent_frame").as_string();
        std::string ns = lidar_namespace_for_tf_;
        while (!ns.empty() && ns.front() == '/') ns.erase(0, 1);

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        const auto stamp = now();
        for (const auto& cfg : panel_configs_for_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = parent_frame;
            tf.child_frame_id =
                substitute_template(optical_frame_template_for_tf_, ns,
                                    cfg.name);
            // Compose three rotations (applied right-to-left in the
            // quaternion product, i.e. body→optical first, then pitch in
            // post-b2o "panel-body" frame, then yaw in lidar_frame):
            //   q = R_z(yaw) * R_y(-pitch) * R_b2o
            //   R_b2o (rpy = -π/2, 0, -π/2): body axes → optical axes
            //   pitch convention: positive pitch_rad → panel looks UP.
            tf2::Quaternion q_b2o, q_pitch, q_yaw;
            q_b2o.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
            q_pitch.setRPY(0.0, -cfg.pitch_rad, 0.0);
            q_yaw.setRPY(0.0, 0.0, cfg.yaw_rad);
            tf2::Quaternion q = q_yaw * q_pitch * q_b2o;
            q.normalize();
            tf.transform.translation.x = 0.0;
            tf.transform.translation.y = 0.0;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = tf2::toMsg(q);
            transforms.push_back(tf);
        }
        static_tf_broadcaster_->sendTransform(transforms);
    }

    static std::string substitute_template(const std::string& tmpl,
                                           const std::string& ns,
                                           const std::string& name) {
        std::string out = tmpl;
        for (auto& kv : std::map<std::string, std::string>{
                 {"{ns}", ns}, {"{name}", name}}) {
            size_t pos = 0;
            while ((pos = out.find(kv.first, pos)) != std::string::npos) {
                out.replace(pos, kv.first.size(), kv.second);
                pos += kv.second.size();
            }
        }
        return out;
    }

    static std::map<std::string, std::string> pinhole_channel_topics(
        int n_returns) {
        std::map<std::string, std::string> m{
            {ouster::sdk::core::ChanField::RANGE, "range_image"},
            {ouster::sdk::core::ChanField::SIGNAL, "signal_image"},
            {ouster::sdk::core::ChanField::REFLECTIVITY, "reflec_image"},
            {ouster::sdk::core::ChanField::NEAR_IR, "nearir_image"}};
        if (n_returns == 2) {
            m[ouster::sdk::core::ChanField::RANGE2] = "range_image2";
            m[ouster::sdk::core::ChanField::SIGNAL2] = "signal_image2";
            m[ouster::sdk::core::ChanField::REFLECTIVITY2] = "reflec_image2";
        }
        return m;
    }

    struct PanelPublishers {
        std::string name;
        std::map<std::string,
                 rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
            image_pubs;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
            camera_info_pub;
    };

    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub_;
    LidarPacketHandler::HandlerType lidar_packet_handler_;
    std::vector<PanelPublishers> panel_pubs_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::vector<PinholeProcessor::PanelConfig> panel_configs_for_tf_;
    std::string lidar_namespace_for_tf_;
    std::string optical_frame_template_for_tf_;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterPinhole)
