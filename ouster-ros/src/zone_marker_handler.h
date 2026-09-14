/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file zone_marker_handler.h
 * @brief Builds visualization_msgs markers representing the sensor's zone
 * monitoring geometry, colored according to the current zone status.
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace ouster_ros {

/**
 * Builds and republishes one visualization_msgs Marker pair (a translucent
 * mesh plus a text label) per configured zone, using the STL geometry carried
 * by the sensor's zone set. Marker color and label text are refreshed from
 * every ZoneStatus message; the underlying mesh geometry is only computed
 * once, at construction time, since it never changes for the lifetime of a
 * given sensor metadata.
 *
 * Zones whose STL is expressed in the BODY coordinate frame are transformed
 * into the sensor frame using the zone set's sensor_to_body_transform so that
 * the resulting markers can be published directly against the sensor tf
 * frame, alongside the point cloud. Zones without STL geometry (ZRB-only)
 * can't be rendered as a mesh and are skipped.
 */
class ZoneMarkerHandler {
   public:
    using HandlerOutput = visualization_msgs::msg::MarkerArray;
    using HandlerType =
        std::function<HandlerOutput(const ouster_sensor_msgs::msg::ZoneStatus&)>;

   private:
    // Static, per-zone geometry computed once from the sensor's zone set.
    struct ZoneGeometry {
        uint8_t zone_id;
        std::string label;
        visualization_msgs::msg::Marker mesh_marker;   // TRIANGLE_LIST
        visualization_msgs::msg::Marker label_marker;  // TEXT_VIEW_FACING
    };

    static geometry_msgs::msg::Point to_point(const ouster::sdk::core::Coord& c) {
        geometry_msgs::msg::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = c.z();
        return p;
    }

    static std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a) {
        std_msgs::msg::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        return c;
    }

    // Transforms a point expressed in the zone set's BODY frame into the
    // sensor frame using the inverse of sensor_to_body_transform (which maps
    // sensor-frame points into the body frame).
    static ouster::sdk::core::Coord body_to_sensor(
        const ouster::sdk::core::Coord& p_body,
        const ouster::sdk::core::mat4d& sensor_to_body_transform) {
        Eigen::Vector4d p(p_body.x(), p_body.y(), p_body.z(), 1.0);
        Eigen::Vector4d p_sensor = sensor_to_body_transform.inverse() * p;
        return ouster::sdk::core::Coord(static_cast<float>(p_sensor.x()),
                                        static_cast<float>(p_sensor.y()),
                                        static_cast<float>(p_sensor.z()));
    }

    static std::vector<ZoneGeometry> build_zone_geometry(
        const ouster::sdk::core::SensorInfo& info, const std::string& frame) {
        using ouster::sdk::core::Stl;
        std::vector<ZoneGeometry> out;
        if (!info.zone_set) return out;

        for (const auto& kv : info.zone_set->zones) {
            // zone ids reported within a zone monitoring packet are 8 bits wide
            if (kv.first > std::numeric_limits<uint8_t>::max()) continue;
            const auto zone_id = static_cast<uint8_t>(kv.first);
            const auto& zone = kv.second;

            // only zones with STL geometry can be rendered as a mesh; a
            // ZRB-only zone only carries a rendered range image, not a shape
            if (!zone.stl) continue;
            if (zone.stl->coordinate_frame == Stl::CoordinateFrame::NONE)
                continue;

            auto mesh = zone.stl->to_mesh();
            const auto& triangles = mesh.triangles();
            if (triangles.empty()) continue;

            bool is_body_frame =
                zone.stl->coordinate_frame == Stl::CoordinateFrame::BODY;
            const auto& sensor_to_body =
                info.zone_set->sensor_to_body_transform;

            auto transform_point = [&](const ouster::sdk::core::Coord& c) {
                return is_body_frame ? body_to_sensor(c, sensor_to_body) : c;
            };

            ZoneGeometry geom;
            geom.zone_id = zone_id;
            geom.label = zone.label;

            auto& mesh_marker = geom.mesh_marker;
            mesh_marker.header.frame_id = frame;
            mesh_marker.ns = "ouster_zones";
            mesh_marker.id = zone_id;
            mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            mesh_marker.action = visualization_msgs::msg::Marker::ADD;
            mesh_marker.pose.orientation.w = 1.0;
            mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = 1.0;
            mesh_marker.frame_locked = true;
            mesh_marker.points.reserve(triangles.size() * 3);
            for (const auto& tri : triangles) {
                for (const auto& c : tri.coords) {
                    mesh_marker.points.push_back(to_point(transform_point(c)));
                }
            }

            auto center = transform_point(mesh.bounding_sphere().first);
            auto radius = mesh.bounding_sphere().second;

            auto& label_marker = geom.label_marker;
            label_marker.header.frame_id = frame;
            label_marker.ns = "ouster_zone_labels";
            label_marker.id = zone_id;
            label_marker.type =
                visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label_marker.action = visualization_msgs::msg::Marker::ADD;
            label_marker.pose.position = to_point(center);
            // lift the label above the zone volume so it isn't buried in it
            label_marker.pose.position.z += radius + 0.1;
            label_marker.pose.orientation.w = 1.0;
            label_marker.scale.z = 0.25;  // text height, meters
            label_marker.frame_locked = true;

            out.push_back(std::move(geom));
        }
        return out;
    }

    static std_msgs::msg::ColorRGBA mesh_color(
        const ouster_sensor_msgs::msg::ZoneState& zone_state) {
        using ZoneState = ouster_sensor_msgs::msg::ZoneState;
        if (zone_state.error_flags != 0)
            return make_color(1.0f, 0.6f, 0.0f, 0.5f);  // orange: degraded
        if (zone_state.trigger_status == ZoneState::TRIGGER_STATUS_ASSERTED)
            return make_color(1.0f, 0.0f, 0.0f, 0.5f);  // red: triggered
        return make_color(0.0f, 1.0f, 0.0f, 0.35f);      // green: clear
    }

    static std::string label_text(const std::string& label,
                                  const ouster_sensor_msgs::msg::ZoneState& zone_state) {
        using ZoneState = ouster_sensor_msgs::msg::ZoneState;
        std::string text =
            label.empty() ? ("zone " + std::to_string(zone_state.id)) : label;
        text += zone_state.trigger_status == ZoneState::TRIGGER_STATUS_ASSERTED
                   ? "\nTRIGGERED"
                   : "\nclear";
        text += "\ncount: " + std::to_string(zone_state.count);
        if (zone_state.error_flags != 0) text += "\nerror";
        return text;
    }

   public:
    static HandlerType create(const ouster::sdk::core::SensorInfo& info,
                              const std::string& frame) {
        auto geometry = build_zone_geometry(info, frame);

        return [geometry](const ouster_sensor_msgs::msg::ZoneStatus& status) {
            HandlerOutput out;
            out.markers.reserve(geometry.size() * 2);
            for (const auto& geom : geometry) {
                auto it = std::find_if(
                    status.zones.begin(), status.zones.end(),
                    [&](const ouster_sensor_msgs::msg::ZoneState& z) {
                        return z.live && z.id == geom.zone_id;
                    });

                auto mesh_marker = geom.mesh_marker;
                auto label_marker = geom.label_marker;
                mesh_marker.header.stamp = status.header.stamp;
                label_marker.header.stamp = status.header.stamp;

                if (it == status.zones.end()) {
                    // the zone slot isn't live in this status (e.g. sensor
                    // reconfigured with fewer zones); clear any marker we may
                    // have previously published for it
                    mesh_marker.action = visualization_msgs::msg::Marker::DELETE;
                    label_marker.action = visualization_msgs::msg::Marker::DELETE;
                } else {
                    mesh_marker.color = mesh_color(*it);
                    label_marker.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);
                    label_marker.text = label_text(geom.label, *it);
                }

                out.markers.push_back(std::move(mesh_marker));
                out.markers.push_back(std::move(label_marker));
            }
            return out;
        };
    }
};

}  // namespace ouster_ros
