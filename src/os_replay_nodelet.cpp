/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_replay_nodelet.cpp
 * @brief This nodelet mainly handles publishing saved metadata
 *
 */

#include <pluginlib/class_list_macros.h>

#include "ouster_ros/os_sensor_nodelet_base.h"

namespace sensor = ouster::sensor;

namespace ouster_ros {

class OusterReplay : public OusterSensorNodeletBase {
   private:
    virtual void onInit() override {
        auto meta_file = get_meta_file();
        create_metadata_pub();
        load_metadata_from_file(meta_file);
        publish_metadata();
        create_get_metadata_service();
        NODELET_INFO("Running in replay mode");
    }

    std::string get_meta_file() const {
        auto meta_file =
            getPrivateNodeHandle().param("metadata", std::string{});
        if (!is_arg_set(meta_file)) {
            NODELET_FATAL("Must specify metadata file in replay mode");
            throw std::runtime_error("metadata not specificed");
        }
        return meta_file;
    }

    void load_metadata_from_file(const std::string& meta_file) {
        try {
            cached_metadata = read_text_file(meta_file);
            info = sensor::parse_metadata(cached_metadata);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            cached_metadata.clear();
            NODELET_ERROR_STREAM(
                "Error when running in replay mode: " << e.what());
        }
    }
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterReplay, nodelet::Nodelet)