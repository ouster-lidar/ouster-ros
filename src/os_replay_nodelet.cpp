/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_replay_nodelet.cpp
 * @brief This nodelet mainly handles publishing saved metadata
 *
 */

#include <fstream>
#include <pluginlib/class_list_macros.h>
#include "ouster_ros/os_client_base_nodelet.h"

namespace sensor = ouster::sensor;

namespace nodelets_os {

class OusterReplay : public OusterClientBase {
   private:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();
        auto meta_file = pnh.param("metadata", std::string{});
        if (!is_arg_set(meta_file)) {
            NODELET_ERROR("Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }

        NODELET_INFO("Running in replay mode");

        // populate info for config service
        try {
            std::ifstream in_file(meta_file);
            std::stringstream buffer;
            buffer << in_file.rdbuf();
            cached_metadata = buffer.str();
            info = sensor::parse_metadata(cached_metadata);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            cached_metadata.clear();
            NODELET_ERROR("Error when running in replay mode: %s", e.what());
        }

        OusterClientBase::onInit();
    }
};

}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterReplay, nodelet::Nodelet)