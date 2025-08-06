/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file imu-model.h
 * @brief provide orientation esimtate based on imu data
 */

#pragma once

#include <Eigen/Core>

// A class that can estimates the current orientation based on imu telemetry
class ImuModel {
   public:
    /*
     * TODO: Add description
     * param: orientation represented as euler
     * TODO: add initial angular velocity
    */
    virtual void set_initial_state(const Eigen::Vector3d& initial_orientation) = 0;

    /*
     * TODO: Update
     * params la: linear acceleration
     * params av: angular velocity
    */
    virtual Eigen::Quaterniond update(uint64_t ts, const Eigen::Vector3d& la, const Eigen::Vector3d& av) = 0;
};


class SimpleImuModel : public ImuModel {
   public:
    void set_initial_state(const Eigen::Vector3d& initial_orientation) override {
        this->initial_orientation = initial_orientation;
    }

    Eigen::Quaterniond update(uint64_t ts, const Eigen::Vector3d& la, const Eigen::Vector3d& av) override {
        // TODO: compute dt from consecutive ts
        double dt = 0.01;

        Eigen::Vector3d orientation = initial_orientation + av * dt;
        orientation.x() += atan2(la.y(), la.z());
        orientation.y() += atan2(-la.x(), sqrt(la.y() * la.y() + la.z() * la.z()));

        Eigen::Quaterniond q =
            Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ());
        return q;
    }

private:
    // using euler angles for now
    Eigen::Vector3d initial_orientation;
};