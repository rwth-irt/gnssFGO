//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//

#ifndef GNSSFGOBOREAS_H
#define GNSSFGOBOREAS_H

#pragma once

//general
#include <algorithm>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <irt_nav_msgs/msg/fgo_state.hpp>
#include <irt_nav_msgs/msg/error2_gt.hpp>
#include <irt_nav_msgs/msg/elapsed_time_fgo.hpp>
#include <irt_nav_msgs/msg/pps.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/inscov.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

//gtsam
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Value.h>

//fgo
#include "GNSSFGOLocalizationBase.h"
#include "graph/GraphTimeCentric.h"
#include "param/GNSSFGOParams.h"
#include "data/DataTypes.h"
#include "data/Buffer.h"
#include "utils/Constants.h"
#include "utils/GNSSUtils.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
using gtsam::symbol_shorthand::N;  // integer ambiguities
using gtsam::symbol_shorthand::M;  // integer ddambiguities
using gtsam::symbol_shorthand::A;  // acceleration
using gtsam::symbol_shorthand::O;

namespace gnss_fgo
{
    class GNSSFGOBoreasNode : public GNSSFGOLocalizationBase
    {
    protected:
        fgo::buffer::CircularDataBuffer<fgo::data_types::PVASolution> pvaBuffer_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subPVAGT_;

    protected:
        void onPVAGTMsgCb(nav_msgs::msg::Odometry::ConstSharedPtr pva);
        void calculateErrorOnState(const fgo::data_types::State& state) override;

    public:
        explicit GNSSFGOBoreasNode(const rclcpp::NodeOptions &opt);
    };

}

#endif //GNSSFGOBOREAS_H
