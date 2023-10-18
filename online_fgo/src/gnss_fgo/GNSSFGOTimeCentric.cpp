// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//


#include "gnss_fgo/GNSSFGOTimeCentric.h"


namespace gnss_fgo {

    GNSSFGOTimeCentricNode::GNSSFGOTimeCentricNode(const rclcpp::NodeOptions &opt)
        : GNSSFGOLocalizationBase("GNSSFGOTimeCentricNode", opt)
    {
      RCLCPP_INFO(this->get_logger(), "---------------------  GNSSFGOTimeCentricNode initializing! --------------------- ");
      std::lock_guard lk(allBufferMutex_);

      paramsPtr_ = std::make_shared<GNSSFGOParams>();

      if (!this->initializeCommon()) {
        RCLCPP_ERROR(this->get_logger(), "initializeParameters went wrong. Please check the parameters in config.");
        return;
      }

      callbackGroupMap_.insert(std::make_pair("PVTDelayCalculator", this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
      callbackGroupMap_.insert(std::make_pair("PPS", this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
      callbackGroupMap_.insert(std::make_pair("PVT", this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));

      PVTDelayCalculator_ = std::make_unique<fgo::utils::MeasurementDelayCalculator>(*this,
                                                                                     callbackGroupMap_["PVTDelayCalculator"],
                                                                                     false);

      auto subPPSopt = rclcpp::SubscriptionOptions();
      subPPSopt.callback_group = callbackGroupMap_["PPS"];
      subPPS_ = this->create_subscription<irt_nav_msgs::msg::PPS>("/irt_gpio_novatel/jetson_pps",
                                                              rclcpp::SystemDefaultsQoS(),
                                                              [this](const irt_nav_msgs::msg::PPS::ConstSharedPtr msg) -> void
                                                              {
                                                                  PVTDelayCalculator_->setPPS(msg);
                                                              },
                                                              subPPSopt);

      auto subPVTOpt = rclcpp::SubscriptionOptions();
      subPVTOpt.callback_group = callbackGroupMap_["PVT"];
      subPVA_ = this->create_subscription<irt_nav_msgs::msg::PVAGeodetic>("/irt_gnss_preprocessing/PVT",
                                                                      rclcpp::SensorDataQoS(),
                                                                      [this](const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvtMsg)->void
                                                                      {
        this->onPVAMsgCb(pvtMsg);
                                                                      },
                                                                      subPVTOpt);

      // User llh and clock error for gnss preprocessing
      userEstimationPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("user_estimation",
                                                                                    rclcpp::SystemDefaultsQoS());
      pubUserEstimationTimer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                        [this]()->void
                                                        {
                                                            if(!isStateInited_)
                                                              return;
                                                            std_msgs::msg::Float64MultiArray user_est_msg;
                                                            user_est_msg.data.resize(7);
                                                            fgo::data_types::UserEstimation_T user_est = userEstimationBuffer_.get_last_buffer();
                                                            std::copy(user_est.begin(), user_est.end(), user_est_msg.data.begin());
                                                            userEstimationPub_->publish(user_est_msg);
                                                        });

      RCLCPP_INFO(this->get_logger(), "---------------------  GNSSFGOTimeCentricNode initialized! --------------------- ");
    }

    void GNSSFGOTimeCentricNode::onPVAMsgCb(const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvaMsg)
    {
        auto thisPVATime = rclcpp::Time(pvaMsg->header.stamp.sec, pvaMsg->header.stamp.nanosec, RCL_ROS_TIME);
        static double lastDelay = 0.;
        static auto lastPVATime = thisPVATime;
        static bool first_measurement = true;
        sensor_msgs::msg::NavSatFix pvtNavMsg;
        pvtNavMsg.header.stamp = thisPVATime;
        pvtNavMsg.latitude = pvaMsg->phi_geo * fgo::constants::rad2deg;
        pvtNavMsg.longitude = pvaMsg->lambda_geo * fgo::constants::rad2deg;
        pvtNavMsg.altitude= pvaMsg->h_geo;
        pvtTestPub_->publish(pvtNavMsg);

        PVTDelayCalculator_->setTOW(pvaMsg->tow);

        fgo::data_types::PVASolution pva;
        pva.tow = pvaMsg->tow;
        pva.llh = gtsam::Point3(pvaMsg->phi_geo, pvaMsg->lambda_geo, pvaMsg->h_geo);
        pva.xyz_ecef = fgo::utils::llh2xyz(pva.llh);
        pva.nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix_asLLH(pva.llh));
        const auto eRn = pva.nRe.inverse();
        pva.has_velocity = true;
        pva.undulation = pvaMsg->undulation;
        pva.xyz_var = gtsam::Vector3(pvaMsg->phi_geo_var * pvaMsg->phi_geo_var,
                                   pvaMsg->lambda_geo_var * pvaMsg->lambda_geo_var,
                                   pvaMsg->h_geo_var *  pvaMsg->h_geo_var);
        pva.has_heading = true;
        pva.heading = fgo::utils::deg2rad * pvaMsg->yaw;
        pva.rot = gtsam::Rot3::Yaw(pva.heading);
        pva.rot_ecef = eRn.compose(pva.rot);
        pva.cog = pvaMsg->cog;
        pva.heading_var = std::pow(fgo::utils::deg2rad * pvaMsg->yaw_var, 2); // in deg^2
        pva.has_roll_pitch = true;
        pva.roll_pitch = pvaMsg->pitch_roll;
        pva.roll_pitch_var = pvaMsg->pitch_roll_var * pvaMsg->pitch_roll_var;
        pva.rot_var = gtsam::Vector3(pva.roll_pitch_var, pva.roll_pitch_var, pva.heading_var);
        pva.wnc = pvaMsg->wnc;
        pva.error = pvaMsg->error;
        pva.type = fgo::utils::GNSS::getSBFSolutionType(pva.error, pvaMsg->mode);
        pva.vel_n = gtsam::Vector3(pvaMsg->vn, pvaMsg->ve, -pvaMsg->vu);
        pva.vel_ecef = eRn.rotate(pva.vel_n);
        pva.vel_var = gtsam::Vector3(1, 1, 1);

        pva.clock_bias = pvaMsg->rx_clk_bias;
        pva.clock_bias_var = std::pow(pvaMsg->rx_clk_bias_var, 2);
        pva.clock_drift = pvaMsg->rx_clk_drift;
        pva.clock_drift_var = pvaMsg->rx_clk_drift_var * pvaMsg->rx_clk_drift_var;

        pva.num_sat = pvaMsg->nrsv;
        pva.num_sat_used = pvaMsg->nrsv_used;
        pva.num_sat_used_l1 = pvaMsg->nrsv_used_with_l1;
        pva.num_sat_used_multi = pvaMsg->nrsv_used_multi;
        pva.num_bases = pvaMsg->nr_bases;
        pva.reference_id = pvaMsg->reference_id;
        pva.correction_age = pvaMsg->diff_age;
        pva.solution_age = pvaMsg->sol_age;

        //rclcpp::sleep_for(std::chrono::nanoseconds(1000000));  // 10000000
        auto pvaDelay = this->PVTDelayCalculator_->getDelay()  + paramsPtr_->pvtMeasTimeOffset;

        auto delayFromMsg = (thisPVATime - lastPVATime).seconds() - 0.1;

        if(first_measurement)
        {
            first_measurement = false;
            delayFromMsg = 0.;
            pvaDelay = 0.;
        }

        if (delayFromMsg < -0.005 && lastDelay > 0)
            delayFromMsg += lastDelay;

        if (delayFromMsg < 0.)
            delayFromMsg = 0.;

        double pvaTimeCorrected = thisPVATime.seconds();

        if(!paramsPtr_->delayFromPPS)
        {
         pvaTimeCorrected -= delayFromMsg;
        }
        else
            pvaTimeCorrected -= pvaDelay;

        const auto pvaTime = rclcpp::Time(pvaTimeCorrected * fgo::constants::sec2nanosec, RCL_ROS_TIME);

        pva.timestamp = pvaTime;
        pvaSolutionBuffer_.update_buffer(pva, pvaTime);

        if(abs(delayFromMsg) < 0.005 || delayFromMsg > 0.3)
            lastDelay = 0.;
        else
            lastDelay = delayFromMsg;
        lastPVATime = thisPVATime;
    }



}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_fgo::GNSSFGOTimeCentricNode)

