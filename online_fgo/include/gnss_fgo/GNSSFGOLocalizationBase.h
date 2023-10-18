//  Copyright 2023 Institute of Automatic Control RWTH Aachen University
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
#ifndef GNSSFGOLOCALIZATIONBASE_H
#define GNSSFGOLOCALIZATIONBASE_H

#pragma once

//general
#include <algorithm>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <GeographicLib/Geodesic.hpp>
//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <pluginlib/class_loader.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <irt_nav_msgs/msg/fgo_state.hpp>
#include <irt_nav_msgs/msg/error2_gt.hpp>
#include <irt_nav_msgs/msg/elapsed_time_fgo.hpp>
#include <irt_nav_msgs/msg/pps.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

//gtsam
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Value.h>

//boost
#include <boost/circular_buffer.hpp>

//fgo
#include "param/GNSSFGOParams.h"
#include "data/DataTypes.h"
#include "data/Buffer.h"
#include "graph/GraphBase.h"
#include "graph/GraphTimeCentric.h"
#include "graph/GraphSensorCentric.h"
#include "utils/Constants.h"
#include "utils/GNSSUtils.h"
#include "utils/NavigationTools.h"

#include "utils/MeasurmentDelayCalculator.h"
#include "utils/ROSParameter.h"

//third party
#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"
#include "InitGyroBias_ert_rtw/InitGyroBias.h"
#include "InitStatePVT_ert_rtw/InitStatePVT.h"

namespace gnss_fgo
{
    using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
    using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
    using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
    using gtsam::symbol_shorthand::N;  // integer ambiguities
    using gtsam::symbol_shorthand::M;  // integer ddambiguities
    using gtsam::symbol_shorthand::A;  // acceleration
    using gtsam::symbol_shorthand::O;


    class GNSSFGOLocalizationBase : public rclcpp::Node
    {
    public:
        GNSSFGOLocalizationBase(const std::string& name, const rclcpp::NodeOptions &opt) : rclcpp::Node(name, opt) {};
        ~GNSSFGOLocalizationBase() override = default;
        GNSSFGOParamsPtr getParamPtr() {return paramsPtr_;}

    protected:
        friend class fgo::graph::GraphBase;
        // Param
        GNSSFGOParamsPtr paramsPtr_;

        // Graph and utils
        fgo::graph::GraphBase::Ptr graph_;

        // Buffer containers and utils:
        fgo::buffer::CircularDataBuffer<fgo::data_types::IMUMeasurement> imuDataBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::PVASolution> pvaSolutionBuffer_;
        fgo::buffer::CircularDataBuffer<std::array<double, 3>> initGyroBiasBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::UserEstimation_T> userEstimationBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> fgoPredStateBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> fgoOptStateBuffer_;
        fgo::data_types::State lastOptimizedState_;
        fgo::data_types::State currentPredState_;

        // IMU utils
        std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> currentIMUPreintegrator_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;
        std::unique_ptr<InitGyroBias> gyroBiasInitializer_;

        // ROS
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMU_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStatePredPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStateOptPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStateExtrapolatedPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::Error2GT>::SharedPtr pvtErrorPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::Error2GT>::SharedPtr pvtErrorFromPredPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::ElapsedTimeFGO>::SharedPtr timerPub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr userEstimationPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStatePredNavFixPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStateOptNavFixPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pvtInterpolatedPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pvtTestPub_;
        rclcpp::TimerBase::SharedPtr pubUserEstimationTimer_;

        // Control variables
        std::atomic_bool isDoingPropagation_ = true;
        std::atomic_bool isStateInited_{};

        std::condition_variable conDoOpt_;
        bool triggeredOpt_ = false;
        std::atomic_bool lastOptFinished_ = true;
        std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callbackGroupMap_;
        bool triggeredInit_ = false;
        std::atomic_bool lastInitFinished_ = true;
        rclcpp::Time lastInitROSTimestamp_{};
        std::condition_variable conDoInit_;

        // threading management
        std::shared_ptr<std::thread> optThread_;
        std::shared_ptr<std::thread> initFGOThread_;
        std::mutex allBufferMutex_;
        std::mutex currentIMUPreIntMutex_;

    protected:
        /***
         * Initialize common variables and ROS parameters
         * @return initialize successful
         */
        bool initializeCommon();

        /***
         * Initialize the prior state X_0 using GNSS PVA solution
         * @return initialize the prior state X_0 of FGO successful
         */
        virtual bool initializeFGO();

        /***
         * callback IMU, general function
         * @param imuMeasurement: imu message
         */
        virtual void onIMUMsgCb(const sensor_msgs::msg::Imu::ConstSharedPtr& imuMeasurement);

        /***
         * this function contains the endless loop for time-centric graph construction and optimization
         * this is an alternative of onTriggerOptimization. Only one of these functions will be called in the optThread_
         */
        void timeCentricFGO();

        /***
         * Trigger Optimization if the imu is used as the timing reference
         */
        void onTriggerOptimization();

        /***
         * Update the graph after graph construction
         * @return runtime taken for the optimization
         */
        double optimize();

        /***
         * calculate error metrics online using (linear) interpolated GNSS PVA solution if RTK-fixed mode is given
         * may be implemented differently
         * @param stateIn: current FGO state
         */
        virtual void calculateErrorOnState(const fgo::data_types::State& stateIn);

        /***
         * bypass the conditional waiting in onTriggerOptimization(), if the imu is used as the timing reference
         */
        void notifyOptimization()
        {
            lastOptFinished_ = false;
            triggeredOpt_ = true;
            conDoOpt_.notify_one();
        }

        /***
         * convert FGO position to navfix
         * @param fgoMsg FGO state
         * @param isAntMain indicate which leverarm should be used
         * @return msg
         */
        static sensor_msgs::msg::NavSatFix convertPositionToNavFixMsg(const irt_nav_msgs::msg::FGOState& fgoMsg, bool isAntMain = true)
        {

          sensor_msgs::msg::NavSatFix navfix_msg;
          navfix_msg.header.stamp = fgoMsg.header.stamp;
          if(isAntMain)
          {
            navfix_msg.header.frame_id = "gnss_ant_main";
            navfix_msg.latitude = fgoMsg.llh_ant_main[0] * fgo::constants::rad2deg;
            navfix_msg.longitude = fgoMsg.llh_ant_main[1] * fgo::constants::rad2deg;
            navfix_msg.altitude = fgoMsg.llh_ant_main[2];
          }
          else
          {
            navfix_msg.header.frame_id = "gnss_ant_aux";
            navfix_msg.latitude = fgoMsg.llh_ant_aux[0] * fgo::constants::rad2deg;
            navfix_msg.longitude = fgoMsg.llh_ant_aux[1] * fgo::constants::rad2deg;
            navfix_msg.altitude = fgoMsg.llh_ant_aux[2];
          }
          navfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          navfix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
          return navfix_msg;
        };

        /***
         * convert FGO position as gtsam navstate to navfix
         * @param state gtsam navstate
         * @param timestamp
         * @param toAnt1 ndicate which leverarm should be used
         * @return msg
         */
        [[nodiscard]] sensor_msgs::msg::NavSatFix convertPositionToNavFixMsg(const gtsam::NavState& state, const rclcpp::Time& timestamp, bool toAnt1) const
        {
          auto lb = toAnt1 ? paramsPtr_->transIMUToAnt1 : paramsPtr_->transIMUToAnt2;
          auto pos_ant = state.position() + state.attitude().rotate(lb);

          gtsam::Point3 llh = fgo::utils::xyz2llh(pos_ant);
          sensor_msgs::msg::NavSatFix navfix_msg;
          navfix_msg.header.stamp = timestamp;
          navfix_msg.header.frame_id = "gnss_ant_main";
          navfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          navfix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
          navfix_msg.latitude = llh.x() * fgo::constants::rad2deg;
          navfix_msg.longitude = llh.y() * fgo::constants::rad2deg;
          navfix_msg.altitude = llh.z();

          return navfix_msg;
        }

        /***
         * convert intern FGO state to ros message
         * @param state FGO state
         * @return msg
         */
        irt_nav_msgs::msg::FGOState convertFGOStateToMsg(const fgo::data_types::State &state) {
          static auto l_b_ant_main = paramsPtr_->transIMUToReference;
          static auto l_b_ant_aux = paramsPtr_->transIMUToAnt2;
          irt_nav_msgs::msg::FGOState state_msg;
          state_msg.header.frame_id = "imu";
          state_msg.header.stamp = state.timestamp;
          state_msg.pose.position.x = state.state.t().x();
          state_msg.pose.position.y = state.state.t().y();
          state_msg.pose.position.z = state.state.t().z();
          state_msg.pose.orientation.x = state.state.quaternion().x();
          state_msg.pose.orientation.y = state.state.quaternion().y();
          state_msg.pose.orientation.z = state.state.quaternion().z();
          state_msg.pose.orientation.w = state.state.quaternion().w();
          state_msg.vel.linear.x = state.state.v().x();
          state_msg.vel.linear.y = state.state.v().y();
          state_msg.vel.linear.z = state.state.v().z();
          state_msg.vel.angular.x = state.omega.x();
          state_msg.vel.angular.y = state.omega.y();
          state_msg.vel.angular.z = state.omega.z();

          auto fgo_ecef = state.state.position();
          auto fgo_ori  = state.state.attitude();
          gtsam::Vector2 fgo_cbd  = state.cbd;
          auto fgo_imu_bias = state.imuBias;

          gtsam::Rot3 nRe(fgo::utils::nedRe_Matrix(fgo_ecef));
          gtsam::Vector3 fgo_vel_ned = nRe.rotate(state.state.v() + fgo_ori.rotate(gtsam::skewSymmetric((-l_b_ant_main)) * state.omega));

          auto nRb_rpy = fgo::utils::func_DCM2EulerAngles((nRe.compose(fgo_ori)).matrix());

          auto fgo_yaw = nRb_rpy(2) * 180. / M_PI;
          fgo_yaw = (fgo_yaw >= 0. ? fgo_yaw : (fgo_yaw + 360.));
          auto pos_ant_main = fgo_ecef + fgo_ori.rotate(l_b_ant_main);
          auto llh_ant_main = fgo::utils::xyz2llh(pos_ant_main);
          state_msg.llh_ant_main[0] = llh_ant_main.x();
          state_msg.llh_ant_main[1] = llh_ant_main.y();
          state_msg.llh_ant_main[2] = llh_ant_main.z();
          state_msg.rpy_ant_main[0] = nRb_rpy(0) * 180. / M_PI;
          state_msg.rpy_ant_main[1] = nRb_rpy(1) * 180. / M_PI;
          state_msg.rpy_ant_main[2] = fgo_yaw;
          state_msg.vel_ned_ant_main[0] = fgo_vel_ned.x();
          state_msg.vel_ned_ant_main[1] = fgo_vel_ned.y();
          state_msg.vel_ned_ant_main[2] = fgo_vel_ned.z();

          gtsam::Vector3 fgo_vel_ned_ant_aux = nRe.rotate(state.state.v() + fgo_ori.rotate((gtsam::skewSymmetric((-l_b_ant_aux)) * state.omega)));
          auto pos_ant_aux = fgo_ecef + fgo_ori.rotate(l_b_ant_aux);
          auto llh_ant_aux = fgo::utils::xyz2llh(pos_ant_aux);
          state_msg.llh_ant_aux[0] = llh_ant_aux.x();
          state_msg.llh_ant_aux[1] = llh_ant_aux.y();
          state_msg.llh_ant_aux[2] = llh_ant_aux.z();
          state_msg.rpy_ant_aux[0] = nRb_rpy(0) * 180. / M_PI;
          state_msg.rpy_ant_aux[1] = nRb_rpy(1) * 180. / M_PI;
          state_msg.rpy_ant_aux[2] = fgo_yaw;
          state_msg.vel_ned_ant_aux[0] = fgo_vel_ned_ant_aux.x();
          state_msg.vel_ned_ant_aux[1] = fgo_vel_ned_ant_aux.y();
          state_msg.vel_ned_ant_aux[2] = fgo_vel_ned_ant_aux.z();

          try {
            state_msg.pose_var = std::vector(state.poseVar.data(),
                                             state.poseVar.data() + state.poseVar.rows() * state.poseVar.cols());
            state_msg.vel_var = std::vector(state.velVar.data(),
                                            state.velVar.data() + state.velVar.rows() * state.velVar.cols());
            //gtsam::Vector::Map(&state_msg.vel_var[0], state.velVar.size()) = state.velVar;
            //state_msg.imu_bias.resize(state.imuBias.vector().size());
            //gtsam::Vector::Map(&state_msg.imu_bias[0], state.imuBias.vector().size()) = state.imuBias.vector();
            state_msg.imu_bias = std::vector(state.imuBias.vector().data(),
                                             state.imuBias.vector().data() +
                                             state.imuBias.vector().rows() * state.imuBias.vector().cols());
            //state_msg.imu_bias_var.resize(state.imuBiasVar.size());
            //gtsam::Vector::Map(&state_msg.imu_bias_var[0], state.imuBiasVar.size()) = state.imuBiasVar;
            state_msg.imu_bias_var = std::vector(state.imuBiasVar.data(),
                                                 state.imuBiasVar.data() +
                                                 state.imuBiasVar.rows() * state.imuBiasVar.cols());
            state_msg.cbd.resize(state.cbd.size());
            gtsam::Vector::Map(&state_msg.cbd[0], state.cbd.size()) = state.cbd;
            //state_msg.cbd_var.resize(state.cbdVar.size());
            //gtsam::Vector::Map(&state_msg.cbd_var[0], state.cbdVar.size()) = state.cbdVar;
            state_msg.cbd_var = std::vector(state.cbdVar.data(),
                                            state.cbdVar.data() + state.cbdVar.rows() * state.cbdVar.cols());
            state_msg.amb = std::vector<uint32_t>(state.ddIntAmb.data(),
                                                  state.ddIntAmb.data() + state.ddIntAmb.rows() * state.ddIntAmb.cols()); //TODO error amb CAN be neg, because it is DD
            state_msg.amb_var = std::vector(state.ddIntAmbVar.data(),
                                            state.ddIntAmbVar.data() + state.ddIntAmbVar.rows() * state.ddIntAmbVar.cols());
          }
          catch (std::exception &ex) {
            RCLCPP_WARN(rclcpp::get_logger("online_fgo"), "Publishing: Error while copy data...");
          }
          state_msg.cog =
              state.state.attitude().yaw() > 0 ? state.state.attitude().yaw() : state.state.attitude().yaw() + 360.;
          return state_msg;
        }
    };

}
#endif //GNSSFGOLOCALIZATIONBASE_H
