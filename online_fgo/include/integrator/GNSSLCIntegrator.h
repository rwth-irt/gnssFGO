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

#ifndef ONLINE_FGO_INTEGRATEGNSSLC_H
#define ONLINE_FGO_INTEGRATEGNSSLC_H

#pragma once

#include <irt_nav_msgs/msg/pps.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/fgo_state.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/bestvel.hpp>
#include <novatel_oem7_msgs/msg/dualantennaheading.hpp>
#include <novatel_oem7_msgs/msg/clockmodel.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/inscov.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include "IntegratorBase.h"

#include "factor/measurement/gnss/GPSFactor.h"
#include "factor/measurement/gnss/GPInterpolatedGPSFactor.h"
#include "factor/measurement/gnss/PVTFactor.h"
#include "factor/measurement/gnss/GPInterpolatedPVTFactor.h"

#include "utils/GNSSUtils.h"
#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"

#include "third_party/matlab_utils.h"

namespace fgo::integrator
{
    using namespace ::utils;
    class GNSSLCIntegrator : public IntegratorBase
    {
        IntegratorGNSSLCParamsPtr paramPtr_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolator_;

        fgo::buffer::CircularDataBuffer<fgo::data_types::PVASolution> GNSSPVABuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::PVASolution> referencePVTBuffer_;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavfix_;

        typedef message_filters::sync_policies::ApproximateTime<novatel_oem7_msgs::msg::BESTPOS, novatel_oem7_msgs::msg::BESTVEL,
                                                                novatel_oem7_msgs::msg::DUALANTENNAHEADING> OEM7DualAntennaSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<novatel_oem7_msgs::msg::BESTPOS, novatel_oem7_msgs::msg::BESTVEL> OEM7SyncPolicy;

        message_filters::Subscriber<ublox_msgs::msg::NavPVT> subUbloxNavPVT_;
        message_filters::Subscriber<novatel_oem7_msgs::msg::BESTPOS> subNovatelBestpos_;
        message_filters::Subscriber<novatel_oem7_msgs::msg::BESTVEL> subNovatelBestvel_;
        message_filters::Subscriber<novatel_oem7_msgs::msg::DUALANTENNAHEADING> subNovatelHeading_;
        rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subNovatelBestposAlone_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr pubPVAInFGOStata_;

        rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr subUbloxPVT_;
        std::unique_ptr<message_filters::Synchronizer<OEM7DualAntennaSyncPolicy>> novatelPVTDualAntennaSync_;
        std::unique_ptr<message_filters::Synchronizer<OEM7SyncPolicy>> novatelPVTSync_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subPVAOdom_;

        rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr subNovatelPVA_;
        rclcpp::Subscription<irt_nav_msgs::msg::PVAGeodetic>::SharedPtr subPVA_;
        rclcpp::Subscription<irt_nav_msgs::msg::PPS>::SharedPtr subPPS_;
        std::unique_ptr<fgo::utils::MeasurementDelayCalculator> PVTDelayCalculator_;

        std::atomic_bool zeroVelocity_{};

    public:
        explicit GNSSLCIntegrator() = default;
        ~GNSSLCIntegrator() override = default;

        void initialize(rclcpp::Node& node, fgo::graph::GraphBase& graphPtr, const std::string& integratorName, bool isPrimarySensor = false) override;

        bool factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                       const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                       const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                       std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                       gtsam::Values& values,
                       fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
                       gtsam::KeyVector& relatedKeys) override;

        bool fetchResult(
            const gtsam::Values &result,
            const gtsam::Marginals &martinals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
            fgo::data_types::State& optState
        ) override;

        void dropMeasurementBefore(double timestamp) override{
          GNSSPVABuffer_.cleanBeforeTime(timestamp);
        }

        std::vector<fgo::data_types::PVASolution> getPVAData()
        {
          return GNSSPVABuffer_.get_all_buffer();
        }

        std::vector<fgo::data_types::PVASolution> getPVADataAndClean()
        {
          return GNSSPVABuffer_.get_all_buffer_and_clean();
        }

        bool checkZeroVelocity() override{
          return zeroVelocity_;
        }

        void cleanBuffers() override
        {
          const auto buffer = GNSSPVABuffer_.get_all_buffer_and_clean();
        }

        bool checkHasMeasurements() override
        {
          return referencePVTBuffer_.size() != 0;
        }


    protected:

        void addGNSSFactor(const gtsam::Key& poseKey,
                           const gtsam::Point3 &posMeasured,
                           const gtsam::Vector3 &posVar,
                           const gtsam::Vector3 &lb)
        {
          const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                          posVar, paramPtr_->robustParamPosition, "GPS");
          graphPtr_->emplace_shared<fgo::factor::GPSFactor>(poseKey, posMeasured, lb, noiseModel, paramPtr_->AutoDiffNormalFactor);
        }

        void addGPInterpolatedGNSSFactor(const gtsam::Key& poseKeyI, const gtsam::Key& velKeyI, const gtsam::Key& omegaKeyI,
                                         const gtsam::Key& poseKeyJ, const gtsam::Key& velKeyJ, const gtsam::Key& omegaKeyJ,
                                         const gtsam::Point3 &posMeasured, const gtsam::Vector3 &posVar, const gtsam::Vector3 &lb,
                                         const std::shared_ptr<fgo::models::GPInterpolator> &interpolator)
        {
          const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                          posVar, paramPtr_->robustParamPosition, "GPInterpolatedGPS");
          noiseModel->print("pvt noise");

            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedGPSFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ, posMeasured,
                                                                            lb, noiseModel, interpolator, false);
        }

        void addGNSSPVTFactor(const gtsam::Key& poseKey, const gtsam::Key& velKey, const gtsam::Key& omegaKey,
                              const gtsam::Point3& posMeasured, const gtsam::Vector3& velMeasured,
                              const gtsam::Vector3& posVar, const gtsam::Vector3& velVar, const gtsam::Vector3 &lb)
        {
          const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                          (gtsam::Vector6() << posVar, velVar).finished(),
                                                          paramPtr_->robustParamPosition);
            noiseModel->print("pvt noise");
          graphPtr_->emplace_shared<fgo::factor::PVTFactor>(poseKey, velKey, omegaKey,
                                                            posMeasured, velMeasured,
                                                            lb, paramPtr_->velocityFrame, noiseModel, false);
        }

        void addGPInterpolatedGNSSPVTFactor(const gtsam::Key& poseKeyI, const gtsam::Key& velKeyI, const gtsam::Key& omegaKeyI,
                                            const gtsam::Key& poseKeyJ, const gtsam::Key& velKeyJ, const gtsam::Key& omegaKeyJ,
                                            const gtsam::Point3& posMeasured, const gtsam::Vector3& velMeasured,
                                            const gtsam::Vector3& posVar, const gtsam::Vector3& velVar, const gtsam::Vector3 &lb,
                                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator)
        {
          const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                          (gtsam::Vector6() << posVar, velVar).finished(),
                                                          paramPtr_->robustParamPosition);

            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedPVTFactor>(poseKeyI, velKeyI, omegaKeyI,
                                                                            poseKeyJ, velKeyJ, omegaKeyJ,
                                                                            posMeasured, velMeasured, lb, paramPtr_->velocityFrame,
                                                                            noiseModel, interpolator, paramPtr_->AutoDiffGPInterpolatedFactor);
        }

    private:

        void onOdomMsgCb(const nav_msgs::msg::Odometry::ConstSharedPtr pva)
        {
          // ATTENTION: here we use ENU as tangent frame
          fgo::data_types::PVASolution this_pva{};
          rclcpp::Time ts = rclcpp::Time(pva->header.stamp.sec, pva->header.stamp.nanosec, RCL_ROS_TIME);
          this_pva.timestamp = ts;
          this_pva.tow = ts.seconds();
          this_pva.type = fgo::data_types::GNSSSolutionType::RTKFIX;

          this_pva.llh = (gtsam::Vector3() << pva->pose.pose.position.x * fgo::constants::deg2rad,
                                              pva->pose.pose.position.y * fgo::constants::deg2rad,
                                              pva->pose.pose.position.z - 37.).finished();

          /* !
           *  TODO: variance for the pva: is this provided in the odometry msg?
           */

          this_pva.xyz_ecef = fgo::utils::llh2xyz(this_pva.llh);
          this_pva.xyz_var =  (gtsam::Vector3() << .5, 0.5, 2.).finished();
          //const auto nedRenu = gtsam::Rot3(fgo::utils::nedRenu_llh(this_pva.llh));
          const auto eRenu = gtsam::Rot3(fgo::utils::enuRe_Matrix_asLLH(this_pva.llh)).inverse();

          this_pva.vel_n = (gtsam::Vector3() << pva->twist.twist.linear.x,
                                                pva->twist.twist.linear.y,
                                                pva->twist.twist.linear.z).finished();
          this_pva.vel_ecef = eRenu.rotate(this_pva.vel_n);

          this_pva.rot = gtsam::Rot3::Quaternion(pva->pose.pose.orientation.w,
                                                 pva->pose.pose.orientation.x,
                                                 pva->pose.pose.orientation.y,
                                                 pva->pose.pose.orientation.z);

          this_pva.rot_ecef = eRenu.compose(this_pva.rot);
          this_pva.has_heading = true;
          this_pva.has_roll_pitch = true;
          this_pva.has_velocity_3D = true;
          this_pva.has_velocity = true;
          GNSSPVABuffer_.update_buffer(this_pva, ts);

          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(this_pva.tow, this_pva.timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onOdomMsgCb: graph not initialized, waiting ...");
          }

        }


        void onIRTPVTMsgCb(const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvaMsg) {
          const auto thisPVATime = rclcpp::Time(pvaMsg->header.stamp.sec, pvaMsg->header.stamp.nanosec, RCL_ROS_TIME);
          static double lastDelay = 0.;
          static auto lastPVATime = thisPVATime;
          static bool first_measurement = true;
          static size_t calcZeroVelocityCounter = 1;
          static gtsam::Vector3 sumVelocity = gtsam::Z_3x1;

          irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
          thisProcessingReport.ts_measurement = thisPVATime.seconds();
          const auto ts_start_processing = rosNodePtr_->now();
          thisProcessingReport.ts_start_processing = ts_start_processing.seconds();
          thisProcessingReport.sensor_name = "GNSSLC";
          thisProcessingReport.observation_available = true;

          //sensor_msgs::msg::NavSatFix pvtNavMsg;
          //pvtNavMsg.header.stamp = thisPVTTime;
          //pvtNavMsg.latitude = pvtMsg->phi_geo * fgo::constants::rad2deg;
          //pvtNavMsg.longitude = pvtMsg->lambda_geo * fgo::constants::rad2deg;
          //pvtNavMsg.altitude= pvtMsg->h_geo;
          //pvtTestPub_->publish(pvtNavMsg);

          if(pvaMsg->sol_age > 0.15)
          {
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ << " onIRTPVTMsgCb solution out of date: " << pvaMsg->sol_age);
            return;
          }

          PVTDelayCalculator_->setTOW(pvaMsg->tow);
          auto pva = fgo::data_types::PVASolution();
          pva.tow = pvaMsg->tow;
          pva.timestamp = thisPVATime;
          pva.has_heading = true;
          pva.has_velocity = true;
          pva.has_roll_pitch = true;
          pva.error = pvaMsg->error;
          pva.llh = (gtsam::Vector3() << pvaMsg->phi_geo, pvaMsg->lambda_geo, pvaMsg->h_geo).finished();
          pva.xyz_ecef = fgo::utils::llh2xyz(pva.llh);
          const auto eRn = gtsam::Rot3(fgo::utils::nedRe_Matrix(pva.xyz_ecef)).inverse();
          pva.xyz_var = (gtsam::Vector3() << std::pow(pvaMsg->phi_geo_var, 2), std::pow(pvaMsg->lambda_geo_var, 2), std::pow(pvaMsg->h_geo_var, 2)).finished();
          //pva.xyz_var = eRn.rotate(pva.xyz_var);
          pva.vel_n = (gtsam::Vector3() << pvaMsg->vn, pvaMsg->ve, -pvaMsg->vu).finished();
          pva.vel_ecef = eRn.rotate(pva.vel_n);
          pva.vel_var = (gtsam::Vector3() << paramPtr_->fixedVelVar, paramPtr_->fixedVelVar, paramPtr_->fixedVelVar).finished();
          pva.heading = pvaMsg->yaw * fgo::constants::deg2rad;
          pva.heading_var = std::pow(pvaMsg->yaw_var * fgo::constants::deg2rad, 2);

          sumVelocity += pva.vel_n;

          if((pvaMsg->yaw == 270. && pvaMsg->yaw_var == 0.) || pvaMsg->yaw_var > 10.)
          {
            pva.has_heading = false;
          }
          else
          {
            const auto heading_rot = gtsam::Rot3::Yaw(pva.heading);
            pva.heading_ecef = eRn.compose(heading_rot).yaw();
            pva.roll_pitch = pvaMsg->pitch_roll * fgo::constants::deg2rad;
            pva.roll_pitch_var = std::pow(pvaMsg->pitch_roll_var * fgo::constants::deg2rad, 2) * paramPtr_->headingVarScale;

            if(paramPtr_->hasRoll)
            {
              pva.rot = gtsam::Rot3::Ypr(pva.heading, 0., pva.roll_pitch);
              pva.rot_var = (gtsam::Vector3() << pva.roll_pitch_var, 0., pva.heading_var).finished();
            }
            else if(paramPtr_->hasPitch)
            {
              pva.rot = gtsam::Rot3::Ypr(pva.heading, pva.roll_pitch, 0.);
              pva.rot_var = (gtsam::Vector3() << 0., pva.roll_pitch_var, pva.heading_var).finished();
            }
            else
            {
              pva.rot = gtsam::Rot3::Yaw(pva.heading);
              pva.rot_var = (gtsam::Vector3() << 0., 0., pva.heading_var).finished();
            }

            pva.rot_ecef = eRn.compose(pva.rot);
          }
          pva.type = fgo::utils::GNSS::getSBFSolutionType(pvaMsg->error, pvaMsg->mode);
          //std::cout << "pvtMsg->error " << unsigned(pvtMsg->error) << std::endl;
          //std::cout << "pvtMsg->mode " << unsigned(pvtMsg->mode) << std::endl;
          //std::cout << "pva.type " << pva.type << std::endl;
          pva.num_sat = pvaMsg->nrsv_used_with_l1;

          //rclcpp::sleep_for(std::chrono::nanoseconds(1000000));  // 10000000
          auto pvtDelay = this->PVTDelayCalculator_->getDelay()  + paramPtr_->pvtMeasTimeOffset;

          auto delayFromMsg = (thisPVATime - lastPVATime).seconds() - 0.1;
          //
          //
          //std::cout << std::fixed << "pvt delay: " << pvtDelay << std::endl;

          if(first_measurement)
          {
            first_measurement = false;
            delayFromMsg = 0.;
            pvtDelay = 0.;
          }

          //std::cout << std::fixed << "pvt delay from msg: " << delayFromMsg << std::endl;

          if (delayFromMsg < -0.005 && lastDelay > 0)
            delayFromMsg += lastDelay;

          if (delayFromMsg < 0.)
            delayFromMsg = 0.;
          //std::cout << std::fixed << "pvt delay from msg: " << delayFromMsg << std::endl;

          double pvtTimeCorrected = thisPVATime.seconds();
          if(!paramPtr_->delayFromPPS)
          {
            pvtTimeCorrected -= delayFromMsg;
            pva.delay = delayFromMsg;
          }
          else
          {
            pvtTimeCorrected -= pvtDelay;
            pva.delay = pvtDelay;
          }

          fgo::data_types::State fgoState;
          fgoState.timestamp = rclcpp::Time(pvtTimeCorrected * fgo::constants::sec2nanosec, RCL_ROS_TIME);
          fgoState.state = gtsam::NavState(pva.rot_ecef, pva.xyz_ecef - pva.rot_ecef.rotate(paramPtr_->transIMUToAnt1), pva.vel_ecef);
          fgoState.cbd = (gtsam::Vector2() << pvaMsg->rx_clk_bias, pvaMsg->rx_clk_drift).finished();
          graphPtr_->updateReferenceState(fgoState, fgoState.timestamp);

          irt_nav_msgs::msg::FGOState state_msg;
          state_msg.header.frame_id = "antmain";
          state_msg.header.stamp = fgoState.timestamp;
          state_msg.amb_var.emplace_back(pva.tow);
          state_msg.amb_var.emplace_back(thisPVATime.seconds());
          state_msg.pose.position.x = fgoState.state.t().x();
          state_msg.pose.position.y = fgoState.state.t().y();
          state_msg.pose.position.z = fgoState.state.t().z();
          state_msg.vel.linear.x = fgoState.state.v().x();
          state_msg.vel.linear.y = fgoState.state.v().y();
          state_msg.vel.linear.z = fgoState.state.v().z();
          state_msg.vel.angular.x = fgoState.omega.x();
          state_msg.vel.angular.y = fgoState.omega.y();
          state_msg.vel.angular.z = fgoState.omega.z();
          state_msg.cbd.resize(fgoState.cbd.size());
          state_msg.cog = pva.heading;
          gtsam::Vector::Map(&state_msg.cbd[0], fgoState.cbd.size()) = fgoState.cbd;
          pubPVAInFGOStata_->publish(state_msg);

          GNSSPVABuffer_.update_buffer(pva, pva.timestamp);

          if(calcZeroVelocityCounter > 6)
          {
            const auto avgVelocity = (gtsam::Vector3() << sumVelocity.x() / calcZeroVelocityCounter,
                sumVelocity.y() / calcZeroVelocityCounter,
                sumVelocity.z() / calcZeroVelocityCounter).finished();
            //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), integratorName_ << " avg: velocity " << avgVelocity.norm());
            if(avgVelocity.norm() < paramPtr_->zeroVelocityThreshold)
            {
              RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ << " onIRTPVTMsgCb reported near zero velocity: " << pva.vel_n);
              zeroVelocity_ = true;
              GNSSPVABuffer_.clean();
            }
            else
              zeroVelocity_ = false;
            calcZeroVelocityCounter = 1;
            sumVelocity.setZero();
          }

          calcZeroVelocityCounter ++;

          if(abs(delayFromMsg) < 0.005 || delayFromMsg > 0.3)
            lastDelay = 0.;
          else
            lastDelay = delayFromMsg ;

          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(pva.tow, fgoState.timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onIRTPVTMsgCb: graph not initialized, waiting ...");
          }
          //std::cout << std::fixed << "pvt lsat delay : " << delayFromMsg << std::endl;
          thisProcessingReport.measurement_delay = pva.delay;
          thisProcessingReport.header.stamp = rosNodePtr_->now();
          thisProcessingReport.duration_processing = (rosNodePtr_->now() - ts_start_processing).seconds();
          thisProcessingReport.num_measurements = 1;
          if(pubSensorReport_)
              pubSensorReport_->publish(thisProcessingReport);

            lastPVATime = thisPVATime;
        }


        void onINSPVAXMsgCb(novatel_oem7_msgs::msg::INSPVAX::ConstSharedPtr pva)
        {
          rclcpp::Time msg_timestamp;
          if(paramPtr_->useHeaderTimestamp)
            msg_timestamp = rclcpp::Time(pva->header.stamp.sec, pva->header.stamp.nanosec, RCL_ROS_TIME);
          else
            msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.tow = pva->nov_header.gps_week_milliseconds * 0.001;
          sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(pva->pos_type.type);
          sol.llh = (gtsam::Vector3() << pva->latitude * fgo::constants::deg2rad, pva->longitude * fgo::constants::deg2rad, pva->height + pva->undulation).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.xyz_var = (gtsam::Vector3() << pva->latitude_stdev, pva->longitude_stdev, pva->height_stdev).finished();
          sol.vel_n = (gtsam::Vector3() << pva->north_velocity, pva->east_velocity, -pva->up_velocity).finished();
          //sol.vel_var = (gtsam::Vector3() << pva->north_velocity_stdev, pva->east_velocity_stdev, -pva->up_velocity_stdev).finished();



          const auto eRned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef)).inverse();
          sol.vel_ecef = eRned.rotate(sol.vel_n);
          sol.rot = gtsam::Rot3::Yaw(pva->azimuth * fgo::constants::deg2rad);

          sol.rot_ecef = eRned.compose(sol.rot);

          sol.rot_var = (gtsam::Vector3() << pva->azimuth_stdev * fgo::constants::deg2rad,
              pva->roll_stdev * fgo::constants::deg2rad,
              pva->pitch_stdev * fgo::constants::deg2rad).finished();
          sol.heading = -pva->azimuth * fgo::constants::deg2rad;
          sol.has_heading = true;
          sol.has_velocity = true;
          sol.has_roll_pitch = true;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onINSPVAXMsgCb: graph not initialized, waiting ...");
          }
        }

        void onOEM7PVTHeadingMsgCb(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos,
                                   const novatel_oem7_msgs::msg::BESTVEL::ConstSharedPtr bestvel,
                                   const novatel_oem7_msgs::msg::DUALANTENNAHEADING::ConstSharedPtr heading)
        {
          const auto msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.tow = bestpos->nov_header.gps_week_milliseconds * 0.001;
          sol.has_heading = true;
          sol.has_velocity = true;
          sol.llh = (gtsam::Vector3() << bestpos->lat * fgo::constants::deg2rad, bestpos->lon * fgo::constants::deg2rad, bestpos->hgt).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos->lat_stdev, 2), std::pow(bestpos->lon_stdev, 2), std::pow(bestpos->hgt_stdev, 2)).finished() * paramPtr_->posVarScale;
          const auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
          sol.vel_n = (gtsam::Vector3() << bestvel->hor_speed * std::cos(bestvel->trk_gnd * fgo::constants::deg2rad),
                                             bestvel->hor_speed * std::sin(bestvel->trk_gnd * fgo::constants::deg2rad),
                                             -bestvel->ver_speed).finished();
          sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
          sol.vel_var = (gtsam::Vector3() << paramPtr_->fixedVelVar, paramPtr_->fixedVelVar, paramPtr_->fixedVelVar).finished() * paramPtr_->velVarScale;
          sol.heading = heading->heading * fgo::constants::deg2rad;
          sol.heading_var = std::pow(heading->heading_std_dev * fgo::constants::deg2rad, 2) * paramPtr_->headingVarScale;
          auto heading_rot = gtsam::Rot3::Yaw(sol.heading);
          sol.heading_ecef = ecef_R_ned.compose(heading_rot).yaw();
          sol.roll_pitch = heading->pitch * fgo::constants::deg2rad;
          sol.roll_pitch_var = std::pow(heading->pitch_std_dev * fgo::constants::deg2rad, 2) * paramPtr_->headingVarScale;

          if(paramPtr_->hasRoll)
          {
            sol.rot = gtsam::Rot3::Ypr(sol.heading, 0., sol.roll_pitch);
            sol.rot_var = (gtsam::Vector3() << sol.roll_pitch_var, 0., sol.heading_var).finished();
          }
          else if(paramPtr_->hasPitch)
          {
            sol.rot = gtsam::Rot3::Ypr(sol.heading, sol.roll_pitch, 0.);
            sol.rot_var = (gtsam::Vector3() << 0, sol.roll_pitch_var, sol.heading_var).finished();
          }
          else
          {
            sol.rot = gtsam::Rot3::Yaw(sol.heading);
            sol.rot_var = (gtsam::Vector3() << 0, 0., sol.heading_var).finished();
          }

          sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos->pos_type.type);
          sol.num_sat = bestpos->num_sol_svs;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7PVTHeadingMsgCb: graph not initialized, waiting ...");
          }
        }

        void onOEM7PVTMsgCb(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos,
                             const novatel_oem7_msgs::msg::BESTVEL::ConstSharedPtr bestvel)
        {
          rclcpp::Time msg_timestamp;
          if(paramPtr_->useHeaderTimestamp)
            msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
          else
            msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.tow = bestpos->nov_header.gps_week_milliseconds  * 0.001;
          sol.has_heading = false;
          sol.has_velocity = true;
          sol.llh = (gtsam::Vector3() << bestpos->lat * fgo::constants::deg2rad,
                                         bestpos->lon * fgo::constants::deg2rad,
                                         bestpos->hgt + bestpos->undulation).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos->lat_stdev, 2), std::pow(bestpos->lon_stdev, 2), std::pow(bestpos->hgt_stdev, 2)).finished();
          const auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
          sol.vel_n = (gtsam::Vector3() << bestvel->hor_speed * std::cos(bestvel->trk_gnd * fgo::constants::deg2rad),
              bestvel->hor_speed * std::sin(bestvel->trk_gnd * fgo::constants::deg2rad),
              -bestvel->ver_speed).finished();
          sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
          sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos->pos_type.type);
          sol.num_sat = bestpos->num_sol_svs;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7PVTMsgCb: graph not initialized, waiting ...");
          }
        }

        void onOEM7Bestpos(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos)
        {
          rclcpp::Time msg_timestamp;
          if(paramPtr_->useHeaderTimestamp)
            msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
          else
            msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.tow = bestpos->nov_header.gps_week_milliseconds  * 0.001;
          sol.has_heading = false;
          sol.has_velocity = false;
          sol.llh = (gtsam::Vector3() << bestpos->lat * fgo::constants::deg2rad,
              bestpos->lon * fgo::constants::deg2rad,
              bestpos->hgt + bestpos->undulation).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos->lat_stdev, 2), std::pow(bestpos->lon_stdev, 2), std::pow(bestpos->hgt_stdev, 2)).finished();

          //auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
          sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos->pos_type.type);
          sol.num_sat = bestpos->num_sol_svs;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7Bestpos: graph not initialized, waiting ...");
          }
        }

        void onUbloxPVTMsgCb(const ublox_msgs::msg::NavPVT::ConstSharedPtr navpvt)
        {

          static size_t calcZeroVelocityCounter = 1;
          static gtsam::Vector3 sumVelocity = gtsam::Z_3x1;
          const auto now = rosNodePtr_->now();
          rclcpp::Time msg_timestamp = rclcpp::Time(now.nanoseconds(), RCL_ROS_TIME);
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "on ubloxPVT at " << std::fixed << msg_timestamp.seconds());
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.tow = navpvt->i_tow * 0.001;
          sol.has_velocity = true;
          sol.has_heading = true;
          sol.llh = (gtsam::Vector3() << navpvt->lat * 1e-7 * fgo::constants::deg2rad,
              navpvt->lon * 1e-7 * fgo::constants::deg2rad,
              navpvt->height * 1e-3).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.xyz_var = (gtsam::Vector3() << std::pow(navpvt->h_acc * 1e-3, 2), std::pow(navpvt->h_acc * 1e-3, 2), std::pow(navpvt->v_acc * 1e-3, 2) * 2).finished();
          sol.vel_n = (gtsam::Vector3() << navpvt->vel_n * 1e-3, navpvt->vel_e * 1e-3, navpvt->vel_d * 1e-3).finished();
          const auto ned_R_e= gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef));
          const auto ecef_R_ned = ned_R_e.inverse();
          sumVelocity += sol.vel_n;
          //sol.xyz_var = ecef_R_ned.rotate(sol.xyz_var);
          gtsam::Matrix H1, H2;
          sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
          const auto velVar = std::pow(navpvt->s_acc * 1e-3, 2);
          sol.vel_var = (gtsam::Vector3() << velVar, velVar, velVar).finished();
          //sol.vel_var = ecef_R_ned.rotate(sol.vel_var);
          const auto heading = navpvt->heading * 1e-5 + paramPtr_->heading_offset_deg;
          if(heading > 360.)
            sol.heading = (heading - 360.) * fgo::constants::deg2rad;
          else
            sol.heading = heading * fgo::constants::deg2rad;
          sol.rot = gtsam::Rot3::Yaw(sol.heading);
          sol.rot_ecef = ecef_R_ned.compose(sol.rot);
          sol.heading_ecef = sol.rot_ecef.yaw();
          sol.heading_var = std::pow(navpvt->head_acc * 1e-5 * fgo::constants::deg2rad, 2);
          sol.rot_var = (gtsam::Vector3() << 0, 0., sol.heading_var).finished();
          sol.type = fgo::utils::GNSS::getUbloxSolutionType(navpvt->fix_type, navpvt->flags);
          sol.num_sat = navpvt->num_sv;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);

          if(calcZeroVelocityCounter > 6)
          {
            const auto avgVelocity = (gtsam::Vector3() << sumVelocity.x() / calcZeroVelocityCounter,
                sumVelocity.y() / calcZeroVelocityCounter,
                sumVelocity.z() / calcZeroVelocityCounter).finished();
            //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), integratorName_ << " avg: velocity " << avgVelocity.norm());
            if(avgVelocity.norm() < paramPtr_->zeroVelocityThreshold)
            {
              RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ << " onIRTPVTMsgCb reported near zero velocity: " << sol.vel_n);
              zeroVelocity_ = true;
              GNSSPVABuffer_.clean();
            }
            else
              zeroVelocity_ = false;
            calcZeroVelocityCounter = 0;

            sumVelocity.setZero();
          }

          calcZeroVelocityCounter ++;

          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onUbloxPVTMsgCb: graph not initialized, waiting ...");
          }
        }

        void onNavFixMsgCb(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
        {
          rclcpp::Time msg_timestamp;
          if(paramPtr_->useHeaderTimestamp)
            msg_timestamp = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
          else
            msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
          fgo::data_types::PVASolution sol{};
          sol.timestamp = msg_timestamp;
          sol.llh = (gtsam::Vector3() << msg->latitude * fgo::constants::deg2rad,
              msg->longitude * fgo::constants::deg2rad,
              msg->altitude).finished();
          sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
          sol.type =  data_types::GNSSSolutionType::SINGLE;
          GNSSPVABuffer_.update_buffer(sol, msg_timestamp);



          if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
            graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
            RCLCPP_WARN(rosNodePtr_->get_logger(), "onNavFixMsgCb: graph not initialized, waiting ...");
          }
        }



    };

}

#endif //ONLINE_FGO_INTEGRATEGNSSLC_H
