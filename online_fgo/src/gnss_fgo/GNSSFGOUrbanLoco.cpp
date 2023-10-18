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

#include "gnss_fgo/GNSSFGOUrbanLoco.h"

namespace gnss_fgo
{
    GNSSFGOTimeCentricUrbanLocoNode::GNSSFGOTimeCentricUrbanLocoNode(const rclcpp::NodeOptions &opt)
    : GNSSFGOLocalizationBase("GNSSFGOTimeCentricUrbanLocoNode", opt){
      RCLCPP_INFO(this->get_logger(), "---------------------  GNSSFGOTimeCentricUrbanLocoNode initializing! --------------------- ");
      std::lock_guard lk(allBufferMutex_);

      if (!initializeCommon()) {
        RCLCPP_ERROR(this->get_logger(), "initializeParameters went wrong. Please check the parameters in config.");
      }

      subPVA_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVAX>("/novatel_data/inspvax",
                                                                           rclcpp::SensorDataQoS(),
                                                                           std::bind(&GNSSFGOTimeCentricUrbanLocoNode::onINSPVAXMsgCb, this,
                                                                                     std::placeholders::_1));

      SPANNavFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("SPANNavFix",
                                                                             rclcpp::SystemDefaultsQoS());
      RCLCPP_INFO(this->get_logger(), "---------------------  GNSSFGOTimeCentricUrbanLocoNode initialized! --------------------- ");
    }


    void GNSSFGOTimeCentricUrbanLocoNode::onINSPVAXMsgCb(novatel_oem7_msgs::msg::INSPVAX::ConstSharedPtr pva)
    {
      fgo::data_types::PVASolution this_pva{};
      rclcpp::Time ts;

      if(paramsPtr_->useHeaderTimestamp)
        ts = rclcpp::Time(pva->header.stamp.sec, pva->header.stamp.nanosec, RCL_ROS_TIME);
      else
        ts = rclcpp::Time(this->now(), RCL_ROS_TIME);
      //RCLCPP_INFO_STREAM(this->get_logger(), "onINSPVAXMsgCb at " << std::fixed << ts.seconds());
      this_pva.timestamp = ts;
      this_pva.tow = pva->nov_header.gps_week_milliseconds * .001;
      this_pva.type = fgo::utils::GNSS::getOEM7PVTSolutionType(pva->pos_type.type);

      sensor_msgs::msg::NavSatFix span_nav_fix;
      span_nav_fix.header.stamp = ts;
      span_nav_fix.header.frame_id = "span";
      span_nav_fix.latitude = pva->latitude;
      span_nav_fix.longitude = pva->longitude;
      span_nav_fix.altitude = pva->height;
      SPANNavFixPub_->publish(span_nav_fix);

      this_pva.llh = (gtsam::Vector3() << pva->latitude * fgo::constants::deg2rad, pva->longitude * fgo::constants::deg2rad, pva->height + pva->undulation).finished();
      this_pva.xyz_ecef = fgo::utils::llh2xyz(this_pva.llh);
      this_pva.xyz_var = (gtsam::Vector3() << std::pow(pva->latitude_stdev, 2), std::pow(pva->longitude_stdev, 2), std::pow(pva->height_stdev, 2)).finished();
      this_pva.vel_n = (gtsam::Vector3() << pva->north_velocity, pva->east_velocity, -pva->up_velocity).finished();
      this_pva.vel_var = (gtsam::Vector3() << std::pow(pva->north_velocity_stdev, 2), std::pow(pva->east_velocity_stdev, 2), std::pow(pva->up_velocity_stdev, 2)).finished();
      auto eRned = gtsam::Rot3(fgo::utils::nedRe_Matrix(this_pva.xyz_ecef)).inverse();
      this_pva.vel_ecef = eRned.rotate(this_pva.vel_n);

      this_pva.has_rotation_3D = true;
      this_pva.rot = gtsam::Rot3::Ypr(pva->azimuth * fgo::constants::deg2rad,
                                      pva->pitch * fgo::constants::deg2rad,
                                      pva->roll * fgo::constants::deg2rad);

      this_pva.rot_ecef = eRned.compose(this_pva.rot);

      this_pva.rot_var = (gtsam::Vector3() << pva->azimuth_stdev * fgo::constants::deg2rad,
                                              pva->roll_stdev * fgo::constants::deg2rad,
                                              pva->pitch_stdev * fgo::constants::deg2rad).finished();
      this_pva.heading = -pva->azimuth * fgo::constants::deg2rad;
      this_pva.has_heading = true;
      this_pva.has_velocity = true;
      this_pva.has_roll_pitch = true;
      pvaSolutionBuffer_.update_buffer(this_pva, ts);
    }

    void GNSSFGOTimeCentricUrbanLocoNode::calculateErrorOnState(const fgo::data_types::State &stateIn) {
      static const auto lb_ref = paramsPtr_->transIMUToReference;
      static std::vector<fgo::data_types::State> stateCached;

      const auto pvtBuffer = pvaSolutionBuffer_.get_all_buffer();
      const auto lastPVTTime = pvtBuffer.back().timestamp.seconds();
      stateCached.emplace_back(stateIn);

      auto stateIter = stateCached.begin();
      while(stateIter != stateCached.end())
      {
        double stateTime = stateIter->timestamp.seconds();
        if(stateTime < lastPVTTime)
        {
          // buffer is sorted
          auto itAfter = std::lower_bound(pvtBuffer.begin(), pvtBuffer.end(), stateTime,
                                          [](const fgo::data_types::PVASolution &pva, double timestamp) -> bool {
                                              // true, ... true, true, false(HERE), false, ... false
                                              return pva.timestamp.seconds() < timestamp;
                                          });
          auto itBefore = itAfter - 1;

          //RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: PVT before: " << std::fixed << itBefore->timestamp.seconds());
          //RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: state : " << std::fixed << stateTime);
          //RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: PVT before: " << std::fixed << itAfter->timestamp.seconds());

          const auto coeff = (stateTime - itBefore->timestamp.seconds()) / (itAfter->timestamp.seconds() - itBefore->timestamp.seconds());
          const double tiffDiff = stateTime - itBefore->timestamp.seconds();

          //RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: PVT scale: " << std::fixed << coeff);

          //const auto pva_llh = gtsam::interpolate(itBefore->llh, itAfter->llh, coeff);
          const auto pva_llh_std = gtsam::interpolate(itBefore->xyz_var, itAfter->xyz_var, coeff).cwiseSqrt();
          const auto pva_pos_ecef = gtsam::interpolate(itBefore->xyz_ecef, itAfter->xyz_ecef, coeff);
          const auto pva_rot_ecef = gtsam::interpolate(itBefore->rot_ecef, itAfter->rot_ecef, coeff);
          const auto pva_rot_ned = gtsam::interpolate(itBefore->rot, itAfter->rot, coeff);
          const auto pva_rot_ned_std = gtsam::interpolate(itBefore->rot_var, itAfter->rot_var, coeff).cwiseSqrt();
          const auto pva_vel_ned = gtsam::interpolate(itBefore->vel_n, itAfter->vel_n, coeff);
          const auto pva_vel_ned_std = gtsam::interpolate(itBefore->vel_var, itAfter->vel_var, coeff);
          const auto pva_vel_ecef = gtsam::interpolate(itBefore->vel_ecef, itAfter->vel_ecef, coeff);

          const auto pva_llh = fgo::utils::WGS84InterpolationLLH(itBefore->llh,
                                                                     itAfter->llh,
                                                                     coeff);

          sensor_msgs::msg::NavSatFix pvtMsg;
          pvtMsg.header.stamp = stateIter->timestamp;
          pvtMsg.altitude = pva_llh.z() ;
          pvtMsg.latitude = pva_llh.x() * fgo::constants::rad2deg;
          pvtMsg.longitude = pva_llh.y() * fgo::constants::rad2deg;
          pvtInterpolatedPub_->publish(pvtMsg);

          irt_nav_msgs::msg::Error2GT error2Gt;

          const auto fgo_pos_ecef = stateIter->state.position();
          const auto fgo_ori_ecef = stateIter->state.attitude();
          const auto fgo_omega = stateIter->omega;
          const auto fgo_bias = stateIter->imuBias;
          const auto fgo_cbd = stateIter->cbd;
          const auto fgo_vel_ecef = stateIter->state.v();
          const auto fgo_pose_ecef_std = stateIter->poseVar.diagonal().cwiseSqrt();
          const auto fgo_vel_ecef_std = stateIter->velVar.diagonal().cwiseSqrt();
          const auto fgo_omega_std = stateIter->omegaVar.diagonal().cwiseSqrt();
          const auto fgo_bias_std = stateIter->imuBiasVar.diagonal().cwiseSqrt();

          const auto pos_ant_main = fgo_pos_ecef + fgo_ori_ecef.rotate(lb_ref);
          const auto pos_ant_llh = fgo::utils::xyz2llh(pos_ant_main);
          const gtsam::Rot3 nRe(fgo::utils::nedRe_Matrix(pos_ant_main));

          const auto fgo_pose_std_ned = (gtsam::Vector6() << nRe.rotate(fgo_pose_ecef_std.block<3, 1>(0, 0)), nRe.rotate(fgo_pose_ecef_std.block<3, 1>(3, 0))).finished();
          const auto fgo_vel_ned_std = nRe.rotate(fgo_vel_ecef_std);

          //gtsam::Rot3 nRePVT(fgo::utils::nedRe_Matrix_asLLH(posllh));
          const gtsam::Vector3 fgo_vel_ned = nRe.rotate(fgo_vel_ecef + fgo_ori_ecef.rotate(gtsam::skewSymmetric((-lb_ref)) * fgo_omega));
          const auto fgo_rot_ned =  nRe.compose(fgo_ori_ecef);
          auto nRb_rpy = fgo_rot_ned.rpy(); //fgo::utils::func_DCM2EulerAngles(fgo_rot_ned.matrix());

          auto fgo_yaw = nRb_rpy(2) * 180. / M_PI;
          //RCLCPP_INFO_STREAM(this->get_logger(), "########## EVALUATE ANGLE ########## ");
          //RCLCPP_INFO_STREAM(this->get_logger(), "FGO yaw rot: " << std::fixed << fgo_yaw);
          fgo_yaw = (fgo_yaw >= 0. ? fgo_yaw : (fgo_yaw + 360.));
          //RCLCPP_INFO_STREAM(this->get_logger(), "FGO yaw corrected: " << std::fixed << fgo_yaw);
          //RCLCPP_INFO_STREAM(this->get_logger(), "PVT yaw: " << std::fixed << pvtMeas.Yaw);
          //RCLCPP_INFO_STREAM(this->get_logger(), "PVT yaw not corrected: " << std::fixed << pvtMeas.COG);
          auto fgo_pitch = nRb_rpy(1) * 180. / M_PI;
          auto fgo_roll = nRb_rpy(0) * 180. / M_PI;

          fgo::utils::eigenMatrix2stdVector(pva_llh_std, error2Gt.ref_llh_std);
          fgo::utils::eigenMatrix2stdVector(pva_vel_ned_std, error2Gt.ref_vel_std);
          fgo::utils::eigenMatrix2stdVector(fgo_vel_ecef_std, error2Gt.vel_std_ecef);
          fgo::utils::eigenMatrix2stdVector(fgo_vel_ned_std, error2Gt.vel_std_ned);
          fgo::utils::eigenMatrix2stdVector(fgo_vel_ned, error2Gt.vel_ned);
          fgo::utils::eigenMatrix2stdVector(pva_vel_ned, error2Gt.ref_vel);
          fgo::utils::eigenMatrix2stdVector(fgo_bias.accelerometer(), error2Gt.acc_bias);
          fgo::utils::eigenMatrix2stdVector(fgo_bias.gyroscope(), error2Gt.gyro_bias);
          fgo::utils::eigenMatrix2stdVector(fgo_bias_std.head(3), error2Gt.acc_bias_std);
          fgo::utils::eigenMatrix2stdVector(fgo_bias_std.tail(3), error2Gt.gyro_bias_std);
          fgo::utils::eigenMatrix2stdVector(fgo_omega, error2Gt.omega_body);
          fgo::utils::eigenMatrix2stdVector(fgo_omega_std, error2Gt.omega_body_std);

          const auto pos_diff_ecef = pos_ant_main - pva_pos_ecef;
          const auto pos_error_ned = nRe.rotate(pos_diff_ecef);//
          const auto pos_error_body = fgo_ori_ecef.unrotate(pos_diff_ecef);
          const auto vel_error_ned = fgo_vel_ned - pva_vel_ned;

          const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
          double dist;
          geod.Inverse(pva_llh.x() * fgo::constants::rad2deg, pva_llh.y() * fgo::constants::rad2deg,
                       pos_ant_llh.x() * fgo::constants::rad2deg, pos_ant_llh.y() * fgo::constants::rad2deg, dist);

          error2Gt.pos_2d_error_geographic = dist;
          error2Gt.pos_3d_error_geographic = std::sqrt(dist * dist + std::pow((pva_llh.z() - pos_ant_llh.z()), 2));
          error2Gt.pos_1d_error_ned = abs(pos_error_ned(1));//fgo_ori.unrotate(pos_error_ecef)(1)
          error2Gt.pos_2d_error_ned = (pos_error_ned).block<2, 1>(0,0).norm();
          error2Gt.pos_3d_error_ned = pos_error_ned.norm();
          error2Gt.pos_1d_error_body = abs(pos_error_body(1));//fgo_ori.unrotate(pos_error_ecef)(1)
          error2Gt.pos_2d_error_body = (pos_error_body).block<2, 1>(0,0).norm();
          error2Gt.pos_3d_error_body = pos_error_body.norm();
          error2Gt.pos_2d_error_ecef = (pos_diff_ecef).block<2, 1>(0,0).norm();
          error2Gt.pos_3d_error_ecef = pos_diff_ecef.norm();
          error2Gt.vel_2d_error = vel_error_ned.block<2, 1>(0,0).norm();
          error2Gt.vel_3d_error = vel_error_ned.norm();
          double yaw_error = fgo_yaw - pva_rot_ned.yaw() * fgo::constants::rad2deg;
          while (yaw_error > 180.0 || yaw_error < -180.0){
            if (yaw_error > 180.0)
              yaw_error -= 360.0;
            if (yaw_error < -180.0)
              yaw_error += 360.0;
          }
          yaw_error = abs(yaw_error);
          error2Gt.yaw_error = yaw_error;

          error2Gt.ref_mode = itAfter->type;
          error2Gt.yaw = fgo_yaw;
          error2Gt.ref_yaw =  pva_rot_ned.yaw() * fgo::constants::rad2deg;
          error2Gt.ref_yaw_std = pva_rot_ned_std.y() * fgo::constants::rad2deg;
          error2Gt.pitch = fgo_pitch;
          error2Gt.roll = fgo_roll;
          error2Gt.ref_pitch_roll = itAfter->roll_pitch;
          error2Gt.header.stamp = stateIter->timestamp;
          //RCLCPP_INFO(this->get_logger(), "calculateError successful");
          pvtErrorPub_->publish(error2Gt);
          /*
          auto related_labeling_iter = labelBuffer.rbegin();
          while(related_labeling_iter != labelBuffer.rend())
          {
            if(related_labeling_iter->related_state_timestamp_nanosec == stateTimeNanoSec)
            {
              auto& label = *related_labeling_iter;
              label.ant_pos = fgo::utils::convertGTVec3ToROS(pos_ant_main);
              label.ant_vel = fgo::utils::convertGTVec3ToROS(fgo_vel_ned);
              label.gt_pos = fgo::utils::convertGTVec3ToROS(gt_posecef);
              label.gt_vel = fgo::utils::convertGTVec3ToROS(gt_velned);
              label.clock_bias = fgo_cbd(0);
              label.clock_drift = fgo_cbd(1);
              label.gt_clock_bias = pvt_RxClkBias;
              label.gt_clock_drift = pvt_RxClkDrift;
              label.solution_type = itAfter->Mode;
              if (itAfter->Mode == fgo::constants::PVTType::NARROW_INT)
                label.gt_available = true;
              gnssLabelingPub_->publish(label);
              break;
            }
            related_labeling_iter ++;
          }
          */
          stateIter = stateCached.erase(stateIter);
          continue;
        }
        stateIter ++;
      }
    }


}