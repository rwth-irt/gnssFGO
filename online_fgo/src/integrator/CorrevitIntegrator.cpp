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


#include "integrator/CorrevitIntegrator.h"

namespace fgo::integrator
{

    void CorrevitIntegrator::initialize(rclcpp::Node &node, graph::GraphBase &graphPtr, const std::string& integratorName, bool isPrimarySensor) {
      integratorName_ = integratorName;
      isPrimarySensor_ = isPrimarySensor;
      rosNodePtr_ = &node;
      graphPtr_ = &graphPtr;
      this->initIntegratorBaseParams();

      paramPtr_ = std::make_shared<IntegratorCorrevitParams>(integratorBaseParamPtr_);
      preIntergratorParams_ = std::make_shared<fgo::factor::VelocityPreintegrationParams>();
      /**
       *  Parameters
       */
      ::utils::RosParameter<bool> integrateVelocity("GNSSFGO." + integratorName_ + ".integrateVelocity", node);
      paramPtr_->integrateVelocity = integrateVelocity.value();

      ::utils::RosParameter<int>solutionSyncQueueSize("GNSSFGO." + integratorName_ + ".solutionSyncQueueSize", 10, node);

      ::utils::RosParameter<int>msgLowerBound("GNSSFGO." + integratorName_ + ".MsgSyncLowerBound", 50000000, node);

      ::utils::RosParameter<double> fixedVelVar("GNSSFGO." + integratorName_ + ".fixedVelVar", 1., node);
      paramPtr_->fixedVelVar = fixedVelVar.value();

      ::utils::RosParameter<double> velVarScale("GNSSFGO." + integratorName_ + ".velVarScale", 1., node);
      paramPtr_->velVarScale = velVarScale.value();

      ::utils::RosParameter<double> robustParamVel("GNSSFGO." + integratorName_ + ".roustParamVel", 1., node);
      paramPtr_->robustParamVelocity = robustParamVel.value();
      integratorBaseParamPtr_->robustParamVelocity = robustParamVel.value();

      ::utils::RosParameter<std::string> velocityFrame("GNSSFGO." + integratorName_ + ".velocityFrame", "body", *rosNodePtr_);
      auto velocityFrameStr = velocityFrame.value();
      setSensorFrameFromParam(velocityFrameStr, paramPtr_->velocityFrame, "Correvit");
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityFrame: " << velocityFrameStr);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityFrame: " << paramPtr_->velocityFrame);
      integratorBaseParamPtr_->velocityFrame = paramPtr_->velocityFrame;

      ::utils::RosParameter<std::string> velocityType("GNSSFGO." + integratorName_ + ".velocityType", "3d", *rosNodePtr_);
      auto velocityTypeStr = velocityType.value();
      setVelocityType(velocityTypeStr, paramPtr_->velocityType, "Correvit");
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityType: " << velocityTypeStr);
      integratorBaseParamPtr_->velocityType = paramPtr_->velocityType;

      ::utils::RosParameter<std::string> noiseModelVelocity("GNSSFGO." + integratorName_ + ".noiseModelVelocity", "gaussian", *rosNodePtr_);
      auto noiseModelVelocityStr = noiseModelVelocity.value();
      setNoiseModelFromParam(noiseModelVelocityStr, paramPtr_->noiseModelVelocity, "GNSSLC Velocity");
      integratorBaseParamPtr_->noiseModelVelocity = paramPtr_->noiseModelVelocity;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModelVelocity: " << noiseModelVelocityStr);

      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LeverARMToCorrevit: " << paramPtr_->transIMUToCorrevit);

      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "AutoDiff: " << paramPtr_->AutoDiffNormalFactor);

      ::utils::RosParameter<double> nearZeroVelocityThreshold("GNSSFGO." + integratorName_ + ".nearZeroVelocityThreshold", 0., *rosNodePtr_);
      paramPtr_->nearZeroVelocityThreshold = nearZeroVelocityThreshold.value();

      ::utils::RosParameter<double> StateMeasSyncUpperBound("GNSSFGO." + integratorName_ + ".StateMeasSyncUpperBound", 0.02, node);
      paramPtr_->StateMeasSyncUpperBound = StateMeasSyncUpperBound.value();
      ::utils::RosParameter<double> StateMeasSyncLowerBound("GNSSFGO." + integratorName_ + ".StateMeasSyncLowerBound", -0.02, node);
      paramPtr_->StateMeasSyncLowerBound = StateMeasSyncLowerBound.value();
      ::utils::RosParameter<std::vector<double>> velVarVector("GNSSFGO." + integratorName_ + ".velVarVector", *rosNodePtr_);
      paramPtr_->velVar = gtsam::Vector3(velVarVector.value().data());

      ::utils::RosParameter<double> factorizeDelay("GNSSFGO." + integratorName_ + ".factorizeDelay", 5., node);
      paramPtr_->factorizeDelay = factorizeDelay.value();
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "factorizeDelay: " << paramPtr_->factorizeDelay);
      // IMUSensorSyncTimeThreshold

      ::utils::RosParameter<bool> enablePreIntegration("GNSSFGO." + integratorName_ + ".enablePreIntegration", node);
      paramPtr_->enablePreIntegration = enablePreIntegration.value();

      //Velocity preintegration parameters
      ::utils::RosParameter<double> velocitySigma("GNSSFGO." + integratorName_ + ".velocitySigma", 1.0, node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocitySigma: " << velocitySigma.value());
      preIntergratorParams_->velocityCovariance = pow(velocitySigma.value(), 2) * gtsam::I_3x3;

      ::utils::RosParameter<double> integrationSigma("GNSSFGO." + integratorName_ + ".integrationSigma", 1.0, node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "integrationSigma: " << integrationSigma.value());
      preIntergratorParams_->integrationCovariance = pow(integrationSigma.value(), 2) * gtsam::I_3x3;

      ::utils::RosParameter<double> omegaIncrementSigma("GNSSFGO." + integratorName_ + ".omegaIncrementSigma", 1.0, node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "omegaIncrementSigma: " << omegaIncrementSigma.value());
      preIntergratorParams_->angularIncrementCovariance = pow(omegaIncrementSigma.value(), 2) * gtsam::I_3x3;

      ::utils::RosParameter<double> zeroVelocityThreshold("GNSSFGO." + integratorName_ + ".zeroVelocityThreshold", node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "zeroVelocityThreshold: " << zeroVelocityThreshold.value());
      paramPtr_->zeroVelocityThreshold = zeroVelocityThreshold.value();
      bufferCorrevitVelAngle_.resize_buffer(1000);
      bufferCorrevitVelAngle_.resize_buffer(1000);
      bufferCorrevit_.resize_buffer(1000);

      subCorrevitPitchRoll_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::CorrevitPitchRoll>("/correvit/pitchroll",
                                                                                                 rclcpp::SystemDefaultsQoS(),
                                                                                                 [this](const irt_nav_msgs::msg::CorrevitPitchRoll::ConstSharedPtr msg) -> void
                                                                                                 {
        const auto ts = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        auto data = CorrevitPitchRoll();
        data.timestamp = ts.seconds();
        data.roll = msg->roll;
        data.pitch = msg->pitch;
        data.radius = msg->radius;
        bufferCorrevitPitchRoll_.update_buffer(data, ts);
                                                                                                 });
      subCorrevitVelAngle_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::CorrevitVelAngle>("/correvit/velangle",
                                                                                               rclcpp::SystemDefaultsQoS(),
                                                                                               [this](const irt_nav_msgs::msg::CorrevitVelAngle::ConstSharedPtr msg) -> void
                                                                                               {
        const auto ts = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        auto data = CorrevitVelAngle();
        data.timestamp = ts.seconds();
        data.vel = msg->vel;
        data.angle = msg->angle;
        data.vel_x = msg->vel_x;
        data.vel_y = msg->vel_y;
        bufferCorrevitVelAngle_.update_buffer(data, ts);
                                                                                               });

      subCorrevit_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::Correvit>("/correvit/correvit",
                                                                               rclcpp::SystemDefaultsQoS(),
                                                                               [this](const irt_nav_msgs::msg::Correvit::ConstSharedPtr msg) -> void
                                                                               {
        static double sumVelocity = 0;
        static uint calcZeroVelocityCounter = 1;
        const auto ts = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        auto data = Correvit();
        data.timestamp = ts.seconds();
        data.angle_correvit = msg->angle_correvit;
        data.vel_correvit = msg->vel_correvit;
        data.vel_x_correvit = msg->vel_x_correvit;
        data.vel_y_correvit = msg->vel_y_correvit;
        bufferCorrevit_.update_buffer(data, ts);

        sumVelocity += msg->vel_correvit;
        if(calcZeroVelocityCounter > 6)
        {
          const auto avgVelocity = abs(sumVelocity) / calcZeroVelocityCounter;

          if(avgVelocity < paramPtr_->zeroVelocityThreshold)
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ << " reported near zero velocity: " << data.angle_correvit);
            bufferCorrevit_.clean();
            zeroVelocity_ = true;
          }
          else
            zeroVelocity_ = false;
          calcZeroVelocityCounter = 0;
          sumVelocity = 0.;
       }
       calcZeroVelocityCounter ++;

                                                                               });
    }

    bool
    CorrevitIntegrator::factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                  const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                                  const solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                  std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                                  gtsam::Values &values,
                                  solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                                  gtsam::KeyVector& relatedKeys) {

      static auto firstCallTime = rosNodePtr_->now();
      static auto fixedVelVar = paramPtr_->velVar * paramPtr_->velVarScale;

      if(!paramPtr_->integrateVelocity)
      {
        return true;
      }

      if((rosNodePtr_->now() - firstCallTime).seconds() < paramPtr_->factorizeDelay)
      {
        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << "Correvit: delaying ");
        bufferCorrevit_.clean();
        return true;
      }

      static gtsam::Key pose_key_j, vel_key_j, bias_key_j, omega_key_j,
             pose_key_i, vel_key_i,  bias_key_i, omega_key_i,
             pose_key_sync, vel_key_sync,  bias_key_sync;
      static uint64_t last_key_index = 0;
      static boost::circular_buffer<CorrevitVelAngle> restCorrevitVelAngle(1000);
      static boost::circular_buffer<Correvit> restCorrevit(1000);
      static boost::circular_buffer<CorrevitPitchRoll> restCorrevitPitchRoll(1000);

      auto dataCorrevit = bufferCorrevit_.get_all_buffer_and_clean();

      if(!restCorrevit.empty())
      {
        dataCorrevit.insert(dataCorrevit.begin(), restCorrevit.begin(), restCorrevit.end());
        restCorrevit.clear();
      }

      /*
      if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ)
      {
        interpolator_ = std::make_shared<fgo::model::GPWNOJInterpolatorPose3>(
            gtsam::noiseModel::Gaussian::Covariance(integratorParamPtr_->QcGPInterpolator * gtsam::I_6x6), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOA)
      {
        interpolator_ = std::make_shared<fgo::model::GPWNOAInterpolatorPose3>(
            gtsam::noiseModel::Gaussian::Covariance(integratorParamPtr_->QcGPInterpolator * gtsam::I_6x6), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else
      {
        RCLCPP_WARN(appPtr_->get_logger(), "NO gpType chosen. Please choose.");
        return false;
      }*/

      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << "Correvit: integrating with " << dataCorrevit.size() << " correvit data");

      //std::vector<StateMeasSyncResult> stateMeasSyncResults_;

      auto dataCorrevitIter = dataCorrevit.begin();
      while(dataCorrevitIter != dataCorrevit.end())
      {
        const auto corrected_time = dataCorrevitIter->timestamp  - 0.0008;
        const auto vel = (gtsam::Vector3() << dataCorrevitIter->vel_x_correvit, -dataCorrevitIter->vel_y_correvit, 0).finished();

        //stateMeasSyncResults_.emplace_back(syncResult);

        if(dataCorrevitIter->vel_x_correvit < paramPtr_->nearZeroVelocityThreshold)
        {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "[Correvit] near zero velocity with vx: " << dataCorrevitIter->vel_x_correvit  <<" not integrating ... ");
          dataCorrevitIter ++;
          continue;
        }

        auto syncResult = findStateForMeasurement(currentKeyIndexTimestampMap, corrected_time, paramPtr_);
        if (syncResult.status == StateMeasSyncStatus::DROPPED ||syncResult.status == StateMeasSyncStatus::INTERPOLATED )
        {
          dataCorrevitIter ++;
          continue;
        }

        //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Found State I: " << std::fixed <<
        //                                                                syncResult.keyIndexI << " : " << gtsam::symbolIndex(syncResult.keyIndexI) << " at: " << syncResult.timestampI << " and J: "
        //                                                                << syncResult.keyIndexJ << " : " << gtsam::symbolIndex(syncResult.keyIndexJ) << " at: " << syncResult.timestampJ
        //                                                                << " DurationToI: " << syncResult.durationFromStateI);
        const auto currentPredState = timePredStates.back().second; //graph::querryCurrentPredictedState(timePredStates, timeSync);

        if(paramPtr_->verbose) {
          std::cout << "Correvit Measured X: " << dataCorrevitIter->vel_x_correvit << std::endl;
          std::cout << "Correvit Measured Y: " << dataCorrevitIter->vel_y_correvit << std::endl;
          const auto test = currentPredState.state.attitude().unrotate(currentPredState.state.velocity());
          std::cout << "current vel_b: " << test << std::endl;
        }

        pose_key_i = X(syncResult.keyIndexI);
        vel_key_i = V(syncResult.keyIndexI);
        bias_key_i = B(syncResult.keyIndexI);
        omega_key_i = W(syncResult.keyIndexI);

        pose_key_j = X(syncResult.keyIndexJ);
        vel_key_j = V(syncResult.keyIndexJ);
        bias_key_j = B(syncResult.keyIndexJ);
        omega_key_j = W(syncResult.keyIndexJ);

        if (!syncResult.stateJExist()){
          RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "GP Correvit: NO state J found !!! ");
        }
        double timeSync;
        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I || syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J)
        {

          // now we found a state which is synchronized with the GNSS obs
          if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
            pose_key_sync = pose_key_i;
            vel_key_sync = vel_key_i;
            bias_key_sync = bias_key_i;
            timeSync = syncResult.timestampI;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
           //                    "[Correvit] Found time synchronized state at I " << gtsam::symbolIndex(pose_key_sync)
           //                                                                    <<
           //                                                                    " with time difference: "
            //                                                                   << syncResult.durationFromStateI);
          }
          else {
            pose_key_sync = pose_key_j;
            vel_key_sync = vel_key_j;
            bias_key_sync = bias_key_j;
            timeSync = syncResult.timestampJ;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
            //                   "[Correvit] Found time synchronized state at J " << gtsam::symbolIndex(pose_key_sync)
             //                                                                  <<
             //                                                                  " with time difference: "
             //                                                                  << syncResult.durationFromStateI);
          }
          const auto [fountGyro, this_gyro] = findOmegaToMeasurement(corrected_time, timestampGyroMap);
          const auto this_gyro_unbiased = currentPredState.imuBias.correctGyroscope(this_gyro);
          //this_gyro = currentPredState.imuBias.correctGyroscope(this_gyro);
          //std::cout << "Correvit currentKey: " << gtsam::symbolIndex(pose_key_sync) << std::endl;
          //std::cout << "Correvit lastKey: " << last_key_index << std::endl;
          //std::cout << "Correvit found omega: " << this_gyro << std::endl;
          //std::cout << "Correvit found omega: " << this_gyro_unbiased << std::endl;
          if(last_key_index == gtsam::symbolIndex(pose_key_sync))
          {
            dataCorrevitIter ++;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
            //                   "[Correvit] Not integrating of  " << last_key_index);
            continue;
          }
          //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
         //                    "[Correvit] Integrating of  " << last_key_index);
          this->addNavVelocityFactor(pose_key_sync, vel_key_sync, vel, fixedVelVar, this_gyro_unbiased,
                                     paramPtr_->transIMUToCorrevit, paramPtr_->velocityType);

        }
/*
        else if (syncResult.status == StateMeasSyncStatus::INTERPOLATED && integratorParamPtr_->useGPInterpolatedFactor)
        {
          RCLCPP_WARN_STREAM(appPtr_->get_logger(),
                             "[Correvit] Found not synchronized between " << syncResult.keyIndexI << " and " << syncResult.keyIndexJ <<
                                                                          " with time difference: "
                                                                         << syncResult.durationFromStateI);

          const double delta_t = syncResult.timestampJ - syncResult.timestampI;
          const double taui = syncResult.durationFromStateI;
          //recalculate interpolator // set up interpolator
          //corrected_time_gnss_meas - timestampI;
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), integratorName_ + ": GP delta: " << delta_t << " tau: " << taui);
          //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE

          if(integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ) {
            const auto [foundAccI, accI, foundAccJ, accJ] = findAccelerationToState(syncResult.keyIndexI, stateIDAccMap);

            if(!foundAccI || !foundAccJ) {
              dataCorrevitIter ++;
              continue;
            }

            interpolator_->recalculate(delta_t, taui, accI, accJ);
          }
          else
            interpolator_->recalculate(delta_t, taui);
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Integrating GP interpolated Correvit ...");
          this->addGPInterpolatedNavVelocityFactor(pose_key_i, vel_key_i, omega_key_i,
                                                   pose_key_j, vel_key_j, omega_key_j,
                                                   vel, fixedVelVar,
                                                   integratorParamPtr_->transIMUToCorrevit, interpolator_, integratorParamPtr_->velocityType);
        }*/
        else if(syncResult.status == StateMeasSyncStatus::CACHED)
        {
          restCorrevit.push_back(*dataCorrevitIter);
        }
        last_key_index = gtsam::symbolIndex(pose_key_sync);
        dataCorrevitIter ++;
      }

      return true;
    }

    bool CorrevitIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &martinals,
                                         const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                         data_types::State &optState) {
      return true;
    }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::integrator::CorrevitIntegrator, fgo::integrator::IntegratorBase)