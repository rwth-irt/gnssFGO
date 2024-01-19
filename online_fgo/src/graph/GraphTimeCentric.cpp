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


#include "graph/GraphTimeCentric.h"
#include "gnss_fgo/GNSSFGOLocalizationBase.h"


namespace fgo::graph
{
    GraphTimeCentric::GraphTimeCentric(gnss_fgo::GNSSFGOLocalizationBase& node) : GraphBase(node)
    {
      RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphTimeCentric initializing! --------------------- ");
      paramPtr_ = std::make_shared<GraphTimeCentricParam>(graphBaseParamPtr_);

      pubIMUFactorReport_ = appPtr_->create_publisher<irt_nav_msgs::msg::SensorProcessingReport>("sensor_processing_report/imu",
                                                                                                 rclcpp::SystemDefaultsQoS());

      RCLCPP_INFO(appPtr_->get_logger(), "--------------------- GraphTimeCentric initialized! ---------------------");
    }

    StatusGraphConstruction GraphTimeCentric::constructFactorGraphOnIMU(std::vector<fgo::data_types::IMUMeasurement> &dataIMU)
    {
      // example: imu on 100Hz and we optimize on 10 Hz, then we should count 10 imu measurements
      static uint notifyCounter = graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->stateFrequency;
      static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(20 * graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->stateFrequency);
      static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> stateIDAccMap(50 * graphBaseParamPtr_->smootherLag * graphBaseParamPtr_->stateFrequency);
      static gtsam::Vector6 lastAcc = gtsam::Vector6::Zero();
      static gtsam::Vector3 meanAccA, meanAccG = gtsam::Vector3();
      static uint64_t lastNState = 0;
      static bool skippedOpt = false;

      gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, omega_key_j,
          pose_key_i, vel_key_i, bias_key_i, cbd_key_i, omega_key_i;

      if (dataIMU.empty() || !isStateInited_) {
        RCLCPP_WARN(appPtr_->get_logger(), "constructFactorGraphOnIMU: No IMU measurement, waiting ..."); // this shouldn't happen
        return StatusGraphConstruction::FAILED;
      }

      bool hasMeasurements = true;
      double voteZeroVelocity = 0;
      for(const auto& integrator : integratorMap_)
      {
        hasMeasurements &= integrator.second->checkHasMeasurements();
        if(integrator.second->checkZeroVelocity())
        {
          RCLCPP_WARN_STREAM(appPtr_->get_logger(), integrator.first << " reported ZERO VELOCITY"); // this shouldn't happen

          voteZeroVelocity++;
        }
      }

      static double noOptimizationDuration = 0.;
      static rclcpp::Time firstNoOptimizationDecision;
      if(paramPtr_->NoOptimizationNearZeroVelocity && (nState_ - lastNState) > paramPtr_->NoOptimizationAfterStates)
      {
        if(voteZeroVelocity / integratorMap_.size() > paramPtr_->VoteNearZeroVelocity)
        {
          RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "Near zero velocity with " << voteZeroVelocity << " integrator of " << integratorMap_.size() << " not optimizing...");

          if(!skippedOpt)
            firstNoOptimizationDecision = appPtr_->now();
          noOptimizationDuration = (appPtr_->now() - firstNoOptimizationDecision).seconds();
          for(const auto& integrator : integratorMap_)
          {
            integrator.second->cleanBuffers();
          }
          RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "Near zero velocity duration " << noOptimizationDuration);

          skippedOpt = true;
          solver_->setNotMarginalizing();
          return StatusGraphConstruction::NO_OPTIMIZATION;
        }
      }

      auto currentPredState = currentPredictedBuffer_.get_last_buffer(); //graph::querryCurrentPredictedState(timePredStates, currentStateTimestamp);

      if(skippedOpt) {
        skippedOpt = false;
        lastNState = nState_;
        solver_->setSmootherLagInflation(noOptimizationDuration);

        //solver_->setMarginalizing();
        for(const auto& integrator : integratorMap_)
        {
          integrator.second->notifyOptimization(noOptimizationDuration);
        }

        // To overcome problems of timing drifting due to no optimization update, we add prior factor after optimization
        // has been stopped

        /*
        nState_++;

        this->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(nState_), currentPredState.state.pose(), currentPredState.poseVar);
        this->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(nState_), currentPredState.state.v(), currentPredState.velVar);
        this->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(nState_), currentPredState.imuBias, currentPredState.imuBiasVar);

        values_.insert(X(nState_), currentPredState.state.pose());
        values_.insert(V(nState_), currentPredState.state.v());
        values_.insert(B(nState_), currentPredState.imuBias);

        const auto first_imu = dataIMU.front();
        const auto current_state_timestamp = first_imu.timestamp.seconds();
        keyTimestampMap_[X(nState_)] =
        keyTimestampMap_[V(nState_)] =
        keyTimestampMap_[B(nState_)] = current_state_timestamp;


        if(graphBaseParamPtr_->useConstDriftFactor) {
          this->emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(C(nState_), currentPredState.cbd, currentPredState.cbdVar);
          values_.insert(C(nState_), currentPredState.cbd);
          keyTimestampMap_[C(nState_)] = current_state_timestamp;
        }

        currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, current_state_timestamp));

        if(graphBaseParamPtr_->useGPPriorFactor || graphBaseParamPtr_->useGPInterpolatedFactor)
        {
          this->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(nState_), currentPredState.omega, currentPredState.omegaVar);
          values_.insert(W(nState_), currentPredState.omega);
          keyTimestampMap_[W(nState_)] = current_state_timestamp;
        }

        if(!graphBaseParamPtr_->useGPPriorFactor  && graphBaseParamPtr_->useGPInterpolatedFactor)
        {
          fgo::data_types::IMUMeasurement tmpIMU;
          tmpIMU.gyro = currentPredState.omega;
          this->addAngularFactor(W(nState_), B(nState_), tmpIMU);
        }

        //const auto stateAccPair = std::make_pair(nState_, currentAcc);
        stateIDAccMap.push_back(std::make_pair(nState_, currentPredState.accMeasured));
        accBuffer_.update_buffer(currentPredState.accMeasured, first_imu.timestamp);
         */
        noOptimizationDuration = 0.;
      }

      if(paramPtr_->NoOptimizationWhileNoMeasurement && !hasMeasurements && nState_ > paramPtr_->NoOptimizationAfterStates)
      {
        RCLCPP_ERROR(appPtr_->get_logger(), "NO Reference Measurement, not optimizing...");
        return StatusGraphConstruction::NO_OPTIMIZATION;
      }

      if(!dataIMURest_.empty())
      {
        dataIMU.insert(dataIMU.begin(), dataIMURest_.begin(), dataIMURest_.end());
        dataIMURest_.clear();
      }

      //const auto current_pred_state_timestamp = currentPredState.timestamp.seconds();
      //const auto first_imu_meas_timestamp = dataIMU.front().timestamp.seconds();  // in double
      //const auto last_imu_meas_timestamp = dataIMU.back().timestamp.seconds();  // in double

      if(paramPtr_->calibGravity)
        preIntegratorParams_->n_gravity = /*fgo::utils::nedRe_Matrix(lastOptimizedState_.state.position()) * */
            fgo::utils::gravity_ecef(currentPredState.state.position());

      std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
          std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                     currentPredState.imuBias);

      double sum_imu_dt = 0.;
      uint counter_imu = 0;
      //uint64_t num_state_started = nState_;

      size_t i = 0;
      const auto imuSize = dataIMU.size();

      const auto restOfIMU = imuSize % notifyCounter;
      dataIMURest_.resize(restOfIMU);
      std::copy(dataIMU.end() - restOfIMU, dataIMU.end(), dataIMURest_.begin());

      for (; i < imuSize; i++)
      {
        auto current_imu_meas = dataIMU[i];
        sum_imu_dt += current_imu_meas.dt;
        counter_imu++;
        timeGyroMap.push_back(std::make_pair(current_imu_meas.timestamp.seconds(), current_imu_meas.gyro));
        //timeGyroMap[current_imu_meas.timestamp.seconds()] = current_imu_meas.gyro;
        imuPreIntegrationOPT_->integrateMeasurement(current_imu_meas.accLin,
                                                    current_imu_meas.gyro,
                                                    current_imu_meas.dt);
        //meanAccA += current_imu_meas.accLin;
        meanAccG += current_imu_meas.accRot;
        if ( ( (i + 1) % notifyCounter) == 0 )
        {
          // TODO: @haoming if this condition could be run several times, the currentPredState maybe outdated!
          // TODO: @haoming better way to do so is query the currentPredState instead of feeding the whole last buffer!
          // if we have countered enough IMU measurements, then we create state variable and all relative factor
          // the integrated IMU measurements have reached the duration to add a state vector.
          // here we increase the state index and construct IMU factor
          nState_++;
          pose_key_j  = X(nState_);
          vel_key_j   = V(nState_);
          bias_key_j  = B(nState_);
          omega_key_j = W(nState_);

          pose_key_i  = X(nState_ - 1);
          vel_key_i   = V(nState_ - 1);
          bias_key_i  = B(nState_ - 1);
          omega_key_i = W(nState_ - 1);

          meanAccG /= notifyCounter;
          //meanAccA /= notifyCounter;

          currentPredState = currentPredictedBuffer_.get_last_buffer(); //graph::querryCurrentPredictedState(timePredStates, currentStateTimestamp);
          const auto currentStateTimestamp = current_imu_meas.timestamp.seconds();

          gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
          if(paramPtr_->calibGravity)
          {
            const auto gravity = fgo::utils::gravity_ecef(currentPredState.state.position());
            gravity_b = currentPredState.state.attitude().unrotate(gravity);
          }

          auto currentAcc = (gtsam::Vector6() << current_imu_meas.accRot,
                                                 currentPredState.imuBias.correctAccelerometer(current_imu_meas.accLin + gravity_b)).finished();
          //const auto stateAccPair = std::make_pair(nState_, currentAcc);
          stateIDAccMap.push_back(std::make_pair(nState_, currentAcc));
          accBuffer_.update_buffer(currentAcc, current_imu_meas.timestamp);
          meanAccG.setZero();
          //meanAccA.setZero();

          RCLCPP_INFO_STREAM(appPtr_->get_logger(), "FGConIMU: creating state variable " << nState_
                                                                                         << " at: " << std::fixed << current_imu_meas.timestamp.seconds()
                                                                                         << " queried predicted state at " << currentPredState.timestamp.seconds());

          currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, currentStateTimestamp));
          // auto predictedState = imuPreIntegrationOPTTmp_->predict(current_pred_state.state, current_pred_state.imuBias);

          //setup values
          values_.insert(pose_key_j, currentPredState.state.pose());
          values_.insert(vel_key_j, currentPredState.state.velocity());
          values_.insert(bias_key_j, currentPredState.imuBias);

          keyTimestampMap_[pose_key_j]  =
          keyTimestampMap_[vel_key_j]   =
          keyTimestampMap_[bias_key_j]  = currentStateTimestamp;

          //add acc to map
          // IMU Factor
          boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor = //make new IMU Factor
              boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                           pose_key_j, vel_key_j,
                                                           bias_key_i, bias_key_j,
                                                           *imuPreIntegrationOPT_);
          imu_factor->setTypeID(fgo::factor::FactorTypeIDs::CombinedIMU);
          imu_factor->setName("CombinedImuFactor");
          this->push_back(imu_factor);

          irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
          thisProcessingReport.header.stamp = appPtr_->now();
          thisProcessingReport.ts_measurement = appPtr_->now().seconds();
          thisProcessingReport.duration_processing = sum_imu_dt;
          thisProcessingReport.measurement_delay = 0.;
          thisProcessingReport.sensor_name = "IMUPreIntegrated";
          thisProcessingReport.num_measurements = counter_imu;
          thisProcessingReport.observation_available = true;

          if(pubIMUFactorReport_)
              pubIMUFactorReport_->publish(thisProcessingReport);

          // Const GNSS Clock error factor
          if (paramPtr_->useConstDriftFactor) {
            cbd_key_j   = C(nState_);
            cbd_key_i   = C(nState_ - 1);
            values_.insert(cbd_key_j, currentPredState.cbd);
            keyTimestampMap_[cbd_key_j]   = currentStateTimestamp;
            this->addConstDriftFactor(cbd_key_i, cbd_key_j, sum_imu_dt, currentPredState.cbdVar.diagonal());
          }

          //const vel prior factor
          if (paramPtr_->useMMFactor) {
            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "FGConIMU: state variable " << nState_ << " with MM factor.");
            this->addMotionModelFactor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, sum_imu_dt);
          }

          //GP prior
          if (paramPtr_->useGPPriorFactor) {
            if(paramPtr_->gpType == data_types::GPModelType::WNOJ) {
              this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, sum_imu_dt,
                                     lastAcc, currentAcc);
              lastAcc = currentAcc;
            }
            else
              this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, sum_imu_dt);
          }

          if (paramPtr_->useGPPriorFactor || paramPtr_->useGPInterpolatedFactor)
          {
            keyTimestampMap_[omega_key_j] = currentStateTimestamp; //TODO
            values_.insert(omega_key_j, currentPredState.omega);
          }
          // we reset sim_imu_dt for next interpolation.
          sum_imu_dt = 0.;
          counter_imu = 0;

          //current_pred_state.state = imuPreIntegrationOPT_->predict(current_pred_state.state, current_pred_state.imuBias);
          imuPreIntegrationOPT_->resetIntegration();
        }
      }

      /*
       *     Integrating sensor
       */

      bool integrationSuccessfully = true;

      auto statePair = currentPredictedBuffer_.get_all_time_buffer_pair();
      for(const auto& integrator : integratorMap_)
      {
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: starting integrating measurement from " << integrator.first);

        integrationSuccessfully &= integrator.second->factorize(timeGyroMap,
                                                                stateIDAccMap,
                                                                currentKeyIndexTimestampMap_,
                                                                statePair,
                                                                values_,
                                                                keyTimestampMap_,
                                                                relatedKeys_);
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: integrating measurement from " << integrator.first << " was " << (integrationSuccessfully ? "successful" : "failed!"));
      }

      if(paramPtr_->verbose)
       this->print("GraphTimeCentric: ");

      if (integrationSuccessfully)
        return StatusGraphConstruction::SUCCESSFUL;
      else
        return StatusGraphConstruction::FAILED;
    }

    StatusGraphConstruction GraphTimeCentric::constructFactorGraphOnTime(const vector<double> &stateTimestamps,
                                                                         std::vector<fgo::data_types::IMUMeasurement> &dataIMU) {
        // example: imu on 100Hz and we optimize on 10 Hz, then we should count 10 imu measurements
        static const double betweenOptimizationTime = 1. / paramPtr_->stateFrequency;
        static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(20 * graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->stateFrequency);
        static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> stateIDAccMap(50 * graphBaseParamPtr_->smootherLag * graphBaseParamPtr_->stateFrequency);
        static gtsam::Vector6 lastAcc = gtsam::Vector6::Zero();
        static gtsam::Vector3 meanAccA, meanAccG = gtsam::Vector3();
        static uint64_t lastNState = 0;
        static bool skippedOpt = false;

        gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, omega_key_j,
                pose_key_i, vel_key_i, bias_key_i, cbd_key_i, omega_key_i;

        bool hasMeasurements = true;
        double voteZeroVelocity = 0;
        for(const auto& integrator : integratorMap_)
        {
            hasMeasurements &= integrator.second->checkHasMeasurements();
            if(integrator.second->checkZeroVelocity())
            {
                RCLCPP_WARN_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime: " << integrator.first << " reported ZERO VELOCITY"); // this shouldn't happen

                voteZeroVelocity++;
            }
        }

        static double noOptimizationDuration = 0.;
        static rclcpp::Time firstNoOptimizationDecision;
        if(paramPtr_->NoOptimizationNearZeroVelocity && (nState_ - lastNState) > paramPtr_->NoOptimizationAfterStates)
        {
            if(voteZeroVelocity / integratorMap_.size() > paramPtr_->VoteNearZeroVelocity)
            {
                RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime: Near zero velocity with " << voteZeroVelocity << " integrator of " << integratorMap_.size() << " not optimizing...");

                if(!skippedOpt)
                    firstNoOptimizationDecision = appPtr_->now();
                noOptimizationDuration = (appPtr_->now() - firstNoOptimizationDecision).seconds();
                for(const auto& integrator : integratorMap_)
                {
                    integrator.second->cleanBuffers();
                }
                RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime: Near zero velocity duration " << noOptimizationDuration);

                skippedOpt = true;
                solver_->setNotMarginalizing();
                return StatusGraphConstruction::NO_OPTIMIZATION;
            }
        }

        auto currentPredState = currentPredictedBuffer_.get_last_buffer(); //graph::querryCurrentPredictedState(timePredStates, currentStateTimestamp);

        if(skippedOpt) {
            skippedOpt = false;
            lastNState = nState_;
            solver_->setSmootherLagInflation(noOptimizationDuration);

            //solver_->setMarginalizing();
            for(const auto& integrator : integratorMap_)
            {
                integrator.second->notifyOptimization(noOptimizationDuration);
            }

            // To overcome problems of timing drifting due to no optimization update, we add prior factor after optimization
            // has been stopped

            /*
            nState_++;

            this->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(nState_), currentPredState.state.pose(), currentPredState.poseVar);
            this->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(nState_), currentPredState.state.v(), currentPredState.velVar);
            this->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(nState_), currentPredState.imuBias, currentPredState.imuBiasVar);

            values_.insert(X(nState_), currentPredState.state.pose());
            values_.insert(V(nState_), currentPredState.state.v());
            values_.insert(B(nState_), currentPredState.imuBias);

            const auto first_imu = dataIMU.front();
            const auto current_state_timestamp = first_imu.timestamp.seconds();
            keyTimestampMap_[X(nState_)] =
            keyTimestampMap_[V(nState_)] =
            keyTimestampMap_[B(nState_)] = current_state_timestamp;


            if(graphBaseParamPtr_->useConstDriftFactor) {
              this->emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(C(nState_), currentPredState.cbd, currentPredState.cbdVar);
              values_.insert(C(nState_), currentPredState.cbd);
              keyTimestampMap_[C(nState_)] = current_state_timestamp;
            }

            currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, current_state_timestamp));

            if(graphBaseParamPtr_->useGPPriorFactor || graphBaseParamPtr_->useGPInterpolatedFactor)
            {
              this->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(nState_), currentPredState.omega, currentPredState.omegaVar);
              values_.insert(W(nState_), currentPredState.omega);
              keyTimestampMap_[W(nState_)] = current_state_timestamp;
            }

            if(!graphBaseParamPtr_->useGPPriorFactor  && graphBaseParamPtr_->useGPInterpolatedFactor)
            {
              fgo::data_types::IMUMeasurement tmpIMU;
              tmpIMU.gyro = currentPredState.omega;
              this->addAngularFactor(W(nState_), B(nState_), tmpIMU);
            }

            //const auto stateAccPair = std::make_pair(nState_, currentAcc);
            stateIDAccMap.push_back(std::make_pair(nState_, currentPredState.accMeasured));
            accBuffer_.update_buffer(currentPredState.accMeasured, first_imu.timestamp);
             */
            noOptimizationDuration = 0.;
        }

        if(paramPtr_->NoOptimizationWhileNoMeasurement && !hasMeasurements && nState_ > paramPtr_->NoOptimizationAfterStates)
        {
            RCLCPP_ERROR(appPtr_->get_logger(), "constructFactorGraphOnTime: NO Reference Measurement, not optimizing...");
            return StatusGraphConstruction::NO_OPTIMIZATION;
        }

        if(!dataIMURest_.empty())
        {
            dataIMU.insert(dataIMU.begin(), dataIMURest_.begin(), dataIMURest_.end());
            dataIMURest_.clear();
        }

        //const auto current_pred_state_timestamp = currentPredState.timestamp.seconds();
        //const auto first_imu_meas_timestamp = dataIMU.front().timestamp.seconds();  // in double
        //const auto last_imu_meas_timestamp = dataIMU.back().timestamp.seconds();  // in double

        if(paramPtr_->calibGravity)
            preIntegratorParams_->n_gravity = /*fgo::utils::nedRe_Matrix(lastOptimizedState_.state.position()) * */
                    fgo::utils::gravity_ecef(currentPredState.state.position());

        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
                std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                           currentPredState.imuBias);

        std::cout << "constructFactorGraphOnTime: start iterating" << std::endl;
        auto imuIter = dataIMU.begin();
        for(const auto& ts : stateTimestamps)
        {
            double imuCounter = 0;
            auto currentIMU = dataIMU.back();
            while(imuIter != dataIMU.end() && imuIter->timestamp.seconds() < ts)
            {
                currentIMU = *imuIter;
                timeGyroMap.push_back(std::make_pair(currentIMU.timestamp.seconds(), currentIMU.gyro));
                imuPreIntegrationOPT_->integrateMeasurement(currentIMU.accLin,
                                                            currentIMU.gyro,
                                                            currentIMU.dt);
                meanAccG += currentIMU.accRot;
                imuCounter += 1.;
                imuIter = dataIMU.erase(imuIter);
            }

            nState_++;
            pose_key_j  = X(nState_);
            vel_key_j   = V(nState_);
            bias_key_j  = B(nState_);
            omega_key_j = W(nState_);

            pose_key_i  = X(nState_ - 1);
            vel_key_i   = V(nState_ - 1);
            bias_key_i  = B(nState_ - 1);
            omega_key_i = W(nState_ - 1);

            meanAccG /= imuCounter;
            //meanAccA /= notifyCounter;
            currentPredState = graph::queryCurrentPredictedState(currentPredictedBuffer_.get_all_time_buffer_pair(), ts);

            gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
            if(paramPtr_->calibGravity)
            {
                const auto gravity = fgo::utils::gravity_ecef(currentPredState.state.position());
                gravity_b = currentPredState.state.attitude().unrotate(gravity);
            }

            auto currentAcc = (gtsam::Vector6() << currentIMU.accRot,
                    currentPredState.imuBias.correctAccelerometer(currentIMU.accLin + gravity_b)).finished();
            //const auto stateAccPair = std::make_pair(nState_, currentAcc);
            stateIDAccMap.push_back(std::make_pair(nState_, currentAcc));
            accBuffer_.update_buffer(currentAcc, currentIMU.timestamp);
            meanAccG.setZero();
            //meanAccA.setZero();

            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime:  creating state variable " << nState_
                                                                                           << " at: " << std::fixed << ts
                                                                                           << " current IMU time " << currentIMU.timestamp.seconds()
                                                                                           << " queried predicted state at " << currentPredState.timestamp.seconds());

            currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, ts));
            // auto predictedState = imuPreIntegrationOPTTmp_->predict(current_pred_state.state, current_pred_state.imuBias);

            //setup values
            values_.insert(pose_key_j, currentPredState.state.pose());
            values_.insert(vel_key_j, currentPredState.state.velocity());
            values_.insert(bias_key_j, currentPredState.imuBias);

            keyTimestampMap_[pose_key_j]  =
            keyTimestampMap_[vel_key_j]   =
            keyTimestampMap_[bias_key_j]  = ts;

            //add acc to map
            // IMU Factor
            boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor = //make new IMU Factor
                    boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                                 pose_key_j, vel_key_j,
                                                                 bias_key_i, bias_key_j,
                                                                 *imuPreIntegrationOPT_);
            imu_factor->setTypeID(fgo::factor::FactorTypeIDs::CombinedIMU);
            imu_factor->setName("CombinedImuFactor");
            this->push_back(imu_factor);

            irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
            thisProcessingReport.header.stamp = appPtr_->now();
            thisProcessingReport.ts_measurement = appPtr_->now().seconds();
            thisProcessingReport.duration_processing = betweenOptimizationTime;
            thisProcessingReport.measurement_delay = 0.;
            thisProcessingReport.sensor_name = "IMUPreIntegrated";
            thisProcessingReport.num_measurements = imuCounter;
            thisProcessingReport.observation_available = true;

            if(pubIMUFactorReport_)
                pubIMUFactorReport_->publish(thisProcessingReport);

            // Const GNSS Clock error factor
            if (paramPtr_->useConstDriftFactor) {
                cbd_key_j   = C(nState_);
                cbd_key_i   = C(nState_ - 1);
                values_.insert(cbd_key_j, currentPredState.cbd);
                keyTimestampMap_[cbd_key_j]   = ts;
                this->addConstDriftFactor(cbd_key_i, cbd_key_j, betweenOptimizationTime, currentPredState.cbdVar.diagonal());
            }

            //const vel prior factor
            if (paramPtr_->useMMFactor) {
                RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime:  state variable " << nState_ << " with MM factor.");
                this->addMotionModelFactor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, betweenOptimizationTime);
            }

            //GP prior
            if (paramPtr_->useGPPriorFactor) {
                if(paramPtr_->gpType == data_types::GPModelType::WNOJ) {
                    this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, betweenOptimizationTime,
                                           lastAcc, currentAcc);
                    lastAcc = currentAcc;
                }
                else
                    this->addGPMotionPrior(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j, betweenOptimizationTime);
            }

            if (paramPtr_->useGPPriorFactor || paramPtr_->useGPInterpolatedFactor)
            {
                keyTimestampMap_[omega_key_j] = ts; //TODO
                values_.insert(omega_key_j, currentPredState.omega);
            }
            // we reset sim_imu_dt for next interpolation.
            //current_pred_state.state = imuPreIntegrationOPT_->predict(current_pred_state.state, current_pred_state.imuBias);
            imuPreIntegrationOPT_->resetIntegration();
        }

        // if there are still imu measurements left over, we back them up

        if(!dataIMU.empty())
        {
            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constructFactorGraphOnTime: " << dataIMU.size() << " imu measurements are left. Backing up...");
            dataIMURest_.resize(dataIMU.size());
            std::copy(dataIMU.begin(), dataIMU.end(), dataIMURest_.begin());
        }

        /*
       *     Integrating sensor
       */

        bool integrationSuccessfully = true;

        auto statePair = currentPredictedBuffer_.get_all_time_buffer_pair();
        for(const auto& integrator : integratorMap_)
        {
            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: starting integrating measurement from " << integrator.first);

            integrationSuccessfully &= integrator.second->factorize(timeGyroMap,
                                                                    stateIDAccMap,
                                                                    currentKeyIndexTimestampMap_,
                                                                    statePair,
                                                                    values_,
                                                                    keyTimestampMap_,
                                                                    relatedKeys_);
            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: integrating measurement from " << integrator.first << " was " << (integrationSuccessfully ? "successful" : "failed!"));
        }

        if(paramPtr_->verbose)
            this->print("GraphTimeCentric: ");

        if (integrationSuccessfully)
            return StatusGraphConstruction::SUCCESSFUL;
        else
            return StatusGraphConstruction::FAILED;
    }

    double GraphTimeCentric::optimize(data_types::State &new_state) {

      // when we call this function, it means that the optimization was triggered, no matter where the trigger came from
      // this function is then stand alone and independent of threading organization
      std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

      solver_->update(*this, values_, keyTimestampMap_, gtsam::FactorIndices(), relatedKeys_);
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: updating finished!");

      gtsam::Values result = solver_->calculateEstimate();

      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: calculateEstimate finished!");
      gtsam::Marginals marginals;

      currentKeyIndexTimestampMap_ = solver_->keyIndexTimestamps();

      new_state.timestamp = rclcpp::Time(keyTimestampMap_[X(nState_)] * fgo::constants::sec2nanosec, RCL_ROS_TIME);

      if(graphBaseParamPtr_->publishResiduals)
      {
        std::chrono::time_point<std::chrono::system_clock> start_copy_factors = std::chrono::system_clock::now();
        //graphBuffer_.update_buffer(solver_->getFactors(), new_state.timestamp);
        graphResultBuffer_.update_buffer(result, new_state.timestamp);

        std::vector<gtsam::NonlinearFactor::shared_ptr> factorVec;
        std::for_each(begin(), end(), [&](const gtsam::NonlinearFactor::shared_ptr &factor) -> void {
            factorVec.emplace_back(factor);
        });
        factorBuffer_.update_buffer(factorVec, new_state.timestamp);
        factorBuffer_.cleanBeforeTime(currentKeyIndexTimestampMap_.begin()->second);
        auto timeCopyFactors = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start_copy_factors).count();
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Finished copy factor: " << std::fixed << timeCopyFactors);

      }

      try
      {
        marginals = solver_->getMarginals(result);
        new_state.poseVar = marginals.marginalCovariance(X(nState_));
        //std::cout << "PosVar" << lastOptimizedState_.poseVar << std::endl;
        new_state.velVar = marginals.marginalCovariance(V(nState_));
        new_state.imuBiasVar = marginals.marginalCovariance(B(nState_));
        if(graphBaseParamPtr_->useEstimatedVarianceAfterInit)
        {
          preIntegratorParams_->biasAccCovariance = new_state.imuBiasVar.block<3, 3>(0, 0);
          preIntegratorParams_->biasOmegaCovariance = new_state.imuBiasVar.block<3, 3>(3, 3);
        }

        if(paramPtr_->useConstDriftFactor)
        {
          new_state.cbd = result.at<gtsam::Vector2>(C(nState_));
          //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "CBD: " << new_state.cbd);
          new_state.cbdVar = marginals.marginalCovariance(C(nState_));
        }
        if(paramPtr_->useGPPriorFactor || paramPtr_->useGPInterpolatedFactor)
        {
          new_state.omega = result.at<gtsam::Vector3>(W(nState_));
          new_state.omegaVar = marginals.marginalCovariance(W(nState_));
        }


      }
      catch(std::exception &ex)
      {
        RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "SOLVING with exception: " << ex.what());
      }

      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: solving finished!");
      solver_->setMarginalizing();
      new_state.state = gtsam::NavState(result.at<gtsam::Pose3>(X(nState_)),
                                        result.at<gtsam::Vector3>(V(nState_)));
      //RCLCPP_INFO_STREAM(this->get_logger(), "State: " << std::fixed << lastOptimizedState_.state);

      new_state.imuBias = result.at<gtsam::imuBias::ConstantBias>(B(nState_));
      //RCLCPP_INFO_STREAM(this->get_logger(), "Bias: " << lastOptimizedState_.imuBias);


      new_state.accMeasured = accBuffer_.get_buffer_from_id(nState_ - 1);

      /*
       * Fetching results for all integrator
       */

      bool fetchingResultsSuccessful = true;
      for(const auto& integrator : integratorMap_)
      {
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: starting fetching results for the integrator: " << integrator.first);

        fetchingResultsSuccessful &= integrator.second->fetchResult(result,
                                                                    marginals,
                                                                    currentKeyIndexTimestampMap_,
                                                                    new_state);

        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: fetching results for the integrator " << integrator.first << " was " << (fetchingResultsSuccessful ? "successful" : "failed!"));
      }

      this->resetGraph();
      auto timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
      if(graphBaseParamPtr_->verbose){
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Finished graph optimization, fetching results ..., TimeOpt: " << timeOpt);
      }

      return timeOpt;
    }

}

