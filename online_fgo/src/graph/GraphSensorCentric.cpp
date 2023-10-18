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


#include "graph/GraphSensorCentric.h"
#include "gnss_fgo/GNSSFGOLocalizationBase.h"

namespace fgo::graph
{
    GraphSensorCentric::GraphSensorCentric(gnss_fgo::GNSSFGOLocalizationBase &node)
            : GraphTimeCentric(node) {

        RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphSensorCentric initializing! --------------------- ");
        paramPtr_ = std::static_pointer_cast<GraphSensorCentricParam>(graphBaseParamPtr_);
        RCLCPP_INFO(appPtr_->get_logger(), "--------------------- GraphSensorCentric initialized! ---------------------");

    }

    StatusGraphConstruction GraphSensorCentric::constructFactorGraphOnIMU(std::vector<fgo::data_types::IMUMeasurement> &dataIMU) {

        static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(20 * graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->optFrequency);
        static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> stateIDAccMap(50 * graphBaseParamPtr_->smootherLag * graphBaseParamPtr_->optFrequency);
        static gtsam::Vector6 lastAcc = gtsam::Vector6::Zero();
        static gtsam::Vector3 meanAccA, meanAccG = gtsam::Vector3();
        static uint64_t lastNState = 0;
        static bool skippedOpt = false;

        if (!dataIMURest_.empty()) {
            dataIMU.insert(dataIMU.begin(),dataIMURest_.begin(),dataIMURest_.end());
            dataIMURest_.clear();
        }

        auto currentPredState = currentPredictedBuffer_.get_first_buffer(); //graph::querryCurrentPredictedState(timePredStates, currentStateTimestamp);

        if(paramPtr_->calibGravity)
            preIntegratorParams_->n_gravity = /*fgo::utils::nedRe_Matrix(lastOptimizedState_.state.position()) * */
                    fgo::utils::gravity_ecef(currentPredState.state.position());

        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
                std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                           currentPredState.imuBias);

        const auto keyIndexTimestampsMap = primarySensor_->factorizeAsPrimarySensor();

        const auto first_imu_meas_timestamp = dataIMU.front().timestamp.seconds();  // in double
        auto imu_meas_iter = dataIMU.begin();
        fgo::data_types::IMUMeasurement imu_tmp = dataIMU.back();  // we need a temporary data holder of the synchronized IMU measurement

        gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, omega_key_j,
                   pose_key_i, vel_key_i, bias_key_i, cbd_key_i, omega_key_i;

        for(const auto& idTsMap : keyIndexTimestampsMap)
        {
            const auto& id = idTsMap.first;
            const auto& ts = idTsMap.second;
            currentPredState = currentPredictedBuffer_.get_buffer(ts);

            if(ts < first_imu_meas_timestamp)
            {

            }
            else
            {
                double sum_imu_dt = 0.;

                while(imu_meas_iter != dataIMU.end())
                {
                    //RCLCPP_INFO_STREAM(this->get_logger(), "Try Preintegrate IMU:" << std::fixed << imu_meas_iter->timestamp.seconds() << " vs GNSS: " << std::fixed << corrected_time_gnss_meas );
                    // we want to integrate the imu measurements for this measurement in two cases
                    // 1. for all IMU measurements earlier than this meas OR
                    // 2. for the IMU measurement which came later than this meas, but they are still synchronized
                    imu_tmp = *imu_meas_iter;
                    if(imu_meas_iter->timestamp.seconds() < ts)
                    {
                        imuPreIntegrationOPT_->integrateMeasurement(imu_meas_iter->accLin,
                                                                    imu_meas_iter->gyro,
                                                                    imu_meas_iter->dt);
                        sum_imu_dt += imu_meas_iter->dt;
                        timeGyroMap.push_back(std::pair<double, gtsam::Vector3>(imu_meas_iter->timestamp.seconds(), imu_meas_iter->gyro));
                        meanAccG += imu_meas_iter->accRot;
                        imu_meas_iter = dataIMU.erase(imu_meas_iter);
                    } else
                    {
                        break;
                    }
                }

                gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
                if(paramPtr_->calibGravity)
                {
                    const auto gravity = fgo::utils::gravity_ecef(currentPredState.state.position());
                    gravity_b = currentPredState.state.attitude().unrotate(gravity);
                }

                auto currentAcc = (gtsam::Vector6() << imu_tmp.accRot,
                        currentPredState.imuBias.correctAccelerometer(imu_tmp.accLin + gravity_b)).finished();

                //const auto stateAccPair = std::make_pair(nState_, currentAcc);
                stateIDAccMap.push_back(std::make_pair(nState_, currentAcc));
                accBuffer_.update_buffer(currentAcc, imu_tmp.timestamp);
                meanAccG.setZero();

                pose_key_j  = X(id);
                vel_key_j   = V(id);
                bias_key_j  = B(id);
                omega_key_j = W(id);
                pose_key_i  = X(id - 1);
                vel_key_i   = V(id - 1);
                bias_key_i  = B(id - 1);
                omega_key_i = W(id - 1);

                currentKeyIndexTimestampMap_.insert(std::make_pair(id, ts));
                //setup values
                values_.insert(pose_key_j, currentPredState.state.pose());
                values_.insert(vel_key_j, currentPredState.state.velocity());
                values_.insert(bias_key_j, currentPredState.imuBias);

                keyTimestampMap_[pose_key_j]  =
                keyTimestampMap_[vel_key_j]   =
                keyTimestampMap_[bias_key_j]  = ts;

                // IMU Factor
                boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor = //make new IMU Factor
                        boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                                     pose_key_j, vel_key_j,
                                                                     bias_key_i, bias_key_j,
                                                                     *imuPreIntegrationOPT_);

                imu_factor->setTypeID(fgo::factor::FactorTypeIDs::CombinedIMU);
                imu_factor->setName("CombinedImuFactor");
                this->push_back(imu_factor);

                // Const GNSS Clock error factor
                if (paramPtr_->useConstDriftFactor) {
                    cbd_key_j   = C(id);
                    cbd_key_i   = C(id - 1);
                    values_.insert(cbd_key_j, currentPredState.cbd);
                    keyTimestampMap_[cbd_key_j]   = ts;
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
                    keyTimestampMap_[omega_key_j] = ts; //TODO
                    values_.insert(omega_key_j, currentPredState.omega);
                }

                //current_pred_state.state = imuPreIntegrationOPT_->predict(current_pred_state.state, current_pred_state.imuBias);
                imuPreIntegrationOPT_->resetIntegration();
            }
        }

        dataIMURest_ = dataIMU;

        /*
         *     Integrating other sensor
         */

        bool integrationSuccessfully = true;
        auto statePair = currentPredictedBuffer_.get_all_time_buffer_pair();
        for(const auto& sensor : integratorMap_)
        {
            if(sensor.second->isPrimarySensor())
                continue;

            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: starting integrating measurement from " << sensor.first);

            integrationSuccessfully &= sensor.second->factorize(timeGyroMap,
                                                                    stateIDAccMap,
                                                                    currentKeyIndexTimestampMap_,
                                                                    statePair,
                                                                    values_,
                                                                    keyTimestampMap_,
                                                                    relatedKeys_);
            RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: integrating measurement from " << sensor.first << " was " << (integrationSuccessfully ? "successful" : "failed!"));

        }
        return SUCCESSFUL;
    }

    StatusGraphConstruction GraphSensorCentric::constructFactorGraphOnTime(const vector<double> &stateTimestamps,
                                                                           std::vector<fgo::data_types::IMUMeasurement> &dataIMU) {

        return FAILED;
    }


}

