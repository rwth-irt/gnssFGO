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

//
// Created by haoming on 14.06.23.
//



// STD
#include <iostream>
#include "offline_fgo/OfflineTimeCentric.h"

namespace offline_fgo {

    OfflineFGOTimeCentric::OfflineFGOTimeCentric(const rclcpp::NodeOptions& opt) : OnlineFGOBase("OnlineFGOTimeCentricNode", opt){
        RCLCPP_INFO(this->get_logger(), "---------------------  OfflineFGOTimeCentricNode initializing! --------------------- ");

        utils::RosParameter<std::string> bag_path("OfflineFGO.bag_path", *this);
        utils::RosParameter<std::vector<std::string>> topic_filter("OfflineFGO.topic_filter",
                                                                   {"/imu/data","/irt_gnss_preprocessing/gnss_obs_preprocessed"}, *this);
        reader_ = std::make_unique<BagReader>(bag_path.value(),topic_filter.value());
        reader_->parseDataFromBag();

        graph_ = std::make_shared<fgo::graph::GraphTimeCentric>(*this, paramsPtr_);
        GNSSDelayCalculator_ = std::make_unique<fgo::utils::MeasurementDelayCalculator>(*this,
                                                                                        callbackGroupMap_["GNSSDelayCalculator"],
                                                                                        false);

        gyroBiasInitializer_ = std::make_unique<InitGyroBias>();
        gyroBiasInitializer_->initialize();
        /*
        statePVTInitializer_ = std::make_unique<InitStatePVT>();
        statePVTInitializer_->initialize();
        */

        optThread_ = std::make_unique<std::thread>([this]() -> void {
            this->doOfflineFGOProcess();});

        RCLCPP_INFO(this->get_logger(), "---------------------  OnlineFGOTimeCentricNode initialized! --------------------- ");
    }

    void OfflineFGOTimeCentric::doOfflineFGOProcess() {
        const auto first_GNSS = reader_->getNextGNSSMeasurement();
        // After we got first GNSS, we need to find a PVA measurement which can be associated with the GNSS
        // using tow

        auto first_PVA = reader_->getNextPVASolution();
        while (first_PVA.tow != first_GNSS.tow)
            first_PVA = reader_->getNextPVASolution();

        if (first_PVA.tow != first_GNSS.tow)
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Can't get PVA associated with the first GNSS obs. at " << std::fixed
                                                                                        << first_GNSS.tow);

        // now we can initialize the FGO with PVA solution
        /**
         * Init FGO
         */
        //get imu corresponding to first_PVA
        const rclcpp::Time &initial_time = first_PVA.timestamp;
        auto first_IMU = reader_->getNextIMUMeasurement();
        while (first_IMU.timestamp.seconds() < initial_time.seconds()) {
            first_IMU = reader_->getNextIMUMeasurement();
        }

        // initial lastOptimizedState with PVA and imu data
        gtsam::Rot3 init_nRb = gtsam::Rot3::Yaw(first_PVA.heading);  //roll_pitch=?
        gtsam::Rot3 init_nedRe(fgo::utils::nedRe_Matrix_asLLH(first_PVA.llh));
        gtsam::Rot3 init_eRb = init_nedRe.inverse() * init_nRb;
        lastOptimizedState_.timestamp = initial_time;
        if (!paramsPtr_->initGyroBiasAsZero) {
            auto gyroBias = initGyroBias_.back();
            lastOptimizedState_.imuBias = gtsam::imuBias::ConstantBias((gtsam::Vector3() << 0., 0., 0.).finished(),
                                                                       (gtsam::Vector3()
                                                                               << gyroBias[0], gyroBias[1], gyroBias[2]).finished());
        }
        // clock bias and clock drift
        lastOptimizedState_.cbd = (gtsam::Vector2() << first_PVA.clock_bias, first_PVA.clock_drift).finished();
        lastOptimizedState_.omega = first_IMU.gyro;
        //initial position of the antenna in Earth-Centered Earth-Fixed (ECEF)
        auto init_ant_pos = fgo::utils::llh2xyz(
                (gtsam::Point3() << first_PVA.llh[0], first_PVA.llh[1], first_PVA.llh[2]).finished());
        //initial position of the IMU in ECEF coordinates
        auto init_imu_pos = init_ant_pos - init_eRb.rotate(paramsPtr_->transIMUToAnt1);
        auto vecGrav = fgo::utils::gravity_ecef(init_imu_pos);
        auto gravity = fgo::utils::gravity_ecef(first_PVA.xyz_ecef);
        auto gravity_b = init_eRb.unrotate(vecGrav);
        lastOptimizedState_.accMeasured = (gtsam::Vector6() << first_IMU.accRot, first_IMU.accLin + gravity_b).finished();
        lastOptimizedState_.state = gtsam::NavState(init_eRb, init_imu_pos,
                                                    init_nedRe.inverse() * (gtsam::Velocity3() <<
                                                                                               first_PVA.vel_n[0], first_PVA.vel_n[1], first_PVA.vel_n[2]).finished());

        //set up imu_preIntegrator
        preIntegratorParams_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(vecGrav);
        std::cout << std::fixed << "vecGrav: " << vecGrav << std::endl;
        std::cout << std::fixed << "vecGravBody: " << gravity_b << std::endl;
        //imu_params->setBodyPSensor() possible
        preIntegratorParams_->accelerometerCovariance =
                pow(paramsPtr_->accelerometerSigma, 2) * gtsam::I_3x3; //Covariance of Sensor
        preIntegratorParams_->integrationCovariance = pow(paramsPtr_->integrationSigma, 2) * gtsam::I_3x3;
        preIntegratorParams_->gyroscopeCovariance = pow(paramsPtr_->gyroscopeSigma, 2) * gtsam::I_3x3;
        preIntegratorParams_->biasAccCovariance = pow(paramsPtr_->biasAccSigma, 2) * gtsam::I_3x3; //Covariance of Bias
        preIntegratorParams_->biasOmegaCovariance = pow(paramsPtr_->biasOmegaSigma, 2) * gtsam::I_3x3;
        preIntegratorParams_->biasAccOmegaInt = paramsPtr_->biasAccOmegaInt * gtsam::I_6x6;
        preIntegratorParams_->omegaCoriolis = gtsam::Vector3(0, 0, fgo::constants::earthRot); //Coriolis
        preIntegratorParams_->setUse2ndOrderCoriolis(true);
        currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                                              lastOptimizedState_.imuBias);
        graph_->initGraph(lastOptimizedState_, initial_time.seconds(), preIntegratorParams_);

        lastOptimizedState_.poseVar.block<3,3>(3,3) =
                init_nedRe.transpose() * lastOptimizedState_.poseVar.block<3,3>(3,3) * init_nedRe.inverse().matrix();

        currentPredState_ = lastOptimizedState_;
        //add first state time
        statetimeContainer_.insert({0,first_IMU.timestamp.seconds()});
        fgoOptStateContainer_.insert(make_pair(initial_time, currentPredState_));
        fgoPredStateContainer_.insert(make_pair(initial_time, currentPredState_));
        //publish
        fgoStateOptPub_->publish(this->convertFGOStateToMsg(lastOptimizedState_));
        fgoStateOptNavFixPub_->publish(
                this->convertPositionToNavFixMsg(lastOptimizedState_.state, lastOptimizedState_.timestamp, true));
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "(Re-)Initializing FGO successful at: " << std::fixed << initial_time.seconds());

        /**
         * Main Loop
         */
        //set up graph

        size_t imuCounter = 1;
        size_t nState = 0;

        const auto notifyExtendGraph = paramsPtr_->IMUMeasurementFrequency / paramsPtr_->optFrequency;
        while (reader_->hasNextIMUMeasurement()) {
            auto imu = reader_->getNextIMUMeasurement();

            currentIMUPreintegrator_->integrateMeasurement(imu.accLin, imu.gyro, imu.dt);

            if (imuCounter > 1 && imuCounter % notifyExtendGraph == 0) {
                nState++;

                const auto newStateTimestamp = imu.timestamp.seconds();

                // reserve every 10th imu timestamp for later gnss timestamp comparison
                statetimeContainer_.insert({nState,imu.timestamp.seconds()});

                //Adding IMU factor and GNSS factor and optimizing.
                auto gnss = reader_->getNextGNSSMeasurement();
                auto pva = reader_->getNextPVASolution();
                //collected IMU measurement
                auto preint_imu_combined =
                        dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(*currentIMUPreintegrator_);


            }
            // predict current state to offer a reliable initialization point for next iteration of solving.
            auto prop_state = currentIMUPreintegrator_->predict(lastOptimizedState_.state,lastOptimizedState_.imuBias);
            currentPredState_.state = prop_state;
            //+0.01s?
            currentPredState_.timestamp = lastOptimizedState_.timestamp + rclcpp::Duration::from_nanoseconds(imu.dt * fgo::constants::sec2nanosec);
            currentPredState_.omega = lastOptimizedState_.imuBias.correctGyroscope(imu.gyro);
            currentPredState_.omegaVar = lastOptimizedState_.omegaVar;
            currentPredState_.cbd = lastOptimizedState_.cbd;
            currentPredState_.cbdVar = lastOptimizedState_.cbdVar;
            currentPredState_.imuBias = lastOptimizedState_.imuBias;
            currentPredState_.imuBiasVar = lastOptimizedState_.imuBiasVar;
            currentPredState_.poseVar = lastOptimizedState_.poseVar;
            currentPredState_.velVar = lastOptimizedState_.velVar;
            currentPredState_.ddIntAmb = lastOptimizedState_.ddIntAmb;
            currentPredState_.ddIntAmbVar = lastOptimizedState_.ddIntAmbVar;


            imuCounter++;
        }
    }







}




