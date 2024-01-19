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

#include "gnss_fgo/GNSSFGOLocalizationBase.h"

namespace gnss_fgo
{
    bool GNSSFGOLocalizationBase::initializeCommon()
    {
        if(paramsPtr_ == nullptr)
            paramsPtr_ = std::make_shared<GNSSFGOParams>();

        utils::RosParameter<double> pvtMeasTimeOffset("GNSSFGO.pvtMeasTimeOffset", 0.0, *this);
        paramsPtr_->pvtMeasTimeOffset = pvtMeasTimeOffset.value();

        utils::RosParameter<bool> delayFromPPS("GNSSFGO.delayFromPPS", false, *this);
        paramsPtr_->delayFromPPS = delayFromPPS.value(); // only if PRDR activated

        utils::RosParameter<bool> verbose("GNSSFGO.verbose", false, *this);
        paramsPtr_->verbose = verbose.value();

        utils::RosParameter<bool> useIMUAsTimeReference("GNSSFGO.useIMUAsTimeReference", false, *this);
        paramsPtr_->useIMUAsTimeReference = useIMUAsTimeReference.value();

        utils::RosParameter<bool> calibGravity("GNSSFGO.calibGravity", true, *this);
        paramsPtr_->calibGravity = calibGravity.value();

        utils::RosParameter<int> optFrequency("GNSSFGO.optFrequency", 10, *this);
        paramsPtr_->optFrequency = optFrequency.value();

        utils::RosParameter<int> stateFrequency("GNSSFGO.stateFrequency", 10, *this);
        paramsPtr_->stateFrequency = stateFrequency.value();


        utils::RosParameter<bool> calcErrorOnOpt("GNSSFGO.calcErrorOnOpt", false, *this);
        paramsPtr_->calcErrorOnOpt = calcErrorOnOpt.value();

        //initial states sigmas
        utils::RosParameter<std::vector<double>> initSigmaX("GNSSFGO.Initialization.initSigmaX", *this);
        lastOptimizedState_.poseVar = gtsam::Vector6(initSigmaX.value().data()).asDiagonal();
        lastOptimizedState_.poseVar.block<3,3>(0,0) = lastOptimizedState_.poseVar.block<3,3>(0,0);
        RCLCPP_INFO_STREAM(get_logger(), "poseVar:" << lastOptimizedState_.poseVar);

        utils::RosParameter<std::vector<double>> initSigmaV("GNSSFGO.Initialization.initSigmaV", *this);
        lastOptimizedState_.velVar = gtsam::Vector3(initSigmaV.value().data()).asDiagonal();
        RCLCPP_INFO_STREAM(get_logger(), "velVar:" << lastOptimizedState_.velVar);

        utils::RosParameter<std::vector<double>> initSigmaW("GNSSFGO.Initialization.initSigmaW", *this);
        lastOptimizedState_.omegaVar = gtsam::Vector3(initSigmaW.value().data()).asDiagonal();
        lastOptimizedState_.omegaVar = lastOptimizedState_.omegaVar;
        RCLCPP_INFO_STREAM(get_logger(), "omegaVar:" << lastOptimizedState_.omegaVar);

        utils::RosParameter<std::vector<double>> initSigmaB("GNSSFGO.Initialization.initSigmaB", *this);
        lastOptimizedState_.imuBiasVar = gtsam::Vector6(initSigmaB.value().data()).asDiagonal();
        lastOptimizedState_.imuBiasVar.block<3,3>(3,3) = lastOptimizedState_.imuBiasVar.block<3,3>(3,3);
        RCLCPP_INFO_STREAM(get_logger(), "imuBiasVar:" << lastOptimizedState_.imuBiasVar);

        utils::RosParameter<std::vector<double>> initSigmaC("GNSSFGO.Initialization.initSigmaC", *this);
        lastOptimizedState_.cbdVar = gtsam::Vector2(initSigmaC.value().data()).asDiagonal();
        RCLCPP_INFO_STREAM(get_logger(), "cbdVar:" << lastOptimizedState_.cbdVar);

        //Imu parameters
        utils::RosParameter<double> accelerometerSigma("GNSSFGO.IMUPreintegrator.accelerometerSigma", 1.0, *this);
        paramsPtr_->accelerometerSigma = accelerometerSigma.value();

        utils::RosParameter<double> integrationSigma("GNSSFGO.IMUPreintegrator.integrationSigma", 1.0, *this);
        paramsPtr_->integrationSigma = integrationSigma.value();
        RCLCPP_INFO_STREAM(get_logger(), "integrationSigma:" << paramsPtr_->integrationSigma);

        utils::RosParameter<double> gyroscopeSigma("GNSSFGO.IMUPreintegrator.gyroscopeSigma", 1.0, *this);
        paramsPtr_->gyroscopeSigma = gyroscopeSigma.value();
        RCLCPP_INFO_STREAM(get_logger(), "gyroscopeSigma:" << paramsPtr_->gyroscopeSigma);

        utils::RosParameter<double> biasAccSigma("GNSSFGO.IMUPreintegrator.biasAccSigma", 4e-4, *this);
        paramsPtr_->biasAccSigma = biasAccSigma.value();
        RCLCPP_INFO_STREAM(get_logger(), "biasAccSigma:" << paramsPtr_->biasAccSigma);

        utils::RosParameter<double> biasOmegaSigma("GNSSFGO.IMUPreintegrator.biasOmegaSigma", 87e-5, *this);
        paramsPtr_->biasOmegaSigma = biasOmegaSigma.value();
        RCLCPP_INFO_STREAM(get_logger(), "biasOmegaSigma:" << paramsPtr_->biasOmegaSigma);

        utils::RosParameter<double> biasAccOmegaInt("GNSSFGO.IMUPreintegrator.biasAccOmegaInt", 87e-5, *this);
        paramsPtr_->biasAccOmegaInt = biasAccOmegaInt.value();
        RCLCPP_INFO_STREAM(get_logger(), "biasAccOmegaInt:" << paramsPtr_->biasAccOmegaInt);

        // graph
        utils::RosParameter<int> IMUMeasurementFrequency("GNSSFGO.Graph.IMUMeasurementFrequency", 100, *this);
        paramsPtr_->IMUMeasurementFrequency = IMUMeasurementFrequency.value();
        RCLCPP_INFO_STREAM(get_logger(), "IMUMeasurementFrequency: " << paramsPtr_->IMUMeasurementFrequency );

        utils::RosParameter<bool> useGPPriorFactor("GNSSFGO.Graph.useGPPriorFactor", false, *this);
        paramsPtr_->useGPPriorFactor = useGPPriorFactor.value();
        RCLCPP_INFO_STREAM(get_logger(), "useGPPriorFactor:" << (paramsPtr_->useGPPriorFactor ? "true" : "false"));

        utils::RosParameter<std::string> gpType("GNSSFGO.Graph.gpType", "WNOA", *this);
        if( gpType.value() == "WNOA")
            paramsPtr_->gpType = fgo::data_types::GPModelType::WNOA;
        else
            paramsPtr_->gpType = fgo::data_types::GPModelType::WNOJ;
        RCLCPP_INFO_STREAM(this->get_logger(), "gpType: " << paramsPtr_->gpType);

        utils::RosParameter<bool> UseGPInterpolatedFactor("GNSSFGO.Graph.useGPInterpolatedFactor", *this);
        paramsPtr_->useGPInterpolatedFactor = UseGPInterpolatedFactor.value();
        RCLCPP_INFO_STREAM(get_logger(), "useGPInterpolatedFactor: " << (paramsPtr_->useGPInterpolatedFactor ? "true" : "false"));

        // vehicle param
        utils::RosParameter<std::vector<double>> lb("GNSSFGO.VehicleParameters.transIMUToGNSSAnt1", *this);
        paramsPtr_->transIMUToAnt1 = gtsam::Vector3(lb.value().data());

        utils::RosParameter<std::vector<double>> lb2("GNSSFGO.VehicleParameters.transIMUToGNSSAnt2", *this);
        paramsPtr_->transIMUToAnt2 = gtsam::Vector3(lb2.value().data());

        utils::RosParameter<std::vector<double>> lb3("GNSSFGO.VehicleParameters.transIMUToReference", *this);
        paramsPtr_->transIMUToReference = gtsam::Vector3(lb3.value().data());

        utils::RosParameter<std::vector<double>> lbRotRef("GNSSFGO.VehicleParameters.rotIMUToReference", *this);
        const auto rotRef = lbRotRef.value();
        paramsPtr_->rotIMUToReference = gtsam::Rot3(rotRef[0], rotRef[1], rotRef[2],
                                                    rotRef[3], rotRef[4], rotRef[5],
                                                    rotRef[6], rotRef[7], rotRef[8]);

        utils::RosParameter<std::vector<double>> lb4("GNSSFGO.VehicleParameters.transIMUToLiDAR", *this);
        paramsPtr_->transIMUToLiDAR = gtsam::Vector3(lb4.value().data());

        utils::RosParameter<std::vector<double>> lbRotLiDAR("GNSSFGO.VehicleParameters.rotIMUToLiDAR", *this);
        const auto rotLiDAR = lbRotLiDAR.value();
        paramsPtr_->rotIMUtoLiDAR = gtsam::Rot3(rotLiDAR[0], rotLiDAR[1], rotLiDAR[2],
                                                rotLiDAR[3], rotLiDAR[4], rotLiDAR[5],
                                                rotLiDAR[6], rotLiDAR[7], rotLiDAR[8]);

        std::vector<double> imuRotRaw = {1., 0, 0, 0, 1, 0, 0, 0,1};
        utils::RosParameter<std::vector<double>> imuRot("GNSSFGO.VehicleParameters.IMURot", imuRotRaw, *this);
        const auto imuRot_ = imuRot.value();
        paramsPtr_->imuRot = gtsam::Rot3(imuRot_[0], imuRot_[1], imuRot_[2],
                                         imuRot_[3], imuRot_[4], imuRot_[5],
                                         imuRot_[6], imuRot_[7], imuRot_[8]);

        utils::RosParameter<bool> cleanIMUonInit("GNSSFGO.cleanIMUonInit", true, *this);
        paramsPtr_->cleanIMUonInit = cleanIMUonInit.value();
        utils::RosParameter<int> bufferSizeinSec("GNSSFGO.bufferSize", 5, *this);
        paramsPtr_->bufferSize = bufferSizeinSec.value();

        utils::RosParameter<bool> useHeaderTimestamp("GNSSFGO.useHeaderTimestamp", *this);
        paramsPtr_->useHeaderTimestamp = useHeaderTimestamp.value();
        RCLCPP_INFO_STREAM(this->get_logger(), "useHeaderTimestamp: " << paramsPtr_->useHeaderTimestamp);

        utils::RosParameter<bool> pub_pose_vel("GNSSFGO.Publish.pubPoseVel", true, *this);
        if (pub_pose_vel.value())
        {
            posePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("GNSSFGO/pose",
                                                                                             rclcpp::SystemDefaultsQoS());

            velPub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("GNSSFGO/vel",
                                                                                             rclcpp::SystemDefaultsQoS());
        }
        utils::RosParameter<bool> pub_fgo_state("GNSSFGO.Publish.pubFGOState", true, *this);
        if (pub_fgo_state.value())
        {
            fgoStatePredPub_ = this->create_publisher<irt_nav_msgs::msg::FGOState>("statePredicted",
                                                                                   rclcpp::SystemDefaultsQoS());
            fgoStateOptPub_ = this->create_publisher<irt_nav_msgs::msg::FGOState>("stateOptimized",
                                                                                  rclcpp::SystemDefaultsQoS());
            fgoStateExtrapolatedPub_ = this->create_publisher<irt_nav_msgs::msg::FGOState>("stateExtrapolated",
                                                                                           rclcpp::SystemDefaultsQoS());
            fgoStatePredNavFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("statePredictedNavFix",
                                                                                         rclcpp::SystemDefaultsQoS());
            fgoStateOptNavFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("stateOptmizedNavFix",
                                                                                        rclcpp::SystemDefaultsQoS());
        }

        pvtInterpolatedPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("PVTInterpolated",
                                                                                  rclcpp::SystemDefaultsQoS());
        pvtTestPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("PVT",
                                                                          rclcpp::SystemDefaultsQoS());
        pvtErrorFromPredPub_ = this->create_publisher<irt_nav_msgs::msg::Error2GT>("errorPred",
                                                                                   rclcpp::SystemDefaultsQoS());

        utils::RosParameter<bool> pub_pvt_error("GNSSFGO.Publish.pubPVTError", true, *this);
        if (pub_pvt_error.value())
        {
            pvtErrorPub_ = this->create_publisher<irt_nav_msgs::msg::Error2GT>(
                    "GNSSFGO/error",
                    rclcpp::SystemDefaultsQoS());
        }

        utils::RosParameter<bool> pub_timer("GNSSFGO.Publish.pubTimer", true, *this);
        if (pub_timer.value()) {
            timerPub_ = this->create_publisher<irt_nav_msgs::msg::ElapsedTimeFGO>(
                    "GNSSFGO/elapsedTime",
                    rclcpp::SystemDefaultsQoS());
        }

        callbackGroupMap_.insert(std::make_pair("IMU", this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));

        utils::RosParameter<std::string> imuTopic("GNSSFGO.imuTopic", "/imu/data", *this);
        auto subIMUOpt = rclcpp::SubscriptionOptions();
        subIMUOpt.callback_group = callbackGroupMap_["IMU"];
        subIMU_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic.value(),
                                                                   rclcpp::SensorDataQoS(),
                                                                   [this](const sensor_msgs::msg::Imu::ConstSharedPtr& imuMeasurement)->void
                                                                   {
                                                                       this->onIMUMsgCb(imuMeasurement);
                                                                   },
                                                                   subIMUOpt);

        graph_ = std::make_shared<fgo::graph::GraphTimeCentric>(*this);

        optThread_ = std::make_unique<std::thread>([this]() -> void {
            // If we chose the IMU as the time reference, graph will be extended by counting high-frequent imu measurements
            // otherwise, we use ros time
            if(paramsPtr_->useIMUAsTimeReference)
                this->onTriggerOptimization();
            else
                this->timeCentricFGO();
        });

        initFGOThread_ = std::make_unique<std::thread>([this]() -> void {
            this->initializeFGO();
        });

        gyroBiasInitializer_ = std::make_unique<InitGyroBias>();
        gyroBiasInitializer_->initialize();

        //resize buffer
        userEstimationBuffer_.resize_buffer(10);
        initGyroBiasBuffer_.resize_buffer(2);
        imuDataBuffer_.resize_buffer(1000 * paramsPtr_->bufferSize);
        fgoPredStateBuffer_.resize_buffer(50);
        fgoOptStateBuffer_.resize_buffer(50);
        pvaSolutionBuffer_.resize_buffer(1000);
        return true;
    }

    bool GNSSFGOLocalizationBase::initializeFGO()
    { //is called in PVT callback
        RCLCPP_INFO(this->get_logger(), "onInitializeFGO started on a different Thread... ");
        while (rclcpp::ok())
        {
            std::unique_lock<std::mutex> lg(allBufferMutex_);
            conDoInit_.wait(lg, [&] {return triggeredInit_;});
            lastInitFinished_ = false;
            fgo::data_types::PVASolution foundPVA{};
            std::pair<rclcpp::Time, double> foundGNSSTime;

            lastInitFinished_ = false;
            isStateInited_ = false;

            const auto refSensorTimestamps = graph_->getReferenceSensorMeasurementTime();
            auto isPVAFound = false;

            if(refSensorTimestamps.empty() || !pvaSolutionBuffer_.size()) {
                lastInitFinished_ = true;
                triggeredInit_ = false;
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Starting initializeFGO ...");
            const auto pvaBuffer = pvaSolutionBuffer_.get_all_buffer();
            auto refSensorTimestampIter = refSensorTimestamps.rbegin();
            while (refSensorTimestampIter != refSensorTimestamps.rend())
            {
                auto pvaIter = pvaBuffer.rbegin();
                while(pvaIter != pvaBuffer.rend())
                {
                    if(pvaIter->tow == refSensorTimestampIter->second)
                    {
                        isPVAFound = true;
                        RCLCPP_INFO_STREAM(this->get_logger(), "FGO Init.: found GNSS tow at: " << std:: fixed
                                                                                                << pvaIter->tow << " GNSS timestamp "
                                                                                                << refSensorTimestampIter->first.seconds()
                                                                                                <<" PVT timestamp "
                                                                                                << pvaIter->timestamp.seconds());
                        foundPVA = *pvaIter;
                        foundGNSSTime = *refSensorTimestampIter;
                        break;
                    }
                    pvaIter ++;
                }
                if(isPVAFound)
                    break;
                refSensorTimestampIter ++;
            }
            if(!isPVAFound)
            {
                RCLCPP_WARN(get_logger(), "FGO couldn't be initialized! Waiting ...");
                lastInitFinished_ = true;
                triggeredInit_ = false;
                continue;
            }

            if (isStateInited_) {
                RCLCPP_WARN(get_logger(), "is already initialized.");
            }

            RCLCPP_ERROR_STREAM(this->get_logger(), "ON INIT: current pvt tow: " << std::fixed << foundPVA.tow << " GNSS tow: " << foundGNSSTime.second);
            RCLCPP_ERROR_STREAM(this->get_logger(), "ON INIT: current pvt ts: " << std::fixed << foundPVA.timestamp.seconds() << " GNSS ts: " << foundGNSSTime.first.seconds());

            //RCLCPP_INFO_STREAM(this->get_logger(), "Init Pos in LLH: " << std::fixed << initPVTInput.PvtGeodeticBus.phi << " : " << initPVTInput.PvtGeodeticBus.lambda << " : " << initPVTInput.PvtGeodeticBus.h);
            //RCLCPP_INFO_STREAM(this->get_logger(), "CB: " << std::fixed << initPVTInput.PvtGeodeticBus.RxClkBias << " CD: " << initPVTInput.PvtGeodeticBus.RxClkDrift);
            const rclcpp::Time& initTime = foundGNSSTime.first;
            const auto this_imu = imuDataBuffer_.get_buffer(initTime);

            lastOptimizedState_.timestamp = initTime;

            if(!paramsPtr_->initGyroBiasAsZero)
            {
                auto gyroBias = initGyroBiasBuffer_.get_last_buffer();
                lastOptimizedState_.imuBias = gtsam::imuBias::ConstantBias((gtsam::Vector3() << 0., 0., 0.).finished(),
                                                                           (gtsam::Vector3() << gyroBias[0], gyroBias[1], gyroBias[2]).finished());
            }
            lastOptimizedState_.cbd = (gtsam::Vector2() << foundPVA.clock_bias, foundPVA.clock_drift).finished();
            std::cout << std::fixed << "init_cbd: " << lastOptimizedState_.cbd << std::endl;

            //initialization

            const auto init_imu_pos = foundPVA.xyz_ecef - foundPVA.rot_ecef.rotate(paramsPtr_->transIMUToReference);
            const auto vecGrav = /*init_nedRe * */fgo::utils::gravity_ecef(init_imu_pos);
            const auto gravity_b = foundPVA.rot_ecef.unrotate(vecGrav);
            lastOptimizedState_.omega = this_imu.gyro;
            lastOptimizedState_.accMeasured = (gtsam::Vector6() << this_imu.accRot, this_imu.accLin + gravity_b).finished();

            const auto init_imu_vel = foundPVA.vel_ecef + foundPVA.rot_ecef.rotate(gtsam::skewSymmetric(paramsPtr_->transIMUToReference) * this_imu.gyro);
            lastOptimizedState_.state = gtsam::NavState(foundPVA.rot_ecef,
                                                        init_imu_pos,
                                                        init_imu_vel);

            std::cout << std::fixed << "eRb: " << lastOptimizedState_.state.R() << std::endl;
            std::cout << std::fixed << "pos: " << lastOptimizedState_.state.t() << std::endl;
            std::cout << std::fixed << "vel: " << lastOptimizedState_.state.v() << std::endl;

            //setup imu

            preIntegratorParams_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(vecGrav);
            std::cout << std::fixed << "vecGrav: " << vecGrav << std::endl;
            std::cout << std::fixed << "vecGravBody: " << gravity_b << std::endl;
            //imu_params->setBodyPSensor() possible
            preIntegratorParams_->accelerometerCovariance = pow(paramsPtr_->accelerometerSigma, 2) * gtsam::I_3x3; //Covariance of Sensor
            preIntegratorParams_->integrationCovariance = pow(paramsPtr_->integrationSigma, 2) * gtsam::I_3x3;
            preIntegratorParams_->gyroscopeCovariance = pow(paramsPtr_->gyroscopeSigma, 2) * gtsam::I_3x3;
            preIntegratorParams_->biasAccCovariance = pow(paramsPtr_->biasAccSigma, 2) * gtsam::I_3x3; //Covariance of Bias
            preIntegratorParams_->biasOmegaCovariance = pow(paramsPtr_->biasOmegaSigma, 2) * gtsam::I_3x3;
            preIntegratorParams_->biasAccOmegaInt = paramsPtr_->biasAccOmegaInt * gtsam::I_6x6;
            preIntegratorParams_->omegaCoriolis = gtsam::Vector3(0, 0, fgo::constants::earthRot); //Coriolis
            preIntegratorParams_->setUse2ndOrderCoriolis(true);
            currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                                                  lastOptimizedState_.imuBias);
            //the cov matrix is in NED coords, so we need to transform
            lastOptimizedState_.poseVar = (gtsam::Vector6() <<
                                                            foundPVA.rot_var, foundPVA.xyz_var).finished().asDiagonal();

            graph_->initGraph(lastOptimizedState_, initTime.seconds(), preIntegratorParams_);
            lastInitROSTimestamp_ = lastOptimizedState_.timestamp;

            currentPredState_ = lastOptimizedState_;
            fgoPredStateBuffer_.update_buffer(currentPredState_, initTime, this->get_clock()->now());
            fgoOptStateBuffer_.update_buffer(currentPredState_, initTime, this->get_clock()->now());
            fgoStateOptPub_->publish(this->convertFGOStateToMsg(lastOptimizedState_));
            fgoStateOptNavFixPub_->publish(this->convertPositionToNavFixMsg(lastOptimizedState_.state, lastOptimizedState_.timestamp, true));
            RCLCPP_INFO_STREAM(this->get_logger(), "(Re-)Initializing FGO current imu buffer size before cleaning: " << imuDataBuffer_.size());

            imuDataBuffer_.cleanBeforeTime(initTime.seconds()); // we try to make a small compromise so that the first state could be very nery to an imu measurement
            RCLCPP_INFO_STREAM(this->get_logger(), "(Re-)Initializing FGO successful at: " << std::fixed << initTime.seconds() << " current IMU buffer size: "<< imuDataBuffer_.size());
            lastInitFinished_ = true;
            isStateInited_ = true;
            triggeredInit_ = false;
            RCLCPP_INFO(this->get_logger(), "(Re-)Initializing FGO ...");
        }
    }

    void GNSSFGOLocalizationBase::onIMUMsgCb(const sensor_msgs::msg::Imu::ConstSharedPtr& imuMeasurement)
    {
        static rclcpp::Time lastIMUTime{0, 0, RCL_ROS_TIME};
        static gtsam::Vector3 lastGyro{};
        static uint notifyCounter = paramsPtr_->IMUMeasurementFrequency / paramsPtr_->optFrequency;

        auto start_time = std::chrono::system_clock::now();

        rclcpp::Time ts;
        if(paramsPtr_->useHeaderTimestamp)
            ts = rclcpp::Time(imuMeasurement->header.stamp.sec, imuMeasurement->header.stamp.nanosec, RCL_ROS_TIME);
        else
            ts = rclcpp::Time(this->now(), RCL_ROS_TIME);

        if(!paramsPtr_->initGyroBiasAsZero)
        {
            InitGyroBias::ExtU_InitGyroBias_T initGyroBiasInput{};
            initGyroBiasInput.InitAsZeros = false;
            initGyroBiasInput.InertialMeasurementBus.angularRate[0] = imuMeasurement->angular_velocity.x;
            initGyroBiasInput.InertialMeasurementBus.angularRate[1] = imuMeasurement->angular_velocity.y;
            initGyroBiasInput.InertialMeasurementBus.angularRate[2] = imuMeasurement->angular_velocity.z;
            gyroBiasInitializer_->setExternalInputs(&initGyroBiasInput);
            gyroBiasInitializer_->step();
            InitGyroBias::ExtY_InitGyroBias_T initGyroBiasOutput = gyroBiasInitializer_->getExternalOutputs();
            RCLCPP_INFO(this->get_logger(), "UpdateGyroBuffer..");
            initGyroBiasBuffer_.update_buffer({initGyroBiasOutput.gyroBiasArray[0],
                                               initGyroBiasOutput.gyroBiasArray[1],
                                               initGyroBiasOutput.gyroBiasArray[2]},
                                              ts,
                                              this->get_clock()->now());
            RCLCPP_INFO(this->get_logger(), "Update GyroBuffer finished!");
        }

        //change datatype
        fgo::data_types::IMUMeasurement fgoIMUMeasurement;
        fgoIMUMeasurement.timestamp = ts; //timestamp

        fgoIMUMeasurement.accLin = (gtsam::Vector3() << imuMeasurement->linear_acceleration.x, imuMeasurement->linear_acceleration.y, imuMeasurement->linear_acceleration.z).finished(); //acc
        fgoIMUMeasurement.accLin = paramsPtr_->imuRot.rotate(fgoIMUMeasurement.accLin);
        fgoIMUMeasurement.accLinCov = gtsam::Vector9(imuMeasurement->angular_velocity_covariance.data());
        fgoIMUMeasurement.gyro = (gtsam::Vector3() << imuMeasurement->angular_velocity.x, imuMeasurement->angular_velocity.y, imuMeasurement->angular_velocity.z).finished(); //angular vel
        fgoIMUMeasurement.gyro = paramsPtr_->imuRot.rotate(fgoIMUMeasurement.gyro);
        fgoIMUMeasurement.gyroCov = gtsam::Vector9(imuMeasurement->angular_velocity_covariance.data());
        //ORIENTATION ALWAYS 0,0,0,1
        fgoIMUMeasurement.AHRSOri = gtsam::Quaternion(imuMeasurement->orientation.w,
                                                      imuMeasurement->orientation.x,
                                                      imuMeasurement->orientation.y,
                                                      imuMeasurement->orientation.z);
        fgoIMUMeasurement.AHRSOriCov = gtsam::Vector9(imuMeasurement->orientation_covariance.data());

        if (!lastIMUTime.nanoseconds()) // we got first measurement
        {
            fgoIMUMeasurement.dt = 1. / paramsPtr_->IMUMeasurementFrequency;
            // calculate the angular acceleration
            fgoIMUMeasurement.accRot = gtsam::Vector3();
        } else {
            fgoIMUMeasurement.dt = fgoIMUMeasurement.timestamp.seconds() - lastIMUTime.seconds();
            fgoIMUMeasurement.accRot =  (fgoIMUMeasurement.gyro - lastGyro) / fgoIMUMeasurement.dt;
        }
        imuDataBuffer_.update_buffer(fgoIMUMeasurement, fgoIMUMeasurement.timestamp, this->get_clock()->now());

        if (!this->isStateInited_)
        {
            lastIMUTime = ts;
            lastGyro = (gtsam::Vector3() << imuMeasurement->angular_velocity.x,
                    imuMeasurement->angular_velocity.y,
                    imuMeasurement->angular_velocity.z).finished();

            if(lastInitFinished_)
            {
                triggeredInit_ = true;
                conDoInit_.notify_one();
            }
            return;
        }

        if(paramsPtr_->useIMUAsTimeReference)
        {
            if ( ( imuDataBuffer_.size() % notifyCounter) == 0)
            {
                if(lastOptFinished_)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "onIMU: Notify by IMU with imuBuffer: " << imuDataBuffer_.size() << " and notifycounter: " << notifyCounter);
                    this->notifyOptimization();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "onIMU: last optimization not finished! not triggering new optimization");
                }
            }
        }

        const auto gravity = fgo::utils::gravity_ecef(currentPredState_.state.position());
        const auto gravity_b = currentPredState_.state.attitude().unrotate(gravity);

        currentPredState_.mutex.lock();
        currentPredState_.timestamp = fgoIMUMeasurement.timestamp;
        currentPredState_.omega = currentPredState_.imuBias.correctGyroscope(fgoIMUMeasurement.gyro);
        currentPredState_.cbd = (gtsam::Matrix22() << 1, fgoIMUMeasurement.dt, 0, 1).finished() * currentPredState_.cbd;
        currentPredState_.accMeasured = (gtsam::Vector6() << fgoIMUMeasurement.accRot, currentPredState_.imuBias.correctAccelerometer(fgoIMUMeasurement.accLin + gravity_b)).finished();

        if(isDoingPropagation_) {
            currentIMUPreIntMutex_.lock();
            //preintegrate till time of newest IMU meas| dt is for the future, but we dont know when the next data will arrive
            currentIMUPreintegrator_->integrateMeasurement(fgoIMUMeasurement.accLin,
                                                           fgoIMUMeasurement.gyro,
                                                           fgoIMUMeasurement.dt);
            lastOptimizedState_.mutex.lock_shared();
            const auto new_state = currentIMUPreintegrator_->predict(lastOptimizedState_.state,
                                                                     lastOptimizedState_.imuBias);
            lastOptimizedState_.mutex.unlock_shared();
            currentIMUPreIntMutex_.unlock();
            // currentPredState_.timestamp += rclcpp::Duration::from_nanoseconds(fgoIMUMeasurement.dt * fgo::constants::sec2nanosec);
            currentPredState_.state = new_state;
        }

        currentPredState_.mutex.unlock();
        graph_->updatePredictedBuffer(currentPredState_);
        fgoPredStateBuffer_.update_buffer(currentPredState_, fgoIMUMeasurement.timestamp, this->get_clock()->now());
        const auto FGOStateMsg = this->convertFGOStateToMsg(currentPredState_);
        fgo::data_types::UserEstimation_T userEstimation = {FGOStateMsg.llh_ant_main[0], FGOStateMsg.llh_ant_main[1], FGOStateMsg.llh_ant_main[2], FGOStateMsg.cbd[0],
                                                            FGOStateMsg.llh_ant_aux[0], FGOStateMsg.llh_ant_aux[1], FGOStateMsg.llh_ant_aux[2]};
        fgoStatePredPub_->publish(FGOStateMsg);
        userEstimationBuffer_.update_buffer(userEstimation, fgoIMUMeasurement.timestamp);
        fgoStatePredNavFixPub_->publish(convertPositionToNavFixMsg(FGOStateMsg, true));

        if (this->isStateInited_ && !paramsPtr_->calcErrorOnOpt) {
            calculateErrorOnState(currentPredState_);
        }

        lastIMUTime = fgoIMUMeasurement.timestamp;
        lastGyro = fgoIMUMeasurement.gyro;

        double duration_cb = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start_time).count();
        if(duration_cb > 0.011)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "onIMUMsgCb: cb takes " << duration_cb << "s, more than expected 0.01s");
        }
    }

    void GNSSFGOLocalizationBase::timeCentricFGO()
    {
        RCLCPP_INFO(this->get_logger(), "Time centric graph optimization started in a different Thread... ");
        while(rclcpp::ok())
        {
            static const double betweenOptimizationTime = 1. / paramsPtr_->optFrequency;
            static const double betweenStateTime = 1. / paramsPtr_->stateFrequency;
            static auto lastNotInitializedTimestamp = std::chrono::system_clock::now();
            static double lastGraphTimestamp = 0;
            static auto firstRun = true;

            if(!this->isStateInited_) {
                lastNotInitializedTimestamp = std::chrono::system_clock::now();
                firstRun = true;
                continue;
            }

            //const auto currentTimestamp = std::chrono::system_clock::now();
            //if(firstRun && std::chrono::duration_cast<std::chrono::duration<double>>(currentTimestamp - lastNotInitializedTimestamp).count() <= betweenOptimizationTime)
            //{
            //    RCLCPP_WARN_STREAM(this->get_logger(), "onTimer: wait first run to be set " << std::fixed << std::chrono::duration_cast<std::chrono::duration<double>>(currentTimestamp - lastNotInitializedTimestamp).count());
            //    continue;
            //}

            if(firstRun)
            {
                lastGraphTimestamp = lastInitROSTimestamp_.seconds();
                firstRun = false;
            }

            //if(!this->lastOptFinished_)
            //{
            //    RCLCPP_WARN(this->get_logger(), "onTimer: last optimization not finished! not triggering new optimization");
            //    continue;
            //}

            // static uint notifyCounter = paramsPtr_->IMUMeasurementFrequency * betweenOptimizationTime;
            //const auto imuSize = imuDataBuffer_.size();
            //if(imuSize < notifyCounter)
            //{
            //    //RCLCPP_WARN_STREAM(this->get_logger(), "onTimer: no sufficient imu data received, current imu size " << imuSize);
            //    return;
            // }

            const auto currentROSTime = rclcpp::Time(this->now().nanoseconds(), RCL_ROS_TIME);
            double timeDiff = currentROSTime.seconds() - lastGraphTimestamp;

            if( timeDiff >= betweenOptimizationTime )
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "onTimer: notify new optimization after: " << std::fixed << timeDiff << " seconds since last notification." );
                //this->notifyOptimization();

                std::chrono::time_point<std::chrono::system_clock> start;
                start = std::chrono::system_clock::now();
                const auto start_fgo_construction = this->now();

                //std::cout << "State before opti " << std::fixed << currentState.timestamp.seconds() <<currentState.state << std::endl;

                std::vector<fgo::data_types::IMUMeasurement> imuData = imuDataBuffer_.get_all_buffer_and_clean();

                // we plan the state timestamps for the next graph extension
                std::vector<double> newStateTimestamps;

                while(timeDiff >= betweenStateTime)
                {
                    lastGraphTimestamp += betweenStateTime;
                    RCLCPP_WARN_STREAM(this->get_logger(), "Time-Centric Graph: create state at " << std::fixed << lastGraphTimestamp );
                    newStateTimestamps.emplace_back(lastGraphTimestamp);
                    timeDiff -= betweenStateTime;
                }

                RCLCPP_WARN_STREAM(this->get_logger(), "Time-Centric Graph: updated new state timestamps, remaining time diff: " << std::fixed << timeDiff );
                RCLCPP_WARN_STREAM(this->get_logger(), "Time-Centric Graph: current IMU size " << imuData.size());

                const auto constructGraphStatus = graph_->constructFactorGraphOnTime(newStateTimestamps, imuData);

                if (constructGraphStatus == fgo::graph::StatusGraphConstruction::FAILED){
                    throw std::invalid_argument( "FGC failed" );
                }

                double timeCFG = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
                //lg.unlock();
                //start = std::chrono::system_clock::now();
                if (constructGraphStatus == fgo::graph::StatusGraphConstruction::SUCCESSFUL){
                    RCLCPP_INFO(this->get_logger(), "Start Optimization...");
                    irt_nav_msgs::msg::ElapsedTimeFGO elapsedTimeFGO;
                    elapsedTimeFGO.ts_start_construction = start_fgo_construction.seconds();
                    elapsedTimeFGO.duration_construction = timeCFG;
                    elapsedTimeFGO.ts_start_optimization = this->now().seconds();
                    elapsedTimeFGO.num_new_factors = graph_->nrFactors();
                    double timeOpt = this->optimize();
                    isDoingPropagation_ = true;
                    //double timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
                    RCLCPP_INFO_STREAM(this->get_logger(), "Finished  optimization with a duration:" << timeOpt);
                    elapsedTimeFGO.header.stamp = this->now();
                    elapsedTimeFGO.duration_optimization = timeOpt;
                    timerPub_->publish(elapsedTimeFGO);
                }
                else if(constructGraphStatus == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION)
                {
                    isDoingPropagation_ = false;
                }
            }
        }
    }

    void GNSSFGOLocalizationBase::onTriggerOptimization()
    {
        // in this function, the optimization trigger will be managed using the conditions,
        // on default, this function is running in a endless loop, it will stop util condition is set
        RCLCPP_INFO(this->get_logger(), "onTriggerOptimization started on a different Thread... ");
        while (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Starting optimization thread, waiting for trigger ...");
            std::unique_lock<std::mutex> lg(allBufferMutex_);
            conDoOpt_.wait(lg, [&] { return triggeredOpt_; });
            // RCLCPP_INFO_STREAM(this->get_logger(),
            //                    "Opt triggered, start FGC, Time of GNSS: " << std::fixed << gnssDataBuffer_.get_last_buffer().measMainAnt.timestamp.seconds()
            //                    << " Time of Arrival:" << gnssDataBuffer_.get_last_time().seconds()
            //                    << " Time now: " << this->get_clock()->now().seconds());
            std::chrono::time_point<std::chrono::system_clock> start;
            start = std::chrono::system_clock::now();
            const auto start_fgo_construction = this->now();

            //std::cout << "State before opti " << std::fixed << currentState.timestamp.seconds() <<currentState.state << std::endl;

            std::vector<fgo::data_types::IMUMeasurement> imuData = imuDataBuffer_.get_all_buffer_and_clean();

            RCLCPP_WARN_STREAM(this->get_logger(), "Triggered Optimization with " << imuData.size() << " IMU data");

            const auto constructGraphStatus = graph_->constructFactorGraphOnIMU(imuData);

            if (constructGraphStatus == fgo::graph::StatusGraphConstruction::FAILED){
                throw std::invalid_argument( "FGC failed" );
            }

            //TODO continue is better
            //https://stackoverflow.com/questions/11062804/measuring-the-runtime-of-a-c-code
            double timeCFG = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
            //lg.unlock();
            //start = std::chrono::system_clock::now();
            if (constructGraphStatus == fgo::graph::StatusGraphConstruction::SUCCESSFUL){
                RCLCPP_INFO(this->get_logger(), "Start Optimization...");
                irt_nav_msgs::msg::ElapsedTimeFGO elapsedTimeFGO;
                elapsedTimeFGO.ts_start_construction = start_fgo_construction.seconds();
                elapsedTimeFGO.duration_construction = timeCFG;
                elapsedTimeFGO.ts_start_optimization = this->now().seconds();
                elapsedTimeFGO.num_new_factors = graph_->nrFactors();
                double timeOpt = this->optimize();
                isDoingPropagation_ = true;
                //double timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
                RCLCPP_INFO_STREAM(this->get_logger(), "Finished  optimization with a duration:" << timeOpt);
                elapsedTimeFGO.header.stamp = this->now();
                elapsedTimeFGO.duration_optimization = timeOpt;
                timerPub_->publish(elapsedTimeFGO);
            }
            else if(constructGraphStatus == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION)
            {
                isDoingPropagation_ = false;
            }
            triggeredOpt_ = false;
            lastOptFinished_ = true;
        }
    }

    double GNSSFGOLocalizationBase::optimize()
    {
        fgo::data_types::State newOptState;

        double optTime = graph_->optimize(newOptState);
        fgoOptStateBuffer_.update_buffer(newOptState, newOptState.timestamp);
        auto fgoStateMsg = this->convertFGOStateToMsg(newOptState);
        fgoStateOptPub_->publish(fgoStateMsg);
        fgoStateOptNavFixPub_->publish(convertPositionToNavFixMsg(fgoStateMsg, true));

        lastOptimizedState_.mutex.lock();
        lastOptimizedState_ = newOptState;
        lastOptimizedState_.mutex.unlock();

        std::vector<fgo::data_types::IMUMeasurement> dataIMU = imuDataBuffer_.get_all_buffer();
        auto restIMUData = graph_->getRestIMUData();
        currentIMUPreIntMutex_.lock();
        // Because the optimization took sometime, after the optimization, we again need to get the imu measurements, which are received while optimizing
        preIntegratorParams_->n_gravity = /*fgo::utils::nedRe_Matrix(lastOptimizedState_.state.position()) * */
                fgo::utils::gravity_ecef(newOptState.state.position());
        currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_, newOptState.imuBias);
        //first measurement only needs to be integrated from state to next meas
        double dt = 0.0;
        for (const auto &meas_imu: dataIMU) {
            currentIMUPreintegrator_->integrateMeasurement(meas_imu.accLin,
                                                           meas_imu.gyro,
                                                           meas_imu.dt);

            //std::cout << "Intergrated IMU: " << std::fixed << meas_imu.timestamp.seconds() << std::endl;
            dt += meas_imu.dt;
        }
        auto currentState = currentIMUPreintegrator_->predict(newOptState.state, newOptState.imuBias);
        currentIMUPreIntMutex_.unlock();
        //refresh state

        currentPredState_.mutex.lock();
        currentPredState_.timestamp = newOptState.timestamp + rclcpp::Duration::from_nanoseconds(dt * fgo::constants::sec2nanosec);
        //std::cout << "current state after opt: " << newOptState.timestamp.seconds() << newOptState.state << std::endl;
        //std::cout << "current state after opt updated: " << currentPredState_.timestamp.seconds() << currentState << std::endl;
        currentPredState_.state = currentState;
        if(paramsPtr_->useGPPriorFactor || paramsPtr_->useGPInterpolatedFactor)
        {
            if(!dataIMU.empty())
            {
                currentPredState_.omega = newOptState.imuBias.correctGyroscope(dataIMU.back().gyro);
            }
            else
                currentPredState_.omega = newOptState.omega;
            currentPredState_.omegaVar = newOptState.omegaVar;
        }
        currentPredState_.cbd = newOptState.cbd;
        currentPredState_.cbdVar = newOptState.cbdVar;
        currentPredState_.imuBias = newOptState.imuBias;
        currentPredState_.imuBiasVar = newOptState.imuBiasVar;
        currentPredState_.poseVar = newOptState.poseVar;
        currentPredState_.velVar = newOptState.velVar;
        currentPredState_.ddIntAmb = newOptState.ddIntAmb;
        currentPredState_.ddIntAmbVar = newOptState.ddIntAmbVar;
        currentPredState_.mutex.unlock();

        if (this->isStateInited_ && paramsPtr_->calcErrorOnOpt) {
            calculateErrorOnState(newOptState);
        }
        return optTime;
    }

    void GNSSFGOLocalizationBase::calculateErrorOnState(const fgo::data_types::State &stateIn)
    {
        static const auto l_b_ant_main = paramsPtr_->transIMUToReference;
        static std::vector<fgo::data_types::State> stateCached;

        //auto labelBuffer = gnssLabelingMsgBuffer_.get_all_buffer();

        auto pvaBuffer = pvaSolutionBuffer_.get_all_buffer();

        try
        {
            stateCached.emplace_back(stateIn);
            if(pvaBuffer.size() < 3) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: NO ENOUGH PVA Data, returning");
                return; }

            const auto lastPVTTime = pvaBuffer.back().timestamp.seconds();
            const auto firstPVTTime = pvaBuffer.front().timestamp.seconds();
            auto stateIter = stateCached.begin();
            while(stateIter != stateCached.end())
            {
                double stateTime = stateIter->timestamp.seconds();
                auto stateTimeNanoSec = stateIter->timestamp.nanoseconds();

                if(stateTime < firstPVTTime)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: state timestamp in front of first reference timestamp. Considering enlarge buffer!");
                    continue;
                }

                if(stateTime < lastPVTTime)
                {
                    // buffer is sorted

                    auto itAfter = std::lower_bound(pvaBuffer.begin(), pvaBuffer.end(), stateTime,
                                                    [](const fgo::data_types::PVASolution &pva, double timestamp) -> bool {
                                                        // true, ... true, true, false(HERE), false, ... false
                                                        return pva.timestamp.seconds() < timestamp;
                                                    });
                    auto itBefore = itAfter - 1;

                    //std::cout << "################################################" << std::endl;
                    //std::cout << itAfter - pvaBuffer.begin() << std::endl;

                    const auto coeff = (stateTime - itBefore->timestamp.seconds()) / (itAfter->timestamp - itBefore->timestamp).seconds();
                    const double tiffDiff = stateTime - itBefore->timestamp.seconds();

                    const auto ref_pos_llh_std = gtsam::interpolate(itBefore->xyz_var, itAfter->xyz_var, coeff).cwiseSqrt();
                    const auto pva_yaw = gtsam::interpolate(itBefore->heading, itAfter->heading, coeff);
                    const auto pva_yaw_std = gtsam::interpolate(sqrt(itBefore->heading_var), sqrt(itAfter->heading_var), coeff);
                    const auto pva_cog = gtsam::interpolate(itBefore->cog, itAfter->cog, coeff);
                    const auto pva_tow = gtsam::interpolate(itBefore->tow, itAfter->tow, coeff);
                    const auto pva_clock_bias = gtsam::interpolate(itBefore->clock_bias, itAfter->clock_bias, coeff);
                    const auto pva_clock_dirft = gtsam::interpolate(itBefore->clock_drift, itAfter->clock_drift, coeff);
                    const auto ref_vel_ned = gtsam::interpolate(itBefore->vel_n, itAfter->vel_n, coeff);
                    const auto ref_pos_llh = fgo::utils::WGS84InterpolationLLH(itBefore->llh,
                                                                               itAfter->llh,
                                                                               coeff);

                    sensor_msgs::msg::NavSatFix pvtMsg;
                    pvtMsg.header.stamp = stateIter->timestamp;
                    pvtMsg.altitude = ref_pos_llh.z() ;
                    pvtMsg.latitude = ref_pos_llh.x() * fgo::constants::rad2deg;
                    pvtMsg.longitude = ref_pos_llh.y() * fgo::constants::rad2deg;
                    pvtInterpolatedPub_->publish(pvtMsg);

                    irt_nav_msgs::msg::Error2GT error2Gt;

                    const gtsam::Vector2 ref_cbd(pva_clock_bias, pva_clock_dirft);

                    const auto& fgo_pos_ecef = stateIter->state.position();
                    const auto& fgo_ori_ecef = stateIter->state.attitude();
                    const auto& fgo_omega = stateIter->omega;
                    const auto& fgo_bias = stateIter->imuBias;
                    const auto& fgo_cbd = stateIter->cbd;
                    const auto& fgo_vel_ecef = stateIter->state.v();
                    const auto fgo_pose_ecef_std = stateIter->poseVar.diagonal().cwiseSqrt();
                    const auto fgo_vel_ecef_std = stateIter->velVar.diagonal().cwiseSqrt();
                    const auto fgo_omega_std = stateIter->omegaVar.diagonal().cwiseSqrt();
                    const auto fgo_cbd_std = stateIter->cbdVar.diagonal().cwiseSqrt();
                    const auto fgo_bias_std = stateIter->imuBiasVar.diagonal().cwiseSqrt();
                    const auto pos_ant_main_ecef = fgo_pos_ecef + fgo_ori_ecef.rotate(l_b_ant_main);
                    const auto pos_ant_llh = fgo::utils::xyz2llh(pos_ant_main_ecef);
                    const auto pos_ant_llh_inter = fgo::utils::xyz2llh_interative(pos_ant_main_ecef);
                    const gtsam::Rot3 nRe(fgo::utils::nedRe_Matrix(pos_ant_main_ecef));
                    const auto fgo_pose_std_ned = (gtsam::Vector6() << nRe.rotate(fgo_pose_ecef_std.block<3, 1>(0, 0)), nRe.rotate(fgo_pose_ecef_std.block<3, 1>(3, 0))).finished();
                    const auto fgo_vel_ned = nRe.rotate(fgo_vel_ecef + fgo_ori_ecef.rotate(gtsam::skewSymmetric((-l_b_ant_main)) * fgo_omega));
                    const auto fgo_vel_ned_std = nRe.rotate(fgo_vel_ecef_std);
                    const auto nRb_rpy = fgo::utils::func_DCM2EulerAngles((nRe.compose(fgo_ori_ecef)).matrix());

                    auto fgo_yaw = nRb_rpy(2) * 180. / M_PI;
                    fgo_yaw = (fgo_yaw >= 0. ? fgo_yaw : (fgo_yaw + 360.));
                    const auto fgo_pitch = nRb_rpy(1) * 180. / M_PI;
                    const auto fgo_roll = nRb_rpy(0) * 180. / M_PI;

                    const auto ref_pos_ecef = fgo::utils::llh2xyz(ref_pos_llh);
                    const gtsam::Rot3 nReGT(fgo::utils::nedRe_Matrix_asLLH(ref_pos_llh));

                    fgo::utils::eigenMatrix2stdVector(ref_pos_llh, error2Gt.ref_llh);
                    fgo::utils::eigenMatrix2stdVector(ref_pos_llh_std, error2Gt.ref_llh_std);
                    fgo::utils::eigenMatrix2stdVector(fgo_pose_ecef_std, error2Gt.pose_std_ecef);
                    fgo::utils::eigenMatrix2stdVector(fgo_pose_std_ned, error2Gt.pose_std_ned);
                    fgo::utils::eigenMatrix2stdVector(fgo_vel_ecef_std, error2Gt.vel_std_ecef);
                    fgo::utils::eigenMatrix2stdVector(fgo_vel_ned_std, error2Gt.vel_std_ned);
                    fgo::utils::eigenMatrix2stdVector(fgo_cbd, error2Gt.cbd);
                    fgo::utils::eigenMatrix2stdVector(fgo_cbd_std, error2Gt.cbd_std);
                    fgo::utils::eigenMatrix2stdVector(ref_cbd, error2Gt.ref_cbd);
                    fgo::utils::eigenMatrix2stdVector(fgo_vel_ned, error2Gt.vel_ned);
                    fgo::utils::eigenMatrix2stdVector(ref_vel_ned, error2Gt.ref_vel);
                    fgo::utils::eigenMatrix2stdVector(fgo_bias.accelerometer(), error2Gt.acc_bias);
                    fgo::utils::eigenMatrix2stdVector(fgo_bias.gyroscope(), error2Gt.gyro_bias);
                    fgo::utils::eigenMatrix2stdVector(fgo_bias_std.head(3), error2Gt.acc_bias_std);
                    fgo::utils::eigenMatrix2stdVector(fgo_bias_std.tail(3), error2Gt.gyro_bias_std);
                    fgo::utils::eigenMatrix2stdVector(fgo_omega, error2Gt.omega_body);
                    fgo::utils::eigenMatrix2stdVector(fgo_omega_std, error2Gt.omega_body_std);

                    if (itAfter->type == fgo::data_types::GNSSSolutionType::RTKFIX)
                    {
                        const auto pos_diff_ecef = pos_ant_main_ecef - ref_pos_ecef;
                        const auto pos_error_ned = nRe.rotate(pos_diff_ecef);//
                        const auto pos_error_body = fgo_ori_ecef.unrotate(pos_diff_ecef);
                        const auto vel_error_ned = fgo_vel_ned - ref_vel_ned;

                        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
                        double dist;
                        geod.Inverse(ref_pos_llh.x() * fgo::constants::rad2deg, ref_pos_llh.y() * fgo::constants::rad2deg,
                                     pos_ant_llh.x() * fgo::constants::rad2deg, pos_ant_llh.y() * fgo::constants::rad2deg, dist);

                        error2Gt.pos_2d_error_geographic = dist;
                        error2Gt.pos_3d_error_geographic = std::sqrt(dist * dist + std::pow((ref_pos_llh.z() - pos_ant_llh.z()), 2));
                        error2Gt.pos_1d_error_ned = abs(pos_error_ned(1));//fgo_ori.unrotate(pos_error_ecef)(1)
                        error2Gt.pos_2d_error_ned = (pos_error_ned).block<2, 1>(0,0).norm();
                        error2Gt.pos_3d_error_ned = pos_error_ned.norm();
                        error2Gt.pos_1d_error_body = abs(pos_error_body(1));
                        error2Gt.pos_2d_error_body = (pos_error_body).block<2, 1>(0,0).norm();
                        error2Gt.pos_3d_error_body = pos_error_body.norm();
                        error2Gt.pos_2d_error_ecef = (pos_diff_ecef).block<2, 1>(0,0).norm();
                        error2Gt.pos_3d_error_ecef = pos_diff_ecef.norm();
                        error2Gt.vel_2d_error = vel_error_ned.block<2, 1>(0,0).norm();
                        error2Gt.vel_3d_error = vel_error_ned.norm();

                        if((pva_yaw == 270. && pva_yaw == 0.) || pva_yaw_std > 30.)
                            error2Gt.yaw_error = std::numeric_limits<double>::max();
                        else
                        {
                            double yaw_error = fgo_yaw - pva_yaw;
                            while (yaw_error > 180.0 || yaw_error < -180.0){
                                if (yaw_error > 180.0)
                                    yaw_error -= 360.0;
                                if (yaw_error < -180.0)
                                    yaw_error += 360.0;
                            }
                            yaw_error = abs(yaw_error);
                            error2Gt.yaw_error = yaw_error;
                        }
                    }
                    error2Gt.tow = pva_tow;
                    error2Gt.ref_tow_before = itBefore->tow;
                    error2Gt.ref_tow_after = itAfter->tow;
                    error2Gt.ref_error = itAfter->error;
                    error2Gt.ref_mode = itAfter->type;
                    error2Gt.yaw = fgo_yaw;
                    error2Gt.ref_yaw = pva_yaw;
                    error2Gt.ref_yaw_std = pva_yaw_std;
                    error2Gt.pitch = fgo_pitch;
                    error2Gt.roll = fgo_roll;
                    error2Gt.ref_pitch_roll = itAfter->roll_pitch;
                    error2Gt.ref_pitch_roll_std = sqrt(itAfter->roll_pitch_var);
                    error2Gt.header.stamp = stateIter->timestamp;
                    pvtErrorPub_->publish(error2Gt);

                    stateIter = stateCached.erase(stateIter);
                    continue;
                }
                stateIter ++;
            }
        }
        catch(std::exception& ex)
        {

            std::cout << ex.what() << std::endl;
            RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: " << ex.what());
        }
    }

}
