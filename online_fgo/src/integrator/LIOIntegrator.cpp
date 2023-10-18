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

#include "integrator/LIOIntegrator.h"

namespace fgo::integrator
{
    void LIOIntegrator::initialize(rclcpp::Node& node, fgo::graph::GraphBase& graphPtr, const std::string& integratorName, bool isPrimarySensor)
    {
        integratorName_ = integratorName;
        isPrimarySensor_ = isPrimarySensor;
        rosNodePtr_ = &node;
        graphPtr_ = &graphPtr;
        this->initIntegratorBaseParams();

        integratorParamPtr_ = std::make_shared<IntegratorOdomParams>(integratorBaseParamPtr_);

        callbackGroupMap_.insert(std::make_pair("LIOSAM", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
        callbackGroupMap_.insert(std::make_pair("LIOSAMLoop", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
        callbackGroupMap_.insert(std::make_pair("LIOSAMVisualization", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));

        ::utils::RosParameter<bool> notIntegrating("GNSSFGO." + integratorName_ + ".notIntegrating", *rosNodePtr_);
        integratorParamPtr_->notIntegrating = notIntegrating.value();

        ::utils::RosParameter<bool> integrateBetweenPose("GNSSFGO." + integratorName_ + ".integrateBetweenPose", true, *rosNodePtr_);
        integratorParamPtr_->integrateBetweenPose = integrateBetweenPose.value();
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOIntegrator integrateBetweenPose" << integratorParamPtr_->integrateBetweenPose);
        ::utils::RosParameter<bool> integrateGlobalPose("GNSSFGO." + integratorName_ + ".integrateGlobalPose", false, *rosNodePtr_);
        integratorParamPtr_->integrateGlobalPose = integrateGlobalPose.value();
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOIntegrator integrateGlobalPose" << integratorParamPtr_->integrateGlobalPose);

        ::utils::RosParameter<std::vector<double>> odomPoseVar("GNSSFGO." + integratorName_ + ".odomPoseVar", *rosNodePtr_);
        integratorParamPtr_->odomPoseVar = gtsam::Vector6(odomPoseVar.value().data());

        ::utils::RosParameter<std::string> noiseModelOdomPose("GNSSFGO." + integratorName_ + ".noiseModelOdomPose", "gaussian", *rosNodePtr_);
        auto noiseModelOdomPoseStr = noiseModelOdomPose.value();
        setNoiseModelFromParam(noiseModelOdomPoseStr, integratorParamPtr_->noiseModelOdomPose, "LIO");
        integratorBaseParamPtr_->noiseModelOdomPose = integratorParamPtr_->noiseModelOdomPose;
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModelOdomPose: " << noiseModelOdomPoseStr);

        ::utils::RosParameter<double> robustParamOdomPose("GNSSFGO." + integratorName_ + ".robustParamOdomPose", 0.5, node);
        integratorParamPtr_->robustParamOdomPose = robustParamOdomPose.value();
        integratorBaseParamPtr_->robustParamOdomPose = robustParamOdomPose.value();

        pubSensorReport_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::SensorProcessingReport>("sensor_processing_report/" + integratorName_,
                                                                                                    rclcpp::SystemDefaultsQoS());

        LIOSAM_ = std::make_shared<sensors::LiDAR::LIOSAM::LIOSAMOdometry>(node, integratorParamPtr_, callbackGroupMap_["LIOSAM"],
                                                                           callbackGroupMap_["LIOSAMLoop"],
                                                                           callbackGroupMap_["LIOSAMVisualization"],
                                                                           integratorName_,
                                                                           pubSensorReport_);

        if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ){
            interpolatorI_ = std::make_shared<fgo::models::GPWNOJInterpolatorPose3>(
                    gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                    integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
            interpolatorJ_ = std::make_shared<fgo::models::GPWNOJInterpolatorPose3>(
                    gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                    integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        } else if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOA) {
            interpolatorI_ = std::make_shared<fgo::models::GPWNOAInterpolatorPose3>(
                    gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                    integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
            interpolatorJ_ = std::make_shared<fgo::models::GPWNOAInterpolatorPose3>(
                    gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
                    integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        } else {
            RCLCPP_WARN(rosNodePtr_->get_logger(), "LIOIntegrator::factorize NO gpType chosen. Please choose.");
        }

      RCLCPP_INFO(rosNodePtr_->get_logger(), "--------------------- LIOIntegrator initialized! ---------------------");

    };

    bool LIOIntegrator::factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                  const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                                  const solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                  std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                                  gtsam::Values& values,
                                  fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
                                  gtsam::KeyVector& relatedKeys)  {

      nState_ = currentKeyIndexTimestampMap.end()->first;

      //LIOSAM_->updateKeyIndexTimestampMap(currentKeyIndexTimestampMap);

      static std::vector<fgo::data_types::Odom> dataSensorLocalBuffer;
      static uint64_t counter = 0;
      auto dataSensor = LIOSAM_->getOdomAndClean();

      if (dataSensor.empty()) {
        RCLCPP_ERROR(rosNodePtr_->get_logger(), "LIOSAM: no odom available. ");
        return true;
      }

      uint32_t  numOdom_ = 0;
      for(const auto& odom : dataSensor)
      {
        bool hasBetweenPose = checkStatePresentedInCurrentLag(odom.queryOutputPrevious.keyIndexI);

        if(!hasBetweenPose)
        {
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "LIOSAM: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Not integrating this odom because previous state with id " <<  odom.queryOutputPrevious.keyIndexI << " is not presented in current lag!");
          std::cout << "Lastlag Keys: " << std::endl;
            for(const auto& k : lastLagKeys_)
            {
              std::cout << k << " : ";
            }
            std::cout << std::endl;

            LIOSAM_->retractKeyPoseCounter(1);
            LIOSAM_->notifyOptimization();
        }

        if (hasBetweenPose)
        {
            numOdom_ ++;
            fgo::data_types::OdomResult this_result;
            //gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Variances(odom.noise);
            const auto noise_model = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                            odom.noise,
                                                            integratorBaseParamPtr_->robustParamOdomPose,
                                                            "odomBetweenFactor");

            this_result.frameIndexCurrent = odom.frameIndexCurrent;
            this_result.frameIndexPrevious = odom.frameIndexPrevious;
            this_result.timestampPrevious = odom.timestampPrevious;
            this_result.timestampCurrent = odom.timestampCurrent;
            this_result.keyIndexII = odom.queryOutputPrevious.keyIndexI;
            this_result.keyIndexIJ = odom.queryOutputPrevious.keyIndexJ;
            this_result.keyIndexJI = odom.queryOutputCurrent.keyIndexI;
            this_result.keyIndexJJ = odom.queryOutputCurrent.keyIndexJ;
            this_result.timestampII = odom.queryOutputPrevious.timestampI;
            this_result.timestampIJ = odom.queryOutputPrevious.timestampJ;
            this_result.timestampJI = odom.queryOutputCurrent.timestampI;
            this_result.timestampJJ = odom.queryOutputCurrent.timestampJ;
            this_result.durationII = odom.queryOutputPrevious.durationI;
            this_result.durationJI = odom.queryOutputCurrent.durationI;
            this_result.keyISynchronized = odom.queryOutputPrevious.keySynchronized;
            this_result.keyJSynchronized = odom.queryOutputCurrent.keySynchronized;
            this_result.posePreviousIMUECEFQueried = odom.queryOutputPrevious.poseIMUECEF;
            this_result.poseCurrentIMUECEFQueried = odom.queryOutputCurrent.poseIMUECEF;

            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "LIOSAM: THIS ODOM IDs II IJ JI JJ" <<  this_result.keyIndexII << " : " <<  this_result.keyIndexIJ
                                                                                              << " : " <<  this_result.keyIndexJI << " : " <<  this_result.keyIndexJJ << "**************************************");
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "LIOSAM: THIS ODOM: " << std::fixed << odom.poseRelativeECEF);
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "LIOSAM: THIS ODOM FROMECEF: " << std::fixed << odom.poseFromECEF);
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "LIOSAM: THIS ODOM FROMECEF Queried: " << std::fixed << odom.poseFromECEF);
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "LIOSAM: THIS ODOM TOECEF Queried: " << std::fixed << odom.poseToECEF);
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "LIOSAM: THIS ODOM Queried : " << std::fixed << odom.queryOutputPrevious.poseIMUECEF.between(odom.queryOutputCurrent.poseIMUECEF));

            if(!odom.queryOutputPrevious.keySynchronized)
            {
                const double delta_t = odom.queryOutputPrevious.timestampJ - odom.queryOutputPrevious.timestampI;
                //RCLCPP_INFO_STREAM(rclcpp::get_logger(rosLoggerName_), "LIOSAM: statusQueryPreviousState GP  delta: " << delta_t << " tau: " << odom.queryOutputPrevious.);
                //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE
                //std::cout<<"ACC statusQueryPreviousState" << accI << " \n " << accJ << std::endl;
                if(integratorParamPtr_->gpType == data_types::GPModelType::WNOJ) {
                    this_result.accII = odom.queryOutputPrevious.accI;
                    this_result.accIJ = odom.queryOutputPrevious.accJ;
                    interpolatorI_->recalculate(delta_t, odom.queryOutputPrevious.durationI, odom.queryOutputPrevious.accI,
                                                odom.queryOutputPrevious.accJ);
                }
                else
                    interpolatorI_->recalculate(delta_t, odom.queryOutputPrevious.durationI);
            }

            if(!odom.queryOutputCurrent.keySynchronized)
            {
                const double delta_t = odom.queryOutputCurrent.timestampJ - odom.queryOutputCurrent.timestampI;
                //std::cout<<"ACC statusQueryCurrentState" << accI << " \n " << accJ << std::endl;
                if(integratorParamPtr_->gpType == data_types::GPModelType::WNOJ) {
                    this_result.accJI = odom.queryOutputCurrent.accI;
                    this_result.accJJ = odom.queryOutputCurrent.accJ;
                    interpolatorJ_->recalculate(delta_t, odom.queryOutputCurrent.durationI, odom.queryOutputCurrent.accI,
                                                odom.queryOutputCurrent.accJ);
                }
                else
                    interpolatorJ_->recalculate(delta_t, odom.queryOutputCurrent.durationI);
            }

            // NOTICE: LiDAR ODOM is already transformed in IMU CS

            if(integratorParamPtr_->integrateBetweenPose)
            {

                // ToDo: better formulation?
                relatedKeys.emplace_back(X(odom.queryOutputPrevious.keyIndexI));
                relatedKeys.emplace_back(X(odom.queryOutputCurrent.keyIndexJ));

                if(!odom.queryOutputPrevious.keySynchronized && !odom.queryOutputCurrent.keySynchronized)
                {
                    // both are interporlated
                    if(!integratorParamPtr_->notIntegrating) {
                        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOSAM: Integrating DOUBLE BETWEEN Factor");
                        graphPtr_->emplace_shared<fgo::factor::GPInterpolatedDoublePose3BetweenFactor>(
                                X(odom.queryOutputPrevious.keyIndexI), V(odom.queryOutputPrevious.keyIndexI),
                                W(odom.queryOutputPrevious.keyIndexI),
                                X(odom.queryOutputPrevious.keyIndexJ), V(odom.queryOutputPrevious.keyIndexJ),
                                W(odom.queryOutputPrevious.keyIndexJ),
                                X(odom.queryOutputCurrent.keyIndexI), V(odom.queryOutputCurrent.keyIndexI),
                                W(odom.queryOutputCurrent.keyIndexI),
                                X(odom.queryOutputCurrent.keyIndexJ), V(odom.queryOutputCurrent.keyIndexJ),
                                W(odom.queryOutputCurrent.keyIndexJ),
                                odom.poseRelativeECEF, interpolatorI_, interpolatorJ_, noise_model);
                    }
                    odomResults_.emplace_back(this_result);
                }
                else if(!odom.queryOutputPrevious.keySynchronized && odom.queryOutputCurrent.keySynchronized)
                {
                    if(!integratorParamPtr_->notIntegrating) {
                        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOSAM: Integrating SINGLE BETWEEN Factor by querying the PREVIOUS state.");
                        graphPtr_->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
                                X(odom.queryOutputPrevious.keyIndexI), V(odom.queryOutputPrevious.keyIndexI),
                                W(odom.queryOutputPrevious.keyIndexI),
                                X(odom.queryOutputPrevious.keyIndexJ), V(odom.queryOutputPrevious.keyIndexJ),
                                W(odom.queryOutputPrevious.keyIndexJ),
                                X(odom.queryOutputCurrent.keyIndexI),
                                odom.poseRelativeECEF, false,
                                interpolatorI_, noise_model);
                    }
                    odomResults_.emplace_back(this_result);
                }
                else if(odom.queryOutputPrevious.keySynchronized && !odom.queryOutputCurrent.keySynchronized )
                {
                    if(!integratorParamPtr_->notIntegrating) {
                        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOSAM: Integrating SINGLE BETWEEN Factor by querying the CURRENT state.");
                        graphPtr_->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
                                X(odom.queryOutputCurrent.keyIndexI), V(odom.queryOutputCurrent.keyIndexI),
                                W(odom.queryOutputCurrent.keyIndexI),
                                X(odom.queryOutputCurrent.keyIndexJ), V(odom.queryOutputCurrent.keyIndexJ),
                                W(odom.queryOutputCurrent.keyIndexJ),
                                X(odom.queryOutputPrevious.keyIndexI),
                                odom.poseRelativeECEF, true,
                                interpolatorJ_, noise_model);
                    }
                    odomResults_.emplace_back(this_result);

                }
                else if(odom.queryOutputPrevious.keySynchronized && odom.queryOutputCurrent.keySynchronized)
                {
                    if(!integratorParamPtr_->notIntegrating) {
                        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LIOSAM: Integrating BETWEEN Factor.");
                        if(integratorParamPtr_->integrateBetweenPose) {
                          auto betweenFactor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(odom.queryOutputPrevious.keyIndexI),
                                                                                                      X(odom.queryOutputCurrent.keyIndexI),
                                                                                                      odom.poseRelativeECEF, noise_model);
                          betweenFactor->setTypeID(fgo::factor::FactorTypeIDs::BetweenPose);
                          betweenFactor->setName("LiDARBetweenFactor");
                            graphPtr_->push_back(betweenFactor);
                        }
                    }
                    odomResults_.emplace_back(this_result);
                }
                else
                {
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "LIOSAM: Cant integrate this odom!");
                }
            }
        }

        if(integratorParamPtr_->integrateGlobalPose)
        {

            bool noGlobalPose = checkStatePresentedInCurrentLag(odom.queryOutputCurrent.keyIndexI);

            if(!noGlobalPose)
            {
                RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "LIOSAM: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Not integrating this odom as GLOBAL POSE because previous state with id " <<  odom.queryOutputCurrent.keyIndexI << " is not presented in current lag!");
                continue;
            }

          //if(counter % 5 == 0)
          //{
            if(odom.queryOutputCurrent.keySynchronized)
            {
              //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "LIOSAM: INTEGRATING SYNC. GLOBALPOSE!!!!!!");
              this->addNavPoseFactor(X(odom.queryOutputCurrent.keyIndexI),
                                     odom.poseToECEF, integratorParamPtr_->odomPoseVar);
            } else
            {
              //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "LIOSAM: INTEGRATING GPINTERPOLATED GLOBALPOSE!!!!!!");
              this->addGPinteporatedNavPoseFactor(X(odom.queryOutputCurrent.keyIndexI), V(odom.queryOutputCurrent.keyIndexI),
                                                  W(odom.queryOutputCurrent.keyIndexI),
                                                  X(odom.queryOutputCurrent.keyIndexJ), V(odom.queryOutputCurrent.keyIndexJ),
                                                  W(odom.queryOutputCurrent.keyIndexJ),
                                                  odom.poseToECEF, integratorParamPtr_->odomPoseVar, interpolatorJ_);
            }
         // }

        }
        counter++;
      }
      return true;
    }


    std::map<uint64_t, double> LIOIntegrator::factorizeAsPrimarySensor() {

        std::map<uint64_t, double> keyIndexTimestampsMap;

        auto dataSensor = LIOSAM_->getOdomAndClean();


        if(!graphPtr_->isGraphInitialized())
        {
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "PrimarySensor on " << integratorName_ << " graph not initialized!");
            return {};
        }

        for(const auto& odom : dataSensor)
        {
            nState_ ++;
            const auto& current_timestamp = odom.timestampCurrent;
            keyIndexTimestampsMap.insert(std::make_pair(nState_, current_timestamp));

            if(integratorParamPtr_->integrateBetweenPose)
            {
                const auto noise_model = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                                 odom.noise,
                                                                 integratorBaseParamPtr_->robustParamOdomPose,
                                                                 "odomBetweenFactor");

                auto betweenFactor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(nState_ - 1),
                                                                                            X(nState_),
                                                                                            odom.poseRelativeECEF, noise_model);
                betweenFactor->setTypeID(fgo::factor::FactorTypeIDs::BetweenPose);
                betweenFactor->setName("LiDARBetweenFactor");
                graphPtr_->push_back(betweenFactor);

            }

            if(integratorParamPtr_->integrateGlobalPose)
            {
                this->addNavPoseFactor(X(nState_),
                                       odom.poseToECEF, integratorParamPtr_->odomPoseVar);
            }

        }
        return keyIndexTimestampsMap;
    }

    bool LIOIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &martinals,
                                    const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                    data_types::State &optState) {
      for(const auto& odom : odomResults_)
      {
        //continue;
        gtsam::Pose3 poseI, poseJ;
        if(odom.keyJSynchronized && odom.keyISynchronized)
        {
          //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Case 3: odom.keyISynchronized && odom.keyJSynchronized");
          if(result.exists(X(odom.keyIndexII)))
          {
            poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 1: Continue because no keyindexII in results, should be marginalized out!");
            poseI = odom.posePreviousIMUECEFQueried;
          }
          if(result.exists(X(odom.keyIndexJI)))
          {
            poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 1: Continue because no keyindexJI in results, should be marginalized out!");
            poseJ = odom.poseCurrentIMUECEFQueried;
          }

        }
        else if(odom.keyISynchronized && !odom.keyJSynchronized)
        {
         // RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Case 3: odom.keyISynchronized && !odom.keyJSynchronized");
          if(result.exists(X(odom.keyIndexII)))
          {
            poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
          }
          else
          {
           RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 2: Continue because no keyindexII in results, should be marginalized out!");
            poseI = odom.posePreviousIMUECEFQueried;
          }
          interpolatorJ_->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI);

          if(result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ)) )
          {
            poseJ = interpolatorJ_->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)), result.at<gtsam::Vector3>(V(odom.keyIndexJI)), result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                    result.at<gtsam::Pose3>(X(odom.keyIndexJJ)), result.at<gtsam::Vector3>(V(odom.keyIndexJJ)), result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));
          }
          else
          {
           RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 2: Continue because no keyIndexJI or keyIndexJI in results, should be marginalized out!");
            poseJ = odom.poseCurrentIMUECEFQueried;
          }
        }
        else if(!odom.keyISynchronized && odom.keyJSynchronized)
        {
          //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Case 3: !odom.keyISynchronized && odom.keyJSynchronized");
          if(result.exists(X(odom.keyIndexJI)))
          {
            poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 3: Continue because no keyindexJI in results, should be marginalized out!");
            poseJ = odom.poseCurrentIMUECEFQueried;
          }

          interpolatorI_->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII);

          if(result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ)))
          {
            poseI = interpolatorI_->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)), result.at<gtsam::Vector3>(V(odom.keyIndexII)), result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                    result.at<gtsam::Pose3>(X(odom.keyIndexIJ)), result.at<gtsam::Vector3>(V(odom.keyIndexIJ)), result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));

          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 3: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
            poseI = odom.posePreviousIMUECEFQueried;
          }

        }
        else if(!odom.keyISynchronized && !odom.keyJSynchronized)
        {
          //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Case 3: !odom.keyISynchronized && !odom.keyJSynchronized");
          interpolatorI_->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII);
          interpolatorJ_->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI);

          if(result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ)))
          {
            poseI = interpolatorI_->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)), result.at<gtsam::Vector3>(V(odom.keyIndexII)), result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                    result.at<gtsam::Pose3>(X(odom.keyIndexIJ)), result.at<gtsam::Vector3>(V(odom.keyIndexIJ)), result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));

          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 4: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
            poseI = odom.posePreviousIMUECEFQueried;
          }

          if(result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ)))
          {
            poseJ = interpolatorJ_->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)), result.at<gtsam::Vector3>(V(odom.keyIndexJI)), result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                    result.at<gtsam::Pose3>(X(odom.keyIndexJJ)), result.at<gtsam::Vector3>(V(odom.keyIndexJJ)), result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));

          }
          else
          {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Case 4: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
            poseJ = odom.poseCurrentIMUECEFQueried;
          }

        }
        else
        {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Can't update key cloud pose for odom with scan index " << odom.frameIndexPrevious << " and " << odom.frameIndexCurrent);
          continue;
        }

        //RCLCPP_ERROR(rclcpp::get_logger(rosLoggerName_), "----------------- LIOSAM RESULT -----------------");

        //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose I: \n" << std::fixed <<poseI);
        //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose J: \n" << std::fixed<< poseJ);

        //RCLCPP_WARN_STREAM(rclcpp::get_logger(rosLoggerName_), "Pose Between: \n" << std::fixed << poseI.between(poseJ));

        //RCLCPP_ERROR(rclcpp::get_logger(rosLoggerName_), "----------------- LIOSAM RESULT DONE -----------------");


        LIOSAM_->updateCloudKeyPose(odom.frameIndexPrevious, poseI, odom.frameIndexCurrent, poseJ);
      }
      odomResults_.clear();

      for(const auto& keyIndexTs : keyIndexTimestampMap)
      {
        fgo::data_types::QueryStateInput input;
        input.pose = result.at<gtsam::Pose3>(X(keyIndexTs.first));
        input.vel = result.at<gtsam::Vector3>(V(keyIndexTs.first));
        if(integratorParamPtr_->useGPPriorFactor || integratorParamPtr_->useGPInterpolatedFactor) {
          input.omega = result.at<gtsam::Vector3>(W(keyIndexTs.first));
          input.acc = optState.accMeasured;
        }
        LIOSAM_->updateOptPose(keyIndexTs.first, rclcpp::Time(keyIndexTs.second * fgo::constants::sec2nanosec, RCL_ROS_TIME), input);
      }
      LIOSAM_->updateKeyIndexTimestampMap(keyIndexTimestampMap);
      lastKeyIndexTimestampMap_ = keyIndexTimestampMap;
      lastLagKeys_.clear();
      for(const auto& m : keyIndexTimestampMap)
          lastLagKeys_.emplace_back(m.first);
      //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "UPDATING LIOSAM OPTIMIZED POSE DONE!");
      return true;
    }


}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::integrator::LIOIntegrator, fgo::integrator::IntegratorBase)