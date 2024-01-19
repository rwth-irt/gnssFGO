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


#include "integrator/GNSSLCIntegrator.h"

namespace fgo::integrator
{
    void GNSSLCIntegrator::initialize(rclcpp::Node &node, graph::GraphBase &graphPtr, const std::string& integratorName, bool isPrimarySensor)
    {
      integratorName_ = integratorName;
      isPrimarySensor_ = isPrimarySensor;
      rosNodePtr_ = &node;
      graphPtr_ = &graphPtr;
      this->initIntegratorBaseParams();

      paramPtr_ = std::make_shared<IntegratorGNSSLCParams>(integratorBaseParamPtr_);

      /*
       * Init Parameters
       */

      GNSSPVABuffer_.resize_buffer(100);
      referencePVTBuffer_.resize_buffer(100);
      RosParameter<int> GNSSMeasurementFrequency("GNSSFGO." + integratorName_ + ".GNSSMeasurementFrequency", node);
      paramPtr_->GNSSMeasurementFrequency = GNSSMeasurementFrequency.value();

      RosParameter<std::string> PVTSource("GNSSFGO." + integratorName_ + ".PVTSource", node);
      //paramPtr_->PVTSource = PVTSource.value();

      RosParameter<bool> hasHeading("GNSSFGO." + integratorName_ + ".hasHeading", node);
      paramPtr_->hasHeading = hasHeading.value();

      RosParameter<bool> hasPitch("GNSSFGO." + integratorName_ + ".hasPitch", node);
      paramPtr_->hasPitch = hasPitch.value();

      RosParameter<bool> hasRoll("GNSSFGO." + integratorName_ + ".hasRoll", node);
      paramPtr_->hasRoll = hasRoll.value();

      RosParameter<bool> integrateVelocity("GNSSFGO." + integratorName_ + ".integrateVelocity", node);
      paramPtr_->integrateVelocity = integrateVelocity.value();

      RosParameter<bool> integrateAttitude("GNSSFGO." + integratorName_ + ".integrateAttitude", node);
      paramPtr_->integrateAttitude = integrateAttitude.value();

      RosParameter<int>solutionSyncQueueSize("GNSSFGO." + integratorName_ + ".solutionSyncQueueSize", 10, node);

      RosParameter<int>msgLowerBound("GNSSFGO." + integratorName_ + ".MsgSyncLowerBound", 50000000, node);

      RosParameter<double> fixedVelVar("GNSSFGO." + integratorName_ + ".fixedVelVar", 1., node);
      paramPtr_->fixedVelVar = fixedVelVar.value();

      RosParameter<double> PVTPosVarScale("GNSSFGO." + integratorName_ + ".posVarScale", 1., node);
      paramPtr_->posVarScale = PVTPosVarScale.value();

      RosParameter<double> velVarScale("GNSSFGO." + integratorName_ + ".velVarScale", 1., node);
      paramPtr_->velVarScale = velVarScale.value();

      RosParameter<double> headingVarScale("GNSSFGO." + integratorName_ + ".headingVarScale", 1., node);
      paramPtr_->headingVarScale = headingVarScale.value();

      RosParameter<double> robustParamPos("GNSSFGO." + integratorName_ + ".roustParamPos", 1., node);
      paramPtr_->robustParamPosition = robustParamPos.value();

      RosParameter<double> robustParamVel("GNSSFGO." + integratorName_ + ".roustParamVel", 1., node);
      paramPtr_->robustParamVelocity = robustParamVel.value();
      integratorBaseParamPtr_->robustParamVelocity = robustParamVel.value();

      RosParameter<double> robustParamHeading("GNSSFGO." + integratorName_ + ".roustParamAtt", 1., node);
      paramPtr_->robustParamAttitude = robustParamHeading.value();
      integratorBaseParamPtr_->robustParamAttitude = robustParamHeading.value();

      RosParameter<double> headingOffsetDeg("GNSSFGO." + integratorName_ + ".headingOffsetDeg", 0., node);
      paramPtr_->heading_offset_deg = headingOffsetDeg.value();

      RosParameter<std::string> attitudeFrame("GNSSFGO." + integratorName_ + ".attitudeFrame", "ned", *rosNodePtr_);
      auto attitudeFrameStr = attitudeFrame.value();
      setSensorFrameFromParam(attitudeFrameStr, paramPtr_->attitudeFrame, "GNSSLC");
      integratorBaseParamPtr_->attitudeFrame = paramPtr_->attitudeFrame;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "attitudeFrame: " << attitudeFrameStr);

      RosParameter<std::string> attitudeType("GNSSFGO." + integratorName_ + ".attitudeType", "rpy", *rosNodePtr_);
      auto attitudeTypeStr = attitudeType.value();
      setAttitudeType(attitudeTypeStr, paramPtr_->attitudeType, "GNSSLC");
      integratorBaseParamPtr_->attitudeType = paramPtr_->attitudeType;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "attitudeType: " << attitudeTypeStr);

      RosParameter<std::string> velocityFrame("GNSSFGO." + integratorName_ + ".velocityFrame", "body", *rosNodePtr_);
      auto velocityFrameStr = velocityFrame.value();
      setSensorFrameFromParam(velocityFrameStr, paramPtr_->velocityFrame, "GNSSLC");
      integratorBaseParamPtr_->velocityFrame = paramPtr_->velocityFrame;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityFrame: " << velocityFrameStr);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityFrame: " << paramPtr_->velocityFrame);

      RosParameter<std::string> velocityType("GNSSFGO." + integratorName_ + ".velocityType", "3d", *rosNodePtr_);
      auto velocityTypeStr = velocityType.value();
      setVelocityType(velocityTypeStr, paramPtr_->velocityType, "GNSSLC");
      integratorBaseParamPtr_->velocityType = paramPtr_->velocityType;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "velocityType: " << velocityTypeStr);

      RosParameter<std::string> noiseModelAttitude("GNSSFGO." + integratorName_ + ".noiseModelAttitude", "gaussian", *rosNodePtr_);
      auto noiseModelAttitudeStr = noiseModelAttitude.value();
      setNoiseModelFromParam(noiseModelAttitudeStr, paramPtr_->noiseModelAttitude, "GNSSLC Attitude");
      integratorBaseParamPtr_->noiseModelAttitude = paramPtr_->noiseModelAttitude;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModelAttitude: " << noiseModelAttitudeStr);

      RosParameter<std::string> noiseModelVelocity("GNSSFGO." + integratorName_ + ".noiseModelVelocity", "gaussian", *rosNodePtr_);
      auto noiseModelVelocityStr = noiseModelVelocity.value();
      setNoiseModelFromParam(noiseModelVelocityStr, paramPtr_->noiseModelVelocity, "GNSSLC Velocity");
      integratorBaseParamPtr_->noiseModelVelocity = paramPtr_->noiseModelVelocity;
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModelVelocity: " << noiseModelVelocityStr);

      RosParameter<std::string> noiseModelPosition("GNSSFGO." + integratorName_ + ".noiseModelPosition", "gaussian", *rosNodePtr_);
      auto noiseModelPositionStr = noiseModelPosition.value();
      setNoiseModelFromParam(noiseModelPositionStr, paramPtr_->noiseModelPosition, "GNSSLC Position");
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModelPosition: " << noiseModelPositionStr);

      RosParameter<bool> onlyRTKFixed("GNSSFGO." + integratorName_ + ".onlyRTKFixed", node);
      paramPtr_->onlyRTKFixed = onlyRTKFixed.value();

      ::utils::RosParameter<bool> notIntegrating("GNSSFGO." + integratorName_ + ".notIntegrating", false, *rosNodePtr_);
      paramPtr_->notIntegrating = notIntegrating.value();

      // reloading the leverarm
      std::vector<double> lbOri = {paramPtr_->transIMUToAnt1.x(), paramPtr_->transIMUToAnt1.y(), paramPtr_->transIMUToAnt1.z()};
      RosParameter<std::vector<double>> lb("GNSSFGO." + integratorName_ + ".transIMUToGNSSAnt1", lbOri, *rosNodePtr_);
      paramPtr_->transIMUToAnt1 = gtsam::Vector3(lb.value().data());

      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LeverARMToAnt1: " << paramPtr_->transIMUToAnt1);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "LeverARMToAnt2: " << paramPtr_->transIMUToAnt2);
      RosParameter<bool> useForInitialization("GNSSFGO." + integratorName_ + ".useForInitialization", *rosNodePtr_);
      paramPtr_->useForInitialization = useForInitialization.value();
      RosParameter<bool> useHeaderTimestamp("GNSSFGO." + integratorName_ + ".useHeaderTimestamp", *rosNodePtr_);
      paramPtr_->useHeaderTimestamp = useHeaderTimestamp.value();

      RosParameter<double> varScaleRTKFloat("GNSSFGO." + integratorName_ + ".varScaleRTKFloat", 1., node);
      paramPtr_->varScaleRTKFloat = varScaleRTKFloat.value();
      RosParameter<double> varScaleSingle("GNSSFGO." + integratorName_ + ".varScaleSingle", 2., node);
      paramPtr_->varScaleSingle = varScaleSingle.value();
      RosParameter<double> varScaleNoSolution("GNSSFGO." + integratorName_ + ".varScaleNoSolution", 1000., node);
      paramPtr_->varScaleNoSolution = varScaleNoSolution.value();

      RosParameter<double> zeroVelocityThreshold("GNSSFGO." + integratorName_ + ".zeroVelocityThreshold", node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "zeroVelocityThreshold: " << zeroVelocityThreshold.value());
      paramPtr_->zeroVelocityThreshold = zeroVelocityThreshold.value();

      RosParameter<double> varScaleHeadingRTKFloat("GNSSFGO." + integratorName_ + ".varScaleHeadingRTKFloat", node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "varScaleHeadingRTKFloat: " << varScaleHeadingRTKFloat.value());
      paramPtr_->varScaleHeadingRTKFloat = varScaleHeadingRTKFloat.value();

      RosParameter<double> varScaleHeadingSingle("GNSSFGO." + integratorName_ + ".varScaleHeadingSingle", node);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "varScaleHeadingSingle: " << varScaleHeadingSingle.value());
      paramPtr_->varScaleHeadingSingle = varScaleHeadingSingle.value();

        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "bound: " << paramPtr_->StateMeasSyncLowerBound);


      if(PVTSource.value() == "oem7")
      {
        RosParameter<std::string> bestposTopic("GNSSFGO." + integratorName_ + ".NovatelBestposTopic", "/novatel/oem7/bestpos", node);
        RosParameter<std::string> bestvelTopic("GNSSFGO." + integratorName_ + ".NovatelBestvelTopic", "/novatel/oem7/bestvel", node);
        if(paramPtr_->integrateVelocity)
        {
          subNovatelBestpos_.subscribe(&node, bestposTopic.value());
          subNovatelBestvel_.subscribe(&node, bestvelTopic.value());
          if(hasHeading.value())
          {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +":: using oem7 with velocity and heading ...");
            RosParameter<std::string> headingTopic("GNSSFGO." + integratorName_ + ".NovatelHeadingTopic", "/novatel/oem7/dualantennaheading", node);
            subNovatelHeading_.subscribe(&node, headingTopic.value());

            novatelPVTDualAntennaSync_ = std::make_unique<message_filters::Synchronizer<OEM7DualAntennaSyncPolicy>>(OEM7DualAntennaSyncPolicy(solutionSyncQueueSize.value()),
                                                                                                                    subNovatelBestpos_,
                                                                                                                    subNovatelBestvel_,
                                                                                                                    subNovatelHeading_);
            novatelPVTDualAntennaSync_->setAgePenalty(0);
            novatelPVTDualAntennaSync_->setInterMessageLowerBound(0, rclcpp::Duration(0, msgLowerBound.value()));
            novatelPVTDualAntennaSync_->setInterMessageLowerBound(1, rclcpp::Duration(0, msgLowerBound.value()));
            novatelPVTDualAntennaSync_->setInterMessageLowerBound(2, rclcpp::Duration(0, msgLowerBound.value()));
            novatelPVTDualAntennaSync_->registerCallback(std::bind(&GNSSLCIntegrator::onOEM7PVTHeadingMsgCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
          } else
          {
            RCLCPP_INFO(rosNodePtr_->get_logger(), "[GNSSLCIntegrator]: using oem7 with velocity without heading ...");
            novatelPVTSync_ = std::make_unique<message_filters::Synchronizer<OEM7SyncPolicy>>(OEM7SyncPolicy(solutionSyncQueueSize.value()),
                                                                                              subNovatelBestpos_,
                                                                                              subNovatelBestvel_);
            novatelPVTSync_->setAgePenalty(0);
            novatelPVTSync_->setInterMessageLowerBound(0, rclcpp::Duration(0, msgLowerBound.value()));
            novatelPVTSync_->setInterMessageLowerBound(1, rclcpp::Duration(0, msgLowerBound.value()));
            novatelPVTSync_->registerCallback(std::bind(&GNSSLCIntegrator::onOEM7PVTMsgCb, this, std::placeholders::_1, std::placeholders::_2));
          }
        }
        else
        {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using oem7 without velocity and heading ...");
          subNovatelBestposAlone_ = rosNodePtr_->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(bestposTopic.value(),
                                                                                              rclcpp::SystemDefaultsQoS(),
                                                                                              std::bind(&GNSSLCIntegrator::onOEM7Bestpos, this, std::placeholders::_1));

        }

      }
      else if(PVTSource.value() == "ublox")
      {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using ublox with velocity and heading ...");
        RosParameter<std::string> ubloxNavPVTTopic("GNSSFGO." + integratorName_ + ".ubloxPVTTopic", "/ublox/navpvt", node);
        subUbloxPVT_ = rosNodePtr_->create_subscription<ublox_msgs::msg::NavPVT>(ubloxNavPVTTopic.value(),
                                                                                 rclcpp::SystemDefaultsQoS(),
                                                                                 std::bind(&GNSSLCIntegrator::onUbloxPVTMsgCb, this, std::placeholders::_1));

      }
      else if(PVTSource.value() == "navfix")
      {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using navfix without velocity and heading ...");
        RosParameter<std::string> navFixTopic("GNSSFGO." + integratorName_ + ".navfixTopic", "/ublox/fix", node);

        subNavfix_ = rosNodePtr_->create_subscription<sensor_msgs::msg::NavSatFix>(navFixTopic.value(),
                                                                                   rclcpp::SystemDefaultsQoS(),
                                                                                   std::bind(&GNSSLCIntegrator::onNavFixMsgCb, this, std::placeholders::_1));

      }
      else if(PVTSource.value() == "span")
      {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using span with velocity and heading ...");
        subNovatelPVA_ = rosNodePtr_->create_subscription<novatel_oem7_msgs::msg::INSPVAX>("/novatel_data/inspvax",
                                                                                           rclcpp::SensorDataQoS(),
                                                                                           std::bind(&GNSSLCIntegrator::onINSPVAXMsgCb, this,
                                                                                                     std::placeholders::_1));
      }
      else if(PVTSource.value() == "irt")
      {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using irt pva with velocity and heading ...");
        callbackGroupMap_.insert(std::make_pair("PVTDelayCalculator", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
        callbackGroupMap_.insert(std::make_pair("PPS", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
        callbackGroupMap_.insert(std::make_pair("PVA", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
        PVTDelayCalculator_ = std::make_unique<fgo::utils::MeasurementDelayCalculator>(*rosNodePtr_,
                                                                                       callbackGroupMap_["PVTDelayCalculator"],
                                                                                       false);

        auto subPPSopt = rclcpp::SubscriptionOptions();
        subPPSopt.callback_group = callbackGroupMap_["PPS"];
        subPPS_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::PPS>("/irt_gpio_novatel/jetson_pps",
                                                                       rclcpp::SystemDefaultsQoS(),
                                                                       [this](const irt_nav_msgs::msg::PPS::ConstSharedPtr msg) -> void
                                                                       {
                                                                           PVTDelayCalculator_->setPPS(msg);
                                                                           //RCLCPP_INFO_STREAM(this->get_logger(), "onPPS: " << msg->pps_counter);
                                                                       },
                                                                       subPPSopt);

        auto subPVAOpt = rclcpp::SubscriptionOptions();
        subPVAOpt.callback_group = callbackGroupMap_["PVA"];
        subPVA_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::PVAGeodetic>("/irt_gnss_preprocessing/PVT",
                                                                               rclcpp::SensorDataQoS(),
                                                                               std::bind(&GNSSLCIntegrator::onIRTPVTMsgCb,
                                                                                         this, std::placeholders::_1),
                                                                               subPVAOpt);

        pubPVAInFGOStata_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::FGOState>("pvaInFGOState",
                                                                                   rclcpp::SensorDataQoS());
      }
      else if (PVTSource.value() == "boreas")
      {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ +": using boreas pva ...");
        RosParameter<std::string> boreasTopic("GNSSFGO." + integratorName_ + ".odomPVATopic", "/boreas/gps_raw", node);
        auto subPVAOpt = rclcpp::SubscriptionOptions();
        subPVAOpt.callback_group = callbackGroupMap_["PVA"];
        subPVAOdom_ = rosNodePtr_->create_subscription<nav_msgs::msg::Odometry>(boreasTopic.value(),
                                                                                rclcpp::SystemDefaultsQoS(),
                                                                                std::bind(&GNSSLCIntegrator::onOdomMsgCb, this, std::placeholders::_1),
                                                                                subPVAOpt);
      }
      else
      {
        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ +": PVTSource " << PVTSource.value() << " is not supported!");
      }

      if (paramPtr_->gpType == fgo::data_types::GPModelType::WNOJ)
      {
          interpolator_ = std::make_shared<fgo::models::GPWNOJInterpolatorPose3>(
                  gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
                  paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (paramPtr_->gpType == fgo::data_types::GPModelType::WNOA)
      {
          interpolator_ = std::make_shared<fgo::models::GPWNOAInterpolatorPose3>(
                  gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
                  paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
      } else
      {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": NO gpType chosen. Please choose.");
      }

        pubSensorReport_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::SensorProcessingReport>("sensor_processing_report/" + integratorName_,
                                                                                                    rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(rosNodePtr_->get_logger(), "--------------------- GNSSLCIntegrator initialized! ---------------------");
    }

    bool GNSSLCIntegrator::factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                     const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                                     const solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                     std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                                     gtsam::Values& values,
                                     fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
                                     gtsam::KeyVector& relatedKeys)
    {

      if(paramPtr_->notIntegrating)
      {
        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << integratorName_ +" not integrating ...");
        return true;
      }

      static gtsam::Key pose_key_j, vel_key_j, omega_key_j, bias_key_j,
                        pose_key_i, vel_key_i, omega_key_i, bias_key_i,
                        pose_key_sync, vel_key_sync, bias_key_sync, omega_key_sync;

      static boost::circular_buffer<fgo::data_types::PVASolution> restGNSSMeas(100);

      auto dataSensor = GNSSPVABuffer_.get_all_buffer_and_clean();

      if (!restGNSSMeas.empty())
      {
        dataSensor.insert(dataSensor.begin(), restGNSSMeas.begin(), restGNSSMeas.end());
        restGNSSMeas.clear();
      }

      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << integratorName_ +": integrating with " << dataSensor.size() << " pvt data");

      auto pvaIter = dataSensor.begin();
      while(pvaIter != dataSensor.end())
      {
        const auto corrected_time = pvaIter->timestamp.seconds() - pvaIter->delay;
        auto posVarScale = paramPtr_->posVarScale;
        auto velVarScale = paramPtr_->velVarScale;
        auto rotVarScale = paramPtr_->headingVarScale;

        if(paramPtr_->onlyRTKFixed && pvaIter->type != fgo::data_types::GNSSSolutionType::RTKFIX)
        {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": PVA at " << std::fixed << corrected_time <<
                                                                                      " not in RTK-Fixed. Current mode " << pvaIter->type << " ignoring ...");
          pvaIter ++;
          continue;
        }
        else if (pvaIter->type != fgo::data_types::GNSSSolutionType::RTKFIX)
        {
          RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": PVA at " << std::fixed << corrected_time <<
                                                                                     " integrating without RTK-Fix. Current mode " << pvaIter->type);
          switch (pvaIter->type) {
            case fgo::data_types::GNSSSolutionType::RTKFLOAT:
            {
              posVarScale *= paramPtr_->varScaleRTKFloat;
              velVarScale *= paramPtr_->varScaleRTKFloat;
              rotVarScale *= paramPtr_->varScaleHeadingRTKFloat;
              break;
            }
            case fgo::data_types::GNSSSolutionType::SINGLE:
            {
              posVarScale *= paramPtr_->varScaleSingle;
              velVarScale *= paramPtr_->varScaleSingle;
              rotVarScale *= paramPtr_->varScaleHeadingSingle;
              break;
            }
            case fgo::data_types::GNSSSolutionType::NO_SOLUTION:
            {
              posVarScale *= paramPtr_->varScaleNoSolution;
              velVarScale *= paramPtr_->varScaleNoSolution;
              rotVarScale *= paramPtr_->varScaleHeadingSingle;
              break;
            }
            default:
              break;
          }
        }
        else
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": PVA in RTK-fixed mode");

        const auto integrateVelocity = paramPtr_->integrateVelocity && pvaIter->has_velocity;
        const auto integrateAttitude = paramPtr_->integrateAttitude && pvaIter->has_heading;

        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << integratorName_ + ": Current PVA ts: " << corrected_time << " integrateVelocity: " << integrateVelocity << " integrateAttitude: " << integrateAttitude);

        //std::cout << "current pos var: " << pvaIter->xyz_var * posVarScale << std::endl;
        //std::cout << "current pos var: " << pvaIter->vel_var * velVarScale << std::endl;
        //std::cout << "ecef: " << pvaIter->xyz_ecef << std::endl;

        auto syncResult = findStateForMeasurement(currentKeyIndexTimestampMap, corrected_time, paramPtr_);
        const auto current_pred_state = timePredStates.back().second; //graph::querryCurrentPredictedState(timePredStates, corrected_time);

        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ + ":  Found State I: " << std::fixed <<
                                                                                             syncResult.keyIndexI << " : " << gtsam::symbolIndex(syncResult.keyIndexI) << " at: " << syncResult.timestampI << " and J: "
                                                                                             << syncResult.keyIndexJ << " : " << gtsam::symbolIndex(syncResult.keyIndexJ) << " at: " << syncResult.timestampJ
                                                                        << " DurationToI: " << syncResult.durationFromStateI);

        if (syncResult.durationFromStateI > 1.)
        {
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": DIRTY fix, jump measurement falling between zero-velocity trick");
            return true;
        }

        if (syncResult.stateJExist()) {
          pose_key_i  = X(syncResult.keyIndexI);
          vel_key_i   = V(syncResult.keyIndexI);
          omega_key_i = W(syncResult.keyIndexI);

          pose_key_j  = X(syncResult.keyIndexJ);
          vel_key_j   = V(syncResult.keyIndexJ);
          omega_key_j = W(syncResult.keyIndexJ);
          // RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "accI.head(3): " << accI.head(3) << "\n" << accI.tail(3));
        } else {
          RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": GP PVT: NO state J found !!! ");
        }

        gtsam::Vector3 thisVelocity;

        if (paramPtr_->velocityFrame == fgo::factor::MeasurementFrame::ECEF)
            thisVelocity = pvaIter->vel_ecef;
        else if (paramPtr_->velocityFrame == fgo::factor::MeasurementFrame::NED || paramPtr_->velocityFrame == fgo::factor::MeasurementFrame::ENU)
            thisVelocity = pvaIter->vel_n;

        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I || syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J)
        {
          //auto [foundGyro, this_gyro] = findOmegaToMeasurement(corrected_time, timestampGyroMap);
          //this_gyro = current_pred_state.imuBias.correctGyroscope(this_gyro);

          // now we found a state which is synchronized with the GNSS obs
          if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
            pose_key_sync = pose_key_i;
            vel_key_sync = vel_key_i;
              omega_key_sync = omega_key_i;
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                               integratorName_ + ": Found time synchronized state at I " << gtsam::symbolIndex(pose_key_sync)
                                                                               <<
                                                                               " with time difference: "
                                                                               << syncResult.durationFromStateI);
          }
          else {
            pose_key_sync = pose_key_j;
            vel_key_sync = vel_key_j;
              omega_key_sync = omega_key_j;
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                               integratorName_ + ": Found time synchronized state at J " << gtsam::symbolIndex(pose_key_sync)
                                                                               <<
                                                                               " with time difference: "
                                                                               << syncResult.durationFromStateI);
          }


          if(integrateVelocity)
          {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "Integrating PVT ...");

            this->addGNSSPVTFactor(pose_key_sync, vel_key_sync, omega_key_sync, pvaIter->xyz_ecef, thisVelocity,
                                   pvaIter->xyz_var * posVarScale, pvaIter->vel_var * velVarScale, paramPtr_->transIMUToAnt1);
          }
          else
          {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "Integrating GNSS positioning ...");
            this->addGNSSFactor(pose_key_sync, pvaIter->xyz_ecef, pvaIter->xyz_var * posVarScale, paramPtr_->transIMUToAnt1);
          }

          if(integrateAttitude && pvaIter->has_heading)
          {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "Integrating GNSS Attitude ...");
            this->addNavAttitudeFactor(pose_key_sync, pvaIter->rot, pvaIter->rot_var * rotVarScale, paramPtr_->attitudeType);
          }
        }
        else if (syncResult.status == StateMeasSyncStatus::INTERPOLATED && paramPtr_->useGPInterpolatedFactor)
        {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                             integratorName_ + ": Found not synchronized between " << syncResult.keyIndexI << " and " << syncResult.keyIndexJ <<
                                                                                   " with time difference: "
                                                                          << syncResult.durationFromStateI);
          const double delta_t = syncResult.timestampJ - syncResult.timestampI;
          const double taui = syncResult.durationFromStateI;

          //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE

          if(paramPtr_->gpType == fgo::data_types::GPModelType::WNOJ) {
            const auto [foundI, accI, fountJ, accJ] = findAccelerationToState(syncResult.keyIndexI, stateIDAccMap);
            interpolator_->recalculate(delta_t, taui, accI, accJ);
          }
          else
            interpolator_->recalculate(delta_t, taui);

          if(integrateVelocity)
          {
             RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "Integrating GP interpolated PVT ...");
              this->addGPInterpolatedGNSSPVTFactor(pose_key_i, vel_key_i, omega_key_i,
                                                   pose_key_j, vel_key_j, omega_key_j,
                                                   pvaIter->xyz_ecef, thisVelocity,
                                                   pvaIter->xyz_var * posVarScale, pvaIter->vel_var * velVarScale,
                                                   paramPtr_->transIMUToAnt1, interpolator_);
          }
          else
          {
            //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Integrating GP interpolated GNSS positioning ...");
              this->addGPInterpolatedGNSSFactor(pose_key_i, vel_key_i, omega_key_i,
                                                pose_key_j, vel_key_j, omega_key_j,
                                                pvaIter->xyz_ecef, pvaIter->xyz_var * posVarScale, paramPtr_->transIMUToAnt1, interpolator_);
          }

          if(integrateAttitude && pvaIter->has_heading)
          {
           // RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Integrating GP interpolated GNSS Attitude ...");

              this->addGPInterpolatedNavAttitudeFactor(pose_key_i, vel_key_i, omega_key_i,
                                                       pose_key_j, vel_key_j, omega_key_j,
                                                       pvaIter->rot, pvaIter->rot_var * rotVarScale,
                                                       interpolator_, paramPtr_->attitudeType);
          }
        }
        else if(syncResult.status == StateMeasSyncStatus::CACHED)
        {
          restGNSSMeas.push_back(*pvaIter);
        }
        pvaIter ++;
      }
      return true;
    }

    bool GNSSLCIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &martinals,
                                       const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                       data_types::State &optState) {
      return true;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::integrator::GNSSLCIntegrator, fgo::integrator::IntegratorBase)