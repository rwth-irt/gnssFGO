// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef ONLINE_FGO_INTEGRATIONBASE_H
#define ONLINE_FGO_INTEGRATIONBASE_H

#pragma once
#include <tuple>
#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>
#include <gtsam/navigation/AttitudeFactor.h>
#include <irt_nav_msgs/msg/pps.hpp>
#include <irt_nav_msgs/msg/sensor_processing_report.hpp>

#include "graph/GraphBase.h"
#include "data/DataTypes.h"
#include "utils/ROSParameter.h"
#include "factor/motion/GPWNOAPriorPose3.h"
#include "factor/motion/GPWNOJPriorPose3.h"
#include "model/gp_interpolator/GPWNOAInterpolatorPose3.h"
#include "model/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "utils/MeasurmentDelayCalculator.h"
#include "graph/GraphUtils.h"
#include "integrator/param/IntegratorParams.h"
#include "factor/measurement/odometry/NavAttitudeFactor.h"
#include "factor/measurement/odometry/GPInterpolatedNavAttitudeFactor.h"
#include "factor/measurement/odometry/NavVelocityFactor.h"
#include "factor/measurement/odometry/GPInterpolatedNavVelocityFactor.h"
#include "factor/measurement/odometry/NavPoseFactor.h"
#include "factor/measurement/odometry/GPInterpolatedNavPoseFactor.h"
#include "factor/motion/VelocityPreintegration.h"
#include "solver/FixedLagSmoother.h"

namespace fgo::integrator
{
    using namespace fgo::integrator::param;
    using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
    using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
    using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
    using gtsam::symbol_shorthand::N;  // integer ddambiguities
    using gtsam::symbol_shorthand::M;  // integer ambiguities for td cp without dd
    using gtsam::symbol_shorthand::A;
    using gtsam::symbol_shorthand::O;

    enum StateMeasSyncStatus
    {
        SYNCHRONIZED_I = 0,
        SYNCHRONIZED_J = 1,
        INTERPOLATED = 2,
        CACHED = 3,
        DROPPED = 4
    };

    struct StateMeasSyncResult
    {
        StateMeasSyncStatus status = StateMeasSyncStatus::DROPPED;
        bool foundI = false;
        size_t keyIndexI = 0;
        size_t keyIndexJ = 0;
        double timestampI = std::numeric_limits<double>::max();
        double timestampJ = std::numeric_limits<double>::max();
        double durationFromStateI = std::numeric_limits<double>::max();

        [[nodiscard]] bool stateJExist() const
        {return timestampJ < std::numeric_limits<double>::max();}

        [[nodiscard]] bool stateIExist() const
        {return timestampI < std::numeric_limits<double>::max();}

        [[nodiscard]] bool statesExist() const
        {return stateJExist() && stateIExist();}
    };

    class IntegratorBase
    {
    protected:
        std::string integratorName_;
        bool isPrimarySensor_ = false;

        rclcpp::Node* rosNodePtr_{}; // ROS Node Handle.
        rclcpp::Publisher<irt_nav_msgs::msg::SensorProcessingReport>::SharedPtr pubSensorReport_;

        fgo::graph::GraphBase* graphPtr_{};
        IntegratorBaseParamsPtr integratorBaseParamPtr_;
        uint64_t nState_{};
        double noOptimizationDuration_ = 0.;
        fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap lastKeyIndexTimestampMap_;
        std::vector<size_t> lastLagKeys_;

        std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callbackGroupMap_;

        /*
         *  Utilities
         */
        rclcpp::Subscription<irt_nav_msgs::msg::PPS>::SharedPtr subPPS_;
        std::unique_ptr<fgo::utils::MeasurementDelayCalculator> delayCalculator_;

    public:

        /***
         * if a state out of the current lag size should be queried for e.g. delayed sensor observations, the solver
         * would crash, we filter these observations out by checking if the related states are alive
         * @param id state id as number (not gtsam identifier)
         * @return true if the state can be queried
         */
        bool checkStatePresentedInCurrentLag(size_t id)
        {
            if(lastLagKeys_.empty())
            {
                RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ << " checking state presence for key: " << id << " current key vector empty!");
                return true;
            }
            auto keyIter = std::find(lastLagKeys_.begin(), lastLagKeys_.end(), id);
            if(keyIter == lastLagKeys_.end()) {
                return false;
            }
            else
                return true;
        }

        /***
         * set attitude type from the ros parameter, different reference sensor may provide different types of attitude observations
         * @param typeStr must be one of rpy, yawroll, yawpitch, yaw, roll, pitch
         * @param type intern rotation type
         * @param name name for logging, usually the sensor name
         */
        static void setAttitudeType(const std::string& typeStr,
                                    fgo::factor::AttitudeType& type,
                                    const std::string& name = "")
        {
          auto typeStrLower = boost::algorithm::to_lower_copy(typeStr); // modifies str
          if(typeStrLower == "rpy")
            type = fgo::factor::AttitudeType::RPY;
          else if(typeStrLower == "yawroll")
            type = fgo::factor::AttitudeType::YAWROLL;
          else if(typeStrLower == "yawpitch")
            type = fgo::factor::AttitudeType::YAWPITCH;
          else if(typeStrLower == "yaw")
            type = fgo::factor::AttitudeType::YAW;
          else if(typeStrLower == "roll")
            type = fgo::factor::AttitudeType::ROLL;
          else if(typeStrLower == "pitch")
            type = fgo::factor::AttitudeType::PITCH;
          else
            RCLCPP_WARN_STREAM(rclcpp::get_logger("onlineFGO"), name << " improper attitude type: " << typeStrLower);
        }

        /***
         * get attitude noise vector according to the attitude type, if the attitude only contain one component,
         * the dimension of the error should be adapted
         * @param noise3D original full noise vector
         * @param type type of the attitude
         * @return noise vector
         */
        static gtsam::Vector getAttitudeNoiseVector(const gtsam::Vector3& noise3D,
                                                    const fgo::factor::AttitudeType& type)
        {
          switch(type)
          {
            case fgo::factor::AttitudeType::RPY:
            {
              return noise3D;
            }
            case fgo::factor::AttitudeType::YAWPITCH:
            {
              return (gtsam::Vector2() << noise3D.y(), noise3D.z()).finished();
            }
            case fgo::factor::AttitudeType::YAWROLL:
            {
              return (gtsam::Vector2() << noise3D.x(), noise3D.z()).finished();
            }
            case fgo::factor::AttitudeType::YAW:
            {
              return (gtsam::Vector1() << noise3D.z()).finished();
            }
            case fgo::factor::AttitudeType::ROLL:
            {
              return (gtsam::Vector1() << noise3D.x()).finished();
            }
            case fgo::factor::AttitudeType::PITCH:
            {
              return (gtsam::Vector1() << noise3D.y()).finished();
            }
          }
        }

        /***
         * set velocity type from the ros parameter, different reference sensor may provide different types of velocity observations
         * @param typeStr must be one of 3d, 2d, y, x, z
         * @param type intern velocity type
         * @param name name for logging, usually the sensor name
         */
        static void setVelocityType(const std::string& typeStr,
                                    fgo::factor::VelocityType& type,
                                    const std::string& name = "")
        {
          auto typeStrLower = boost::algorithm::to_lower_copy(typeStr); // modifies str
          if(typeStrLower == "3d")
            type = fgo::factor::VelocityType::VEL3D;
          else if(typeStrLower == "2d")
            type = fgo::factor::VelocityType::VEL2D;
          else if(typeStrLower == "y")
            type = fgo::factor::VelocityType::VELY;
          else if(typeStrLower == "x")
            type = fgo::factor::VelocityType::VELX;
          else if(typeStrLower == "z")
            type = fgo::factor::VelocityType::VELZ;
          else
            RCLCPP_WARN_STREAM(rclcpp::get_logger("onlineFGO"), name << " improper velocity type: " << typeStrLower);
        }

        /***
         * get velocity noise vector according to the velocity type, if the velocity only contain one component,
         * the dimension of the error should be adapted
         * @param noise3D original full noise vector
         * @param type type of the velocity
         * @return noise vector
         */
        static gtsam::Vector getVelocityNoiseVector(const gtsam::Vector3& noise3D,
                                                    const fgo::factor::VelocityType& type)
        {
          switch(type)
          {
            case fgo::factor::VelocityType::VEL3D:
            {
              return noise3D;
            }
            case fgo::factor::VelocityType::VEL2D:
            {
              return (gtsam::Vector2() << noise3D.x(), noise3D.y()).finished();
            }
            case fgo::factor::VelocityType::VELX:
            {
              return (gtsam::Vector1() << noise3D.x()).finished();
            }
            case fgo::factor::VelocityType::VELY:
            {
              return (gtsam::Vector1() << noise3D.y()).finished();
            }
            case fgo::factor::VelocityType::VELZ:
            {
              return (gtsam::Vector1() << noise3D.z()).finished();
            }
          }
        }

        /***
         * set sensor observation frame
         * @param frameStr must be one of body, ned, enu, ecef, default is ned
         * @param frame intern frame variable
         * @param type name for logging, usually the sensor name
         */
        static void setSensorFrameFromParam(const std::string& frameStr,
                                            fgo::factor::MeasurementFrame& frame,
                                            const std::string& type = "")
        {
          auto frameStrLower = boost::algorithm::to_lower_copy(frameStr); // modifies str
          if(frameStrLower == "body")
            frame = fgo::factor::MeasurementFrame::BODY;
          else if(frameStrLower == "ned")
            frame = fgo::factor::MeasurementFrame::NED;
          else if(frameStrLower == "enu")
            frame = fgo::factor::MeasurementFrame::ENU;
          else if(frameStrLower == "ecef")
            frame = fgo::factor::MeasurementFrame::ECEF;
          else {
            frame = fgo::factor::MeasurementFrame::NED;
            RCLCPP_WARN_STREAM(rclcpp::get_logger("onlineFGO"), type << " improper frame: " << frameStrLower << " setting as NED");
          }
        }

        /***
         * set the noise model from ros parameter
         * @param modelStr name of the noise model
         * @param model intern model variable
         * @param type name for logging, usually the sensor name
         */
        static void setNoiseModelFromParam(const std::string& modelStr,
                                           fgo::data_types::NoiseModel& model,
                                           const std::string& type = "")
        {
          auto modelStrLower = boost::algorithm::to_lower_copy(modelStr); // modifies str
          if(modelStrLower == "gaussian")
            model = data_types::NoiseModel::GAUSSIAN;
          else if(modelStrLower == "huber")
            model = data_types::NoiseModel::HUBER;
          else if(modelStrLower == "cauchy")
            model = data_types::NoiseModel::CAUCHY;
          else if(modelStrLower == "dcs")
            model = data_types::NoiseModel::DCS;
          else if(modelStrLower == "tukey")
            model = data_types::NoiseModel::Tukey;
          else if(modelStrLower == "gemanmcclure")
            model = data_types::NoiseModel::GemanMcClure;
          else if(modelStrLower == "welsch")
            model = data_types::NoiseModel::Welsch;
          else{
            model = data_types::NoiseModel::GAUSSIAN;
            RCLCPP_WARN_STREAM(rclcpp::get_logger("onlineFGO"), type <<" improper noise model: " << modelStrLower << " setting as Gaussian");
          }

        }


    protected:
        void addNavAttitudeFactor(const gtsam::Key& poseKey,
                                  const gtsam::Rot3& rotMeasured,
                                  const gtsam::Vector3& rotMeasuredVar,
                                  factor::AttitudeType type = factor::AttitudeType::RPY)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelAttitude,
                                                          getAttitudeNoiseVector(rotMeasuredVar, type),
                                                          integratorBaseParamPtr_->robustParamAttitude,
                                                          "NavAttitude");

          graphPtr_->emplace_shared<fgo::factor::NavAttitudeFactor>(poseKey,
                                                                    rotMeasured,
                                                                    integratorBaseParamPtr_->attitudeFrame,
                                                                    type,
                                                                    noiseModel,
                                                                    integratorBaseParamPtr_->AutoDiffNormalFactor);
        }

        void addGPInterpolatedNavAttitudeFactor(const gtsam::Key& poseKeyI, const gtsam::Key& velKeyI, const gtsam::Key& omegaKeyI,
                                                const gtsam::Key& poseKeyJ, const gtsam::Key& velKeyJ, const gtsam::Key& omegaKeyJ,
                                                const gtsam::Rot3 &rotMeasured, const gtsam::Vector3& rotMeasuredVar,
                                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                                factor::AttitudeType type = factor::AttitudeType::RPY)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelAttitude,
                                                          getAttitudeNoiseVector(rotMeasuredVar, type),
                                                          integratorBaseParamPtr_->robustParamAttitude,
                                                          "GPInterpolatedNavAttitude");
            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavAttitudeFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ,
                                                                                    rotMeasured, integratorBaseParamPtr_->attitudeFrame, type,
                                                                                    noiseModel, interpolator, integratorBaseParamPtr_->AutoDiffGPInterpolatedFactor);

        }

        void addNavVelocityFactor(const gtsam::Key& poseKey, const gtsam::Key& velKey,
                                  const gtsam::Vector3 &velMeasured, const gtsam::Vector3& velMeasuredVar,
                                  const gtsam::Vector3 &omega,
                                  const gtsam::Vector3& lb, factor::VelocityType type = factor::VelocityType::VEL3D)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelVelocity,
                                                          getVelocityNoiseVector(velMeasuredVar, type),
                                                          integratorBaseParamPtr_->robustParamVelocity,
                                                          "NavVelocity");
          //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "velocityFrame: " << integratorBaseParamPtr_->velocityFrame);
          graphPtr_->emplace_shared<fgo::factor::NavVelocityFactor>(poseKey, velKey, velMeasured, omega, lb,
                                                                    integratorBaseParamPtr_->velocityFrame, type,
                                                                    noiseModel,  1);
        }

        void addGPInterpolatedNavVelocityFactor(const gtsam::Key& poseKeyI, const gtsam::Key& velKeyI, const gtsam::Key& omegaKeyI,
                                                const gtsam::Key& poseKeyJ, const gtsam::Key& velKeyJ, const gtsam::Key& omegaKeyJ,
                                                const gtsam::Vector3 &velMeasured, const gtsam::Vector3& velMeasuredVar,
                                                const gtsam::Vector3& lb,
                                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                                factor::VelocityType type = factor::VelocityType::VEL3D)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelVelocity,
                                                          getVelocityNoiseVector(velMeasuredVar, type),
                                                          integratorBaseParamPtr_->robustParamVelocity,
                                                          "GPInterpolatedNavVelocity");
          graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavVelocityFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ,
                                                                                  velMeasured, lb, integratorBaseParamPtr_->velocityFrame, type,
                                                                                  noiseModel, interpolator, true);
        }

        void addNavPoseFactor(const gtsam::Key& poseKey, const gtsam::Pose3& poseMeasured, const gtsam::Vector6& poseVar)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                          poseVar,
                                                          integratorBaseParamPtr_->robustParamOdomPose,
                                                          "addNavPoseFactor");
          graphPtr_->emplace_shared<fgo::factor::NavPoseFactor>(poseKey, poseMeasured, noiseModel);
        }

        void addGPinteporatedNavPoseFactor(const gtsam::Key& poseKeyI, const gtsam::Key& velKeyI, const gtsam::Key& omegaKeyI,
                                           const gtsam::Key& poseKeyJ, const gtsam::Key& velKeyJ, const gtsam::Key& omegaKeyJ,
                                           const gtsam::Pose3 &poseMeasured, const gtsam::Vector6& poseVar,
                                           const std::shared_ptr<fgo::models::GPInterpolator> &interpolator)
        {
          const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                          poseVar,
                                                          integratorBaseParamPtr_->robustParamOdomPose,
                                                          "addNavPoseFactor");
          graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavPoseFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ,
                                                                              poseMeasured,
                                                                              interpolator, noiseModel, integratorBaseParamPtr_->AutoDiffGPInterpolatedFactor);
        }


        /***
         * initialize the common integrator parameters
         */
        void initIntegratorBaseParams()
        {
          RCLCPP_INFO(rosNodePtr_->get_logger(), "---------------------  IntegratorBase initializing! --------------------- ");
          RCLCPP_INFO(rosNodePtr_->get_logger(), "------- IntegratorBase Parameters: -------");
          integratorBaseParamPtr_ = std::make_shared<fgo::integrator::param::IntegratorBaseParams>(graphPtr_->getParamPtr());
          ::utils::RosParameter<double> IMUGNSSSyncTimeThreshold("GNSSFGO.IntegratorBase.IMUSensorSyncTimeThreshold", *rosNodePtr_);
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "IMUSensorSyncTimeThreshold: " << IMUGNSSSyncTimeThreshold.value());
          integratorBaseParamPtr_->IMUSensorSyncTimeThreshold = IMUGNSSSyncTimeThreshold.value();
          ::utils::RosParameter<double> StateSensorSyncTimeThreshold("GNSSFGO.IntegratorBase.StateSensorSyncTimeThreshold", *rosNodePtr_);
          integratorBaseParamPtr_->StateSensorSyncTimeThreshold = StateSensorSyncTimeThreshold.value();
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "StateSensorSyncTimeThreshold: " << StateSensorSyncTimeThreshold.value());
          RCLCPP_INFO(rosNodePtr_->get_logger(), "---------------------  IntegratorBase initialized! --------------------- ");
        }

    public:
        typedef std::shared_ptr<IntegratorBase> Ptr;
        typedef std::map<std::string, Ptr> IntegratorMap;
        explicit IntegratorBase() = default;

        /***
         * initialize the integrator instance, used in graphBase
         * @param node ros node, may be its derived object
         * @param graphPtr graph pointer
         * @param integratorName name of the sensor
         * @param isPrimarySensor indicate if the sensor is used as primayry sensor for sensor-centric integration
         */
        virtual void initialize(rclcpp::Node& node,
                                fgo::graph::GraphBase& graphPtr,
                                const std::string& integratorName,
                                bool isPrimarySensor = false) {
          rosNodePtr_ = &node;
          graphPtr_ = &graphPtr;
          integratorName_ = integratorName;
          integratorBaseParamPtr_ = std::make_shared<fgo::integrator::param::IntegratorBaseParams>(graphPtr_->getParamPtr());
          isPrimarySensor_ = isPrimarySensor;

          pubSensorReport_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::SensorProcessingReport>("sensor_processing_report/" + integratorName_,
                                                                                                      rclcpp::SystemDefaultsQoS());
        };

        virtual ~IntegratorBase() = default;

        /***
         * convert and formulate the sensor observations and add them into the graph
         * @param timestampGyroMap gyro measurements could be used to correct the leverarm effect
         * @param stateIDAccMap acc measurement could be used for GP interpolation
         * @param currentKeyIndexTimestampMap current state keyindex and timestamp map, used to querry states
         * @param timePredStates some sensor observations may need current system state to be pre-processed
         * @param values prior values
         * @param keyTimestampMap reverse of keyindexTimestampMap, TODO@Haoming, may be unseless
         * @param relatedKeys a vector of related states for all observations, used in the solver
         * @return
         */
        virtual bool factorize(
            const boost::circular_buffer<std::pair<double, gtsam::Vector3>>& timestampGyroMap,
            const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& currentKeyIndexTimestampMap,
            std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
            gtsam::Values& values,
            fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
            gtsam::KeyVector& relatedKeys
            ) {};

        virtual std::map<uint64_t, double> factorizeAsPrimarySensor() {};

        virtual void bufferIMUData(double imuTimestamp, const gtsam::Vector6 acc) {};

        virtual bool checkZeroVelocity() {return false;};

        virtual bool checkHasMeasurements() {return true;};

        virtual void cleanBuffers() {};

        virtual void notifyOptimization(double noOptimizationDuration){
          noOptimizationDuration_ = 0.;
        };

        /***
         * some algorithms, such as odometries, need optimized system state to update keyframes
         * @param result gtsam result containing all optimized state variables
         * @param marginals gtsam marginal containing all optimized state variables
         * @param keyIndexTimestampMap current state keyindex and timestamp map, used to querry states
         * @param optState current optimized state
         * @return
         */
        virtual bool fetchResult(
            const gtsam::Values& result,
            const gtsam::Marginals& marginals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& keyIndexTimestampMap,
            fgo::data_types::State& optState
            ) {};

        virtual void dropMeasurementBefore(double timestamp){};

        std::string getName() {return integratorName_;}

        [[nodiscard]] bool isPrimarySensor() const {return isPrimarySensor_;}

        /***
         * find acc measurement according to the state id
         * @param idStateI state id
         * @param stateIDAccMap map
         * @return if a measurement could be found, associated acc measurement
         */
        std::tuple<bool, gtsam::Vector6, bool, gtsam::Vector6> findAccelerationToState(size_t idStateI,
                                                                          const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap)
        {
          bool foundAccI = false, foundAccJ = false;
          gtsam::Vector6 accI = (stateIDAccMap.end() - 1)->second, accJ = stateIDAccMap.back().second;
          auto accMapIterator = stateIDAccMap.begin();
          while (accMapIterator != stateIDAccMap.end()) {
            if (accMapIterator->first == idStateI) {
              accI = accMapIterator->second;
              accMapIterator++;
              foundAccI = true;
              if(accMapIterator == stateIDAccMap.end())
                accJ = (accMapIterator - 1)->second;
              else {
                foundAccJ = true;
                accJ = accMapIterator->second;
              }
              break;
            }
            accMapIterator++;
          }
          if(!foundAccI || !foundAccJ)
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ << " can find accI? " << foundAccI << " or accJ? " << foundAccJ);
          return {foundAccI, accI, foundAccJ, accJ};
        }

        /***
         * find gyro measurement according to the state id
         * @param idStateI state id
         * @param stateIDAccMap map
         * @return if a measurement could be found, associated gyro measurement
         */
        std::tuple<bool, gtsam::Vector3> findOmegaToMeasurement(double timestamp,
                                              const boost::circular_buffer<std::pair<double, gtsam::Vector3>>& timestampGyroMap)
        {
          gtsam::Vector3 this_gyro = timestampGyroMap.back().second;
          bool find_gyro = false;
          auto timeGyroIter = timestampGyroMap.rbegin();
          while (timeGyroIter != timestampGyroMap.rend()) {
            const auto gyroTimediff = abs(timeGyroIter->first - timestamp);
            if ( gyroTimediff < integratorBaseParamPtr_->IMUSensorSyncTimeThreshold)
            {
              find_gyro = true;
              this_gyro = timeGyroIter->second;
              //RCLCPP_WARN_STREAM(appPtr_->get_logger(), integratorName_ + ": found a gyro at " << std::fixed << gyroTimediff);
              break;
            }
            timeGyroIter++;
          }
          if(!find_gyro)
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": cannot find a gyro, using last one ...");
          return {find_gyro, this_gyro};
        }

        /***
         * check if an observation can be synchronized with a state given a timing lower and upper bound
         * @param varIDTimestampMap state id and timestampe map
         * @param correctedTimestampMeas delay corrected observations timestamp
         * @param paramPtr parameters
         * @return StateMeasSyncResult
         */
        static StateMeasSyncResult findStateForMeasurement(const std::map<size_t, double>& varIDTimestampMap,
                                                           const double& correctedTimestampMeas,
                                                           IntegratorBaseParamsPtr paramPtr)
        {
          StateMeasSyncResult result;
          if(varIDTimestampMap.empty()) {
            result.status = StateMeasSyncStatus::DROPPED;
            return result;
          }

          if(varIDTimestampMap.begin()->second > correctedTimestampMeas && (correctedTimestampMeas - varIDTimestampMap.begin()->second) < paramPtr->StateMeasSyncLowerBound)
          {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("online_fgo"), "Measurement at: "<< std::fixed <<
                                                                                   correctedTimestampMeas << " is out of the current lag with earliest state: " << varIDTimestampMap.begin()->second);
            result.status = StateMeasSyncStatus::DROPPED;
            return result;
          }

          auto varIDTimeIter = varIDTimestampMap.rbegin();

          while(varIDTimeIter != varIDTimestampMap.rend())
          {
            double varGNSSdt = correctedTimestampMeas - varIDTimeIter->second;

            //RCLCPP_INFO_STREAM(rclcpp::get_logger("fgo"), "Iterate state: "<< std::fixed <<
            //                                                                      varIDTimeIter->first << " at: " << varIDTimeIter->second << " with dt to sensor: " << varGNSSdt);

            if(varIDTimeIter == varIDTimestampMap.rbegin() && varGNSSdt > paramPtr->StateMeasSyncUpperBound)
            {
              // If this measurement is in front of the LAST created state, we can't interpolate!
              // then we cache this measurement for next iteration
              // which also means, that there is no need to iterate all measurements AFTER this measurement

              RCLCPP_WARN_STREAM(rclcpp::get_logger("online_fgo"), "findStateForMeasurement: "<< std::fixed <<
                                                                                              " Measurement at " << correctedTimestampMeas << " in front of LAST state at " << varIDTimeIter->second
                                                                                              << ". This Measurement will be cached! ");
              result.status = StateMeasSyncStatus::CACHED;
              return result;
            }

            if(paramPtr->StateMeasSyncLowerBound <= varGNSSdt && varGNSSdt <= paramPtr->StateMeasSyncUpperBound)
            {
              // UPDATE 12.07: here we found the i, which can be exactly synchronized with the current measurement, we don't need to find J!
              // We should still check if it is synced with I or J, because of TDCP
              result.foundI = true;
              if (varGNSSdt >= 0 && varIDTimeIter != varIDTimestampMap.rbegin()){ //Found sync I
                result.keyIndexI = varIDTimeIter->first;
                result.keyIndexJ = result.keyIndexI + 1;
                result.timestampI = varIDTimeIter->second;
                result.timestampJ = std::prev(varIDTimeIter)->second;
                result.durationFromStateI = abs(varGNSSdt);
                result.status = StateMeasSyncStatus::SYNCHRONIZED_I;
                return result;
              } else { //Found sync J
                result.keyIndexJ = varIDTimeIter->first;
                result.keyIndexI = result.keyIndexJ - 1;
                result.timestampJ = varIDTimeIter->second;
                result.timestampI = std::next(varIDTimeIter)->second;
                result.durationFromStateI = result.timestampJ - result.timestampI - abs(varGNSSdt);
                result.status = StateMeasSyncStatus::SYNCHRONIZED_J;
                return result;
              }
            }

            // If we go here, means, the meas is in between!
            // at this time, varGNSSdt is larger than 0 and has a reference of stateI.
            // the varGNSSdt will be used at tau between stateI and stateJ for the integrator
            if(varIDTimeIter->second < correctedTimestampMeas)
            {
              result.keyIndexI = varIDTimeIter->first;
              result.timestampI = varIDTimeIter->second;
              result.durationFromStateI = varGNSSdt;
              result.keyIndexJ = result.keyIndexI + 1;
              result.timestampJ = std::prev(varIDTimeIter)->second;
              result.foundI = true;
              result.status = StateMeasSyncStatus::INTERPOLATED;
              return result;
            }
            varIDTimeIter ++;
          }
          return result;
        }
    };
}
#endif //ONLINE_FGO_INTEGRATIONBASE_H
