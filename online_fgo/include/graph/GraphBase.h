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
#ifndef ONLINE_FGO_GRAPHBASE_H
#define ONLINE_FGO_GRAPHBASE_H
#pragma once

#include <tuple>

//boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
//ros
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <utility>

#include <irt_nav_msgs/msg/factor_residual.hpp>
#include <irt_nav_msgs/msg/factor_residuals.hpp>
#include <irt_nav_msgs/msg/sensor_processing_report.hpp>
//gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
//internal
#include "graph/GraphUtils.h"
#include "graph/param/GraphParams.h"
#include "factor/FactorTypeIDs.h"
#include "factor/measurement/inertial/MagFactor_eRb.h"
#include "factor/motion/ConstAngularRateFactor.h"
#include "factor/motion/ConstDriftFactor.h"
#include "factor/motion/ConstVelPriorFactor.h"
#include "factor/motion/GPWNOAPriorPose3.h"
#include "factor/motion/GPWNOJPriorPose3.h"
#include "solver/FixedLagSmoother.h"
#include "solver/BatchFixedLagSmoother.h"
#include "solver/IncrementalFixedLagSmoother.h"
#include "data/DataTypes.h"
#include "data/Buffer.h"
#include "utils/NavigationTools.h"
#include "utils/ROSParameter.h"
#include "utils/Pose3utils.h"
#include "gnss_fgo/param/GNSSFGOParams.h"
#include "utils/GPutils.h"


namespace fgo::integrator
{
    class IntegratorBase;
    typedef std::map<std::string, std::shared_ptr<IntegratorBase>> IntegratorMap;
}

namespace gnss_fgo
{
    class GNSSFGOLocalizationBase;
}

namespace fgo::graph {
    using namespace ::utils;

    using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
    using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
    using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
    using gtsam::symbol_shorthand::N;  // integer ddambiguities
    using gtsam::symbol_shorthand::M;  // integer ambiguities for td cp without dd
    using gtsam::symbol_shorthand::A;  // acceleration
    using gtsam::symbol_shorthand::O;

    enum StateMeasSyncStatus
    {
        SYNCHRONIZED_I = 0,
        SYNCHRONIZED_J = 1,
        INTERPOLATED = 2,
        CACHED = 3,
        DROPPED = 4
    };

    enum StatusGraphConstruction
    {
        FAILED = 0,
        SUCCESSFUL = 1,
        NO_OPTIMIZATION = 2,
    };

    class GraphBase : public gtsam::NonlinearFactorGraph, public std::enable_shared_from_this<GraphBase> {

    protected:
        std::string graphName_ = "OnlineFGO";
        gnss_fgo::GNSSFGOLocalizationBase* appPtr_;
        rclcpp::Publisher<irt_nav_msgs::msg::FactorResiduals>::SharedPtr pubResiduals_;

        std::shared_ptr<std::thread> pubResidualsThread_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;
        pluginlib::ClassLoader<fgo::integrator::IntegratorBase> integratorLoader_; ///< Plugin loader
        GraphParamBasePtr graphBaseParamPtr_;

        fgo::integrator::IntegratorMap integratorMap_;
        std::shared_ptr<fgo::integrator::IntegratorBase> primarySensor_;

        //fgo::param::GNSSINSIntegrationParams LIOParams_;
        fgo::solvers::FixedLagSmoother::KeyTimestampMap keyTimestampMap_;
        fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap currentKeyIndexTimestampMap_;
        gtsam::Values values_;
        uint64_t nState_ = 0; //counter for states
        std::atomic_bool isStateInited_{};
        std::unique_ptr<fgo::solvers::FixedLagSmoother> solver_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> fgoOptStateBuffer_;
        std::vector<fgo::data_types::IMUMeasurement> dataIMURest_;

        fgo::buffer::CircularDataBuffer<std::pair<rclcpp::Time, double>> referenceSensorTimestampBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> referenceStateBuffer_;
        fgo::buffer::CircularDataBuffer<gtsam::Vector6> accBuffer_;
        fgo::buffer::CircularDataBuffer<fgo::data_types::State> currentPredictedBuffer_;
        fgo::buffer::CircularDataBuffer<std::vector<gtsam::NonlinearFactor::shared_ptr>> factorBuffer_;
        fgo::buffer::CircularDataBuffer<gtsam::Values> graphResultBuffer_;
        gtsam::KeyVector relatedKeys_;

        //lists

    protected: // protected functions

        inline void addAngularFactor(const gtsam::Key &omega_j, const gtsam::Key &bias_j,
                                     const fgo::data_types::IMUMeasurement &imuM,
                                     const gtsam::Vector3& variance = gtsam::Vector3::Identity()) {
          gtsam::SharedNoiseModel noise_model;

          if(graphBaseParamPtr_->useEstimatedVarianceAfterInit)
            noise_model = gtsam::noiseModel::Diagonal::Variances(variance);
          else
            noise_model = gtsam::noiseModel::Diagonal::Sigmas(graphBaseParamPtr_->angularRateStd * gtsam::Vector3::Identity());

          this->emplace_shared<fgo::factor::ConstAngularRateFactor>(omega_j, bias_j, imuM.gyro, noise_model);
        }

        inline void addMagFactor_eRb(const gtsam::Key &pose_j, const fgo::data_types::IMUMeasurement &imuM,
                                     const fgo::data_types::State &currentPredState) {
          gtsam::Vector3 measured_mag = imuM.mag;
          gtsam::Vector3 direction_ecef = fgo::utils::getMagOrientation(currentPredState.state.position());//magneticModel_()
          gtsam::Vector3 bias = gtsam::Point3(0, 0, 0);
          gtsam::SharedNoiseModel noise_model_mag = gtsam::noiseModel::Diagonal::Sigmas(
              graphBaseParamPtr_->magnetometerStd / measured_mag.norm());
          this->emplace_shared<fgo::factor::MagFactor_eRb>(pose_j, measured_mag, direction_ecef, bias, noise_model_mag);
        }

        // TODO: @Haoming, move this to GNSSTCIntegrator
        inline void addConstDriftFactor(const gtsam::Key &cbd_i, const gtsam::Key &cbd_j, const double &dt, const gtsam::Vector2& variance = gtsam::Vector2::Identity()) {
          gtsam::Vector2 var;
          if(graphBaseParamPtr_->useEstimatedVarianceAfterInit)
            var = variance;
          else
            var = gtsam::Vector2(std::pow(graphBaseParamPtr_->constBiasStd, 2), std::pow(graphBaseParamPtr_->constDriftStd, 2));

          gtsam::SharedNoiseModel noise_model_cbd = assignNoiseModel(graphBaseParamPtr_->noiseModelClockFactor,
                                                                     var,
                                                                     graphBaseParamPtr_->robustParamClockFactor,
                                                                     "ConstClockDriftFactor");
          this->emplace_shared<fgo::factor::ConstDriftFactor>(cbd_i, cbd_j, dt, noise_model_cbd);
        }

        inline void addMotionModelFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &pose_j,
                                         const gtsam::Key &vel_j, const double &dt) {
          //std::cout << "dt internal: " << std::fixed << dt << std::endl;
          gtsam::SharedNoiseModel noise_model_mm = //TODO change 1 to dt
              gtsam::noiseModel::Diagonal::Sigmas(graphBaseParamPtr_->motionModelStd * dt * gtsam::Vector6::Ones());
          this->emplace_shared<fgo::factor::MotionModelFactor>(pose_i, vel_i, pose_j, vel_j, dt, noise_model_mm);
        }

        inline void addGPMotionPrior(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                     const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                     double dt,
                                     const boost::optional<gtsam::Vector6>& acc_i = boost::none,
                                     const boost::optional<gtsam::Vector6>& acc_j = boost::none) {
            gtsam::SharedNoiseModel  qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);

            if(acc_i && acc_j)
            {
              this->emplace_shared<fgo::factor::GPWNOJPriorPose3>(pose_i, vel_i, omega_i, pose_j, vel_j,
                                                                  omega_j, *acc_i, *acc_j, dt, qcModel,
                                                                  graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                                  graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian);

            }
            else
              this->emplace_shared<fgo::factor::GPWNOAPriorPose3>(pose_i, vel_i, omega_i, pose_j, vel_j,
                                                                  omega_j, dt, qcModel, graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                                  graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian);
        }

        void notifyOptimization();

    public:
        //constructor
        typedef std::shared_ptr<GraphBase> Ptr;
        typedef std::weak_ptr<GraphBase> WeakPtr;

        explicit GraphBase(gnss_fgo::GNSSFGOLocalizationBase& node);

        [[nodiscard]] GraphParamBasePtr getParamPtr() {return graphBaseParamPtr_;}

        /***
         * initialize the graph, used in the application after collecting all prior variables
         * @param initState state
         * @param initTimestamp timestamp, usually the timestamp of the referrence sensor
         * @param preIntegratorParams pram for imu pre-integration
         */
        virtual void initGraph(const fgo::data_types::State& initState,
                               const double& initTimestamp,
                               boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams);

        /***
         * init iSAM2
         * @param params iSAM2 params
         */
        void initSolver(const gtsam::ISAM2Params& params)
        {
          solver_ = std::make_unique<fgo::solvers::IncrementalFixedLagSmoother>(graphBaseParamPtr_->smootherLag, params);
        }

        /***
         * init batch
         * @param batch params
         */
        void initSolver(const gtsam::LevenbergMarquardtParams& params)
        {
          solver_ = std::make_unique<fgo::solvers::BatchFixedLagSmoother>(graphBaseParamPtr_->smootherLag, params);
        }

        /***
         * bridge the propagated state in the imu thread back for all sensors
         * @param state propagated state
         */
        void updatePredictedBuffer(const fgo::data_types::State& state)
        {
          currentPredictedBuffer_.update_buffer(state, state.timestamp);
        }

        /***
         * construct/extend the graph using imu as the timing reference
         * @param dataIMU
         * @return StatusGraphConstruction
         */
        virtual StatusGraphConstruction constructFactorGraphOnIMU(
            std::vector<fgo::data_types::IMUMeasurement>& dataIMU
            ) = 0;

        /***
         * construct/extend the graph using clock as the timing reference
         * @param stateTimestamps
         * @param dataIMU
         * @return StatusGraphConstruction
         */
        virtual StatusGraphConstruction constructFactorGraphOnTime(
            const std::vector<double>& stateTimestamps,
            std::vector<fgo::data_types::IMUMeasurement> &dataIMU
            ) = 0;

        /***
         * call the update function of solvers and collect state information
         * @param new_state
         * @return time for the optimization
         */
        virtual double optimize(fgo::data_types::State& new_state) = 0;

        /***
         * reset current temporary graph, the graph associated in the solver is not reset!
         */
        void resetGraph()
        {
          this->resize(0);
          keyTimestampMap_.clear();
          values_.clear();
          relatedKeys_.clear();
        }

        [[nodiscard]] const gtsam::Values& getValues() const {return values_;}
        [[nodiscard]] fgo::solvers::FixedLagSmoother::KeyTimestampMap& getNewKeyTimestampMap() {return keyTimestampMap_;}
        [[nodiscard]] const uint64_t& getStateIndex() const {return nState_;}
        [[nodiscard]] const std::vector<fgo::data_types::IMUMeasurement>& getRestIMUData() const {return dataIMURest_;}

        [[nodiscard]] std::vector<std::pair<rclcpp::Time, double>> getReferenceSensorMeasurementTime(){
          return referenceSensorTimestampBuffer_.get_all_buffer();
        };

        [[nodiscard]] std::vector<fgo::data_types::State> getReferenceStates(){
          return referenceStateBuffer_.get_all_buffer();
        };

        [[nodiscard]] std::vector<fgo::data_types::State> getReferenceStatesAndClean(){
          return referenceStateBuffer_.get_all_buffer_and_clean();
        };

        [[nodiscard]] bool isGraphInitialized() const {return isStateInited_; };

        /***
         * because all reference sensors should be running as the integrator, we update their timestamps for the above
         * application for e.g. graph initialization
         * @param timestamp
         * @param rostimestamp
         */
        void updateReferenceMeasurementTimestamp(const double& timestamp,
                                                 const rclcpp::Time& rostimestamp)
        {
          referenceSensorTimestampBuffer_.update_buffer(std::make_pair(rostimestamp, timestamp), rostimestamp);
        }

        /***
         * update the system prior states from a reference sensor
         * @param state
         * @param rostimestamp
         */
        void updateReferenceState(const fgo::data_types::State& state,
                                  const rclcpp::Time& rostimestamp)
        {
          referenceStateBuffer_.update_buffer(state, rostimestamp);
        }
    };
}

#endif //ONLINE_FGO_GRAPHBASE_H
