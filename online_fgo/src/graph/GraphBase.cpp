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

#include "graph/GraphBase.h"
#include "integrator/IntegratorBase.h"
#include "gnss_fgo/GNSSFGOLocalizationBase.h"
#include "factor/measurement/gnss/PrDrFactor.h"
#include "factor/measurement/gnss/GPInterpolatedPrDrFactor.h"
#include "factor/measurement/gnss/GPSFactor.h"
#include "factor/measurement/gnss/GPInterpolatedGPSFactor.h"

#include "factor/measurement/odometry/NavAttitudeFactor.h"
#include "factor/measurement/odometry/GPInterpolatedNavAttitudeFactor.h"


namespace fgo::graph
{
    using namespace std::chrono_literals;

    GraphBase::GraphBase(gnss_fgo::GNSSFGOLocalizationBase& node)  :
            appPtr_(&node),
            integratorLoader_("online_fgo", "fgo::integrator::IntegratorBase")
    {
      RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphBase initializing! --------------------- ");
      graphBaseParamPtr_ = std::make_shared<GraphParamBase>(node.getParamPtr());
      referenceSensorTimestampBuffer_.resize_buffer(50);
      referenceStateBuffer_.resize_buffer(50);
      fgoOptStateBuffer_.resize_buffer(50);
      accBuffer_.resize_buffer(20000);
      currentPredictedBuffer_.resize_buffer(50);

      RosParameter<bool> publishResiduals("GNSSFGO.Graph.publishResiduals", true, node);
      graphBaseParamPtr_->publishResiduals = publishResiduals.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "publishResiduals:" << graphBaseParamPtr_->publishResiduals);

      RosParameter<bool> onlyLastResiduals("GNSSFGO.Graph.onlyLastResiduals", true, node);
      graphBaseParamPtr_->onlyLastResiduals = onlyLastResiduals.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "onlyLastResiduals:" << graphBaseParamPtr_->onlyLastResiduals);

      if(graphBaseParamPtr_->publishResiduals)
      {
        factorBuffer_.resize_buffer(100);
        graphResultBuffer_.resize_buffer(100);
        pubResiduals_ = appPtr_->create_publisher<irt_nav_msgs::msg::FactorResiduals>("residuals", rclcpp::SystemDefaultsQoS());
        pubResidualsThread_ = std::make_shared<std::thread>(
            [this]()->void {
              while(rclcpp::ok())
              {
                const auto factorBuffer = factorBuffer_.get_all_time_buffer_pair();
                const auto results = graphResultBuffer_.get_all_time_buffer_pair_and_clean();
                if(results.empty())
                {
                  rclcpp::sleep_for(100ms);
                  continue;
                }

                for(size_t i = 0; i < results.size(); i++)
                {

                  if(graphBaseParamPtr_->onlyLastResiduals && i < results.size() - 1)
                    continue;

                  const auto& timestamp = results[i].first;
                  const auto& result = results[i].second;

                  for(const auto& factorVectorTimePair : factorBuffer)
                  {
                    if(factorVectorTimePair.first < timestamp)
                      continue;
                    irt_nav_msgs::msg::FactorResiduals resMsg;
                    resMsg.header.stamp = timestamp;
                    const auto& factorVec = factorVectorTimePair.second;
                    std::for_each(factorVec.begin(), factorVec.end(), [&](const gtsam::NonlinearFactor::shared_ptr &factor) -> void
                    {
                        bool factorImplemented = true;
                        gtsam::NoiseModelFactor::shared_ptr thisFactorCasted;
                        switch (factor->getTypeID()) {
                          case fgo::factor::FactorTypeIDs::GPWNOAMotionPrior:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOAPriorPose3>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::GPWNOJMotionPrior:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOJPriorPose3>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::CombinedIMU:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::BetweenPose:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::ReceiverClock:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::ConstDriftFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::PRDR:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::PrDrFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::GPPRDR:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedPrDrFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::GPS:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPSFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::GPGPS:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedGPSFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::NavAttitude:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::NavAttitudeFactor>(factor);
                            break;
                          }
                          case fgo::factor::FactorTypeIDs::GPNavAttitude:
                          {
                            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedNavAttitudeFactor>(factor);
                            break;
                          }
                          default:
                          {
                            RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "onResidualPublishing factor " << factor->getName() << " not implemented");
                            factorImplemented = false;
                            break;
                          }}


                          if(thisFactorCasted && factorImplemented)
                          {
                            irt_nav_msgs::msg::FactorResidual res;
                            res.name = thisFactorCasted->getName();
                            gtsam::Values values;
                            const gtsam::KeyVector keys = thisFactorCasted->keys();
                            res.keys.resize(keys.size());
                            std::copy(keys.begin(), keys.end(), res.keys.begin());
                            bool allKeyValued = true;
                            std::for_each(keys.cbegin(), keys.cend(), [&](size_t key) -> void {
                              if(result.exists(key))
                                values.insert(key, result.at(key));
                              else
                                allKeyValued = false;
                            });
                            const auto unwhitenedError = thisFactorCasted->unwhitenedError(values);
                            const auto whitenedError = thisFactorCasted->whitenedError(values);
                            res.unwhitened_error.resize(unwhitenedError.size());
                            gtsam::Vector::Map(&res.unwhitened_error[0], unwhitenedError.size()) = unwhitenedError;

                            res.whitened_error.resize(whitenedError.size());
                            gtsam::Vector::Map(&res.whitened_error[0], whitenedError.size()) = whitenedError;

                            res.noise_model_weight = thisFactorCasted->weight(values);
                            res.loss_error = thisFactorCasted->error(values);
                            resMsg.residuals.emplace_back(res);
                          }

                    });
                    pubResiduals_->publish(resMsg);
                  }
                 // const auto gpFactor = boost::dynamic_pointer_cast<fgo::factor::GPWNOJPriorPose3>();



                }
              }
            }
          );
      }

      RosParameter<double> smootherLag("GNSSFGO.Optimizer.smootherLag", 0.1, node);
      graphBaseParamPtr_->smootherLag = smootherLag.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "smootherLag:" << graphBaseParamPtr_->smootherLag);

      RosParameter<std::string> smootherType("GNSSFGO.Optimizer.smootherType", "undefined", node);
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "smootherType:" << smootherType.value());

      if (smootherType.value() == "BatchFixedLag") {
        graphBaseParamPtr_->smootherType = SmootherType::BatchFixedLag;
        //not all param
        gtsam::LevenbergMarquardtParams lmp = gtsam::LevenbergMarquardtParams::LegacyDefaults();
        //The initial Levenberg-Marquardt damping term (default: 1e-5)
        RosParameter<double> lambdaInitial("Optimizer.LM.lambdaInitial", 1e-5, node);
        lmp.lambdaInitial = lambdaInitial.value();
        //The amount by which to multiply or divide lambda when adjusting lambda (default: 10.0)
        RosParameter<double> lambdaFactor("Optimizer.LM.lambdaFactor", 10.0, node);
        lmp.lambdaFactor = lambdaFactor.value();
        //The maximum lambda to try before assuming the optimization has failed (default: 1e5)
        RosParameter<double> lambdaUpperBound("Optimizer.LM.lambdaUpperBound", 1e5, node);
        lmp.lambdaUpperBound = lambdaUpperBound.value();
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "lmp.lambdaUpperBound: " << lmp.lambdaUpperBound);
        //The minimum lambda used in LM (default: 0)
        RosParameter<double> lambdaLowerBound("Optimizer.LM.lambdaLowerBound", 0, node);
        lmp.lambdaLowerBound = lambdaLowerBound.value();
        //if true, use diagonal of Hessian
        RosParameter<bool> diagonalDamping("Optimizer.LM.diagonalDamping", false, node);
        lmp.diagonalDamping = diagonalDamping.value();
        //if true applies constant increase (or decrease) to lambda according to lambdaFactor
        RosParameter<bool> useFixedLambdaFactor("Optimizer.LM.useFixedLambdaFactor", true, node);
        lmp.useFixedLambdaFactor = useFixedLambdaFactor.value();
        //when using diagonal damping saturates the minimum diagonal entries (default: 1e-6)
        RosParameter<double> minDiagonal("Optimizer.LM.minDiagonal", 1e-6, node);
        lmp.minDiagonal = minDiagonal.value();
        //when using diagonal damping saturates the maximum diagonal entries (default: 1e32)
        RosParameter<double> maxDiagonal("Optimizer.LM.maxDiagonal", 1e32, node);
        lmp.maxDiagonal = maxDiagonal.value();
        this->initSolver(lmp);
      } else {
        if (smootherType.value()  == "IncrementalFixedLag") {
          graphBaseParamPtr_->smootherType = SmootherType::ISAM2FixedLag;
          gtsam::ISAM2Params isam2Params;
          //Optimization parameters, this both selects the nonlinear optimization method and specifies its parameters, either ISAM2GaussNewtonParams or ISAM2DoglegParams.
          RosParameter<std::string> optimizationParams("Optimizer.ISAM2.optimizationParams", "GN", node);
          if (optimizationParams.value() == "DOGLEG") {
            isam2Params.optimizationParams = gtsam::ISAM2DoglegParams();
          } else {
            isam2Params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
          }
          //Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1).
          RosParameter<double> relinearizeThreshold("Optimizer.ISAM2.relinearizeThreshold", 0.1, node);
          isam2Params.relinearizeThreshold = relinearizeThreshold.value();//0.1 or 0.01
          //Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
          RosParameter<int> relinearizeSkip("Optimizer.ISAM2.relinearizeSkip", 10, node);
          isam2Params.relinearizeSkip = relinearizeSkip.value(); //10
          //Specifies whether to use QR or CHOESKY numerical factorization (default: CHOLESKY).
          RosParameter<std::string> factorization("Optimizer.ISAM2.factorization", "CHOLESKY", node);
          if (factorization.value() == "QR") {
            isam2Params.factorization = gtsam::ISAM2Params::QR;
          } else {
            isam2Params.factorization = gtsam::ISAM2Params::CHOLESKY;
          }
          //Whether to compute and return ISAM2Result::detailedResults, this can increase running time (default: false)
          RosParameter<bool> enableDetailedResults("Optimizer.ISAM2.enableDetailedResults", false, node);
          isam2Params.enableDetailedResults = enableDetailedResults.value();
          //When you will be removing many factor when using ISAM2 as a fixed-lag smoother, enable this option to add factor in the first available factor slots,
          RosParameter<bool> findUnusedFactorSlots("Optimizer.ISAM2.findUnusedFactorSlots", false, node);
          isam2Params.findUnusedFactorSlots = findUnusedFactorSlots.value();
          this->initSolver(isam2Params);
        } else {
          RCLCPP_ERROR(appPtr_->get_logger(), "Smoothertype undefined. Please choose Smoothertype.");
          return;
        }
      }

      RosParameter<double> testDt("GNSSFGO.Optimizer.testDt", 0.1, node);

      if(graphBaseParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ)
      {
        //RosParameter<double> QcGPMotionPrior("GNSSFGO.Optimizer.QcGPWNOJMotionPrior", 1000., node);
        //graphBaseParamPtr_->QcGPMotionPrior = QcGPMotionPrior.value();
        //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOJMotionPrior:" << graphBaseParamPtr_->QcGPMotionPrior);

        RosParameter<std::vector<double>> QcGPMotionPriorFull("GNSSFGO.Optimizer.QcGPWNOJMotionPriorFull", node);
        graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFull.value().data());
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOJMotionPrior:" << graphBaseParamPtr_->QcGPMotionPriorFull);

        RosParameter<std::vector<double>> QcFull("GNSSFGO.Optimizer.QcGPWNOJInterpolatorFull", node);
        graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFull.value().data());
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOJInterpolator:" << graphBaseParamPtr_->QcGPInterpolatorFull);
        gtsam::SharedNoiseModel  qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);
        const auto qc = gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ3<6>(fgo::utils::getQc(qcModel), testDt.value()));

        std::cout << "#################################################################" << std::endl;

        std::cout << fgo::utils::getQc(qcModel) << std::endl;
        std::cout << "###" << std::endl;
        qc->print("Q3:");

        std::cout << "#################################################################" << std::endl;

      }
      else
      {
        RosParameter<std::vector<double>> QcGPMotionPriorFull("GNSSFGO.Optimizer.QcGPWNOAMotionPriorFull", node);
        graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFull.value().data());
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOAMotionPrior:" << graphBaseParamPtr_->QcGPMotionPriorFull);

        RosParameter<std::vector<double>> QcFull("GNSSFGO.Optimizer.QcGPWNOAInterpolatorFull", node);
        graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFull.value().data());
        RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOAInterpolator:" << graphBaseParamPtr_->QcGPInterpolatorFull);
        gtsam::SharedNoiseModel  qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);
        auto qc = gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(qcModel), testDt.value()));

        std::cout << "#################################################################" << std::endl;

        std::cout << fgo::utils::getQc(qcModel) << std::endl;
        std::cout << "###" << std::endl;
        qc->print("Q:");


        std::cout << "#################################################################" << std::endl;
      }

      RosParameter<bool> AutoDiffNormalFactor("GNSSFGO.Graph.AutoDiffNormalFactor", node);
      graphBaseParamPtr_->AutoDiffNormalFactor = AutoDiffNormalFactor.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "AutoDiffNormalFactor: " << (graphBaseParamPtr_->AutoDiffNormalFactor ? "true" : "false"));

      RosParameter<bool> AutoDiffGPInterpolatedFactor("GNSSFGO.Graph.AutoDiffGPInterpolatedFactor", node);
      graphBaseParamPtr_->AutoDiffGPInterpolatedFactor = AutoDiffGPInterpolatedFactor.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "AutoDiffGPInterpolatedFactor: " << (graphBaseParamPtr_->AutoDiffGPInterpolatedFactor ? "true" : "false"));

      RosParameter<bool> AutoDiffGPMotionPriorFactor("GNSSFGO.Graph.AutoDiffGPMotionPriorFactor", node);
      graphBaseParamPtr_->AutoDiffGPMotionPriorFactor = AutoDiffGPMotionPriorFactor.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "AutoDiffGPMotionPriorFactor: " << (graphBaseParamPtr_->AutoDiffGPMotionPriorFactor ? "true" : "false"));

      RosParameter<bool> GPInterpolatedFactorCalcJacobian("GNSSFGO.Graph.GPInterpolatedFactorCalcJacobian", node);
      graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian = GPInterpolatedFactorCalcJacobian.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GPInterpolatedFactorCalcJacobian: " << (graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian ? "true" : "false"));

      RosParameter<bool> useEstimatedVarianceAfterInit("GNSSFGO.Graph.useEstimatedVarianceAfterInit", node);
      graphBaseParamPtr_->useEstimatedVarianceAfterInit = useEstimatedVarianceAfterInit.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "useEstimatedVarianceAfterInit: " << (graphBaseParamPtr_->useEstimatedVarianceAfterInit ? "true" : "false"));

      RosParameter<bool> useConstDriftFactor("GNSSFGO.Graph.useConstDriftFactor", node);
      graphBaseParamPtr_->useConstDriftFactor = useConstDriftFactor.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "useConstDriftFactor: " << (graphBaseParamPtr_->useConstDriftFactor ? "true" : "false"));

      RosParameter<std::string> noiseModelClockFactor("GNSSFGO.Graph.noiseModelClockFactor", node);
      integrator::IntegratorBase::setNoiseModelFromParam(noiseModelClockFactor.value(),
                                                          graphBaseParamPtr_->noiseModelClockFactor,
                                                          "ClockFactor");
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "clockDriftNoiseModel: " << noiseModelClockFactor.value());

      RosParameter<double> robustParamClockFactor("GNSSFGO.Graph.robustParamClockFactor", node);
      graphBaseParamPtr_->robustParamClockFactor = robustParamClockFactor.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "robustParamClockFactor: " << graphBaseParamPtr_->robustParamClockFactor);

      RosParameter<double> angularRateStd("GNSSFGO.Graph.angularRateStd", 1, node);
      graphBaseParamPtr_->angularRateStd = angularRateStd.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "angularRateStd: " << graphBaseParamPtr_->angularRateStd);

      RosParameter<double> constDriftStd("GNSSFGO.Graph.constDriftStd", 1, node);
      graphBaseParamPtr_->constDriftStd = constDriftStd.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constDriftStd: " << graphBaseParamPtr_->constDriftStd);

      RosParameter<double> constBiasStd("GNSSFGO.Graph.constBiasStd", 1, node);
      graphBaseParamPtr_->constBiasStd = constBiasStd.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constBiasStd: " << graphBaseParamPtr_->constBiasStd);

      RosParameter<double> motionModelStd("GNSSFGO.Graph.motionModelStd", 1, node);
      graphBaseParamPtr_->motionModelStd = motionModelStd.value();

      RosParameter<double> magnetometerStd("GNSSFGO.Graph.magnetometerStd", 1, node);
      graphBaseParamPtr_->magnetometerStd = gtsam::Vector3(magnetometerStd.value(), magnetometerStd.value(), magnetometerStd.value());

      RosParameter<double> StateMeasSyncUpperBound("GNSSFGO.Graph.StateMeasSyncUpperBound", 0.02, node);
      graphBaseParamPtr_->StateMeasSyncUpperBound = StateMeasSyncUpperBound.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "StateMeasSyncUpperBound: " << graphBaseParamPtr_->StateMeasSyncUpperBound);
      RosParameter<double> StateMeasSyncLowerBound("GNSSFGO.Graph.StateMeasSyncLowerBound", -0.02, node);
      graphBaseParamPtr_->StateMeasSyncLowerBound = StateMeasSyncLowerBound.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "StateMeasSyncLowerBound: " << graphBaseParamPtr_->StateMeasSyncLowerBound);

      RosParameter<bool> NoOptimizationWhileNoMeasurement("GNSSFGO.Graph.NoOptimizationWhileNoMeasurement", node);
      graphBaseParamPtr_->NoOptimizationWhileNoMeasurement = NoOptimizationWhileNoMeasurement.value();
      RosParameter<bool> NoOptimizationNearZeroVelocity("GNSSFGO.Graph.NoOptimizationNearZeroVelocity", node);
      graphBaseParamPtr_->NoOptimizationNearZeroVelocity = NoOptimizationNearZeroVelocity.value();
      RosParameter<double> VoteNearZeroVelocity("GNSSFGO.Graph.VoteNearZeroVelocity", 0.5, node);
      graphBaseParamPtr_->VoteNearZeroVelocity = VoteNearZeroVelocity.value();
      RosParameter<int> NoOptimizationAfterStates("GNSSFGO.Graph.NoOptimizationAfterStates", node);
      graphBaseParamPtr_->NoOptimizationAfterStates = NoOptimizationAfterStates.value();

      RosParameter<std::vector<std::string>> integratorNames("GNSSFGO.Integrators", node);

      bool primarySensorSet = false;
      for(const auto& integratorName : integratorNames.value())
      {
        RosParameter<std::string> thisIntegratorPlugin("GNSSFGO." + integratorName + ".integratorPlugin", "" ,node);
        const auto plugin = thisIntegratorPlugin.value();
        if(integratorLoader_.isClassAvailable(plugin))
        {
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GraphTimeCentric: initializing integrator: " << integratorName << " with plugin: " << plugin);
          auto integrator = integratorLoader_.createSharedInstance(plugin);
          integrator->initialize(node, *this, integratorName);
          integratorMap_.insert(std::make_pair(integratorName, integrator));
          RosParameter<bool> isPrimarySensor("GNSSFGO." + integratorName + ".isPrimarySensor", false, node);
          if(isPrimarySensor.value())
          {
              if(primarySensorSet)
                  RCLCPP_WARN_STREAM(appPtr_->get_logger(), "GraphTimeCentric: primary sensor already set, reset to: " << integratorName << " with plugin: " << plugin);
              primarySensor_ = integrator;
              primarySensorSet = true;
          }
        }
        else
        {
          RCLCPP_WARN_STREAM(appPtr_->get_logger(), "GraphTimeCentric: cannot find integrator: " << integratorName << " with plugin: " << plugin);
        }
      }

      RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphBase initialized! --------------------- ");
    };

    void GraphBase::initGraph(const data_types::State &initState, const double &initTimestamp,
                              boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams)  {
      isStateInited_ = false;
      preIntegratorParams_ = preIntegratorParams;

      this->resetGraph();

      auto priorPoseFactor = gtsam::PriorFactor<gtsam::Pose3>(X(0), initState.state.pose(), initState.poseVar);
      priorPoseFactor.setTypeID(999);
      priorPoseFactor.setName("priorPoseFactor");
      this->push_back(priorPoseFactor);
      auto priorVelFactor = gtsam::PriorFactor<gtsam::Vector3>(V(0), initState.state.v(), initState.velVar);
      priorVelFactor.setTypeID(999);
      priorVelFactor.setName("priorVelFactor");
      this->push_back(priorVelFactor);
      auto priorBiasFactor = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), initState.imuBias, initState.imuBiasVar);
      priorBiasFactor.setTypeID(999);
      priorBiasFactor.setName("priorBiasFactor");
      this->push_back(priorBiasFactor);

      values_.insert(X(0), initState.state.pose());
      values_.insert(V(0), initState.state.v());
      values_.insert(B(0), initState.imuBias);

      keyTimestampMap_[X(0)] =
      keyTimestampMap_[V(0)] =
      keyTimestampMap_[B(0)] = initTimestamp;

      if(graphBaseParamPtr_->useConstDriftFactor) {
        auto priorClockErrorFactor =gtsam::PriorFactor<gtsam::Vector2>(C(0), initState.cbd, initState.cbdVar);
        priorClockErrorFactor.setTypeID(999);
        priorClockErrorFactor.setName("priorClockErrorFactor");
        this->push_back(priorClockErrorFactor);
        values_.insert(C(0), initState.cbd);
        keyTimestampMap_[C(0)] = initTimestamp;
      }

      currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, initTimestamp));

      //if(LIOParams_.useTDCarrierPhase)
      //{
      //auto noise_model = gtsam::noiseModel::Diagonal::Variances(1000 * gtsam::Vector::Ones(40));
      //this->emplace_shared<gtsam::PriorFactor<gtsam::Vector>>(N(0), gtsam::Vector::Zero(40),
      //                                                        noise_model);
      //gtsam::Vector xVec; xVec.resize(40);
      //values_.insert(N(0), gtsam::Vector::Zero(40));
      //keyTimestampMap_[N(0)] = initTimestamp;
      // }

      if(graphBaseParamPtr_->useGPPriorFactor || graphBaseParamPtr_->useGPInterpolatedFactor)
      {
        auto priorOmegaFactor =gtsam::PriorFactor<gtsam::Vector3>(W(0), initState.omega, initState.omegaVar);
        priorOmegaFactor.setTypeID(999);
        priorOmegaFactor.setName("priorOmegaFactor");
        this->push_back(priorOmegaFactor);
        values_.insert(W(0), initState.omega);
        keyTimestampMap_[W(0)] = initTimestamp;
      }

      if(!graphBaseParamPtr_->useGPPriorFactor  && graphBaseParamPtr_->useGPInterpolatedFactor)
      {
        fgo::data_types::IMUMeasurement tmpIMU;
        tmpIMU.gyro = initState.omega;
       // this->addAngularFactor(W(0), B(0), tmpIMU);
      }

      for(const auto& integrator : integratorMap_)
      {
        integrator.second->dropMeasurementBefore(initTimestamp);
      }

      isStateInited_ = true;
      RCLCPP_WARN(appPtr_->get_logger(), "------------- GraphBase: Init.FGO Done! -------------");
    }

    void GraphBase::notifyOptimization() {
        appPtr_->notifyOptimization();
    }
}
