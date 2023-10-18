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

#ifndef ONLINE_FGO_INTERGRATEGNSSTC_H
#define ONLINE_FGO_INTERGRATEGNSSTC_H

#include <boost/optional.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <irt_nav_msgs/msg/sat_label.hpp>
#include <irt_nav_msgs/msg/gnss_labeling.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>

#include "factor/measurement/gnss/PrDrFactor.h"
#include "factor/measurement/gnss/PrFactor.h"
#include "factor/measurement/gnss/DrFactor.h"
#include "factor/measurement/gnss/DDCpFactor.h"
#include "factor/measurement/gnss/DDPrDrFactor.h"
#include "factor/measurement/gnss/TripDCpFactor.h"

#include "factor/measurement/gnss/GPInterpolatedPrFactor.h"
#include "factor/measurement/gnss/GPInterpolatedDrFactor.h"
#include "factor/measurement/gnss/GPInterpolatedPrDrFactor.h"
#include "factor/measurement/gnss/GPInterpolatedTDCpFactor.h"
#include "factor/measurement/gnss/GPInterpolatedTDCPFactorNormalCP.h"
#include "factor/measurement/gnss/GPInterpolatedDDPrDrFactor.h"
#include "factor/measurement/gnss/GPInterpolatedDDCpFactor.h"

#include "utils/GNSSUtils.h"
#include "utils/LambdaAlgorithm.h"
#include "utils/rapidcsv.h"
#include "IntegratorBase.h"

namespace fgo::integrator
{
    using namespace ::utils;

    class GNSSTCIntegrator : public IntegratorBase {
        std::shared_ptr<fgo::models::GPInterpolator> interpolatorI_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolatorJ_;
        IntegratorGNSSTCParamsPtr paramPtr_;

        uint64_t nDDIntAmb_ = 0; //counter for number of intamb
        std::vector<bool> IntAmbFixed_;
        std::vector<double> lastIntAmbVal_; //can be removed, needed if DDCP RTCM AND Aux together used
        std::vector<uint> lastIntAmbSatId_; //can be made static in onGNSS

        /*
         * ROS Utilities
         */
        fgo::buffer::CircularDataBuffer<fgo::data_types::GNSSMeasurement> gnssDataBuffer_;
        rclcpp::Subscription<irt_nav_msgs::msg::GNSSObsPreProcessed>::SharedPtr subGNSS_;
        rclcpp::Subscription<irt_nav_msgs::msg::PVAGeodetic>::SharedPtr subPVA_;

        rclcpp::Publisher<irt_nav_msgs::msg::GNSSLabeling>::SharedPtr gnssLabelingPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::GNSSLabeling>::SharedPtr gnssLabelingPubRaw_;
        fgo::buffer::CircularDataBuffer<irt_nav_msgs::msg::GNSSLabeling> gnssLabelingMsgBuffer_;
        std::atomic_bool zeroVelocity_{};

        std::map<int, std::map<int, bool>> LOSLoopUpTable_;

    private:

        void onIRTPVTMsgCb(const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvtMsg) {
          auto thisPVTTime = rclcpp::Time(pvtMsg->header.stamp.sec, pvtMsg->header.stamp.nanosec, RCL_ROS_TIME);

          static size_t calcZeroVelocityCounter = 1;
          static gtsam::Vector3 sumVelocity = gtsam::Z_3x1;

          auto vel_ned = (gtsam::Vector3() << pvtMsg->vn, pvtMsg->ve, -pvtMsg->vu).finished();

          sumVelocity += vel_ned;

          if(calcZeroVelocityCounter > 4)
          {
            const auto avgVelocity = (gtsam::Vector3() << sumVelocity.x() / calcZeroVelocityCounter,
                sumVelocity.y() / calcZeroVelocityCounter,
                sumVelocity.z() / calcZeroVelocityCounter).finished();
            if(avgVelocity.norm() < paramPtr_->zeroVelocityThreshold)
            {
              RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ << " onIRTPVTMsgCb reported near zero velocity: " << vel_ned);
              zeroVelocity_ = true;
              gnssDataBuffer_.clean();
            }
            else
              zeroVelocity_ = false;
            calcZeroVelocityCounter = 1;
            sumVelocity.setZero();
          }

          calcZeroVelocityCounter ++;
        }

        void onGNSSMsgCb(irt_nav_msgs::msg::GNSSObsPreProcessed::ConstSharedPtr gnssMeasurement);

        //we use this function to trigger the factor builder and disconnect the Callback of GNSS and the FactorBuilder
        //when GNSS data comes it just gets saved and trigger checks if new data is there and then builds factor from it
        fgo::data_types::GNSSMeasurement convertGNSSMsg(irt_nav_msgs::msg::GNSSObsPreProcessed::ConstSharedPtr gnssMsg,
                                                        boost::optional<irt_nav_msgs::msg::GNSSLabeling&> satLabels = boost::none);

        inline void extractGNSSObs(const irt_nav_msgs::msg::GNSSObs &gnssObsMsg,
                                   std::vector<fgo::data_types::GNSSObs> &gnssObsVec,
                                   bool isMainAnt = true,
                                   boost::optional<std::vector<irt_nav_msgs::msg::SatLabel>&> satLabels = boost::none,
                                   boost::optional<std::map<int, bool>&> LOSLoopUp = boost::none) {
          //RCLCPP_INFO(this->get_logger(), "/////////////////////////////////////////////");
          //std::list<int> alreadyin;
          for (size_t i = 0; i < gnssObsMsg.prn.size(); i++) {
            double pr = gnssObsMsg.pseudorange[i];
            //when pr = 0 or nan then do nothing
            if ( pr == 0.0 || (pr != pr) )
              continue;

            fgo::data_types::GNSSObs gnssObs;
            //RCLCPP_INFO_STREAM(this->get_logger(), "PRN: " << gnssObsMsg.prn[i]);
            gnssObs.satId = gnssObsMsg.prn[i];
            //RCLCPP_INFO_STREAM(this->get_logger(), "SatPos: x: " << std::fixed << gnssObsMsg.satelite_pos[i].x << " y: " <<  gnssObsMsg.satelite_pos[i].y << " z: " << gnssObsMsg.satelite_pos[i].z);
            gnssObs.satPos = gtsam::Vector3(gnssObsMsg.satelite_pos[i].x,
                                            gnssObsMsg.satelite_pos[i].y,
                                            gnssObsMsg.satelite_pos[i].z);
            //RCLCPP_INFO_STREAM(this->get_logger(), "SatPos: "<< std::fixed  <<  gnssObs.satPos.transpose());
            //RCLCPP_INFO_STREAM(this->get_logger(), "POSNORM: "<< std::fixed  <<  gnssObs.satPos.norm());
            gnssObs.satVel = gtsam::Vector3(gnssObsMsg.satelite_vec[i].x,
                                            gnssObsMsg.satelite_vec[i].y,
                                            gnssObsMsg.satelite_vec[i].z);
            //RCLCPP_INFO_STREAM(this->get_logger(), "SatVelNorm: " << std::fixed  <<  gnssObs.satVel.norm());
            //RCLCPP_INFO_STREAM(this->get_logger(), "PR: " << std::fixed  <<  gnssObsMsg.pseudorange[i]);
            gnssObs.pr = gnssObsMsg.pseudorange[i];
            //RCLCPP_INFO_STREAM(this->get_logger(), "PR: " << std::fixed  <<  gnssObs.pr);

            if(isMainAnt)
            {
              gnssObs.drVar = gnssObsMsg.deltarange_var[i] * pow(paramPtr_->dopplerrangeVarScaleAntMain, 2);
              if(!paramPtr_->pseudorangeUseRawStd)
              {
                gnssObs.prVar = gnssObsMsg.pseudorange_var[i] * pow(paramPtr_->pseudorangeVarScaleAntMain, 2);
              }
              else
              {
                gnssObs.prVar = pow(gnssObsMsg.pseudorange_var_measured[i], 2) * paramPtr_->pseudorangeVarScaleAntMain;
              }

              if(LOSLoopUp)
              {
                auto LOSIter = LOSLoopUp->find(gnssObs.satId);
                if(LOSIter != LOSLoopUp->end())
                {
                  RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "PRN " << gnssObs.satId << " is LOS? " << LOSIter->second);
                  gnssObs.isLOS = LOSIter->second;
                } else
                  RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "PRN " << gnssObs.satId << " NOT IN LOOPUP Table");

              }

            }
            else
            {
              gnssObs.drVar = gnssObsMsg.deltarange_var[i] * pow(paramPtr_->dopplerrangeVarScaleAntAux, 2);
              if(!paramPtr_->pseudorangeUseRawStd)
              {
                gnssObs.prVar = gnssObsMsg.pseudorange_var[i] * pow(paramPtr_->pseudorangeVarScaleAntAux, 2);
              }
              else
              {
                gnssObs.prVar = pow(gnssObsMsg.pseudorange_var_measured[i], 2) * paramPtr_->pseudorangeVarScaleAntAux;
              }
            }
            gnssObs.prVarRaw = std::pow(gnssObsMsg.pseudorange_var_measured[i], 2);
            gnssObs.dr = gnssObsMsg.deltarange[i];
            //RCLCPP_INFO_STREAM(this->get_logger(), "DR: " << std::fixed  <<  gnssObs.dr);

            gnssObs.cp = gnssObsMsg.carrierphase[i];
            //TODO For Now ELev here
            //gnssObs.el = gnssObsMsg.elevation_angle[i];
            //currentPredState_.mutex.lock_shared();
            gnssObs.el = gnssObsMsg.elevation_angle[i]; //fgo::utils::calcEl(gnssObs.satPos, currentPredState_.state.t());
            gnssObs.cn0 = gnssObsMsg.cn0[i];
            //currentPredState_.mutex.unlock_shared();
            //std::cout << gnssObs.el << " ";
            gnssObs.cpVar = fgo::utils::GNSS::calculateDDVariance(paramPtr_->weightingModel, paramPtr_->carrierphaseStd, paramPtr_->lambdaL1, //TODO @Haoming how to fix?
                                                                  gnssObs.el, gnssObsMsg.carrierphase_var_measured[i], rosNodePtr_->get_name());
            gnssObs.cpVarRaw = std::pow(gnssObsMsg.carrierphase_var_measured[i] * paramPtr_->lambdaL1, 2) * paramPtr_->carrierStdScale;
            gnssObs.locktime = gnssObsMsg.locktime[i];

            if(satLabels)
            {
              irt_nav_msgs::msg::SatLabel label;
              label.prn = gnssObs.satId;
              label.sat_pos = fgo::utils::convertGTVec3ToROS(gnssObs.satPos);
              label.sat_vel = fgo::utils::convertGTVec3ToROS(gnssObs.satVel);
              label.psr = gnssObs.pr;
              label.psr_raw = gnssObsMsg.pseudorange_raw[i];
              label.psr_satclk_corrected = gnssObsMsg.pseudorange_satclk_corrected[i];
              label.psr_dev_preproc = gnssObsMsg.pseudorange_var[i];
              label.psr_dev_measured = gnssObsMsg.pseudorange_var_measured[i];
              label.dr = gnssObs.dr;
              label.dr_raw = gnssObsMsg.deltarange_raw[i];
              label.dr_satclk_corrected = gnssObsMsg.deltarange_satclk_corrected[i];
              label.dr_dev_preproc = gnssObsMsg.deltarange_var[i];
              label.cp = gnssObs.cp;
              label.cp_raw = gnssObsMsg.carrierphase_raw[i];
              label.cp_satclk_corrected = gnssObsMsg.carrierphase_satclk_corrected[i];
              label.cp_dev_measured = gnssObsMsg.carrierphase_var_measured[i];
              label.locktime = gnssObs.locktime;
              label.cn0 = gnssObsMsg.cn0[i];
              label.elevation_angle = gnssObs.el;
              label.azimuth_angle = gnssObsMsg.azimuth_angle[i];
              satLabels->emplace_back(label);
            }
            gnssObsVec.emplace_back(gnssObs);
          }
          //sort( gnssObsVec.begin(), gnssObsVec.end() );
          //gnssObsVec.erase( unique( gnssObsVec.begin(), gnssObsVec.end() ), gnssObsVec.end() );
        }


    public:
        explicit GNSSTCIntegrator() = default;

        void initialize(rclcpp::Node& node, fgo::graph::GraphBase& graphPtr, const std::string& integratorName, bool isPrimarySensor = false) override;

        bool factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                       const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                       const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                       std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                       gtsam::Values& values,
                       fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
                       gtsam::KeyVector& relatedKeys) override;

        bool fetchResult(
            const gtsam::Values &result,
            const gtsam::Marginals &martinals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
            fgo::data_types::State& optState
        ) override;

        ~GNSSTCIntegrator() override = default;

        void dropMeasurementBefore(double timestamp) override{
          std::cout << "GNSSIntegrator: buffer cleaning before: " << std::fixed << timestamp << std::endl;
          gnssDataBuffer_.cleanBeforeTime(timestamp);
        }

        bool checkZeroVelocity() override{
          return zeroVelocity_;
        }

        bool checkHasMeasurements() override
        {
          return gnssDataBuffer_.size() != 0;
        }

        void cleanBuffers() override
        {
          gnssDataBuffer_.clean();
        }
        
    protected:
        inline void addGNSSPrFactor(const gtsam::Key &poseJ,
                                    const gtsam::Key &cbdJ,
                                    const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                    const int &antenna)
        {
          auto levelArm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            levelArm = paramPtr_->transIMUToAnt2;
          }

          for (const auto &obs: obsVector) {
            auto prVar = obs.prVar;
            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector1() << prVar).finished(),
                                                            paramPtr_->robustParameterPRDR);

            graphPtr_->emplace_shared<fgo::factor::PrFactor>(poseJ, cbdJ, obs.pr, obs.satPos, levelArm, noiseModel);
          }
        }

        inline void addGPInterpolatedPrFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                              const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                              const gtsam::Key &cbd_i,
                                              const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                              const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                              const int &antenna) {
          auto level_arm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            level_arm = paramPtr_->transIMUToAnt2;
          }
          for (auto &obs: obsVector) {
            auto prVar = obs.prVar;

            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector1() << prVar).finished(),
                                                            paramPtr_->robustParameterPRDR);

            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedPrFactor>(pose_i, vel_i, omega_i,
                                                                           pose_j, vel_j, omega_j,
                                                                           cbd_i, obs.pr, obs.satPos, obs.satVel,
                                                                           level_arm,
                                                                           noiseModel, interpolator,
                                                                           paramPtr_->AutoDiffGPInterpolatedFactor);

          }
        }

        inline void addGNSSDrFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key cbd_i,
                                    const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                    const gtsam::Vector3 &omegaUnBiased,
                                    const int &antenna)
        {
          auto levelArm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            levelArm = paramPtr_->transIMUToAnt2;
          }
          for (const auto &obs: obsVector) {
            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector1() << obs.drVar).finished(),
                                                            paramPtr_->robustParameterPRDR);

            graphPtr_->emplace_shared<fgo::factor::DrFactor>(pose_i, vel_i, cbd_i, obs.dr, obs.satPos, obs.satVel, levelArm, omegaUnBiased, noiseModel);
          }
        }

        inline void addGPInterpolatedDrFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                              const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                              const gtsam::Key &cbd_i,
                                              const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                              const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                              const int &antenna)
        {
          auto level_arm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            level_arm = paramPtr_->transIMUToAnt2;
          }
          for (auto &obs: obsVector)
          {
            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector1() << obs.drVar).finished(),
                                                            paramPtr_->robustParameterPRDR);
            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedDrFactor>(pose_i, vel_i, omega_i,
                                                                           pose_j, vel_j, omega_j, cbd_i, obs.dr,
                                                                           obs.satPos, obs.satVel, level_arm,
                                                                           noiseModel, interpolator,
                                                                           paramPtr_->AutoDiffGPInterpolatedFactor);
          }
        }

        inline void addGNSSPrDrFactor(const gtsam::Key &poseJ,
                                      const gtsam::Key &velJ,
                                      const gtsam::Key &biasJ,
                                      const gtsam::Key &cbdJ,
                                      const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                      const gtsam::Vector3 &omegaUnbiased,
                                      const int &antenna) {
          auto levelArm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            levelArm = paramPtr_->transIMUToAnt2;
          }

          for (const auto& obs: obsVector) {

            if(!obs.isLOS)
            {
              std::cout << "SKIPPING OBS prn: " << obs.satId << " FOR NLOS Exclusion" << std::endl;
              continue;
            }

            //std::cout << "PositionWe: " << std::fixed << currentPredState_.state.t() << std::endl;
            //std::cout << "satID: " << std::fixed << obs.satId << std::endl;
            //std::cout << "satAltitude + Radius of earth: " << std::fixed << obs.satPos.norm() << std::endl;
            //double distance = gtsam::distance3( fgo::utils::llh2xyz(gtsam::Vector3(pvtMeas.phi, pvtMeas.lambda, pvtMeas.h)), obs.satPos);
            //std::cout << "Distance: " << std::fixed << distance << std::endl;
            //std::cout << "PR: " << std::fixed << obs.pr << std::endl;
            //gtsam::distance3(gtsam::Vector3(4018401,425592,4918225)
            //std::cout << "error: " << std::fixed << distance - obs.pr << std::endl;

            //if (integratorParamPtr_->.usePseudoRange){
            auto prVar = obs.prVar;
            auto drVar = obs.drVar;

            //std::cout << "SatId: " << obs.satId << " Ant: " << unsigned(antenna) << " pr: " << obs.pr << " : " << prVar << " : " << obs.prVarRaw <<" CN0 " << obs.cn0 <<std::endl;
            //std::cout << "SatId: " << obs.satId << " Ant: " << unsigned(antenna) << " pr: " << obs.dr << " : " << drVar << " : " << obs.cpVarRaw << std::endl;

            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector2() << prVar, drVar).finished(),
                                                            paramPtr_->robustParameterPRDR);

            graphPtr_->emplace_shared<fgo::factor::PrDrFactor>(poseJ, velJ, biasJ, cbdJ, obs.pr, obs.dr,
                                                               obs.satPos, obs.satVel, levelArm, omegaUnbiased, noiseModel,
                                                               paramPtr_->AutoDiffNormalFactor);
          }

        }

        inline void
        addGPInterpolatedGNSSPrDrFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                        const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                        const gtsam::Key &cbd_i,
                                        const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                        const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                        const int &antenna) {
          auto level_arm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            level_arm = paramPtr_->transIMUToAnt2;
          }
          for (auto &obs: obsVector)
          {
            if(!obs.isLOS)
            {
              std::cout << "SKIPPING OBS prn: " << obs.satId << " FOR NLOS Exclusion" << std::endl;
              continue;
            }

            auto prVar = obs.prVar;
            auto drVar = obs.drVar;

            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            (gtsam::Vector2() << prVar, drVar).finished(),
                                                            paramPtr_->robustParameterPRDR);
            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedPrDrFactor>(pose_i, vel_i, omega_i, pose_j,
                                                                             vel_j, omega_j, cbd_i, obs.pr, obs.dr,
                                                                             obs.satPos, obs.satVel, level_arm,
                                                                             noiseModel, interpolator,
                                                                             paramPtr_->AutoDiffGPInterpolatedFactor);
          }
        }

        inline void addGNSSDDPrDrFactor(const gtsam::Key &pose_j, const gtsam::Key &vel_j,
                                        const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                        const fgo::data_types::RefSat &refSat,
                                        const gtsam::Vector3 &posBase, const gtsam::Vector3 &omegaUnbiased, const int &antenna) {
          auto level_arm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            level_arm = paramPtr_->transIMUToAnt2;
          }

          for (auto &obs: obsVector) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "satID: " << obs.satId << " PR: " << obs.pr);
            auto prVar = obs.prVar;

            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            gtsam::Vector2(prVar, obs.drVar),
                                                            paramPtr_->robustParameterPRDR);

            graphPtr_->emplace_shared<fgo::factor::DDPrDrFactor>(
                pose_j, vel_j, obs.pr, 0, refSat.refSatPos, refSat.refSatVel, obs.satPos, obs.satVel,
                posBase, level_arm, omegaUnbiased, noiseModel);
          }
        }

        inline void
        addGPInterpolatedDDPrDrFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                      const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                      const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                      const fgo::data_types::RefSat &refSat, const gtsam::Vector3 &posBase,
                                      const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                      const int &antenna) {
          auto level_arm = paramPtr_->transIMUToAnt1;
          if (antenna == 2) {
            level_arm = paramPtr_->transIMUToAnt2;
          }
          for (const auto &obs: obsVector) {
            auto prVar = obs.prVar;

            const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPRDR,
                                                            gtsam::Vector2(prVar, obs.drVar),
                                                            paramPtr_->robustParameterPRDR);
            graphPtr_->emplace_shared<fgo::factor::GPInterpolatedDDPrDrFactor>(
                pose_i, vel_i, omega_i, pose_j, vel_j, omega_j,
                obs.pr, obs.dr, refSat.refSatPos, refSat.refSatVel, obs.satPos, obs.satVel,
                posBase, level_arm, noiseModel, interpolator);
          }

        }

        inline void addGNSSDDCPFactor(const gtsam::Key &pose_j,
                                      const gtsam::Key &amb_j,
                                      const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                      const fgo::data_types::RefSat &refSat,
                                      const gtsam::Vector3 &posBase,
                                      const int &antenna,
                                      fgo::data_types::State& last_opt_state) {

          if (antenna == 1) {
            for (const auto &obs: obsVector) {
              RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "satID: " << obs.satId << " PR: " << obs.pr << " CP: " << obs.cp << " Diff: "
                                                                      << obs.pr - paramPtr_->lambdaL1 * obs.cp);
              //double ddcp_var = calculateDDVariance(integratorParamPtr_->.weightingModel, obs.cpVar, integratorParamPtr_->.lambda, obs.el);

              const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelDDCP,
                                                              gtsam::Vector1(obs.cpVar),
                                                              paramPtr_->robustParameterDDCP);

              graphPtr_->emplace_shared<fgo::factor::DDCarrierPhaseFactor>(pose_j, amb_j, obs.cp, refSat.refSatPos,
                                                                           obs.satPos, posBase, nDDIntAmb_++, paramPtr_->transIMUToAnt1,
                                                                           paramPtr_->lambdaL1, noiseModel);
            }

            //TODO RTCM AND DDANTENNA DOESNT WORK TOGEHTER
            static uint lastRefSatID = 0;
            std::vector<bool> tempFixed;
            gtsam::Vector lastIntAmbVal = last_opt_state.ddIntAmb;
            uint newRefSatID = refSat.refSatSVID;
            if (newRefSatID != lastRefSatID) {
              for (auto &&i: IntAmbFixed_) {
                i = false;
              }
              //calc new Ambiguities MA Uedelhofen p43
              double RefSatDDCP;
              //search the measruement of the new refSat
              for (uint i = 0; i < lastIntAmbSatId_.size(); i++) {
                if (lastIntAmbSatId_[i] == newRefSatID)
                  RefSatDDCP = lastIntAmbVal[i];
              }
              //all not ref measurements get substracted by RefSatDDCP, the refSat measurement is - RefSatDDCP
              for (uint i = 0; i < lastIntAmbSatId_.size(); i++) {
                if (lastIntAmbSatId_[i] == newRefSatID) {
                  lastIntAmbVal[i] = -RefSatDDCP;
                  lastIntAmbSatId_[i] = lastRefSatID;
                  continue;
                }
                lastIntAmbVal[i] -= RefSatDDCP;
              }
            }

            //setup IntAmb calc
            gtsam::Matrix oldCovMatrix = last_opt_state.ddIntAmbVar;
            gtsam::Vector valuesVector = gtsam::Vector::Zero(long(nDDIntAmb_));
            gtsam::Matrix valuesCovMatrix =
                paramPtr_->initCovforIntAmb * gtsam::Matrix::Identity(long(nDDIntAmb_), long(nDDIntAmb_));

            uint ddCounter = 0;
            for (auto &obs: obsVector) {
              bool found = false;
              for (uint i = 0; i < lastIntAmbSatId_.size(); i++) {
                if (obs.satId == lastIntAmbSatId_[i]) {
                  if (obs.locktime > 1) {//&& !obs.cycleSlip
                    valuesCovMatrix(ddCounter, ddCounter) = oldCovMatrix(i, i);
                  } else {
                    IntAmbFixed_[i] = false;
                  }
                  tempFixed.push_back(IntAmbFixed_[i]);
                  valuesVector[ddCounter] = lastIntAmbVal[i];
                  found = true;
                  if (IntAmbFixed_[i])
                    valuesCovMatrix(ddCounter, ddCounter) = 0.001;
                }
              }
              if (!found) {
                valuesVector[ddCounter] = 0.0;
                tempFixed.push_back(false);
              }
              ddCounter++;
            }
            IntAmbFixed_ = tempFixed;
            last_opt_state.ddIntAmb = valuesVector;
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "x:" << valuesVector.transpose() << " nIntAmb:" << nDDIntAmb_);

            if (nState_ > 10 * paramPtr_->ddCPStart && !paramPtr_->useTDCarrierPhase) { //100 for P2, i think 500 for P1, 10 for P2.2
              gtsam::SharedNoiseModel noise_model;
              std::cout << "------------------------------------------------------------------------------------"
                        << std::endl;
              noise_model = gtsam::noiseModel::Gaussian::Covariance(valuesCovMatrix);
              graphPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector>>(amb_j, valuesVector, noise_model);
            }

            lastIntAmbSatId_.clear();
            for (auto &obs: obsVector) {
              lastIntAmbSatId_.push_back(obs.satId);
            }

            lastRefSatID = refSat.refSatSVID;
            //reset lastIntAmbSatID
            nDDIntAmb_ = 0;

          }
          else if (antenna == 2) {
            for (const auto &obs: obsVector) {
              //double ddcp_var = calculateDDVariance(integratorParamPtr_->.weightingModel, obs.cpVar, integratorParamPtr_->.lambda, obs.el);
              //std::cout << "satID: " << std::fixed << obs.satId << std::endl;
              //std::cout << "satAltitude + Radius of earth: " << std::fixed << obs.satPos.norm() << std::endl;
              const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelDDCP,
                                                              gtsam::Vector1(obs.cpVar),
                                                              paramPtr_->robustParameterDDCP);
              graphPtr_->emplace_shared<fgo::factor::DDCarrierPhaseFactor>(pose_j, amb_j, obs.cp, refSat.refSatPos,
                                                                           obs.satPos, nDDIntAmb_++, paramPtr_->transIMUToAnt1,
                                                                           paramPtr_->transIMUToAnt2, paramPtr_->lambdaL1, noiseModel);
            }
          }

        }

        inline void
        addGPInterpolatedDDCPFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                    const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                    const gtsam::Key &amb_j,
                                    const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                    const gtsam::Vector3 &posRefSat, const gtsam::Vector3 &posBase,
                                    const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                    std::list<std::pair<uint32_t, bool>> &notSlippedSatellites,
                                    bool resetList)
        {
          //static std::list<std::pair<uint32_t, bool>> notSlippedSatellites;
          if (resetList) {
            notSlippedSatellites.clear();
            for (auto &obs: obsVector) {
              notSlippedSatellites.emplace_back(obs.satId, false);
            }
          } else {
            //set measurements which slipped to true
            for (const auto &obs: obsVector) {
              if (obs.cycleSlip) {
                for (auto pair: notSlippedSatellites) {
                  if (pair.first == obs.satId) {
                    pair.second = true;
                    break;
                  }
                }
              }
            }
          }

          for (const auto &obs: obsVector) {
            int ddAmbInt = 0;
            for (const auto& nsID: notSlippedSatellites) {
              if (obs.satId == nsID.first && !nsID.second) {
                //double ddcp_var = calculateDDVariance(integratorParamPtr_->.weightingModel, obs.cpVar, integratorParamPtr_->.lambda, obs.el); //TODO
                auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(obs.cpVar));


                graphPtr_->emplace_shared<fgo::factor::GPInterpolatedDDCpFactor>(pose_i, vel_i, omega_i,
                                                                                     pose_j, vel_j, omega_j, amb_j,
                                                                                     obs.cp, posRefSat, posBase,
                                                                                     obs.satPos, ddAmbInt, paramPtr_->transIMUToAnt1,
                                                                                     paramPtr_->lambdaL1, noise_model,
                                                                                     interpolator);
                break;
              }
              ddAmbInt++;
            }
          }

        }

        inline void addGNSSTDCPFactor(const gtsam::Key &pose_i, const gtsam::Key &pose_j,
                                      const gtsam::Key &amb_i, const gtsam::Key &amb_j,
                                      const std::vector<fgo::data_types::GNSSObs> &obsVector_j,
                                      const gtsam::Point3 &posRefSat_j,
                                      const uint &refSatID,
                                      const gtsam::Point3 &posBase, const int &antenna, u_int64_t state) {
          if (antenna == 1)
          { //RTCM
            static uint lastRefSatID = 0;
            static gtsam::Vector3 lastPosRefSat = gtsam::Vector3(0, 0, 0);
            static std::vector<fgo::data_types::CSDataStruct> lastMeasRTCM;
            static u_int64_t lastState = 0;

            int m = 0;
            if (lastRefSatID == refSatID && state - lastState < 11) { //if new ref sat just skip
              for (auto &obs: obsVector_j) {
                uint32_t satID = obs.satId;
                if (obs.locktime > 3 && !obs.cycleSlip) {
                  int n = -1; //search for lastMeasRTCM of same satellite
                  for (uint i = 0; i < lastMeasRTCM.size(); i++) {
                    if (lastMeasRTCM[i].satID == satID)
                      n = int(i);
                  }
                  if (n != -1) {
                    if (paramPtr_->useDDCarrierPhase) {
                      //std::cout << "n: " << n << " m: " << m << std::endl;
                      auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(0.001)); //TODO
                      graphPtr_->emplace_shared<fgo::factor::AmbiguityLockFactor>(N(lastState), amb_j, n, m, noise_model);

                    } else {
                      std::cout << "createTDCP: " << obs.cp - lastMeasRTCM[n].cp << std::endl;
                      //double var_i = calculateDDVariance(integratorParamPtr_->.weightingModel, lastMeasRTCM[n].cpVar,
                      //                                   integratorParamPtr_->.lambda, lastMeasRTCM[n].el);
                      //double var_j = calculateDDVariance(integratorParamPtr_->.weightingModel, obs.cpVar, integratorParamPtr_->.lambda, obs.el);
                      //std::cout << "lastState: " << lastState << " thisState: " << nStates_
                      //<< " lastMeasCP: " << lastMeasRTCM[n].cp << " this cp: " << obs.cp;
                      auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(
                          0.5 * (lastMeasRTCM[n].cpVar + obs.cpVar)));
                      graphPtr_->emplace_shared<fgo::factor::TripleDiffCPFactor>(
                          X(lastState), pose_j, lastMeasRTCM[n].cp, obs.cp,
                          lastPosRefSat, lastMeasRTCM[n].satPos,
                          posRefSat_j, obs.satPos, posBase, paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1, noise_model);
                      //X(lastState)
                    }
                  }
                }
                m++;
              }
            }

            //prepare next one
            int c = 0;
            lastMeasRTCM.resize(obsVector_j.size());
            for (auto &obs: obsVector_j) {
              lastMeasRTCM[c].cp = obs.cp;
              lastMeasRTCM[c].cpVar = obs.cpVar;
              lastMeasRTCM[c].satPos = obs.satPos;
              lastMeasRTCM[c].el = obs.el;
              lastMeasRTCM[c].satID = obs.satId;
              c++;
            }
            lastRefSatID = refSatID;
            lastPosRefSat = posRefSat_j;
            lastState = state;
          } else if (antenna == 2){ //DDDUAL

            static uint lastRefSatID = 0;
            static gtsam::Vector3 lastPosRefSat = gtsam::Vector3(0, 0, 0);
            static std::vector<fgo::data_types::CSDataStruct> lastMeasAux;

            int m = 0;
            if (lastRefSatID == refSatID) { //if new ref sat just skip
              for (auto &obs: obsVector_j) {
                uint32_t satID = obs.satId;
                if (!obs.cycleSlip) {
                  int n = -1; //search for lastMeasRTCM of same satellite
                  for (uint i = 0; i < lastMeasAux.size(); i++) {
                    if (lastMeasAux[i].satID == satID)
                      n = int(i);
                  }
                  if (n != -1) {
                    if (paramPtr_->useDDCarrierPhase) {

                      auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(0.001)); //TODO
                      graphPtr_->emplace_shared<fgo::factor::AmbiguityLockFactor>(amb_i, n, amb_j, m, noise_model);

                    } else {
                      RCLCPP_WARN(rosNodePtr_->get_logger(), "Not implemented TDCP Aux Factor");
                      /*auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(
                              0.5 * (lastMeasRTCM[n].cpVar + obs.cpVar)));
                      graph_->emplace_shared<fgo::factor::TripleDiffCPFactor>(
                              X(lastState), pose_j, lastMeasRTCM[n].cp, obs.cp,
                              lastPosRefSat, lastMeasRTCM[n].satPos,
                              posRefSat_j, obs.satPos, posBase, integratorParamPtr_->.lb, integratorParamPtr_->.lambda, noise_model);*/
                    }
                  }
                }
                m++;
              }
            }

            //prepare next one
            int c = 0;
            lastMeasAux.resize(obsVector_j.size());
            for (auto &obs: obsVector_j) {
              lastMeasAux[c].cp = obs.cp;
              lastMeasAux[c].cpVar = obs.cpVar;
              lastMeasAux[c].satPos = obs.satPos;
              lastMeasAux[c].el = obs.el;
              lastMeasAux[c].satID = obs.satId;
              c++;
            }
            lastRefSatID = refSatID;
            lastPosRefSat = posRefSat_j;
          }


        }

        inline void
        addGPInterpolatedTDNormalCPFactor( const gtsam::Key &point_1, const gtsam::Key &vel_1, const gtsam::Key &omega_1,
                                           const gtsam::Key &cbd,
                                           const gtsam::Key &point_2, const gtsam::Key &vel_2, const gtsam::Key &omega_2,
                                           const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                           uint consecSyncs, double time,
                                           const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
                                           const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j, double dt, double tauI,
                                           StateMeasSyncStatus status,
                                           gtsam::Values& values,
                                           fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
                                           bool lastGNSSInterpolated = false) {
          ///DEFINE AMBIGUITY COMPLETLY NEW DOESNOT BELONG TO STATE BUT TO MEASUREMENT, JUST GET INCREMENTED WITH NEW MEAS
          ///THEREFORE WE CAN IGNORE DRIFT IN MEASUREMENT
          static double halfStateBetweenTime = (double)paramPtr_->optFrequency / (double)paramPtr_->IMUMeasurementFrequency / 2. * 0.8;  // 0.04s
          static std::vector<fgo::data_types::GNSSObs> lastObsVector;
          static gtsam::Key last_amb = N(0);
          static bool notCreatNewCycleSlipFactor = false;
          static bool syncAmbIndexWithState = false;
          static size_t lastlastObsSize = 0;
          static StateMeasSyncStatus lastMeasSyncStatus = status;
          static size_t lastKeyIndex = 0;

          // when no data do nothing
          if (lastObsVector.empty() && obsVector.empty()) {
            return;
          }

          auto obsSize = obsVector.size();
          if(obsSize < paramPtr_->thresholdSatNumToCreateTDCP)
          {
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "onTDCP: no enough satellites to create TDCP: current num. of sat.: " << obsSize);
            RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "onTDCP: adding amb softlock factor for: " << lastObsVector.size() << " sat.");

            for(size_t i = 0; i < lastObsVector.size(); i++)
            {
              auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(1000));
              graphPtr_->emplace_shared<fgo::factor::AmbiguitySoftLockFactor>(last_amb, i, noise_model);
            }
            notCreatNewCycleSlipFactor = false;
            syncAmbIndexWithState = true;
            lastObsVector.clear();
            return;
          }

          auto indexLastAmbKey = gtsam::symbolIndex(last_amb);
          auto indexPointIKey = gtsam::symbolIndex(point_1 );
          auto indexPointJKey = gtsam::symbolIndex(point_2);

          // if the indexPointIKey is larger that the indexLastAmbKey, this means that the tdcp factor was not created
          // in the last measurement epochs, possible reason: too few satellites
          // in this case: we need to synchronize the index of AmbKey

          if(syncAmbIndexWithState)
          {
            if(status == StateMeasSyncStatus::INTERPOLATED)
            {
              //if(tauI < halfStateBetweenTime)
              last_amb = N(indexPointIKey) - 1;
              //else
              // last_amb = N(indexPointIKey);
            }
            else if(status == StateMeasSyncStatus::SYNCHRONIZED_I)
              last_amb = N(indexPointIKey) - 1;
            else if(status == StateMeasSyncStatus::SYNCHRONIZED_J)
              last_amb = N(indexPointIKey) ;
            syncAmbIndexWithState = false;
          }
          else
          {
            if(indexLastAmbKey >= indexPointJKey || (tauI != 0 && status == StateMeasSyncStatus::INTERPOLATED &&
                                                     tauI < halfStateBetweenTime && indexPointIKey <= indexLastAmbKey &&
                                                     lastObsVector.size() >= obsVector.size() &&
                                                     lastlastObsSize != 0 && lastlastObsSize >= obsVector.size()))
            {
              RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "onTDCP: earlier interpolated state, not creating cycleSlipFactor ");
              notCreatNewCycleSlipFactor = true;
            } else {
              notCreatNewCycleSlipFactor = false;
            }
          }

          u_int64_t this_amb = last_amb + 1;
          //check for start of new LOS
          if (lastObsVector.empty() && !obsVector.empty()){
            //set this_amb key
            auto noise_model = gtsam::noiseModel::Diagonal::Variances(10000 * gtsam::Vector::Ones(obsVector.size()));
            graphPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector>>(this_amb, gtsam::Vector::Zero(obsVector.size()),
                                                                    noise_model);
            gtsam::Vector xVec; xVec.resize(obsVector.size());
            values.insert(this_amb, xVec);
            keyTimestampMap[this_amb] = time;
            lastObsVector = obsVector;
            last_amb = this_amb;
            return;
          }

          if(notCreatNewCycleSlipFactor)
          {
            this_amb -= 1;
            last_amb -= 1;
          }
          else
          {
            if (!obsVector.empty()){
              gtsam::Vector initAmb = gtsam::Vector::Zero(obsVector.size());
              keyTimestampMap[this_amb] = time;
              try {values.insert(this_amb, initAmb);} catch (const gtsam::ValuesKeyAlreadyExists& e) {
                RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "The value already exist at: " << gtsam::symbolIndex(this_amb));
              }
            }
          }

          //add estimation for this_amb to the graph
          //add TDCP factor
          int j = 0;
          for (const auto &obs: obsVector) {
            int i = 0; bool found = false;
            for (auto &oldObs : lastObsVector) {
              if (obs.satId == oldObs.satId){
                // RCLCPP_WARN_STREAM(appPtr_->get_logger(), "cpVarRaw" << obs.cpVarRaw << " cpVar: " << obs.cpVar);
                const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelTDCP,
                                                                gtsam::Vector1(0.5 * (obs.cpVarRaw + oldObs.cpVarRaw)),
                                                                paramPtr_->robustParameterTDCP);

                if (consecSyncs >= 1){
                  if(paramPtr_->verbose)
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create Synced TDCP Factor");
                  if (status == StateMeasSyncStatus::SYNCHRONIZED_I){
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create SyncedI TDCP Factor amb between: " << gtsam::symbolIndex(last_amb) << " : " << gtsam::symbolIndex(this_amb));
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create SyncedI TDCP Factor between state: " << gtsam::symbolIndex(point_1 - 1) << " : " << gtsam::symbolIndex(point_1));
                    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Create SyncedI TDCP at sat: " << obs.satId);
                    graphPtr_->emplace_shared<fgo::factor::TDNCPFactor>(point_1 - 1, cbd, last_amb, point_1, this_amb,
                                                                        oldObs.cp, obs.cp, oldObs.satPos, obs.satPos, i, j, dt,
                                                                        paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1,
                                                                        noiseModel);
                    //std::cout << "dt: " << dt << " i: " << i << " j: " << j << " old: " << oldObs.satId << " current: " << obs.satId << std::endl;
                  }

                  if (status == StateMeasSyncStatus::SYNCHRONIZED_J){
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create SyncedJ TDCP Factor amb: " << gtsam::symbolIndex(last_amb) << " : " << gtsam::symbolIndex(this_amb));
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create SyncedJ TDCP Factor between state: " << gtsam::symbolIndex(point_1) << " : " << gtsam::symbolIndex(point_2));
                    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Create SyncedJ TDCP Factor cbd: " << gtsam::symbolIndex(cbd));
                    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Create SyncedJ TDCP at sat: " << obs.satId);
                    graphPtr_->emplace_shared<fgo::factor::TDNCPFactor>(point_1, cbd, last_amb, point_2, this_amb,
                                                                        oldObs.cp, obs.cp, oldObs.satPos, obs.satPos, i, j, dt,
                                                                        paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1,
                                                                        noiseModel);
                  }
                } else {
                  if(paramPtr_->verbose)
                    RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create Unsynced TDCP Factor");
                  RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create Unsynced TDCP Factor amb: " << gtsam::symbolIndex(last_amb) << " : " << gtsam::symbolIndex(this_amb));
                  RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Create Unsynced TDCP Factor between state: " << gtsam::symbolIndex(point_1 - 1) << " : " << gtsam::symbolIndex(point_1) << " : " << gtsam::symbolIndex(point_2));
                  //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Create Unsynced TDCP Factor cbd: " << gtsam::symbolIndex(cbd));
                  //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Create Unsynced TDCP at sat: " << obs.satId);


                  graphPtr_->emplace_shared<fgo::factor::GPInterpolatedTDNCPFactor>(point_1 - 1, vel_1 - 1, omega_1 - 1, point_1, vel_1, omega_1,
                                                                                    cbd, last_amb, point_2, vel_2, omega_2, this_amb,
                                                                                    oldObs.cp, obs.cp, oldObs.satPos, obs.satPos, i, j,
                                                                                    paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1,
                                                                                    noiseModel, interpolator_i, interpolator_j);
                }
                // calculate noise model for CSfactor
                if(!notCreatNewCycleSlipFactor)
                {
                  boost::shared_ptr<gtsam::noiseModel::Diagonal> noise_model2;
                  if (obs.cycleSlip) {
                    RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "TDCP Factor on cycleSlip!");
                    noise_model2 = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(1000));
                  } else {
                    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "TDCP Factor NO cycleSlip!");
                    noise_model2 = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(0.01));
                  }
                  graphPtr_->emplace_shared<fgo::factor::CycleSlipFactor>(last_amb, this_amb, i, j, noise_model2);
                }
                found = true; oldObs.satId = 0;
                break;
              }
              i++;
            }
            if (!found){
              RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "NEW Satellite: " << obs.satId << " at vec: " << j);
              auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(10000));
              graphPtr_->emplace_shared<fgo::factor::AmbiguitySoftLockFactor>(this_amb, j, noise_model);
            }
            j++;
          }
          //check if any ambiguity ends and therefore need an end softconstraint
          j = 0;
          for ( auto& obs : lastObsVector){
            if ( obs.satId != 0){
              RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Old Satellite " << obs.satId << " fell out of TDCP: " << gtsam::symbolIndex(last_amb) << " place in vec: " << j);
              auto noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector1(10000));
              graphPtr_->emplace_shared<fgo::factor::AmbiguitySoftLockFactor>(last_amb, j, noise_model);
            }
            j++;
          }
          //set for next iteration
          lastlastObsSize = lastObsVector.size();
          lastObsVector = obsVector;
          last_amb = this_amb;
        }

        inline void
        addGPInterpolatedTDCPFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                    const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                    const std::vector<fgo::data_types::GNSSObs> &obsVector,
                                    const gtsam::Point3 &posRefSat, const uint &refSatID, const gtsam::Point3 &posBase,
                                    const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
                                    const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j, size_t stateJ) {
          static uint lastRefSatID = -1;
          static gtsam::Vector3 lastPosRefSat = gtsam::Vector3(0, 0, 0);
          static std::vector<fgo::data_types::CSDataStruct> lastMeasurement;
          static size_t lastStateJ = 0;

          if (lastRefSatID == refSatID) {
            for (const auto &obs: obsVector) {
              if (obs.cycleSlip)
                continue;
              uint32_t satID = obs.satId;
              for (const fgo::data_types::CSDataStruct &lastSat: lastMeasurement) {
                if (satID == lastSat.satID) {
                  //NO LOCK FACTOR TOO DIFFERENT
                  //double ddcp_var_i = calculateDDVariance(integratorParamPtr_->.weightingModel, obs.cpVar, integratorParamPtr_->.lambda, obs.el);
                  //double ddcp_var_j = calculateDDVariance(integratorParamPtr_->.weightingModel, lastSat.cpVar, integratorParamPtr_->.lambda, lastSat.el);

                  auto noise_model = gtsam::noiseModel::Diagonal::Variances(
                      gtsam::Vector1(0.5 * (obs.cpVar + lastSat.cpVar)));
                  if (stateJ != lastStateJ) {
                    graphPtr_->emplace_shared<fgo::factor::GPInterpolated3TDCpFactor>(
                        X(stateJ-1), V(stateJ-1), W(stateJ-1), pose_i, vel_i, omega_i, pose_j, vel_j, omega_j,
                        lastSat.cp, obs.cp, lastPosRefSat, lastSat.satPos, posRefSat, obs.satPos, posBase,
                        paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1, noise_model, interpolator_i, interpolator_j);
                  } else {
                      graphPtr_->emplace_shared<fgo::factor::GPInterpolatedTDCpFactor>(
                          pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, lastSat.cp, obs.cp,
                          lastPosRefSat, lastSat.satPos, posRefSat, obs.satPos, posBase,
                          paramPtr_->transIMUToAnt1, paramPtr_->lambdaL1, noise_model, interpolator_i, interpolator_j);
                  }
                }
              }
            }
          }

          //prepare next one measurement
          int c = 0;
          lastMeasurement.resize(obsVector.size());
          for (auto &obs: obsVector) {
            lastMeasurement[c].cp = obs.cp;
            lastMeasurement[c].cpVar = obs.cpVar;
            lastMeasurement[c].satPos = obs.satPos;
            lastMeasurement[c].el = obs.el;
            lastMeasurement[c].satID = obs.satId;
            c++;
          }
          lastStateJ = stateJ;
          lastRefSatID = refSatID;
          lastPosRefSat = posRefSat;
        }
        
        
    };
}

#endif //ONLINE_FGO_INTERGRATEGNSSTC_H
