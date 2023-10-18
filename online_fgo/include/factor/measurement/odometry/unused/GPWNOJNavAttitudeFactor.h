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
// Created by haoming on 19.02.23.
//

#ifndef ONLINE_FGO_GPWNOJNAVATTITUDEFACTOR_H
#define ONLINE_FGO_GPWNOJNAVATTITUDEFACTOR_H

#pragma once

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"


namespace fgo::factor {

    class GPWNOJNavAttitudeFactor : public fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                                                  gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6>
    {
    protected:
        gtsam::Rot3 rotMeasured_;
        MeasurementFrame measuredRotFrame_ = MeasurementFrame::NED;
        AttitudeType type_ = AttitudeType::RPY;
        double tau_{};
        bool useAutoDiff_ = false;

        typedef GPWNOJNavAttitudeFactor This;
        typedef fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                       gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

        GPBase GPBase_;
    public:
        GPWNOJNavAttitudeFactor() = default;

        GPWNOJNavAttitudeFactor(gtsam::Key poseKeyI, gtsam::Key velKeyI, gtsam::Key omegaKeyI, gtsam::Key accKeyI,
                                gtsam::Key poseKeyJ, gtsam::Key velKeyJ, gtsam::Key omegaKeyJ, gtsam::Key accKeyJ,
                                const gtsam::Rot3 &rotMeasured, MeasurementFrame rotFrame, AttitudeType type,
                                const gtsam::SharedNoiseModel &model,
                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                bool useAutoDiff = true) :
            Base(model, poseKeyI, velKeyI, omegaKeyI, accKeyI, poseKeyJ, velKeyJ, omegaKeyJ, accKeyJ), rotMeasured_(rotMeasured), measuredRotFrame_(rotFrame),
            type_(type), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPBase_(interpolator)
        {}

        ~GPWNOJNavAttitudeFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** factor error */
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI, const gtsam::Vector6 &accI,
                                                  const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ, const gtsam::Vector6 &accJ,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H7 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
          if (useAutoDiff_) {
            if (H1) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), poseI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), poseI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), poseI, 1e-5);
                  break;
                }
              }
            }
            if (H2) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), velI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), velI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, accI, poseJ, velJ,
                                  omegaJ, accJ), velI, 1e-5);
                  break;
                }
              }
            }
            if (H3) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, accI, poseJ, velJ,
                                  omegaJ, accJ), omegaI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, accI, poseJ, velJ,
                                  omegaJ, accJ), omegaI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, accI, poseJ, velJ,
                                  omegaJ, accJ), omegaI, 1e-5);
                  break;
                }
              }
            }
            if (H4) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, poseJ, velJ,
                                  omegaJ, accJ), accI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, poseJ, velJ,
                                  omegaJ, accJ), accI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, poseJ, velJ,
                                  omegaJ, accJ), accI, 1e-5);
                  break;
                }
              }
            }
            if (H5) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, boost::placeholders::_1, velJ,
                                  omegaJ, accJ), poseJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, boost::placeholders::_1, velJ,
                                  omegaJ, accJ), poseJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, boost::placeholders::_1, velJ,
                                  omegaJ, accJ), poseJ, 1e-5);
                  break;
                }
              }
            }
            if (H6) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, boost::placeholders::_1,
                                  omegaJ, accJ), velJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, boost::placeholders::_1,
                                  omegaJ, accJ), velJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, boost::placeholders::_1,
                                  omegaJ, accJ), velJ, 1e-5);
                  break;
                }
              }
            }
            if (H7) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H7 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ,
                                  boost::placeholders::_1, accJ), omegaJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H7 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ,
                                  boost::placeholders::_1, accJ), omegaJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H7 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ,
                                  boost::placeholders::_1, accJ), omegaJ, 1e-5);
                  break;
                }
              }
            }
            if (H8) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H8 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, omegaJ,
                                  boost::placeholders::_1), accJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H8 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, omegaJ,
                                  boost::placeholders::_1), accJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H8 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector6>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, omegaJ,
                                  boost::placeholders::_1), accJ, 1e-5);
                  break;
                }
              }
            }
            return evaluateError_(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
          } else {

            gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P;

            gtsam::Pose3 pose;
            if (H1 || H2 || H3 || H4 || H5 || H6 || H7 || H8) {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ,
                                              Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P);
            }
            else {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
            }

            gtsam::Matrix Hpose, Hrot1, Hrot2, D_dR_R, D_xi_R;
            gtsam::Matrix36 H14_;

            /*
             *   Shape
             *   H1: (1-3) * 6
             *   H2: (1-3) * 3
             *   H3: (1-3) * 3
             *   H4: (1-3) * 6
             *   H5: (1-3) * 6
             *   H6: (1-3) * 3
             *   H7: (1-3) * 3
             *   H8: (1-3) * 6
             */

            const auto &pos = pose.translation(Hpose);
            const auto &rot = pose.rotation(Hrot1);

            gtsam::Matrix HH1, HH2, HH3, HH4, HH5, HH6, HH7, HH8;

            if(H4) HH4 = Hint4_P.block<3, 6>(0, 0);
            if(H8) HH8 = Hint8_P.block<3, 6>(0, 0);

            if(H2) HH2 = Hint2_P.block<3, 3>(0, 0);  // Shape Hint2_p 6 * 3
            if(H3) HH3 = Hint3_P.block<3, 3>(0, 0);  // Shape Hint2_p 6 * 3;
            if(H6) HH6 = Hint6_P.block<3, 3>(0, 0);  // Shape Hint2_p 6 * 3;
            if(H7) HH7 = Hint7_P.block<3, 3>(0, 0);  // Shape Hint2_p 6 * 3;

            gtsam::Vector3 error;
            switch (measuredRotFrame_) {
              case MeasurementFrame::NED: {
                const auto &nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, (H1 || H5) ? &Hrot2 : nullptr);
                error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_, (H1 || H5) ? &D_dR_R : nullptr), (H1 || H5) ? &D_xi_R : nullptr);
                if(H1 || H5)
                {
                  H14_ << D_xi_R * D_dR_R * Hrot2 * Hrot1 + Hpose; // 36 + 36
                }
                if(H1) HH1 = H14_ * Hint1_P;  // 36 * 66
                if(H5) HH5 = H14_ * Hint5_P;
                break;
              }
              case MeasurementFrame::ENU: {
                const auto &nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, (H1 || H5) ? &Hrot2 : nullptr);
                error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_, (H1 || H5) ? &D_dR_R : nullptr), (H1 || H5) ? &D_xi_R : nullptr);
                if(H1 || H5)
                {
                  H14_ << D_xi_R * D_dR_R * Hrot2 * Hrot1 + Hpose; // 36 + 36
                }
                if(H1) HH1 = H14_ * Hint1_P;  // 36 * 66
                if(H5) HH5 = H14_ * Hint5_P;
                break;
              }
              default: {
                error = gtsam::Rot3::Logmap(rot.between(rotMeasured_, H1 ? &D_dR_R : nullptr), H1 ? &D_xi_R : nullptr);
                if(H1 || H5)
                {
                  H14_ << D_xi_R * D_dR_R * Hrot1 + Hpose; // 36 + 36
                }
                if(H1) HH1 = H14_ * Hint1_P;  // 36 * 66
                if(H5) HH5 = H14_ * Hint5_P;
                break;
              }
            }

            switch (type_) {
              case AttitudeType::RPY:
              {
                return error;
              }
              case AttitudeType::YAWPITCH:
              {
                if(H1) *H1 = HH1.block<2, 6>(1, 0);
                if(H2) *H2 = HH2.block<2, 3>(1, 0);
                if(H3) *H3 = HH3.block<2, 3>(1, 0);
                if(H4) *H4 = HH4.block<2, 6>(1, 0);
                if(H5) *H5 = HH5.block<2, 6>(1, 0);
                if(H6) *H6 = HH6.block<2, 3>(1, 0);
                if(H7) *H7 = HH7.block<2, 3>(1, 0);
                if(H8) *H8 = HH8.block<2, 6>(1, 0);
                return (gtsam::Vector2() << error.y(), error.x()).finished();
              }
              case AttitudeType::YAWROLL:
              {
                if(H1) *H1 = (gtsam::Matrix26() << HH1.block<1, 6>(0, 0), HH1.block<1, 6>(2, 0)).finished();
                if(H2) *H2 = (gtsam::Matrix23() << HH2.block<1, 3>(0, 0), HH2.block<1, 3>(2, 0)).finished();
                if(H3) *H3 = (gtsam::Matrix23() << HH3.block<1, 3>(0, 0), HH3.block<1, 3>(2, 0)).finished();
                if(H4) *H4 = (gtsam::Matrix26() << HH4.block<1, 6>(0, 0), HH4.block<1, 6>(2, 0)).finished();
                if(H5) *H5 = (gtsam::Matrix26() << HH5.block<1, 6>(0, 0), HH5.block<1, 6>(2, 0)).finished();
                if(H6) *H6 = (gtsam::Matrix23() << HH6.block<1, 3>(0, 0), HH6.block<1, 3>(2, 0)).finished();
                if(H7) *H7 = (gtsam::Matrix23() << HH7.block<1, 3>(0, 0), HH7.block<1, 3>(2, 0)).finished();
                if(H8) *H8 = (gtsam::Matrix26() << HH8.block<1, 6>(0, 0), HH8.block<1, 6>(2, 0)).finished();
                return (gtsam::Vector2() << error.x(), error.z()).finished();
              }
              case AttitudeType::YAW:
              {
                if(H1) *H1 = HH1.block<1, 6>(2, 0);
                if(H2) *H2 = HH2.block<1, 3>(2, 0);
                if(H3) *H3 = HH3.block<1, 3>(2, 0);
                if(H4) *H4 = HH4.block<1, 6>(2, 0);
                if(H5) *H5 = HH5.block<1, 6>(2, 0);
                if(H6) *H6 = HH6.block<1, 3>(2, 0);
                if(H7) *H7 = HH7.block<1, 3>(2, 0);
                if(H8) *H8 = HH8.block<1, 6>(2, 0);
                return (gtsam::Vector1() << error.z()).finished();
              }
              case AttitudeType::ROLL:
              {

                if(H1) *H1 = HH1.block<1, 6>(0, 0);
                if(H2) *H2 = HH2.block<1, 3>(0, 0);
                if(H3) *H3 = HH3.block<1, 3>(0, 0);
                if(H4) *H4 = HH4.block<1, 6>(0, 0);
                if(H5) *H5 = HH5.block<1, 6>(0, 0);
                if(H6) *H6 = HH6.block<1, 3>(0, 0);
                if(H7) *H7 = HH7.block<1, 3>(0, 0);
                if(H8) *H8 = HH8.block<1, 6>(0, 0);
                return (gtsam::Vector1() << error.x()).finished();
              }
              case AttitudeType::PITCH:
              {

                if(H1) *H1 = HH1.block<1, 6>(1, 0);
                if(H2) *H2 = HH2.block<1, 3>(1, 0);
                if(H3) *H3 = HH3.block<1, 3>(1, 0);
                if(H4) *H4 = HH4.block<1, 6>(1, 0);
                if(H5) *H5 = HH5.block<1, 6>(1, 0);
                if(H6) *H6 = HH6.block<1, 3>(1, 0);
                if(H7) *H7 = HH7.block<1, 3>(1, 0);
                if(H8) *H8 = HH8.block<1, 6>(1, 0);
                return (gtsam::Vector1() << error.y()).finished();
              }
            }
          }
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI, const gtsam::Vector6 &accI,
                                                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ, const gtsam::Vector6 &accJ) const {
          const auto& pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
          gtsam::Vector3 error;
          switch (measuredRotFrame_) {
            case MeasurementFrame::NED: {
              const auto nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
              const auto rotNav = nRe.compose(pose.rotation());
              error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              break;
            }
            case MeasurementFrame::ENU: {
              const auto nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
              const auto rotNav = nRe.compose(pose.rotation());
              error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              break;
            }
            default:
              error = gtsam::Rot3::Logmap(pose.rotation().between(rotMeasured_));
              break;
          }
          switch (type_) {
            case AttitudeType::RPY:
            {
              return error;
            }
            case AttitudeType::YAWPITCH:
            {
              return (gtsam::Vector2() << error.y(), error.x()).finished();
            }
            case AttitudeType::YAWROLL:
            {
              return (gtsam::Vector2() << error.x(), error.z()).finished();
            }
            case AttitudeType::YAW:
            {
              return (gtsam::Vector1() << error.z()).finished();
            }
            case AttitudeType::ROLL:
            {
              return (gtsam::Vector1() << error.x()).finished();
            }
            case AttitudeType::PITCH:
            {
              return (gtsam::Vector1() << error.y()).finished();
            }
          }
        }
    };
}
/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::GPWNOJNavAttitudeFactor> :
        public Testable<fgo::factor::GPWNOJNavAttitudeFactor> {
    };
}
#endif //ONLINE_FGO_GPWNOJNAVATTITUDEFACTOR_H
