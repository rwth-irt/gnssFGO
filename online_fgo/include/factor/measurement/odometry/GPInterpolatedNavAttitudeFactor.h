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


#ifndef ONLINE_FGO_GPINTERPOLATEDNAVATTITUDEFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDNAVATTITUDEFACTOR_H

#pragma once

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include "model/gp_interpolator/GPWNOAInterpolatorPose3.h"
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

    class GPInterpolatedNavAttitudeFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    protected:
        gtsam::Rot3 rotMeasured_;
        MeasurementFrame measuredRotFrame_ = MeasurementFrame::NED;
        AttitudeType type_ = AttitudeType::RPY;
        double tau_{};
        bool useAutoDiff_ = false;

        typedef GPInterpolatedNavAttitudeFactor This;
        typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

        GPBase GPBase_;
    public:
        GPInterpolatedNavAttitudeFactor() = default;

        GPInterpolatedNavAttitudeFactor(gtsam::Key poseKeyI, gtsam::Key velKeyI, gtsam::Key omegaKeyI,
                                        gtsam::Key poseKeyJ, gtsam::Key velKeyJ, gtsam::Key omegaKeyJ,
                                        const gtsam::Rot3 &rotMeasured, MeasurementFrame rotFrame, AttitudeType type,
                                        const gtsam::SharedNoiseModel &model,
                                        const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                        bool useAutoDiff = true) :
            Base(model, poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ), rotMeasured_(rotMeasured), measuredRotFrame_(rotFrame),
            type_(type), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPBase_(interpolator)
        {
          factorTypeID_ = FactorTypeIDs::GPNavAttitude;
          factorName_ = "GPInterpolatedNavAttitudeFactor";
        }

        ~GPInterpolatedNavAttitudeFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** factor error */
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                                                  const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
          if (useAutoDiff_) {
            if (H1) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  break;
                }
              }
            }
            if (H2) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
              }
            }
            if (H3) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  break;
                }
              }
            }
            if (H4) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  break;
                }
              }
            }
            if (H5) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
              }
            }
            if (H6) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  break;
                }
              }
            }
            return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
          } else {

            gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, D_dR_R, D_xi_R;

            gtsam::Pose3 pose;
            if (H1 || H2 || H3 || H4 || H5 || H6) {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                              Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
            }
            else {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
            }

            gtsam::Matrix Hpose, Hrot1, Hrot2;

            const auto &pos = pose.translation((H1 || H2 || H3 || H4 || H5 || H6) ? &Hpose : nullptr);
            const auto &rot = pose.rotation((H1 || H2 || H3 || H4 || H5 || H6) ? &Hrot1 : nullptr);

            gtsam::Matrix36 H14_;
            gtsam::Matrix HH1, HH2, HH3, HH4, HH5, HH6;

            /*
             *   Shape
             *   H1: (1-3) * 6
             *   H2: (1-3) * 3
             *   H3: (1-3) * 3
             *   H4: (1-3) * 6
             *   H5: (1-3) * 3
             *   H6: (1-3) * 3
             */

            gtsam::Vector3 error;
            switch (measuredRotFrame_) {
              case MeasurementFrame::NED: {
                const auto &nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, (H1 || H2 || H3 || H4 || H5 || H6) ? &Hrot2 : nullptr);
                error = rotNav.rpy(D_xi_R) - rotMeasured_.rpy();
                // Shape H1 == Shape H4: 3 x 6
                if(H1 || H2 || H3 || H4 || H5 || H6)
                {
                  H14_ = D_xi_R  * Hrot2 * Hrot1 + Hpose; // 36 + 36
                }
                break;
              }
              case MeasurementFrame::ENU: {
                const auto &nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, (H1 || H4) ? &Hrot2 : nullptr);
                error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_, (H1 || H4) ? &D_dR_R : nullptr), (H1 || H4) ? &D_xi_R : nullptr);
                if(H1 || H2 || H3 || H4 || H5 || H6)
                {
                  H14_ << D_xi_R * D_dR_R * Hrot2 * Hrot1 + Hpose; // 36 + 36
                }
                break;
              }
              default: {
                error = gtsam::Rot3::Logmap(rot.between(rotMeasured_, (H1 || H4) ? &D_dR_R : nullptr), (H1 || H4) ? &D_xi_R : nullptr);
                if(H1 || H2 || H3 || H4 || H5 || H6)
                {
                  H14_ << D_xi_R * D_dR_R * Hrot1 + Hpose; // 36 + 36
                }
                break;
              }
            }

            if(H1) HH1 = H14_ * Hint1_P;  // 36 * 66
            if(H4) HH4 = H14_ * Hint4_P;
            if(H2) HH2 = H14_ * Hint2_P;  // Shape Hint2_p 6 * 3
            if(H3) HH3 = H14_ * Hint3_P;
            if(H5) HH5 = H14_ * Hint5_P;
            if(H6) HH6 = H14_ * Hint6_P;

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
                if(H5) *H5 = HH5.block<2, 3>(1, 0);
                if(H6) *H6 = HH6.block<2, 3>(1, 0);
                return (gtsam::Vector2() << error.y(), error.x()).finished();
              }
              case AttitudeType::YAWROLL:
              {
                if(H1) *H1 = (gtsam::Matrix26() << HH1.block<1, 6>(0, 0), HH1.block<1, 6>(2, 0)).finished();
                if(H2) *H2 = (gtsam::Matrix23() << HH2.block<1, 3>(0, 0), HH2.block<1, 3>(2, 0)).finished();
                if(H3) *H3 = (gtsam::Matrix23() << HH3.block<1, 3>(0, 0), HH3.block<1, 3>(2, 0)).finished();
                if(H4) *H4 = (gtsam::Matrix26() << HH4.block<1, 6>(0, 0), HH4.block<1, 6>(2, 0)).finished();
                if(H5) *H5 = (gtsam::Matrix23() << HH5.block<1, 3>(0, 0), HH5.block<1, 3>(2, 0)).finished();
                if(H6) *H6 = (gtsam::Matrix23() << HH6.block<1, 3>(0, 0), HH6.block<1, 3>(2, 0)).finished();
                return (gtsam::Vector2() << error.x(), error.z()).finished();
              }
              case AttitudeType::YAW:
              {
                if(H1) *H1 = HH1.block<1, 6>(2, 0);
                if(H2) *H2 = HH2.block<1, 3>(2, 0);
                if(H3) *H3 = HH3.block<1, 3>(2, 0);
                if(H4) *H4 = HH4.block<1, 6>(2, 0);
                if(H5) *H5 = HH5.block<1, 3>(2, 0);
                if(H6) *H6 = HH6.block<1, 3>(2, 0);
                return (gtsam::Vector1() << error.z()).finished();
              }
              case AttitudeType::ROLL:
              {
                if(H1) *H1 = HH1.block<1, 6>(0, 0);
                if(H2) *H2 = HH2.block<1, 3>(0, 0);
                if(H3) *H3 = HH3.block<1, 3>(0, 0);
                if(H4) *H4 = HH4.block<1, 6>(0, 0);
                if(H5) *H5 = HH5.block<1, 3>(0, 0);
                if(H6) *H6 = HH6.block<1, 3>(0, 0);
                return (gtsam::Vector1() << error.x()).finished();
              }
              case AttitudeType::PITCH:
              {
                if(H1) *H1 = HH1.block<1, 6>(1, 0);
                if(H2) *H2 = HH2.block<1, 3>(1, 0);
                if(H3) *H3 = HH3.block<1, 3>(1, 0);
                if(H4) *H4 = HH4.block<1, 6>(1, 0);
                if(H5) *H5 = HH5.block<1, 3>(1, 0);
                if(H6) *H6 = HH6.block<1, 3>(1, 0);
                return (gtsam::Vector1() << error.y()).finished();
              }
            }
          }
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                                                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ) const {
          const auto& pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
          gtsam::Vector3 error;
          switch (measuredRotFrame_) {
            case MeasurementFrame::NED: {
              const auto &nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
              const auto &rotNav = nRe.compose(pose.rotation());
              //error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              error = rotNav.rpy() - rotMeasured_.rpy();
              //error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              break;
            }
            case MeasurementFrame::ENU: {
              const auto &nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
              const auto &rotNav = nRe.compose(pose.rotation());
              //error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              error = rotNav.rpy() - rotMeasured_.rpy();
              break;
            }
            default: {
              error = gtsam::Rot3::Logmap(pose.rotation().between(rotMeasured_));
              break;
            }
          }

          if(error.hasNaN())
            error.setZero();

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
    struct traits<fgo::factor::GPInterpolatedNavAttitudeFactor> :
        public Testable<fgo::factor::GPInterpolatedNavAttitudeFactor> {
    };
}

#endif //ONLINE_FGO_GPINTERPOLATEDNAVATTITUDEFACTOR_H
