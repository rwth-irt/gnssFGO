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

#ifndef ONLINE_FGO_NAVROTATIONFACTOR_H
#define ONLINE_FGO_NAVROTATIONFACTOR_H

#pragma once

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

    class NavAttitudeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    protected:
        gtsam::Rot3 rotMeasured_;
        MeasurementFrame measuredRotFrame_ = MeasurementFrame::NED;
        AttitudeType type_ = AttitudeType::RPY;
        bool useAutoDiff_ = false;

        typedef NavAttitudeFactor This;
        typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

    public:
        NavAttitudeFactor() = default;

        NavAttitudeFactor(gtsam::Key poseKey,
                          const gtsam::Rot3 &rotMeasured,
                          MeasurementFrame rotFrame, AttitudeType type,
                          const gtsam::SharedNoiseModel &model, bool useAutoDiff = true) :
            Base(model, poseKey), rotMeasured_(rotMeasured), measuredRotFrame_(rotFrame), type_(type), useAutoDiff_(useAutoDiff)
            {
              factorTypeID_ = FactorTypeIDs::NavAttitude;
              factorName_ = "NavAttitudeFactor";
            };

        ~NavAttitudeFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** factor error */
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none) const override {
          if (1 || useAutoDiff_) {
            if (H1) {
              switch (type_) {
                case AttitudeType::RPY:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1), pose, 1e-5);
                  break;
                }
                case AttitudeType::YAWPITCH:
                case AttitudeType::YAWROLL:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1), pose, 1e-5);
                  break;
                }
                case AttitudeType::YAW:
                case AttitudeType::ROLL:
                case AttitudeType::PITCH:
                {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1), pose, 1e-5);
                  break;
                }
              }
            }
            return evaluateError_(pose);
          } else {
            gtsam::Matrix Hpose, Hrot1, Hrot2, D_xi_R;

            const auto &pos = pose.translation(H1 ? &Hpose : nullptr);
            const auto &rot = pose.rotation(H1 ? &Hrot1 : nullptr);

            gtsam::Vector3 error;
            gtsam::Matrix36 H;
            // shape of H1 6 x 6  rot(3,6) pos(3,6)
            switch (measuredRotFrame_) {
              case MeasurementFrame::NED: {
                const auto &nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, H1 ? &Hrot2 : nullptr);
                error = rotNav.rpy(D_xi_R) - rotMeasured_.rpy();
                if (H1) H = D_xi_R * Hrot2 * Hrot1 + Hpose;
                break;
              }
              case MeasurementFrame::ENU: {
                const auto &nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos));
                const auto &rotNav = nRe.compose(rot, H1 ? &Hrot2 : nullptr);
                error = rotNav.rpy(D_xi_R) - rotMeasured_.rpy();
                if (H1) H = D_xi_R * Hrot2 * Hrot1 + Hpose;
                break;
              }
              default: {
                error = rot.rpy(D_xi_R) - rotMeasured_.rpy();
                if (H1) H = D_xi_R * Hrot1 + Hpose;
                break;
              }
            }

            switch (type_) {
              case AttitudeType::RPY:
              {
                if(H1) *H1 = H;
                return error;
              }
              case AttitudeType::YAWPITCH:
              {
                if(H1) *H1 = (gtsam::Matrix26() << H.block<1, 6>(1, 0), H.block<1, 6>(2, 0)).finished();
                return (gtsam::Vector2() << error.y(), error.z()).finished();
              }
              case AttitudeType::YAWROLL:
              {
                if(H1) *H1 = (gtsam::Matrix26() << H.block<1, 6>(0, 0), H.block<1, 6>(2, 0)).finished();
                return (gtsam::Vector2() << error.x(), error.z()).finished();
              }
              case AttitudeType::YAW:
              {
                if(H1) *H1 = H.block<1, 6>(2, 0);
                return (gtsam::Vector1() << error.z()).finished();
              }
              case AttitudeType::ROLL:
              {
                if(H1) *H1 = H.block<1, 6>(0, 0);
                return (gtsam::Vector1() << error.x()).finished();
              }
              case AttitudeType::PITCH:
              {
                if(H1) *H1 = H.block<1, 6>(1, 0);
                return (gtsam::Vector1() << error.y()).finished();
              }
            }
          }
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose) const {
          gtsam::Vector3 error;
          switch (measuredRotFrame_) {
            case MeasurementFrame::NED: {
              const auto nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
              const auto rotNav = nRe.compose(pose.rotation());
              //error = gtsam::Rot3::Logmap(rotNav.between(rotMeasured_));
              error = rotNav.rpy() - rotMeasured_.rpy();
              break;
            }
            case MeasurementFrame::ENU: {
              const auto &nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
              const auto &rotNav = nRe.compose(pose.rotation());
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
        struct traits<fgo::factor::NavAttitudeFactor> :
            public Testable<fgo::factor::NavAttitudeFactor> {
        };
}
#endif //ONLINE_FGO_NAVROTATIONFACTOR_H
