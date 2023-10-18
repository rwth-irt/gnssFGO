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

#ifndef ONLINE_FGO_GPWNOAVELOCITYFACTOR_H
#define ONLINE_FGO_GPWNOAVELOCITYFACTOR_H

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

namespace fgo::factor{

    class GPInterpolatedNavVelocityFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
        gtsam::Vector3 velMeasured_;
        gtsam::Vector3 lb_;
        MeasurementFrame measuredVelFrame_ = MeasurementFrame::BODY;
        VelocityType type_ = VelocityType::VEL3D;
        double tau_{};
        bool useAutoDiff_ = false;

        typedef GPInterpolatedNavVelocityFactor This;
        typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

        GPBase GPBase_;
    public:
        GPInterpolatedNavVelocityFactor() = default;

        GPInterpolatedNavVelocityFactor(gtsam::Key poseKeyI, gtsam::Key velKeyI, gtsam::Key omegaKeyI, gtsam::Key poseKeyJ, gtsam::Key velKeyJ, gtsam::Key omegaKeyJ,
                                        const gtsam::Vector3 &velMeasured, const gtsam::Vector3& lb, MeasurementFrame velFrame, VelocityType type,
                                        const gtsam::SharedNoiseModel &model,
                                        const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                        bool useAutoDiff = true) :
            Base(model, poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ), velMeasured_(velMeasured), lb_(lb), measuredVelFrame_(velFrame),
            type_(type), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPBase_(interpolator)
        {
          factorTypeID_ = FactorTypeIDs::GPNavVelocity;
          factorName_ = "GPInterpolatedNavVelocityFactor";
        }

        ~GPInterpolatedNavVelocityFactor() override = default;

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
          static const auto varTrick6D= (gtsam::Vector6() << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9).finished();
          static const auto varTrick3D = (gtsam::Vector3() << 1e-9, 1e-9, 1e-9).finished();
          if(useAutoDiff_) {
            if (H1) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {

                  *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ),
                      poseI, 1e-5);
                  *H1 += varTrick6D;
                  break;
                }
              }
            }
            if (H2) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ),
                      velI, 1e-5);
                  break;
                }
              }
            }
            if (H3) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ),
                      omegaI, 1e-5);
                  *H3 += varTrick3D;
                  break;
                }
              }
            }
            if (H4) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ),
                      poseJ, 1e-5);
                  *H4 += varTrick6D;
                  break;
                }
              }
            }
            if (H5) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ),
                      velJ, 1e-5);
                  break;
                }
              }
            }
            if (H6) {
              switch (type_) {
                case VelocityType::VEL3D: {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  break;
                }
                case VelocityType::VEL2D: {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  break;
                }
                case VelocityType::VELX:
                case VelocityType::VELY: {
                  *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                      boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1),
                      omegaJ, 1e-5);
                  *H6 += varTrick3D;
                  break;
                }
              }
            }
            return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
          }
          else
          {
            gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P;
            gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V;
            gtsam::Pose3 pose;
            gtsam::Vector6 vel;

            if (H1 || H2 || H3 || H4 || H5 || H6) {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                              Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
              vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                                   Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V);
            }
            else {
              pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
              vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ);
            }

            // ToDo: check jacobians!!
            gtsam::Matrix Hpose, Hrot, Hrot_lb, Hvel, Hrot_be;

            const auto lb_skew = gtsam::skewSymmetric((-lb_));
            const auto vel_b = vel.tail(3) + lb_skew * vel.head(3);
            const auto& rot = pose.rotation(Hrot);
            const auto& pos = pose.translation(Hpose) + rot.rotate(lb_, Hrot_lb);

            gtsam::Matrix H1_, H2_, H3_, H4_, H5_, H6_;

            if(H1) H1_ << (Hpose + Hrot_lb * Hrot) * Hint1_P + Hint1_V.block<3, 6>(3, 0);
            if(H4) H4_ << (Hpose + Hrot_lb * Hrot) * Hint4_P + Hint4_V.block<3, 6>(3, 0);

            gtsam::Vector3 error;
            switch (measuredVelFrame_) {
              case MeasurementFrame::NED:
              {
                const auto vel_e = rot.rotate(vel_b, Hrot_be);
                const auto nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos));
                const auto veln = nRe.rotate(vel_e, Hvel);
                if(H2) H2_ << ((Hpose + Hrot) * Hint2_P).block<3, 3>(0, 0) + Hvel * Hrot_be * Hint2_V.block<3, 3>(3, 0);
                if(H3) H3_ << ((Hpose + Hrot) * Hint3_P).block<3, 3>(0, 0) + Hvel * Hrot_be * (lb_skew + Hint3_V.block<3, 3>(0, 0));
                if(H5) H5_ << ((Hpose + Hrot) * Hint5_P).block<3, 3>(0, 0) + Hvel * Hrot_be * Hint5_V.block<3, 3>(3, 0);
                if(H6) H6_ << ((Hpose + Hrot) * Hint6_P).block<3, 3>(0, 0) + Hvel * Hrot_be * (lb_skew + Hint6_V.block<3, 3>(0, 0));
                error = veln - velMeasured_;
                break;
              }
              case MeasurementFrame::ENU:
              {
                const auto vel_e = rot.rotate(vel_b, Hrot_be);
                const auto nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos));
                const auto veln = nRe.rotate(vel_e, Hvel);
                if(H2) H2_ << ((Hpose + Hrot) * Hint2_P).block<3, 3>(0, 0) + Hvel * Hrot_be * Hint2_V.block<3, 3>(3, 0);
                if(H3) H3_ << ((Hpose + Hrot) * Hint3_P).block<3, 3>(0, 0) + Hvel * Hrot_be * (lb_skew + Hint3_V.block<3, 3>(0, 0));
                if(H5) H5_ << ((Hpose + Hrot) * Hint5_P).block<3, 3>(0, 0) + Hvel * Hrot_be * Hint5_V.block<3, 3>(3, 0);
                if(H6) H6_ << ((Hpose + Hrot) * Hint6_P).block<3, 3>(0, 0) + Hvel * Hrot_be * (lb_skew + Hint6_V.block<3, 3>(0, 0));
                error = veln - velMeasured_;
                break;
              }
              case MeasurementFrame::BODY:
              {
                if(H2) H2_ << ((Hpose + Hrot) * Hint2_P).block<3, 3>(0, 0) + Hint2_V.block<3, 3>(3, 0);
                if(H3) H3_ << ((Hpose + Hrot) * Hint3_P).block<3, 3>(0, 0) + (lb_skew + Hint3_V.block<3, 3>(0, 0));
                if(H5) H5_ << ((Hpose + Hrot) * Hint5_P).block<3, 3>(0, 0) + Hint5_V.block<3, 3>(3, 0);
                if(H6) H6_ << ((Hpose + Hrot) * Hint6_P).block<3, 3>(0, 0) + (lb_skew + Hint6_V.block<3, 3>(0, 0));
                error = vel_b  - velMeasured_;
                break;

              }
              default: {
                const auto vel_e = rot.rotate(vel_b, Hrot_be);
                if(H2) H2_ << ((Hpose + Hrot) * Hint2_P).block<3, 3>(0, 0) + Hrot_be * Hint2_V.block<3, 3>(3, 0);
                if(H3) H3_ << ((Hpose + Hrot) * Hint3_P).block<3, 3>(0, 0) + Hrot_be * (lb_skew + Hint3_V.block<3, 3>(0, 0));
                if(H5) H5_ << ((Hpose + Hrot) * Hint5_P).block<3, 3>(0, 0) + Hrot_be * Hint5_V.block<3, 3>(3, 0);
                if(H6) H6_ << ((Hpose + Hrot) * Hint6_P).block<3, 3>(0, 0) + Hrot_be * (lb_skew + Hint6_V.block<3, 3>(0, 0));
                error = vel_e - velMeasured_;
                break;
              }
            }
            switch (type_) {
              case VelocityType::VEL3D: {
                if(H1) *H1 = H1_;
                if(H2) *H2 = H2_;
                if(H3) *H3 = H3_;
                if(H4) *H4 = H4_;
                if(H5) *H5 = H5_;
                if(H6) *H6 = H6_;
                return error;
              }
              case VelocityType::VEL2D: {
                if(H1) *H1 = H1_.block<2, 6>(0, 0);
                if(H2) *H2 = H2_.block<2, 6>(0, 0);
                if(H3) *H3 = H3_.block<2, 6>(0, 0);
                if(H4) *H4 = H4_.block<2, 6>(0, 0);
                if(H5) *H5 = H5_.block<2, 6>(0, 0);
                if(H6) *H6 = H6_.block<2, 6>(0, 0);
                std::cout << error.head(2) << std::endl;
                return error.head(2);
              }
              case VelocityType::VELX:
              {
                if(H1) *H1 = H1_.block<1, 6>(0, 0);
                if(H2) *H2 = H2_.block<1, 6>(0, 0);
                if(H3) *H3 = H3_.block<1, 6>(0, 0);
                if(H4) *H4 = H4_.block<1, 6>(0, 0);
                if(H5) *H5 = H5_.block<1, 6>(0, 0);
                if(H6) *H6 = H6_.block<1, 6>(0, 0);
                return (gtsam::Vector1() << error.x()).finished();
              }
              case VelocityType::VELY: {
                if(H1) *H1 = H1_.block<2, 6>(1, 0);
                if(H2) *H2 = H2_.block<2, 6>(1, 0);
                if(H3) *H3 = H3_.block<2, 6>(1, 0);
                if(H4) *H4 = H4_.block<2, 6>(1, 0);
                if(H5) *H5 = H5_.block<2, 6>(1, 0);
                if(H6) *H6 = H6_.block<2, 6>(1, 0);
                return (gtsam::Vector1() << error.y()).finished();
              }
            }
          }
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                                                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ) const {

          const auto& vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ);
          const auto& vel_b = vel.tail(3) + gtsam::skewSymmetric(-lb_) * vel.head(3);

          gtsam::Vector3 error;
          switch (measuredVelFrame_) {
            case MeasurementFrame::NED:
            {
              const auto& pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
              const auto nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
              const auto vel_e = pose.rotation().rotate(vel_b);
              const auto veln = nRe.rotate(vel_e);
              error = veln - velMeasured_;
              break;
            }
            case MeasurementFrame::ENU:
            {
              const auto& pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
              const auto nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
              const auto vel_e = pose.rotation().rotate(vel_b);
              const auto veln = nRe.rotate(vel_e);
              error = veln - velMeasured_;
              break;
            }
            case MeasurementFrame::BODY:
            {
              error = vel_b - velMeasured_;
              break;
            }
            default: {
              const auto& pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
              const auto vel_e = pose.rotation().rotate(vel_b);
              error = vel_e - velMeasured_;
              break;
            }
          }

          if(error.hasNaN())
            error.setZero();

          switch (type_) {
            case VelocityType::VEL3D: {
              return error;
            }
            case VelocityType::VEL2D: {
              //std::cout << error.head(2) << std::endl;
              return error.head(2);
            }
            case VelocityType::VELX:
            {
              return (gtsam::Vector1() << error.x()).finished();
            }
            case VelocityType::VELY: {
              return (gtsam::Vector1() << error.y()).finished();
            }
          }
        }

/** return the measured */
        [[nodiscard]] gtsam::Vector3 measured() const {
          return velMeasured_;
        }

        /** equals specialized to this factor */
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol)
                 && gtsam::equal_with_abs_tol(this->velMeasured_,
                                              e->measured(), tol);
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
          std::cout << s << "GPInterpolatedPRDRFactor" << std::endl;
          Base::print("", keyFormatter);
        }

    private:
        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int version) {
          ar & boost::serialization::make_nvp("GPInterpolatedNavVelocityFactor",
                                              boost::serialization::base_object<Base>(*this));
          ar & BOOST_SERIALIZATION_NVP(velMeasured_);
        }

    };
}

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::GPInterpolatedNavVelocityFactor> :
        public Testable<fgo::factor::GPInterpolatedNavVelocityFactor> {
    };
}

#endif //ONLINE_FGO_GPWNOAVELOCITYFACTOR_H
