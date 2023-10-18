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

#ifndef ONLINE_FGO_GPWNOJPRIORPOSE3_H
#define ONLINE_FGO_GPWNOJPRIORPOSE3_H

#pragma once

#include <ostream>
#include <boost/lexical_cast.hpp>

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>

#include "data/FactorTypes.h"
#include "utils/GPutils.h"
#include "factor/FactorTypeIDs.h"

namespace gtsam
{
    GTSAM_MAKE_VECTOR_DEFS(18)
}

namespace fgo::factor {

    class GPWNOJPriorPose3 : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                                           gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    private:
      typedef GPWNOJPriorPose3 This;
      typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                       gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;

      double delta_t_{};
      bool useAutoDiff_ = false;
      bool calcJacobian_ = false;

      gtsam::Vector6 accI_;   // measured
      gtsam::Vector6 accJ_;

    public:
      GPWNOJPriorPose3() = default;    /* Default constructor only for serialization */

      /// Constructor
      /// @param delta_t is the time between the two states
      GPWNOJPriorPose3(
              gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key omegaKey1,
              gtsam::Key poseKey2, gtsam::Key velKey2, gtsam::Key omegaKey2,
              const gtsam::Vector6& accI, const gtsam::Vector6& accJ,
              double dt, const gtsam::SharedNoiseModel &Qc_model, bool useAutoDiff = false, bool calc_jacobian = true) :
              Base(gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ3<6>(fgo::utils::getQc(Qc_model), dt)),
                   poseKey1, velKey1, omegaKey1, poseKey2, velKey2, omegaKey2),
                   delta_t_(dt), accI_(accI), accJ_(accJ),
                   useAutoDiff_(useAutoDiff), calcJacobian_(calc_jacobian) {
        factorTypeID_ = FactorTypeIDs::GPWNOJMotionPrior;
        factorName_ = "GPWNOJPriorPose3";
      }

      ~GPWNOJPriorPose3() override = default;


      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      /// factor error function
      [[nodiscard]] gtsam::Vector evaluateError(
              const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
              const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
              boost::optional<gtsam::Matrix &> H1 = boost::none,
              boost::optional<gtsam::Matrix &> H2 = boost::none,
              boost::optional<gtsam::Matrix &> H3 = boost::none,
              boost::optional<gtsam::Matrix &> H4 = boost::none,
              boost::optional<gtsam::Matrix &> H5 = boost::none,
              boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

        using namespace gtsam;
        using namespace fgo::utils;
        if(useAutoDiff_)
        {
          // jacobians
          if (H1) {
            *H1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, pose2, vel2, omega2), pose1, 1e-5);
          }
          if (H2) {
            *H2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1, omega1, pose2, vel2, omega2), vel1, 1e-5);
          }
          if (H3) {
            *H3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, boost::placeholders::_1, pose2, vel2, omega2), omega1, 1e-5);
          }

          if (H4) {
            *H4 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, boost::placeholders::_1, vel2, omega2), pose2, 1e-5);
          }
          if (H5) {
            *H5 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, boost::placeholders::_1, omega2), vel2, 1e-5);
          }
          if (H6) {
            *H6 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, boost::placeholders::_1), omega2, 1e-5);
          }

          return evaluateError_(pose1, vel1, omega1, pose2, vel2, omega2);
        }
        else
        {
          Matrix6 Hinv, Hcomp1, Hcomp2, Hlogmap;
          Vector6 r;
          if(H1 || H4)
            r = Pose3::Logmap(pose1.inverse(&Hinv).compose(pose2, &Hcomp1, &Hcomp2), &Hlogmap);
          else
            r = Pose3::Logmap(pose1.inverse().compose(pose2));

          Matrix6 Jinv = gtsam::I_6x6;
          if(calcJacobian_) {
            Jinv = fgo::utils::rightJacobianPose3inv(r);
          }

          Matrix63 H1v, H1w, H2v, H2w;
          Matrix6 H1p, H2p;
          Matrix_18_6 Hv1, Hv2;

          Vector6 v1, v2;

          if(H2 || H3 || H5 || H6)
          {
            v1 = convertVwWbToVbWb(vel1, omega1, pose1, &H1v, &H1w, &H1p);
            v2 = convertVwWbToVbWb(vel2, omega2, pose2, &H2v, &H2w, &H2p);
            Hv1 = (Matrix_18_6() << -delta_t_ * Matrix6::Identity(), -Matrix6::Identity(), Matrix6::Zero()).finished();
          } else {
            v1 = convertVwWbToVbWb(vel1, omega1, pose1);
            v2 = convertVwWbToVbWb(vel2, omega2, pose2);
          }

          const gtsam::Vector6 JinvVel2 = Jinv * v2;
          const gtsam::Vector3 omegaJinvVel2 = JinvVel2.head(3), rho2JinvVel2 = JinvVel2.tail(3);
          const gtsam::Matrix3 X = gtsam::skewSymmetric(omegaJinvVel2), Y = gtsam::skewSymmetric(rho2JinvVel2);
          const gtsam::Matrix6 JinvVel2SkewMatrix = (gtsam::Matrix6() << X, gtsam::Matrix3::Zero(), Y,  X).finished();
          const gtsam::Vector3 omegaV2 = v2.head(3), rhoV2 = v2.tail(3);
          const gtsam::Matrix3 XV2 = gtsam::skewSymmetric(omegaV2), YV2 = gtsam::skewSymmetric(rhoV2);
          const gtsam::Matrix6 V2SkewMatrix = (gtsam::Matrix6() << XV2, gtsam::Matrix3::Zero(), YV2,  XV2).finished();
          const gtsam::Matrix3 XA2 = gtsam::skewSymmetric(accJ_.head(3)), YA2 = gtsam::skewSymmetric(accJ_.tail(3));
          const gtsam::Matrix6 A2SkewMatrix = (gtsam::Matrix6() << XA2 , gtsam::Matrix3::Zero(), YA2,  XA2).finished();

          if(H5 || H6)
          {
            const gtsam::Vector6 JinvAccJ = Jinv * accJ_;
            const gtsam::Vector3 omegaAccJ = JinvVel2.head(3), rhoAccJ = JinvVel2.tail(3);
            const gtsam::Matrix3 XAccJ = gtsam::skewSymmetric(omegaAccJ), YAccJ = gtsam::skewSymmetric(rhoAccJ);
            const gtsam::Matrix6 JinvAccJSkewMatrix = (gtsam::Matrix6() << XAccJ, gtsam::Matrix3::Zero(), YAccJ,  XAccJ).finished();

            Hv2 = (Matrix_18_6() << Matrix6::Zero(),
                                    Jinv,
                                    -0.5 * JinvAccJSkewMatrix  + 0.5 * V2SkewMatrix * Jinv).finished();

            /**
             * (gtsam::Vector(18) << (r - v1 * delta_t_ - accI/2 * pow(delta_t_,2)),
                                      Jinv * v2 - v1 - accI * delta_t_,
                                     -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ - accI)
             */
          }

          gtsam::Matrix6 JdiffV2 = gtsam::Matrix6::Zero();
          if((H1 || H4) && calcJacobian_)
            JdiffV2 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, v2);


          if (H1) {
            gtsam::Matrix6 J_Ti =  Hlogmap * Hcomp1 * Hinv;

            gtsam::Matrix6 Jdiff_Ti = gtsam::Matrix6::Zero();
            if(calcJacobian_)
            {
              Jdiff_Ti = JdiffV2 * J_Ti;
            }

            // JinvVel2SkewMatrix * v2 = - v2SkewMatrix * Jinv * V2
            *H1 = (fgo::utils::Matrix_18_6() << J_Ti - delta_t_ * H1p,
                                                Jdiff_Ti - H1p,
                                             (0.25 * V2SkewMatrix * V2SkewMatrix + 0.5 * A2SkewMatrix) * J_Ti).finished();

            // checked on 28th. Aug.
          }

          if (H2) *H2 = Hv1 * H1v;
          if (H3) *H3 = Hv1 * H1w;

          if (H4) {
            gtsam::Matrix6 J_Tj =  Hlogmap * Hcomp2;

            gtsam::Matrix6 Jdiff_Ti = gtsam::Matrix6::Zero();
            if(calcJacobian_)
            {
              Jdiff_Ti = JdiffV2 * J_Tj;
            }
            // JinvVel2SkewMatrix * v2 = - v2SkewMatrix * Jinv*V2
            *H4 = (fgo::utils::Matrix_18_6() << J_Tj,
                                               Jdiff_Ti + Jinv * H2p,
                                               (0.25 * V2SkewMatrix * V2SkewMatrix + 0.5 * A2SkewMatrix) * J_Tj).finished();  // ToDo
          }

          if (H5) *H5 = Hv2 * H2v;
          if (H6) *H6 = Hv2 * H2w;

          return (gtsam::Vector(18) << (r - v1 * delta_t_ - accI_/2 * pow(delta_t_, 2)),
                                          Jinv * v2 - v1 - accI_ * delta_t_,
                                          -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ_ - accI_).finished();

        }
      }

      [[nodiscard]] gtsam::Vector evaluateError_( const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                                  const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2) const {
        gtsam::Vector6 r = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));

        gtsam::Matrix6 Jinv = gtsam::I_6x6;

        gtsam::Vector6 v1 = fgo::utils::convertVwWbToVbWb(vel1, omega1, pose1);
        gtsam::Vector6 v2 = fgo::utils::convertVwWbToVbWb(vel2, omega2, pose2);

        if(calcJacobian_)
        {
          Jinv = fgo::utils::rightJacobianPose3inv(r);
        }

        const gtsam::Vector6 JinvVel2 = Jinv * v2;
        const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
        const gtsam::Matrix3 skew_v_omega = gtsam::skewSymmetric(omega), skew_vel_linear = gtsam::skewSymmetric(rho);
        // eq. (35) and (42) in https://arxiv.org/pdf/1809.06518.pdf
        const auto JinvVel2SkewMatrix = (gtsam::Matrix6() << skew_v_omega, gtsam::Matrix3::Zero(), skew_vel_linear,  skew_v_omega ).finished();
        // RA-L Tang et al. eq (41)
        //std::cout << JinvVel2SkewMatrix << std::endl;
        gtsam::Vector err = (gtsam::Vector(18) << (r - v1 * delta_t_ - 0.5 * accI_ * pow(delta_t_,2)),
                                                   Jinv * v2 - v1 - accI_ * delta_t_,
                                                   -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ_ - accI_).finished();
        return err;
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != nullptr && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "4-way Gaussian Process Factor Pose3 VW" << std::endl;
        Base::print("", keyFormatter);
      }

    private:
      /** Serialization function */
      //friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar & BOOST_SERIALIZATION_NVP(delta_t_);
      }
    }; // GaussianProcessPriorPose3
  } // namespace fgo

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::GPWNOJPriorPose3> : public Testable<fgo::factor::GPWNOJPriorPose3> {};
}



#endif //ONLINE_FGO_GPWNOJPRIORPOSE3_H
