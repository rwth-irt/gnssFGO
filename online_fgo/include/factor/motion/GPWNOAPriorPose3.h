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

#ifndef FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H
#define FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H


/**
 *  @file  GPWNOAPriorPose3.h
 *  @brief Pose3 GP prior, use Vn/Wb velocity representation
 *  (Vn:translation velocity in navigation frame, Wb: angular velocity in body frame.)
 **/

#pragma once

#include <ostream>
#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>

#include "utils/GPutils.h"
#include "utils/Pose3utils.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

/**
 * 4-way factor for Gaussian Process prior factor, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 */
    class GPWNOAPriorPose3 : public gtsam::NoiseModelFactor6<gtsam::Pose3,
            gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

    private:
      typedef GPWNOAPriorPose3 This;
      typedef gtsam::NoiseModelFactor6<gtsam::Pose3,
              gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;

    public:
      GPWNOAPriorPose3() = default;    /* Default constructor only for serialization */

      /// Constructor
      /// @param delta_t is the time between the two states
      double delta_t_{};
      bool useAutoDiff_ = false;
      bool calcJacobian_ = false;

      GPWNOAPriorPose3(
              gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key omegaKey1,
              gtsam::Key poseKey2, gtsam::Key velKey2, gtsam::Key omegaKey2,
              double dt, const gtsam::SharedNoiseModel &Qc_model, bool useAutoDiff = false, bool calcJacobian = true) :
              Base(gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(Qc_model), dt)),
                   poseKey1, velKey1, omegaKey1, poseKey2, velKey2, omegaKey2), delta_t_(dt), useAutoDiff_(useAutoDiff), calcJacobian_(calcJacobian)
                   {
                     factorTypeID_ = FactorTypeIDs::GPWNOAMotionPrior;
                     factorName_ = "GPWNOAPriorPose3";
                   }

      ~GPWNOAPriorPose3() override = default;


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
              boost::optional<gtsam::Matrix &> H6 = boost::none
      ) const override {

        using namespace gtsam;
        using namespace fgo::utils;

        if(useAutoDiff_)
        {
          // jacobians
          if (H1) {
            *H1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, pose2, vel2, omega2),
                pose1, 1e-5);
          }
          if (H2) {
            *H2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1, omega1, pose2, vel2, omega2),
                vel1, 1e-5);
          }
          if (H3) {
            *H3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, boost::placeholders::_1, pose2, vel2, omega2),
                omega1, 1e-5);
          }
          if (H4) {
            *H4 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, boost::placeholders::_1, vel2, omega2),
                pose2, 1e-5);
          }
          if (H5) {
            *H5 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, boost::placeholders::_1, omega2),
                vel2, 1e-5);
          }
          if (H6) {
            *H6 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, boost::placeholders::_1),
                omega2, 1e-5);
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

          Matrix6 Jinv = (gtsam::Matrix6() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, gtsam::I_3x3).finished();
          if(calcJacobian_)
            Jinv = fgo::utils::rightJacobianPose3inv(r);

          Matrix63 H1v, H1w, H2v, H2w;
          Matrix6 H1p, H2p;
          Matrix_12_6 Hv1, Hv2;

          Vector6 v1, v2;

          if(H2 || H3 || H5 || H6)
          {
            v1 = convertVwWbToVbWb(vel1, omega1, pose1, &H1v, &H1w, &H1p);
            v2 = convertVwWbToVbWb(vel2, omega2, pose2, &H2v, &H2w, &H2p);
            Hv1 = (Matrix_12_6() << -delta_t_ * Matrix6::Identity(), -Matrix6::Identity()).finished();
            Hv2 = (Matrix_12_6() << Matrix6::Zero(), Jinv).finished();
          } else {
            v1 = convertVwWbToVbWb(vel1, omega1, pose1);
            v2 = convertVwWbToVbWb(vel2, omega2, pose2);
          }

          Matrix6 Jdiff = Matrix6::Zero();
          if((H1 || H4) && calcJacobian_)
            Jdiff = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, v2);

          if(H1)
          {
            const Matrix6 J_Ti = Hlogmap * Hcomp1 * Hinv;
            Matrix6 Jdiff_Ti = Matrix6::Zero();
            if(calcJacobian_)
              Jdiff_Ti = Jdiff * J_Ti;
            *H1 = (Matrix_12_6() << J_Ti - delta_t_ * H1p, Jdiff_Ti - H1p).finished();
          }
          if (H2) *H2 = Hv1 * H1v;
          if (H3) *H3 = Hv1 * H1w;

          if (H4) {
            const Matrix6 J_Tj = Hlogmap * Hcomp2;
            Matrix6 Jdiff_Tj = Matrix6::Zero();
            if(calcJacobian_)
              Jdiff_Tj = Jdiff * J_Tj;
            *H4 = (Matrix_12_6() << J_Tj, Jdiff_Tj + Jinv * H2p).finished();
          }

          if (H5) *H5 = Hv2 * H2v;
          if (H6) *H6 = Hv2 * H2w;

          Vector err = (Vector(12) << (r - v1 * delta_t_), Jinv * v2 - v1).finished();

          return err;
        }
      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                                 const gtsam::Vector3 &omega2) const {


        gtsam::Vector6 r = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));
        gtsam::Matrix6 Jinv = fgo::utils::rightJacobianPose3inv(r);
        gtsam::Vector6 v1 = utils::convertVwWbToVbWb(vel1, omega1, pose1);
        gtsam::Vector6 v2 = utils::convertVwWbToVbWb(vel2, omega2, pose2);
        gtsam::Vector err = (gtsam::Vector(12) << (r - v1 * delta_t_), Jinv * v2 - v1).finished();
        return err;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
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
  struct traits<fgo::factor::GPWNOAPriorPose3>
          : public Testable<fgo::factor::GPWNOAPriorPose3> {
  };
}
#endif //FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H