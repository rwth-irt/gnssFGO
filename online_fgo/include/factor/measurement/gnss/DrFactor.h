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

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

/*Inputs:
* Keys: pose of time i X(i), velocity of time i V(i), clock bias drift of time i C(i)
* Pseudorange measurement and doppler measurement: measRho and measdRho
* gyroscope measurement of time i: angularRate
* Position of the satellite and velocity of satellite: satXYZ, satVEL
* Position of sensor with respect to the Body: lb
* Covariance Matrix of the measurement/s: model*/
/* measurement equations used:
 * Pseudorange = Distance of Satellite and Receiver + Range through clock bias,
 * Doppler = Velocity between Satellite and Receiver (in direction) + Velocity through clock drift */
/*Jacobian: for X(i) = (e_RS * R_eb * skrew(lb_), e_RS * R_eb), V(i) = 0, e_RS, C(i) = 1,0,0,1
 * */

namespace fgo::factor {
class DrFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector2> {

    protected:
      gtsam::Vector3 satXYZ_;
      gtsam::Vector3 satVEL_;
      gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame
      gtsam::Vector3 omega_;
      double measdRho_{};
      typedef DrFactor This;
      typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,  gtsam::Vector2> Base;
      bool useAutoDiff_ = false;

    public:

      DrFactor() = default;  /* Default constructor */

      /**
       * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
       */
      DrFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key cbd_i, const double &measdRho, const gtsam::Vector3 &satXYZ,
               const gtsam::Vector3 &satVEL, const gtsam::Vector3 &lb, const gtsam::Vector3 &omega, const gtsam::SharedNoiseModel &model,
               bool useAutoDiff = false) :
              Base(model, pose_i, vel_i, cbd_i), satXYZ_(satXYZ), satVEL_(satVEL),
              lb_(lb), omega_(omega), measdRho_(measdRho), useAutoDiff_(useAutoDiff) {
        factorTypeID_ = FactorTypeIDs::DR;
        factorName_ = "DrFactor";
      }

      ~DrFactor() override = default;

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel, const gtsam::Vector2 &cbd,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none) const override {

        if(useAutoDiff_)
        {
          if (H1) {
            *H1 = gtsam::numericalDerivative31<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3, gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                            boost::placeholders::_3), pose, vel, cbd, 1e-5);
          }
          if (H2)
            *H2 = gtsam::numericalDerivative32<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3, gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                            boost::placeholders::_3), pose, vel, cbd, 1e-5);
          if (H3)
            *H3 = gtsam::numericalDerivative33<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3, gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                            boost::placeholders::_3), pose, vel, cbd, 1e-5);
          return evaluateError_(pose, vel, cbd);
        }
        else
        {
          //gtsam::Matrix36 Ha;
          gtsam::Matrix13 Hd;
          //gtsam::Point3 P_eb_e = pose.translation(H1 ? &Ha : nullptr);
          gtsam::Matrix Hpose_r;
          gtsam::Matrix3 Hrho_p, Hdrho_p, Hdrho_t;

          gtsam::Point3 P_eA_e = pose.translation() + pose.rotation().rotate(lb_, &Hrho_p); //pose.compose(body_P_sensor,H0_P).translation(Hpose_P);
          double real_range = gtsam::distance3(P_eA_e, satXYZ_,&Hd);

          const gtsam::Rot3& eRb = pose.rotation(&Hpose_r);
          gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * omega_;
          gtsam::Point3 vel_eA_e = vel + eRb.rotate(lbv, &Hdrho_p, &Hdrho_t);

          double vel_delta = (Hd * (vel_eA_e - satVEL_))(0);

          if (H1) {
            const gtsam::Matrix tmp_drho = Hdrho_p * Hpose_r;
            const gtsam::Matrix H1_drho = Hd * (tmp_drho + Hdrho_t);
            *H1 = (gtsam::Matrix16() <<0, 0, 0, H1_drho).finished();
          }

          if (H2) *H2 = (gtsam::Matrix13() << Hd).finished();
          if (H3) *H3 = (gtsam::Matrix12() << 0, 1).finished();

          return (gtsam::Vector1() << vel_delta + cbd(1) - measdRho_ ).finished();

        }
      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel,
                                    const gtsam::Vector2 &cbd) const {

          gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * omega_;
          const gtsam::Point3& P_eb_e = pose.translation();
          const gtsam::Rot3& eRb = pose.rotation();
          gtsam::Point3 vel_eA_e = vel + eRb.rotate(lbv);
          gtsam::Point3 P_eA_e = P_eb_e + eRb.rotate(lb_);
          gtsam::Matrix13 Hd;
          double real_range = gtsam::distance3(P_eA_e, satXYZ_, &Hd);
          double vel_delta = (Hd * (vel_eA_e - satVEL_))(0);

        return (gtsam::Vector1() << vel_delta + cbd(1) - measdRho_).finished();

      }

      /** return the measured */
      [[nodiscard]] gtsam::Vector1 measured() const {
        return gtsam::Vector1(measdRho_);
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->measdRho_).finished(),
                                            (gtsam::Vector1() << e->measdRho_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "DrFactor" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("DrFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measdRho_);
      }
    }; // DrFactor
  } // namespace fgo_online

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::DrFactor> : public Testable<fgo::factor::DrFactor> {
  };
}
