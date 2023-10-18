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

#ifndef ONLINE_FGO_GPINTERPOLATEDPRDRFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDPRDRFACTOR_H

#pragma once
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>

#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "data/FactorTypes.h"
#include "factor/FactorTypeIDs.h"

/*Inputs:
* Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j), clock bias drift of time i C(i)
* Pseudorange measurement and doppler measurement: measRho and measdRho
* Position of the satellite and velocity of satellite: satXYZ, satVEL
* Position of sensor with respect to the Body: lb
 *delta_t: timedifference of state i & j tau: timedifference of state i & time of measurement
 * omega1&2: angularvelocity of time i and j
* Covariance Matrix of the measurement/s: model, qcModel*/
/* measurement equations used:
 * Pseudorange = Distance of Satellite and Receiver + Range through clock bias,
 * Doppler = Velocity between Satellite and Receiver (in direction) + Velocity through clock drift */
/*Jacobian: for X(i/j) = (e_RS * R_eb * skrew(lb_) , e_RS * R_eb), V(i/j) = 0, e_RS, C(i) = 1,tau,0,1
 * NOTE every Jacobian is multiplied with its designated GPSLAMJacobian at the end
 * */

namespace fgo::factor {

    class GPInterpolatedPrDrFactor : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector2> {
    private:
      double measRho_{}; /** measurement */
      double measdRho_{};
      gtsam::Vector3 angularRate_;
      gtsam::Point3 lb_; ///< The pose of the sensor in the body frame
      gtsam::Vector3 satXYZ_;
      gtsam::Vector3 satVEL_;
      double tau_{};
      bool useAutoDiff_ = false;

      typedef GPInterpolatedPrDrFactor This;
      typedef NoiseModelFactor7 <gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector2> Interpolator;
      typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

      // interpolator
      GPBase GPbase_;
    public:

        GPInterpolatedPrDrFactor() = default; /* Default constructor */

      /**
       * Constructor
       * @param body_P_sensor transformation from body to sensor
       */
      GPInterpolatedPrDrFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key omega_i,
                               gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key omega_j,
                               gtsam::Key cbd_i,
                               const double &measRho, const double &measdRho,
                               const gtsam::Vector3 &satXYZ, const gtsam::Vector3 &satVEL, gtsam::Vector3 &lb,
                               const gtsam::SharedNoiseModel &model,
                               const std::shared_ptr<fgo::models::GPInterpolator> &interpolator, bool useAutoDiff = true) :
              Interpolator(model, pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, cbd_i), measRho_(measRho),
              measdRho_(measdRho), lb_(lb),
              satXYZ_(satXYZ), satVEL_(satVEL), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPbase_(interpolator)
              {
                factorTypeID_ = FactorTypeIDs::GPPRDR;
                factorName_ = "GPInterpolatedPrDrFactor";
              }

      ~GPInterpolatedPrDrFactor() override = default;

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      /** factor error */
      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                  const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                  const gtsam::Vector2 &cbd1,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                                  boost::optional<gtsam::Matrix &> H7 = boost::none) const override {
        using namespace gtsam;

        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, pose2, vel2, omega2, cbd1), pose1);

          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1, omega1, pose2, vel2, omega2, cbd1), vel1);

          if (H3)
            *H3 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, boost::placeholders::_1, pose2, vel2, omega2, cbd1), omega1);

          if (H4)
            *H4 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, boost::placeholders::_1, vel2, omega2, cbd1), pose2);

          if (H5)
            *H5 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1,  pose2, boost::placeholders::_1, omega2, cbd1), vel2);

          if (H6)
            *H6 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, boost::placeholders::_1, cbd1), omega2);

          if (H7)
            *H7 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, omega2, boost::placeholders::_1), cbd1);

          return evaluateError_(pose1, vel1, omega1, pose2, vel2, omega2, cbd1);
        }
        else
        {
          gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P;
          gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V;
          gtsam::Pose3 pose;
          gtsam::Vector6 vel_b;

          if (H1 || H2 || H3 || H4 || H5 || H6) {
            pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2,
                                            Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
            vel_b = GPbase_->interpolateVelocity(pose1, vel1, omega1, pose2, vel2, omega2,
                                                 Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V);
          }
          else {
            pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
            vel_b = GPbase_->interpolateVelocity(pose1, vel1, omega1, pose2, vel2, omega2);
          }

          gtsam::Matrix Hpose_P,Hpose_Pr,Hpose_r;     // jacobian of pose
          gtsam::Matrix3 Hrho_p,Hdrho_p,Hdrho_t;
          gtsam::Matrix13 Hd;

          gtsam::Point3 P_eA_e = pose.translation(&Hpose_P) + pose.rotation(&Hpose_Pr).rotate(lb_,&Hrho_p);
          double real_range = gtsam::distance3(P_eA_e, satXYZ_,&Hd);

          gtsam::Vector3 lin_vel_b(vel_b(3),vel_b(4),vel_b(5));
          gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * vel_b.head(3);
          gtsam::Point3 vel_eA_e = pose.rotation(&Hpose_r).rotate(lbv + lin_vel_b, &Hdrho_p, &Hdrho_t);
          double vel_delta = Hd * (vel_eA_e - satVEL_);

          //  gtsam::Point3 velocityReceiver = vel.block<3, 1>(3, 0) + pose.rotation() * lbv;
          gtsam::Matrix H1_rho, H2_rho, H3_rho, H4_rho, H5_rho, H6_rho;
          gtsam::Matrix H1_drho, H2_drho, H3_drho, H4_drho, H5_drho, H6_drho;
          if (H1 || H2 || H3 || H4 || H5 || H6) {
            gtsam::Matrix tmp_rho = Hrho_p * Hpose_Pr;
            H1_rho = Hd * (Hpose_P + tmp_rho) * Hint1_P;
            H2_rho = Hd * (Hpose_P + tmp_rho) * Hint2_P;
            H3_rho = Hd * (Hpose_P + tmp_rho) * Hint3_P ;
            H4_rho = Hd * (Hpose_P + tmp_rho) * Hint4_P;
            H5_rho = Hd * (Hpose_P + tmp_rho) * Hint5_P;
            H6_rho = Hd * (Hpose_P + tmp_rho) * Hint6_P;

            gtsam::Matrix tmp_drho = Hdrho_p * Hpose_r;
            H1_drho = Hd * (tmp_drho * Hint1_P + Hdrho_t * Hint1_V.block<3,6>(3,0));
            H2_drho = Hd * (tmp_drho * Hint2_P + Hdrho_t * Hint2_V.block<3,3>(3,0));
            H3_drho = Hd * (tmp_drho * Hint3_P + Hdrho_t * Hint3_V.block<3,3>(0,0));
            H4_drho = Hd * (tmp_drho * Hint4_P + Hdrho_t * Hint4_V.block<3,6>(3,0));
            H5_drho = Hd * (tmp_drho * Hint5_P + Hdrho_t * Hint5_V.block<3,3>(3,0));
            H6_drho = Hd * (tmp_drho * Hint6_P + Hdrho_t * Hint6_V.block<3,3>(0,0));

          }

          if(H1) *H1 = (gtsam::Matrix26() <<H1_rho, H1_drho).finished();
          if(H2) *H2 = (gtsam::Matrix23() <<H2_rho, H2_drho).finished();
          if(H3) *H3 = (gtsam::Matrix23() <<H3_rho, H3_drho).finished();
          if(H4) *H4 = (gtsam::Matrix26() <<H4_rho, H4_drho).finished();
          if(H5) *H5 = (gtsam::Matrix23() <<H5_rho, H5_drho).finished();
          if(H6) *H6 = (gtsam::Matrix23() <<H6_rho, H6_drho).finished();
          if(H7) *H7 = (gtsam::Matrix22() << 1, tau_, 0, 1).finished();

          return (gtsam::Vector2() << real_range + cbd1(0) + tau_* cbd1(1) - measRho_,
                                      vel_delta + cbd1(1) - measdRho_ ).finished();
        }

      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                                 const gtsam::Vector2 &cbd1) const {
        // get position
        gtsam::Pose3 pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
        gtsam::Point3 positionReceiver = pose.translation() + pose.rotation().rotate(lb_) ;

        //get velocity
        gtsam::Vector6 vel = GPbase_->interpolateVelocity(pose1, vel1, omega1, pose2, vel2, omega2);
        gtsam::Point3 lbv = gtsam::skewSymmetric(-lb_) * vel.head(3); //velocity through rotation of antenna
        gtsam::Point3 velocityReceiver = pose.rotation().rotate(vel.block<3, 1>(3, 0) + lbv);

        //RCLCPP_ERROR_STREAM(rclcpp::get_logger("fgo"), "\n pose1: " << std::fixed << pose1 << "\n pose2: " << pose2 << "\n pose: " << pose);


        //RCLCPP_ERROR_STREAM(rclcpp::get_logger("fgo"), "\n vel1: " << std::fixed << vel1 << "\n vel2: " << vel2 << "\n vel: " << pose.rotation().rotate(vel.block<3, 1>(3, 0)) << "\n velb " << vel);
        // pose.rotation(&Hpose_r).rotate(lbv + lin_vel_b, &Hdrho_p,&Hdrho_t);
        //error pos
        gtsam::Matrix13 e;
        double distance = gtsam::distance3(positionReceiver, satXYZ_, e); //distance between receiver and sat
        double err = distance + cbd1(0) + tau_ * cbd1(1) - measRho_;
        // error vel
        gtsam::Point3 velocityDifference = velocityReceiver - satVEL_;
        double errd = e * velocityDifference + cbd1(1) - measdRho_;

        //std::cout << "err: " << err << " errd: " << errd << std::endl;
        return gtsam::Vector2(err, errd);
      }

      /** return the measured */
      [[nodiscard]] gtsam::Vector2 measured() const {
        return gtsam::Vector2(measRho_, measdRho_);
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != nullptr && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->measRho_, this->measdRho_).finished(),
                                            (gtsam::Vector2() << e->measRho_, e->measdRho_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterpolatedPrDrFactor" << std::endl;
        Base::print("", keyFormatter);
      }

    private:
      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("GPInterpolatedPrDrFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measRho_);
      }
    }; // PrDrFactor
  } //namespace


/// traits
  namespace gtsam {
    template<>
    struct traits<fgo::factor::GPInterpolatedPrDrFactor> :
            public Testable<fgo::factor::GPInterpolatedPrDrFactor> {
    };
  }

#endif //ONLINE_FGO_GPINTERPOLATEDPRDRFACTOR_H
