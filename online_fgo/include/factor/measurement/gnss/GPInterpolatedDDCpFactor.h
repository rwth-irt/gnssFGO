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

#ifndef ONLINE_FGO_GPINTERPOLATEDDDCPFACTORO_H
#define ONLINE_FGO_GPINTERPOLATEDDDCPFACTORO_H

#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "data/FactorTypes.h"
#include "factor/FactorTypeIDs.h"

/*Inputs:
 * Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j), integer double differenced ambiguity of time i M(i)
 * Double Difference CarrierPhase measurement: ddphi (already calculated) or carPhaMR,carPhaMB,carPhaIR,carPhaIB
 * Position of the satellite Master (got chosen) and the other satellite I: pointMaster ,pointSatI
 * Position of the Base station we double differentiate to: pointBase
 * Integer to find the right ambiguity in the vector: nAmbiguity (maybe it can be done in a more elegant way)
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * delta_t: timedifference of state i & j tau: timedifference of state i & time of measurement
 * omega1&2: angularvelocity of time i and j
 * Covariance Matrix of the measurement/s: model and QcModel*/
/* measurement equation used:
 * Wavelength * DDCarrierPhase = Double Differenced Distance of Satellite and Receiver -(+) lambda * DDAmbiguityCycles + noise
 * somtimes the AmbiguityCycles get added, but its just a definition in DD measurements it doesnt matter because it can be neg anyway*/
/*Jacobian: (e_RM - e_RI) * (R * skew(t), I) * GPSLAMJacobian (Pose of measurement derived by vel or pose or omega)
 * */
//Dong, Boots, Dellaert - Sparse Gaussian Processes for Continuous-Time Trajectory Estimation on Matrix Lie Groups
//Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)

//IT IS IMPORTANT TO CHECK FOR CYCLESLIP BETWEEN THIS AND LAST MEASUREMENT FOR THIS FACTOR

namespace fgo::factor {

    class GPInterpolatedDDCpFactor : public fgo::NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Matrix , gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
    private:
      //measurements
      double ddphi_{}; //DD measurement
      int nAmbiguity_{}; //position of ambiguity in vector
      //constants
      double lambda_{}; //wavelength
      gtsam::Point3 lb_; //position of antenna in body frame
      gtsam::Point3 lb2_ = gtsam::Point3(0, 0, 0);
      gtsam::Point3 pointMaster_; //position Master Satellite
      gtsam::Point3 pointSatI_;  //position Satellite I
      gtsam::Point3 pointBase_; //position of Base
      bool useAutoDiff_ = false;
      std::shared_ptr<fgo::models::GPInterpolator> GPbase_;

      typedef GPInterpolatedDDCpFactor This;
      typedef fgo::NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
              gtsam::Matrix, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;


    public:
        GPInterpolatedDDCpFactor() = default;

      // ddmeasurements not known when constructed
      GPInterpolatedDDCpFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                   const gtsam::Key &point_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                   const gtsam::Key &ambiguity_j,
                                   const double &ddCarPha,
                                   const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                   const gtsam::Point3 &pointBase, const int nAmbiguity,
                                   const gtsam::Point3 &lb, const double &lambda, const gtsam::SharedNoiseModel &model,
                                   const std::shared_ptr<fgo::models::GPInterpolator>& interpolator, bool useAutoDiff = false) :
              Interpolator(model, point_i, vel_i, omega_i, ambiguity_j, point_j, vel_j, omega_j), ddphi_(ddCarPha),
              nAmbiguity_(nAmbiguity), lambda_(lambda), lb_(lb), pointMaster_(pointMaster), pointSatI_(pointSatI),
              pointBase_(pointBase), GPbase_(interpolator), useAutoDiff_(useAutoDiff) {
        factorTypeID_ = FactorTypeIDs::GPDDCP;
        factorName_ = "GPInterpolatedDDCpFactor";
      }

        GPInterpolatedDDCpFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                   const gtsam::Key &point_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                   const gtsam::Key &ambiguity_j,
                                   const double &ddCarPha, const gtsam::Point3 &pointMaster,
                                   const gtsam::Point3 &pointSatI, const int nAmbiguity, const gtsam::Point3 &lb,
                                   const gtsam::Point3 &lb2,
                                   const double &lambda, const gtsam::SharedNoiseModel &model,
                                   const std::shared_ptr<fgo::models::GPInterpolator>& interpolator, bool useAutoDiff = false) :
              Interpolator(model, point_i, vel_i, omega_i, ambiguity_j, point_j, vel_j, omega_j), ddphi_(ddCarPha),
              nAmbiguity_(nAmbiguity), lambda_(lambda), lb_(lb), lb2_(lb2), pointMaster_(pointMaster),
              pointSatI_(pointSatI), GPbase_(interpolator), useAutoDiff_(useAutoDiff) {
          factorTypeID_ = FactorTypeIDs::GPDDCP;
          factorName_ = "GPInterpolatedDDCpFactor";
      }

      ~GPInterpolatedDDCpFactor() override = default;

      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                  const gtsam::Matrix &ddambiguity_j,
                                  const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                                  boost::optional<gtsam::Matrix &> H7 = boost::none) const override {


        if(useAutoDiff_)
        {
          if (H1) *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, ddambiguity_j, pose2, vel2, omega2), pose1);
          if (H2) *H2 = gtsam::numericalDerivative11<gtsam::Vector1,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, boost::placeholders::_1, omega1, ddambiguity_j, pose2, vel2, omega2), vel1);
          if (H3) *H3 = gtsam::numericalDerivative11<gtsam::Vector1,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, boost::placeholders::_1, ddambiguity_j, pose2, vel2, omega2), omega1);
          //if (H4) *H4 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector>(
          //          boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, boost::placeholders::_1, pose2, vel2, omega2), ddambiguity_j);
          if (H4) {
            *H4 = gtsam::Matrix::Zero(1, ddambiguity_j.size());
            H4->coeffRef(nAmbiguity_) = -lambda_;
          }
          if (H5) *H5 = gtsam::numericalDerivative11<gtsam::Vector1,gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, ddambiguity_j, boost::placeholders::_1, vel2, omega2), pose2);
          if (H6) *H6 = gtsam::numericalDerivative11<gtsam::Vector1,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, ddambiguity_j, pose2, boost::placeholders::_1, omega2), vel2);
          if (H7) *H7 = gtsam::numericalDerivative11<gtsam::Vector1,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, ddambiguity_j, pose2, vel2, boost::placeholders::_1), omega2);

          return evaluateError_(pose1, vel1, omega1, ddambiguity_j, pose2, vel2, omega2);
        }
        else
        {
          gtsam::Pose3 poseBody; //interpolated position
          gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P; //matrices for interpolate position

          if (H1 || H2 || H3 || H5 || H6 || H7) {
            poseBody = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2,
                                                Hint1_P, Hint2_P, Hint3_P, Hint4_P);
          } else {
            poseBody = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
          }

          gtsam::Matrix Hpose_P, Hpose_P2, Hpose_Pr, Hpose_Pr2, Hpose_r;     // jacobian of pose
          gtsam::Matrix3 Hrho_p, Hrho_p2, Hdrho_p,Hdrho_t;
          gtsam::Matrix13 Hd;

          double rangeRM, rangeRI, rangeBM, rangeBI;
          gtsam::Matrix13 e_RM, e_RI; //unity vector pointing from receiver to satellite (1x3)
          gtsam::Matrix13 e_RM2, e_RI2 = gtsam::Matrix13(0, 0, 0);
          gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
          gtsam::Point3 positionReceiver = poseBody.translation(&Hpose_P) + poseBody.rotation(&Hpose_Pr).rotate(lb_,&Hrho_p);

          //calculate distances
          if (lb2_.norm() == 0) {
            rangeRM = gtsam::distance3(positionReceiver, pointMaster_, &e_RM);
            rangeRI = gtsam::distance3(positionReceiver, pointSatI_, &e_RI);
            rangeBM = gtsam::distance3(pointBase_, pointMaster_);
            rangeBI = gtsam::distance3(pointBase_, pointSatI_);
          } else {
            gtsam::Point3 positionReceiver2 = poseBody.translation(&Hpose_P) + poseBody.rotation(&Hpose_Pr2).rotate(lb2_, &Hrho_p2);
            rangeRM = gtsam::distance3(positionReceiver, pointMaster_, &e_RM);
            rangeRI = gtsam::distance3(positionReceiver, pointSatI_, &e_RI);
            rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, &e_RM2);
            rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, &e_RI2);
          }

          if (H1 || H2 || H3 || H5 || H6 || H7) {
            gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
            gtsam::Matrix16 jacobian = //calculate Jacobian for poseReceiver
                (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                                      (e_RM2 - e_RI2) * rotation * gtsam::skewSymmetric(-lb2_),
                                       e_RM - e_RI - (e_RM2 - e_RI2)).finished();

            //CORRECT MATRICES
            gtsam::Matrix66 correction1 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix()).finished();
            gtsam::Matrix66 correction2 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix()).finished();
            //gtsam::Matrix66 correction = (gtsam::Matrix66()
            //        << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
            //Hint1_P = correction * Hint1_P * correction1;
            //Hint3_P = correction * Hint3_P * correction2;

            if (H1) *H1 = (gtsam::Matrix16() << jacobian * Hint1_P * correction1).finished(); //pose1
            if (H2) *H2 = (gtsam::Matrix13() << jacobian * Hint2_P.block<6, 3>(0, 3)).finished(); //vel1
            if (H3) *H3 = (gtsam::Matrix13() << jacobian * Hint2_P.block<6, 3>(0, 0)).finished(); //omega1
            if (H5) *H5 = (gtsam::Matrix16() << jacobian * Hint3_P * correction2).finished(); //pose2
            if (H6) *H6 = (gtsam::Matrix13() << jacobian * Hint4_P.block<6, 3>(0, 3)).finished(); //vel2
            if (H7) *H7 = (gtsam::Matrix13() << jacobian * Hint4_P.block<6, 3>(0, 0)).finished(); //omega2
          }

          if (H4) *H4 = (gtsam::Matrix11() << lambda_).finished(); //ddAmbiguity

          return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) +
                                      lambda_ * (ddambiguity_j(nAmbiguity_) - ddphi_)).finished();
        }

      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                                 const gtsam::Matrix &ddambiguity_j,
                                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2) const {
        gtsam::Pose3 poseBody; //interpolated position

        poseBody = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);

        gtsam::Point3 positionReceiver = poseBody.translation() + poseBody.rotation() * lb_; //pose_Antenna wrt to Earth
        double rangeRM, rangeRI, rangeBM, rangeBI;

        //calculate distances
        if (lb2_.norm() == 0) {
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_);
          rangeBM = gtsam::distance3(pointBase_, pointMaster_);
          rangeBI = gtsam::distance3(pointBase_, pointSatI_);
        } else {
          gtsam::Point3 positionReceiver2 = poseBody.translation() + poseBody.rotation() * lb2_; //pose_Antenna wrt to Earth
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_);
          rangeBM = gtsam::distance3(positionReceiver2, pointMaster_);
          rangeBI = gtsam::distance3(positionReceiver2, pointSatI_);
        }

        return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) + lambda_ * (ddambiguity_j(nAmbiguity_) - ddphi_)).finished();
      }

      /** return the measured */

      double measured() const {
        return ddphi_;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->ddphi_).finished(),
                                            (gtsam::Vector1() << e->ddphi_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterpolatedDDCpFactor:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("GPInterpolatedDDCpFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(ddphi_);
      }
    };
  }


#endif //ONLINE_FGO_GPWNOAINTERDDCPFACTORO_H

namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedDDCpFactor> :
          public Testable<fgo::factor::GPInterpolatedDDCpFactor> {
  };
}