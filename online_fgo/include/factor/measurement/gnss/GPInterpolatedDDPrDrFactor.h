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

#ifndef ONLINE_FGO_GPINTERPODDPRFACTORO_H
#define ONLINE_FGO_GPINTERPODDPRFACTORO_H

#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "factor/FactorTypeIDs.h"

/*Inputs:
* Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j)
* DDPseudorange measurement or 4 Pseudorange measurements: ddPseuRa_ or pseuRaMR, pseuRaMB, pseuRaIR, pseuRaIB
* Position of the satellite Master (got chosen) and the other satellite I: pointMaster ,pointSatI
 * Position of the Base station we double differentiate to: pointBase
* Position of sensor with respect to the Body: lb
 * delta_t: timedifference of state i & j tau: timedifference of state i & time of measurement
 * omega1&2: angularvelocity of time i and j
* Covariance Matrix of the measurement/s: model, qcModel*/
/* measurement equations used:
 *DDCarrierPhase = Double Differenced Distance of Satellite and Receiver */
/*Jacobian: (e_RM - e_RI) * (R * skew(t), I) * jac of gpslam
 * */
//sagnac effect correction https://paulba.no/temp/RelativisticCorrectionsInTheEu.pdf s141
//Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)
//Dong, Boots, Dellaert - Sparse Gaussian Processes for Continuous-Time Trajectory Estimation on Matrix Lie Groups
//Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)

namespace fgo::factor {
    class GPInterpolatedDDPrDrFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3,
            gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

    private:
      //measurements
      double dDPseuRa_{}; //PseudoRange Measurement
      double ddDRange_{};
      //constants
      gtsam::Point3 lb_; //position of antenna in body frame
      gtsam::Point3 lb2_ = gtsam::Point3(0, 0, 0);
      gtsam::Point3 pointMaster_; //position Master Satellite
      gtsam::Point3 velMaster_;
      gtsam::Point3 pointSatI_;  //position Satellite I
      gtsam::Point3 velSatI_;
      gtsam::Point3 pointBase_; //position of Base

      const std::shared_ptr<fgo::models::GPInterpolator> GPbase_; //Interpolator

      typedef GPInterpolatedDDPrDrFactor This;
      typedef gtsam::NoiseModelFactor6<gtsam::Pose3,
              gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;

    public:

        GPInterpolatedDDPrDrFactor() = default;

      //when meausrment are already known when build
      GPInterpolatedDDPrDrFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                 const gtsam::Key &point_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                 const double &PseuRaMR, const double &PseuRaMB,
                                 const double &PseuRaIR, const double &PseuRaIB,
                                 const double &dRangeMR, const double &dRangeMB,
                                 const double &dRangeIR, const double &dRangeIB,
                                 const gtsam::Point3 &pointMaster, const gtsam::Point3 &velMaster,
                                 const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &velSatI, const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &lb,
                                 const gtsam::SharedNoiseModel &model,
                                 const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) :
              Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j),
              lb_(lb), pointMaster_(pointMaster), velMaster_(velMaster), pointSatI_(pointSatI),
              velSatI_(velSatI), pointBase_(pointBase), GPbase_(interpolator) {
        dDPseuRa_ = (PseuRaMR - PseuRaMB) - (PseuRaIR - PseuRaIB);
        ddDRange_ = (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB);
        factorTypeID_ = FactorTypeIDs::DDPRDR;
        factorName_ = "GPInterpolatedDDPrDrFactor";
      }

      //when measurements are not known when build and ddmeasurement already made
      GPInterpolatedDDPrDrFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                 const gtsam::Key &point_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                 const double &ddPseuRa, const double &ddDRange, const gtsam::Point3 &pointMaster,
                                 const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &velSatI, const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &lb, const gtsam::SharedNoiseModel &model,
                                 const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) :
              Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j), dDPseuRa_(ddPseuRa),
              ddDRange_(ddDRange), lb_(lb), pointMaster_(pointMaster),
              velMaster_(velMaster), pointSatI_(pointSatI), velSatI_(velSatI), pointBase_(pointBase),
              GPbase_(interpolator) {
        factorTypeID_ = FactorTypeIDs::GPDDPRDR;
        factorName_ = "GPInterpolatedDDPrDrFactor";
      }

        GPInterpolatedDDPrDrFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                                   const gtsam::Key &point_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                                   const double &ddPseuRa, const double &ddDRange, const gtsam::Point3 &pointMaster,
                                   const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI,
                                   const gtsam::Point3 &velSatI, const gtsam::Point3 &lb,
                                   const gtsam::SharedNoiseModel &model, const gtsam::Point3 &lb2,
                                   const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) :
              Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j), dDPseuRa_(ddPseuRa),
              ddDRange_(ddDRange), lb_(lb), lb2_(lb2), pointMaster_(pointMaster),
              velMaster_(velMaster), pointSatI_(pointSatI), velSatI_(velSatI), GPbase_(interpolator) {
          factorTypeID_ = FactorTypeIDs::GPDDPRDR;
          factorName_ = "GPInterpolatedDDPrDrFactor";
      }

      ~GPInterpolatedDDPrDrFactor() override = default;

      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                  const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                  boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

        /*gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
        gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

        gtsam::Pose3 poseBody; //interpolated position
        gtsam::Vector3 velBody;
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P; //matrices for interpolate position
        gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V; //matrices for interpolate position
        //get the interpolated position of tau, where the measurement was made
        if (H1 || H2 || H3 || H4 || H5 || H6) {
          poseBody = GPbase_->interpolatePose(pose1, omVel1, pose2, omVel2,
                                             Hint1_P, Hint2_P, Hint3_P, Hint4_P);
          velBody = GPbase_->interpolateVelocity(pose1, omVel1, pose2, omVel2,
                                                 Hint1_V, Hint2_V, Hint3_V, Hint4_V).block<3, 1>(3, 0);
        } else {
          poseBody = GPbase_->interpolatePose(pose1, omVel1, pose2, omVel2);
          velBody = GPbase_->interpolateVelocity(pose1, omVel1, pose2, omVel2).block<3, 1>(3, 0);
        }

        gtsam::Point3 lbv, lbv2;
        double rangeRM, rangeRI, rangeBM, rangeBI;
        double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
        gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI; //unity vector pointing from receiver to satellite (1x3)

        gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
        gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth

        //calc vel
        lbv = gtsam::skewSymmetric((-lb_)) * angularRate_;
        gtsam::Point3 velRec = velBody + poseBody.rotation().rotate(lbv);

        //ranges
        if (lb2_.norm() == 0) {
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(pointBase_, pointMaster_, e_BM);
          rangeBI = gtsam::distance3(pointBase_, pointSatI_, e_BI);

          dRangeMR = e_RM * (velRec - velMaster_);
          dRangeIR = e_RI * (velRec - velSatI_);
          dRangeMB = e_BM * (-velMaster_);
          dRangeIB = e_BI * (-velSatI_);

          e_BM = gtsam::Vector3(0, 0, 0), e_BI = gtsam::Vector3(0, 0, 0);
        } else {
          gtsam::Point3 positionReceiver2 = poseBody.compose(gtsam::Pose3(gtsam::Rot3(), lb2_)).translation();

          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, e_BM);
          rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, e_BI);

          lbv2 = gtsam::skewSymmetric((-lb2_)) * angularRate_;
          gtsam::Point3 velRec2 = velBody + poseBody.rotation().rotate(lbv2);
          dRangeMR = e_RM * (velRec - velMaster_);
          dRangeIR = e_RI * (velRec - velSatI_);
          dRangeMB = e_BM * (velRec2 - velMaster_);
          dRangeIB = e_BI * (velRec2 - velSatI_);
        }

        if (H1 || H2 || H3 || H4 || H5 || H6) {
          gtsam::Matrix66 correction1 = (gtsam::Matrix66()
                  << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
          gtsam::Matrix66 correction2 = (gtsam::Matrix66()
                  << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
          gtsam::Matrix66 correction = (gtsam::Matrix66()
                  << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
          //CORRECT MATRICES
          Hint1_P = correction * Hint1_P * correction1;
          Hint1_V = Hint1_V * correction1;
          Hint3_P = correction * Hint3_P * correction2;
          Hint3_V = Hint3_V * correction2;

          //calculate Jacobian for poseReceiver
          gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
          gtsam::Matrix16 dPrdX =
                  (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                                        (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lb2_),
                          (e_RM - e_RI) - (e_BM - e_BI)).finished();
          gtsam::Matrix16  dDrdX = (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lbv) -
                                                         (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lbv2),
                                                         0,0,0).finished();
          gtsam::Matrix16  dDrdVel = (gtsam::Matrix16() << 0, 0, 0, (e_RM - e_BM) - (e_RI - e_BI)).finished();
          if (H1) *H1 = (gtsam::Matrix26() << dPrdX * Hint1_P, dDrdX * Hint1_P + dDrdVel * Hint1_V).finished(); //pose1

          if (H2) *H2 = (gtsam::Matrix23() << dPrdX * Hint2_P.block<6, 3>(0, 3),
                    dDrdX * Hint2_P.block<6, 3>(0, 3) + dDrdVel * Hint2_V.block<6, 3>(0, 3)).finished(); //vel1

          if (H3) *H3 = (gtsam::Matrix23() << dPrdX * Hint2_P.block<6, 3>(0, 0),
                    dDrdX * Hint2_P.block<6, 3>(0, 0) + dDrdVel * Hint2_V.block<6, 3>(0, 0)).finished(); //omega1

          if (H4) *H4 = (gtsam::Matrix26() << dPrdX * Hint3_P, dDrdX * Hint3_P + dDrdVel * Hint3_V).finished(); //pose2

          if (H5) *H5 = (gtsam::Matrix23() << dPrdX * Hint4_P.block<6, 3>(0, 3),
                    dDrdX * Hint4_P.block<6, 3>(0, 3) + dDrdVel * Hint4_V.block<6, 3>(0, 3)).finished(); //vel2

          if (H6) *H6 = (gtsam::Matrix23() << dPrdX * Hint4_P.block<6, 3>(0, 0),
                    dDrdX * Hint4_P.block<6, 3>(0, 0) + dDrdVel * Hint4_V.block<6, 3>(0, 0)).finished(); //omega2
        }*/

        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, pose2, vel2, omega2),
                  pose1);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1, omega1, pose2, vel2, omega2),
                  vel1);
        if (H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1, boost::placeholders::_1, pose2, vel2, omega2),
                  omega1);
        if (H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, boost::placeholders::_1, vel2, omega2),
                  pose2);
        if (H5)
          *H5 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, boost::placeholders::_1, omega2),
                  vel2);
        if (H6)
          *H6 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, boost::placeholders::_1),
                  omega2);

        return evaluateError_(pose1, vel1, omega1, pose2, vel2, omega2);
      }

      gtsam::Vector2 evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                    const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                    const gtsam::Vector3 &omega2) const {

        gtsam::Pose3 poseBody = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
        gtsam::Vector6 velBody = GPbase_->interpolateVelocity(pose1, vel1, omega1, pose2, vel2, omega2);
        //calc ReceiverPos
        gtsam::Point3 positionReceiver = poseBody.translation() + poseBody.rotation() * lb_; //pose_Antenna wrt to Earth
        //calc ReceiverVel
        gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * velBody.head(3);
        gtsam::Point3 velRec = velBody.block<3, 1>(3, 0) + poseBody.rotation().rotate(lbv);
        double rangeRM, rangeRI, rangeBM, rangeBI;
        double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
        gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI; //unity vector pointing from receiver to satellite (1x3)
        //ranges
        if (lb2_.norm() == 0) {
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(pointBase_, pointMaster_, e_BM);
          rangeBI = gtsam::distance3(pointBase_, pointSatI_, e_BI);

          dRangeMR = e_RM * (velRec - velMaster_);
          dRangeIR = e_RI * (velRec - velSatI_);
          dRangeMB = e_BM * (-velMaster_);
          dRangeIB = e_BI * (-velSatI_);
        } else {
          gtsam::Point3 positionReceiver2 = poseBody.compose(gtsam::Pose3(gtsam::Rot3(), lb2_)).translation();

          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, e_BM);
          rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, e_BI);

          gtsam::Point3 lbv2 = gtsam::skewSymmetric((-lb2_)) * velBody.head(3);
          gtsam::Point3 velRec2 = velBody.tail(3) + poseBody.rotation().rotate(lbv2);
          dRangeMR = e_RM * (velRec - velMaster_);
          dRangeIR = e_RI * (velRec - velSatI_);
          dRangeMB = e_BM * (velRec2 - velMaster_);
          dRangeIB = e_BI * (velRec2 - velSatI_);
        }

        return (gtsam::Vector2() << (rangeRM - rangeBM) - (rangeRI - rangeBI) - dDPseuRa_,
                (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB) - ddDRange_).finished();
      }

      /** return the measured */
      [[nodiscard]] gtsam::Vector2 measured() const {
        return (gtsam::Vector2() << dDPseuRa_, ddDRange_).finished();
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->dDPseuRa_).finished(),
                                            (gtsam::Vector1() << e->dDPseuRa_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterpolatedDDPrDrFactor:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("GPInterpolatedDDPrDrFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(dDPseuRa_);
      }

    };

}

#endif //ONLINE_FGO_GPINTERDDPRFACTORO_H

namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedDDPrDrFactor> :
          public Testable<fgo::factor::GPInterpolatedDDPrDrFactor> {
  };
}