//
// Created by lars on 22.02.22.
//

#ifndef ONLINE_FGO_GPINTERDDPRFACTORO_H
#define ONLINE_FGO_GPINTERDDPRFACTORO_H

#include "include/models/archive/GPInterpolatorPose3WNOA_O.h"

/*Inputs:
* Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j)
* DDPseudorange measurement or 4 Pseudorange measurements: ddPseuRa or pseuRaMR, pseuRaMB, pseuRaIR, pseuRaIB
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
namespace fgo::factors {
  class GPWONAInterpolatedDDPrFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3,
          gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

  private:
    //measurements
    double dDPseuRa_; //PseudoRange Measurement
    //constants
    gtsam::Point3 lb_; //position of antenna in body frame
    gtsam::Point3 lb2_;
    //points
    gtsam::Point3 pointMaster_; //position Master Satellite
    gtsam::Point3 pointSatI_;  //position Satellite I
    gtsam::Point3 pointBase_; //position of Base

    fgo::models::GaussianProcessInterpolatorPose3WNOA_O GPbase_; //Interpolator

    typedef GPWONAInterpolatedDDPrFactor This;
    typedef gtsam::NoiseModelFactor6<gtsam::Pose3,
            gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;

  public:

      GPWONAInterpolatedDDPrFactor() {}

    //when meausrment are already known when build
    GPWONAInterpolatedDDPrFactor(const gtsam::Key& point_i, const gtsam::Key& vel_i, const gtsam::Key& omega_i,
                                  const gtsam::Key& point_j, const gtsam::Key& vel_j, const gtsam::Key& omega_j,
                                 const double &PseuRaMR, const double &PseuRaMB,
                                 const double &PseuRaIR, const double &PseuRaIB,
                                 const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &lb,
                                 const double &delta_t, const double &tau,
                                 const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                 const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
            Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j),
            GPbase_(Qc_model, delta_t, tau), pointMaster_(pointMaster), pointSatI_(pointSatI),
            pointBase_(pointBase), lb_(lb) {
      dDPseuRa_ = (PseuRaMR - PseuRaMB) - (PseuRaIR - PseuRaIB);
    }

    //when measurements are not known when build and ddmeasurement already made
    GPWONAInterpolatedDDPrFactor(const gtsam::Key& point_i, const gtsam::Key& vel_i, const gtsam::Key& omega_i,
                                  const gtsam::Key& point_j, const gtsam::Key& vel_j, const gtsam::Key& omega_j,
                                 const double &ddPseuRa,
                                 const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &lb, const double &delta_t, const double &tau,
                                 const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                 const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
            Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j), GPbase_(Qc_model, delta_t, tau),
            pointMaster_(pointMaster), pointSatI_(pointSatI), pointBase_(pointBase),
            lb_(lb), dDPseuRa_(ddPseuRa) {}

      GPWONAInterpolatedDDPrFactor(const gtsam::Key& point_i, const gtsam::Key& vel_i, const gtsam::Key& omega_i,
                                  const gtsam::Key& point_j, const gtsam::Key& vel_j, const gtsam::Key& omega_j,
                                 const double &ddPseuRa,
                                 const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &lb, const double &delta_t, const double &tau,
                                 const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                 const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model, const gtsam::Point3 &lb2) :
            Interpolator(model, point_i, vel_i, omega_i, point_j, vel_j, omega_j), GPbase_(Qc_model, delta_t, tau),
            pointMaster_(pointMaster), pointSatI_(pointSatI),
            lb_(lb), dDPseuRa_(ddPseuRa), lb2_(lb2) {}

    virtual ~GPWONAInterpolatedDDPrFactor() {}

    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none,
                                boost::optional<gtsam::Matrix &> H3 = boost::none,
                                boost::optional<gtsam::Matrix &> H4 = boost::none,
                                boost::optional<gtsam::Matrix &> H5 = boost::none,
                                boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      gtsam::Pose3 poseBody; //interpolated position
      gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P; //matrices for interpolate position
      //get the interpolated position of tau, where the measurement was made
      if (H1 || H2 || H3 || H4 || H5 || H6) {
        poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                           Hint1_P, Hint2_P, Hint3_P, Hint4_P);
      } else {
        poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
      }

      double rangeRM, rangeRI, rangeBM, rangeBI;
      gtsam::Matrix13 e_RM, e_RI; //unity vector pointing from receiver to satellite (1x3)
      gtsam::Matrix13 e_RM2, e_RI2 = gtsam::Matrix13(0, 0, 0);
      gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
      gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth

      //ranges
      if (lb2_.norm() == 0){
        rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
        rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
        rangeBM = gtsam::distance3(pointBase_, pointMaster_);
        rangeBI = gtsam::distance3(pointBase_, pointSatI_);
      } else {
        gtsam::Point3 positionReceiver2 = poseBody.compose(gtsam::Pose3(gtsam::Rot3(), lb2_)).translation();
        rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
        rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
        rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, e_RM2);
        rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, e_RI2);
      }

      if (H1 || H2 || H3 || H4 || H5 || H6) {
        gtsam::Matrix66 correction1 = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction2 = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
        //CORRECT MATRICES
        Hint1_P = correction * Hint1_P * correction1;
        Hint3_P = correction * Hint3_P * correction2;

        gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
        gtsam::Matrix16 jacobian =
                (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                                      (e_RM2 - e_RI2) * rotation * gtsam::skewSymmetric(-lb2_),
                        (e_RM - e_RI) - (e_RM2 - e_RI2)).finished(); //calculate Jacobian for poseReceiver
        if (H1) *H1 = (gtsam::Matrix16() << jacobian * Hint1_P).finished(); //pose1
        if (H2) *H2 = (gtsam::Matrix13() << jacobian * Hint2_P.block<6,3>(0,3)).finished(); //vel1
        if (H3) *H3 = (gtsam::Matrix13() << jacobian * Hint2_P.block<6,3>(0,0)).finished(); //omega1
        if (H4) *H4 = (gtsam::Matrix16() << jacobian * Hint3_P).finished(); //pose2
        if (H5) *H5 = (gtsam::Matrix13() << jacobian * Hint4_P.block<6,3>(0,3)).finished(); //vel2
        if (H6) *H6 = (gtsam::Matrix13() << jacobian * Hint4_P.block<6,3>(0,0)).finished(); //omega2
      }

      return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) - dDPseuRa_).finished();
    }

    /** return the measured */
    const double &measured() const {
      return dDPseuRa_;
    }

    /** number of variables attached to this factor */
    size_t size() const {
      return 6;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != NULL && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->dDPseuRa_).finished(),
                                          (gtsam::Vector1() << e->dDPseuRa_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterDDPseudorange:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(dDPseuRa_);
    }

  };

}

#endif //ONLINE_FGO_GPINTERDDPRFACTORO_H

namespace gtsam {
  template<>
  struct traits<fgo::factors::GPWONAInterpolatedDDPrFactor> :
          public Testable<fgo::factors::GPWONAInterpolatedDDPrFactor> {
  };
}