//
// Created by lars on 15.12.21.
//

#ifndef ONLINE_FGO_GPINTERPOLATEDDDPSEURA_H
#define ONLINE_FGO_GPINTERPOLATEDDDPSEURA_H

//#include "model/gp_interpolator/GPInterpolatorPose3WNOA.h"

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
namespace fgo::factors {
  class GPInterpolatedDDPseuRaFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3,
          gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> {

  private:
    //measurements
    double dDPseuRa_; //PseudoRange Measurement
    double ddDRange_;
    gtsam::Vector3 omega1_; //angular Rate of time i already corrected
    gtsam::Vector3 omega2_; // angular rate of time j already corrected
    gtsam::Point3 angularRate_;
    //double tau_;
    //constants
    gtsam::Point3 lb_; //position of antenna in body frame
    gtsam::Point3 lb2_;
    //points
    gtsam::Point3 pointMaster_; //position Master Satellite
    gtsam::Point3 velMaster_;
    gtsam::Point3 pointSatI_;  //position Satellite I
    gtsam::Point3 velSatI_;
    gtsam::Point3 pointBase_; //position of Base

    fgo::models::GaussianProcessInterpolatorPose3WNOA GPbase_; //Interpolator

    typedef GPInterpolatedDDPseuRaFactor This;
    typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> Interpolator;

  public:

    GPInterpolatedDDPseuRaFactor() {}

    //when meausrment are already known when build
    GPInterpolatedDDPseuRaFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i,
                                 const gtsam::Key &point_j, const gtsam::Key &vel_j,
                                 const double &PseuRaMR, const double &PseuRaMB,
                                 const double &PseuRaIR, const double &PseuRaIB,
                                 const double &dRangeMR, const double &dRangeMB,
                                 const double &dRangeIR, const double &dRangeIB,
                                 const gtsam::Point3 &pointMaster, const gtsam::Point3 &velMaster,
                                 const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &velSatI, const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &omega1, const gtsam::Point3 &omega2,
                                 const gtsam::Point3 &lb, const gtsam::Point3 &angularRate,
                                 const double &delta_t, const double &tau,
                                 const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
            Interpolator(model, point_i, vel_i, point_j, vel_j), omega1_(omega1), omega2_(omega2),
            angularRate_(angularRate), lb_(lb), pointMaster_(pointMaster), velMaster_(velMaster),
            pointSatI_(pointSatI), velSatI_(velSatI), pointBase_(pointBase), GPbase_(Qc_model, delta_t, tau) {
      dDPseuRa_ = (PseuRaMR - PseuRaMB) - (PseuRaIR - PseuRaIB);
      ddDRange_ = (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB);
    }

    //when measurements are not known when build and ddmeasurement already made
    GPInterpolatedDDPseuRaFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i,
                                 const gtsam::Key &point_j, const gtsam::Key &vel_j,
                                 const double &ddPseuRa, const gtsam::Point3 &pointMaster,
                                 const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &velSatI, const gtsam::Point3 &pointBase,
                                 const gtsam::Point3 &omega1, const gtsam::Point3 &omega2,
                                 const gtsam::Point3 &lb, const gtsam::Point3 &angularRate,
                                 const double &delta_t, const double &tau, const gtsam::SharedNoiseModel &model,
                                 const gtsam::SharedNoiseModel &Qc_model) :
            Interpolator(model, point_i, vel_i, point_j, vel_j), dDPseuRa_(ddPseuRa), omega1_(omega1), omega2_(omega2),
            angularRate_(angularRate), lb_(lb), pointMaster_(pointMaster), velMaster_(velMaster), pointSatI_(pointSatI),
            velSatI_(velSatI), pointBase_(pointBase), GPbase_(Qc_model, delta_t, tau) {}

    GPInterpolatedDDPseuRaFactor(const gtsam::Key &point_i, const gtsam::Key &vel_i,
                                 const gtsam::Key &point_j, const gtsam::Key &vel_j,
                                 const double &ddPseuRa, const gtsam::Point3 &pointMaster,
                                 const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI,
                                 const gtsam::Point3 &velSatI,
                                 const gtsam::Point3 &omega1, const gtsam::Point3 &omega2,
                                 const gtsam::Point3 &lb,
                                 const gtsam::Point3 &angularRate,
                                 const double &delta_t,
                                 const double &tau, const gtsam::SharedNoiseModel &model,
                                 const gtsam::SharedNoiseModel &Qc_model, const gtsam::Point3 &lb2) :
            Interpolator(model, point_i, vel_i, point_j, vel_j), dDPseuRa_(ddPseuRa), omega1_(omega1),
            omega2_(omega2), angularRate_(angularRate), lb_(lb), lb2_(lb2), pointMaster_(pointMaster),
            velMaster_(velMaster), pointSatI_(pointSatI), velSatI_(velSatI), GPbase_(Qc_model, delta_t, tau) {}

    ~GPInterpolatedDDPseuRaFactor() = default;

    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1,
                                const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none,
                                boost::optional<gtsam::Matrix &> H3 = boost::none,
                                boost::optional<gtsam::Matrix &> H4 = boost::none) const override {

      gtsam::Pose3 poseBody; //interpolated position
      gtsam::Vector3 velBody;
      gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P; //matrices for interpolate position
      gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V; //matrices for interpolate position
      gtsam::Point3 lbv, lbv2;
      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1_, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2_, vel2).finished();
      //get the interpolated position of tau, where the measurement was made
      if (H1 || H2 || H3 || H4) {
        poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                           Hint1_P, Hint2_P, Hint3_P, Hint4_P);
        velBody = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2,
                                              Hint1_P, Hint2_P, Hint3_P, Hint4_P).block<3, 1>(3, 0);
      } else {
        poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
        velBody = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2).block<3, 1>(3, 0);
      }

      double rangeRM, rangeRI, rangeBM, rangeBI;
      double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
      gtsam::Matrix13 e_RM, e_RI; //unity vector pointing from receiver to satellite (1x3)
      gtsam::Matrix13 e_BM, e_BI = gtsam::Matrix13(0, 0, 0);
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
        rangeBI = gtsam::distance3(pointBase_, pointSatI_, e_RM);

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

      if (H1 || H2 || H3 || H4) {
        gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
        gtsam::Matrix16 dPrdX = (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                                                         (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lb2_),
                (e_RM - e_RI) - (e_BM - e_BI)).finished();
        gtsam::Matrix16 dDrdX = (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lbv) -
                                                      (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lbv2), 0,0,0).finished();
        gtsam::Matrix13 dDrdVel = (gtsam::Matrix13() << (e_RM - e_RI) - (e_BM - e_BI)).finished();
        //calculate Jacobian for poseReceiver
        gtsam::Matrix66 correction1 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction2 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
        Hint1_P = correction * Hint1_P * correction1;
        Hint1_V = Hint1_V * correction1;
        Hint3_P = correction * Hint3_P * correction2;
        Hint3_V = Hint3_V * correction2;
        //cut H2 and H4 because omega is not variable
        Hint2_P = Hint2_P.block<6,3>(0,3); Hint2_V = Hint2_V.block<6,3>(0,3);
        Hint4_P = Hint4_P.block<6,3>(0,3); Hint4_V = Hint4_V.block<6,3>(0,3);

        if (H1) *H1 = (gtsam::Matrix26() << dPrdX * Hint1_P, dDrdX * Hint1_P + dDrdVel * Hint1_V).finished(); //pose1
        if (H2) *H2 = (gtsam::Matrix23() << dPrdX * Hint2_P, dDrdX * Hint2_P + dDrdVel * Hint2_V).finished(); //vel1
        if (H3) *H3 = (gtsam::Matrix26() << dPrdX * Hint3_P, dDrdX * Hint3_P + dDrdVel * Hint3_V).finished(); //pose2
        if (H4) *H4 = (gtsam::Matrix23() << dPrdX * Hint4_P, dDrdX * Hint4_P + dDrdVel * Hint4_V).finished(); //vel2
      }

      std::cout << "H1:" << Hint1_P << std::endl << std::endl;
      std::cout << "H2:" << Hint2_P << std::endl << std::endl;
      std::cout << "H3:" << Hint3_P << std::endl << std::endl;
      std::cout << "H4:" << Hint4_P << std::endl << std::endl;


      return (gtsam::Vector2() << (rangeRM - rangeBM) - (rangeRI - rangeBI) - dDPseuRa_,
              (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB) - ddDRange_).finished();
    }

    gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1,
                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2) const {

      gtsam::Pose3 poseBody; //interpolated position
      gtsam::Vector3 velBody;
      gtsam::Point3 lbv, lbv2;
      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1_, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2_, vel2).finished();

      //get the interpolated position of tau, where the measurement was made
      poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
      velBody = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2).block<3, 1>(3, 0);

      double rangeRM, rangeRI, rangeBM, rangeBI;
      double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
      gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
      gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth
      gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI;
      //calc vel
      lbv = gtsam::skewSymmetric((-lb_)) * angularRate_;
      gtsam::Point3 velRec = velBody + poseBody.rotation().rotate(lbv);

      //ranges
      if (lb2_.norm() == 0) {
        rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
        rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
        rangeBM = gtsam::distance3(pointBase_, pointMaster_, e_BM);
        rangeBI = gtsam::distance3(pointBase_, pointSatI_, e_RM);

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

        lbv2 = gtsam::skewSymmetric((-lb2_)) * angularRate_;
        gtsam::Point3 velRec2 = velBody + poseBody.rotation().rotate(lbv2);
        dRangeMR = e_RM * (velRec - velMaster_);
        dRangeIR = e_RI * (velRec - velSatI_);
        dRangeMB = e_BM * (velRec2 - velMaster_);
        dRangeIB = e_BI * (velRec2 - velSatI_);
      }

      return (gtsam::Vector2() << (rangeRM - rangeBM) - (rangeRI - rangeBI) - dDPseuRa_,
              (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB) - ddDRange_).finished();
    }


    /** return the measured */
    double measured() const {
      return dDPseuRa_;
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

#endif //ONLINE_FGO_GPINTERPOLATEDDDPSEURA_H

namespace gtsam {
  template<>
  struct traits<fgo::factors::GPInterpolatedDDPseuRaFactor> :
          public Testable<fgo::factors::GPInterpolatedDDPseuRaFactor> {
  };
}