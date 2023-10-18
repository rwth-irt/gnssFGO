//
// Created by lars on 01.03.22.
//

#ifndef ONLINE_FGO_GPINTERTDCARPHAFACTORWNOJO_H
#define ONLINE_FGO_GPINTERTDCARPHAFACTORWNOJO_H

#include "include/data/FactorTypes.h"
#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"


/*Inputs:
 * Keys: pose of time i X(i)&X(j), velocity of time i&j V(i)&V(j), clock bias drift of time i C(i)
 * CarrierPhase measurements at i and j: carPhaMi, carPhaMj
 * Position of the satellite at i and j: pointSatM_i, pointSatM_j
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * delta_t: timedifference of state i & j taui & tauj: timedifference of state i & j and time of measurement
 * omega1&2: angularvelocity of time i & j
 * Covariance Matrix of the measurement/s: model & QcModel*/
/* measurement equation used:
 * lambda * (carrierphase j - carrierphase i) = Distance of Satellite and Receiver j - Distance of Satellite and Receiver j + clock bias j - clock bias i
 * somtimes the AmbiguityCycles get added, but its just a definition in DD measurements it doesnt matter because it can be neg anyway*/
/*Jacobian: e_RS * (R * skew(t), R) for i and j
 * */
//Lyu, Bai, Lai, Wang and Huang - Optimal Time Difference-based TDCP-GPS/IMU Navigation using Graph Optimization
namespace fgo::factors {
  //WHEN BOTH MEASUREMENTS ARE BETWEEN 2 STATES 1,2
  //Lyu, Bai, Lai, Wang and Huang - Optimal Time Difference-based TDCP-GPS/IMU Navigation using Graph Optimization
  class GPInterTDCPFactorWNOJO
          : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                  gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

  private:
    double lambda_;
    gtsam::Point3 lb_;
    //double carPhaMi_; //carPha Satellite (Master) at time i
    //double carPhaMj_; //carPha Satellite (Master) at time j
    double phi_ji;

    gtsam::Point3 pointMasteri_; //position Master Satellite
    gtsam::Point3 pointSatIi_; //position Satellite I
    gtsam::Point3 pointBase_; //position of Base
    gtsam::Point3 pointMasterj_; //position Master Satellite
    gtsam::Point3 pointSatIj_; //position Satellite I

    fgo::models::GPInterpolatorPose3WNOJO GPbasei_; //Interpolator
    fgo::models::GPInterpolatorPose3WNOJO GPbasej_; //Interpolator
    typedef GPInterTDCPFactorWNOJO This;
    typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;

  public:
    //Standard Constructor
    GPInterTDCPFactorWNOJO() = default;

    //Constructor for between two states
    GPInterTDCPFactorWNOJO(gtsam::Key point_1, gtsam::Key vel_1, gtsam::Key omega_1,
                           gtsam::Key point_2, gtsam::Key vel_2, gtsam::Key omega_2,
                           const double &ddphi_i, const double &ddphi_j,
                           const gtsam::Point3 &pointMaster_i, const gtsam::Point3 &pointSatI_i,
                           const gtsam::Point3 &pointMaster_j, const gtsam::Point3 &pointSatI_j,
                           const gtsam::Point3 &pointBase, const gtsam::Point3 &lb, const double &lambda,
                           const gtsam::SharedNoiseModel &model,
                           const models::GPInterpolatorPose3WNOJO& interpolator_i,
                           const models::GPInterpolatorPose3WNOJO& interpolator_j) :
            Interpolator(model, point_1, vel_1, omega_1, point_2, vel_2, omega_2),
            lambda_(lambda), lb_(lb), pointMasteri_(pointMaster_i),
            pointSatIi_(pointSatI_i), pointBase_(pointBase), pointMasterj_(pointMaster_j),
            pointSatIj_(pointSatI_j), GPbasei_(interpolator_i), GPbasej_(interpolator_j) {
      phi_ji = ddphi_j - ddphi_i;
    }

    virtual ~GPInterTDCPFactorWNOJO() {}

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
                                boost::optional<gtsam::Matrix &> H6 = boost::none
    ) const override {

      gtsam::Pose3 poseBodyi, poseBodyj; //interpolated position
      gtsam::Matrix Hint1_Pi, Hint2_Pi, Hint3_Pi, Hint4_Pi; //matrices for interpolate position
      gtsam::Matrix Hint1_Pj, Hint2_Pj, Hint3_Pj, Hint4_Pj; //get the interpolated position of tau, where the measurement was made

      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      if (H1 || H2 || H4 || H5) {
        poseBodyi = GPbasei_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                             Hint1_Pi, Hint2_Pi, Hint3_Pi, Hint4_Pi);
        poseBodyj = GPbasej_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                             Hint1_Pj, Hint2_Pj, Hint3_Pj, Hint4_Pj);
      } else {
        poseBodyi = GPbasei_.interpolatePose(pose1, omVel1, pose2, omVel2);
        poseBodyj = GPbasej_.interpolatePose(pose1, omVel1, pose2, omVel2);
      }

      //error
      gtsam::Point3 positionReceiveri = poseBodyi.translation() + poseBodyi.rotation() * lb_;
      gtsam::Point3 positionReceiverj = poseBodyj.translation() + poseBodyj.rotation() * lb_;
      gtsam::Matrix13 e_RMi, e_RIi, e_RMj, e_RIj;

      double rangeRMi = gtsam::distance3(positionReceiveri, pointMasteri_, e_RMi);
      double rangeRIi = gtsam::distance3(positionReceiveri, pointSatIi_, e_RIi);
      double rangeBMi = gtsam::distance3(pointBase_, pointMasteri_);
      double rangeBIi = gtsam::distance3(pointBase_, pointSatIi_);

      double rangeRMj = gtsam::distance3(positionReceiverj, pointMasterj_, e_RMj);
      double rangeRIj = gtsam::distance3(positionReceiverj, pointSatIj_, e_RIj);
      double rangeBMj = gtsam::distance3(pointBase_, pointMasterj_);
      double rangeBIj = gtsam::distance3(pointBase_, pointSatIj_);

      double rangei = (rangeRMi - rangeBMi) - (rangeRIi - rangeBIi);
      double rangej = (rangeRMj - rangeBMj) - (rangeRIj - rangeBIj);

      double error = rangej - rangei - lambda_ * phi_ji;

      if (H1 || H2 || H3 || H5 || H6) {
        gtsam::Matrix rotationi = poseBodyi.rotation().matrix(); // eRb (3x3)
        gtsam::Matrix rotationj = poseBodyj.rotation().matrix(); // eRb (3x3)
        gtsam::Matrix16 jacobiani =
                (gtsam::Matrix16() << (e_RMi - e_RIi) * rotationi * gtsam::skewSymmetric(-lb_), e_RMi -
                                                                                                e_RIi).finished(); //calculate Jacobian for Posei
        gtsam::Matrix16 jacobianj =
                (gtsam::Matrix16() << (e_RMj - e_RIj) * rotationj * gtsam::skewSymmetric(-lb_), e_RMj -
                                                                                                e_RIj).finished(); //calculate Jacobian for Posej

        //CORRECT MATRICES
        gtsam::Matrix66 correction1 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction2 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correctioni = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, rotationi.matrix()).finished();
        gtsam::Matrix66 correctionj = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, rotationj.matrix()).finished();
        Hint1_Pi = correctioni * Hint1_Pi * correction1;
        Hint3_Pi = correctioni * Hint3_Pi * correction2;
        Hint1_Pj = correctionj * Hint1_Pj * correction1;
        Hint3_Pj = correctionj * Hint3_Pj * correction2;

        if (H1) *H1 = (gtsam::Matrix16() << jacobianj * Hint1_Pj - jacobiani * Hint1_Pi).finished(); //pose1
        if (H2)
          *H2 = (gtsam::Matrix13() << jacobianj * Hint2_Pj.block<6, 3>(0, 3) -
                                      jacobiani * Hint2_Pi.block<6, 3>(0, 3)).finished(); //vel1
        if (H3)
          *H3 = (gtsam::Matrix13() << jacobianj * Hint2_Pj.block<6, 3>(0, 0) -
                                      jacobiani * Hint2_Pi.block<6, 3>(0, 0)).finished(); //omega1
        if (H4) *H4 = (gtsam::Matrix16() << jacobianj * Hint3_Pj - jacobiani * Hint3_Pi).finished(); //pose2
        if (H5)
          *H5 = (gtsam::Matrix13() << jacobianj * Hint4_Pj.block<6, 3>(0, 3) -
                                      jacobiani * Hint4_Pi.block<6, 3>(0, 3)).finished(); //vel2
        if (H6)
          *H6 = (gtsam::Matrix13() << jacobianj * Hint4_Pj.block<6, 3>(0, 0) -
                                      jacobiani * Hint4_Pi.block<6, 3>(0, 0)).finished(); //omega2
      }

      return (gtsam::Vector1(error));
    }

    /** return the measured */
    double measuredRho() const {
      return phi_ji;
    }

    double measureddRho() const {
      return phi_ji;
    }

    /** number of variables attached to this factor */
    size_t size() const {
      return 6;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != NULL && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->phi_ji).finished(),
                                          (gtsam::Vector1() << e->phi_ji).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "Gnssfactor specifically designed for septentrio" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor3",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(phi_ji);
    }

  };

}
#endif //ONLINE_FGO_GPINTERTDCARPHAFACTORWNOJO_H

  namespace gtsam {

    template<>
    struct traits<fgo::factors::GPInterTDCPFactorWNOJO> :
            public Testable<fgo::factors::GPInterTDCPFactorWNOJO> {
    };

  }