//
// Created by lars on 01.03.22.
//

#ifndef ONLINE_FGO_GPINTERPRDRFACTORWNOJO_H
#define ONLINE_FGO_GPINTERPRDRFACTORWNOJO_H

#include <utility>
#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "include/data/FactorTypes.h"

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

namespace fgo::factors {

  class GPInterPRDRFactorWNOJO : public fgo::NoiseModelFactor7<gtsam::Pose3,
          gtsam::Vector3, gtsam::Vector3, gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  private:


    double measRho_; /** measurement */
    double measdRho_;
    gtsam::Vector3 angularRate_;

    gtsam::Point3 lb_; ///< The pose of the sensor in the body frame
    gtsam::Vector3 satXYZ_;
    gtsam::Vector3 satVEL_;
    double tau_;

    typedef GPInterPRDRFactorWNOJO This;
    typedef NoiseModelFactor7 <gtsam::Pose3,
    gtsam::Vector3, gtsam::Vector3, gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;
    typedef models::GPInterpolatorPose3WNOJO GPBase;

    // interpolater
    GPBase GPbase_;
  public:

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    GPInterPRDRFactorWNOJO() = default; /* Default constructor */

    /**
     * Constructor
     * @param body_P_sensor transformation from body to sensor
     */
    GPInterPRDRFactorWNOJO(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &omega_i,
                           const gtsam::Key &cbd_i,
                           const gtsam::Key &pose_j, const gtsam::Key &vel_j, const gtsam::Key &omega_j,
                           const double &measRho, const double &measdRho, const gtsam::Point3 &satXYZ,
                           const gtsam::Point3 &satVEL, gtsam::Point3 &lb,
                           const gtsam::Vector3 &angularRate,
                           const gtsam::SharedNoiseModel &model,
                           const models::GPInterpolatorPose3WNOJO& interpolator) :
            Interpolator(model, pose_i, vel_i, omega_i, cbd_i, pose_j, vel_j, omega_j), measRho_(measRho),
            measdRho_(measdRho), angularRate_(angularRate), lb_(lb),
            satXYZ_(satXYZ), satVEL_(satVEL), tau_(interpolator.getTau()), GPbase_(interpolator) {}

    ~GPInterPRDRFactorWNOJO() override = default;

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** factor error */
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                const gtsam::Vector2 &cbd1,
                                const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none,
                                boost::optional<gtsam::Matrix &> H3 = boost::none,
                                boost::optional<gtsam::Matrix &> H4 = boost::none,
                                boost::optional<gtsam::Matrix &> H5 = boost::none,
                                boost::optional<gtsam::Matrix &> H6 = boost::none,
                                boost::optional<gtsam::Matrix &> H7 = boost::none) const override {
      using namespace gtsam;

      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      // interpolate pose
      Matrix Hint1, Hint2, Hint3, Hint4;
      Matrix Hint1v, Hint2v, Hint3v, Hint4v;
      Pose3 pose;
      gtsam::Vector3 linVel;
      if (H1 || H2 || H3 || H5 || H6 || H7) {
        gtsam::Vector6 vel;
        pose = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                           Hint1, Hint2, Hint3, Hint4);
        vel = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2,
                                          Hint1v, Hint2v, Hint3v, Hint4v);
        linVel = vel.block<3, 1>(3, 0);
      } else {
        gtsam::Vector6 vel;
        pose = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
        vel = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2);
        linVel = vel.block<3, 1>(3, 0); //we only need linVel because angular vel was measured and is better
      }


      //error
      gtsam::Matrix13 e;
      gtsam::Point3 lbv = gtsam::skewSymmetric(-lb_) * angularRate_; //velocity through rotation of antenna
      gtsam::Point3 positionReceiver = pose.translation() + pose.rotation() * lb_;
      gtsam::Point3 velocityReceiver = linVel + pose.rotation() * lbv;
      double distance = gtsam::distance3(positionReceiver, satXYZ_, e); //distance between receiver and sat
      gtsam::Point3 velocityDifference = velocityReceiver - satVEL_;
      double err = distance + cbd1(0) + tau_ * cbd1(1) - measRho_;
      double errd = e * velocityDifference + cbd1(1) - measdRho_;

      //derivations
      if (H1 || H2 || H3 || H4 || H5 || H6 || H7) {
        //Derivation of e derivated by positionReceiver
        gtsam::Matrix33 dEdPosRec = (gtsam::Matrix33() <<
                                                       1 - e.x() * e.x(), -e.x() * e.y(), -e.x() * e.z(),
                -e.x() * e.y(), 1 - e.y() * e.y(), -e.y() * e.z(),
                -e.x() * e.z(), -e.y() * e.z(), 1 - e.z() * e.z()).finished();
        //Derivation of velocityReceiver derivated by Rotation of Pose
        gtsam::Matrix13 dVelRecdRot1 =
                (gtsam::Matrix13() << velocityDifference.transpose() * dEdPosRec /
                                      distance * pose.rotation().matrix() * gtsam::skewSymmetric(-lb_)).finished();
        gtsam::Matrix13 dDrdRot2 = (gtsam::Matrix13()
                << e * pose.rotation().matrix() * gtsam::skewSymmetric(-lbv)).finished();
        //Derivation of velocityReceiver derivated by Position of Pose
        gtsam::Matrix13 dVelRecdPos =
                velocityDifference.transpose() * dEdPosRec / distance *
                pose.rotation().matrix();
        //Derivation of error derivated by Pose
        gtsam::Matrix16 dPrdPose = (gtsam::Matrix16()
                << e * pose.rotation().matrix() * gtsam::skewSymmetric(-lb_), e).finished();
        //Derivation of derror(velocity) derivated by Pose
        //gtsam::Matrix16 dDrdPose = (gtsam::Matrix16() << dDrdRot2, 0, 0, 0).finished();
        gtsam::Matrix16 dDrdPose = (gtsam::Matrix16() << dVelRecdRot1 + dDrdRot2, dVelRecdPos).finished();
        gtsam::Matrix16 dDrdVel = (gtsam::Matrix16() << 0, 0, 0, e).finished(); //because the derivations should be same

        gtsam::Matrix66 correction1 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction2 = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
        gtsam::Matrix66 correction = (gtsam::Matrix66()
                << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose.rotation().matrix()).finished();
        //CORRECT MATRICES
        Hint1 = correction * Hint1 * correction1;
        Hint1v = Hint1v * correction1;

        Hint3 = correction * Hint3 * correction2;
        Hint3v = Hint3v * correction2;

        if (H1)
          *H1 = (gtsam::Matrix26() <<
                                   dPrdPose * Hint1, dDrdPose * Hint1 + dDrdVel * Hint1v).finished(); //pose1
        if (H2)
          *H2 = (gtsam::Matrix23() <<
                                   dPrdPose * Hint2.block<6, 3>(0, 3), dDrdPose * Hint2.block<6, 3>(0, 3)
                                                                       + dDrdVel *
                                                                         Hint2v.block<6, 3>(0, 3)).finished(); //vel1
        if (H3)
          *H3 = (gtsam::Matrix23() <<
                                   dPrdPose * Hint2.block<6, 3>(0, 0), dDrdPose * Hint2.block<6, 3>(0, 0)
                                                                       + dDrdVel *
                                                                         Hint2v.block<6, 3>(0, 0)).finished(); //omega1
        if (H4) *H4 = (gtsam::Matrix22() << 1, tau_, 0, 1).finished(); //cbd
        if (H5)
          *H5 = (gtsam::Matrix26() << dPrdPose * Hint3, dDrdPose * Hint3 + dDrdVel * Hint3v).finished(); //pose2
        if (H6)
          *H6 = (gtsam::Matrix23() <<
                                   dPrdPose * Hint4.block<6, 3>(0, 3), dDrdPose * Hint4.block<6, 3>(0, 3) + dDrdVel *
                                                                                                            Hint4v.block<6, 3>(
                                                                                                                    0,
                                                                                                                    3)).finished(); //vel2
        if (H7)
          *H7 = (gtsam::Matrix23() <<
                                   dPrdPose * Hint4.block<6, 3>(0, 0), dDrdPose * Hint4.block<6, 3>(0, 0) + dDrdVel *
                                                                                                            Hint4v.block<6, 3>(
                                                                                                                    0,
                                                                                                                    0)).finished(); //omega2
      }
      /*
      if (H1) {
        gtsam::Matrix66 correction = (gtsam::Matrix66() <<
                                                        gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        *H1 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, cbd1, pose2, vel2, omega2), pose1) * correction;
      }

      if (H2) *H2 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector3>(
              boost::bind(&This::evaluateError_, this,pose1, boost::placeholders::_1, omega1, cbd1, pose2, vel2, omega2), vel1);
      if (H3) *H3 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, boost::placeholders::_1, cbd1, pose2, vel2, omega2), omega1);
      if (H4) *H4 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, boost::placeholders::_1, pose2, vel2, omega2), cbd1);
      if (H5) {
        gtsam::Matrix66 correction = (gtsam::Matrix66() <<
                                                        gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
        *H5 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, cbd1, boost::placeholders::_1, vel2, omega2), pose2) * correction;
      }
      if (H6) *H6 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, cbd1, pose2, boost::placeholders::_1, omega2), vel2);
      if (H7) *H7 = gtsam::numericalDerivative11<gtsam::Vector2,gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this,pose1, vel1, omega1, cbd1, pose2, vel2, boost::placeholders::_1), omega2);

    */
      return gtsam::Vector2(err, errd);
      //return evaluateError_(pose1,vel1,omega1,cbd1,pose2,vel2,omega2);
    }

    gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                 const gtsam::Vector2 &cbd1,
                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                 const gtsam::Vector3 &omega2) const {
      using namespace gtsam;

      gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      // interpolate pose
      Pose3 pose;
      gtsam::Vector3 linVel;
      gtsam::Vector6 vel;
      pose = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
      vel = GPbase_.interpolateVelocity(pose1, omVel1, pose2, omVel2);
      linVel = vel.block<3, 1>(3, 0); //we only need linVel because angular vel was measured and is better


      //error
      gtsam::Matrix13 e;
      gtsam::Point3 lbv = gtsam::skewSymmetric(-lb_) * angularRate_; //velocity through rotation of antenna
      gtsam::Point3 positionReceiver = pose.translation() + pose.rotation() * lb_;
      gtsam::Point3 velocityReceiver = linVel + pose.rotation() * lbv;
      double distance = gtsam::distance3(positionReceiver, satXYZ_, e); //distance between receiver and sat
      gtsam::Point3 velocityDifference = velocityReceiver - satVEL_;
      double err = distance + cbd1(0) + tau_ * cbd1(1) - measRho_;
      double errd = e * velocityDifference + cbd1(1) - measdRho_;

      return gtsam::Vector2(err, errd);
    }

    /** return the measured */
    gtsam::Vector2 measured() const {
      return gtsam::Vector2(measRho_, measdRho_);
    }

    /** number of variables attached to this factor */
    size_t size() const {
      return 7;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != NULL && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->measRho_, this->measdRho_).finished(),
                                          (gtsam::Vector2() << e->measRho_, e->measdRho_).finished(), tol);
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
      ar & boost::serialization::make_nvp("GPInterpolatedPRDRFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measRho_);
    }
  }; // GnssFactor

} //namespace


/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factors::GPInterPRDRFactorWNOJO> :
          public Testable<fgo::factors::GPInterPRDRFactorWNOJO> {
  };
}

#endif //ONLINE_FGO_GPINTERPRDRFACTORWNOJO_H
