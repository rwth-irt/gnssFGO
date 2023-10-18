//
// Created by lars on 22.02.22.
//

#ifndef ONLINE_FGO_GPWNOAINTERPRFACTOR_H
#define ONLINE_FGO_GPWNOAINTERPRFACTOR_H
#pragma once
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>

#include "models/gp_interpolator/GPWNOAInterpolatorPose3.h"
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

namespace fgo::factor {

    class GPWNOAInterpolatedPrFactor : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
        gtsam::Vector2> {
    private:
      double measRho_; /** measurement */
      gtsam::Point3 lb_; ///< The pose of the sensor in the body frame
      gtsam::Vector3 satXYZ_;
      gtsam::Vector3 satVEL_;
      double tau_;
      bool useAutoDiff_ = false;

      typedef GPWNOAInterpolatedPrFactor This;
      typedef NoiseModelFactor7 <gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector2> Interpolator;
      typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;

      // interpolator
      GPBase GPbase_;
    public:

        GPWNOAInterpolatedPrFactor() = default; /* Default constructor */

      /**
       * Constructor
       * @param body_P_sensor transformation from body to sensor
       */
      GPWNOAInterpolatedPrFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key omega_i,
                                 gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key omega_j,
                                 gtsam::Key cbd_i,
                                 const double &measRho,
                                 const gtsam::Vector3 &satXYZ, const gtsam::Vector3 &satVEL, gtsam::Vector3 &lb,
                                 const gtsam::SharedNoiseModel &model,
                                 const std::shared_ptr<fgo::models::GPInterpolator> &interpolator, bool useAutoDiff = false) :
              Interpolator(model, pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, cbd_i), measRho_(measRho), lb_(lb),
              satXYZ_(satXYZ), satVEL_(satVEL), tau_(interpolator->getTau()), useAutoDiff_(useAutoDiff), GPbase_(interpolator){}

      ~GPWNOAInterpolatedPrFactor() override = default;

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
            *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1, omega1, pose2, vel2, omega2, cbd1), pose1);

          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1, omega1, pose2, vel2, omega2, cbd1), vel1);

          if (H3)
            *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, boost::placeholders::_1, pose2, vel2, omega2, cbd1), omega1);

          if (H4)
            *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, boost::placeholders::_1, vel2, omega2, cbd1), pose2);

          if (H5)
            *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1,  pose2, boost::placeholders::_1, omega2, cbd1), vel2);

          if (H6)
            *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, boost::placeholders::_1, cbd1), omega2);

          if (H7)
            *H7 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector2>(
                boost::bind(&This::evaluateError_, this, pose1, vel1, omega1, pose2, vel2, omega2, boost::placeholders::_1), cbd1);

          return evaluateError_(pose1, vel1, omega1, pose2, vel2, omega2, cbd1);
        }
        else
        {
          gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P;
          gtsam::Pose3 pose;

          if (H1 || H2 || H3 || H4 || H5 || H6) {
            pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2,
                                            Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
          }
          else {
            pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
          }

          gtsam::Matrix Hpose_P,Hpose_Pr,Hpose_r;     // jacobian of pose
          gtsam::Matrix3 Hrho_p;
          gtsam::Matrix13 Hd;

          gtsam::Point3 P_eA_e = pose.translation(&Hpose_P) + pose.rotation(&Hpose_Pr).rotate(lb_,&Hrho_p);
          double real_range = gtsam::distance3(P_eA_e, satXYZ_,&Hd);

          gtsam::Matrix H1_rho, H2_rho, H3_rho, H4_rho, H5_rho, H6_rho;

          if (H1 || H2 || H3 || H4 || H5 || H6) {
            gtsam::Matrix tmp_rho = Hrho_p * Hpose_Pr;
            H1_rho = Hd * (Hpose_P * Hint1_P + tmp_rho * Hint1_P);
            H2_rho = Hd * (Hpose_P * Hint2_P + tmp_rho * Hint2_P);
            H3_rho = Hd * (Hpose_P * Hint3_P + tmp_rho * Hint3_P);
            H4_rho = Hd * (Hpose_P * Hint4_P + tmp_rho * Hint4_P);
            H5_rho = Hd * (Hpose_P * Hint5_P + tmp_rho * Hint5_P);
            H6_rho = Hd * (Hpose_P * Hint6_P + tmp_rho * Hint6_P);
          }

          if(H1) *H1 = (gtsam::Matrix16() <<H1_rho, 0, 0, 0).finished();
          if(H2) *H2 = (gtsam::Matrix13() <<H2_rho).finished();
          if(H3) *H3 = (gtsam::Matrix13() <<H3_rho).finished();
          if(H4) *H4 = (gtsam::Matrix16() <<H4_rho, 0, 0, 0).finished();
          if(H5) *H5 = (gtsam::Matrix13() <<H5_rho).finished();
          if(H6) *H6 = (gtsam::Matrix13() <<H6_rho).finished();
          if(H7) *H7 = (gtsam::Matrix12() << 1, tau_, 0, 1).finished();

          return (gtsam::Vector1() << real_range + cbd1(0) + tau_* cbd1(1) - measRho_).finished();
        }

      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                                 const gtsam::Vector2 &cbd1) const {
        // get position
        gtsam::Pose3 pose = GPbase_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);
        gtsam::Point3 positionReceiver = pose.translation() + pose.rotation() * lb_;
        //error pos
        gtsam::Matrix13 e;
        double distance = gtsam::distance3(positionReceiver, satXYZ_, e); //distance between receiver and sat
        double err = distance + cbd1(0) + tau_ * cbd1(1) - measRho_;

        return gtsam::Vector1(err);
      }

      /** return the measured */
      [[nodiscard]] gtsam::Vector1 measured() const {
        return (gtsam::Vector1() << measRho_).finished();
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->measRho_).finished(),
                                            (gtsam::Vector1() << e->measRho_).finished(), tol);
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
    }; // PrDrFactor
  } //namespace


/// traits
  namespace gtsam {
    template<>
    struct traits<fgo::factor::GPWNOAInterpolatedPrFactor> :
            public Testable<fgo::factor::GPWNOAInterpolatedPrFactor> {
    };
  }

#endif //ONLINE_FGO_GPWNOAINTERPRDRFACTOR_H
