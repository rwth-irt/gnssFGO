//
// Created by xia on 14.04.21.
//

#ifndef FGONAV_GAUSSIANPROCESSINTERPOLATORPOSE3VNWB_H
#define FGONAV_GAUSSIANPROCESSINTERPOLATORPOSE3VNWB_H

/**
 * @file GaussianProcessInterpolatorPose3WNOA.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, SE(3). use Vn/Wb velocity representation
 * @author Jing Dong, Xinyan Yan
 */

#pragma once

#include "models/gp_interpolator/GPtemp.h"
#include "utils/GPutils.h"
#include <gtsam/base/numericalDerivative.h>
#include "include/models/gp_interpolator/GPInterpolatorBase.h"


namespace fgo {
  namespace models {

/**
 * 4-way factor for Gaussian Process interpolator, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 * interpolate pose and velocity given consecutive poses and velocities
 */
    class GaussianProcessInterpolatorPose3WNOA : public GPInterpolator {

    private:
      typedef GaussianProcessInterpolatorPose3WNOA This;
      //double delta_t_;        // t_{i+1} - t_i
      //double tau_;            // tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers
      //gtsam::Matrix6 Qc_;
      fgo::utils::Matrix_12 Lambda_;
      fgo::utils::Matrix_12 Psi_;

    public:

      /// Default constructor: only for serialization
      GaussianProcessInterpolatorPose3WNOA() : GPInterpolator() {}

      /**
       * Constructor
       * @param Qc noise model of Qc
       * @param delta_t the time between the two states
       * @param tau the time of interval status
       */
      explicit GaussianProcessInterpolatorPose3WNOA(const gtsam::SharedNoiseModel &Qc_model,  double delta_t = 0.0, double tau = 0.0) :
              GPInterpolator(fgo::utils::getQc(Qc_model), delta_t, tau) {
        // Calcuate Lambda and Psi
        Lambda_ = fgo::utils::calcLambda(Qc_, delta_t_, tau_);
        Psi_ = fgo::utils::calcPsi(Qc_, delta_t_, tau_);
      }

      GaussianProcessInterpolatorPose3WNOA(const This &interpolator) :
              GPInterpolator(interpolator.Qc_, interpolator.delta_t_, interpolator.tau_) {
        // Calcuate Lambda and Psi
        Lambda_ = fgo::utils::calcLambda(Qc_, delta_t_, tau_);
        Psi_ = fgo::utils::calcPsi(Qc_, delta_t_, tau_);
      }

      /** Virtual destructor */
      virtual ~GaussianProcessInterpolatorPose3WNOA() = default;

      void recalculate(const double &delta_t, const double &tau,
                       const gtsam::Vector6 &acc1_b = gtsam::Vector6(), const gtsam::Vector6 &acc2_b = gtsam::Vector6()) override {
        Lambda_ = fgo::utils::calcLambda(Qc_, delta_t, tau);
        Psi_ = fgo::utils::calcPsi(Qc_, delta_t, tau);
        update(delta_t, tau);
      }

      /// interpolate pose with Jacobians
      gtsam::Pose3 interpolatePose(
              const gtsam::Pose3 &pose1, const gtsam::Vector6 &omVel1,
              const gtsam::Pose3 &pose2, const gtsam::Vector6 &omVel2,
              gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 6> H2 = boost::none,
              gtsam::OptionalJacobian<6, 6> H3 = boost::none,
              gtsam::OptionalJacobian<6, 6> H4 = boost::none) const override {

        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                  boost::bind(&This::interpolatePose_, this, boost::placeholders::_1, omVel1,
                              pose2, omVel2), pose1, 1e-5);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector6>(
                  boost::bind(&This::interpolatePose_, this, pose1, boost::placeholders::_1,
                              pose2, omVel2), omVel1, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                  boost::bind(&This::interpolatePose_, this, pose1, omVel1,
                              boost::placeholders::_1, omVel2), pose2, 1e-5);
        if (H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector6>(
                  boost::bind(&This::interpolatePose_, this, pose1, omVel1,
                              pose2, boost::placeholders::_1), omVel2, 1e-5);

        return interpolatePose_(pose1, omVel1, pose2, omVel2);

      }

      gtsam::Pose3 interpolatePose_(
              const gtsam::Pose3 &pose1, const gtsam::Vector6 &omVel1,
              const gtsam::Pose3 &pose2, const gtsam::Vector6 &omVel2) const override {

        gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(pose1.rotation().inverse() * pose2.rotation(),
                                                             -pose1.translation() + pose2.translation()));

        //Could be a bit wrong, because of the velocity and rotation not getting exp
        //gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3,
        //        pose1.rotation().inverse().matrix() * pose2.rotation().matrix()).finished();
        //vel2 = vel2 * correction;

        const fgo::utils::Vector_12 r1 = (fgo::utils::Vector_12() << gtsam::Vector6::Zero(), omVel1).finished();
        const fgo::utils::Vector_12 r2 = (fgo::utils::Vector_12() << r, omVel2).finished();

        gtsam::Pose3 poseRel = gtsam::Pose3::Expmap(Lambda_.block<6, 12>(0, 0) * r1 +
                                                    Psi_.block<6, 12>(0, 0) * r2);
        gtsam::Pose3 pose(pose1.rotation() * poseRel.rotation(), pose1.translation() + poseRel.translation());

        return pose;

      }

      /// update jacobian based on interpolated jacobians
      static void updatePoseJacobians(const gtsam::Matrix &Hpose,
                                      const gtsam::Matrix &Hint1, const gtsam::Matrix &Hint2,
                                      const gtsam::Matrix &Hint3, const gtsam::Matrix &Hint4,
                                      boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2,
                                      boost::optional<gtsam::Matrix &> H3, boost::optional<gtsam::Matrix &> H4) {
        if (H1) *H1 = Hpose * Hint1;
        if (H2) *H2 = Hpose * Hint2;
        if (H3) *H3 = Hpose * Hint3;
        if (H4) *H4 = Hpose * Hint4;
      }

      /// interpolate velocity with Jacobians
      gtsam::Vector6 interpolateVelocity(
              const gtsam::Pose3 &pose1, const gtsam::Vector6 &omVel1,
              const gtsam::Pose3 &pose2, const gtsam::Vector6 &omVel2,
              gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 6> H2 = boost::none,
              gtsam::OptionalJacobian<6, 6> H3 = boost::none,
              gtsam::OptionalJacobian<6, 6> H4 = boost::none) const override {

        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                  boost::bind(&This::interpolateVelocity_, this, boost::placeholders::_1, omVel1,
                              pose2, omVel2), pose1, 1e-5);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
                  boost::bind(&This::interpolateVelocity_, this, pose1, boost::placeholders::_1,
                              pose2, omVel2), omVel1, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                  boost::bind(&This::interpolateVelocity_, this, pose1, omVel1,
                              boost::placeholders::_1, omVel2), pose2, 1e-5);
        if (H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
                  boost::bind(&This::interpolateVelocity_, this, pose1, omVel1,
                              pose2, boost::placeholders::_1), omVel2, 1e-5);

        return interpolateVelocity_(pose1, omVel1, pose2, omVel2);

      }

      gtsam::Vector6 interpolateVelocity_(
              const gtsam::Pose3 &pose1, const gtsam::Vector6 &omVel1,
              const gtsam::Pose3 &pose2, const gtsam::Vector6 &omVel2) const override {

        gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(pose1.rotation().inverse() * pose2.rotation(),
                                                             -pose1.translation() + pose2.translation()));

        //Could be a bit wrong, because of the velocity and rotation not getting exp
        //gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3,
        //        pose1.rotation().inverse().matrix() * pose2.rotation().matrix()).finished();
        //vel2 = vel2 * correction;

        const fgo::utils::Vector_12 r1 = (fgo::utils::Vector_12() << gtsam::Vector6::Zero(), omVel1).finished();
        const fgo::utils::Vector_12 r2 = (fgo::utils::Vector_12() << r, omVel2).finished();

        gtsam::Pose3 poseRel = gtsam::Pose3::Expmap(Lambda_.block<6, 12>(0, 0) * r1 +
                                                    Psi_.block<6, 12>(0, 0) * r2);

        gtsam::Vector6 vel = Lambda_.block<6, 12>(6, 0) * r1 + Psi_.block<6, 12>(6, 0) * r2;

        return vel;
      }



      /**
       * Testables
       */

      /** equals specialized to this factor */
      virtual bool equals(const This &expected, double tol = 1e-9) const {
        return fabs(this->delta_t_ - expected.delta_t_) < tol &&
               fabs(this->tau_ - expected.tau_) < tol &&
               gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
               gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
               gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
      }

      /** print contents */
      void print(const std::string &s = "") const override {
        std::cout << s << "GaussianProcessInterpolatorPose3VW" << std::endl;
        std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
        //std::cout << "Qc = " << Qc_ << std::endl;
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(delta_t_);
        ar & BOOST_SERIALIZATION_NVP(tau_);
        using namespace boost::serialization;
        ar & make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
        ar & make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
        ar & make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
      }
    };
  } //namespace solver
} // \ namespace fgonav


/// traits
namespace gtsam {
  template<>
  struct traits<fgo::models::GaussianProcessInterpolatorPose3WNOA> : public Testable<
          fgo::models::GaussianProcessInterpolatorPose3WNOA> {
  };
}

#endif //FGONAV_GAUSSIANPROCESSINTERPOLATORPOSE3VNWB_H