//
// Created by lars on 10.02.22.
//

#ifndef ONLINE_FGO_GPINTERPOLATORPOSE3WNOJ_H
#define ONLINE_FGO_GPINTERPOLATORPOSE3WNOJ_H

#pragma once

#include "include/utils/GPutils.h"
#include "models/gp_interpolator/GPtemp.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>


namespace fgo {
    namespace models {
        class GPInterpolatorPose3WNOJTwRb { //Translation world/earth Rotation body

        private:
            typedef GPInterpolatorPose3WNOJTwRb This;
            double delta_t_;        // t_{i+1} - t_i
            double tau_;            // tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers
            gtsam::Matrix6 Qc_;
            fgo::utils::Matrix_18 Lambda_;
            fgo::utils::Matrix_18 Psi_;

        public:

            /// Default constructor: only for serialization
            GPInterpolatorPose3WNOJTwRb() {}

            /**
             * Constructor
             * @param Qc noise model of Qc
             * @param delta_t the time between the two states
             * @param tau the time of interval status
             */
            GPInterpolatorPose3WNOJTwRb(const gtsam::SharedNoiseModel &Qc_model, double delta_t, double tau) :
                    delta_t_(delta_t), tau_(tau) {
                // Calcuate Lambda and Psi
                Qc_ = fgo::utils::getQc(Qc_model);
                Lambda_ = fgo::utils::calcLambda3(Qc_, delta_t_, tau_);
                Psi_ = fgo::utils::calcPsi3(Qc_, delta_t_, tau_);
            }

            GPInterpolatorPose3WNOJTwRb(const This& interpolator): delta_t_(interpolator.delta_t_),
                                                                            tau_(interpolator.tau_){
                Qc_ = interpolator.Qc_;
                Lambda_ = fgo::utils::calcLambda3(Qc_, delta_t_, tau_);
                Psi_ = fgo::utils::calcPsi3(Qc_, delta_t_, tau_);
            }

            /** Virtual destructor */
            virtual ~GPInterpolatorPose3WNOJTwRb() {}

            /// interpolate pose with Jacobians
            gtsam::Pose3 interpolatePose(
                    const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
                    const gtsam::Vector3 &a1_b, const gtsam::Vector3 &g1_b,
                    const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
                    const gtsam::Vector3 &a2_b, const gtsam::Vector3 &g2_b,
                    gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 3> H2 = boost::none,
                    gtsam::OptionalJacobian<6, 6> H3 = boost::none, gtsam::OptionalJacobian<6, 3> H4 = boost::none) const {

              gtsam::Vector6 acc1 = fgo::utils::convertGbAbtoGbAw(g1_b, a1_b, pose1);
              gtsam::Vector6 acc2 = fgo::utils::convertGbAbtoGbAw(g2_b, a2_b, pose2);

              if (H1)
                *H1 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                        boost::bind(&This::interpolatePose_, this, boost::placeholders::_1, v1_n, omega1_b, acc1,
                                    pose2, v2_n, omega2_b, acc2), pose1, 1e-5);
              if (H2)
                *H2 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                        boost::bind(&This::interpolatePose_, this, pose1, boost::placeholders::_1, omega1_b, acc1,
                                    pose2, v2_n, omega2_b, acc2), v1_n, 1e-5);
              if (H3)
                *H3 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                        boost::bind(&This::interpolatePose_, this, pose1, v1_n, omega1_b, acc1,
                                    boost::placeholders::_1, v2_n, omega2_b, acc2), pose2, 1e-5);
              if (H4)
                *H4 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                        boost::bind(&This::interpolatePose_, this, pose1, v1_n, omega1_b, acc1,
                                    pose2, boost::placeholders::_1, omega2_b, acc2), v1_n, 1e-5);

                return interpolatePose_(pose1,v1_n,omega1_b,acc1,pose2,v2_n,omega2_b,acc2);
            }

          gtsam::Pose3 interpolatePose_( const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n,
                                         const gtsam::Vector3 &omega1_b, const gtsam::Vector6 &acc1,
                                         const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n,
                                         const gtsam::Vector3 &omega2_b, const gtsam::Vector6 &acc2) const {

            gtsam::Vector6 vel1 = (gtsam::Vector6() << omega1_b,v1_n).finished();
            gtsam::Vector6 vel2 = (gtsam::Vector6() << omega2_b,v2_n).finished();

            gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(pose1.rotation().inverse() * pose2.rotation(),
                                           - pose1.translation() + pose2.translation()));

            //Could be a bit wrong, because of the velocity and rotation not getting exp
            //gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3,
            //        pose1.rotation().inverse().matrix() * pose2.rotation().matrix()).finished();
            //vel2 = vel2 * correction; here with acc too

            const fgo::utils::Vector_18 r1 = (fgo::utils::Vector_18() << gtsam::Vector6::Zero(), vel1, acc1).finished();
            const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, vel2, acc2).finished();

            gtsam::Pose3 poseRel = gtsam::Pose3::Expmap(Lambda_.block<6, 18>(0, 0) * r1 +
                                                Psi_.block<6, 18>(0, 0) * r2);
            gtsam::Pose3 pose(pose1.rotation()*poseRel.rotation(), pose1.translation() + poseRel.translation());

            return pose;
            }

            /// update jacobian based on interpolated jacobians
            static void updatePoseJacobians(const gtsam::Matrix &Hpose,
                                            const gtsam::Matrix &Hint1, const gtsam::Matrix &Hint2,
                                            const gtsam::Matrix &Hint3,
                                            const gtsam::Matrix &Hint4,
                                            boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2,
                                            boost::optional<gtsam::Matrix &> H3, boost::optional<gtsam::Matrix &> H4) {
                if (H1) *H1 = Hpose * Hint1;
                if (H2) *H2 = Hpose * Hint2;
                if (H3) *H3 = Hpose * Hint3;
                if (H4) *H4 = Hpose * Hint4;
            }

            /// interpolate velocity with Jacobians

            gtsam::Vector6 interpolateVelocity(
                    const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n, const gtsam::Vector3 &omega1_b,
                    const gtsam::Vector3 &a1_b, const gtsam::Vector3 &g1_b,
                    const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n, const gtsam::Vector3 &omega2_b,
                    const gtsam::Vector3 &a2_b, const gtsam::Vector3 &g2_b,
                    gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 3> H2 = boost::none,
                    gtsam::OptionalJacobian<6, 6> H3 = boost::none, gtsam::OptionalJacobian<6, 3> H4 = boost::none) const {

              gtsam::Vector6 acc1 = fgo::utils::convertGbAbtoGbAw(g1_b, a1_b, pose1);
              gtsam::Vector6 acc2 = fgo::utils::convertGbAbtoGbAw(g2_b, a2_b, pose2);

              if (H1)
                *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                        boost::bind(&This::interpolateVelocity_, this, boost::placeholders::_1, v1_n, omega1_b, acc1,
                                    pose2, v2_n, omega2_b, acc2), pose1, 1e-5);
              if (H2)
                *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                        boost::bind(&This::interpolateVelocity_, this, pose1, boost::placeholders::_1, omega1_b, acc1,
                                    pose2, v2_n, omega2_b, acc2), v1_n, 1e-5);
              if (H3)
                *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                        boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, omega1_b, acc1,
                                    boost::placeholders::_1, v2_n, omega2_b, acc2), pose2, 1e-5);
              if (H4)
                *H4 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                        boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, omega1_b, acc1,
                                    pose2, boost::placeholders::_1, omega2_b, acc2), v1_n, 1e-5);

              return interpolateVelocity_(pose1,v1_n,omega1_b,acc1,pose2,v2_n,omega2_b,acc2);
            }

          gtsam::Vector6 interpolateVelocity_( const gtsam::Pose3 &pose1, const gtsam::Vector3 &v1_n,
                                         const gtsam::Vector3 &omega1_b, const gtsam::Vector6 &acc1,
                                         const gtsam::Pose3 &pose2, const gtsam::Vector3 &v2_n,
                                         const gtsam::Vector3 &omega2_b, const gtsam::Vector6 &acc2) const {

            gtsam::Vector6 vel1 = (gtsam::Vector6() << omega1_b,v1_n).finished();
            gtsam::Vector6 vel2 = (gtsam::Vector6() << omega2_b,v2_n).finished();

            gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(pose1.rotation().inverse() * pose2.rotation(),
                                                                 - pose1.translation() + pose2.translation()));

            //Could be a bit wrong, because of the velocity and rotation not getting exp
            //gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3,
            //        pose1.rotation().inverse().matrix() * pose2.rotation().matrix()).finished();
            //vel2 = vel2 * correction; here with acc too

            const fgo::utils::Vector_18 r1 = (fgo::utils::Vector_18() << gtsam::Vector6::Zero(), vel1, acc1).finished();
            const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, vel2, acc2).finished();

            gtsam::Vector6 vel = Lambda_.block<6, 18>(6, 0) * r1 + Psi_.block<6, 18>(6, 0) * r2;

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
            void print(const std::string &s = "") const {
                std::cout << s << "GPInterpolatorPose3WNOJTwRb" << std::endl;
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
    }
}

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::models::GPInterpolatorPose3WNOJTwRb> : public Testable<
            fgo::models::GPInterpolatorPose3WNOJTwRb> {};
}


#endif //ONLINE_FGO_GPINTERPOLATORPOSE3WNOJ_H
