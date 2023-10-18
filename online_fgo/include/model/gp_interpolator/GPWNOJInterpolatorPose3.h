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

#ifndef ONLINE_FGO_GPWNOJINTERPOLATORPOSE3_H
#define ONLINE_FGO_GPWNOJINTERPOLATORPOSE3_H

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>

#include "utils/NavigationTools.h"
#include "utils/Pose3utils.h"
#include "utils/GPutils.h"
#include "GPInterpolatorBase.h"


namespace fgo::models {
    class GPWNOJInterpolatorPose3 : public GPInterpolator{ //Translation world/earth Rotation body

    private:
      fgo::utils::Matrix_18 Lambda_;
      fgo::utils::Matrix_18 Psi_;
      gtsam::Vector6 accI_;
      gtsam::Vector6 accJ_;
      typedef GPWNOJInterpolatorPose3 This;

    public:
      /// Default constructor: only for serialization
      GPWNOJInterpolatorPose3() : GPInterpolator() {}

      /**
       * Constructor
       * @param Qc noise model of Qc
       * @param delta_t the time between the two states
       * @param tau the time of interval status
       */
      explicit GPWNOJInterpolatorPose3(const gtsam::SharedNoiseModel &Qc_model, double delta_t = 0.0, double tau = 0.0, bool useAutoDiff = false, bool calcJacobian = true) :
              GPInterpolator(fgo::utils::getQc(Qc_model), delta_t,tau,useAutoDiff,calcJacobian) {
        // Calcuate Lambda and Psi
        Lambda_ = fgo::utils::calcLambda3(Qc_, delta_t_, tau_);
        Psi_ = fgo::utils::calcPsi3(Qc_, delta_t_, tau_);
      }

      GPWNOJInterpolatorPose3(const This &interpolator) :
              GPInterpolator(interpolator.Qc_, interpolator.delta_t_, interpolator.tau_) {
        Lambda_ = fgo::utils::calcLambda3(Qc_, delta_t_, tau_);
        Psi_ = fgo::utils::calcPsi3(Qc_, delta_t_, tau_);
        useAutoDiff_ = interpolator.useAutoDiff_;
        calcJacobian_ = interpolator.calcJacobian_;
      }

      /** Virtual destructor */
      ~GPWNOJInterpolatorPose3() override = default;

      void recalculate(const double &delta_t, const double &tau,
                       const gtsam::Vector6 &accI, const gtsam::Vector6 &accJ) override {
        Lambda_ = fgo::utils::calcLambda3(Qc_, delta_t, tau);
        Psi_ = fgo::utils::calcPsi3(Qc_, delta_t, tau);

        accI_ = accI;
        accJ_ = accJ;

        update(delta_t, tau);
      }

      [[nodiscard]] double getTau() const override {
        return tau_;
      }

      /// interpolate pose with Jacobians

      [[nodiscard]] gtsam::Pose3 interpolatePose(
          const gtsam::Pose3& pose1, const gtsam::Vector3& v1_n, const gtsam::Vector3& omega1_b,
          const gtsam::Pose3& pose2, const gtsam::Vector3& v2_n, const gtsam::Vector3& omega2_b,
          boost::optional<gtsam::Matrix &> H1 = boost::none,
          boost::optional<gtsam::Matrix &> H2 = boost::none,
          boost::optional<gtsam::Matrix &> H3 = boost::none,
          boost::optional<gtsam::Matrix &> H4 = boost::none,
          boost::optional<gtsam::Matrix &> H5 = boost::none,
          boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                boost::bind(&This::interpolatePose_, this, boost::placeholders::_1, v1_n, omega1_b, pose2, v2_n, omega2_b),
                pose1, 1e-5);

          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                boost::bind(&This::interpolatePose_, this, pose1, boost::placeholders::_1, omega1_b, pose2, v2_n, omega2_b),
                v1_n, 1e-5);

          if (H3)
            *H3 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                boost::bind(&This::interpolatePose_, this, pose1, v1_n, boost::placeholders::_1, pose2, v2_n, omega2_b),
                omega1_b, 1e-5);

          if (H4)
            *H4 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose3>(
                boost::bind(&This::interpolatePose_, this, pose1, v1_n, omega1_b, boost::placeholders::_1, v2_n, omega2_b),
                pose2, 1e-5);

          if (H5)
            *H5 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                boost::bind(&This::interpolatePose_, this, pose1, v1_n, omega1_b, pose2, boost::placeholders::_1, omega2_b ),
                v2_n, 1e-5);

          if (H6)
            *H6 = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Vector3>(
                boost::bind(&This::interpolatePose_, this, pose1, v1_n, omega1_b, pose2, v2_n, boost::placeholders::_1),
                omega2_b, 1e-5);

          return interpolatePose_(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b);
        }
        else
        {
          using namespace gtsam;
          using namespace fgo::utils;

          Matrix6 Hinv, Hcomp11, Hcomp12, Hlogmap1;
          Vector6 r;
          if(H1 || H4)
            r = Pose3::Logmap(pose1.inverse(&Hinv).compose(pose2, &Hcomp11, &Hcomp12), &Hlogmap1);
          else
            r = Pose3::Logmap(pose1.inverse().compose(pose2));

          Matrix63 H1v, H1w, H2v, H2w;
          Matrix6 H1p, H2p;

          Vector6 v1, v2;

          if(H2 || H3 || H5 || H6)
          {
            v1 = convertVwWbToVbWb(v1_n, omega1_b, pose1, &H1v, &H1w, &H1p);
            v2 = convertVwWbToVbWb(v2_n, omega2_b, pose2, &H2v, &H2w, &H2p);
          } else {
            v1 = convertVwWbToVbWb(v1_n, omega1_b, pose1);
            v2 = convertVwWbToVbWb(v2_n, omega2_b, pose2);
          }

          Matrix6 Jinv = gtsam::Matrix6::Identity();

          if(calcJacobian_) {
            Jinv = fgo::utils::rightJacobianPose3inv(r);
          }

          const gtsam::Vector6 JinvVel2 = Jinv * v2;
          const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
          const gtsam::Matrix3 X = gtsam::skewSymmetric(omega), Y = gtsam::skewSymmetric(rho);
          Matrix6 JinvVel2SkewMatrix = (gtsam::Matrix6() << X, gtsam::Matrix3::Zero(), Y,  X).finished();

          const fgo::utils::Vector_18 r1 = (fgo::utils::Vector_18() << gtsam::Vector6::Zero(), v1, accI_).finished();
          const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, Jinv * v2, -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ_).finished();

          Pose3 pose;
          if (H1 || H2 || H3 || H4 || H5 || H6) {
            const gtsam::Vector3 omegaV2 = v2.head(3), rhoV2 = v2.tail(3);
            const gtsam::Matrix3 XV2 = gtsam::skewSymmetric(omegaV2), YV2 = gtsam::skewSymmetric(rhoV2);
            const gtsam::Matrix6 V2SkewMatrix = (gtsam::Matrix6() << XV2, gtsam::Matrix3::Zero(),  YV2, XV2).finished();
            const gtsam::Matrix3 XA2 = gtsam::skewSymmetric(accJ_.head(3)), YA2 = gtsam::skewSymmetric(accJ_.tail(3));
            const gtsam::Matrix6 A2SkewMatrix = (gtsam::Matrix6() << XA2, gtsam::Matrix3::Zero(), YA2,  XA2).finished();
            Matrix6 Hcomp21, Hcomp22, Hlogmap2;

            const gtsam::Vector6 xi_i_tau = Lambda_.block<6, 18>(0, 0) * r1 + Psi_.block<6, 18>(0, 0) * r2;

            Matrix6 JinvXiTau = gtsam::Matrix6::Identity();
            if(calcJacobian_)
            {
              JinvXiTau = fgo::utils::leftJacobianPose3(xi_i_tau);
            }

            pose = pose1.compose(Pose3::Expmap(xi_i_tau, &Hlogmap2), &Hcomp21, &Hcomp22);

            Matrix6 Hexpr1 = Hcomp22 * Hlogmap2;
            Matrix6 Hvel1 = Hexpr1 * Lambda_.block<6, 6>(0, 6) * JinvXiTau;
            Matrix6 Hvel2 = Hexpr1 * (Psi_.block<6, 6>(0, 6) * JinvXiTau + (-0.5) * Psi_.block<6, 6>(0, 12) * JinvXiTau * (JinvVel2SkewMatrix - V2SkewMatrix)) * Jinv;
           // Matrix6 Hacc1 = Hexpr1 * Lambda_.block<6, 6>(0, 12);

            if (H1) {
              const Matrix6 J_Ti = Hlogmap1 * Hcomp11 * Hinv;
              gtsam::Matrix6 Jdiff_Ti = gtsam::Matrix6::Zero();
              gtsam::Matrix6 Jdiff_TAi = gtsam::Matrix6::Zero();

              if (calcJacobian_) {
                Matrix6 dr = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, v2);
                Matrix_12_6 dr_dT1 = (Matrix_12_6() << Matrix6::Identity(), dr).finished();
                Matrix6 dJinv_dA2 = 0.5 * A2SkewMatrix; //jacobianMethodNumercialDiff(rightJacobianPose3inv, r, accJ_) * tmp;
                Matrix6 dJinv_dV2 = 0.25 * V2SkewMatrix * V2SkewMatrix;
                *H1 = Hcomp21 +
                      Hvel1  * H1p +
                      Hexpr1 * Psi_.block<6, 12>(0, 0) * dr_dT1 * J_Ti +
                      Hexpr1 * Psi_.block<6, 6>(0, 12) * (dJinv_dV2 + dJinv_dA2) * J_Ti;
              }
              else
                *H1 = Hcomp21 + Hvel1 * H1p  + Hexpr1 * Psi_.block<6, 6>(0, 0) * J_Ti;
            }

            if (H2) *H2 = Hvel1 * H1v;
            if (H3) *H3 = Hvel1 * H1w;

            if (H4) {
              const gtsam::Matrix6 J_Tj = Hlogmap1 * Hcomp12;
              if (calcJacobian_) {
                const Matrix6 dr_dV21 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, v2);
                const Matrix_12_6 dr2_dT2 = (Matrix_12_6() << Matrix6::Identity(), dr_dV21).finished();
                const Matrix6 dJinv_dA2 = 0.5 * A2SkewMatrix; //jacobianMethodNumercialDiff(rightJacobianPose3inv, r, accJ_) * J_Tj;
                const Matrix6 dJinv_dV2 = 0.25 * V2SkewMatrix * V2SkewMatrix; //0.5 * (V2SkewMatrix * (dr_dV21 + Jinv * H2p)); // H2p * A2SkewMatrix * Jinv +

                *H4 = Hvel2 * H2p +
                      Hexpr1 * Psi_.block<6, 12>(0, 0) * dr2_dT2 * J_Tj +
                      Hexpr1 * Psi_.block<6, 6>(0, 12) * (dJinv_dV2 + dJinv_dA2) * J_Tj;
              }
              else
                *H4 = Hexpr1 * Psi_.block<6, 6>(0, 0) * J_Tj + Hvel2 * H2p;

            }
            if (H5) *H5 = Hvel2 * H2v;
            if (H6) *H6 = Hvel2 * H2w;

          } else
            pose = pose1.compose(gtsam::Pose3::Expmap(Lambda_.block<6, 18>(0, 0) * r1 + Psi_.block<6, 18>(0, 0) * r2));
          return pose;
        }
      }

      [[nodiscard]] gtsam::Pose3 interpolatePose_(const gtsam::Pose3& pose1, const gtsam::Vector3& v1_n, const gtsam::Vector3& omega1_b,
                                                  const gtsam::Pose3& pose2, const gtsam::Vector3& v2_n, const gtsam::Vector3& omega2_b) const override
       {

        gtsam::Vector6 r = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));

        gtsam::Vector6 v1 = fgo::utils::convertVwWbToVbWb(v1_n, omega1_b, pose1);
        gtsam::Vector6 v2 = fgo::utils::convertVwWbToVbWb(v2_n, omega2_b, pose2);

        gtsam::Matrix6 Jinv = (gtsam::Matrix6() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, gtsam::I_3x3).finished();
        if(calcJacobian_) {
          Jinv = fgo::utils::rightJacobianPose3inv(r);
        }
        const gtsam::Vector6 JinvVel2 = Jinv * v2;
        const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
        const gtsam::Matrix3 skew_v_omega = gtsam::skewSymmetric(omega), skew_vel_linear = gtsam::skewSymmetric(rho);
        // eq. (35) and (42) in https://arxiv.org/pdf/1809.06518.pdf
        //gtsam::Matrix6  JinvVel2SkewMatrix = (gtsam::Matrix6() << X, Y, gtsam::Matrix3::Zero(), X).finished();
        // here, we have v = [v_omega, v_lin]^T
        const auto JinvVel2SkewMatrix = (gtsam::Matrix6() << skew_v_omega, gtsam::Matrix3::Zero(), skew_vel_linear,  skew_v_omega).finished();
        const fgo::utils::Vector_18 r1 = (fgo::utils::Vector_18() << gtsam::Vector6::Zero(), v1, accI_).finished();
        const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, Jinv * v2, -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ_).finished();

        return pose1.compose(gtsam::Pose3::Expmap(Lambda_.block<6, 18>(0, 0) * r1 + Psi_.block<6, 18>(0, 0) * r2));
      }

      /// update jacobian based on interpolated jacobians
      static void updatePoseJacobians(const gtsam::Matrix& Hpose,
                                      const gtsam::Matrix& Hint1, const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3,
                                      const gtsam::Matrix& Hint4, const gtsam::Matrix& Hint5, const gtsam::Matrix& Hint6,
                                      boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
                                      boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4,
                                      boost::optional<gtsam::Matrix&> H5, boost::optional<gtsam::Matrix&> H6) {
        if (H1) *H1 = Hpose * Hint1;
        if (H2) *H2 = Hpose * Hint2;
        if (H3) *H3 = Hpose * Hint3;
        if (H4) *H4 = Hpose * Hint4;
        if (H5) *H5 = Hpose * Hint5;
        if (H6) *H6 = Hpose * Hint6;
      }

      /// interpolate velocity with Jacobians

      [[nodiscard]] gtsam::Vector6 interpolateVelocity(
          const gtsam::Pose3& pose1, const gtsam::Vector3& v1_n, const gtsam::Vector3& omega1_b,
          const gtsam::Pose3& pose2, const gtsam::Vector3& v2_n, const gtsam::Vector3& omega2_b,
          boost::optional<gtsam::Matrix &> H1 = boost::none,
          boost::optional<gtsam::Matrix &> H2 = boost::none,
          boost::optional<gtsam::Matrix &> H3 = boost::none,
          boost::optional<gtsam::Matrix &> H4 = boost::none,
          boost::optional<gtsam::Matrix &> H5 = boost::none,
          boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                boost::bind(&This::interpolateVelocity_, this, boost::placeholders::_1, v1_n, omega1_b, pose2, v2_n, omega2_b),
                pose1, 1e-5);

          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                boost::bind(&This::interpolateVelocity_, this, pose1, boost::placeholders::_1, omega1_b, pose2, v2_n, omega2_b),
                v1_n, 1e-5);

          if (H3)
            *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, boost::placeholders::_1, pose2, v2_n, omega2_b),
                omega1_b, 1e-5);

          if (H4)
            *H4 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, omega1_b, boost::placeholders::_1, v2_n, omega2_b),
                pose2, 1e-5);

          if (H5)
            *H5 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, omega1_b, pose2, boost::placeholders::_1, omega2_b ),
                v2_n, 1e-5);

          if (H6)
            *H6 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                boost::bind(&This::interpolateVelocity_, this, pose1, v1_n, omega1_b, pose2, v2_n, boost::placeholders::_1),
                omega2_b, 1e-5);

          return interpolateVelocity_(pose1, v1_n, omega1_b, pose2, v2_n, omega2_b);
        }
        else
        {
          using namespace gtsam;
          using namespace fgo::utils;
          Matrix6 Hinv, Hcomp11, Hcomp12, Hlogmap;
          Vector6 r;
          if (H1 || H4)
            r = Pose3::Logmap(pose1.inverse(&Hinv).compose(pose2, &Hcomp11, &Hcomp12), &Hlogmap);
          else
            r = Pose3::Logmap(pose1.inverse().compose(pose2));

          // vel
          Matrix63 H1v, H1w, H2v, H2w;
          Matrix6 H1p, H2p;
          Vector6 vel1, vel2;
          if (H2 || H3 || H5 || H6) {
            vel1 = convertVwWbToVbWb(v1_n, omega1_b, pose1, &H1v, &H1w, &H1p);
            vel2 = convertVwWbToVbWb(v2_n, omega2_b, pose2, &H2v, &H2w, &H2p);
          } else {
            vel1 = convertVwWbToVbWb(v1_n, omega1_b, pose1);
            vel2 = convertVwWbToVbWb(v2_n, omega2_b, pose2);
          }

          Matrix6 Jinv = (gtsam::Matrix6() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, gtsam::I_3x3).finished();
          Matrix6 JinvVel2SkewMatrix = gtsam::Matrix6::Identity();
          if(calcJacobian_) {
            Jinv = fgo::utils::rightJacobianPose3inv(r);
            const gtsam::Vector6 JinvVel2 = Jinv * vel2;
            const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
            const gtsam::Matrix3 X = gtsam::skewSymmetric(omega), Y = gtsam::skewSymmetric(rho);
            // JinvVel2SkewMatrix = (gtsam::Matrix6() <<X, Y, gtsam::Matrix3::Zero(), X).finished();  NOT THIS ONE
            JinvVel2SkewMatrix = (gtsam::Matrix6() << X, gtsam::Matrix3::Zero(),  Y, X).finished();
          }

          const gtsam::Vector6 xi_I_tau = Lambda_.block<6, 6>(0, 6) * vel1 + Lambda_.block<6, 6>(0, 12) * accI_ +
                                          Psi_.block<6, 6>(0, 0) * r + Psi_.block<6, 6>(0, 6) * Jinv * vel2 +
                                          Psi_.block<6, 6>(0, 12) * (-0.5 * JinvVel2SkewMatrix * vel2 + Jinv * accJ_);
          const gtsam::Vector6 xi_J_tau = Lambda_.block<6, 6>(6, 6) * vel1 + Lambda_.block<6, 6>(6, 12) * accI_ +
                                          Psi_.block<6, 6>(6, 0) * r + Psi_.block<6, 6>(6, 6) * Jinv * vel2 +
                                          Psi_.block<6, 6>(6, 12) * (-0.5 * JinvVel2SkewMatrix * vel2 + Jinv * accJ_);
          const gtsam::Matrix6 Jac_Xi_I_tau = fgo::utils::rightJacobianPose3(xi_I_tau);

          //  const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, Jinv * v2, -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ).finished();
          Vector6 vel = Jac_Xi_I_tau * xi_J_tau;
          if (H1 || H2 || H3 || H4 || H5 || H6)
          {
            const gtsam::Vector3 omegaV2 = vel2.head(3), rhoV2 = vel2.tail(3);
            const gtsam::Matrix3 XV2 = gtsam::skewSymmetric(omegaV2), YV2 = gtsam::skewSymmetric(rhoV2);
            const gtsam::Matrix6 V2SkewMatrix = (gtsam::Matrix6() << XV2, gtsam::Matrix3::Zero(), YV2,  XV2).finished();
            const gtsam::Matrix3 XA2 = gtsam::skewSymmetric(accJ_.head(3)), YA2 = gtsam::skewSymmetric(accJ_.tail(3));
            const gtsam::Matrix6 A2SkewMatrix = (gtsam::Matrix6() <<XA2, gtsam::Matrix3::Zero(), YA2,  XA2).finished();

            Matrix6 Hvel1 = Jac_Xi_I_tau * Lambda_.block<6, 6>(6, 6);
            Matrix6 Hvel2 = Jac_Xi_I_tau * (Psi_.block<6, 6>(6, 6) * Jinv + Psi_.block<6, 6>(6, 12) * -0.5 * (JinvVel2SkewMatrix - V2SkewMatrix * Jinv));
            //Matrix6 Hacc1 = Lambda_.block<6, 6>(12, 12);
            //Matrix6 Hacc2 = Psi_.block<6, 6>(12, 12) * Jinv;

            Matrix6 JinvJacobianVel2 = Matrix6::Zero(), DJinvJacobianVel2 = Matrix6::Zero();
            Matrix_18_6 dvel_dr = Matrix_18_6::Identity();

            if(calcJacobian_) {
              JinvJacobianVel2 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, vel2);
              DJinvJacobianVel2 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, xi_J_tau);
              Hvel1 += DJinvJacobianVel2 * Lambda_.block<6, 6>(0, 6);
              Hvel2 += DJinvJacobianVel2 * (Psi_.block<6, 6>(0, 6) * Jinv + Psi_.block<6, 6>(0, 12) * -0.5 * (JinvVel2SkewMatrix - V2SkewMatrix * Jinv));
              dvel_dr = (Matrix_18_6() << Matrix6::Identity(), JinvJacobianVel2, (0.25 * V2SkewMatrix * V2SkewMatrix + 0.5 * A2SkewMatrix)).finished();
            }

            const gtsam::Matrix dV2Acc = 0.5 * V2SkewMatrix * JinvVel2SkewMatrix;

            if (H1) {
              const Matrix6 JTi = Hlogmap * Hcomp11 * Hinv;
              if(calcJacobian_)
              {
                *H1 =  Jac_Xi_I_tau * Psi_.block<6, 18>(6, 0) * dvel_dr * JTi +
                       DJinvJacobianVel2 * Psi_.block<6, 18>(0, 0) * dvel_dr * JTi + Hvel1 * H1p;
              }
              else
                *H1 =  Jac_Xi_I_tau * Psi_.block<6, 6>(6, 0) * JTi + Hvel1 * H1p;
            }

            if (H2) *H2 = Hvel1 * H1v;
            if (H3) *H3 = Hvel1 * H1w;

            if (H4) {
              const gtsam::Matrix6 JTj = Hlogmap * Hcomp12;
              if (calcJacobian_) {
                 *H4 = Hvel2 * H2p +
                     Jac_Xi_I_tau * Psi_.block<6, 18>(6, 0) * dvel_dr * JTj +
                     DJinvJacobianVel2 * Psi_.block<6, 18>(0, 0) * dvel_dr * JTj;
              }
              else
                *H4 = Jac_Xi_I_tau * Psi_.block<6, 6>(6, 0) * JTj + Hvel2 * H2p;
            }

            if (H5) *H5 = Hvel2 * H2v;
            if (H6) *H6 = Hvel2 * H2w;

          }
          return vel;
        }
      }

      [[nodiscard]] gtsam::Vector6 interpolateVelocity_(const gtsam::Pose3& pose1, const gtsam::Vector3& v1_n, const gtsam::Vector3& omega1_b,
                                                        const gtsam::Pose3& pose2, const gtsam::Vector3& v2_n, const gtsam::Vector3& omega2_b) const override {

        gtsam::Vector6 r = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));

        gtsam::Vector6 vel1 = fgo::utils::convertVwWbToVbWb(v1_n, omega1_b, pose1);
        gtsam::Vector6 vel2 = fgo::utils::convertVwWbToVbWb(v2_n, omega2_b, pose2);

        gtsam::Matrix6 Jinv = gtsam::Matrix6::Identity();
        if(calcJacobian_) {
          Jinv = fgo::utils::rightJacobianPose3inv(r);
        }

        const gtsam::Vector6 JinvVel2 = Jinv * vel2;
        const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
        const gtsam::Matrix3 skew_v_omega = gtsam::skewSymmetric(omega), skew_vel_linear = gtsam::skewSymmetric(rho);
        // eq. (35) and (42) in https://arxiv.org/pdf/1809.06518.pdf
        //gtsam::Matrix6  JinvVel2SkewMatrix = (gtsam::Matrix6() << X, Y, gtsam::Matrix3::Zero(), X).finished();

        const auto JinvVel2SkewMatrix = (gtsam::Matrix6() << skew_v_omega,  gtsam::Matrix3::Zero(), skew_vel_linear, skew_v_omega).finished();

        const gtsam::Vector6 xi_I_tau = Lambda_.block<6, 6>(0, 6) * vel1 + Lambda_.block<6, 6>(0, 12) * accI_ +
                                        Psi_.block<6, 6>(0, 0) * r + Psi_.block<6, 6>(0, 6) * Jinv * vel2 +
                                        Psi_.block<6, 6>(0, 12) * (-0.5 * JinvVel2SkewMatrix * vel2 + Jinv * accJ_);
        const gtsam::Vector6 xi_J_tau = Lambda_.block<6, 6>(6, 6) * vel1 + Lambda_.block<6, 6>(6, 12) * accI_ +
                                        Psi_.block<6, 6>(6, 0) * r + Psi_.block<6, 6>(6, 6) * Jinv * vel2 +
                                        Psi_.block<6, 6>(6, 12) * (-0.5 * JinvVel2SkewMatrix * vel2 + Jinv * accJ_);
        const gtsam::Matrix6 Jac_Xi_I_tau = fgo::utils::rightJacobianPose3(xi_I_tau);

        // const fgo::utils::Vector_18 r2 = (fgo::utils::Vector_18() << r, Jinv * v2, -0.5 * JinvVel2SkewMatrix * v2 + Jinv * accJ).finished();
        return Jac_Xi_I_tau * xi_J_tau;
      }

      /**
      * Testables
      */

      /** equals specialized to this factor */
      [[nodiscard]] virtual bool equals(const This &expected, double tol = 1e-9) const {
        return fabs(this->delta_t_ - expected.delta_t_) < tol &&
               fabs(this->tau_ - expected.tau_) < tol &&
               gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
               gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
               gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
      }

      /** print contents */
      void print(const std::string &s = "") const override {
        std::cout << s << "GPWNOJInterpolatorPose3" << std::endl;
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

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::models::GPWNOJInterpolatorPose3> : public Testable<
          fgo::models::GPWNOJInterpolatorPose3> {
  };
}

#endif //ONLINE_FGO_GPWNOJINTERPOLATORPOSE3_H
