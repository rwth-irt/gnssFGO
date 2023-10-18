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


#ifndef ONLINE_FGO_VELOCITYPREINTEGRATION_H
#define ONLINE_FGO_VELOCITYPREINTEGRATION_H

#pragma once
#include <iosfwd>
#include <string>
#include <utility>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/PreintegratedRotation.h>

namespace fgo::factor
{
    struct VelocityPreintegrationParams : gtsam::PreintegratedRotationParams {

        gtsam::Matrix3 velocityCovariance;
        gtsam::Matrix3 angularIncrementCovariance;
        gtsam::Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty
        gtsam::Vector3 leverArmToRotationCenter;
        bool considerLeverArm = false;
    };

    class VelocityPreintegration
    {
#define D_R_R(H) (H)->block<3,3>(0,0)
#define D_R_t(H) (H)->block<3,3>(0,3)
#define D_R_v(H) (H)->block<3,3>(0,6)
#define D_t_R(H) (H)->block<3,3>(3,0)
#define D_t_t(H) (H)->block<3,3>(3,3)
#define D_t_v(H) (H)->block<3,3>(3,6)
#define D_v_R(H) (H)->block<3,3>(6,0)
#define D_v_t(H) (H)->block<3,3>(6,3)
#define D_v_v(H) (H)->block<3,3>(6,6)
    protected:
        gtsam::Vector9 preintegrated_;
        std::shared_ptr<VelocityPreintegrationParams> params_;
        gtsam::Matrix93 preintegrated_H_biasOmega_;  ///< Jacobian of preintegrated_ w.r.t. angular rate bias
    public:
        /// Time interval from i to j
        double deltaTij_{};

    public:
        VelocityPreintegration() = default;
        explicit VelocityPreintegration(const std::shared_ptr<VelocityPreintegrationParams>& p) : params_(p), deltaTij_(0.){}
        virtual ~VelocityPreintegration() = default;

        [[nodiscard]] VelocityPreintegrationParams& p() const {
          return *params_;
        }

        virtual void integrateMeasurement(const gtsam::Vector3& measuredVelBody,
                                          const gtsam::Vector3& measuredOmegaIncrement,
                                          const double dt)
        {
          gtsam::Matrix9 A;
          gtsam::Matrix93 B, C;
          update(measuredVelBody, measuredOmegaIncrement, dt, &A, &B, &C);
        }

        void update(const gtsam::Vector3& measuredVelBody, const gtsam::Vector3& measuredOmegaIncrement,
                    const double dt, gtsam::Matrix9* A, gtsam::Matrix93* B, gtsam::Matrix93* C)
        {

          const auto& velBody = measuredVelBody;
          // if(LIOParams_->considerLeverArm)
          //    velBody += gtsam::skewSymmetric(-LIOParams_->leverArmToRotationCenter) * measuredOmegaIncrement / dt;

          deltaTij_ += dt;

          const auto theta = preintegrated_.segment<3>(0);
          const auto position = preintegrated_.segment<3>(3);
          const auto velocity = preintegrated_.segment<3>(6);
          // This functor allows for saving computation when exponential map and its
          // derivatives are needed at the same location in so<3>
          gtsam::so3::DexpFunctor local(theta);

          gtsam::Matrix3 w_tangent_H_theta, invH;
          const gtsam::Vector3 oi_tangent = // angular increment mapped back to tangent space
              local.applyInvDexp(measuredOmegaIncrement, A ? &w_tangent_H_theta : 0, C ? &invH : 0);
          const gtsam::Rot3 R(local.expmap());  // nRb: rotation of body in nav frame
          const gtsam::Vector3 vel_nav = R * velBody;

          preintegrated_ = (gtsam::Vector9() << theta + oi_tangent,
              position + velocity * dt,
              vel_nav).finished();

          if (A) {
            // Exact derivative of R*a with respect to theta:
            A->setIdentity();
            A->block<3, 3>(0, 0).noalias() += w_tangent_H_theta;  // theta
            A->block<3, 3>(3, 0).setZero();  // position wrpt theta...
            A->block<3, 3>(3, 6) = gtsam::I_3x3 * dt;            // .. and velocity
            A->block<3, 3>(6, 0) = R.matrix();    // velocity wrpt theta
          }
          if (B) {  // derivatives to measured v_b
            B->block<3, 3>(0, 0) = gtsam::Z_3x3;
            B->block<3, 3>(3, 0).setZero();
            B->block<3, 3>(6, 0) = R.matrix();
          }
          if (C) {  // derivatives to measured oI
            C->block<3, 3>(0, 0) = invH;
            C->block<3, 3>(3, 0) = gtsam::Z_3x3;
            C->block<3, 3>(6, 0) = gtsam::Z_3x3;
          }
          preintegrated_H_biasOmega_ = (*A) * preintegrated_H_biasOmega_ - (*C);
        }

        [[nodiscard]] gtsam::NavState predict(const gtsam::NavState& state_i,
                                              gtsam::OptionalJacobian<9, 9> H1) const
        {
          // Correct for initial velocity and gravity
          gtsam::Matrix9 D_delta_state;

          const gtsam::Rot3& nRb = state_i.attitude();
          const gtsam::Velocity3& n_v = state_i.velocity();

          gtsam::Vector9 xi;
          gtsam::Matrix3 D_dP_Ri1, D_dP_Ri2, D_dP_nv, D_dV_Ri;
          gtsam::NavState::dR(xi) = gtsam::NavState::dR(preintegrated_);
          gtsam::NavState::dP(xi) = gtsam::NavState::dP(preintegrated_) + deltaTij_ * nRb.unrotate(n_v, H1 ? &D_dP_Ri1 : nullptr);
          gtsam::NavState::dV(xi) = gtsam::NavState::dV(preintegrated_);

          gtsam::Matrix9 D_predict_state, D_predict_delta;

          gtsam::NavState state_j = state_i.retract(xi,
                                                    H1 ? &D_predict_state : nullptr,
                                                    H1 ? &D_predict_delta : nullptr);
          if (H1) {
            if (H1) {
              gtsam::Matrix3 Ri = nRb.matrix();
              D_delta_state.setZero(); // if coriolis H1 is already initialized
              D_t_R(&D_delta_state) += deltaTij_ * D_dP_Ri1;
              D_t_v(&D_delta_state) += deltaTij_ * D_dP_nv * Ri;
              D_v_R(&D_delta_state) += deltaTij_ * D_dV_Ri;

              *H1 = D_predict_state + D_predict_delta * D_delta_state;
            }
          }
        }

        //------------------------------------------------------------------------------
        [[nodiscard]] gtsam::Vector9 computeError(const gtsam::NavState& state_i,
                                                  const gtsam::NavState& state_j,
                                                  gtsam::OptionalJacobian<9, 9> H1,
                                                  gtsam::OptionalJacobian<9, 9> H2) const {
          // Predict state at time j
          gtsam::Matrix9 D_predict_state_i;
          gtsam::NavState predictedState_j = predict(
              state_i, H1 ? &D_predict_state_i : 0);

          // Calculate error
          gtsam::Matrix9 D_error_state_j, D_error_predict;
          gtsam::Vector9 error = state_j.localCoordinates(predictedState_j, H2 ? &D_error_state_j : 0,
                                                          H1 ? &D_error_predict : 0);

          if (H1) *H1 << D_error_predict * D_predict_state_i;
          if (H2) *H2 << D_error_state_j;
          return error;
        }

        [[nodiscard]] gtsam::Vector9 computeErrorAndJacobians(const gtsam::Pose3& pose_i,
                                                              const gtsam::Vector3& vel_i, const gtsam::Pose3& pose_j, const gtsam::Vector3& vel_j,
                                                              gtsam::OptionalJacobian<9, 6> H1, gtsam::OptionalJacobian<9, 3> H2,
                                                              gtsam::OptionalJacobian<9, 6> H3, gtsam::OptionalJacobian<9, 3> H4) const {
          gtsam::NavState state_i(pose_i, vel_i);
          gtsam::NavState state_j(pose_j, vel_j);
          // Predict state at time j
          gtsam::Matrix9 D_error_state_i, D_error_state_j;
          gtsam::Vector9 error = computeError(state_i, state_j,
                                              H1 || H2 ? &D_error_state_i : 0, H3 || H4 ? &D_error_state_j : 0);

          // Separate out derivatives in terms of 5 arguments
          // Note that doing so requires special treatment of velocities, as when treated as
          // separate variables the retract applied will not be the semi-direct product in NavState
          // Instead, the velocities in nav are updated using a straight addition
          // This is difference is accounted for by the R().transpose calls below
          if (H1) *H1 << D_error_state_i.leftCols<6>();
          if (H2) *H2 << D_error_state_i.rightCols<3>() * state_i.R().transpose();
          if (H3) *H3 << D_error_state_j.leftCols<6>();
          if (H4) *H4 << D_error_state_j.rightCols<3>() * state_j.R().transpose();
          return error;
        }
        virtual void resetIntegration()
        {
          deltaTij_ = 0.0;
          preintegrated_.setZero();
        }

        [[nodiscard]] const gtsam::Vector9& preintegrated() const { return preintegrated_; }


    };


}
#endif //ONLINE_FGO_VELOCITYPREINTEGRATION_H
