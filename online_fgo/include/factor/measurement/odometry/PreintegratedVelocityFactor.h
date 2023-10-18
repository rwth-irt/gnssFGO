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


#ifndef ONLINE_FGO_PREINTEGRATEDVELOCITYFACTOR_H
#define ONLINE_FGO_PREINTEGRATEDVELOCITYFACTOR_H

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include "factor/motion/VelocityPreintegration.h"
#include "factor/FactorTypeIDs.h"

/* External or standard includes */

#include <ostream>
namespace fgo::factor
{
    class PreintegratedVelocityMeasurements: public VelocityPreintegration
    {
        friend class PreintegratedVelocityFactor;
        friend class GPInterpolatedDoublePreintegratedVelocityFactor;
        friend class GPInterpolatedSinglePreintegratedVelocityFactor;

    protected:
        gtsam::Matrix9 preintMeasCov_;

    public:
        PreintegratedVelocityMeasurements()
        {
          preintMeasCov_.setZero();
        }

        PreintegratedVelocityMeasurements(const std::shared_ptr<VelocityPreintegrationParams>& p) : VelocityPreintegration(p)
        {
          preintMeasCov_.setZero();
        }

        ~PreintegratedVelocityMeasurements() override = default;

        void resetIntegration() override{
          VelocityPreintegration::resetIntegration();
          preintMeasCov_.setZero();
        }

        void integrateMeasurement(const gtsam::Vector3& measuredVel,
                                  const gtsam::Vector3& measuredOmegaIncrement, const double dt) override
        {
          if (dt <= 0) {
            throw std::runtime_error(
                "PreintegratedVelocityMeasurements::integrateMeasurement: dt <=0");
          }

          // Update preintegrated measurements (also get Jacobian)
          gtsam::Matrix9 A;  // overall Jacobian wrt preintegrated measurements (df/dx)
          gtsam::Matrix93 B, C;
          VelocityPreintegration::update(measuredVel, measuredOmegaIncrement, dt, &A, &B, &C);

          const gtsam::Matrix3& velCov = p().velocityCovariance;
          const gtsam::Matrix3& oICov = p().angularIncrementCovariance;
          const gtsam::Matrix3& iCov = p().integrationCovariance;

          // (1/dt) allows to pass from continuous time noise to discrete time noise
          // Update the uncertainty on the state (matrix A in [4]).
          preintMeasCov_ = A * preintMeasCov_ * A.transpose();
          // These 2 updates account for uncertainty on the IMU measurement (matrix B in [4]).
          preintMeasCov_.noalias() += B * (velCov / dt) * B.transpose();
          preintMeasCov_.noalias() += C * (oICov / dt) * C.transpose();

          // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt), with Gi << Z_3x3, I_3x3, Z_3x3 (9x3 matrix)
          preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
        }

        /// Add multiple measurements, in matrix columns
        void integrateMeasurements(const gtsam::Matrix& measuredAccs, const gtsam::Matrix& measuredOmegas,
                                   const gtsam::Matrix& dts)
        {
          auto n = static_cast<size_t>(dts.cols());
          for (size_t j = 0; j < n; j++) {
            integrateMeasurement(measuredAccs.col(j), measuredOmegas.col(j), dts(0, j));
          }
        }

    private:
        /// Serialization function
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          namespace bs = ::boost::serialization;
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(VelocityPreintegration);
          ar & BOOST_SERIALIZATION_NVP(preintMeasCov_);
        }
    };

  class PreintegratedVelocityFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>
  {
  private:
      typedef PreintegratedVelocityFactor This;
      typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> Base;

      PreintegratedVelocityMeasurements PIM_;

  public:
      typedef std::shared_ptr<PreintegratedVelocityFactor> shared_ptr;
      PreintegratedVelocityFactor() = default;

      PreintegratedVelocityFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key pose_j, gtsam::Key vel_j,
                                  const PreintegratedVelocityMeasurements& pim)
                                  : Base(gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i, pose_j, vel_j),
                                  PIM_(pim){}

      [[nodiscard]] NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<PreintegratedVelocityFactor>(
            NonlinearFactor::shared_ptr(new This(*this)));
      }

      friend std::ostream& operator<<(std::ostream& os, const PreintegratedVelocityFactor& f) {
        os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
        return os;
      }

      //------------------------------------------------------------------------------
      void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const override {
        std::cout << (s.empty() ? s : s + "\n") << "PreintegratedVelocityFactor(" << keyFormatter(this->key1())
             << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3())
             << "," << keyFormatter(this->key4())
             << ")\n";
        //std::cout << *this << std::endl;
      }

      [[nodiscard]] bool equals(const NonlinearFactor& other, double tol) const {
        const This *e = dynamic_cast<const This*>(&other);
        const bool base = Base::equals(*e, tol);
        const bool pim = true; //PIM_.equals(e->PIM_, tol);
        return e != nullptr && base && pim;
      }

      //------------------------------------------------------------------------------
      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
                                                const gtsam::Pose3& pose_j, const gtsam::Vector3& vel_j,
                                                boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
                                                boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {
        return PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j,
                                              H1, H2, H3, H4);
      }
  };

}
#endif //ONLINE_FGO_PREINTEGRATEDVELOCITYFACTOR_H
