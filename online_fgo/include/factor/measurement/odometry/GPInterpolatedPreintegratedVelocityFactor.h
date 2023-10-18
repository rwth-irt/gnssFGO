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

#ifndef ONLINE_FGO_GPINTERPOLATEDPREINTEGRATEDVELOCITYFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDPREINTEGRATEDVELOCITYFACTOR_H

#pragma once
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "PreintegratedVelocityFactor.h"
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor
{
    class GPInterpolatedDoublePreintegratedVelocityFactor : public fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                                                                           gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    private:
        typedef GPInterpolatedDoublePreintegratedVelocityFactor This;
        typedef fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
        GPBase GPbasePose1_;
        GPBase GPbasePose2_;
        PreintegratedVelocityMeasurements PIM_;

    public:
        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<GPInterpolatedDoublePreintegratedVelocityFactor> shared_ptr;
        /** default constructor - only use for serialization */
        GPInterpolatedDoublePreintegratedVelocityFactor() = default;

        /** Constructor */
        GPInterpolatedDoublePreintegratedVelocityFactor(gtsam::Key pose1i, gtsam::Key vel1i, gtsam::Key omega1i,
                                                        gtsam::Key pose1j, gtsam::Key vel1j, gtsam::Key omega1j,
                                                        gtsam::Key pose2i, gtsam::Key vel2i, gtsam::Key omega2i,
                                                        gtsam::Key pose2j, gtsam::Key vel2j, gtsam::Key omega2j,
                                                        const PreintegratedVelocityMeasurements& pim,
                                                        const std::shared_ptr<fgo::models::GPInterpolator> &interpolatorPose1,
                                                        const std::shared_ptr<fgo::models::GPInterpolator> &interpolatorPose2)
        : Base(gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose1i, vel1i, omega1i, pose1j, vel1j, omega1j, pose2i, vel2i, omega2i, pose2j, vel2j, omega2j),
          GPbasePose1_(interpolatorPose1), GPbasePose2_(interpolatorPose2){}

        ~GPInterpolatedDoublePreintegratedVelocityFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /// @}
        /// @name Testable
        /// @{

        /// print with optional string

        void print(
            const std::string &s = "",
            const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
          std::cout << s << "GPInterpolatedDoublePreintegratedVelocityFactor("
                    << keyFormatter(this->key1()) << ","
                    << keyFormatter(this->key2()) << ","
                    << keyFormatter(this->key3()) << ","
                    << keyFormatter(this->key4()) << ")\n";
          //gtsam::traits<gtsam::Pose3>::Print(measured_, "  measured: ");
          this->noiseModel_->print("  noise model: ");
        }

        /// assert equality up to a tolerance
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol);// && gtsam::traits<gtsam::Pose3>::Equals(this->measured_, e->measured_, tol);
        }

        /// @}
        /// @name NoiseModelFactor12 methods
        /// @{

        /// evaluate error, returns vector of errors size of tangent space
        gtsam::Vector evaluateError(const gtsam::Pose3 &p1i, const gtsam::Vector3& v1i, const gtsam::Vector3& omega1i,
                                    const gtsam::Pose3 &p1j, const gtsam::Vector3& v1j, const gtsam::Vector3& omega1j,
                                    const gtsam::Pose3 &p2i, const gtsam::Vector3& v2i, const gtsam::Vector3& omega2i,
                                    const gtsam::Pose3 &p2j, const gtsam::Vector3& v2j, const gtsam::Vector3& omega2j,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none,
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none,
                                    boost::optional<gtsam::Matrix &> H6 = boost::none,
                                    boost::optional<gtsam::Matrix &> H7 = boost::none,
                                    boost::optional<gtsam::Matrix &> H8 = boost::none,
                                    boost::optional<gtsam::Matrix &> H9 = boost::none,
                                    boost::optional<gtsam::Matrix &> H10 = boost::none,
                                    boost::optional<gtsam::Matrix &> H11 = boost::none,
                                    boost::optional<gtsam::Matrix &> H12 = boost::none) const override {
          using namespace gtsam;
          using namespace fgo::utils;
          Pose3 pose1, pose2;
          Vector6 vel1, vel2;

          Matrix Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P,
                 Hint21_P, Hint22_P, Hint23_P, Hint24_P, Hint25_P, Hint26_P,
                Hint11_V, Hint12_V, Hint13_V, Hint14_V, Hint15_V, Hint16_V,
                Hint21_V, Hint22_V, Hint23_V, Hint24_V, Hint25_V, Hint26_V,
                 Hpose1, Hpose2, HPIMPos1, HPIMVel1, HPIMPos2, HPIMVel2, HPIMPose1Rot, HPIMVel1Rot, HPIMPose2Rot, HPIMVel2Rot;

          if(H1 || H2 || H3 || H4 || H5 || H6)
          {
            pose1 = GPbasePose1_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j,
                                                  Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P);
            vel1 = GPbasePose1_->interpolateVelocity(p1i, v1i, omega1i, p1j, v1j, omega1j, Hint11_V, Hint12_V, Hint13_V, Hint14_V, Hint15_V, Hint16_V);
          }
          else
          {
            pose1 = GPbasePose1_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j);
            vel1 = GPbasePose1_->interpolateVelocity(p1i, v1i, omega1i, p1j, v1j, omega1j);
          }

          if(H7 || H8 || H9 || H10 || H11 || H12)
          {
            pose2 = GPbasePose2_->interpolatePose(p2i, v2i, omega2i, p2j, v2j, omega2j,
                                                  Hint21_P, Hint22_P, Hint23_P, Hint24_P, Hint25_P, Hint26_P);
            vel2 = GPbasePose2_->interpolateVelocity(p2i, v2i, omega2i, p2j, v2j, omega2j,  Hint21_V, Hint22_V, Hint23_V, Hint24_V, Hint25_V, Hint26_V);
          }
          else
          {
            pose2 = GPbasePose2_->interpolatePose(p2i, v2i, omega2i, p2j, v2j, omega2j);
            vel2 = GPbasePose2_->interpolateVelocity(p2i, v2i, omega2i, p2j, v2j, omega2j);
          }

          const auto vel1LinearGlobal = pose1.rotation(HPIMPose1Rot).rotate(vel1.tail(3), HPIMVel1Rot);
          const auto vel2LinearGlobal = pose2.rotation(HPIMPose2Rot).rotate(vel2.tail(3), HPIMVel2Rot);
          const auto error = PIM_.computeErrorAndJacobians(pose1, vel1LinearGlobal, pose2, vel2LinearGlobal, HPIMPos1, HPIMVel1, HPIMPos2, HPIMVel2);

          // Shapes
          // HPIMPos1 \in \R^{6\times6}
          if(H1)  // shape: 9 x 6
          {
            gtsam::Matrix96 H1_ = HPIMPos1;
            H1_.block<6, 6>(0, 0) *= (Hint11_P + Hint11_V);
            *H1 << H1_;
          }


          if(H2)  // shape: 9 x 3
          {
            gtsam::Matrix93 H2_ = HPIMVel1;
            H2_.block<3, 3>(6, 6) *= (HPIMVel1Rot * Hint12_V.block<3, 3>(3, 3));
            H2_.block<3, 6>(0, 0) += (HPIMPose1Rot * Hint12_P);
            *H2 << H2_;
          }

          if(H3) // shape: 9 x 3
          {
            gtsam::Matrix93 H3_ =  gtsam::Matrix93::Zero();
            H3_.block<3, 6>(0, 0) += (HPIMPose1Rot * Hint13_P);
            *H3 << H3_;
          }

          if(H4) // shape: 9 x 6
          {
            gtsam::Matrix96 H4_ = HPIMPos1;
            H4_.block<6, 6>(0, 0) *= (Hint14_P + Hint14_V);
            *H4 << H4_;
          }

          if(H5) // shape: 9 x 3
          {
            gtsam::Matrix93 H5_ = HPIMVel1;
            H5_.block<3, 3>(6, 6) *= (HPIMVel1Rot * Hint15_V.block<3, 3>(3, 3));
            H5_.block<3, 6>(0, 0) += (HPIMPose1Rot * Hint15_P);
            *H5 << H5_;
          }

          if(H6) // shape: 9 x 3
          {
            gtsam::Matrix93 H6_ =  gtsam::Matrix93::Zero();
            H6_.block<3, 6>(0, 0) += (HPIMPose1Rot * Hint16_P);
            *H6 << H6_;
          }

          if(H7) // shape: 9 x 6
          {
            gtsam::Matrix96 H7_ = HPIMPos2;
            H7_.block<6, 6>(0, 0) *= (Hint21_P + Hint21_V);
            *H7 << H7_;
          }

          if(H8) // shape: 9 x 3
          {
            gtsam::Matrix93 H8_ = HPIMVel2;
            H8_.block<3, 3>(6, 6) *= (HPIMVel2Rot * Hint22_V.block<3, 3>(3, 3));
            H8_.block<3, 6>(0, 0) += (HPIMPose2Rot * Hint22_P);
            *H8 << H8_;
          }

          if(H9) // shape: 9 x 3
          {
            gtsam::Matrix93 H9_ =  gtsam::Matrix93::Zero();
            H9_.block<3, 6>(0, 0) += (HPIMPose2Rot * Hint23_P);
            *H9 << H9_;
          }

          if(H10) // shape: 9 x 6
          {
            gtsam::Matrix96 H10_ = HPIMPos2;
            H10_.block<6, 6>(0, 0) *= (Hint24_P + Hint24_V);
            *H10 << H10_;
          }

          if(H11) // shape: 9 x 3
          {
            gtsam::Matrix93 H11_ = HPIMVel2;
            H11_.block<3, 3>(6, 6) *= (HPIMVel2Rot * Hint25_V.block<3, 3>(3, 3));
            H11_.block<3, 6>(0, 0) += (HPIMPose2Rot * Hint25_P);
            *H11 << H11_;
          }

          if(H12) // shape: 9 x 3
          {
            gtsam::Matrix93 H12_ =  gtsam::Matrix93::Zero();
            H12_.block<3, 6>(0, 0) += (HPIMPose2Rot * Hint26_P);
            *H12 << H12_;
          }

          return error;
        }
    };

    class GPInterpolatedSinglePreintegratedVelocityFactor : public fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                                                                          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                                                                          gtsam::Pose3, gtsam::Vector3>
    {
    private:
        typedef GPInterpolatedSinglePreintegratedVelocityFactor This;
        typedef fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                       gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
        GPBase GPbase_;
        bool pose2Interpolated_{};
        PreintegratedVelocityMeasurements PIM_;

    public:
        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<GPInterpolatedSinglePreintegratedVelocityFactor> shared_ptr;
        /** default constructor - only use for serialization */
        GPInterpolatedSinglePreintegratedVelocityFactor() = default;

        /** Constructor */
        GPInterpolatedSinglePreintegratedVelocityFactor(gtsam::Key pose1i, gtsam::Key vel1i, gtsam::Key omega1i,
                                                        gtsam::Key pose1j, gtsam::Key vel1j, gtsam::Key omega1j,
                                                        gtsam::Key pose2, gtsam::Key vel2,
        bool pose2Interpolated,
        const PreintegratedVelocityMeasurements& pim,
        const std::shared_ptr<fgo::models::GPInterpolator> &interpolator)
        : Base(gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose1i, vel1i, omega1i, pose1j, vel1j, omega1j, pose2, vel2),
        pose2Interpolated_(pose2Interpolated), GPbase_(interpolator){}

        ~GPInterpolatedSinglePreintegratedVelocityFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /// @}
        /// @name Testable
        /// @{
        /// print with optional string

        void print(
            const std::string &s = "",
            const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
          std::cout << s << "GPInterpolatedSinglePreintegratedVelocityFactor("
                    << keyFormatter(this->key1()) << ","
                    << keyFormatter(this->key2()) << ","
                    << keyFormatter(this->key3()) << ","
                    << keyFormatter(this->key4()) << ")\n";
          //gtsam::traits<gtsam::Pose3>::Print(measured_, "  measured: ");
          this->noiseModel_->print("  noise model: ");
        }

        /// assert equality up to a tolerance
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol);// && gtsam::traits<gtsam::Pose3>::Equals(this->measured_, e->measured_, tol);
        }


        /// @}
        /// @name NoiseModelFactor8 methods
        /// @{

        /// evaluate error, returns vector of errors size of tangent space
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &p1i, const gtsam::Vector3& v1i, const gtsam::Vector3& omega1i,
                                                  const gtsam::Pose3 &p1j, const gtsam::Vector3& v1j, const gtsam::Vector3& omega1j,
                                                  const gtsam::Pose3 &p2, const gtsam::Vector3& v2,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H7 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
          using namespace gtsam;
          using namespace fgo::utils;

          Pose3 poseInterpolated;
          Vector6 velInterpolated;

          Matrix Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P,
                 Hint11_V, Hint12_V, Hint13_V, Hint14_V, Hint15_V, Hint16_V,
                 HPIMPos1, HPIMVel1, HPIMPos2, HPIMVel2, HPIMPoseRot, HPIMVelRot, HPIMPose2Rot, HPIMVel2Rot;

          if(H1 || H2 || H3 || H4 || H5 || H6)
          {
            poseInterpolated = GPbase_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j,
                                                        Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P);
            velInterpolated = GPbase_->interpolateVelocity(p1i, v1i, omega1i, p1j, v1j, omega1j, Hint11_V, Hint12_V, Hint13_V, Hint14_V, Hint15_V, Hint16_V);
          }
          else
          {
            poseInterpolated = GPbase_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j);
            velInterpolated = GPbase_->interpolateVelocity(p1i, v1i, omega1i, p1j, v1j, omega1j);
          }

          const auto velInterpolatedLinearGlobal = poseInterpolated.rotation(HPIMPoseRot).rotate(velInterpolated.tail(3), HPIMVelRot);

          Vector9 error;
          if(!pose2Interpolated_)
          {
            error = PIM_.computeErrorAndJacobians(poseInterpolated, velInterpolatedLinearGlobal, p2, v2, HPIMPos1, HPIMVel1, HPIMPos2, HPIMVel2);
          }
          else
          {
            error = PIM_.computeErrorAndJacobians(p2, v2, poseInterpolated, velInterpolatedLinearGlobal, HPIMPos2, HPIMVel2, HPIMPos1, HPIMVel1);
          }

          if(H1)
          {
            gtsam::Matrix96 H1_ = HPIMPos1;
            H1_.block<6, 6>(0, 0) *= (Hint11_P + Hint11_V);
            *H1 << H1_;
          }

          if(H2)
          {
            gtsam::Matrix93 H2_ = HPIMVel1;
            H2_.block<3, 3>(6, 6) *= (HPIMVelRot * Hint12_V.block<3, 3>(3, 3));
            H2_.block<3, 6>(0, 0) += (HPIMPoseRot * Hint12_P);
            *H2 << H2_;
          }

          if(H3)
          {
            gtsam::Matrix93 H3_ =  gtsam::Matrix93::Zero();
            H3_.block<3, 6>(0, 0) += (HPIMPoseRot * Hint13_P);
            *H3 << H3_;
          }

          if(H4)
          {
            gtsam::Matrix96 H4_ = HPIMPos1;
            H4_.block<6, 6>(0, 0) *= (Hint14_P + Hint14_V);
            *H4 << H4_;
          }

          if(H5)
          {
            gtsam::Matrix93 H5_ = HPIMVel1;
            H5_.block<3, 3>(6, 6) *= (HPIMVelRot * Hint15_V.block<3, 3>(3, 3));
            H5_.block<3, 6>(0, 0) += (HPIMPoseRot * Hint15_P);
            *H5 << H5_;
          }

          if(H6)
          {
            gtsam::Matrix93 H6_ =  gtsam::Matrix93::Zero();
            H6_.block<3, 6>(0, 0) += (HPIMPoseRot * Hint16_P);
            *H6 << H6_;
          }

          if(H7)
          {
            *H7 = HPIMPos2;
          }

          if(H8)
          {
            gtsam::Matrix93 H8_ = HPIMVel2;
            H8_.block<3, 6>(0, 0) += (HPIMPoseRot * HPIMPos2);
            *H8 << H8_;
          }
          return error;
        }
    };

}

#endif //ONLINE_FGO_GPINTERPOLATEDPREINTEGRATEDVELOCITYFACTOR_H
