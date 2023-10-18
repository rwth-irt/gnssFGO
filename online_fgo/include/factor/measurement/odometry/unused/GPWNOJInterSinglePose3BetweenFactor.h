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

//
// Created by haoming on 24.07.22.
//

#ifndef ONLINE_FGO_GPWNOJINTESINGLERBETWEENFACTORO_H
#define ONLINE_FGO_GPWNOJINTESINGLERBETWEENFACTORO_H
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "include/data/FactorTypes.h"

#ifdef _WIN32
#define BETWEENFACTOR_VISIBILITY
#else
// This will trigger a LNKxxxx on MSVC, so disable for MSVC build
// Please refer to https://github.com/borglab/gtsam/blob/develop/Using-GTSAM-EXPORT.md
#define BETWEENFACTOR_VISIBILITY GTSAM_EXPORT
#endif

namespace fgo::factor {
    /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */

class GPWNOJInterSinglePose3BetweenFactor : public fgo::NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                                                          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                                                          gtsam::Pose3> {
    public:

    private:

        typedef GPWNOJInterSinglePose3BetweenFactor This;
        typedef fgo::NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                       gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                       gtsam::Pose3>  Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
        GPBase GPbasePose_;
        gtsam::Pose3 measured_; /** The measurement */
        bool pose2Interpolated_{};

    public:

        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<GPInterpolatedSinglePose3BetweenFactor> shared_ptr;

        /** default constructor - only use for serialization */
        GPWNOJInterSinglePose3BetweenFactor() = default;

        /** Constructor */
        GPWNOJInterSinglePose3BetweenFactor(gtsam::Key pose1i, gtsam::Key vel1i, gtsam::Key omega1i, gtsam::Key acc1i,
                                            gtsam::Key pose1j, gtsam::Key vel1j, gtsam::Key omega1j, gtsam::Key acc1j,
                                            gtsam::Key pose2,
                                            const gtsam::Pose3 &measured, bool pose2Interpolated,
                                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolatorPose,
                                            const gtsam::SharedNoiseModel &model = nullptr) :
            Base(model, pose1i, vel1i, omega1i, acc1i, pose1j, vel1j, omega1j, acc1j, pose2), measured_(measured),
            pose2Interpolated_(pose2Interpolated), GPbasePose_(interpolatorPose){
        }

        ~GPWNOJInterSinglePose3BetweenFactor() override = default;

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
          std::cout << s << "GPInterpolatedSinglePose3BetweenFactor("
                    << keyFormatter(this->key1()) << ","
                    << keyFormatter(this->key2()) << ","
                    << keyFormatter(this->key3()) << ","
                    << keyFormatter(this->key4()) << ")\n";
          gtsam::traits<gtsam::Pose3>::Print(measured_, "  measured: ");
          this->noiseModel_->print("  noise model: ");
        }

        /// assert equality up to a tolerance
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol) && gtsam::traits<gtsam::Pose3>::Equals(this->measured_, e->measured_, tol);
        }

        /// @}
        /// @name NoiseModelFactor2 methods
        /// @{

        /// evaluate error, returns vector of errors size of tangent space
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &p1i, const gtsam::Vector3& v1i, const gtsam::Vector3& omega1i, const gtsam::Vector6& acc1i,
                                                  const gtsam::Pose3 &p1j, const gtsam::Vector3& v1j, const gtsam::Vector3& omega1j, const gtsam::Vector6& acc1j,
                                                  const gtsam::Pose3 &p2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none,
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none,
                                    boost::optional<gtsam::Matrix &> H6 = boost::none,
                                    boost::optional<gtsam::Matrix &> H7 = boost::none,
                                    boost::optional<gtsam::Matrix &> H8 = boost::none,
                                    boost::optional<gtsam::Matrix &> H9 = boost::none) const override {

          using namespace gtsam;
          using namespace fgo::utils;

          Pose3 poseInter;

          gtsam::Matrix Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P, Hint17_P, Hint18_P,
                        Hpose1, Hpose2;

          if(H1 || H2 || H3 || H4 || H5 || H6 || H7 || H8)
          {
            poseInter = GPbasePose_->interpolatePose(p1i, v1i, omega1i, acc1i, p1j, v1j, omega1j, acc1j,
                                                     Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P, Hint17_P, Hint18_P);
          }
          else
            poseInter = GPbasePose_->interpolatePose(p1i, v1i, omega1i, acc1i, p1j, v1j, omega1j, acc1j);

          gtsam::Pose3 hx;
          if(pose2Interpolated_)
             hx = gtsam::traits<gtsam::Pose3>::Between(p2, poseInter, &Hpose2, &Hpose1); // h(x)
          else
             hx = gtsam::traits<gtsam::Pose3>::Between(poseInter, p2, &Hpose1, &Hpose2); // h(x)

          // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
          typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlocal;
          gtsam::Vector rval = gtsam::traits<gtsam::Pose3>::Local(measured_, hx, boost::none, &Hlocal);

         // std::cout << "pose2Interpolated_" << pose2Interpolated_ << std::endl;
         // std::cout << "poseInter: " << poseInter << std::endl;
          //std::cout << "pose: " << p2 << std::endl;
         // std::cout << "hx: " << hx << std::endl;
         // std::cout << "measured_: " << measured_ << std::endl;
         // std::cout << "rval: " << rval << std::endl;
          if (H1) *H1 = Hlocal * Hpose1 * Hint11_P;
          if (H2) *H2 = Hlocal * Hint12_P;
          if (H3) *H3 = Hlocal * Hint13_P;
          if (H4) *H4 = Hlocal * Hint14_P;
          if (H5) *H5 = Hlocal * Hpose1 * Hint15_P;
          if (H6) *H6 = Hlocal * Hint16_P;
          if (H7) *H7 = Hlocal * Hint17_P;
          if (H8) *H8 = Hlocal * Hint18_P;
          if (H9) *H9 = Hlocal * Hpose2;

          //gtsam::Vector6 zero = (gtsam::Vector6() << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001).finished();

          return rval;
#else
          return traits<gtsam::Pose3>::Local(measured_, hx);
#endif
        }

        /// @}
        /// @name Standard interface
        /// @{

        /// return the measurement
        [[nodiscard]] const gtsam::Pose3 &measured() const {
          return measured_;
        }
        /// @}

    private:

        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
          ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                              boost::serialization::base_object<Base>(*this));
          ar & BOOST_SERIALIZATION_NVP(measured_);
        }

        // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
        enum {
            NeedsToAlign = (sizeof(gtsam::Pose3) % 16) == 0
        };
    public:
        GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
    }; // \class BetweenFactor



    //class GPIterBetweenConstraint : public GPIterPose3BetweenFactor {
    //public:
    //    typedef boost::shared_ptr<GPIterBetweenConstraint > shared_ptr;

        /** Syntactic sugar for constrained version */
    //    GPIterBetweenConstraint(const gtsam::Pose3 &measured, gtsam::Key key1, gtsam::Key key2, double mu = 1000.0) :
    //        GPIterPose3BetweenFactor(key1, key2, measured,
    //                                   gtsam::noiseModel::Constrained::All(gtsam::traits<gtsam::Pose3>::GetDimension(measured),
    //                                                                       std::abs(mu))) {}

    //private:

        /** Serialization function */
    //    friend class boost::serialization::access;

    //    template<class ARCHIVE>
    //    void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    //      ar & boost::serialization::make_nvp("BetweenFactor",
    //                                          boost::serialization::base_object<GPIterPose3BetweenFactor<VALUE> >(*this));
    //    }
    //}; // \class BetweenConstraint
}

namespace gtsam {
/// traits
    template<>
    struct traits<fgo::factor::GPWNOJInterSinglePose3BetweenFactor> : public gtsam::Testable<fgo::factor::GPWNOJInterSinglePose3BetweenFactor> {};

/**
 * Binary between constraint - forces between to a given value
 * This constraint requires the underlying type to a Lie type
 *
 */
    /// traits
   // struct gtsam::traits<fgo::factor::GPIterBetweenConstraint> : public gtsam::Testable<fgo::factor::GPIterBetweenConstraint> {};
}




#endif //ONLINE_FGO_GPWNOJINTESINGLERBETWEENFACTORO_H
