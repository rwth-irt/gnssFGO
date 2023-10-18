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

#ifndef ONLINE_FGO_GPWNOAINTERDOUBLEBETWEENFACTORO_H
#define ONLINE_FGO_GPWNOAINTERDOUBLEBETWEENFACTORO_H

#pragma once
#include <ostream>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "model/gp_interpolator/GPWNOAInterpolatorPose3.h"
#include "data/FactorTypes.h"
#include "factor/FactorTypeIDs.h"


// This will trigger a LNKxxxx on MSVC, so disable for MSVC build
// Please refer to https://github.com/borglab/gtsam/blob/develop/Using-GTSAM-EXPORT.md
#define BETWEENFACTOR_VISIBILITY GTSAM_EXPORT

namespace fgo::factor {
    /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */

class GPInterpolatedDoublePose3BetweenFactor : public fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
    gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
    private:

        typedef GPInterpolatedDoublePose3BetweenFactor This;
        typedef fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>  Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
        GPBase GPbasePose1_;
        GPBase GPbasePose2_;
        gtsam::Pose3 measured_; /** The measurement */

    public:

        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<GPInterpolatedDoublePose3BetweenFactor> shared_ptr;

        /** default constructor - only use for serialization */
        GPInterpolatedDoublePose3BetweenFactor() = default;

        /** Constructor */
        GPInterpolatedDoublePose3BetweenFactor(gtsam::Key pose1i, gtsam::Key vel1i, gtsam::Key omega1i,
                                               gtsam::Key pose1j, gtsam::Key vel1j, gtsam::Key omega1j,
                                               gtsam::Key pose2i, gtsam::Key vel2i, gtsam::Key omega2i,
                                               gtsam::Key pose2j, gtsam::Key vel2j, gtsam::Key omega2j,
                                               const gtsam::Pose3 &measured,
                                               const std::shared_ptr<fgo::models::GPInterpolator> &interpolatorPose1,
                                               const std::shared_ptr<fgo::models::GPInterpolator> &interpolatorPose2,
                                               const gtsam::SharedNoiseModel &model = nullptr) :
            Base(model, pose1i, vel1i, omega1i, pose1j, vel1j, omega1j,
                 pose2i, vel2i, omega2i, pose2j, vel2j, omega2j), measured_(measured),
            GPbasePose1_(interpolatorPose1), GPbasePose2_(interpolatorPose2){
          factorTypeID_ = FactorTypeIDs::GPDoubleBetweenPose;
          factorName_ = "GPInterpolatedDoublePose3BetweenFactor";
        }

        ~GPInterpolatedDoublePose3BetweenFactor() override = default;

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
          std::cout << s << "GPInterpolatedDoublePose3BetweenFactor("
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

          gtsam::Matrix Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P,
                        Hint21_P, Hint22_P, Hint23_P, Hint24_P, Hint25_P, Hint26_P,
                        Hpose1, Hpose2;

          if(H1 || H2 || H3 || H4 || H5 || H6)
          {
            pose1 = GPbasePose1_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j,
                                                  Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P, Hint16_P);
          }
          else
            pose1 = GPbasePose1_->interpolatePose(p1i, v1i, omega1i, p1j, v1j, omega1j);

          if(H7 || H8 || H9 || H10 || H11 || H12)
          {
            pose2 = GPbasePose2_->interpolatePose(p2i, v2i, omega2i, p2j, v2j, omega2j,
                                                  Hint21_P, Hint22_P, Hint23_P, Hint24_P, Hint25_P, Hint26_P);
          }
          else
            pose2 = GPbasePose2_->interpolatePose(p2i, v2i, omega2i, p2j, v2j, omega2j);

          gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(pose1, pose2, &Hpose1, &Hpose2); // h(x)

          //std::cout << "pose1: " << pose1 << std::endl;
          //std::cout << "pose2: " << pose2 << std::endl;
         // std::cout << "hx: " << hx << std::endl;
          //std::cout << "measured_: " << measured_ << std::endl;

          // manifold equivalent of h(x)-z -> log(z,h(x))
          typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlocal;
          gtsam::Vector rval = gtsam::traits<gtsam::Pose3>::Local(measured_, hx, boost::none,  &Hlocal);

          if (H1) { *H1 = Hlocal * Hpose1 * Hint11_P;}// std::cout << "size H1: " << H1->size() << std::endl;}
          if (H2) { *H2 = Hlocal * Hint12_P;}// std::cout << "size H2: " << H2->size() << std::endl;}
          if (H3) { *H3 = Hlocal * Hint13_P;}// std::cout << "size H3: " << H3->size() << std::endl;}
          if (H4) { *H4 = Hlocal * Hpose1 * Hint14_P;}// std::cout << "size H4: " << H4->size() << std::endl;}
          if (H5) { *H5 = Hlocal * Hint15_P;}// std::cout << "size H5: " << H5->size() << std::endl;}
          if (H6) { *H6 = Hlocal * Hint16_P;}// std::cout << "size H6: " << H6->size() << std::endl;}
          if (H7) { *H7 = Hlocal * Hpose2 * Hint21_P;}// std::cout << "size H7: " << H7->size() << std::endl;}
          if (H8) { *H8 = Hlocal * Hint22_P;}// std::cout << "size H8: " << H8 ->size() << std::endl;}
          if (H9) { *H9 = Hlocal * Hint23_P;}// std::cout << "size H9: " << H9->size() << std::endl;}
          if (H10) { *H10 = Hlocal * Hpose2 * Hint24_P;}// std::cout << "size H10: " <<H10->size() << std::endl;}
          if (H11) { *H11 = Hlocal * Hint25_P;}// std::cout << "size H11: " << H11->size() << std::endl;}
          if (H12) { *H12 = Hlocal * Hint26_P;}// std::cout << "size H12: " << H12->size() << std::endl;}
          return rval;
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
          ar & boost::serialization::make_nvp("GPInterpolatedDoublePose3BetweenFactor",
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
    struct traits<fgo::factor::GPInterpolatedDoublePose3BetweenFactor> : public gtsam::Testable<fgo::factor::GPInterpolatedDoublePose3BetweenFactor> {};

/**
 * Binary between constraint - forces between to a given value
 * This constraint requires the underlying type to a Lie type
 *
 */
    /// traits
   // struct gtsam::traits<fgo::factor::GPIterBetweenConstraint> : public gtsam::Testable<fgo::factor::GPIterBetweenConstraint> {};
}




#endif //ONLINE_FGO_GPWNOAINTERDOUBLEBETWEENFACTORO_H
