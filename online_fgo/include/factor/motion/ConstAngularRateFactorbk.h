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

#ifndef FGONAV_ARGULARRATEFACTOR_H
#define FGONAV_ARGULARRATEFACTOR_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include "factor/FactorTypeIDs.h"


namespace fgo::factor {
/**
 * 3-way angular rate factor, including pose and angular rate in reference ecef frame, imu bias.
 */
    class ConstAngularRateFactor : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::imuBias::ConstantBias> {

    private:
      gtsam::Vector3 angularRate_;
      typedef ConstAngularRateFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::imuBias::ConstantBias> Base;

    public:

      ConstAngularRateFactor() = default;  /* Default constructor */

      /**
       * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
       */
      ConstAngularRateFactor(gtsam::Key omega_i, gtsam::Key bias_i,
                             const gtsam::Vector3 &angularRate, const gtsam::SharedNoiseModel &model) :
              Base(model, omega_i, bias_i), angularRate_(angularRate) {}

      virtual ~ConstAngularRateFactor() {}

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Vector3 &omega, const gtsam::imuBias::ConstantBias &bias,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
#if AUTO_DIFF
        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector3,gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, bias), omega);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector3,gtsam::imuBias::ConstantBias>(
                  boost::bind(&This::evaluateError_, this, omega, boost::placeholders::_1), bias);
        return evaluateError_(omega, bias);
#else
        gtsam::Vector3 err;
        gtsam::Matrix36 Hbias;
        if (H2) {
          err = omega - bias.correctGyroscope(angularRate_, Hbias);
          *H2 = -Hbias;
        } else {
          err = omega - bias.correctGyroscope(angularRate_);
        }
        if (H1) *H1 = gtsam::I_3x3;
        return err;
#endif
      }

      [[nodiscard]] gtsam::Vector3 evaluateError_(const gtsam::Vector3 &omega, const gtsam::imuBias::ConstantBias &bias) const {
        return omega - bias.correctGyroscope(angularRate_);
      }

      /** return the measured */
      [[nodiscard]] const gtsam::Vector3 &measured() const {
        return angularRate_;
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != nullptr && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector3() << this->angularRate_).finished(),
                                            (gtsam::Vector3() << e->angularRate_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "ConstAngularRateFactor" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(angularRate_);
      }
    }; // PrDrFactor
  } // namespace fgonav

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::ConstAngularRateFactor> : public Testable<fgo::factor::ConstAngularRateFactor> {};
}

#endif //FGONAV_ARGULARRATEFACTOR_H
