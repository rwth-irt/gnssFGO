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


#ifndef ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H
#define ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H

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

#include "utils/NavigationTools.h"


namespace fgo::factor {
/**
 * 3-way angular rate factor, including pose and angular rate in reference ecef frame, imu bias.
 */
class ConstLinearAccelerationFactor : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Pose3, gtsam::imuBias::ConstantBias> {

    private:
        gtsam::Vector3 accLinMeasured_;
        gtsam::Vector3 accRotMeasured_;
        typedef ConstLinearAccelerationFactor This;
        typedef gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Pose3, gtsam::imuBias::ConstantBias> Base;

    public:

        ConstLinearAccelerationFactor() = default;  /* Default constructor */

        /**
         * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
         */
        ConstLinearAccelerationFactor(gtsam::Key acc_j, gtsam::Key pose_j, gtsam::Key bias_j,
                                      const gtsam::Vector3 &linAcc, const gtsam::Vector3 &rotAcc,
                                      const gtsam::SharedNoiseModel &model)
                                      : Base(model, acc_j, pose_j, bias_j),
                                      accLinMeasured_(linAcc),
                                      accRotMeasured_(rotAcc){}

        ~ConstLinearAccelerationFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Vector6 &acc,
                                                  const gtsam::Pose3 &pose,
                                                  const gtsam::imuBias::ConstantBias &bias,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none) const override {
#if AUTO_DIFF
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, pose, bias), acc);
          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, acc, boost::placeholders::_1, bias), pose);
          if (H3)
            *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::imuBias::ConstantBias>(
                boost::bind(&This::evaluateError_, this, acc, pose, boost::placeholders::_1), bias);
          return evaluateError_(acc, pose, bias);
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

        [[nodiscard]] gtsam::Vector6 evaluateError_(const gtsam::Vector6 &acc,
                                                    const gtsam::Pose3 &pose,
                                                    const gtsam::imuBias::ConstantBias &bias) const {
          auto gravity = fgo::utils::gravity_ecef(pose.translation());
          auto gravity_b = pose.rotation().unrotate(gravity);
          return (gtsam::Vector6() <<acc.head(3) - accRotMeasured_, acc.tail(3) - bias.correctAccelerometer(accLinMeasured_ + gravity_b)).finished();
        }

        /** return the measured */
        [[nodiscard]] const gtsam::Vector6 &measured() const {
          return (gtsam::Vector6() << accRotMeasured_, accLinMeasured_).finished();
        }

        /** equals specialized to this factor */
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol)
                 && gtsam::equal_with_abs_tol((gtsam::Vector6() << this->accRotMeasured_,  accLinMeasured_).finished(),
                                              (gtsam::Vector6() << e->accRotMeasured_ , accLinMeasured_).finished(), tol);
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
          ar & boost::serialization::make_nvp("NoiseModelFactor3",
                                              boost::serialization::base_object<Base>(*this));
          ar & BOOST_SERIALIZATION_NVP(accLinMeasured_);
        }
    }; // PrDrFactor
} // namespace fgonav

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::ConstLinearAccelerationFactor> : public Testable<fgo::factor::ConstLinearAccelerationFactor> {};
}



#endif //ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H
