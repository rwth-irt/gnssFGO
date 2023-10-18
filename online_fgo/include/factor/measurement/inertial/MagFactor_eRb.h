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

#ifndef ONLINE_FGO_MAGFACTOR_ERB_H
#define ONLINE_FGO_MAGFACTOR_ERB_H

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MagFactor.h
 * @brief   Factor involving magnetometers to estimate Rotation
 * @author  Frank Dellaert
 * @date   January 29, 2014
 * @changed now doesnt estimate nRb but eRb
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>

#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

namespace fgo {
  namespace factor {
/**
 * Factor to estimate rotation given magnetometer reading
 * This version uses model measured bM = scale * bRn * direction + bias
 * and assumes scale, direction, and the bias are given
 */
    class MagFactor_eRb : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

    private:
      const gtsam::Vector3 measuredb_; ///< The measured magnetometer values
      const gtsam::Vector3 magField_ecef; ///< Local magnetic field (mag output units)
      const gtsam::Vector3 bias_; ///< bias

      typedef MagFactor_eRb This;

    public:
      MagFactor_eRb() = default;

      /** Constructor */
      MagFactor_eRb(gtsam::Key pose_i, const gtsam::Vector3 &measured_b, const gtsam::Vector3 &expected_ecef,
                    const gtsam::Vector3 &bias, const gtsam::SharedNoiseModel &model) :
              NoiseModelFactor1<gtsam::Pose3>(model, pose_i), //
              measuredb_(measured_b / measured_b.norm()), magField_ecef(expected_ecef / expected_ecef.norm()),
              bias_(bias) {
      }

      /// @return a deep copy of this factor
      NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<NonlinearFactor>(
                NonlinearFactor::shared_ptr(new MagFactor_eRb(*this)));
      }

      /**
       * @brief vector of errors
       */
      gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                  boost::optional<gtsam::Matrix &> H = boost::none) const override {
        // measured bM = nRb� * nM + b
#if AUTO_DIFF
        if (H)
          *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1), pose);
        return evaluateError_(pose);
#else
        const gtsam::Rot3 Rb2e = pose.rotation();
        gtsam::Matrix33 JacRot;
        gtsam::Point3 hx = Rb2e.unrotate(magField_xyz, JacRot) + bias_;
        if (H)
          *H = (gtsam::Matrix36() << JacRot, gtsam::Z_3x3).finished();
        gtsam::Vector3 error = hx - measuredb_;
        return error;
#endif
      }

      gtsam::Vector3 evaluateError_(const gtsam::Pose3 &pose) const {
        // measured bM = nRb� * nM + b
        gtsam::Rot3 eRb = pose.rotation();

        gtsam::Point3 hx = eRb.transpose() * magField_ecef;
        gtsam::Vector3 error = hx - bias_ - measuredb_;
        return error;
      }

      /** return the measured */
      const gtsam::Vector3 measured() const {
        return measuredb_;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol(this->measuredb_, e->measuredb_, tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "MagFactor" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("MagFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measuredb_);
      }
    };

  }
}

#endif //ONLINE_FGO_MAGFACTOR_ERB_H
