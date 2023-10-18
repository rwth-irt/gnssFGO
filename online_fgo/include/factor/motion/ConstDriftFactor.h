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

#ifndef FGONAV_CONSTDRIFTFACTOR_H
#define FGONAV_CONSTDRIFTFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/numericalDerivative.h>
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

    class ConstDriftFactor : public gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector2> {

    private:
      typedef ConstDriftFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector2> Base;
      double deltatime_{};

    public:
      typedef boost::shared_ptr<ConstDriftFactor> shared_ptr;

      ConstDriftFactor() = default;

      ConstDriftFactor(gtsam::Key i, gtsam::Key j, const double deltatime, const gtsam::SharedNoiseModel &model) :
              Base(model, i, j), deltatime_(deltatime) {
        factorTypeID_ = FactorTypeIDs::ReceiverClock;
        factorName_ = "ConstDriftFactor";
      }

      ~ConstDriftFactor() override {}

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      // Calculate error and Jacobians
      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Vector2 &cbd1, const gtsam::Vector2 &cbd2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
#if AUTO_DIFF
        const auto trick = (gtsam::Vector2() << 1e-5, 1e-5).finished();
        if (H1) {
          *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(
              boost::bind(&This::evaluateError_, this, boost::placeholders::_1, cbd2), cbd1);
          //*H1 += trick;
        }
        if (H2) {
          *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(
              boost::bind(&This::evaluateError_, this, cbd1, boost::placeholders::_1), cbd2);
          //*H2 += trick;
        }
        return evaluateError_(cbd1, cbd2);
#else
        if (H1) {
          (*H1) = (gtsam::Matrix22() << -1.0, -deltatime_,
                  0.0, -1.0).finished();
        }
        if (H2) {
          (*H2) = (gtsam::Matrix22() << 1.0, 0.0,
                  0.0, 1.0).finished();
        }
        return (gtsam::Vector2() << cbd2(0) - cbd1(0) - deltatime_ * cbd1(1),
                cbd2(1) - cbd1(1)).finished();
#endif
      }

      [[nodiscard]] gtsam::Vector2 evaluateError_(const gtsam::Vector2 &cbd1, const gtsam::Vector2 &cbd2) const {
        return (gtsam::Vector2() << cbd2(0) - cbd1(0) - deltatime_ * cbd1(1),
                cbd2(1) - cbd1(1)).finished();
      }

      [[nodiscard]] const double &measured() const {
        return deltatime_;
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "binary ConstDriftFactor" << std::endl;
        Base::print("", keyFormatter);
      }
    };//ConstDriftFactor
  }//namespace fgonav

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::ConstDriftFactor> : public Testable<fgo::factor::ConstDriftFactor> {};
}


#endif //FGONAV_CONSTDRIFTFACTOR_H