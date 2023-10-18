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
#ifndef FGONAV_MOTIONMODELFACTOR_H
#define FGONAV_MOTIONMODELFACTOR_H


#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/base/numericalDerivative.h>
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

    class MotionModelFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3> {

    private:
      typedef MotionModelFactor This;
      typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
              gtsam::Pose3, gtsam::Vector3> Base;
      double deltatime_{};

    public:
      MotionModelFactor() = default;

      MotionModelFactor(gtsam::Key pose_i, gtsam::Key vel_i,
                        gtsam::Key pose_j, gtsam::Key vel_j,
                        const double deltatime, const gtsam::SharedNoiseModel &model) :
              Base(model, pose_i, vel_i, pose_j, vel_j), deltatime_(deltatime) {}

      ~MotionModelFactor() override {}

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      // Calculate error and Jacobians
      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &x1, const gtsam::Vector3 &v1,
                                  const gtsam::Pose3 &x2, const gtsam::Vector3 &v2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                  boost::optional<gtsam::Matrix &> H4 = boost::none) const override {
#if AUTO_DIFF
        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector6,gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, v1, x2, v2), x1);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector6,gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, x1, boost::placeholders::_1, x2, v2), v1);
        if (H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Vector6,gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, x1, v1, boost::placeholders::_1, v2), x2);
        if (H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Vector6,gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, x1, v1, x2, boost::placeholders::_1), v2);
        return evaluateError_(x1, v1, x2, v2);
#else
        if (H1) { (*H1) = (gtsam::Matrix66() << gtsam::Z_3x3, -gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3).finished(); }
        if (H2) { (*H2) = (gtsam::Matrix63() << - gtsam::I_3x3 * deltatime_, -gtsam::I_3x3).finished(); }
        if (H3) { (*H3) = (gtsam::Matrix66() << gtsam::Z_3x3, gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3).finished(); }
        if (H4) { (*H4) = (gtsam::Matrix63() << gtsam::Z_3x3, gtsam::I_3x3).finished(); }
        return (gtsam::Vector6() << (x2.translation() - x1.translation() - v1 * deltatime_, v2 - v1)).finished();
#endif
      }

      [[nodiscard]] gtsam::Vector6 evaluateError_(const gtsam::Pose3 &x1, const gtsam::Vector3 &v1, const gtsam::Pose3 &x2, const gtsam::Vector3 &v2) const {
        return (gtsam::Vector6() << x2.translation() - x1.translation() - v1 * deltatime_,
                x2.rotation().unrotate(v2) - x1.rotation().unrotate(v1) ).finished();
      }

      [[nodiscard]] const double &measured() const {
        return deltatime_;
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "binary MotionModelFactor" << std::endl;
        Base::print("", keyFormatter);
      }

    };//MotionModelFactor
  }//namespace fgonav

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::MotionModelFactor> : public Testable<fgo::factor::MotionModelFactor> {};
}

#endif //FGONAV_MOTIONMODELFACTOR_H