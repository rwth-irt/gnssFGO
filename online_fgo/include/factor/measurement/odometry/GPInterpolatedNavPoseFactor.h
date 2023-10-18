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


#ifndef ONLINE_FGO_GPINTERPOLATEDNAVPOSEFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDNAVPOSEFACTOR_H

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor
{
class GPInterpolatedNavPoseFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
{
protected:
    gtsam::Pose3 poseMeasured_;

    bool useAutoDiff_ = false;
    double tau_{};
    typedef GPInterpolatedNavPoseFactor This;
    typedef  gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
    GPBase GPBase_;

public:
    GPInterpolatedNavPoseFactor() = default;

    GPInterpolatedNavPoseFactor(gtsam::Key poseKeyI, gtsam::Key velKeyI, gtsam::Key omegaKeyI,
                                gtsam::Key poseKeyJ, gtsam::Key velKeyJ, gtsam::Key omegaKeyJ,
                                const gtsam::Pose3 &poseMeasured,
                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                const gtsam::SharedNoiseModel &model,
                                bool useAutoDiff = true) :
        Base(model, poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ, omegaKeyJ), poseMeasured_(poseMeasured), useAutoDiff_(useAutoDiff),
        tau_(interpolator->getTau()), GPBase_(interpolator)
    {
      factorTypeID_ = FactorTypeIDs::GPNavPose;
      factorName_ = "GPInterpolatedNavPoseFactor";
    }

    ~GPInterpolatedNavPoseFactor() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** factor error */
    [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 & omegaI,
                                              const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 & omegaJ,
                                              boost::optional<gtsam::Matrix &> H1 = boost::none,
                                              boost::optional<gtsam::Matrix &> H2 = boost::none,
                                              boost::optional<gtsam::Matrix &> H3 = boost::none,
                                              boost::optional<gtsam::Matrix &> H4 = boost::none,
                                              boost::optional<gtsam::Matrix &> H5 = boost::none,
                                              boost::optional<gtsam::Matrix &> H6 = boost::none) const override
    {
      if(useAutoDiff_)
      {
        if(H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
              boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, poseJ, velJ, omegaJ), poseI, 1e-5);

        if(H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
              boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, poseJ, velJ, omegaJ), velI, 1e-5);

        if(H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
              boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, poseJ, velJ, omegaJ), omegaI, 1e-5);

        if(H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
              boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, velJ, omegaJ), poseJ, 1e-5);

        if(H5)
          *H5 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
              boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, boost::placeholders::_1, omegaJ), velJ, 1e-5);

        if(H6)
          *H6 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
              boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, poseJ, velJ, boost::placeholders::_1), omegaJ, 1e-5);

        return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      }
      else
      {
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hpose;
        gtsam::Pose3 pose;
        gtsam::Vector6 err;

        if (H1 || H2 || H3 || H4 || H5 || H6) {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                          Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
          err = pose.localCoordinates(poseMeasured_, Hpose);

          if(H1) *H1 = Hpose * Hint1_P;
          if(H2) *H2 = Hpose * Hint2_P;
          if(H3) *H3 = Hpose * Hint3_P;
          if(H4) *H4 = Hpose * Hint4_P;
          if(H5) *H5 = Hpose * Hint5_P;
          if(H6) *H6 = Hpose * Hint6_P;
        }
        else {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
          err = pose.localCoordinates(poseMeasured_);
        }
        return err;
      }
    }

    [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 & omegaI,
                                              const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 & omegaJ) const
    {
      const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      return pose.localCoordinates(poseMeasured_);
    }

    /** return the measured */
    [[nodiscard]] gtsam::Pose3 measured() const {
      return poseMeasured_;
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol(this->poseMeasured_.translation(),
                                          e->poseMeasured_.translation(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterpolatedNavPoseFactor" << std::endl;
      Base::print("", keyFormatter);
    }

private:
    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedNavPoseFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(poseMeasured_);
    }


};
}
/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::GPInterpolatedNavPoseFactor> :
        public Testable<fgo::factor::GPInterpolatedNavPoseFactor> {
    };
}
#endif //ONLINE_FGO_GPINTERPOLATEDNAVPOSEFACTOR_H
