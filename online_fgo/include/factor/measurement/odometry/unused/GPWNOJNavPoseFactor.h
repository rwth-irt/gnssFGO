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
// Created by haoming on 12.03.23.
//

#ifndef ONLINE_FGO_GPWNOJNAVPOSEFACTOR_H
#define ONLINE_FGO_GPWNOJNAVPOSEFACTOR_H

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "data/FactorTypes.h"
#include "utils/NavigationTools.h"

namespace fgo::factor
{
    class GPWNOJNavPoseFactor : public fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                                              gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6>
    {
    protected:
        gtsam::Pose3 poseMeasured_;

        bool useAutoDiff_ = false;
        double tau_{};
        typedef GPWNOJNavPoseFactor This;
        typedef  fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> Base;
        typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
        GPBase GPBase_;

    public:
        GPWNOJNavPoseFactor() = default;

        GPWNOJNavPoseFactor(gtsam::Key poseKeyI, gtsam::Key velKeyI, gtsam::Key omegaKeyI, gtsam::Key accKeyI,
                            gtsam::Key poseKeyJ, gtsam::Key velKeyJ, gtsam::Key omegaKeyJ, gtsam::Key accKeyJ,
                            const gtsam::Pose3 &poseMeasured,
                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                            const gtsam::SharedNoiseModel &model,
                            bool useAutoDiff = true) :
            Base(model, poseKeyI, velKeyI, omegaKeyI, accKeyI, poseKeyJ, velKeyJ, omegaKeyJ, accKeyJ), poseMeasured_(poseMeasured), useAutoDiff_(useAutoDiff),
            tau_(interpolator->getTau()), GPBase_(interpolator)
        {}

        ~GPWNOJNavPoseFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** factor error */
        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 & omegaI, const gtsam::Vector6 & accI,
                                                  const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 & omegaJ, const gtsam::Vector6 & accJ,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H7 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H8 = boost::none) const override
        {
          if(useAutoDiff_)
          {
            if(H1)
              *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ), poseI, 1e-5);

            if(H2)
              *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, poseI, boost::placeholders::_1, omegaI, accI, poseJ, velJ, omegaJ, accJ), velI, 1e-5);

            if(H3)
              *H3 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, boost::placeholders::_1, accI, poseJ, velJ, omegaJ, accJ), omegaI, 1e-5);

            if(H4)
              *H4 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, boost::placeholders::_1, poseJ, velJ, omegaJ, accJ), accI, 1e-5);

            if(H5)
              *H5 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, boost::placeholders::_1, velJ, omegaJ, accJ), poseJ, 1e-5);

            if(H6)
              *H6 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, boost::placeholders::_1, omegaJ, accJ), velJ, 1e-5);

            if(H7)
              *H7 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, boost::placeholders::_1, accJ), omegaJ, 1e-5);

            if(H8)
              *H8 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
                  boost::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, boost::placeholders::_1), accJ, 1e-5);

            return evaluateError_(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
          }
          else
          {

          }
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 & omegaI, const gtsam::Vector6 &accI,
                                                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 & omegaJ, const gtsam::Vector6 &accJ) const
        {
          const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
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
          std::cout << s << "GPWNOJNavPoseFactor" << std::endl;
          Base::print("", keyFormatter);
        }

    private:
        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int version) {
          ar & boost::serialization::make_nvp("GPWNOJNavPoseFactor",
                                              boost::serialization::base_object<Base>(*this));
          ar & BOOST_SERIALIZATION_NVP(poseMeasured_);
        }


    };
}
/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::GPWNOJNavPoseFactor> :
        public Testable<fgo::factor::GPWNOJNavPoseFactor> {
    };
}
#endif //ONLINE_FGO_GPWNOJNAVPOSEFACTOR_H
