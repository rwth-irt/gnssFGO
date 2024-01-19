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


#ifndef ONLINE_FGO_PVTFACTOR_H
#define ONLINE_FGO_PVTFACTOR_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include "utils/NavigationTools.h"
#include "data/FactorTypes.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor
{
    class PVTFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
    {
    protected:
        gtsam::Point3 posMeasured_;
        gtsam::Vector3 velMeasured_;
        MeasurementFrame velocityFrame_ = MeasurementFrame::BODY;
        gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame
        gtsam::Vector3 omega_;

        typedef PVTFactor This;
        typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> Base;
        bool useAutoDiff_ = false;

    public:
        PVTFactor() = default;

        PVTFactor(const gtsam::Key& poseKey, const gtsam::Key& velKey,
                  const gtsam::Point3& positionMeasured,
                  const gtsam::Vector3& velocityMeasured,
                  const gtsam::Vector3 &lb,
                  const gtsam::Vector3 &omega,
                  MeasurementFrame velocityFrame,
                  const gtsam::SharedNoiseModel& model,
                  bool useAutoDiff = false) : Base(model, poseKey, velKey), posMeasured_(positionMeasured),
                  velMeasured_(velocityMeasured), velocityFrame_(velocityFrame), lb_(lb), omega_(omega), useAutoDiff_(useAutoDiff)
                  {
                    factorTypeID_ = FactorTypeIDs::PVT;
                    factorName_ = "PVTFactor";
                  }

        ~PVTFactor() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
          return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override
        {
            /*
          if(useAutoDiff_)
          {
            if(H1)
              *H1 = gtsam::numericalDerivative21<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2), pose, vel, 1e-5);
            if(H2)
              *H2 = gtsam::numericalDerivative22<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2), pose, vel, 1e-5);

            return evaluateError_(pose, vel);
          }
          else
          {*/
            gtsam::Matrix Hpose_P, Hpose_Pr, Hpose_r;

            const gtsam::Rot3& eRb = pose.rotation(&Hpose_Pr);
            const auto& pos = pose.translation(Hpose_P);
            const auto ePos = pos + eRb.rotate(lb_, Hpose_r) - posMeasured_;
            const auto lbv = gtsam::skewSymmetric(-lb_) * omega_;
            const auto vel_ant = vel + eRb.rotate(lbv);
            gtsam::Vector3 eVel;

            if(H1)
              *H1 = (gtsam::Matrix66() <<Hpose_P +  Hpose_r * Hpose_Pr, gtsam::Matrix36::Zero()).finished();

            if(velocityFrame_ == MeasurementFrame::ECEF)
            {
              eVel = vel_ant - velMeasured_;
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Z_3x3, gtsam::I_3x3).finished();  // Shape 6 x 3
            }
            else if(velocityFrame_ == MeasurementFrame::NED)
            {
              gtsam::Matrix Hpose, Hvel;
              const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos));
              eVel = nedRe.rotate(vel_ant, Hpose, Hvel) - velMeasured_;  // This should be inaccurate
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Z_3x3, Hvel).finished();  // Shape 6 x 3
            }
            else
            {
              gtsam::Matrix Hpose, Hvel;
              const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos));
              eVel = enuRe.rotate(vel_ant, Hpose, Hvel) - velMeasured_;
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Z_3x3, Hvel).finished();  // Shape 6 x 3
            }

              return (gtsam::Vector6() << ePos, eVel).finished();
          //}
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose,
                                                   const gtsam::Vector3 &vel) const
        {
            // NOT USED DUE TO UNSTABLE ERROR EVALUATION!
          const auto& eRb =  pose.rotation();
          const auto ePos = pose.translation() + eRb.rotate(lb_) - posMeasured_;
          const auto vel_ant = vel + eRb.rotate(gtsam::skewSymmetric(lb_) * omega_);
          gtsam::Vector3 eVel;
          if(velocityFrame_ == MeasurementFrame::ECEF)
          {
              eVel = vel_ant - velMeasured_;
          }
          else if(velocityFrame_ == MeasurementFrame::NED)
          {
            const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
            //const auto eVel = nedRe.rotate(vel) - velMeasured_;
            eVel = nedRe.rotate(vel_ant) - velMeasured_;
          }
          else
          {
            const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
            eVel = enuRe.rotate(vel_ant) - velMeasured_;
          }
            return (gtsam::Vector6() << ePos, eVel).finished();
        }

        /** return the measured */
        [[nodiscard]] std::pair<gtsam::Point3, gtsam::Vector3> measured() const {
          return {posMeasured_, velMeasured_};
        }

        /** equals specialized to this factor */
        bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
          const This *e = dynamic_cast<const This *> (&expected);
          return e != nullptr && Base::equals(*e, tol)
                 && gtsam::equal_with_abs_tol((gtsam::Point3 () << this->posMeasured_).finished(),
                                              (gtsam::Point3 () << e->posMeasured_).finished(), tol);
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
          std::cout << s << "PVTFactor" << std::endl;
          Base::print("", keyFormatter);
        }

    private:

        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int version) {
          ar & boost::serialization::make_nvp("PVTFactor",
                                              boost::serialization::base_object<Base>(*this));
          ar & BOOST_SERIALIZATION_NVP(posMeasured_);
          ar & BOOST_SERIALIZATION_NVP(velMeasured_);
        }

    };
}

/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factor::PVTFactor> : public Testable<fgo::factor::PVTFactor> {
    };
}
#endif //ONLINE_FGO_PVTFACTOR_H
