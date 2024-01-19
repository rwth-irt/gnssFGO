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


#ifndef ONLINE_FGO_PVTFACTOR_WITHOMEGA_H
#define ONLINE_FGO_PVTFACTOR_WITHOMEGA_H

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

#include "third_party/matlab_utils.h"

namespace fgo::factor
{
    class PVTFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    protected:
        gtsam::Point3 posMeasured_;
        gtsam::Vector3 velMeasured_;
        MeasurementFrame velocityFrame_ = MeasurementFrame::BODY;
        gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame

        typedef PVTFactor This;
        typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
        bool useAutoDiff_ = false;

    public:
        PVTFactor() = default;

        PVTFactor(const gtsam::Key& poseKey, const gtsam::Key& velKey, const gtsam::Key& omegaKey,
                  const gtsam::Point3& positionMeasured,
                  const gtsam::Vector3& velocityMeasured,
                  const gtsam::Vector3 &lb,
                  MeasurementFrame velocityFrame,
                  const gtsam::SharedNoiseModel& model,
                  bool useAutoDiff = false) : Base(model, poseKey, velKey, omegaKey), posMeasured_(positionMeasured),
                  velMeasured_(velocityMeasured), velocityFrame_(velocityFrame), lb_(lb), useAutoDiff_(useAutoDiff)
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
                                                  const gtsam::Vector3& omega,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none) const override
        {
            /*
          if(useAutoDiff_)
          {
            if(H1)
              *H1 = gtsam::numericalDerivative31<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, vel, omega, 1e-5);
            if(H2)
              *H2 = gtsam::numericalDerivative32<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, vel, omega, 1e-5);
            if(H3)
              *H3 = gtsam::numericalDerivative33<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, vel, omega, 1e-5);
            return evaluateError_(pose, vel, omega);
          }
          else
          {*/
            gtsam::Matrix Hpose_P, Hpose_Pr, Hpose_r, Hdrho_p, Hdrho_t;

            const gtsam::Rot3& eRb = pose.rotation(&Hpose_Pr);
            const auto pos_ant = pose.translation(Hpose_P) + eRb.rotate(lb_, Hpose_r);
            const auto ePos = pos_ant - posMeasured_;
            const auto lb_skew = gtsam::skewSymmetric(-lb_);
            const auto lbv = lb_skew * omega;
            const auto vel_ant = vel + eRb.rotate(lbv, Hdrho_t, Hdrho_p);

            gtsam::Vector3 eVel;


            if(velocityFrame_ == MeasurementFrame::ECEF)
            {
              eVel = vel_ant - velMeasured_;
              if(H1) *H1 = (gtsam::Matrix66() <<Hpose_P +  Hpose_r * Hpose_Pr, Hdrho_t * Hpose_Pr + Hdrho_p).finished();
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), gtsam::Matrix33::Identity()).finished();  // Shape 6 x 3
              if(H3) *H3 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), Hdrho_p ).finished();  // Shape 6 x 3

            }
            else if(velocityFrame_ == MeasurementFrame::NED)
            {
              const auto nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pos_ant));
              eVel = nRe.rotate(vel_ant) - velMeasured_;
              const auto jac = matlab_utils::jacobianECEF2NED(pos_ant, vel_ant);

              if(H1) *H1 = (gtsam::Matrix66() <<Hpose_P +  Hpose_r * Hpose_Pr, jac.block<3,3>(0,0)* (Hdrho_t * Hpose_Pr + Hdrho_p)).finished();
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), jac.block<3,3>(0,0)).finished();  // Shape 6 x 3
              if(H3) *H3 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), jac.block<3,3>(0,0)*  Hdrho_p).finished();  // Shape 6 x 3
            }
            else
            {
              const auto nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pos_ant));
              eVel = nRe.rotate(vel_ant) - velMeasured_;
              const auto jac = matlab_utils::jacobianECEF2ENU(pos_ant, vel_ant);
              if(H1) *H1 = (gtsam::Matrix66() <<Hpose_P +  Hpose_r * Hpose_Pr, jac.block<3,3>(0,0) * (Hdrho_t * Hpose_Pr + Hdrho_p)).finished();
              if(H2) *H2 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), jac.block<3,3>(0,0)).finished();  // Shape 6 x 3
              if(H3) *H3 = (gtsam::Matrix63() << gtsam::Matrix33::Zero(), jac.block<3,3>(0,0) * Hdrho_p ).finished();  // Shape 6 x 3
            }
            return (gtsam::Vector6() << ePos, eVel).finished();
          //}
        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose,
                                                   const gtsam::Vector3 &vel,
                                                   const gtsam::Vector3& omega) const
        {

          const auto& eRb =  pose.rotation();
          const auto ePos = pose.translation() + eRb.rotate(lb_) - posMeasured_;
          const auto lbv = gtsam::skewSymmetric(-lb_) * omega;

          if(velocityFrame_ == MeasurementFrame::ECEF)
          {
            const auto eVel = vel - velMeasured_;
            return (gtsam::Vector6() << ePos, eVel).finished();
          }
          else if(velocityFrame_ == MeasurementFrame::NED)
          {
            const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
            //const auto eVel = nedRe.rotate(vel) - velMeasured_;
            const auto eVel = nedRe.rotate(vel + eRb.rotate(lbv)) - velMeasured_;
            const auto e = (gtsam::Vector6() << ePos, eVel).finished();
            return e;
          }
          else
          {
            const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
            const auto eVel = enuRe.rotate(vel + eRb.rotate(lbv)) - velMeasured_;
            //const auto eVel = enuRe.rotate(vel) - velMeasured_;
            return (gtsam::Vector6() << ePos, eVel).finished();
          }
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
#endif //ONLINE_FGO_PVTFACTOR_WITHOMEGA_H
