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
#include "factor/GenericTypes.h"
#include "factor/FactorTypeIDs.h"

/*Inputs:
* Keys: pose of time i X(i), velocity of time i V(i), clock bias drift of time i C(i)
* Pseudorange measurement and doppler measurement: measRho and measdRho
* gyroscope measurement of time i: angularRate
* Position of the satellite and velocity of satellite: satXYZ, satVEL
* Position of sensor with respect to the Body: lb
* Covariance Matrix of the measurement/s: model*/
/* measurement equations used:
 * Pseudorange = Distance of Satellite and Receiver + Range through clock bias,
 * Doppler = Velocity between Satellite and Receiver (in direction) + Velocity through clock drift */
/*Jacobian: for X(i) = (e_RS * R_eb * skrew(lb_), e_RS * R_eb), V(i) = 0, e_RS, C(i) = 1,0,0,1
 * */

namespace fgo::factors {
    class PrFactorGMM : public MixtureModelFactor3<gtsam::Pose3, gtsam::Vector2, gtsam::Vector1> {

    protected:
      gtsam::Vector3 satXYZ_;
      gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame
      double measRho_;
      typedef PRFactorGMM This;
      typedef MixtureModelFactor3<gtsam::Pose3, gtsam::Vector2, gtsam::Vector1> Base;

    public:

        PrFactorGMM() = default;  /* Default constructor */

      /**
       * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
       */
      PrFactorGMM(gtsam::Key pose_i, gtsam::Key cbd_i, gtsam::Key weight_i, const double &measRho,
                  const gtsam::Vector3 &satXYZ, const gtsam::Vector3 &lb, const gtsam::SharedNoiseModel &model) :
                  Base(model, pose_i, cbd_i, weight_i), satXYZ_(satXYZ),
                  lb_(lb), measRho_(measRho)
                  {
                    factorTypeID_ = FactorTypeIDs::PRGMM;
                    factorName_ = "PrGMMFactor";
                  }

      ~PrFactorGMM() override = default;

      /// @return a deep copy of this factor
      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector2 &cbd, const gtsam::Vector1 &weight,
                                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                boost::optional<gtsam::Matrix &> H3 = boost::none) const override {

          if (H1) {
            *H1 = gtsam::numericalDerivative31<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                    boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);
          }
          if (H2)
            *H2 = gtsam::numericalDerivative32<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                    boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);
          if (H3)
              *H3 = gtsam::numericalDerivative33<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                      boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);

        return evaluateError_(pose, cbd, weight);

      }

      [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose, const gtsam::Vector2 &cbd, const gtsam::Vector1 &weight) const {

        const gtsam::Point3& P_eb_e = pose.translation();
        const gtsam::Rot3& eRb = pose.rotation();
        gtsam::Point3 P_eA_e = P_eb_e + eRb.rotate(lb_);
        gtsam::Matrix13 Hd;
        double real_range = gtsam::distance3(P_eA_e, satXYZ_, Hd);
        //double logisticWeight = 1.0 / 1.0 + std::exp(-1.0 * weight.value());

        return (gtsam::Vector1() << weight.cwiseAbs() * (real_range + cbd(0) - measRho_)).finished();

      }

      /** return the measured */
      gtsam::Vector1 measured() const {
        return gtsam::Vector1(measRho_);
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != nullptr && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->measRho_).finished(),
                                            (gtsam::Vector1() << e->measRho_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "PRFactorGMM" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("PRFactorGMM",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measRho_);
      }
    }; // PRFactorGmm


    class PrFactorMEst : public MixtureModelFactor3<gtsam::Pose3, gtsam::Vector2, gtsam::Vector1> {

    protected:
        gtsam::Vector3 satXYZ_;
        gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame
        double measRho_;
        typedef PRFactorMEst This;
        typedef MixtureModelFactor3<gtsam::Pose3, gtsam::Vector2, gtsam::Vector1> Base;

    public:

        PrFactorMEst() = default;  /* Default constructor */

        /**
         * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
         */
        PrFactorMEst(gtsam::Key pose_i, gtsam::Key cbd_i, gtsam::Key weight_i, const double &measRho,
                     const gtsam::Vector3 &satXYZ, const gtsam::Vector3 &lb, const gtsam::SharedNoiseModel &model) :
                     Base(model, pose_i, cbd_i, weight_i), satXYZ_(satXYZ),
                     lb_(lb), measRho_(measRho) {}

        ~PrFactorMEst() override = default;

        /// @return a deep copy of this factor
        [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector2 &cbd, const gtsam::Vector1 &weight,
                                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                                  boost::optional<gtsam::Matrix &> H3 = boost::none) const override {

            if (H1) {
                *H1 = gtsam::numericalDerivative31<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                        boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);
            }
            if (H2)
                *H2 = gtsam::numericalDerivative32<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                        boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);
            if (H3)
                *H3 = gtsam::numericalDerivative33<gtsam::Vector1, gtsam::Pose3, gtsam::Vector2, gtsam::Vector1>(
                        boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3), pose, cbd, weight, 1e-5);

            return evaluateError_(pose, cbd, weight);

        }

        [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose, const gtsam::Vector2 &cbd, const gtsam::Vector1 &weight) const {

            const gtsam::Point3& P_eb_e = pose.translation();
            const gtsam::Rot3& eRb = pose.rotation();
            gtsam::Point3 P_eA_e = P_eb_e + eRb.rotate(lb_);
            gtsam::Matrix13 Hd;
            double real_range = gtsam::distance3(P_eA_e, satXYZ_, Hd);
            // double logisticWeight = 1.0 / 1.0 + std::exp(-1.0 * weight.value());

            return (gtsam::Vector1() << (gtsam::Vector1::Ones() - weight.cwiseAbs()) * (real_range + cbd(0) - measRho_)).finished();

        }

        /** return the measured */
        [[nodiscard]] gtsam::Vector1 measured() const {
            return gtsam::Vector1(measRho_);
        }

        /** equals specialized to this factor */
        [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
            const This *e = dynamic_cast<const This *> (&expected);
            return e != NULL && Base::equals(*e, tol)
                   && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->measRho_).finished(),
                                                (gtsam::Vector1() << e->measRho_).finished(), tol);
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
            std::cout << s << "PRFactorMEst" << std::endl;
            Base::print("", keyFormatter);
        }

    private:

        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int version) {
            ar & boost::serialization::make_nvp("PRFactorMEst",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(measRho_);
        }
    }; // PRFactorMEst
  } // namespace fgo_online

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factors::PrFactorGMM> : public Testable<fgo::factors::PrFactorGMM> {};
  template<>
  struct traits<fgo::factors::PrFactorMEst> : public Testable<fgo::factors::PrFactorMEst> {};
}
