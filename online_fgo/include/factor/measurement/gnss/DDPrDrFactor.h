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

#ifndef ONLINE_FGO_DDPSEUDORANGEFACTOR_H
#define ONLINE_FGO_DDPSEUDORANGEFACTOR_H

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include <utils/NavigationTools.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include "factor/FactorTypeIDs.h"

/*Inputs:
* Keys: pose of time i X(i)
* DDPseudorange measurement or 4 Pseudorange measurements: ddPseuRa_ or pseuRaMR, pseuRaMB, pseuRaIR, pseuRaIB
* Position of the satellite Master (got chosen) and the other satellite I: pointMaster ,pointSatI
 * Position of the Base station we double differentiate to: pointBase
* Position of sensor with respect to the Body: lb
* Covariance Matrix of the measurement/s: model*/
/* measurement equations used:
 *DDCarrierPhase = Double Differenced Distance of Satellite and Receiver */
/*Jacobian: (e_RM - e_RI) * (R * skew(t), (e_RM - e_RI))
 * */
//sagnac effect correction https://paulba.no/temp/RelativisticCorrectionsInTheEu.pdf s141
//Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)

namespace fgo {
  namespace factor {
    class DDPrDrFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
    protected:
      double ddPseuRa_; //double difference measurement
      double ddDRange_;
      gtsam::Point3 pointMaster_; //position Master Satellite
      gtsam::Point3 velMaster_;
      gtsam::Point3 pointSatI_; //position Satellite I
      gtsam::Point3 velSatI_;
      gtsam::Point3 pointBase_; //position of Base
      gtsam::Point3 lb_;
      gtsam::Point3 lb2_ = gtsam::Point3(0, 0, 0); //position of antenna in body frame
      gtsam::Vector3 omega_;
      bool useAutoDiff_ = false;

      typedef DDPrDrFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> Base;

    public:
      DDPrDrFactor() = default;

      //when dd have to be calc by us
      DDPrDrFactor(gtsam::Key point_i, gtsam::Key vel_i, const double &pseuRaMR,
                   const double &pseuRaMB,
                   const double &pseuRaIR, const double &pseuRaIB, const double &dRangeMR,
                   const double &dRangeMB,
                   const double &dRangeIR, const double &dRangeIB, const gtsam::Point3 &pointMaster,
                   const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI,
                   const gtsam::Point3 &velSatI,
                   const gtsam::Point3 &pointBase, const gtsam::Point3 &lb, const gtsam::Point3 &omega,
                   const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, point_i, vel_i), pointMaster_(pointMaster), velMaster_(velMaster),
              pointSatI_(pointSatI),
              velSatI_(velSatI), pointBase_(pointBase), lb_(lb), omega_(omega), useAutoDiff_(useAutoDiff) {
        ddPseuRa_ = (pseuRaMR - pseuRaMB) - (pseuRaIR - pseuRaIB);
        ddDRange_ = (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB);
        factorTypeID_ = FactorTypeIDs::DDPRDR;
        factorName_ = "DDPrDrFactor";
      }

      //when dd is already calculated
      DDPrDrFactor(gtsam::Key point_i, gtsam::Key vel_i, const double &ddPseuRa,
                   const double &ddDRange, const gtsam::Point3 &pointMaster,
                   const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI, const gtsam::Point3 &velSatI,
                   const gtsam::Point3 &pointBase, const gtsam::Point3 &lb, const gtsam::Point3 &omega,
                   const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, point_i, vel_i), ddPseuRa_(ddPseuRa), ddDRange_(ddDRange), pointMaster_(pointMaster),
              velMaster_(velMaster),
              pointSatI_(pointSatI), velSatI_(velSatI), pointBase_(pointBase), lb_(lb), omega_(omega), useAutoDiff_(useAutoDiff) {
        factorTypeID_ = FactorTypeIDs::DDPRDR;
        factorName_ = "DDPrDrFactor";
      }

      //for dual antenna setup, where we dont have basestattion
      DDPrDrFactor(gtsam::Key point_i, gtsam::Key vel_i, const double &ddPseuRa,
                   const double &ddDRange, const gtsam::Point3 &pointMaster,
                   const gtsam::Point3 &velMaster, const gtsam::Point3 &pointSatI, const gtsam::Point3 &velSatI,
                   const gtsam::Point3 &lb, const gtsam::Point3 &omega,
                   const gtsam::SharedNoiseModel &model, const gtsam::Point3 &lb2, bool useAutoDiff = false) :
              Base(model, point_i, vel_i), ddPseuRa_(ddPseuRa), ddDRange_(ddDRange), pointMaster_(pointMaster),
              velMaster_(velMaster),
              pointSatI_(pointSatI), velSatI_(velSatI), lb_(lb), omega_(omega), lb2_(lb2), useAutoDiff_(useAutoDiff) {
        factorTypeID_ = FactorTypeIDs::DDPRDR;
        factorName_ = "DDPrDrFactor";
      }

      ~DDPrDrFactor() override = default;

      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &poseBody, const gtsam::Vector3 &velBody,
                                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, velBody), poseBody, 1e-5);
          if (H2)
            *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector3>(
                boost::bind(&This::evaluateError_, this, poseBody, boost::placeholders::_1), velBody, 1e-5);
          return evaluateError_(poseBody, velBody);
        }
        else
        {
          //param
          gtsam::Point3 lbv, lbv2;
          gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI; //direction unity vectors
          double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
          double rangeMR, rangeIR, rangeMB, rangeIB; //ranges
          //calculate position
          gtsam::Vector3 positionReceiver = poseBody.translation() + poseBody.rotation() * lb_; //pose_Antenna wrt to Earth
          //calc vel
          lbv = gtsam::skewSymmetric((-lb_)) * omega_;
          gtsam::Point3 velRec = velBody + poseBody.rotation().rotate(lbv);
          //calculate ranges
          if (lb2_.norm() == 0) {
            rangeMR = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
            rangeIR = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
            rangeMB = gtsam::distance3(pointBase_, pointMaster_, e_BM);
            rangeIB = gtsam::distance3(pointBase_, pointSatI_, e_BI);

            dRangeMR = e_RM * (velRec - velMaster_);
            dRangeIR = e_RI * (velRec - velSatI_);
            dRangeMB = e_BM * (-velMaster_);
            dRangeIB = e_BI * (-velSatI_);
            e_BM = gtsam::Vector3(0,0,0), e_BI = gtsam::Vector3(0,0,0);
          } else {
            gtsam::Pose3 body_P_sensor2 = gtsam::Pose3(gtsam::Rot3(), lb2_);
            gtsam::Point3 positionReceiver2 = poseBody.compose(body_P_sensor2).translation();
            rangeMR = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
            rangeIR = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
            rangeMB = gtsam::distance3(positionReceiver2, pointMaster_, e_BM);
            rangeIB = gtsam::distance3(positionReceiver2, pointSatI_, e_BI);

            lbv2 = gtsam::skewSymmetric((-lb2_)) * omega_;
            gtsam::Point3 velRec2 = velBody + poseBody.rotation().rotate(lbv2);
            dRangeMR = e_RM * (velRec - velMaster_);
            dRangeIR = e_RI * (velRec - velSatI_);
            dRangeMB = e_BM * (velRec2 - velMaster_);
            dRangeIB = e_BI * (velRec2 - velSatI_);
          }

          if (H1 || H2) {
            //calculate Jacobian for Rotation
            gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
            gtsam::Matrix dPrdRot = (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                                    (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lb2_);
            gtsam::Matrix dDrdRot = (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lbv) -
                                    (e_BM - e_BI) * rotation * gtsam::skewSymmetric(-lbv2);

            if(H1)
              *H1 = (gtsam::Matrix26() << dPrdRot, ((e_RM - e_RI) - (e_BM - e_BI)) * rotation /*THIS ROT ONLY CORRECTION*/,
                  dDrdRot, 0, 0, 0).finished(); // derivation pose
            if (H2)
              *H2 = (gtsam::Matrix23() << 0, 0, 0, (e_RM - e_RI) - (e_BM - e_BI)).finished();

          }
          return (gtsam::Vector2() << (rangeMR - rangeMB) - (rangeIR - rangeIB) - ddPseuRa_,
              (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB) - ddDRange_).finished();
        }
      }

      [[nodiscard]] gtsam::Vector2 evaluateError_(const gtsam::Pose3 &poseBody, const gtsam::Vector3 &velBody) const {
        //calc poseReceiver
        gtsam::Point3 positionReceiver = poseBody.translation() + poseBody.rotation().rotate(lb_);
        //calc velReceiver
        gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * omega_;
        gtsam::Point3 velRec = velBody + poseBody.rotation().rotate(lbv);

        gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI; //direction unity vectors
        double dRangeMR, dRangeIR, dRangeMB, dRangeIB;
        double rangeMR, rangeIR, rangeMB, rangeIB; //ranges
        gtsam::Vector2 error_;

        //calc ranges
        if (lb2_.norm() == 0) {
          rangeMR = gtsam::distance3(positionReceiver, pointMaster_);
          rangeIR = gtsam::distance3(positionReceiver, pointSatI_);
          rangeMB = gtsam::distance3(pointBase_, pointMaster_);
          rangeIB = gtsam::distance3(pointBase_, pointSatI_);

          //dRangeMR = e_RM * (velRec - velMaster_);
          //dRangeIR = e_RI * (velRec - velSatI_);
          //dRangeMB = e_BM * (-velMaster_);
          //dRangeIB = e_BI * (-velSatI_);
          error_ = (gtsam::Vector2() << (rangeMR - rangeMB) - (rangeIR - rangeIB) - ddPseuRa_,0).finished();
        } else {
          gtsam::Point3 positionReceiver2 = poseBody.translation() + poseBody.rotation() * lb2_;
          rangeMR = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeIR = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeMB = gtsam::distance3(positionReceiver2, pointMaster_, e_BM);
          rangeIB = gtsam::distance3(positionReceiver2, pointSatI_, e_BI);

          gtsam::Point3 lbv2 = gtsam::skewSymmetric((-lb2_)) * omega_;
          gtsam::Point3 velRec2 = velBody + poseBody.rotation().rotate(lbv2);
          dRangeMR = e_RM * (velRec - velMaster_);
          dRangeIR = e_RI * (velRec - velSatI_);
          dRangeMB = e_BM * (velRec2 - velMaster_);
          dRangeIB = e_BI * (velRec2 - velSatI_);
          error_ = (gtsam::Vector2() << (rangeMR - rangeMB) - (rangeIR - rangeIB) - ddPseuRa_,
                  (dRangeMR - dRangeMB) - (dRangeIR - dRangeIB) - ddDRange_).finished();
        }

        return error_;
      }

      /** return the measured */

      [[nodiscard]] double measured() const {
        return ddPseuRa_;
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->ddPseuRa_, this->ddDRange_).finished(),
                                            (gtsam::Vector2() << e->ddPseuRa_, e->ddDRange_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "DDPseudorangeDopplerrange:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("DDPseudorangeDopplerrange",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(ddPseuRa_);
      }

    };
  } //fgo
}

#endif //ONLINE_FGO_DDPSEUDORANGEFACTOR_H

namespace gtsam {
  template<>
    struct traits<fgo::factor::DDPrDrFactor> :
            public Testable<fgo::factor::DDPrDrFactor> {
    };
}