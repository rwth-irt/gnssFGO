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

#ifndef ONLINE_FGO_DDCPFACTOR_H
#define ONLINE_FGO_DDCPFACTOR_H

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include "factor/FactorTypeIDs.h"

/*Inputs:
 * Keys: pose of time i X(i), integer double differenced ambiguity of time i M(i)
 * Double Difference CarrierPhase measurement: ddphi (already calculated) or carPhaMR,carPhaMB,carPhaIR,carPhaIB
 * Position of the satellite Master (got chosen) and the other satellite I: pointMaster ,pointSatI
 * Position of the Base station we double differentiate to: pointBase
 * Integer to find the right ambiguity in the vector: nAmbiguity (maybe it can be done in a more elegant way)
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * Covariance Matrix of the measurement/s: model*/
/* measurement equation used:
 * Wavelength * DDCarrierPhase = Double Differenced Distance of Satellite and Receiver -(+) lambda * DDAmbiguityCycles + noise
 * somtimes the AmbiguityCycles get added, but its just a definition in DD measurements it doesnt matter because it can be neg anyway*/
/*Jacobian: (e_RM - e_RI) * (R * skew(t), (e_RM - e_RI))
 * */
//sagnac effect correction https://paulba.no/temp/RelativisticCorrectionsInTheEu.pdf p141 (not used)
//Information to Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)


namespace fgo::factor {
    class DDCarrierPhaseFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector> {
    private:
      double ddphi_{}; //double difference measurement
      double lambda_{}; //wavelength
      int nAmbiguity_{}; //where in the vector is the satellite
      gtsam::Vector3 pointMaster_; //position Master Satellite
      gtsam::Vector3 pointSatI_; //position Satellite I
      gtsam::Vector3 pointBase_; //position of Base
      gtsam::Vector3 lb_; //position of antenna in body frame
      gtsam::Vector3 lb2_ = gtsam::Point3(0, 0, 0); //position of antenna in body frame secondary antenna
      bool useAutoDiff_ = false;

      typedef DDCarrierPhaseFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector> Base;

    public:
      DDCarrierPhaseFactor() = default;

      //Constructor when dd has to be calc
      DDCarrierPhaseFactor(gtsam::Key point_i, gtsam::Key ambiguity_i, const double &carPhaMR,
                           const double &carPhaMB, const double &carPhaIR, const double &carPhaIB,
                           const gtsam::Vector3 &pointMaster, const gtsam::Vector3 &pointSatI,
                           const gtsam::Point3 &pointBase, const int nAmbiguity,
                           const gtsam::Vector3 &lb, const double &lambda,
                           const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, point_i, ambiguity_i), lambda_(lambda), nAmbiguity_(nAmbiguity),
              pointMaster_(pointMaster), pointSatI_(pointSatI), pointBase_(pointBase), lb_(lb), useAutoDiff_(useAutoDiff) {
        ddphi_ = (carPhaMR - carPhaMB) - (carPhaIR - carPhaIB);
        factorTypeID_ = FactorTypeIDs::DDCP;
        factorName_ = "DDCarrierPhaseFactor";
      }

      //when measurmeent is already calc
      DDCarrierPhaseFactor(gtsam::Key pose_i, gtsam::Key ambiguity_i, const double &ddphi,
                           const gtsam::Vector3 &pointMaster, const gtsam::Vector3 &pointSatI,
                           const gtsam::Vector3 &pointBase, const int nAmbiguity,
                           const gtsam::Vector3 &lb, const double &lambda,
                           const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, pose_i, ambiguity_i), ddphi_(ddphi), lambda_(lambda),
              nAmbiguity_(nAmbiguity), pointMaster_(pointMaster), pointSatI_(pointSatI), pointBase_(pointBase),
              lb_(lb), useAutoDiff_(useAutoDiff) {
        factorTypeID_ = FactorTypeIDs::DDCP;
        factorName_ = "DDCarrierPhaseFactor";
      }

      //for construction of aux measurement (dual antenna)
      DDCarrierPhaseFactor(gtsam::Key point_i, gtsam::Key ambiguity_i, const double &ddphi,
                           const gtsam::Vector3 &pointMaster, const gtsam::Vector3 &pointSatI, const int nAmbiguity,
                           const gtsam::Vector3 &lb, const gtsam::Vector3 &lb2, const double &lambda,
                           const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, point_i, ambiguity_i), ddphi_(ddphi), lambda_(lambda), nAmbiguity_(nAmbiguity),
              pointMaster_(pointMaster),
              pointSatI_(pointSatI), lb_(lb), lb2_(lb2), useAutoDiff_(useAutoDiff){
        factorTypeID_ = FactorTypeIDs::DDCP;
        factorName_ = "DDCarrierPhaseFactor";
      }

      ~DDCarrierPhaseFactor() override = default;

      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector &ddAmbiguity,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, ddAmbiguity), pose, 1e-5);
          if (H2) {
            gtsam::Matrix temp = gtsam::Matrix::Zero(1, ddAmbiguity.size());
            temp(nAmbiguity_) = lambda_;
            *H2 = temp;
          }
          return evaluateError_(pose, ddAmbiguity);
        }
        else
        {
          //initialize some variables
          gtsam::Matrix13 e_RM, e_RI; //unity vector pointing from receiver to satellite (1x3)
          gtsam::Matrix13 e_RM2, e_RI2;
          gtsam::Vector3 positionReceiver = pose.translation() + pose.rotation() * lb_; //pose_Antenna wrt to Earth

          //calculate ranges
          double rangeRM, rangeRI, rangeBM, rangeBI;
          if (lb2_.norm() == 0) {
            rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
            rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
            rangeBM = gtsam::distance3(pointBase_, pointMaster_);
            rangeBI = gtsam::distance3(pointBase_, pointSatI_);
          } else {
            gtsam::Vector3 positionReceiver2 = pose.translation() + pose.rotation() * lb2_;
            rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
            rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
            rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, e_RM2);
            rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, e_RI2);
          }

          if (H1 || H2) {
            gtsam::Matrix rotation = pose.rotation().matrix(); // eRb (3x3)
            gtsam::Matrix jacobian =
                (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                (e_RM2 - e_RI2) * rotation * gtsam::skewSymmetric(-lb2_); //calculate Jacobian for Rotation
            if (H1)
              *H1 = (gtsam::Matrix16() << jacobian, ((e_RM - e_RI) - (e_RM2 - e_RI2)) *
                                                    rotation/*THIS ROT ONLY CORRECTION*/).finished(); // derivation pose
            if (H2) {
              *H2 = gtsam::Vector::Zero(ddAmbiguity.size(), 1);
              H2->coeffRef(nAmbiguity_) = -lambda_;
            }
          }

          return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) - lambda_ *
                                                                                  (ddAmbiguity(nAmbiguity_) +
                                                                                   ddphi_)).finished();
        }
      }

      [[nodiscard]] gtsam::Vector1 evaluateError_(const gtsam::Pose3 &poseReceiver, const gtsam::Vector &ddAmbiguity) const {

        gtsam::Point3 positionReceiver = poseReceiver.translation() + poseReceiver.rotation().rotate(lb_);

        double rangeRM, rangeRI, rangeBM, rangeBI;
        if (lb2_.norm() == 0) {
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_);
          rangeBM = gtsam::distance3(pointBase_, pointMaster_);
          rangeBI = gtsam::distance3(pointBase_, pointSatI_);
        } else {
          //this factor gets pretty useless, for small lb to distance ratio
          gtsam::Point3 positionReceiver2 = poseReceiver.translation() + poseReceiver.rotation().rotate(lb2_);
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_);
          rangeBM = gtsam::distance3(positionReceiver2, pointMaster_);
          rangeBI = gtsam::distance3(positionReceiver2, pointSatI_);

        }
        //The + before ddphi_ because in PP CP is DD different to PR and DR
        return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) + lambda_ * (ddAmbiguity(nAmbiguity_) - ddphi_) ).finished();
      }

      /** return the measured */
      [[nodiscard]] double measured() const {
        return ddphi_;
      }

      /** equals specialized to this factor */
      [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != nullptr && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->ddphi_).finished(),
                                            (gtsam::Vector1() << e->ddphi_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "DDCarrierphaseFactor:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("DDCarrierphaseFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(ddphi_);
      }

    };
  }

namespace gtsam {
  template<>
  struct traits<fgo::factor::DDCarrierPhaseFactor> : public Testable<fgo::factor::DDCarrierPhaseFactor> {
  };
}

#endif //ONLINE_FGO_DDCPFACTOR_H
