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

#ifndef ONLINE_FGO_TIMEDCARRIERPHASEFACTOR_H
#define ONLINE_FGO_TIMEDCARRIERPHASEFACTOR_H

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
#include "factor/FactorTypeIDs.h"

/*
 * Inputs:
 * Keys X(i) & X(j)
 * DDCarrierPhase measurements at i and j: carPhaMi, carPhaMj
 * pointMaster(i) & pointSatI(i)
 * pointMaster(j) & pointSatI(j)
 * pointBase
 * lb
 * Covariance Matrix model
 * */
/* measurement equation used:
 * lambda * (carrierphase j - carrierphase i) = Distance of Satellite and Receiver j - Distance of Satellite and Receiver j + clock bias j - clock bias i
 * somtimes the AmbiguityCycles get added, but its just a definition in DD measurements it doesnt matter because it can be neg anyway*/
/*Jacobian: e_RS * (R * skew(t), R) for i and j
 * */
//Lyu, Bai, Lai, Wang and Huang - Optimal Time Difference-based TDCP-GPS/IMU Navigation using GraphBase Optimization

namespace fgo::factor {

    class TripleDiffCPFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
    protected:
      double lambda_{}; //wavelength
      gtsam::Point3 lb_; // place of Ant wrt to body
      double phi_ji{}; // measurement
      gtsam::Point3 pointMasteri_; //position Master Satellite
      gtsam::Point3 pointSatIi_; //position Satellite I
      gtsam::Point3 pointBase_; //position of Base
      gtsam::Point3 pointMasterj_; //position Master Satellite
      gtsam::Point3 pointSatIj_; //position Satellite I
      bool useAutoDiff_ = false;


      typedef TripleDiffCPFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;

    public:
      TripleDiffCPFactor() = default;

      TripleDiffCPFactor(gtsam::Key pose_i, gtsam::Key pose_j, const double &ddphi_i, const double &ddphi_j,
                         const gtsam::Point3 &pointMaster_i, const gtsam::Point3 &pointSatI_i,
                         const gtsam::Point3 &pointMaster_j, const gtsam::Point3 &pointSatI_j,
                         const gtsam::Point3 &pointBase, const gtsam::Point3 &lb, const double &lambda,
                         const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
              Base(model, pose_i, pose_j), lambda_(lambda), lb_(lb), pointMasteri_(pointMaster_i),
              pointSatIi_(pointSatI_i), pointBase_(pointBase), pointMasterj_(pointMaster_j),
              pointSatIj_(pointSatI_j), useAutoDiff_(useAutoDiff) {
        phi_ji = ddphi_j - ddphi_i;
        factorTypeID_ = FactorTypeIDs::TDCP3;
        factorName_ = "TripleDiffCPFactor";
      }

      ~TripleDiffCPFactor() override = default;


      [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override {

        if(useAutoDiff_)
        {
          if (H1)
            *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, boost::placeholders::_1, pose_j), pose_i, 1e-5);
          if (H2) {
            *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                boost::bind(&This::evaluateError_, this, pose_i, boost::placeholders::_1), pose_j, 1e-5);
          }
          return evaluateError_(pose_i, pose_j);
        }
        else
        {
          //initialize some variables
          gtsam::Matrix13 e_RMi, e_RIi; //unity vector pointing from receiver to satellite (1x3)
          gtsam::Point3 positionReceiveri = pose_i.translation() + pose_i.rotation() * lb_; //pose_Antenna wrt to Earth

          //calculate ranges
          double rangeRMi = gtsam::distance3(positionReceiveri, pointMasteri_, &e_RMi);
          double rangeRIi = gtsam::distance3(positionReceiveri, pointSatIi_, &e_RIi);
          double rangeBMi = gtsam::distance3(pointBase_, pointMasteri_);
          double rangeBIi = gtsam::distance3(pointBase_, pointSatIi_);

          //initialize some variables
          gtsam::Matrix13 e_RMj, e_RIj; //unity vector pointing from receiver to satellite (1x3)
          gtsam::Point3 positionReceiverj = pose_j.translation() + pose_j.rotation() * lb_; //pose_Antenna wrt to Earth

          //calculate ranges
          double rangeRMj = gtsam::distance3(positionReceiverj, pointMasterj_, &e_RMj);
          double rangeRIj = gtsam::distance3(positionReceiverj, pointSatIj_, &e_RIj);
          double rangeBMj = gtsam::distance3(pointBase_, pointMasterj_);
          double rangeBIj = gtsam::distance3(pointBase_, pointSatIj_);
          //calc jacobians
          if (H1 || H2) {
            gtsam::Matrix rotationi = pose_i.rotation().matrix(); // eRb (3x3)
            gtsam::Matrix rotationj = pose_j.rotation().matrix(); // eRb (3x3)
            gtsam::Matrix16 jacobiani =
                (gtsam::Matrix16() << (e_RMi - e_RIi) * rotationi * gtsam::skewSymmetric(-lb_), (e_RMi - e_RIi) *
                                                                                                rotationi/*THIS ROT ONLY CORRECTION*/).finished(); //calculate Jacobian for Posei
            gtsam::Matrix16 jacobianj =
                (gtsam::Matrix16() << (e_RMj - e_RIj) * rotationj * gtsam::skewSymmetric(-lb_), e_RMj - e_RIj *
                                                                                                        rotationj/*THIS ROT ONLY CORRECTION*/).finished(); //calculate Jacobian for Posej
            if (H1)
              *H1 = (gtsam::Matrix16() << -jacobiani).finished();
            if (H2)
              *H2 = (gtsam::Matrix16() << jacobianj).finished();
          }
          double rangei = (rangeRMi - rangeBMi) - (rangeRIi - rangeBIi);
          double rangej = (rangeRMj - rangeBMj) - (rangeRIj - rangeBIj);
          return (gtsam::Vector1() << rangej - rangei - lambda_ * (phi_ji)).finished();
        }
      }

      [[nodiscard]] gtsam::Vector1 evaluateError_(const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j) const {
        //calc pos
        gtsam::Point3 positionReceiveri = pose_i.translation() + pose_i.rotation() * lb_; //pose_Antenna wrt to Earth
        //calculate ranges
        double rangeRMi = gtsam::distance3(positionReceiveri, pointMasteri_);
        double rangeRIi = gtsam::distance3(positionReceiveri, pointSatIi_);
        double rangeBMi = gtsam::distance3(pointBase_, pointMasteri_);
        double rangeBIi = gtsam::distance3(pointBase_, pointSatIi_);

        //calc pos
        gtsam::Point3 positionReceiverj = pose_j.translation() + pose_j.rotation() * lb_; //pose_Antenna wrt to Earth
        //calculate ranges
        double rangeRMj = gtsam::distance3(positionReceiverj, pointMasterj_);
        double rangeRIj = gtsam::distance3(positionReceiverj, pointSatIj_);
        double rangeBMj = gtsam::distance3(pointBase_, pointMasterj_);
        double rangeBIj = gtsam::distance3(pointBase_, pointSatIj_);

        double rangei = (rangeRMi - rangeBMi) - (rangeRIi - rangeBIi);
        double rangej = (rangeRMj - rangeBMj) - (rangeRIj - rangeBIj);
        double error = rangej - rangei - lambda_ * phi_ji;
        return (gtsam::Vector1() << error).finished();
      }


      /** return the measured */

      [[nodiscard]] double measured() const {
        return phi_ji;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->phi_ji).finished(),
                                            (gtsam::Vector1() << e->phi_ji).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "TripleDiffCPFactor:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("TripleDiffCPFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(phi_ji);
      }

    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    class AmbiguityLockFactor : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
    private:
      typedef AmbiguityLockFactor This;
      typedef gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> Base;

      int n1_;
      int n2_;
    public:
      AmbiguityLockFactor() {}

      AmbiguityLockFactor(gtsam::Key kAmbiguity1, gtsam::Key kAmbiguity2, const int &n1, const int &n2,
                          const gtsam::SharedNoiseModel &model) :
              Base(model, kAmbiguity1, kAmbiguity2), n1_(n1), n2_(n2) {}

      virtual ~AmbiguityLockFactor() {}

      gtsam::Vector evaluateError(const gtsam::Vector &ambiguity1, const gtsam::Vector &ambiguity2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        if (H1) {
          gtsam::Matrix temp = gtsam::Matrix::Zero(1, ambiguity1.size());
          temp(n1_) = 1;
          *H1 = temp;
        }
        if (H2) {
          gtsam::Matrix temp = gtsam::Matrix::Zero(1, ambiguity2.size());
          temp(n2_) = -1;
          *H2 = temp;
        }
        return (gtsam::Vector1() << (ambiguity1(n1_) - ambiguity2(n2_))).finished();
      }

      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      /** number of variables attached to this factor */
      size_t size() const {
        return 2;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol(gtsam::Vector1(n1_, n2_),
                                            gtsam::Vector2(e->n1_, e->n2_), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "AmbiguityLockFactor:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("PrDrFactor",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(n1_);
      }
    };
  }

#endif //ONLINE_FGO_TIMEDCARRIERPHASEFACTOR_H
namespace gtsam {
    template<>
    struct traits<fgo::factor::TripleDiffCPFactor> :
            public Testable<fgo::factor::TripleDiffCPFactor> {
    };
  template<>
  struct traits<fgo::factor::AmbiguityLockFactor> :
          public Testable<fgo::factor::AmbiguityLockFactor> {
  };
}