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

#ifndef ONLINE_FGO_GPINTERPOLATEDTDCPFACTORNORMALCP_H
#define ONLINE_FGO_GPINTERPOLATEDTDCPFACTORNORMALCP_H

#include <utility>
#include "model/gp_interpolator/GPWNOAInterpolatorPose3.h"
#include "data/FactorTypes.h"
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {

class GPSingleInterpolatedTDNCPFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector2, gtsam::Vector>
{

};


  class GPInterpolatedTDNCPFactor : public fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector2, gtsam::Vector ,
          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector> {
  private:
    //measurements
    double phi_i{}, phi_j{}; //DD measurement
    //constants
    gtsam::Point3 pointSatI_i_, pointSatI_j_;  //position Satellite I
    int nAmbiguity_i_{}, nAmbiguity_j_{}; //position of ambiguity in vector
    double lambda_{}; //wavelength
    gtsam::Point3 lb_; //position of antenna in body frame
    gtsam::Point3 lb2_ = gtsam::Point3(0, 0, 0);
    double dt = 0.0; //time between both measurmeents

    std::shared_ptr<fgo::models::GPInterpolator> GPbase_i_, GPbase_j_;

    typedef GPInterpolatedTDNCPFactor This;
    typedef fgo::NoiseModelFactor12<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector2, gtsam::Vector,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector> Interpolator;


  public:
      GPInterpolatedTDNCPFactor() = default;

    // ddmeasurements not known when constructed
    GPInterpolatedTDNCPFactor(
            const gtsam::Key &point_0, const gtsam::Key &vel_0, const gtsam::Key &omega_0,
            const gtsam::Key &point_1, const gtsam::Key &vel_1, const gtsam::Key &omega_1, const gtsam::Key &cbd_1,
            const gtsam::Key &ambiguity_i,
            const gtsam::Key &point_2, const gtsam::Key &vel_2, const gtsam::Key &omega_2,
            const gtsam::Key &ambiguity_j,
            const double &carpha_i, const double &carpha_j,
            const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
            const int nAmbiguity_i, const int nAmbiguity_j,
            const gtsam::Point3 &lb, const double &lambda, const gtsam::SharedNoiseModel &model,
            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j) :
            Interpolator(model, point_0, vel_0, omega_0, point_1, vel_1, omega_1, cbd_1, ambiguity_i, point_2,
                         vel_2, omega_2, ambiguity_j), //
            phi_i(carpha_i), phi_j(carpha_j), pointSatI_i_(pointSat_i), pointSatI_j_(pointSat_j),
            nAmbiguity_i_(nAmbiguity_i), nAmbiguity_j_(nAmbiguity_j), lambda_(lambda), lb_(lb),
            GPbase_i_(interpolator_i),
            GPbase_j_(interpolator_j) {
      dt = GPbase_j_->getDeltat() - GPbase_j_->getTau() + GPbase_i_->getTau();
      //std::cout << " TDCP Dt: " << dt << std::endl;
      //std::cout << " TDCP GPbase_i_->getDeltat(): " << GPbase_i_->getDeltat() << std::endl;
      //std::cout << " TDCP GPbase_i_->getTau(): " << GPbase_i_->getTau() << std::endl;
      //std::cout << " TDCP GPbase_j_->getDeltat(): " << GPbase_j_->getDeltat() << std::endl;
      //std::cout << " TDCP GPbase_j_->getTau(): " << GPbase_j_->getTau() << std::endl;
    }

      GPInterpolatedTDNCPFactor(const gtsam::Key &point_0, const gtsam::Key &vel_0, const gtsam::Key &omega_0,
                                  const gtsam::Key &point_1, const gtsam::Key &vel_1, const gtsam::Key &omega_1,
                                  const gtsam::Key &cbd_1, const gtsam::Key &ambiguity_i,
                                  const gtsam::Key &point_2, const gtsam::Key &vel_2, const gtsam::Key &omega_2,
                                  const gtsam::Key &ambiguity_j,
                                  const double &carpha_i, const double &carpha_j,
                                  const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
                                  const int nAmbiguity_i, const int nAmbiguity_j,
                                  const gtsam::Point3 &lb, const gtsam::Point3 &lb2,
                                  const double &lambda, const gtsam::SharedNoiseModel &model,
                                  const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
                                  const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j) :
            Interpolator(model, point_0, vel_0, omega_0, point_1, vel_1, cbd_1, omega_1, ambiguity_i, point_2,
                         vel_2, omega_2, ambiguity_j), //
            phi_i(carpha_i), phi_j(carpha_j), pointSatI_i_(pointSat_i), pointSatI_j_(pointSat_j),
            nAmbiguity_i_(nAmbiguity_i), nAmbiguity_j_(nAmbiguity_j), lambda_(lambda), lb_(lb), lb2_(lb2),
            GPbase_i_(interpolator_i), GPbase_j_(interpolator_j) {
      dt = GPbase_i_->getDeltat() - GPbase_i_->getTau() + GPbase_j_->getTau();
     // std::cout << " TDCP Dt: " << dt << std::endl;
    }

    ~GPInterpolatedTDNCPFactor() override = default;

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(
            const gtsam::Pose3 &pose0, const gtsam::Vector3 &vel0, const gtsam::Vector3 &omega0,
            const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
            const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
            const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
            const gtsam::Vector &ambiguityj,
            boost::optional<gtsam::Matrix &> H1 = boost::none,
            boost::optional<gtsam::Matrix &> H2 = boost::none,
            boost::optional<gtsam::Matrix &> H3 = boost::none,
            boost::optional<gtsam::Matrix &> H4 = boost::none,
            boost::optional<gtsam::Matrix &> H5 = boost::none,
            boost::optional<gtsam::Matrix &> H6 = boost::none,
            boost::optional<gtsam::Matrix &> H7 = boost::none,
            boost::optional<gtsam::Matrix &> H8 = boost::none,
            boost::optional<gtsam::Matrix &> H9 = boost::none,
            boost::optional<gtsam::Matrix &> H10 = boost::none,
            boost::optional<gtsam::Matrix &> H11 = boost::none,
            boost::optional<gtsam::Matrix &> H12 = boost::none
    ) const override {

      if (H1)
        *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, std::placeholders::_1, vel0, omega0, pose1, vel1, omega1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), pose0);
      if (H2)
        *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, std::placeholders::_1, omega0, pose1, vel1, omega1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), vel0);
      if (H3)
        *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, std::placeholders::_1, pose1, vel1, omega1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), omega0);
      if (H4)
        *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, std::placeholders::_1, vel1, omega1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), pose1);
      if (H5)
        *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, std::placeholders::_1, omega1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), vel1);
      if (H6)
        *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, vel1, std::placeholders::_1, cbd1,
                          ambiguityi, pose2, vel2, omega2, ambiguityj), omega1);
      if (H7)
        *H7 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector2>(
            std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, vel1, omega1, std::placeholders::_1,
                      ambiguityi, pose2, vel2, omega2, ambiguityj), cbd1);

      if (H8) {
        *H8 = gtsam::Matrix::Zero(1, ambiguityi.size());
        H8->coeffRef(nAmbiguity_i_) = -lambda_; //TODO
      }
      if (H9)
        *H9 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, vel1, omega1, cbd1,
                          ambiguityi, std::placeholders::_1, vel2, omega2, ambiguityj), pose2);
      if (H10)
        *H10 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, vel1, omega1, cbd1,
                          ambiguityi, pose2, std::placeholders::_1, omega2, ambiguityj), vel2);
      if (H11)
        *H11 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, pose1, vel1, omega1, cbd1,
                          ambiguityi, pose2, vel2, std::placeholders::_1, ambiguityj), omega2);
      if (H12) {
        *H12 = gtsam::Matrix::Zero(1, ambiguityj.size());
        H12->coeffRef(nAmbiguity_j_) = lambda_; //TODO
      }

      return evaluateError_(pose0, vel0, omega0, pose1, vel1, omega1, cbd1, ambiguityi, pose2, vel2, omega2,
                            ambiguityj);
    }

    [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose0, const gtsam::Vector3 &vel0, const gtsam::Vector3 &omega0,
                                 const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                                 const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
                                 const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                                 const gtsam::Vector &ambiguityj) const {
      gtsam::Pose3 poseBody_i, poseBody_j; //interpolated position

      //gtsam::Vector6 omVel0 = (gtsam::Vector6() << omega0, vel0).finished();
     // gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      //gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      poseBody_i = GPbase_i_->interpolatePose(pose0, vel0, omega0, pose1, vel1, omega1);
      poseBody_j = GPbase_j_->interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);

      gtsam::Point3 positionReceiver_i, positionReceiver_j;
      double range_i, range_j;
      gtsam::Matrix13 e_j;

      //calculate distances
      if (lb2_.norm() == 0) {
        positionReceiver_i = poseBody_i.translation() + poseBody_i.rotation() * lb_; //pose_Antenna wrt to Earth
        positionReceiver_j = poseBody_j.translation() + poseBody_j.rotation() * lb_;
        range_i = gtsam::distance3(positionReceiver_i, pointSatI_i_);
        range_j = gtsam::distance3(positionReceiver_j, pointSatI_j_, e_j);
      } else {
        positionReceiver_i = poseBody_i.translation() + poseBody_i.rotation() * lb2_; //pose_Antenna wrt to Earth
        positionReceiver_j = poseBody_j.translation() + poseBody_j.rotation() * lb2_;
        range_i = gtsam::distance3(positionReceiver_i, pointSatI_i_);
        range_j = gtsam::distance3(positionReceiver_j, pointSatI_j_, e_j);
      }

      //std::cout << " ambiguity sub: " << ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_) << std::endl;

      auto err = (gtsam::Vector1() << (range_j - range_i) + e_j * (positionReceiver_j - positionReceiver_i) + cbd1(1) * dt +
                                      lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))
                                      - lambda_ * (phi_j - phi_i)).finished();

      //std::cout << " range_j - range_i " << range_j - range_i << std::endl;
//std::cout << " amb " << ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_) << std::endl;
      //std::cout << " e_j * (positionReceiver_j - positionReceiver_i) " << e_j * (positionReceiver_j - positionReceiver_i) << std::endl;
      //std::cout << " cbd1(1) * dt_  " << cbd1(1) * dt  << std::endl;
      //std::cout << " summe:  " <<  (range_j - range_i) + e_j * (positionReceiver_j - positionReceiver_i) + cbd1(1) * dt +
       //                                               lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))  << std::endl;
      //std::cout << " meas " << lambda_ * (phi_j - phi_i) << std::endl;
      //std::cout << " err " << err << std::endl;

      return err;

      //return (gtsam::Vector1() << e_j * (positionReceiver_j - positionReceiver_i)
      //return (gtsam::Vector1() << range_j - range_i
      //                            + cbd1(1) * dt + (range_j - range_i) +
      //                            lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))
      //                            - lambda_ * (phi_j - phi_i)).finished();

    }

    /** return the measured */

    [[nodiscard]] gtsam::Vector measured() const {
      return (gtsam::Vector2() << this->phi_i, this->phi_j).finished();
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != NULL && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->phi_i, this->phi_j).finished(),
                                          (gtsam::Vector2() << e->phi_i, e->phi_j).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterDDPseudorange:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(phi_i);
    }
  };

  /////////////////////////////////////////////////////////////////////////

  class TDNCPFactor : public gtsam::NoiseModelFactor5< gtsam::Pose3, gtsam::Vector2, gtsam::Vector,
          gtsam::Pose3, gtsam::Vector> {
  private:
    //measurements
    double phi_i{}, phi_j{}; //DD measurement
    //constants
    gtsam::Point3 pointSatI_i_, pointSatI_j_;  //position Satellite I
    int nAmbiguity_i_{}, nAmbiguity_j_{}; //position of ambiguity in vector
    double lambda_{}; //wavelength
    gtsam::Point3 lb_; //position of antenna in body frame
    gtsam::Point3 lb2_ = gtsam::Point3(0, 0, 0);
    double dt_{}; //time between both measurements

    typedef TDNCPFactor This;
    typedef gtsam::NoiseModelFactor5< gtsam::Pose3, gtsam::Vector2, gtsam::Vector,
            gtsam::Pose3, gtsam::Vector> Interpolator;


  public:
    TDNCPFactor() = default;

    // ddmeasurements not known when constructed
    TDNCPFactor(
            const gtsam::Key &point_1, const gtsam::Key &cbd_1, const gtsam::Key &ambiguity_1,
            const gtsam::Key &point_2, const gtsam::Key &ambiguity_2,
            const double &carpha_i, const double &carpha_j,
            const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
            const int nAmbiguity_i, const int nAmbiguity_j, double dt,
            const gtsam::Point3 &lb, const double &lambda, const gtsam::SharedNoiseModel &model) :
            Interpolator(model, point_1, cbd_1, ambiguity_1, point_2, ambiguity_2), //
            phi_i(carpha_i), phi_j(carpha_j), pointSatI_i_(pointSat_i), pointSatI_j_(pointSat_j),
            nAmbiguity_i_(nAmbiguity_i), nAmbiguity_j_(nAmbiguity_j), lambda_(lambda), lb_(lb) {
      dt_ = dt;
    }

    TDNCPFactor(
                const gtsam::Key &point_1, const gtsam::Key &cbd_1, const gtsam::Key &ambiguity_1,
                const gtsam::Key &point_2, const gtsam::Key &ambiguity_2,
                const double &carpha_i, const double &carpha_j,
                const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
                const int nAmbiguity_i, const int nAmbiguity_j, double dt,
                const gtsam::Point3 &lb, const gtsam::Point3 &lb2,
                const double &lambda, const gtsam::SharedNoiseModel &model,
                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j) :
            Interpolator(model, point_1, cbd_1, ambiguity_1, point_2, ambiguity_2), //
            phi_i(carpha_i), phi_j(carpha_j), pointSatI_i_(pointSat_i), pointSatI_j_(pointSat_j),
            nAmbiguity_i_(nAmbiguity_i), nAmbiguity_j_(nAmbiguity_j), lambda_(lambda), lb_(lb), lb2_(lb2) {
      dt_ = dt;
    }

    ~TDNCPFactor() override = default;

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(
            const gtsam::Pose3 &pose1, const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
            const gtsam::Pose3 &pose2, const gtsam::Vector &ambiguityj,
            boost::optional<gtsam::Matrix &> H1 = boost::none,
            boost::optional<gtsam::Matrix &> H2 = boost::none,
            boost::optional<gtsam::Matrix &> H3 = boost::none,
            boost::optional<gtsam::Matrix &> H4 = boost::none,
            boost::optional<gtsam::Matrix &> H5 = boost::none
    ) const override {

      if (H1)
        *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, std::placeholders::_1, cbd1,
                          ambiguityi, pose2, ambiguityj), pose1);
      if (H2)
        *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector2>(
            std::bind(&This::evaluateError_, this, pose1, std::placeholders::_1,
                      ambiguityi, pose2, ambiguityj), cbd1);
      if (H3) {
        *H3 = gtsam::Matrix::Zero(1, ambiguityi.size());
        H3->coeffRef(nAmbiguity_i_) = -lambda_; //TODO
      }
      if (H4)
        *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, pose1, cbd1,
                          ambiguityi, std::placeholders::_1, ambiguityj), pose2);
      if (H5) {
        *H5 = gtsam::Matrix::Zero(1, ambiguityj.size());
        H5->coeffRef(nAmbiguity_j_) = lambda_; //TODO
      }

      return evaluateError_(pose1, cbd1, ambiguityi, pose2, ambiguityj);
    }

    [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
                                 const gtsam::Pose3 &pose2, const gtsam::Vector &ambiguityj) const {

      gtsam::Point3 positionReceiver_i, positionReceiver_j;
      double range_i, range_j;
      gtsam::Matrix13 e_j;

      //calculate distances
      if (lb2_.norm() == 0) {
        positionReceiver_i = pose1.translation() + pose1.rotation() * lb_; //pose_Antenna wrt to Earth
        positionReceiver_j = pose2.translation() + pose2.rotation() * lb_;
        range_i = gtsam::distance3(positionReceiver_i, pointSatI_i_);
        range_j = gtsam::distance3(positionReceiver_j, pointSatI_j_, e_j);
      } else {
        positionReceiver_i = pose1.translation() + pose1.rotation() * lb2_; //pose_Antenna wrt to Earth
        positionReceiver_j = pose2.translation() + pose2.rotation() * lb2_;
        range_i = gtsam::distance3(positionReceiver_i, pointSatI_i_);
        range_j = gtsam::distance3(positionReceiver_j, pointSatI_j_, e_j);
      }

      /*std::cout << std::setprecision(9) <<"e * p:" << e_j * (positionReceiver_j - positionReceiver_i) << "cbd * dt: " << cbd1(1) * dt_ <<
      "L: " << (range_j - range_i) << "lambda * del_amb: " << lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))
      << "del phi: " << lambda_ * (phi_j - phi_i) << std::endl;*/

     // std::cout << " ambiguity sub: " << ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_) << std::endl;

      auto err = (gtsam::Vector1() << (range_j - range_i) + e_j * (positionReceiver_j - positionReceiver_i) + cbd1(1) * dt_ +
                                      lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))
                                      - lambda_ * (phi_j - phi_i)).finished();

      //std::cout << " range_j - range_i " << range_j - range_i << std::endl;
      //std::cout << " e_j * (positionReceiver_j - positionReceiver_i) " << e_j * (positionReceiver_j - positionReceiver_i) << std::endl;
      //std::cout << " cbd1(1) * dt_  " << cbd1(1) * dt_  << std::endl;
      //std::cout << " summe:  " << (range_j - range_i) + e_j * (positionReceiver_j - positionReceiver_i) + cbd1(1) * dt_ +
       //                                               lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))  << std::endl;
     // std::cout << " meas " << lambda_ * (phi_j - phi_i) << std::endl;
      //std::cout << " err " << err << std::endl;


      return err;
    /* Formel von 鈴木/すずき
      return (gtsam::Vector1() << e_j * (positionReceiver_j - positionReceiver_i)
                                  + cbd1(1) * dt_ + (range_j - range_i) -
                                  lambda_ * (ambiguityj(nAmbiguity_j_) - ambiguityi(nAmbiguity_i_))
                                  - lambda_ * (phi_j - phi_i)).finished();*/
    }

    /** return the measured */

    [[nodiscard]] gtsam::Vector measured() const {
      return (gtsam::Vector2() << this->phi_i, this->phi_j).finished();
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->phi_i, this->phi_j).finished(),
                                          (gtsam::Vector2() << e->phi_i, e->phi_j).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterDDPseudorange:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(phi_i);
    }
  };


////////////////////////////////////////////////////////////////////////

  class CycleSlipFactor : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
  private:

    typedef CycleSlipFactor This;
    typedef gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> Interpolator;
    int n1_ = -1, n2_ = -1;

  public:
    CycleSlipFactor() = default;

    // ddmeasurements not known when constructed
    CycleSlipFactor(const gtsam::Key &amb_1, const gtsam::Key &amb_2, const int n1, const int n2,
                    const gtsam::SharedNoiseModel &model) :
            Interpolator(model, amb_1, amb_2), n1_(n1), n2_(n2) {}

    ~CycleSlipFactor() override = default;

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(
            const gtsam::Vector &amb1, const gtsam::Vector &amb2,
            boost::optional<gtsam::Matrix &> H1 = boost::none,
            boost::optional<gtsam::Matrix &> H2 = boost::none) const override {

      if (H1) {
        *H1 = gtsam::Matrix::Zero(1, amb1.size());
        H1->coeffRef(n1_) = -1;
      }
      if (H2){
        *H2 = gtsam::Matrix::Zero(1, amb2.size());
        H2->coeffRef(n2_) = 1;
      }

      return evaluateError_(amb1, amb2);

    }

    [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Vector &amb1, const gtsam::Vector &amb2) const {

      return (gtsam::Vector1() << amb2(n2_) - amb1(n1_)).finished();

    }


    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterDDPseudorange:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP("0");
    }
  };

  ////////////////////////////////////////////////////////////////////////

  class AmbiguitySoftLockFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
  private:

    typedef AmbiguitySoftLockFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Vector> Interpolator;
    int n_{};

  public:
    AmbiguitySoftLockFactor() = default;

    // ddmeasurements not known when constructed
    AmbiguitySoftLockFactor(const gtsam::Key &amb, const int n, const gtsam::SharedNoiseModel &model) :
            Interpolator(model, amb), n_(n) {}

    ~AmbiguitySoftLockFactor() override = default;

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(
            const gtsam::Vector &amb, boost::optional<gtsam::Matrix &> H1 = boost::none) const override {

      if (H1) {
        *H1 = gtsam::Matrix::Zero(1, amb.size());
        H1->coeffRef(n_) = 1;
      }
      return evaluateError_(amb);
    }

    [[nodiscard]] gtsam::Vector evaluateError_(const gtsam::Vector &amb) const {
      return (gtsam::Vector1() << amb(n_)).finished();
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterDDPseudorange:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP("0");
    }
  };

}

namespace gtsam {
  template<>
  struct traits<fgo::factor::TDNCPFactor> :
          public Testable<fgo::factor::TDNCPFactor> {
  };
}

namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedTDNCPFactor> :
          public Testable<fgo::factor::GPInterpolatedTDNCPFactor> {
  };
}

namespace gtsam {
  template<>
  struct traits<fgo::factor::CycleSlipFactor> :
          public Testable<fgo::factor::CycleSlipFactor> {
  };
}

namespace gtsam {
    template<>
    struct traits<fgo::factor::AmbiguitySoftLockFactor> :
        public Testable<fgo::factor::AmbiguitySoftLockFactor> {
    };
}

#endif //ONLINE_FGO_GPINTERPOLATEDTDCPFACTORNORMALCP_H
