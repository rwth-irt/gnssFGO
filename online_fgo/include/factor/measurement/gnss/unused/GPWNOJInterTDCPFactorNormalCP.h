// Created by lars on 18.07.22

#ifndef ONLINE_FGO_GPWNOJINTERTDCPFACTORNORMALCP_H
#define ONLINE_FGO_GPWNOJINTERTDCPFACTORNORMALCP_H

#include <utility>
#include "models/gp_interpolator/GPWNOJInterpolatorPose3.h"
#include "data/FactorTypes.h"

namespace fgo::factor {

class GPWNOJSingleInterpolatedTDNCPFactor : public NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
    gtsam::Pose3, gtsam::Vector6, gtsam::Vector2, gtsam::Vector>
{

};


class GPWNOJInterpolatedTDNCPFactor : public fgo::NoiseModelFactor15<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector2, gtsam::Vector ,
          gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector> {
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

    typedef GPWNOJInterpolatedTDNCPFactor This;
    typedef fgo::NoiseModelFactor15<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector2, gtsam::Vector ,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector> Interpolator;
  public:
    GPWNOJInterpolatedTDNCPFactor() = default;

    // ddmeasurements not known when constructed
    GPWNOJInterpolatedTDNCPFactor(
            const gtsam::Key &point_0, const gtsam::Key &vel_0, const gtsam::Key &omega_0, const gtsam::Key& acc_0,
            const gtsam::Key &point_1, const gtsam::Key &vel_1, const gtsam::Key &omega_1, const gtsam::Key& acc_1,
            const gtsam::Key &cbd_1, const gtsam::Key &ambiguity_i,
            const gtsam::Key &point_2, const gtsam::Key &vel_2, const gtsam::Key &omega_2, const gtsam::Key& acc_2,
            const gtsam::Key &ambiguity_j,
            const double &carpha_i, const double &carpha_j,
            const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
            const int nAmbiguity_i, const int nAmbiguity_j,
            const gtsam::Point3 &lb, const double &lambda, const gtsam::SharedNoiseModel &model,
            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j) :
            Interpolator(model, point_0, vel_0, omega_0, acc_0, point_1, vel_1, omega_1, acc_1, cbd_1, ambiguity_i, point_2,
                         vel_2, omega_2, acc_2, ambiguity_j), //
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

    GPWNOJInterpolatedTDNCPFactor(const gtsam::Key &point_0, const gtsam::Key &vel_0, const gtsam::Key &omega_0, const gtsam::Key& acc_0,
                                  const gtsam::Key &point_1, const gtsam::Key &vel_1, const gtsam::Key &omega_1, const gtsam::Key& acc_1,
                                  const gtsam::Key &cbd_1, const gtsam::Key &ambiguity_i,
                                  const gtsam::Key &point_2, const gtsam::Key &vel_2, const gtsam::Key &omega_2, const gtsam::Key& acc_2,
                                  const gtsam::Key &ambiguity_j,
                                  const double &carpha_i, const double &carpha_j,
                                  const gtsam::Point3 &pointSat_i, const gtsam::Point3 &pointSat_j,
                                  const int nAmbiguity_i, const int nAmbiguity_j,
                                  const gtsam::Point3 &lb, const gtsam::Point3 &lb2,
                                  const double &lambda, const gtsam::SharedNoiseModel &model,
                                  const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_i,
                                  const std::shared_ptr<fgo::models::GPInterpolator> &interpolator_j) :
            Interpolator(model, point_0, vel_0, omega_0, acc_0, point_1, vel_1, omega_1, acc_1, cbd_1, ambiguity_i,
                         point_2, vel_2, omega_2, acc_2, ambiguity_j), //
            phi_i(carpha_i), phi_j(carpha_j), pointSatI_i_(pointSat_i), pointSatI_j_(pointSat_j),
            nAmbiguity_i_(nAmbiguity_i), nAmbiguity_j_(nAmbiguity_j), lambda_(lambda), lb_(lb), lb2_(lb2),
            GPbase_i_(interpolator_i), GPbase_j_(interpolator_j) {
      dt = GPbase_i_->getDeltat() - GPbase_i_->getTau() + GPbase_j_->getTau();
     // std::cout << " TDCP Dt: " << dt << std::endl;
    }

    ~GPWNOJInterpolatedTDNCPFactor() override = default;

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(
            const gtsam::Pose3 &pose0, const gtsam::Vector3 &vel0, const gtsam::Vector3 &omega0, const gtsam::Vector6& acc0,
            const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1, const gtsam::Vector6& acc1,
            const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
            const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2, const gtsam::Vector6& acc2,
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
            boost::optional<gtsam::Matrix &> H12 = boost::none,
            boost::optional<gtsam::Matrix &> H13 = boost::none,
            boost::optional<gtsam::Matrix &> H14 = boost::none,
            boost::optional<gtsam::Matrix &> H15 = boost::none
    ) const override {

      if (H1)
        *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, std::placeholders::_1, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), pose0);
      if (H2)
        *H2 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, std::placeholders::_1, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), vel0);
      if (H3)
        *H3 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, std::placeholders::_1, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), omega0);
      if (H4)
        *H4 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, pose0, vel0, omega0, std::placeholders::_1, pose1, vel1, omega1, acc1, cbd1,
                      ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), acc0);
      if (H5)
        *H5 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, std::placeholders::_1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), pose1);
      if (H6)
        *H6 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, std::placeholders::_1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), vel1);
      if (H7)
        *H7 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, std::placeholders::_1, acc1, cbd1,
                          ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), omega1);
      if (H8)
        *H8 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, std::placeholders::_1, cbd1,
                      ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), acc1);
      if (H9)
        *H9 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector2>(
            std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, std::placeholders::_1,
                      ambiguityi, pose2, vel2, omega2, acc2, ambiguityj), cbd1);
      if (H10) {
        *H10 = gtsam::Matrix::Zero(1, ambiguityi.size());
        H10->coeffRef(nAmbiguity_i_) = -lambda_; //TODO
      }
      if (H11)
        *H11 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, std::placeholders::_1, vel2, omega2, acc2, ambiguityj), pose2);
      if (H12)
        *H12 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, std::placeholders::_1, omega2, acc2, ambiguityj), vel2);
      if (H13)
        *H13 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
                std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                          ambiguityi, pose2, vel2, std::placeholders::_1, acc2, ambiguityj), omega2);
      if (H14)
        *H14 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1,
                      ambiguityi, pose2, vel2, omega2, std::placeholders::_1, ambiguityj), acc2);
      if (H15) {
        *H15 = gtsam::Matrix::Zero(1, ambiguityj.size());
        H15->coeffRef(nAmbiguity_j_) = lambda_; //TODO
      }

      return evaluateError_(pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1, cbd1, ambiguityi, pose2, vel2, omega2,
                            acc2, ambiguityj);
    }

    [[nodiscard]] gtsam::Vector evaluateError_(
        const gtsam::Pose3 &pose0, const gtsam::Vector3 &vel0, const gtsam::Vector3 &omega0, const gtsam::Vector6& acc0,
        const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1, const gtsam::Vector6& acc1,
        const gtsam::Vector2 &cbd1, const gtsam::Vector &ambiguityi,
        const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2, const gtsam::Vector6& acc2,
        const gtsam::Vector &ambiguityj) const
    {
      gtsam::Pose3 poseBody_i, poseBody_j; //interpolated position

      //gtsam::Vector6 omVel0 = (gtsam::Vector6() << omega0, vel0).finished();
     // gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1, vel1).finished();
      //gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2, vel2).finished();

      poseBody_i = GPbase_i_->interpolatePose(pose0, vel0, omega0, acc0, pose1, vel1, omega1, acc1);
      poseBody_j = GPbase_j_->interpolatePose(pose1, vel1, omega1, acc1, pose2, vel2, omega2, acc2);

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
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->phi_i, this->phi_j).finished(),
                                          (gtsam::Vector2() << e->phi_i, e->phi_j).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPWNOJInterpolatedTDNCPFactor:" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPWNOJInterpolatedTDNCPFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(phi_i);
    }
  };

  /////////////////////////////////////////////////////////////////////////
}

namespace gtsam {
  template<>
  struct traits<fgo::factor::GPWNOJInterpolatedTDNCPFactor> :
          public Testable<fgo::factor::GPWNOJInterpolatedTDNCPFactor> {
  };
}
#endif //ONLINE_FGO_GPWNOJINTERTDCPFACTORNORMALCP_H
