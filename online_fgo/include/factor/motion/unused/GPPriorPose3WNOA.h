//
// Created by xia on 13.04.21.
//

#ifndef FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H
#define FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H

#endif //FGONAV_GAUSSIANPROCESSPRIORPOSE3VNWB_H

/**
 *  @file  GaussianProcessPriorPose3WNOA.h
 *  @brief Pose3 GP prior, use Vn/Wb velocity representation
 *  (Vn:translation velocity in navigation frame, Wb: angular velocity in body frame.)
 *  @author Xiao Xia
 **/

#pragma once

#include "utils/GPutils.h"

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>

#include <boost/lexical_cast.hpp>

#include <ostream>


namespace fgo {
  namespace factors {

/**
 * 4-way factor for Gaussian Process prior factor, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 */
    class GaussianProcessPriorPose3WNOA : public gtsam::NoiseModelFactor4<gtsam::Pose3,
            gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> {

    private:
      typedef GaussianProcessPriorPose3WNOA This;
      typedef gtsam::NoiseModelFactor4<gtsam::Pose3,
              gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> Base;

      double delta_t_;
      gtsam::Vector3 omega1_;
      gtsam::Vector3 omega2_;

    public:
      GaussianProcessPriorPose3WNOA() {}    /* Default constructor only for serialization */

      /// Constructor
      /// @param delta_t is the time between the two states
      GaussianProcessPriorPose3WNOA(
              gtsam::Key poseKey1, gtsam::Key velKey1,
              gtsam::Key poseKey2, gtsam::Key velKey2,
              const gtsam::Vector3& omega1, const gtsam::Vector3& omega2,
              double delta_t, const gtsam::SharedNoiseModel &Qc_model) :
              Base(gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(Qc_model), delta_t).block<9,9>(0,0)),
                   poseKey1, velKey1, poseKey2, velKey2),
              omega1_(omega1), omega2_(omega2) {
        delta_t_ = delta_t;
      }

      virtual ~GaussianProcessPriorPose3WNOA() {}


      /// @return a deep copy of this factor
      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      /// factor error function
      gtsam::Vector evaluateError(
              const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1_n,
              const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2_n,
              boost::optional<gtsam::Matrix &> H1 = boost::none,
              boost::optional<gtsam::Matrix &> H2 = boost::none,
              boost::optional<gtsam::Matrix &> H3 = boost::none,
              boost::optional<gtsam::Matrix &> H4 = boost::none) const override {

        using namespace gtsam;

        // jacobians
        if (H1) {
          gtsam::Matrix66 correction = (gtsam::Matrix66() <<
                                                          gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
          *H1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, boost::placeholders::_1, vel1_n,pose2, vel2_n), pose1, 1e-5) * correction;
        }
        if (H2) {
          *H2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, boost::placeholders::_1,pose2, vel2_n), vel1_n, 1e-5);
        }
        if (H3) {
          gtsam::Matrix66 correction = (gtsam::Matrix66() <<
                                                          gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
          *H3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1_n,boost::placeholders::_1, vel2_n), pose2, 1e-5) * correction;
        }
        if (H4) {
          *H4 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
                  boost::bind(&This::evaluateError_, this, pose1, vel1_n,pose2, boost::placeholders::_1), vel2_n, 1e-5);
        }
        return evaluateError_(pose1, vel1_n, pose2, vel2_n);
      }

      gtsam::Vector evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1_n,
                                   const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2_n) const {

        gtsam::Vector6 vel1 = (gtsam::Vector6() << omega1_,vel1_n).finished();
        gtsam::Vector6 vel2 = (gtsam::Vector6() << omega2_,vel2_n).finished();

        gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(pose1.rotation().inverse() * pose2.rotation(),
                                                             - pose1.translation() + pose2.translation()));
        return  (gtsam::Vector9() << (r - vel1 * delta_t_),
                (pose2.rotation().transpose() * vel2.block<3,1>(3,0) -
                 pose1.rotation().transpose() * vel1.block<3,1>(3,0))).finished();
      }

      /** number of variables attached to this factor */
      size_t size() const {
        return 4;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "4-way Gaussian Process Factor Pose3 VW" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      //friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar & BOOST_SERIALIZATION_NVP(delta_t_);
      }

    }; // GaussianProcessPriorPose3

  } //namespace factor
} // namespace fgo::utils



/// traits
namespace gtsam {
    template<>
    struct traits<fgo::factors::GaussianProcessPriorPose3WNOA> : public Testable<fgo::factors::GaussianProcessPriorPose3WNOA> {};
}

