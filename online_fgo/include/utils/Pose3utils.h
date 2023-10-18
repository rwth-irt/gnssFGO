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

#ifndef POSE3UTILS_H
#define POSE3UTILS_H

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/function.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace fgo::utils {

// fixed size matrix for Dim-6 operation
typedef Eigen::Matrix<double, 12, 1> Vector_12;
typedef Eigen::Matrix<double, 12, 12> Matrix_12;
typedef Eigen::Matrix<double, 6, 12> Matrix_6_12;
typedef Eigen::Matrix<double, 12, 6> Matrix_12_6;
typedef Eigen::Matrix<double, 3, 12> Matrix_3_12;
typedef Eigen::Matrix<double, 12, 3> Matrix_12_3;

///WNOJ addition
typedef Eigen::Matrix<double, 18, 1> Vector_18;
typedef Eigen::Matrix<double, 18, 18> Matrix_18;
//typedef Eigen::Matrix<double, 6, 18> Matrix_6_18;
typedef Eigen::Matrix<double, 18, 6> Matrix_18_6;
//typedef Eigen::Matrix<double, 3, 18> Matrix_3_18;
//typedef Eigen::Matrix<double, 18, 3> Matrix_18_3;

inline geometry_msgs::msg::Vector3 convertGTVec3ToROS(const gtsam::Vector3& gtVec)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = gtVec.x();
  msg.y = gtVec.y();
  msg.z = gtVec.z();
  return msg;
}

/// get body-centric/body-frame velocity from two poses and delta_t
gtsam::Vector6 getBodyCentricVs(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, double delta_t);
gtsam::Vector6 getBodyCentricVb(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, double delta_t);

/// convert 6d body-frame velocity into 3+3 world-frame line and angular velocity
/// with optional jacobians
void convertVbToVW(const gtsam::Vector6& v6, const gtsam::Pose3& pose,
                   gtsam::Vector3& v, gtsam::Vector3& w,
                   gtsam::OptionalJacobian<3, 6> Hv = boost::none, gtsam::OptionalJacobian<3, 6> Hw = boost::none);

gtsam::Vector6 convertVWToVb(const gtsam::Vector3& v, const gtsam::Vector3& w,
                             const gtsam::Pose3& pose, gtsam::OptionalJacobian<6, 3> Hv = boost::none,
                             gtsam::OptionalJacobian<6, 3> Hw = boost::none,
                             gtsam::OptionalJacobian<6, 6> Hpose = boost::none);

gtsam::Vector6 convertVwWbToVbWb(const gtsam::Vector3& vn, const gtsam::Vector3& wb, const gtsam::Pose3& pose,
                      gtsam::OptionalJacobian<6, 3> Hvn = boost::none,
                      gtsam::OptionalJacobian<6, 3> Hwb = boost::none,
                      gtsam::OptionalJacobian<6, 6> Hpose= boost::none);

gtsam::Vector6 convertGbAbToGbAw(const gtsam::Vector3 &gb,
                          const gtsam::Vector3 &ab,
                          const gtsam::Pose3 &pose,
                          gtsam::OptionalJacobian<6, 6> Hpose = boost::none);

/// left Jacobian for Pose3 Expmap
gtsam::Matrix6 leftJacobianPose3(const gtsam::Vector6& xi);
gtsam::Matrix6 leftJacobianPose3inv(const gtsam::Vector6& xi);
gtsam::Matrix3 leftJacobianPose3Q(const gtsam::Vector6& xi);

/// right Jacobian for Pose3: jacobian of expmap/logmap
gtsam::Matrix6 rightJacobianPose3(const gtsam::Vector6& xi);
gtsam::Matrix6 rightJacobianPose3inv(const gtsam::Vector6& xi);
gtsam::Matrix3 rightJacobianPose3Q(const gtsam::Vector6& xi);

/// numerical diff of jacobian matrix methods
/// output d(A(xi)*x)/dxi jacobian matrix
gtsam::Matrix6 jacobianMethodNumercialDiff(boost::function<gtsam::Matrix6(
    const gtsam::Vector6&)> func, const gtsam::Vector6& xi,
    const gtsam::Vector6& x, double dxi = 1e-6);

/// left Jacobian for Rot3 Expmap
gtsam::Matrix3 leftJacobianRot3(const gtsam::Vector3& omega);
gtsam::Matrix3 leftJacobianRot3inv(const gtsam::Vector3& omega);

/// right Jacobian for Rot3: jacobian of expmap/logmap
gtsam::Matrix3 rightJacobianRot3(const gtsam::Vector3& omega);
gtsam::Matrix3 rightJacobianRot3inv(const gtsam::Vector3& omega);

} // namespace gtsam

#endif
