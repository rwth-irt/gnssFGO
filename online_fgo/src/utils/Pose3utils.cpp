#include <cmath>
#include "utils/Pose3utils.h"


using namespace gtsam;

namespace fgo::utils  {
    /* ************************************************************************** */
    // see Barfoot14tro eq. (25)
    Vector6 getBodyCentricVb(const Pose3& pose1, const Pose3& pose2, double delta_t) {
      return Pose3::Logmap(pose1.inverse().compose(pose2)) / delta_t;
    }

    /* ************************************************************************** */
    Vector6 getBodyCentricVs(const Pose3& pose1, const Pose3& pose2, double delta_t) {
      return Pose3::Logmap(pose2.compose(pose1.inverse())) / delta_t;
    }

    /* ************************************************************************** */
    void convertVbToVW(const Vector6 &v6, const Pose3 &pose, Vector3 &v, Vector3 &w,
                       OptionalJacobian<3, 6> Hv, OptionalJacobian<3, 6> Hw) {

      Matrix3 Hrv;
      if (Hv) {
        v = pose.rotation().rotate(v6.tail<3>(), boost::none, Hrv);
        *Hv = (Matrix36() << Matrix3::Zero(), Hrv).finished();
      } else {
        v = pose.rotation().rotate(v6.tail<3>());
      }

      if (Hw)  {
        w = pose.rotation().rotate(v6.head<3>(), boost::none, Hrv);
        *Hw = (Matrix36() << Hrv, Matrix3::Zero()).finished();
      } else {
        w = pose.rotation().rotate(v6.head<3>());
      }
    }

    /* ************************************************************************** */

    Vector6 convertVwWbToVbWb(const gtsam::Vector3& vn, const gtsam::Vector3& wb, const gtsam::Pose3& pose,
                           gtsam::OptionalJacobian<6, 3> Hvn,
                           gtsam::OptionalJacobian<6, 3> Hwb,
                           gtsam::OptionalJacobian<6, 6> Hpose)
    {
      gtsam::Vector6 v6;
      if (Hvn || Hwb || Hpose) {
        gtsam::Matrix3 Hrv, Hpv;
        //Hpw << 0.0, -w.z(),w.y(),w.z(),0.0,-w.x(),-w.y(),w.x(),0.0;

        v6 = (gtsam::Vector6() << wb,
            pose.rotation().unrotate(vn, &Hpv, &Hrv)).finished();
        if (Hvn) *Hvn = (gtsam::Matrix63() << gtsam::Z_3x3, Hrv).finished();
        if (Hwb) *Hwb = (gtsam::Matrix63() << gtsam::I_3x3, gtsam::Z_3x3).finished();
        if (Hpose) *Hpose = (gtsam::Matrix66() << gtsam::Z_3x3, gtsam::Z_3x3, Hpv, gtsam::Z_3x3).finished();
      } else {
        v6 = (gtsam::Vector6() << wb,
            pose.rotation().unrotate(vn)).finished();
      }
      return v6;
    }

    /* ************************************************************************** */

    Vector6 convertGbAbToGbAw(const gtsam::Vector3 &gb,
                              const gtsam::Vector3 &ab,
                              const gtsam::Pose3 &pose,
                              gtsam::OptionalJacobian<6, 6> Hpose)
    {
      /*
        inputs ::
        gb --> body angular acceleration [meter/second]
        ab ---> body linear acceleration [meter/second]
        pose ---> ECEF pose
        output ::
        v6 --> body angular and linear velocity  [rad and m/s]
      */
      gtsam::Vector6 a6;
      if (Hpose){
        gtsam::Matrix3 Hpv;
        a6 = (gtsam::Vector6() << gb,pose.rotation().rotate(ab,Hpv)).finished();
        if (Hpose)
          *Hpose = (gtsam::Matrix66() << gtsam::Z_3x3, gtsam::Z_3x3, gtsam::Z_3x3, Hpv).finished();
      } else {
        a6 = (gtsam::Vector6() << gb, pose.rotation().rotate(ab)).finished();
      }
      return a6;
    }

    /* ************************************************************************** */
    Vector6 convertVWToVb(const Vector3 &v, const Vector3 &w, const Pose3 &pose,
                          OptionalJacobian<6, 3> Hv, OptionalJacobian<6, 3> Hw,
                          OptionalJacobian<6, 6> Hpose) {

      Vector6 v6;
      if (Hv || Hw || Hpose) {
        Matrix3 Hrv, Hrw, Hpv, Hpw;
        v6 = (Vector6() << pose.rotation().unrotate(w, Hpw, Hrw),
            pose.rotation().unrotate(v, Hpv, Hrv)).finished();
        if (Hv) *Hv = (Matrix63() << Matrix3::Zero(), Hrv).finished();
        if (Hw) *Hw = (Matrix63() << Hrw, Matrix3::Zero()).finished();
        if (Hpose) *Hpose = (Matrix6() << Hpw, Matrix3::Zero(), Hpv, Matrix3::Zero()).finished();
      } else {
        v6 = (Vector6() << pose.rotation().unrotate(w),
            pose.rotation().unrotate(v)).finished();
      }
      return v6;
    }

    /* ************************************************************************** */
    // see Barfoot14tro eq. (102)
    Matrix3 leftJacobianPose3Q(const Vector6& xi) {
      const Vector3 omega = xi.head(3), rho = xi.tail(3);
      const double theta = omega.norm();        // rotation angle
      const Matrix3 X = skewSymmetric(omega), Y = skewSymmetric(rho);

      const Matrix3 XY = X*Y, YX = Y*X, XYX = X * YX;
      if (fabs(theta) > 1e-5) {
        const double sin_theta = sin(theta), cos_theta = cos(theta);
        const double theta2 = theta * theta, theta3 = theta2 * theta,
            theta4 = theta3 * theta, theta5 = theta4 * theta;

        return 0.5*Y + (theta - sin_theta)/theta3 * (XY + YX + XYX)
            - (1.0 - 0.5*theta2 - cos_theta)/theta4 * (X*XY + YX*X - 3.0*XYX)
            - 0.5*((1.0 - 0.5*theta2 - cos_theta)/theta4 - 3.0*(theta - sin_theta - theta3/6.0)/theta5)
              * (XYX*X + X*XYX);

      } else {
        return 0.5*Y + 1.0/6.0*(XY + YX + XYX)
            - 1.0/24.0*(X*XY + YX*X - 3.0*XYX)
            -0.5*(1.0/24.0 + 3.0/120.0) * (XYX*X + X*XYX);
      }
    }

    /* ************************************************************************** */
    Matrix3 rightJacobianPose3Q(const Vector6& xi) {
      const Vector3 omega = xi.head(3), rho = xi.tail(3);
      const double theta = omega.norm();        // rotation angle
      const Matrix3 X = skewSymmetric(omega), Y = skewSymmetric(rho);

      const Matrix3 XY = X*Y, YX = Y*X, XYX = X * YX;
      if (fabs(theta) > 1e-5) {
        const double sin_theta = sin(theta), cos_theta = cos(theta);
        const double theta2 = theta * theta, theta3 = theta2 * theta,
            theta4 = theta3 * theta, theta5 = theta4 * theta;

        return -0.5*Y + (theta - sin_theta)/theta3 * (XY + YX - XYX)
            + (1.0 - 0.5*theta2 - cos_theta)/theta4 * (X*XY + YX*X - 3.0*XYX)
            - 0.5*((1.0 - 0.5*theta2 - cos_theta)/theta4 - 3.0*(theta - sin_theta - theta3/6.0)/theta5)
              * (XYX*X + X*XYX);

      } else {
        return -0.5*Y + 1.0/6.0*(XY + YX - XYX)
            + 1.0/24.0*(X*XY + YX*X - 3.0*XYX)
            -0.5*(1.0/24.0 + 3.0/120.0) * (XYX*X + X*XYX);
      }
    }

    /* ************************************************************************** */
    // see Barfoot14tro eq. (100), note that in GTSAM rotation first in pose logmap
    Matrix6 leftJacobianPose3(const Vector6& xi) {
      const Vector3 omega = xi.head(3);
      const Matrix3 Q = leftJacobianPose3Q(xi);
      const Matrix3 J = leftJacobianRot3(omega);

      return (Matrix6() << J, Matrix::Zero(3,3), Q, J).finished();
    }

    /* ************************************************************************** */
    // see Barfoot14tro eq. (103)
    Matrix6 leftJacobianPose3inv(const Vector6& xi) {
      const Vector3 omega = xi.head(3);
      const Matrix3 Q = leftJacobianPose3Q(xi);
      const Matrix3 Jinv = leftJacobianRot3inv(omega);
      //  J << Jw, Matrix::Zero(3,3), Q2, Jw;
      return (Matrix6() << Jinv, Matrix::Zero(3,3), -Jinv*Q*Jinv, Jinv).finished();
    }

    /* ************************************************************************** */
    // see Barfoot14tro eq. (98)
    Matrix3 leftJacobianRot3(const Vector3& omega) {
      double theta2 = omega.dot(omega);
      if (theta2 <= std::numeric_limits<double>::epsilon()) return Matrix::Identity(3,3);
      const double theta = std::sqrt(theta2);     // rotation angle
      const Vector3 dir = omega / theta;          // direction

      const double sin_theta = sin(theta);
      const Matrix3 A = skewSymmetric(omega) / theta;

      return sin_theta / theta * Matrix::Identity(3,3) + (1 - sin_theta / theta) *
          (dir * dir.transpose()) + (1 - cos(theta))/(theta) * A;
    }

    /* ************************************************************************** */
    // see Barfoot14tro eq. (99)
    Matrix3 leftJacobianRot3inv(const Vector3& omega) {
      double theta2 = omega.dot(omega);
      if (theta2 <= std::numeric_limits<double>::epsilon()) return Matrix::Identity(3,3);
      const double theta = std::sqrt(theta2);     // rotation angle
      const Vector3 dir = omega / theta;          // direction

      const double theta_2 = theta/2.0;
      const double cot_theta_2 = 1.0 / tan(theta_2);
      const Matrix3 A = skewSymmetric(omega) / theta;

      return theta_2*cot_theta_2 * Matrix::Identity(3,3) + (1 - theta_2*cot_theta_2) *
          (dir * dir.transpose()) - theta_2 * A;
    }

    /* ************************************************************************** */
    Matrix6 jacobianMethodNumercialDiff(boost::function<Matrix6(const Vector6&)> func,
                                        const Vector6& xi, const Vector6& x, double dxi) {
      Matrix6 Diff = Matrix6();
      for (long i = 0; i < 6; i++) {
        Vector6 xi_dxip = xi, xi_dxin = xi;
        xi_dxip(i) += dxi;
        Matrix6 Jdiffp = func(xi_dxip);
        xi_dxin(i) -= dxi;
        Matrix6 Jdiffn = func(xi_dxin);
        Diff.block<6, 1>(0, i) = (Jdiffp - Jdiffn) / (2.0 * dxi) * x;
      }
      return Diff;
    }

    /* ************************************************************************** */
    Matrix6 rightJacobianPose3(const Vector6& xi) {
      const Vector3 w = xi.head<3>();
      const Matrix3 Jw = rightJacobianRot3(w);
      const Matrix3 Q = rightJacobianPose3Q(xi);
      Matrix6 J;
      J << Jw, Matrix::Zero(3,3), Q, Jw;
      return J;
    }

    /* ************************************************************************** */
    Matrix6 rightJacobianPose3inv(const Vector6& xi) {
      const Vector3 w = xi.head<3>();
      const Matrix3 Jw = rightJacobianRot3inv(w);
      const Matrix3 Q = rightJacobianPose3Q(xi);
      const Matrix3 Q2 = -Jw*Q*Jw;
      Matrix6 J;
      J << Jw, Matrix::Zero(3,3), Q2, Jw;
      return J;
    }

    /* ************************************************************************** */
    Matrix3 rightJacobianRot3(const Vector3& omega) {
      using std::cos;
      using std::sin;
      double theta2 = omega.dot(omega);
      if (theta2 <= std::numeric_limits<double>::epsilon()) return Matrix::Identity(3,3);
      double theta = std::sqrt(theta2);  // rotation angle
      const Matrix3 Y = skewSymmetric(omega) / theta;
      return Matrix::Identity(3,3) - ((1 - cos(theta)) / (theta)) * Y
          + (1 - sin(theta) / theta) * Y * Y; // right Jacobian
    }

    /* ************************************************************************** */
    Matrix3 rightJacobianRot3inv(const Vector3& omega) {
      using std::cos;
      using std::sin;
      double theta2 = omega.dot(omega);
      if (theta2 <= std::numeric_limits<double>::epsilon()) return Matrix::Identity(3,3);
      double theta = std::sqrt(theta2);  // rotation angle
      const Matrix3 X = skewSymmetric(omega); // element of Lie algebra so(3): X = omega^
      return Matrix::Identity(3,3) + 0.5 * X
          + (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) * X*X;
    }

}

