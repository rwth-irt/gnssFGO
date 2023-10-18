// /*********************************************************************
//   *
//   * Software License Agreement (BSD License)
//  *
//   *  Copyright (c) 2021
//   *  RWTH Aachen University - Institute of Automatic Control.
//   *  All rights reserved.
//   *
//   *  Redistribution and use in source and binary forms, with or without
//   *  modification, are permitted provided that the following conditions
//   *  are met:
//   *
//   *   * Redistributions of source code must retain the above copyright
//   *     notice, this list of conditions and the following disclaimer.
//   *   * Redistributions in binary form must reproduce the above
//   *     copyright notice, this list of conditions and the following
//   *     disclaimer in the documentation and/or other materials provided
//   *     with the distribution.
//   *   * Neither the name of the institute nor the names of its
//   *     contributors may be used to endorse or promote products derived
//   *     from this software without specific prior written permission.
//   *
//   *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//   *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//   *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   *  POSSIBILITY OF SUCH DAMAGE.
//   *
//   * Author:  Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//              Adapted from SVO
//   *********************************************************************

//
// Created by haoming on 14.11.21.
//

#ifndef FGO_ONLINE_IMU_CALIBRATION_H
#define FGO_ONLINE_IMU_CALIBRATION_H

#include <deque>
#include <iostream>
#include <memory>
#include <Eigen/Core>

namespace fgonav {
    namespace solver {
        class IMUCalibration {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<IMUCalibration> Ptr;

            /// Camera-IMU delay: delay_imu_cam = cam_ts - cam_ts_delay -> ToDo: Only reserve for future work
            double delay_imu_cam = 0.0;

            /// Gyro noise density (sigma). [rad/s*1/sqrt(Hz)]
            double gyro_noise_density = 0.00073088444;

            /// Accelerometer noise density (sigma). [m/s^2*1/sqrt(Hz)]
            double acc_noise_density = 0.01883649;

            /// IMU integrator sigma (sigma). GTSAM preintegration option.
            double imu_integration_sigma = 0.0;

            /// Gyro bias random walk (sigma). [rad/s^2*1/sqrt(Hz)]
            double gyro_bias_random_walk_sigma = 0.00038765;

            /// Accelerometer bias random walk (sigma). [m/s^3*1/sqrt(Hz)]
            double acc_bias_random_walk_sigma = 0.012589254;

            /// Norm of the Gravitational acceleration. [m/s^2]
            double gravity_magnitude = 9.81007;

            /// Coriolis acceleration (earth rotation rate).
            Eigen::Vector3d omega_coriolis = Eigen::Vector3d::Zero();

            /// Accelerometer saturation. [m/s^2]
            double saturation_accel_max = 150;

            /// Gyroscope saturation. [rad/s]
            double saturation_omega_max = 7.8;

            /// Expected IMU Rate [1/s]
            double imu_rate = 100;

            IMUCalibration() = default;

            ~IMUCalibration() = default;

            inline void print(const std::string &s = "IMU Calibration: ") const {
                std::cout << s << std::endl
                          << "  delay_imu_cam = " << delay_imu_cam << std::endl
                          << "  sigma_omega_c = " << gyro_noise_density << std::endl
                          << "  sigma_acc_c = " << acc_noise_density << std::endl
                          << "  sigma_integration_c = " << imu_integration_sigma << std::endl
                          << "  sigma_omega_bias_c = " << gyro_bias_random_walk_sigma << std::endl
                          << "  sigma_acc_bias_c = " << acc_bias_random_walk_sigma << std::endl
                          << "  g = " << gravity_magnitude << std::endl
                          << "  coriolis = " << omega_coriolis.transpose() << std::endl
                          << "  saturation_acc_max = " << saturation_accel_max << std::endl
                          << "  saturation_omega_max = " << saturation_omega_max << std::endl
                          << "  imu_rate = " << imu_rate << std::endl;
            }
        };

        class IMUInitialization {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<IMUInitialization> Ptr;
            /// Initial velocity, in world frame!
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            /// Initial gyroscope bias
            Eigen::Vector3d omega_bias = Eigen::Vector3d::Zero();
            /// Initial accelerometer bias
            Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
            /// Initial velocity uncertainty
            double velocity_sigma = 2.0;
            /// Initial gyroscope bias uncertainty
            double omega_bias_sigma = 0.01;
            /// Initial acceleromter bias uncertainty
            double acc_bias_sigma = 0.1;
            inline void print(const std::string &s = "IMU Initialization: ") const {
                std::cout << s << std::endl
                          << "  velocity = " << velocity.transpose() << std::endl
                          << "  omega_bias = " << omega_bias.transpose() << std::endl
                          << "  acc_bias = " << acc_bias.transpose() << std::endl
                          << "  velocity_sigma = " << velocity_sigma << std::endl
                          << "  omega_bias_sigma = " << omega_bias_sigma << std::endl
                          << "  acc_bias_sigma = " << acc_bias_sigma << std::endl;
            }
        };
    } //namespace solver
} //namespace fgonav






#endif //FGO_ONLINE_IMU_CALIBRATION_H
