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

#ifndef ONLINE_FGO_GPUTILS_H
#define ONLINE_FGO_GPUTILS_H

#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <cmath>

namespace fgo::utils {
/// get Qc covariance matrix from noise model
    inline gtsam::Matrix getQc(const gtsam::SharedNoiseModel& Qc_model){
      auto *Gassian_model = dynamic_cast<gtsam::noiseModel::Gaussian*>(Qc_model.get());
      return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();  // => R().transpose() * R() = inv(sigma)
    }

/// calculate Q
    template <int Dim>
    Eigen::Matrix<double, 2*Dim, 2*Dim> calcQ(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
        Eigen::Matrix<double, 2*Dim, 2*Dim> Q = (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
                                                                                       1.0 / 3 * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc,
                                                                                       1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc).finished();
        return Q;
    }

/// calculate Q_inv
    template <int Dim>
    Eigen::Matrix<double, 2*Dim, 2*Dim> calcQ_inv(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
        Eigen::Matrix<double, Dim, Dim> Qc_inv = Qc.inverse();
        Eigen::Matrix<double, 2*Dim, 2*Dim> Q_inv =
                (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
                                                       12.0 * pow(tau, -3.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
                        (-6.0) * pow(tau, -2.0) * Qc_inv, 4.0 * pow(tau, -1.0) * Qc_inv).finished();
        return Q_inv;
    }

/// calculate Phi
    template <int Dim>
    Eigen::Matrix<double, 2*Dim, 2*Dim> calcPhi(double tau) {

        Eigen::Matrix<double, 2*Dim, 2*Dim> Phi = (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
        Eigen::Matrix<double, Dim, Dim>::Identity(), tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
        Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity()).finished();
        return Phi;
    }

/// calculate Lambda
    template <int Dim>
    Eigen::Matrix<double, 2*Dim, 2*Dim> calcLambda(const Eigen::Matrix<double, Dim, Dim>& Qc,
                                                   double delta_t, const double tau) {

        Eigen::Matrix<double, 2*Dim, 2*Dim> Lambda = calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
                                                                         * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);
        return Lambda;
    }

/// calculate Psi
    template <int Dim>
    Eigen::Matrix<double, 2*Dim, 2*Dim> calcPsi(const Eigen::Matrix<double, Dim, Dim>& Qc,
                                                double delta_t, double tau) {

        Eigen::Matrix<double, 2*Dim, 2*Dim> Psi = calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
                                                  * calcQ_inv(Qc, delta_t);
        return Psi;
    }

    ///WNOJ addition
    /// calculate Q
    template <int Dim>
    Eigen::Matrix<double, 3*Dim, 3*Dim> calcQ3(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
        Eigen::Matrix<double, 3*Dim, 3*Dim> Q = (Eigen::Matrix<double, 3*Dim, 3*Dim>() <<
                1.0 / 20 * pow(tau, 5.0) * Qc, 1.0 / 8 * pow(tau, 4.0) * Qc, 1.0 / 6 * pow(tau, 3.0) * Qc,
                1.0 / 8  * pow(tau, 4.0) * Qc, 1.0 / 3 * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc,
                1.0 / 6  * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc).finished();
        return Q;
    }

  /// calculate Q
  template <int Dim>
  Eigen::Matrix<double, 2*Dim, 2*Dim> calcQ3_12x12(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
    Eigen::Matrix<double, 2*Dim, 2*Dim> Q = (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
            1.0 / 20 * pow(tau, 5.0) * Qc, 1.0 / 8 * pow(tau, 4.0) * Qc,
            1.0 / 8  * pow(tau, 4.0) * Qc, 1.0 / 3 * pow(tau, 3.0) * Qc).finished();
    return Q;
  }

    /// calculate Q_inv
    template <int Dim>
    Eigen::Matrix<double, 3*Dim, 3*Dim> calcQ_inv3(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
        Eigen::Matrix<double, Dim, Dim> Qc_inv = Qc.inverse();
        Eigen::Matrix<double, 3*Dim, 3*Dim> Q_inv = (Eigen::Matrix<double, 3*Dim, 3*Dim>() <<
                720.0 * pow(tau, -5.0) * Qc_inv, (-360.0) * pow(tau, -4.0) * Qc_inv, 60.0 * pow(tau, -3.0) * Qc_inv,
                (-360.0) * pow(tau, -4.0) * Qc_inv, 192 * pow(tau, -3.0) * Qc_inv, (-36.0) * pow(tau, -2.0) * Qc_inv,
                60.0 * pow(tau, -3.0) * Qc_inv, (-36.0) * pow(tau, -2.0) * Qc_inv, 9.0 * pow(tau, -1.0) * Qc_inv).finished();
        return Q_inv;
    }
    /// calculate Phi
    template <int Dim>
    Eigen::Matrix<double, 3*Dim, 3*Dim> calcPhi3(double tau) {

        Eigen::Matrix<double, 3*Dim, 3*Dim> Phi = (Eigen::Matrix<double, 3*Dim, 3*Dim>() <<
        Eigen::Matrix<double, Dim, Dim>::Identity(), tau * Eigen::Matrix<double, Dim, Dim>::Identity(), 0.5 * pow(tau,2.0) * Eigen::Matrix<double, Dim, Dim>::Identity(),
        Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity(), tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
        Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity()).finished();
        return Phi;
    }
    /// calculate Lambda
    template <int Dim>
    Eigen::Matrix<double, 3*Dim, 3*Dim> calcLambda3(const Eigen::Matrix<double, Dim, Dim>& Qc,
                                                   double delta_t, const double tau) {

       Eigen::Matrix<double, 3*Dim, 3*Dim> Lambda = calcPhi3<Dim>(tau) - calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t - tau).transpose())
                                                                         * calcQ_inv3(Qc, delta_t) * calcPhi3<Dim>(delta_t);

      ///Eigen::Matrix<double, 3*Dim, 3*Dim> Lambda = calcPhi3<Dim>(tau) - calcPsi3(Qc, delta_t, tau) * (calcPhi3<Dim>(delta_t));
      // Eigen::Matrix<double, 2*Dim, 2*Dim> Lambda = calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
      //                                                                         * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);

        return Lambda;
    }
/// calculate Psi
    template <int Dim>
    Eigen::Matrix<double, 3*Dim, 3*Dim> calcPsi3(const Eigen::Matrix<double, Dim, Dim>& Qc,
                                                double delta_t, double tau) {

        Eigen::Matrix<double, 3*Dim, 3*Dim> Psi = calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t - tau).transpose())
                                                  * calcQ_inv3(Qc, delta_t);

        //Eigen::Matrix<double, 3*Dim, 3*Dim> Psi = calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t).transpose())
        //                                        * calcQ_inv3(Qc, tau);
        return Psi;
    }
    ///WNOJ end

}

#endif //ONLINE_FGO_GPUTILS_H
