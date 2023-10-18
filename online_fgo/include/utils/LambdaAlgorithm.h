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

/*
* Ambiguity Resolution using MLAMBDA
* As described in GPSTk : https://github.com/SGL-UT/GPSTk
* GPSTk version of MLAMBDA implemented using Eigen Library
* Created on: Nov 11, 2018
*      Author: Aaron
*/
// C++ Implementation from
// https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/lambda.c original
// https://github.com/aaronboda24/MLAMBDA-Eigen/tree/master/MLAMBDA transformed in c++

#ifndef ONLINE_FGO_LAMBDAALGORITHM_H
#define ONLINE_FGO_LAMBDAALGORITHM_H

#define LOOPMAX      10000

using namespace std;
using namespace Eigen;

namespace fgo::utils{

    /*Math part--------------------------------------------------------------------------*/
    //signum function
    inline double sign(double x)
    {
        return (x <= 0.0) ? -1.0 : 1.0;
    }

    // Rounding Values
    inline double round(double x)
    {
        return double(std::floor(x + 0.5));
    }

    // Swapping values
    inline void swap(double& a, double& b)
    {
        double t(a); a = b; b = t;
    }
    /*--------------------------------------------------------------------------*/

    /*Ambiguity part--------------------------------------------------------------------------*/
    inline int factorize(const MatrixXd &Q, MatrixXd &L, VectorXd &D) {
        const int n = static_cast<int>(Q.rows());
        MatrixXd QC = MatrixXd::Zero(Q.rows(), Q.cols());
        QC = Q;
        L = MatrixXd::Zero(n, n);
        D = VectorXd::Zero(n);

        for (int i = n - 1; i >= 0; i--) {
            D(i) = QC(i, i);
            if (D(i) <= 0.0) return -1;
            double temp = std::sqrt(D(i));
            for (int j = 0; j <= i; j++) L(i, j) = QC(i, j) / temp;
            for (int j = 0; j <= i - 1; j++) {
                for (int k = 0; k <= j; k++) QC(j, k) -= L(i, k) * L(i, j);
            }
            for (int j = 0; j <= i; j++) L(i, j) /= L(i, i);
        }

        return 0;
    }

    inline void gauss(MatrixXd &L, MatrixXd &Z, int i, int j) {
        const int n = L.rows();
        const int mu = (int) round(L(i, j));
        if (mu != 0) {
            for (int k = i; k < n; k++) L(k, j) -= (double) mu * L(k, i);
            for (int k = 0; k < n; k++) Z(k, j) -= (double) mu * Z(k, i);
        }
    }

    inline void permute(MatrixXd &L, VectorXd &D, int j, double del, MatrixXd &Z) {
        const int n = L.rows();
        double eta = D(j) / del;
        double lam = D(j + 1) * L(j + 1, j) / del;

        // *Changed D[j] to D(j) : [] may refer to row slice

        D(j) = eta * D(j + 1);
        D(j + 1) = del;
        for (int k = 0; k <= j - 1; k++) {
            double a0 = L(j, k);
            double a1 = L(j + 1, k);
            L(j, k) = -L(j + 1, j) * a0 + a1;
            L(j + 1, k) = eta * a0 + lam * a1;
        }
        L(j + 1, j) = lam;
        for (int k = j + 2; k < n; k++) swap(L(k, j), L(k, j + 1));
        for (int k = 0; k < n; k++) swap(Z(k, j), Z(k, j + 1));
    }

    inline void reduction(MatrixXd &L, VectorXd &D, MatrixXd &Z) {
        const int n = L.rows();
        int j(n - 2), k(n - 2);

        while (j >= 0) {
            if (j <= k) {
                for (int i = j + 1; i < n; i++) {
                    gauss(L, Z, i, j);
                }
            }

            double del = D(j) + L(j + 1, j) * L(j + 1, j) * D(j + 1);

            if (del + 1E-6 < D(j + 1)) {
                permute(L, D, j, del, Z);
                k = j;
                j = n - 2;
            } else {
                j--;
            }
        }
    }

    inline int search(MatrixXd &L, VectorXd &D, VectorXd &zs, MatrixXd &zn, VectorXd &s, const int &m) {

        // n - number of float parameters
        // m - number of fixed solutions
        // L - nxn
        // D - nx1
        // zs - nxn
        // zn - nxm
        // s  - m

        const int n = L.rows();

        zn = MatrixXd::Zero(n, m);
        s = VectorXd::Zero(m);

        MatrixXd S = MatrixXd::Zero(n, n);
        VectorXd dist = VectorXd::Zero(n);
        VectorXd zb = VectorXd::Zero(n);
        VectorXd z = VectorXd::Zero(n);
        VectorXd step = VectorXd::Zero(n);

        int k = n - 1;
        dist[k] = 0.0;
        zb(k) = zs(k);
        z(k) = round(zb(k));
        double y = zb(k) - z(k);
        step(k) = sign(y);

        int c(0), nn(0), imax(0);
        double maxdist = 1E99;
        for (int c = 0; c < LOOPMAX; c++) {
            double newdist = dist(k) + y * y / D(k);
            if (newdist < maxdist) {
                if (k != 0) {
                    dist(--k) = newdist;
                    for (int i = 0; i <= k; i++) {
                        S(k, i) = S(k + 1, i) + (z(k + 1) - zb(k + 1)) * L(k + 1, i);
                    }
                    zb(k) = zs(k) + S(k, k);
                    z(k) = round(zb(k));
                    y = zb(k) - z(k);
                    step(k) = sign(y);
                } else {
                    if (nn < m) {
                        if (nn == 0 || newdist > s(imax)) imax = nn;
                        for (int i = 0; i < n; i++) zn(i, nn) = z(i);
                        s(nn++) = newdist;
                    } else {
                        if (newdist < s(imax)) {
                            for (int i = 0; i < n; i++) zn(i, imax) = z(i);
                            s(imax) = newdist;
                            for (int i = imax = 0; i < m; i++) if (s(imax) < s(i)) imax = i;
                        }
                        maxdist = s(imax);
                    }
                    z(0) += step(0);
                    y = zb(0) - z(0);
                    step(0) = -step(0) - sign(step(0));
                }
            } else {
                if (k == n - 1) break;
                else {
                    k++;
                    z(k) += step(k);
                    y = zb(k) - z(k);
                    step(k) = -step(k) - sign(step(k));
                }
            }
        }
        for (int i = 0; i < m - 1; i++) {
            for (int j = i + 1; j < m; j++) {
                if (s(i) < s(j)) continue;
                swap(s(i), s(j));
                for (k = 0; k < n; k++) swap(zn(k, i), zn(k, j));
            }
        }

        if (c >= LOOPMAX) {
            return -1;
        }

        return 0;
    }

    /*Main function
     * a    I   float solutions
     * Q    I   Covariance Matrix float solutions
     * F    O   integer solutions
     * s    O   residuals of solutions
     * m    I   how many solutions do you want to generate and compare?
     * */

    inline int lambda(const VectorXd &a, const MatrixXd &Q, MatrixXd &F, VectorXd &s, const int &m) {
        if ((a.size() != Q.rows()) || (Q.rows() != Q.cols())) return -1;
        if (m < 1) return -1;

        const int n = static_cast<int>(a.size());

        MatrixXd L = MatrixXd::Zero(n, n);
        MatrixXd E = MatrixXd::Zero(n, m);

        VectorXd D = VectorXd::Zero(n);
        VectorXd z = VectorXd::Zero(n);
        MatrixXd Z = MatrixXd::Identity(n, n);

        if (factorize(Q, L, D) == 0) {
            reduction(L, D, Z);
            z = Z.transpose() * a;

            if (search(L, D, z, E, s, m) == 0) {
                try {
                    // F=Z'\E - Z nxn  E nxm F nxm
                    F = (Z.transpose().inverse()) * E;
                }
                catch (...) {
                    return -1;
                }
            }
        }

        return 0;
    }
    /*--------------------------------------------------------------------------*/
}

#endif //ONLINE_FGO_LAMBDAALGORITHM_H
