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

#ifndef GNSSTOOLS_H
#define GNSSTOOLS_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/VectorSpace.h>
#include <boost/serialization/nvp.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include "utils/Constants.h"

#define P_T                50  /* T */
#define P_A                30  /* A */
#define P_a                30  /* a */
#define P_F                10  /* F */

namespace fgo::utils {
    using namespace constants;

    inline rclcpp::Time roundTime(const rclcpp::Time& rosTime, size_t precisionAfterSecond = 2, double offsetSecond = 0.)
    {
      auto precision = std::pow(10., precisionAfterSecond);
      auto thisTime = rosTime.seconds();
      thisTime *= precision;
      thisTime += (offsetSecond * precision);
      auto rounded_down = floor(thisTime);
      auto thisTimeRounded = static_cast<int64_t>(rounded_down) ;
      thisTimeRounded = thisTimeRounded * (1000000000 / int64_t(precision));
      return rclcpp::Time(thisTimeRounded, RCL_ROS_TIME);
    }

    //// Convert WGS-84 ECEF coordinates to LLH
    ////
    //// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
    ////        Navigation Systems. Artech House, 2008

    inline gtsam::Point3 xyz2llh(const gtsam::Point3 &p1)
    {
        /*
        inputs ::
        p1 --> ECEF xyz receiver coordinates [meter]
        output ::
        posLLH --> latitude, longitude, height [rad,rad,meter]
        */

        double x2 = pow(p1.x(), 2);
        double y2 = pow(p1.y(), 2);
        double z2 = pow(p1.z(), 2);

        double e = sqrt(1.0 - ((semiMinor / semiMajor) * (semiMinor / semiMajor)));
        double b2 = pow(semiMinor, 2);
        double e2 = pow(e, 2);
        double ep = e * (semiMajor / semiMinor);
        double r = sqrt(x2 + y2);
        double r2 = pow(r, 2);
        double E2 = pow(semiMajor, 2) - pow(semiMinor, 2);
        double F = 54 * b2 * z2;
        double G = r2 + (1 - e2) * z2 - e2 * E2;
        double c = (pow(e2, 2) * F * r2) / (pow(G, 3));
        double s = pow((1.0 + c + sqrt(c * c + 2 * c)), 1.0 / 3.0);

        double P = F / (3.0 * (s + 1 / s + 1) * (s + 1 / s + 1) * G * G);
        double Q = sqrt(1.0 + 2 * e2 * e2 * P);
        double ro = -(P * e2 * r) / (1 + Q) +
                    sqrt((semiMajor * semiMajor / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) -
                         P * r2 / 2);
        double tmp = pow((r - e2 * ro), 2);
        double U = sqrt(tmp + z2);
        double V = sqrt(tmp + (1 - e2) * z2);
        double zo = (b2 * p1.z()) / (semiMajor * V);

        double height = U * (1.0 - b2 / (semiMajor * V));
        double lat = std::atan((p1.z() + ep * ep * zo) / r);
        double temp = std::atan(p1.y() / p1.x());
        double longitude;
        if (p1.x() >= 0.0) {
            longitude = temp;
        } else if ((p1.x() < 0.0) && (p1.y() >= 0.0)) {
            longitude = temp + M_PI;
        } else {
            longitude = temp - M_PI;

        }
        gtsam::Point3 llh(lat, longitude, height);
        return llh;
    }

    inline gtsam::Point3 llh2xyz(const gtsam::Point3 &llh)
    {
        /*
        inputs ::
        llh --> latitude, longitude, height [rad,rad,meter]
        output ::
        pos_ecef --> ECEF xyz receiver coordinates [meter]
        */
        double b = llh.x();
        double l = llh.y();
        double h = llh.z();
        auto a = semiMajor;
        double f = 1 - semiMinor / semiMajor;
        double ex2 = (2 - f) * f / (pow((1 - f), 2));
        double c = a * sqrt(1 + ex2);
        double N = c / sqrt(1 + ex2 * pow(cos(b), 2));
        double x = (N + h) * cos(b) * cos(l);
        double y = (N + h) * cos(b) * sin(l);
        double z = (pow((1 - f), 2) * N + h) * sin(b);
        gtsam::Point3 pos_ecef(x, y, z);
        return pos_ecef;
      }

    inline gtsam::Matrix33 nedRe_Matrix_asLLH(const gtsam::Point3 &llh) {
      double sLat = sin(llh(0));
      double sLon = sin(llh(1));
      double cLat = cos(llh(0));
      double cLon = cos(llh(1));

      gtsam::Matrix R = (gtsam::Matrix33() << -sLat * cLon, -sLat * sLon, cLat,
          -sLon, cLon, 0.0,
          -cLat * cLon, -cLat * sLon, -sLat).finished();
      return R;
    }

    //// Generate rotation matrix from Earth-to-Navigation frame
    // n navigation frame refers to ned frame
    inline gtsam::Matrix33 nedRe_Matrix(const gtsam::Point3 &ECEFxyz)
    {
        gtsam::Vector3 llh = xyz2llh(ECEFxyz);
        return nedRe_Matrix_asLLH(llh);
    }

    inline gtsam::Matrix33 enuRe_Matrix_asLLH(const gtsam::Point3 &llh)
    {
      double sinPhi = sin(llh(0));
      double cosPhi = cos(llh(0));
      double sinLam = sin(llh(1));
      double cosLam = cos(llh(1));
      gtsam::Matrix R = (gtsam::Matrix33() << -1 * sinLam, cosLam, 0,
          -1 * sinPhi * cosLam, -1 * sinPhi * sinLam, cosPhi,
          cosPhi * cosLam, cosPhi * sinLam, sinPhi).finished();
      return R;
    }

    inline gtsam::Matrix33 enuRe_Matrix(const gtsam::Point3 &base)
    {
      /*
         inputs ::
         base --> ECEF xyz origin coordinates [meter]
         outputs ::
          R ---> RotationMatrix xyz 2 enu [meters]
       */
      gtsam::Vector3 orgLLH = xyz2llh(base);
      return enuRe_Matrix_asLLH(orgLLH);
    }

    inline gtsam::Matrix33 nedRenu_llh(const gtsam::Point3& llh)
    {
      const auto nedRe = nedRe_Matrix_asLLH(llh);
      const auto enuRe = enuRe_Matrix_asLLH(llh);

      return nedRe * enuRe.transpose();
    }

    inline gtsam::Matrix33 nedRenu_xyz(const gtsam::Point3& xyz)
    {
      const auto nedRe = nedRe_Matrix(xyz);
      const auto enuRe = enuRe_Matrix(xyz);

      return nedRe * enuRe.transpose();
    }

    inline gtsam::Vector3 func_DCM2EulerAngles(const gtsam::Matrix33 &DCM) {
          double rot_x = atan2(DCM(2, 1), DCM(2, 2));
          double rot_y = asin(-DCM(2, 0));
          double rot_z = atan2(DCM(1, 0), DCM(0, 0));
          return (gtsam::Vector3() << rot_x, rot_y, rot_z).finished();
    }


    inline gtsam::Point3 xyz2enu(const gtsam::Point3 &p1, const gtsam::Point3 &p2)
    {
        /*
           inputs ::
           p1 --> ECEF xyz coordinates [meter]
           p2 --> ECEF xyz origin coordinates [meter]
              outputs ::
                  posENU --> ENU position coordinates [meters]
         */

        gtsam::Vector3 posDiff = p1 - p2;
        gtsam::Vector3 orgLLH = xyz2llh(p2);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        gtsam::Matrix R = (gtsam::Matrix33() << (-1 * sinLam), cosLam, 0,
                ((-1 * sinPhi) * cosLam), ((-1 * sinPhi) * sinLam), cosPhi,
                (cosPhi * cosLam), (cosPhi * sinLam), sinPhi).finished();
        //gtsam::Vector3 pos;
        //pos = R*posDiff;
        //gtsam::Point3 posENU;
        gtsam::Point3 posENU = R * posDiff;
        return posENU;
    }

    inline gtsam::Point3 enu2xyz(const gtsam::Point3 &p1, const gtsam::Point3 &p2) {
        /*
           inputs ::
           p1 --> enu coordinates [meter]
           p2 --> ECEF xyz origin coordinates [meter]
           outputs ::
            posXYZ ---> ECEF XYZ position vector [meters]
         */
        gtsam::Vector3 orgLLH = xyz2llh(p2);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        gtsam::Matrix R = (gtsam::Matrix33() << -1 * sinLam, cosLam, 0,
                -1 * sinPhi * cosLam, -1 * sinPhi * sinLam, cosPhi,
                cosPhi * cosLam, cosPhi * sinLam, sinPhi).finished();
        gtsam::Point3 deltaXYZ = R.inverse() * p1;
        return p2 + deltaXYZ;
    }

    inline gtsam::Point3 enu2xyzTrans(const gtsam::Point3 &p1, const gtsam::Point3 &base)
    {
      /*
         inputs ::
         p1 --> enu coordinates [meter]
         p2 --> ECEF xyz origin coordinates [meter]
         outputs ::
          deltaXYZ ---> a rotated vector in ECEF coords [meters]
       */
      gtsam::Vector3 orgLLH = xyz2llh(base);
      double sinPhi = sin(orgLLH(0));
      double cosPhi = cos(orgLLH(0));
      double sinLam = sin(orgLLH(1));
      double cosLam = cos(orgLLH(1));
      gtsam::Matrix R = (gtsam::Matrix33() << -1 * sinLam, cosLam, 0,
              -1 * sinPhi * cosLam, -1 * sinPhi * sinLam, cosPhi,
              cosPhi * cosLam, cosPhi * sinLam, sinPhi).finished();
      gtsam::Point3 deltaXYZ = R.inverse() * p1;
      return deltaXYZ;
    }

    inline gtsam::Matrix func_skewMatrix(const gtsam::Vector3 &w) {
        return (gtsam::Matrix33() << 0, -w(2), w(1),
                w(2), 0, -w(0),
                -w(1), w(0), 0).finished();
    }

    inline double calcEl(const gtsam::Point3 &p1, const gtsam::Point3 &p2) {
        /*
           inputs ::
           p1 --> ECEF xyz satellite coordinates [meter]
           p2 ---> ECEF xyz receiver coordinates [meter]
           output ::
           El --> elevation angle [rad]
         */
        gtsam::Vector3 posENU = xyz2enu(p1, p2);
        double El = std::atan2(posENU(2), posENU.norm());
        return El;
    }

    inline gtsam::Vector3 gravity_ecef(const gtsam::Point3 &r_eb_e)
    {
      /*
         inputs ::
         r_eb_e --> cartesian position of body frame w.r.t. ECEF frame,
                    resolved about ECEF-frame axes (m)
         output ::
         g_e --> acceleration due to gravity w.r.t ECEF frame (m/s^2)
       */
      //parameters
      double R_0 = 6378137; //WGS84 Equatorial radius in meters
      double mu = 3.986004418e+14; //WGS84 Earth gravitational constant (m^3 s^-2)
      double J_2 = 1.082627e-3; //WGS84 Earth's second gravitational constant
      double omega_ie = 7.292115e-5;  // Earth rotation rate (rad/s)
      gtsam::Vector3 g_e;
      // distance from center of the earth
      double mag_r = gtsam::norm3(r_eb_e);
      // dummy output when the input position is 0,0,0
      if (mag_r == 0) { g_e = gtsam::Vector3(0, 0, 0); }
          // calculate gravitational acceleration using (2.142) in "Groves,Principles of GNSS, Inertial, and Multisensor
          //% Integrated Navigation Systems"
      else {
          double z_scale = 5 * pow(r_eb_e.z() / mag_r, 2);
          gtsam::Vector3 tmp = gtsam::Vector3((1 - z_scale) * r_eb_e.x(),
                                              (1 - z_scale) * r_eb_e.y(),
                                              (3 - z_scale) * r_eb_e.z());
          tmp = r_eb_e + (1.5 * J_2 * pow(R_0 / mag_r, 2)) * tmp;

          gtsam::Vector3 gamma = (-mu / pow(mag_r, 3)) * tmp;
          g_e = gamma + pow(omega_ie, 2) *
                        gtsam::Vector3(r_eb_e.x(), r_eb_e.y(), 0);
      }
      return g_e;
    }

    inline gtsam::Vector3 gravity_ned(const gtsam::Point3 &r_eb_e)
    {
      auto llh = xyz2llh(r_eb_e);

      double R_0 = 6378137; //%WGS84 Equatorial radius in meters
      double R_P = 6356752.31425; //%WGS84 Polar radius in meters
      double e = 0.0818191908425; //%WGS84 eccentricity
      double f = 1 / 298.257223563; //%WGS84 flattening
      double mu = 3.986004418E14; //%WGS84 Earth gravitational constant (m^3 s^-2)
      double omega_ie = 7.292115E-5;  //% Earth rotation rate (rad/s)

      double sinsqL = pow(sin(llh.x()), 2);
      double g_0 = 9.7803253359 * (1 + 0.001931853 * sinsqL) / sqrt(1 - e*e * sinsqL);

      return (gtsam::Vector3() << -8.08E-9 * llh.z() * sin(2 * llh.x()),
                                   0.,
                                   g_0 *
                                   (1 - (2 / R_0) * (1 + f * (1 - 2 * sinsqL) + (omega_ie*omega_ie * R_0*R_0 * R_P / mu)) * llh.z() + (3 * llh.z() * llh.z() / pow(R_0, 2)))).finished();

    }

    inline gtsam::Vector3 getMagOrientation(const gtsam::Vector3 &pos)
    {
      /*
      inputs ::
      pos --> cartesian position of body frame w.r.t. ECEF frame,
      resolved about ECEF-frame axes (m)
      output ::
      - --> expectedMagneticForce in ECEF
      */
      //time_t now = time(nullptr);
      //double year = localtime(&now)->tm_year + 1900.0 + localtime(&now)->tm_yday/365.0;
      double year = 2022.18; //idk if float helps..
      gtsam::Vector3 posllh = fgo::utils::xyz2llh(pos);
      gtsam::Vector3 magField_enu;
      GeographicLib::MagneticModel magMod("wmm2020");
      magMod(year, fgo::utils::rad2deg * posllh.x(), fgo::utils::rad2deg * posllh.y(), posllh.z(),
             magField_enu.x(), magField_enu.y(), magField_enu.z());
      gtsam::Rot3 nedRenu = gtsam::Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1);
      magField_enu = 10e-5 * magField_enu; //nT to Gs
      std::cout << std::fixed << "Mag_enu: " << magField_enu.transpose() << std::endl;
      return fgo::utils::nedRe_Matrix(pos).transpose() * nedRenu.matrix() * magField_enu; //nanotesla
   }

    //goGPS: open source software for enhancing the accuracy of low-cost receivers
    //by single-frequency relative kinematic positioning
    //https://www.researchgate.net/publication/257836193_goGPS_Open_Source_Software_for_Enhancing_the_Accuracy_of_Low-cost_Receivers_by_Single-frequency_Relative_Kinematic_Positioning
    inline double eles2nVariance(const double &snr, const double &ele) {
      double elepowm2 = 1 / pow(sin(ele), 2);
      double tenm = pow(10, -(snr - P_T) / P_a);
      double atenm1 = P_A / pow(10, -(P_F - P_T) / P_a) - 1;
      double cnmt = (snr - P_T) / (P_F - P_T);
      return elepowm2 * tenm * (atenm1 * cnmt + 1); //weight * stdvariance
    }

    inline void eigenMatrix2stdVector(const gtsam::Matrix &gtsamMatrix, std::vector<double> &stdVector)
    {
      stdVector.resize(0);
      for (uint32_t i = 0; i < gtsamMatrix.size(); i++)
      {
        stdVector.push_back(gtsamMatrix(i));
      }
    }

    inline gtsam::Point3 WGS84Interpolation(const gtsam::Point3& llh0,
                                            const gtsam::Point3& llh1,
                                            double dt)
    {
      static const double  mz = 6378137.00000 / 6356752.31414;
      static const double _mz = 6356752.31414 / 6378137.00000;
      static const double mzmz = mz * mz;

      auto posECEF0 = llh2xyz(llh0);
      auto posECEF1 = llh2xyz(llh1);

      double r, r0, r1;
      r0 = sqrt((posECEF0.x() * posECEF0.x()) + (posECEF0.y() * posECEF0.y()) + (posECEF0.z() * posECEF0.z() * mzmz));
      r1 = sqrt((posECEF1.x() * posECEF1.x()) + (posECEF1.y() * posECEF1.y()) + (posECEF1.z() * posECEF1.z() * mzmz));

      r   = r0 + (r1 - r0) * dt;

      gtsam::Point3 posInter{ posECEF0.x() + (posECEF1.x() - posECEF0.x()) * dt,
                             posECEF0.y() + (posECEF1.y() - posECEF0.y()) * dt,
                          (posECEF0.z() + (posECEF1.z() - posECEF0.z()) * dt) * mz};

      r /= posInter.norm();
      return {posInter.x() * r,
              posInter.y() * r,
              posInter.z() * r * _mz};
    }

    inline gtsam::Point3 WGS84InterpolationLLH(const gtsam::Point3& llh0,
                                            const gtsam::Point3& llh1,
                                            double dt)
    {
      static const double  mz = 6378137.00000 / 6356752.31414;
      static const double _mz = 6356752.31414 / 6378137.00000;
      static const double mzmz = mz * mz;

      auto posECEF0 = llh2xyz(llh0);
      auto posECEF1 = llh2xyz(llh1);

      double r, r0, r1;
      r0 = sqrt((posECEF0.x() * posECEF0.x()) + (posECEF0.y() * posECEF0.y()) + (posECEF0.z() * posECEF0.z() * mzmz));
      r1 = sqrt((posECEF1.x() * posECEF1.x()) + (posECEF1.y() * posECEF1.y()) + (posECEF1.z() * posECEF1.z() * mzmz));

      r   = r0 + (r1 - r0) * dt;

      gtsam::Point3 posInter{ posECEF0.x() + (posECEF1.x() - posECEF0.x()) * dt,
                              posECEF0.y() + (posECEF1.y() - posECEF0.y()) * dt,
                              (posECEF0.z() + (posECEF1.z() - posECEF0.z()) * dt) * mz};

      r /= posInter.norm();

      const auto ecef = gtsam::Point3(posInter.x() * r, posInter.y() * r, posInter.z() * r * _mz);
      return xyz2llh(ecef);
    }

} //namespace fgonav

#endif