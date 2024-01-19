//
// File: func_ECEF2LLH.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Jan-2024 11:11:40
//

// Include Files
#include "func_ECEF2LLH.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cmath>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// FUNC_LLH2ECEF Summary of this function goes here
//    Detailed explanation goes here
//
// Arguments    : double X
//                double Y
//                double Z
//                double *phi
//                double *lambda
//                double *h
// Return Type  : void
//
void func_ECEF2LLH(double X, double Y, double Z, double *phi, double *lambda,
                   double *h)
{
  double oldh;
  double phi_tmp;
  int k;
  //  Parameter des Erdmodells
  //      a     = 6378137;          % Große Halbachse Ellipsoid [m]
  //      b     = 6356752.3142;     % Kleine Halbachse Ellipsoid [m]
  //      f     = 1/298.257223563;
  //      e     = sqrt(f*(2-f));    % Exzentrizität des Ellipsoids
  //
  //  p=sqrt(x^2+y^2);
  //  E_square=a^2-b^2;
  //  F=54*b^2*z^2;
  //  G=p^2+(1-e^2)*z^2-e^2*E_square;
  //  c=((e^4)*F*p^2)/(G^3);
  //  s=(1+c+sqrt(c^2+2*c))^(1/3);
  //  P=(F)/((3+s+1/s+1)^2*G^2);
  //  Q=sqrt(1+2*e^4*P);
  //  r_0=-(P*e^2*p)/(1+Q)+sqrt(0.5*a^2*(1+1/Q)-(P*(1-e^2)*z^2)/(Q*(1+Q))-0.5*P*p^2);
  //  U=sqrt((p-e^2*r_0)^2+z^2);
  //  V=sqrt((p-e^2*r_0)^2+(1-e^2)*z^2);
  //  z_0=(b^2*z)/(a*V);
  //  e_strich=e*a/b;
  //  Lat=atan((z+(e_strich)^2*z_0)/(p));
  //  Lon=atan2(y,x);
  //  h=U*((b^2)/(a*V)-1);
  // CART2GEO Conversion of Cartesian coordinates (X,Y,Z) to geographical
  // coordinates (phi, lambda, h) on a selected reference ellipsoid.
  //
  // [phi, lambda, h] = cart2geo(X, Y, Z, i);
  //
  //    Choices i of Reference Ellipsoid for Geographical Coordinates
  //           	  1. International Ellipsoid 1924
  // 	          2. International Ellipsoid 1967
  // 	          3. World Geodetic System 1972
  // 	          4. Geodetic Reference System 1980
  // 	          5. World Geodetic System 1984
  // Kai Borre 10-13-98
  // Copyright (c) by Kai Borre
  // Revision: 1.0   Date: 1998/10/23
  //
  //  CVS record:
  //  $Id: cart2geo.m,v 1.1.2.2 2006/08/22 13:45:59 dpl Exp $
  // ==========================================================================
  phi_tmp = std::sqrt(X * X + Y * Y);
  *phi =
      std::atan(Z / (phi_tmp * -0.99664718933525243 * 0.0033528106647474805));
  *h = 0.1;
  oldh = 0.0;
  k = 0;
  while ((std::abs(*h - oldh) > 1.0E-12) && (k < 3000)) {
    double N;
    // fprintf('test  für cart2geo\n');
    oldh = *h;
    N = std::cos(*phi);
    N = 6.3995936257584924E+6 /
        std::sqrt(0.0067394967422764341 * (N * N) + 1.0);
    *phi =
        std::atan(Z / (phi_tmp * (1.0 - 0.0066943799901413165 * N / (N + *h))));
    *h = phi_tmp / std::cos(*phi) - N;
    k++;
  }
  *phi *= 57.295779513082323;
  *lambda = rt_atan2d_snf(Y, X) * 57.295779513082323;
  //  Kommentar Ha:
  //  - Funktion überarbeitet, so dass auch vektorwertige Größen verarbeitet
  //  werden können.
  //  - Wahl anderer Referenzellipsoiden entfernt.
}

//
// File trailer for func_ECEF2LLH.cpp
//
// [EOF]
//
