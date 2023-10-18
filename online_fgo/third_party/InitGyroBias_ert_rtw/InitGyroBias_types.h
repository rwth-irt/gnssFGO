//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitGyroBias_types.h
//
// Code generated for Simulink model 'InitGyroBias'.
//
// Model version                  : 2.0
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Sun Nov 27 21:11:25 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_InitGyroBias_types_h_
#define RTW_HEADER_InitGyroBias_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_Inertial_Measurement_
#define DEFINED_TYPEDEF_FOR_Inertial_Measurement_

struct Inertial_Measurement
{
  real_T acceleration[3];
  real_T angularRate[3];
  real_T magnetometer[3];
  real_T timestamp;
};

#endif
#endif                                 // RTW_HEADER_InitGyroBias_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
