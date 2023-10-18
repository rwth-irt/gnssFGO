//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitStatePVT_types.h
//
// Code generated for Simulink model 'InitStatePVT'.
//
// Model version                  : 2.12
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Tue Nov 29 11:05:09 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_InitStatePVT_types_h_
#define RTW_HEADER_InitStatePVT_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_gnssraw_pvt_geodetic_t_
#define DEFINED_TYPEDEF_FOR_gnssraw_pvt_geodetic_t_

struct gnssraw_pvt_geodetic_t
{
  real_T timestamp;
  real_T TOW;
  uint16_T WNc;
  uint8_T Mode;
  uint8_T Error;
  real_T phi;
  real32_T phi_var;
  real_T lambda;
  real32_T lambda_var;
  real_T h;
  real32_T h_var;
  real32_T Undulation;
  real_T Vn;
  real_T Ve;
  real_T Vu;
  real_T TrkGND;
  real_T COG;
  real32_T yaw;
  real32_T yaw_var;
  real32_T pitch_roll;
  real32_T pitch_roll_var;
  real_T RxClkBias;
  real_T RxClkBiasVar;
  real_T RxClkDrift;
  real_T RxClkDriftVar;
  uint8_T TimeSystem;
  uint8_T Datum;
  uint8_T NrSV;
  uint8_T NrSVUsed;
  uint8_T NrSVUsedL1;
  uint8_T NrSVUsedMulti;
  uint8_T WACorrInfo;
  uint16_T ReferenceID;
  uint16_T MeanCorrAge;
  uint32_T SignalInfo;
  uint8_T AlertFlag;
  uint8_T NrBases;
  uint16_T PPPInfo;
  boolean_T DoNotUseValues;
};

#endif
#endif                                 // RTW_HEADER_InitStatePVT_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
