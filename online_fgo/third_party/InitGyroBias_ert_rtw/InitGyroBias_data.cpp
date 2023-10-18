//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitGyroBias_data.cpp
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
#include "InitGyroBias.h"

// Block parameters (default storage)
InitGyroBias::P_InitGyroBias_T InitGyroBias::InitGyroBias_P = {
  // Expression: [0 0 0]'
  //  Referenced by: '<Root>/Constant3'

  { 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<Root>/Reset'

  0.0,

  // Expression: 1
  //  Referenced by: '<S1>/Constant'

  1.0,

  // Computed Parameter: Time_gainval
  //  Referenced by: '<S1>/Time'

  0.01,

  // Expression: 0
  //  Referenced by: '<S1>/Time'

  0.0,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<S1>/Discrete-Time Integrator'

  0.01,

  // Expression: 0
  //  Referenced by: '<S1>/Discrete-Time Integrator'

  0.0
};

//
// File trailer for generated code.
//
// [EOF]
//
