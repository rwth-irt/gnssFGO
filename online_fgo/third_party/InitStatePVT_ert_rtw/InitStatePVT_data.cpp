//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitStatePVT_data.cpp
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
#include "InitStatePVT.h"

// Block parameters (default storage)
InitStatePVT::P_InitStatePVT_T InitStatePVT::InitStatePVT_P{
  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S3>/Constant'

  0U,

  // Expression: -1
  //  Referenced by: '<S2>/deg -> rad1'

  -1.0,

  // Expression: 1
  //  Referenced by: '<S1>/enableMemoryReset'

  1.0,

  // Expression: 0
  //  Referenced by: '<S1>/Switch'

  0.0,

  // Computed Parameter: degrad_Gain
  //  Referenced by: '<S2>/deg -> rad'

  0.0174532924F,

  // Expression: false
  //  Referenced by: '<S1>/Constant1'

  false,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S1>/Memory'

  false
};

//
// File trailer for generated code.
//
// [EOF]
//
