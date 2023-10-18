//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitStatePVT.cpp
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

// Model step function
void InitStatePVT::step()
{
  // Outport: '<Root>/positionLlhArray' incorporates:
  //   DataTypeConversion: '<S2>/Data Type Conversion'
  //   DataTypeConversion: '<S2>/Data Type Conversion1'
  //   DataTypeConversion: '<S2>/Data Type Conversion2'
  //   Inport: '<Root>/PvtGeodeticBus'

  InitStatePVT_Y.positionLlhArray[0] = InitStatePVT_U.PvtGeodeticBus.phi;
  InitStatePVT_Y.positionLlhArray[1] = InitStatePVT_U.PvtGeodeticBus.phi_var;
  InitStatePVT_Y.positionLlhArray[2] = InitStatePVT_U.PvtGeodeticBus.lambda;
  InitStatePVT_Y.positionLlhArray[3] = InitStatePVT_U.PvtGeodeticBus.lambda_var;
  InitStatePVT_Y.positionLlhArray[4] = InitStatePVT_U.PvtGeodeticBus.h;
  InitStatePVT_Y.positionLlhArray[5] = InitStatePVT_U.PvtGeodeticBus.h_var;

  // Outport: '<Root>/velocityNedArray' incorporates:
  //   Gain: '<S2>/deg -> rad1'
  //   Inport: '<Root>/PvtGeodeticBus'

  InitStatePVT_Y.velocityNedArray[0] = InitStatePVT_U.PvtGeodeticBus.Vn;
  InitStatePVT_Y.velocityNedArray[1] = InitStatePVT_U.PvtGeodeticBus.Ve;
  InitStatePVT_Y.velocityNedArray[2] = InitStatePVT_P.degrad1_Gain *
    InitStatePVT_U.PvtGeodeticBus.Vu;

  // Outport: '<Root>/receiverYawAngle' incorporates:
  //   Gain: '<S2>/deg -> rad'
  //   Inport: '<Root>/PvtGeodeticBus'

  InitStatePVT_Y.receiverYawAngle = InitStatePVT_P.degrad_Gain *
    InitStatePVT_U.PvtGeodeticBus.yaw;

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<S3>/Constant'
  //   Inport: '<Root>/PvtGeodeticBus'
  //   RelationalOperator: '<S3>/Compare'

  InitStatePVT_Y.successFlag = (InitStatePVT_U.PvtGeodeticBus.Error ==
    InitStatePVT_P.CompareToConstant_const);

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<S1>/enableMemoryReset'

  if (InitStatePVT_P.enableMemoryReset_Value > InitStatePVT_P.Switch_Threshold)
  {
    // Logic: '<S1>/Logical Operator1' incorporates:
    //   Constant: '<S1>/Constant1'

    InitStatePVT_Y.failFlag = InitStatePVT_P.Constant1_Value;
  }

  // End of Switch: '<S1>/Switch'

  // Logic: '<S1>/Logical Operator1' incorporates:
  //   Logic: '<Root>/Logical Operator2'

  InitStatePVT_Y.failFlag = ((!InitStatePVT_Y.successFlag) ||
    InitStatePVT_Y.failFlag);

  // Outport: '<Root>/clockErrorArray' incorporates:
  //   Inport: '<Root>/PvtGeodeticBus'

  InitStatePVT_Y.clockErrorArray[0] = InitStatePVT_U.PvtGeodeticBus.RxClkBias;
  InitStatePVT_Y.clockErrorArray[1] = InitStatePVT_U.PvtGeodeticBus.RxClkDrift;
}

// Model initialize function
void InitStatePVT::initialize()
{
  // InitializeConditions for Logic: '<S1>/Logical Operator1' incorporates:
  //   Memory: '<S1>/Memory'

  InitStatePVT_Y.failFlag = InitStatePVT_P.Memory_InitialCondition;
}

// Model terminate function
void InitStatePVT::terminate()
{
  // (no terminate code required)
}

// Constructor
InitStatePVT::InitStatePVT() :
  InitStatePVT_U(),
  InitStatePVT_Y(),
  InitStatePVT_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
InitStatePVT::~InitStatePVT()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
InitStatePVT::RT_MODEL_InitStatePVT_T * InitStatePVT::getRTM()
{
  return (&InitStatePVT_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
