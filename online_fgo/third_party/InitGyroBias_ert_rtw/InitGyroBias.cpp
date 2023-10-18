//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitGyroBias.cpp
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

// Model step function
void InitGyroBias::step()
{
  // DiscreteIntegrator: '<S1>/Time' incorporates:
  //   Constant: '<Root>/Reset'

  if (InitGyroBias_DW.Time_SYSTEM_ENABLE == 0) {
    if ((InitGyroBias_P.Reset_Value > 0.0) &&
        (InitGyroBias_DW.Time_PrevResetState <= 0)) {
      InitGyroBias_DW.Time_DSTATE = InitGyroBias_P.Time_IC;
    } else {
      // DiscreteIntegrator: '<S1>/Time' incorporates:
      //   Constant: '<S1>/Constant'

      InitGyroBias_DW.Time_DSTATE += InitGyroBias_P.Time_gainval *
        InitGyroBias_P.Constant_Value;
    }
  }

  // End of DiscreteIntegrator: '<S1>/Time'

  // DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
  //   Constant: '<Root>/Reset'

  if (InitGyroBias_DW.DiscreteTimeIntegrator_SYSTEM_E == 0) {
    if ((InitGyroBias_P.Reset_Value > 0.0) &&
        (InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[0] =
        InitGyroBias_P.DiscreteTimeIntegrator_IC;
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[1] =
        InitGyroBias_P.DiscreteTimeIntegrator_IC;
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[2] =
        InitGyroBias_P.DiscreteTimeIntegrator_IC;
    } else {
      // DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
      //   Inport: '<Root>/InertialMeasurementBus'

      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[0] +=
        InitGyroBias_P.DiscreteTimeIntegrator_gainval *
        InitGyroBias_U.InertialMeasurementBus.angularRate[0];
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[1] +=
        InitGyroBias_P.DiscreteTimeIntegrator_gainval *
        InitGyroBias_U.InertialMeasurementBus.angularRate[1];
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[2] +=
        InitGyroBias_P.DiscreteTimeIntegrator_gainval *
        InitGyroBias_U.InertialMeasurementBus.angularRate[2];
    }
  }

  // End of DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  // Switch: '<Root>/Switch' incorporates:
  //   Inport: '<Root>/InitAsZeros'
  //   Switch: '<S1>/Switch'

  if (InitGyroBias_U.InitAsZeros) {
    // Outport: '<Root>/gyroBiasArray' incorporates:
    //   Constant: '<Root>/Constant3'

    InitGyroBias_Y.gyroBiasArray[0] = InitGyroBias_P.Constant3_Value[0];
    InitGyroBias_Y.gyroBiasArray[1] = InitGyroBias_P.Constant3_Value[1];
    InitGyroBias_Y.gyroBiasArray[2] = InitGyroBias_P.Constant3_Value[2];
  } else if (InitGyroBias_DW.Time_DSTATE != 0.0) {
    // Outport: '<Root>/gyroBiasArray' incorporates:
    //   Product: '<S1>/Divide'
    //   Switch: '<S1>/Switch'

    InitGyroBias_Y.gyroBiasArray[0] =
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[0] /
      InitGyroBias_DW.Time_DSTATE;
    InitGyroBias_Y.gyroBiasArray[1] =
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[1] /
      InitGyroBias_DW.Time_DSTATE;
    InitGyroBias_Y.gyroBiasArray[2] =
      InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[2] /
      InitGyroBias_DW.Time_DSTATE;
  } else {
    // Outport: '<Root>/gyroBiasArray' incorporates:
    //   Inport: '<Root>/InertialMeasurementBus'
    //   Switch: '<S1>/Switch'

    InitGyroBias_Y.gyroBiasArray[0] =
      InitGyroBias_U.InertialMeasurementBus.angularRate[0];
    InitGyroBias_Y.gyroBiasArray[1] =
      InitGyroBias_U.InertialMeasurementBus.angularRate[1];
    InitGyroBias_Y.gyroBiasArray[2] =
      InitGyroBias_U.InertialMeasurementBus.angularRate[2];
  }

  // End of Switch: '<Root>/Switch'

  // Update for DiscreteIntegrator: '<S1>/Time' incorporates:
  //   Constant: '<Root>/Reset'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  InitGyroBias_DW.Time_SYSTEM_ENABLE = 0U;
  if (InitGyroBias_P.Reset_Value > 0.0) {
    InitGyroBias_DW.Time_PrevResetState = 1;
    InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese = 1;
  } else {
    if (InitGyroBias_P.Reset_Value < 0.0) {
      InitGyroBias_DW.Time_PrevResetState = -1;
    } else if (InitGyroBias_P.Reset_Value == 0.0) {
      InitGyroBias_DW.Time_PrevResetState = 0;
    } else {
      InitGyroBias_DW.Time_PrevResetState = 2;
    }

    if (InitGyroBias_P.Reset_Value < 0.0) {
      InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese = -1;
    } else if (InitGyroBias_P.Reset_Value == 0.0) {
      InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese = 0;
    } else {
      InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese = 2;
    }
  }

  // End of Update for DiscreteIntegrator: '<S1>/Time'

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  InitGyroBias_DW.DiscreteTimeIntegrator_SYSTEM_E = 0U;
}

// Model initialize function
void InitGyroBias::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S1>/Time'
  InitGyroBias_DW.Time_DSTATE = InitGyroBias_P.Time_IC;
  InitGyroBias_DW.Time_PrevResetState = 2;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' 
  InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[0] =
    InitGyroBias_P.DiscreteTimeIntegrator_IC;
  InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[1] =
    InitGyroBias_P.DiscreteTimeIntegrator_IC;
  InitGyroBias_DW.DiscreteTimeIntegrator_DSTATE[2] =
    InitGyroBias_P.DiscreteTimeIntegrator_IC;
  InitGyroBias_DW.DiscreteTimeIntegrator_PrevRese = 2;

  // Enable for DiscreteIntegrator: '<S1>/Time'
  InitGyroBias_DW.Time_SYSTEM_ENABLE = 1U;

  // Enable for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  InitGyroBias_DW.DiscreteTimeIntegrator_SYSTEM_E = 1U;
}

// Model terminate function
void InitGyroBias::terminate()
{
  // (no terminate code required)
}

// Constructor
InitGyroBias::InitGyroBias() :
  InitGyroBias_U(),
  InitGyroBias_Y(),
  InitGyroBias_DW(),
  InitGyroBias_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
InitGyroBias::~InitGyroBias()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
InitGyroBias::RT_MODEL_InitGyroBias_T * InitGyroBias::getRTM()
{
  return (&InitGyroBias_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
