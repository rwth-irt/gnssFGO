//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CalculateMeasurementDelay.cpp
//
// Code generated for Simulink model 'CalculateMeasurementDelay'.
//
// Model version                  : 2.6
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Nov 30 17:07:43 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "CalculateMeasurementDelay.h"
#include <cmath>
#include "rtwtypes.h"

// Model step function
void CalculateMeasurementDelay::step()
{
  real_T rtb_Gain;
  boolean_T rtb_FixPtRelationalOperator;

  // Gain: '<S7>/Gain' incorporates:
  //   Inport: '<Root>/TOW'

  rtb_Gain = CalculateMeasurementDelay_P.Gain_Gain *
    CalculateMeasurementDelay_U.TOW;

  // Sum: '<S7>/Sum' incorporates:
  //   Constant: '<S7>/msec //s'
  //   Product: '<S7>/Divide'
  //   Product: '<S7>/Product'
  //   Rounding: '<S7>/Rounding Function'

  rtb_Gain -= std::floor(rtb_Gain / CalculateMeasurementDelay_P.msecs_Value) *
    CalculateMeasurementDelay_P.msecs_Value;

  // RelationalOperator: '<S10>/FixPt Relational Operator' incorporates:
  //   Inport: '<Root>/TimePuls'
  //   UnitDelay: '<S10>/Delay Input1'
  //
  //  Block description for '<S10>/Delay Input1':
  //
  //   Store in Global RAM

  rtb_FixPtRelationalOperator = (CalculateMeasurementDelay_U.TimePuls >
    CalculateMeasurementDelay_DW.DelayInput1_DSTATE_p);

  // DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  if (rtb_FixPtRelationalOperator &&
      (CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_DSTATE =
      CalculateMeasurementDelay_P.DiscreteTimeIntegrator_IC;
  }

  // MATLAB Function: '<S6>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'

  CalculateMeasurementDelay_Y.MeasurementDelayCalc =
    CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_DSTATE - rtb_Gain;
  if (CalculateMeasurementDelay_Y.MeasurementDelayCalc >= -0.05) {
    CalculateMeasurementDelay_Y.MeasurementDelayCalc /= 1000.0;
  } else {
    CalculateMeasurementDelay_Y.MeasurementDelayCalc =
      (CalculateMeasurementDelay_Y.MeasurementDelayCalc + 1000.0) / 1000.0;
  }

  if ((!CalculateMeasurementDelay_DW.delay_alignment_not_empty) && (rtb_Gain !=
       0.0)) {
    CalculateMeasurementDelay_DW.delay_alignment =
      CalculateMeasurementDelay_Y.MeasurementDelayCalc;
    CalculateMeasurementDelay_DW.delay_alignment_not_empty = true;
  }

  if (CalculateMeasurementDelay_DW.delay_alignment_not_empty) {
    CalculateMeasurementDelay_Y.MeasurementDelayCalc -=
      CalculateMeasurementDelay_DW.delay_alignment;
  }

  // End of MATLAB Function: '<S6>/MATLAB Function'

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Constant: '<S1>/Constant2'
  //   Inport: '<Root>/TOW'
  //   Logic: '<S1>/Logical Operator'
  //   Logic: '<S5>/FixPt Logical Operator'
  //   Memory: '<S1>/Memory1'
  //   RelationalOperator: '<S2>/FixPt Relational Operator'
  //   RelationalOperator: '<S5>/Lower Test'
  //   RelationalOperator: '<S5>/Upper Test'
  //   UnitDelay: '<S2>/Delay Input1'
  //
  //  Block description for '<S2>/Delay Input1':
  //
  //   Store in Global RAM

  if (CalculateMeasurementDelay_DW.Memory1_PreviousInput &&
      (CalculateMeasurementDelay_U.TOW !=
       CalculateMeasurementDelay_DW.DelayInput1_DSTATE) &&
      ((CalculateMeasurementDelay_P.Constant1_Value <=
        CalculateMeasurementDelay_Y.MeasurementDelayCalc) &&
       (CalculateMeasurementDelay_Y.MeasurementDelayCalc <=
        CalculateMeasurementDelay_P.Constant2_Value))) {
    // Outport: '<Root>/MeasurementDelay'
    CalculateMeasurementDelay_Y.MeasurementDelay =
      CalculateMeasurementDelay_Y.MeasurementDelayCalc;
  } else {
    // Outport: '<Root>/MeasurementDelay' incorporates:
    //   Constant: '<S1>/Constant3'

    CalculateMeasurementDelay_Y.MeasurementDelay =
      CalculateMeasurementDelay_P.Constant3_Value;
  }

  // End of Switch: '<S1>/Switch'

  // Update for UnitDelay: '<S10>/Delay Input1' incorporates:
  //   Inport: '<Root>/TimePuls'
  //
  //  Block description for '<S10>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE_p =
    CalculateMeasurementDelay_U.TimePuls;

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S8>/Constant'

  CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_DSTATE +=
    CalculateMeasurementDelay_P.DiscreteTimeIntegrator_gainval *
    CalculateMeasurementDelay_P.Constant_Value;
  CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_PrevRese =
    static_cast<int8_T>(rtb_FixPtRelationalOperator);

  // Update for Memory: '<S1>/Memory1' incorporates:
  //   Inport: '<Root>/TimePuls'
  //   Logic: '<S1>/Logical Operator1'
  //   RelationalOperator: '<S3>/FixPt Relational Operator'
  //   UnitDelay: '<S3>/Delay Input1'
  //
  //  Block description for '<S3>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.Memory1_PreviousInput =
    ((CalculateMeasurementDelay_U.TimePuls >
      CalculateMeasurementDelay_DW.DelayInput1_DSTATE_e) ||
     CalculateMeasurementDelay_DW.Memory1_PreviousInput);

  // Update for UnitDelay: '<S2>/Delay Input1' incorporates:
  //   Inport: '<Root>/TOW'
  //
  //  Block description for '<S2>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE =
    CalculateMeasurementDelay_U.TOW;

  // Update for UnitDelay: '<S3>/Delay Input1' incorporates:
  //   Inport: '<Root>/TimePuls'
  //
  //  Block description for '<S3>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE_e =
    CalculateMeasurementDelay_U.TimePuls;
}

// Model initialize function
void CalculateMeasurementDelay::initialize()
{
  // InitializeConditions for UnitDelay: '<S10>/Delay Input1'
  //
  //  Block description for '<S10>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE_p =
    CalculateMeasurementDelay_P.DetectIncrease_vinit;

  // InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' 
  CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_DSTATE =
    CalculateMeasurementDelay_P.DiscreteTimeIntegrator_IC;
  CalculateMeasurementDelay_DW.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for Memory: '<S1>/Memory1'
  CalculateMeasurementDelay_DW.Memory1_PreviousInput =
    CalculateMeasurementDelay_P.Memory1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S2>/Delay Input1'
  //
  //  Block description for '<S2>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE =
    CalculateMeasurementDelay_P.DetectChange_vinit;

  // InitializeConditions for UnitDelay: '<S3>/Delay Input1'
  //
  //  Block description for '<S3>/Delay Input1':
  //
  //   Store in Global RAM

  CalculateMeasurementDelay_DW.DelayInput1_DSTATE_e =
    CalculateMeasurementDelay_P.DetectIncrease_vinit_i;
}

// Model terminate function
void CalculateMeasurementDelay::terminate()
{
  // (no terminate code required)
}

// Constructor
CalculateMeasurementDelay::CalculateMeasurementDelay() :
  CalculateMeasurementDelay_U(),
  CalculateMeasurementDelay_Y(),
  CalculateMeasurementDelay_DW(),
  CalculateMeasurementDelay_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
CalculateMeasurementDelay::~CalculateMeasurementDelay()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
CalculateMeasurementDelay::RT_MODEL_CalculateMeasurement_T
  * CalculateMeasurementDelay::getRTM()
{
  return (&CalculateMeasurementDelay_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
