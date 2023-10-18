//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CalculateMeasurementDelay.h
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
#ifndef RTW_HEADER_CalculateMeasurementDelay_h_
#define RTW_HEADER_CalculateMeasurementDelay_h_
#include "rtwtypes.h"
#include "CalculateMeasurementDelay_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model CalculateMeasurementDelay
class CalculateMeasurementDelay final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_CalculateMeasurementDelay_T {
    real_T DiscreteTimeIntegrator_DSTATE;// '<S8>/Discrete-Time Integrator'
    real_T DelayInput1_DSTATE;         // '<S2>/Delay Input1'
    uint64_T DelayInput1_DSTATE_p;     // '<S10>/Delay Input1'
    uint64_T DelayInput1_DSTATE_e;     // '<S3>/Delay Input1'
    real_T delay_alignment;            // '<S6>/MATLAB Function'
    int8_T DiscreteTimeIntegrator_PrevRese;// '<S8>/Discrete-Time Integrator'
    boolean_T Memory1_PreviousInput;   // '<S1>/Memory1'
    boolean_T delay_alignment_not_empty;// '<S6>/MATLAB Function'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_CalculateMeasurementDela_T {
    real_T TOW;                        // '<Root>/TOW'
    uint64_T TimePuls;                 // '<Root>/TimePuls'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_CalculateMeasurementDela_T {
    real_T MeasurementDelay;           // '<Root>/MeasurementDelay'
    real_T MeasurementDelayCalc;       // '<Root>/MeasurementDelayCalc'
  };

  // Parameters (default storage)
  struct P_CalculateMeasurementDelay_T {
    real_T DetectChange_vinit;         // Mask Parameter: DetectChange_vinit
                                          //  Referenced by: '<S2>/Delay Input1'

    uint64_T DetectIncrease_vinit;     // Mask Parameter: DetectIncrease_vinit
                                          //  Referenced by: '<S10>/Delay Input1'

    uint64_T DetectIncrease_vinit_i;   // Mask Parameter: DetectIncrease_vinit_i
                                          //  Referenced by: '<S3>/Delay Input1'

    real_T Constant3_Value;            // Expression: 0.0
                                          //  Referenced by: '<S1>/Constant3'

    real_T Gain_Gain;                  // Expression: 1000
                                          //  Referenced by: '<S7>/Gain'

    real_T msecs_Value;                // Expression: 1000
                                          //  Referenced by: '<S7>/msec //s'

    real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S8>/Discrete-Time Integrator'

    real_T DiscreteTimeIntegrator_IC;  // Expression: 0
                                          //  Referenced by: '<S8>/Discrete-Time Integrator'

    real_T Constant1_Value;            // Expression: 0
                                          //  Referenced by: '<S1>/Constant1'

    real_T Constant2_Value;            // Expression: 0.3
                                          //  Referenced by: '<S1>/Constant2'

    real_T Constant_Value;             // Expression: 1
                                          //  Referenced by: '<S8>/Constant'

    boolean_T Memory1_InitialCondition;
                                 // Computed Parameter: Memory1_InitialCondition
                                    //  Referenced by: '<S1>/Memory1'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_CalculateMeasurement_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  CalculateMeasurementDelay(CalculateMeasurementDelay const&) = delete;

  // Assignment Operator
  CalculateMeasurementDelay& operator= (CalculateMeasurementDelay const&) & =
    delete;

  // Move Constructor
  CalculateMeasurementDelay(CalculateMeasurementDelay &&) = delete;

  // Move Assignment Operator
  CalculateMeasurementDelay& operator= (CalculateMeasurementDelay &&) = delete;

  // Real-Time Model get method
  CalculateMeasurementDelay::RT_MODEL_CalculateMeasurement_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_CalculateMeasurementDela_T
    *pExtU_CalculateMeasurementDela_T)
  {
    CalculateMeasurementDelay_U = *pExtU_CalculateMeasurementDela_T;
  }

  // Root outports get method
  const ExtY_CalculateMeasurementDela_T &getExternalOutputs() const
  {
    return CalculateMeasurementDelay_Y;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  CalculateMeasurementDelay();

  // Destructor
  ~CalculateMeasurementDelay();

  // private data and function members
 private:
  // External inputs
  ExtU_CalculateMeasurementDela_T CalculateMeasurementDelay_U;

  // External outputs
  ExtY_CalculateMeasurementDela_T CalculateMeasurementDelay_Y;

  // Block states
  DW_CalculateMeasurementDelay_T CalculateMeasurementDelay_DW;

  // Tunable parameters
  static P_CalculateMeasurementDelay_T CalculateMeasurementDelay_P;

  // Real-Time Model
  RT_MODEL_CalculateMeasurement_T CalculateMeasurementDelay_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S5>/FixPt Data Type Duplicate' : Unused code path elimination
//  Block '<S1>/Memory' : Unused code path elimination
//  Block '<S4>/Data Type Conversion' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CalculateMeasurementDelay'
//  '<S1>'   : 'CalculateMeasurementDelay/calculate Measurement Delay'
//  '<S2>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/Detect Change'
//  '<S3>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/Detect Increase'
//  '<S4>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter'
//  '<S5>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/Interval Test Dynamic'
//  '<S6>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter/dT_Messung'
//  '<S7>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter/dt_TOW'
//  '<S8>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter/tSim Counter'
//  '<S9>'   : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter/dT_Messung/MATLAB Function'
//  '<S10>'  : 'CalculateMeasurementDelay/calculate Measurement Delay/GPS message delay for navigation filter/tSim Counter/Detect Increase'

#endif                               // RTW_HEADER_CalculateMeasurementDelay_h_

//
// File trailer for generated code.
//
// [EOF]
//
