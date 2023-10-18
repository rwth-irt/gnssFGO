//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitGyroBias.h
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
#ifndef RTW_HEADER_InitGyroBias_h_
#define RTW_HEADER_InitGyroBias_h_
#include "rtwtypes.h"
#include "InitGyroBias_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model InitGyroBias
class InitGyroBias
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_InitGyroBias_T {
    real_T Time_DSTATE;                // '<S1>/Time'
    real_T DiscreteTimeIntegrator_DSTATE[3];// '<S1>/Discrete-Time Integrator'
    int8_T Time_PrevResetState;        // '<S1>/Time'
    int8_T DiscreteTimeIntegrator_PrevRese;// '<S1>/Discrete-Time Integrator'
    uint8_T Time_SYSTEM_ENABLE;        // '<S1>/Time'
    uint8_T DiscreteTimeIntegrator_SYSTEM_E;// '<S1>/Discrete-Time Integrator'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_InitGyroBias_T {
    Inertial_Measurement InertialMeasurementBus;// '<Root>/InertialMeasurementBus' 
    boolean_T InitAsZeros;             // '<Root>/InitAsZeros'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_InitGyroBias_T {
    real_T gyroBiasArray[3];           // '<Root>/gyroBiasArray'
  };

  // Parameters (default storage)
  struct P_InitGyroBias_T {
    real_T Constant3_Value[3];         // Expression: [0 0 0]'
                                          //  Referenced by: '<Root>/Constant3'

    real_T Reset_Value;                // Expression: 0
                                          //  Referenced by: '<Root>/Reset'

    real_T Constant_Value;             // Expression: 1
                                          //  Referenced by: '<S1>/Constant'

    real_T Time_gainval;               // Computed Parameter: Time_gainval
                                          //  Referenced by: '<S1>/Time'

    real_T Time_IC;                    // Expression: 0
                                          //  Referenced by: '<S1>/Time'

    real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S1>/Discrete-Time Integrator'

    real_T DiscreteTimeIntegrator_IC;  // Expression: 0
                                          //  Referenced by: '<S1>/Discrete-Time Integrator'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_InitGyroBias_T {
    const char_T * volatile errorStatus;
  };

  // Real-Time Model get method
  InitGyroBias::RT_MODEL_InitGyroBias_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_InitGyroBias_T *pExtU_InitGyroBias_T)
  {
    InitGyroBias_U = *pExtU_InitGyroBias_T;
  }

  // Root outports get method
  const ExtY_InitGyroBias_T &getExternalOutputs() const
  {
    return InitGyroBias_Y;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  InitGyroBias();

  // Destructor
  ~InitGyroBias();

  // private data and function members
 private:
  // External inputs
  ExtU_InitGyroBias_T InitGyroBias_U;

  // External outputs
  ExtY_InitGyroBias_T InitGyroBias_Y;

  // Block states
  DW_InitGyroBias_T InitGyroBias_DW;

  // Tunable parameters
  static P_InitGyroBias_T InitGyroBias_P;

  // Real-Time Model
  RT_MODEL_InitGyroBias_T InitGyroBias_M;
};

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
//  '<Root>' : 'InitGyroBias'
//  '<S1>'   : 'InitGyroBias/Calc Mean'

#endif                                 // RTW_HEADER_InitGyroBias_h_

//
// File trailer for generated code.
//
// [EOF]
//
