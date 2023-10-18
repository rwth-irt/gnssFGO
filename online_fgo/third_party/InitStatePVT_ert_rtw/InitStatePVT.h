//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: InitStatePVT.h
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
#ifndef RTW_HEADER_InitStatePVT_h_
#define RTW_HEADER_InitStatePVT_h_
#include "rtwtypes.h"
#include "InitStatePVT_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model InitStatePVT
class InitStatePVT final
{
  // public data and function members
 public:
  // External inputs (root inport signals with default storage)
  struct ExtU_InitStatePVT_T {
    gnssraw_pvt_geodetic_t PvtGeodeticBus;// '<Root>/PvtGeodeticBus'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_InitStatePVT_T {
    real_T positionLlhArray[6];        // '<Root>/positionLlhArray'
    real_T velocityNedArray[3];        // '<Root>/velocityNedArray'
    real32_T receiverYawAngle;         // '<Root>/receiverYawAngle'
    real_T clockErrorArray[2];         // '<Root>/clockErrorArray'
    boolean_T successFlag;             // '<Root>/successFlag'
    boolean_T failFlag;                // '<Root>/failFlag'
  };

  // Parameters (default storage)
  struct P_InitStatePVT_T {
    uint8_T CompareToConstant_const;  // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S3>/Constant'

    real_T degrad1_Gain;               // Expression: -1
                                          //  Referenced by: '<S2>/deg -> rad1'

    real_T enableMemoryReset_Value;    // Expression: 1
                                          //  Referenced by: '<S1>/enableMemoryReset'

    real_T Switch_Threshold;           // Expression: 0
                                          //  Referenced by: '<S1>/Switch'

    real32_T degrad_Gain;              // Computed Parameter: degrad_Gain
                                          //  Referenced by: '<S2>/deg -> rad'

    boolean_T Constant1_Value;         // Expression: false
                                          //  Referenced by: '<S1>/Constant1'

    boolean_T Memory_InitialCondition;
                                  // Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S1>/Memory'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_InitStatePVT_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  InitStatePVT(InitStatePVT const&) = delete;

  // Assignment Operator
  InitStatePVT& operator= (InitStatePVT const&) & = delete;

  // Move Constructor
  InitStatePVT(InitStatePVT &&) = delete;

  // Move Assignment Operator
  InitStatePVT& operator= (InitStatePVT &&) = delete;

  // Real-Time Model get method
  InitStatePVT::RT_MODEL_InitStatePVT_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_InitStatePVT_T *pExtU_InitStatePVT_T)
  {
    InitStatePVT_U = *pExtU_InitStatePVT_T;
  }

  // Root outports get method
  const ExtY_InitStatePVT_T &getExternalOutputs() const
  {
    return InitStatePVT_Y;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  InitStatePVT();

  // Destructor
  ~InitStatePVT();

  // private data and function members
 private:
  // External inputs
  ExtU_InitStatePVT_T InitStatePVT_U;

  // External outputs
  ExtY_InitStatePVT_T InitStatePVT_Y;

  // Tunable parameters
  static P_InitStatePVT_T InitStatePVT_P;

  // Real-Time Model
  RT_MODEL_InitStatePVT_T InitStatePVT_M;
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
//  '<Root>' : 'InitStatePVT'
//  '<S1>'   : 'InitStatePVT/Memory'
//  '<S2>'   : 'InitStatePVT/Navigation Solution from Receiver Reference'
//  '<S3>'   : 'InitStatePVT/Navigation Solution from Receiver Reference/Compare To Constant'

#endif                                 // RTW_HEADER_InitStatePVT_h_

//
// File trailer for generated code.
//
// [EOF]
//
