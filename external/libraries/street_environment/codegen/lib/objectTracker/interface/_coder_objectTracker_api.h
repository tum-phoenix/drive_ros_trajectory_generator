/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_objectTracker_api.h
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 06-Oct-2015 16:14:26
 */

#ifndef ___CODER_OBJECTTRACKER_API_H__
#define ___CODER_OBJECTTRACKER_API_H__

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_objectTracker_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void objectTracker(int16_T init, emxArray_real_T *laneModel, real_T
  lanePieceLength, real_T r[4], real_T Pk[16], real_T Q, real_T Rx, real_T Ry,
  real_T x_measure, real_T y_measure, real_T delta_x, int16_T hasMeasurement);
extern void objectTracker_api(const mxArray *prhs[12], const mxArray *plhs[2]);
extern void objectTracker_atexit(void);
extern void objectTracker_initialize(void);
extern void objectTracker_terminate(void);
extern void objectTracker_xil_terminate(void);

#endif

/*
 * File trailer for _coder_objectTracker_api.h
 *
 * [EOF]
 */
