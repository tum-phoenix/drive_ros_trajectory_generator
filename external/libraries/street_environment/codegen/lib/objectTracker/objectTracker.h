//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: objectTracker.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 16:14:26
//
#ifndef __OBJECTTRACKER_H__
#define __OBJECTTRACKER_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "objectTracker_types.h"

// Function Declarations
extern void objectTracker(short init, const emxArray_real_T *laneModel, double
  lanePieceLength, double r[4], double Pk[16], double Q, double Rx, double Ry,
  double x_measure, double y_measure, double delta_x, short hasMeasurement);

#endif

//
// File trailer for objectTracker.h
//
// [EOF]
//
