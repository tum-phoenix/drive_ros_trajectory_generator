//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 16:14:26
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "objectTracker.h"
#include "main.h"
#include "objectTracker_terminate.h"
#include "objectTracker_emxAPI.h"
#include "objectTracker_initialize.h"

// Function Declarations
static void argInit_4x1_real_T(double result[4]);
static void argInit_4x4_real_T(double result[16]);
static emxArray_real_T *argInit_Unboundedx1_real_T();
static short argInit_int16_T();
static double argInit_real_T();
static void main_objectTracker();

// Function Definitions

//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    for (idx1 = 0; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (idx1 << 2)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : emxArray_real_T *
//
static emxArray_real_T *argInit_Unboundedx1_real_T()
{
  emxArray_real_T *result;
  static int iv0[1] = { 2 };

  int idx0;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result = emxCreateND_real_T(1, *(int (*)[1])&iv0[0]);

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result->data[idx0] = argInit_real_T();
  }

  return result;
}

//
// Arguments    : void
// Return Type  : short
//
static short argInit_int16_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_objectTracker()
{
  short init;
  emxArray_real_T *laneModel;
  double lanePieceLength;
  double r[4];
  double Pk[16];

  // Initialize function 'objectTracker' input arguments.
  init = argInit_int16_T();

  // Initialize function input argument 'laneModel'.
  laneModel = argInit_Unboundedx1_real_T();
  lanePieceLength = argInit_real_T();

  // Initialize function input argument 'r'.
  argInit_4x1_real_T(r);

  // Initialize function input argument 'Pk'.
  argInit_4x4_real_T(Pk);

  // Call the entry-point 'objectTracker'.
  objectTracker(init, laneModel, lanePieceLength, r, Pk, argInit_real_T(),
                argInit_real_T(), argInit_real_T(), argInit_real_T(),
                argInit_real_T(), argInit_real_T(), argInit_int16_T());
  emxDestroyArray_real_T(laneModel);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  objectTracker_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_objectTracker();

  // Terminate the application.
  // You do not need to do this more than one time.
  objectTracker_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
