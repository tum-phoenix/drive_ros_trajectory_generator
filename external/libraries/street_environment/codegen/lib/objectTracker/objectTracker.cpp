//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: objectTracker.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 16:14:26
//

// Include Files
#include "rt_nonfinite.h"
#include "objectTracker.h"
#include "objectTracker_emxutil.h"

// Function Definitions

//
// x und y Werte sind in Fzg-Koordinaten
// Arguments    : short init
//                const emxArray_real_T *laneModel
//                double lanePieceLength
//                double r[4]
//                double Pk[16]
//                double Q
//                double Rx
//                double Ry
//                double x_measure
//                double y_measure
//                double delta_x
//                short hasMeasurement
// Return Type  : void
//
void objectTracker(short init, const emxArray_real_T *laneModel, double
                   lanePieceLength, double r[4], double Pk[16], double Q, double
                   Rx, double Ry, double x_measure, double y_measure, double
                   delta_x, short hasMeasurement)
{
  emxArray_real_T *P;
  int i0;
  int r1;
  double phi;
  int s;
  emxArray_real_T *b_P;
  int r2;
  int k;
  int loop_ub;
  emxArray_real_T *c_P;
  emxArray_real_T *d_P;
  emxArray_real_T *e_P;
  double D[4];
  double a21;
  double a22;
  double dist_point;
  double a[2];
  double b_x_measure[2];
  double f_P[2];
  double v[2];
  double lambda;
  double b_v;
  double b_a[3];
  double b[3];
  double cp_idx_2;
  double dist_line;
  double d;
  double c_a[4];
  static const double d_a[16] = { 1.0, 0.0, 0.0, 0.0, 0.01, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.01, 1.0 };

  static const signed char e_a[4] = { -1, 0, 0, 0 };

  double f_a[16];
  double g_a[16];
  static const double b_b[16] = { 1.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.01, 0.0, 0.0, 0.0, 1.0 };

  double b_d[16];
  double h_a[8];
  static const signed char i_a[8] = { 1, 0, 0, 0, 0, 1, 0, 0 };

  static const signed char c_b[8] = { 1, 0, 0, 0, 0, 0, 1, 0 };

  double b_Rx[4];
  double S[4];
  double y[8];
  double K[8];
  double c_x_measure[2];
  emxInit_real_T1(&P, 2);

  //  init: [scalar] set to "1" once when object is first detected to initialize state vector, then set to "0" 
  //  laneModel: [px1] state vector of the current lane model
  //  lanePieceLength: [scalar] length of linear pieces in the lane model
  //  r: [4x1] state vector of the object being tracked
  //      [ s_x (arc length from vehicle to object in the diection of the lane model);  
  //        v_x (velocity of object in direction of lane);
  //        s_y (lateral position of object relative to middle lane);
  //        v_y (lateral velocity of object)]
  //  Pk:[4x4] covariance of the current state
  //  Q: [scalar] standard deviation of the state transition (how reliable is delta_x?)  
  //  Rx: [scalar] standard deviation of the measurement in lane direction
  //  Ry: [scalar] standard deviation of the measurement lateral to the lane
  //  x_measure: [nx1] measured x values of the object being tracked
  //  y_measure: [nx1] measured y values of the object being tracked
  //  delta_x: [scalar] forward movement of the vehicle relative to the lane model 
  // % find distance between lane model and object measurements to get the measurment of s (s_measure) 
  i0 = P->size[0] * P->size[1];
  P->size[0] = laneModel->size[0];
  P->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)P, i0, (int)sizeof(double));
  r1 = laneModel->size[0] * 3;
  for (i0 = 0; i0 < r1; i0++) {
    P->data[i0] = 0.0;
  }

  // x, y, phi
  phi = laneModel->data[1];

  // erster Punkt
  P->data[0] = 0.0;
  P->data[P->size[0]] = laneModel->data[0];
  P->data[P->size[0] << 1] = laneModel->data[1];

  // zweiter Punkt
  P->data[1] = P->data[0] + lanePieceLength * cos(laneModel->data[1]);
  P->data[1 + P->size[0]] = P->data[P->size[0]] + lanePieceLength * sin
    (laneModel->data[1]);
  P->data[1 + (P->size[0] << 1)] = laneModel->data[1];
  for (s = 2; s - 2 <= laneModel->size[0] - 3; s++) {
    //  "-" wegen VZ-Definition der Kruemmung
    phi = (phi + 2.0 * acos(-lanePieceLength * laneModel->data[s] / 2.0)) -
      3.1415926535897931;
    P->data[s] = P->data[s - 1] + lanePieceLength * cos(phi);
    P->data[s + P->size[0]] = P->data[(s + P->size[0]) - 1] + lanePieceLength *
      sin(phi);
    P->data[s + (P->size[0] << 1)] = phi;
  }

  emxInit_real_T(&b_P, 1);
  r1 = P->size[0];
  r2 = P->size[0];
  k = P->size[0];
  loop_ub = P->size[0];
  i0 = b_P->size[0];
  b_P->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_P, i0, (int)sizeof(double));
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_P->data[i0] = P->data[i0];
  }

  emxInit_real_T(&c_P, 1);
  loop_ub = P->size[0];
  i0 = c_P->size[0];
  c_P->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_P, i0, (int)sizeof(double));
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_P->data[i0] = P->data[i0 + P->size[0]];
  }

  emxInit_real_T(&d_P, 1);
  loop_ub = P->size[0];
  i0 = d_P->size[0];
  d_P->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)d_P, i0, (int)sizeof(double));
  for (i0 = 0; i0 < loop_ub; i0++) {
    d_P->data[i0] = P->data[i0 + (P->size[0] << 1)];
  }

  emxFree_real_T(&P);
  emxInit_real_T1(&e_P, 2);
  i0 = e_P->size[0] * e_P->size[1];
  e_P->size[0] = r1;
  e_P->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)e_P, i0, (int)sizeof(double));
  for (i0 = 0; i0 < r1; i0++) {
    e_P->data[i0] = b_P->data[i0];
  }

  emxFree_real_T(&b_P);
  for (i0 = 0; i0 < r2; i0++) {
    e_P->data[i0 + e_P->size[0]] = c_P->data[i0];
  }

  emxFree_real_T(&c_P);
  for (i0 = 0; i0 < k; i0++) {
    e_P->data[i0 + (e_P->size[0] << 1)] = d_P->data[i0];
  }

  emxFree_real_T(&d_P);
  for (i0 = 0; i0 < 4; i0++) {
    D[i0] = 10000.0;
  }

  for (s = 0; s < laneModel->size[0]; s++) {
    a21 = e_P->data[s] - x_measure;
    a22 = e_P->data[s + e_P->size[0]] - y_measure;
    dist_point = sqrt(a21 * a21 + a22 * a22);
    if (dist_point < D[2]) {
      D[0] = 1.0 + (double)s;
      D[1] = 0.0;
      D[2] = dist_point;
      a[0] = (unsigned int)laneModel->size[0];
      a[1] = s + 2U;
      r1 = (int)(unsigned int)a[0] - 1;
      if ((unsigned int)a[1] < (unsigned int)a[0]) {
        r1 = (int)(unsigned int)a[1] - 1;
      }

      if (laneModel->data[r1] < 0.0) {
        a21 = -1.0;
      } else if (laneModel->data[r1] > 0.0) {
        a21 = 1.0;
      } else if (laneModel->data[r1] == 0.0) {
        a21 = 0.0;
      } else {
        a21 = laneModel->data[r1];
      }

      D[3] = -a21;

      //  positive Krümmung -> vorzeichen von d negativ (und anders rum)
    }

    if (1 + s > 1) {
      a[0] = e_P->data[s - 1];
      a[1] = e_P->data[(s + e_P->size[0]) - 1];
      b_x_measure[0] = x_measure;
      b_x_measure[1] = y_measure;

      // Abstand d zwischen der Gerade g (von P in Richtung Q) und dem Punkt M
      // S ist der Punkt auf g mit kleinstem Abstand zu M
      f_P[0] = e_P->data[s];
      f_P[1] = e_P->data[s + e_P->size[0]];
      for (i0 = 0; i0 < 2; i0++) {
        v[i0] = f_P[i0] - a[i0];
      }

      lambda = -(v[0] * (a[0] - x_measure) + v[1] * (a[1] - y_measure)) / (v[0] *
        v[0] + v[1] * v[1]);

      // Kreuzprodukt für Vorzeichen von d
      for (i0 = 0; i0 < 2; i0++) {
        b_v = a[i0] + lambda * v[i0];
        b_a[i0] = b_v - a[i0];
        b[i0] = b_x_measure[i0] - b_v;
        v[i0] = b_v;
      }

      cp_idx_2 = b_a[0] * b[1] - b_a[1] * b[0];
      a21 = x_measure - v[0];
      a22 = y_measure - v[1];
      dist_line = sqrt(a21 * a21 + a22 * a22);
      if ((dist_line < D[2]) && (lambda > 0.0) && (lambda < 1.0)) {
        D[0] = (1.0 + (double)s) - 1.0;
        D[1] = lambda;
        D[2] = dist_line;
        if (cp_idx_2 < 0.0) {
          D[3] = -1.0;
        } else if (cp_idx_2 > 0.0) {
          D[3] = 1.0;
        } else if (cp_idx_2 == 0.0) {
          D[3] = 0.0;
        } else {
          D[3] = cp_idx_2;
        }

        //  vorzeichen von d
      }
    }
  }

  emxFree_real_T(&e_P);
  y_measure = D[3] * D[2];
  x_measure = ((D[0] + D[1]) - 1.0) * lanePieceLength;

  //  initialize the state vector on first object detection
  if (init == 1) {
    r[0] = x_measure;
    r[1] = 0.0;
    r[2] = y_measure;
    r[3] = 0.0;
  } else {
    // % state space model matrices
    //  covariance of the state transition
    d = Q * Q;

    // % Kalman filter
    //  prediction
    for (i0 = 0; i0 < 4; i0++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 4; r1++) {
        a21 += d_a[i0 + (r1 << 2)] * r[r1];
      }

      c_a[i0] = a21 + (double)e_a[i0] * delta_x;
    }

    for (i0 = 0; i0 < 4; i0++) {
      r[i0] = c_a[i0];
      for (r1 = 0; r1 < 4; r1++) {
        f_a[i0 + (r1 << 2)] = 0.0;
        for (r2 = 0; r2 < 4; r2++) {
          f_a[i0 + (r1 << 2)] += d_a[i0 + (r2 << 2)] * Pk[r2 + (r1 << 2)];
        }
      }

      for (r1 = 0; r1 < 4; r1++) {
        g_a[i0 + (r1 << 2)] = 0.0;
        for (r2 = 0; r2 < 4; r2++) {
          g_a[i0 + (r1 << 2)] += f_a[i0 + (r2 << 2)] * b_b[r2 + (r1 << 2)];
        }
      }
    }

    b_d[0] = d;
    b_d[4] = d * 0.01;
    b_d[8] = 0.0;
    b_d[12] = 0.0;
    b_d[1] = d * 0.01;
    b_d[5] = d * 0.0001;
    b_d[9] = 0.0;
    b_d[13] = 0.0;
    b_d[2] = 0.0;
    b_d[6] = 0.0;
    b_d[10] = d;
    b_d[14] = d * 0.01;
    b_d[3] = 0.0;
    b_d[7] = 0.0;
    b_d[11] = d * 0.01;
    b_d[15] = d * 0.0001;
    for (i0 = 0; i0 < 4; i0++) {
      for (r1 = 0; r1 < 4; r1++) {
        Pk[r1 + (i0 << 2)] = g_a[r1 + (i0 << 2)] + b_d[r1 + (i0 << 2)];
      }
    }

    //  kalman gain
    for (i0 = 0; i0 < 2; i0++) {
      for (r1 = 0; r1 < 4; r1++) {
        h_a[i0 + (r1 << 1)] = 0.0;
        for (r2 = 0; r2 < 4; r2++) {
          h_a[i0 + (r1 << 1)] += (double)i_a[i0 + (r2 << 1)] * Pk[r2 + (r1 << 2)];
        }
      }

      for (r1 = 0; r1 < 2; r1++) {
        c_a[i0 + (r1 << 1)] = 0.0;
        for (r2 = 0; r2 < 4; r2++) {
          c_a[i0 + (r1 << 1)] += h_a[i0 + (r2 << 1)] * (double)c_b[r2 + (r1 << 2)];
        }
      }
    }

    b_Rx[0] = Rx * Rx;
    b_Rx[2] = 0.0;
    b_Rx[1] = 0.0;
    b_Rx[3] = Ry * Ry;
    for (i0 = 0; i0 < 2; i0++) {
      for (r1 = 0; r1 < 2; r1++) {
        S[r1 + (i0 << 1)] = c_a[r1 + (i0 << 1)] + b_Rx[r1 + (i0 << 1)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (r1 = 0; r1 < 2; r1++) {
        y[i0 + (r1 << 2)] = 0.0;
        for (r2 = 0; r2 < 4; r2++) {
          y[i0 + (r1 << 2)] += Pk[i0 + (r2 << 2)] * (double)c_b[r2 + (r1 << 2)];
        }
      }
    }

    if (fabs(S[1]) > fabs(S[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }

    a21 = S[r2] / S[r1];
    a22 = S[2 + r2] - a21 * S[2 + r1];
    for (k = 0; k < 4; k++) {
      K[k + (r1 << 2)] = y[k] / S[r1];
      K[k + (r2 << 2)] = (y[4 + k] - K[k + (r1 << 2)] * S[2 + r1]) / a22;
      K[k + (r1 << 2)] -= K[k + (r2 << 2)] * a21;
    }

    //  update
    if (hasMeasurement != 0) {
      c_x_measure[0] = x_measure;
      c_x_measure[1] = y_measure;
      for (i0 = 0; i0 < 2; i0++) {
        a[i0] = 0.0;
        for (r1 = 0; r1 < 4; r1++) {
          a[i0] += (double)i_a[i0 + (r1 << 1)] * r[r1];
        }

        b_x_measure[i0] = c_x_measure[i0] - a[i0];
      }

      for (i0 = 0; i0 < 4; i0++) {
        a21 = 0.0;
        for (r1 = 0; r1 < 2; r1++) {
          a21 += K[i0 + (r1 << 2)] * b_x_measure[r1];
        }

        r[i0] += a21;
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (r1 = 0; r1 < 2; r1++) {
        h_a[i0 + (r1 << 2)] = 0.0;
        for (r2 = 0; r2 < 2; r2++) {
          h_a[i0 + (r1 << 2)] += K[i0 + (r2 << 2)] * S[r2 + (r1 << 1)];
        }
      }

      for (r1 = 0; r1 < 4; r1++) {
        a21 = 0.0;
        for (r2 = 0; r2 < 2; r2++) {
          a21 += h_a[i0 + (r2 << 2)] * K[r1 + (r2 << 2)];
        }

        Pk[i0 + (r1 << 2)] -= a21;
      }
    }
  }
}

//
// File trailer for objectTracker.cpp
//
// [EOF]
//
