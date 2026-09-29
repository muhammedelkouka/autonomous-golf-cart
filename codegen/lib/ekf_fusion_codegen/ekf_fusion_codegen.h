/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf_fusion_codegen.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 04-Apr-2026 21:31:31
 */

#ifndef EKF_FUSION_CODEGEN_H
#define EKF_FUSION_CODEGEN_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ekf_fusion_codegen(const double x[8], const double P[64],
                               const double acc_clean[3], double gyro_z,
                               double dt, const double Q[64], double alpha,
                               double x_new[8], double P_new[64]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ekf_fusion_codegen.h
 *
 * [EOF]
 */
