/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekf_fusion_codegen_api.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 04-Apr-2026 21:31:31
 */

#ifndef _CODER_EKF_FUSION_CODEGEN_API_H
#define _CODER_EKF_FUSION_CODEGEN_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void ekf_fusion_codegen(real_T x[8], real_T P[64], real_T acc_clean[3],
                        real_T gyro_z, real_T dt, real_T Q[64], real_T alpha,
                        real_T x_new[8], real_T P_new[64]);

void ekf_fusion_codegen_api(const mxArray *const prhs[7], int32_T nlhs,
                            const mxArray *plhs[2]);

void ekf_fusion_codegen_atexit(void);

void ekf_fusion_codegen_initialize(void);

void ekf_fusion_codegen_terminate(void);

void ekf_fusion_codegen_xil_shutdown(void);

void ekf_fusion_codegen_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_ekf_fusion_codegen_api.h
 *
 * [EOF]
 */
