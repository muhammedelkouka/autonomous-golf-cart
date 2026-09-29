/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekf_fusion_codegen_mex.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 04-Apr-2026 21:31:31
 */

#ifndef _CODER_EKF_FUSION_CODEGEN_MEX_H
#define _CODER_EKF_FUSION_CODEGEN_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_ekf_fusion_codegen_mexFunction(int32_T nlhs, mxArray *plhs[2],
                                           int32_T nrhs,
                                           const mxArray *prhs[7]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_ekf_fusion_codegen_mex.h
 *
 * [EOF]
 */
