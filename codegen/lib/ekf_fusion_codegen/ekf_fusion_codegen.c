/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf_fusion_codegen.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 04-Apr-2026 21:31:31
 */

/* Include Files */
#include "ekf_fusion_codegen.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * --- Bias-compensated, clamped accelerations ---
 *
 * Arguments    : const double x[8]
 *                const double P[64]
 *                const double acc_clean[3]
 *                double gyro_z
 *                double dt
 *                const double Q[64]
 *                double alpha
 *                double x_new[8]
 *                double P_new[64]
 * Return Type  : void
 */
void ekf_fusion_codegen(const double x[8], const double P[64],
                        const double acc_clean[3], double gyro_z, double dt,
                        const double Q[64], double alpha, double x_new[8],
                        double P_new[64])
{
  static const double y[64] = {1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               1.0E-12};
  static const signed char b[64] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  double F[64];
  double b_F[64];
  double ax;
  double ay;
  int P_new_tmp;
  int i;
  int i1;
  int i2;
  /*  ==========================================================================
   */
  /*   ekf_fusion_codegen.m */
  /*   Complete EKF module extracted from fusion_v33.m — MATLAB Coder ready. */
  /*  */
  /*   Contains FOUR entry-point functions (each independently compilable): */
  /*  */
  /*     1. ekf_predict   — IMU-driven state propagation  (predict step) */
  /*     2. ekf_update    — NDT-measurement fusion with Mahalanobis gate */
  /*                        (full update step — was inline in the main loop) */
  /*     3. ekf_zupt      — Zero-velocity update when robot is stationary */
  /*     4. ekf_init      — Returns the initial state vector and covariance */
  /*                        (seeding from first valid NDT pose) */
  /*  */
  /*   STATE VECTOR  x  (8×1): */
  /*     x(1)  — position   X        [m] */
  /*     x(2)  — position   Y        [m] */
  /*     x(3)  — yaw angle           [rad]   wrapped to (−π, π] */
  /*     x(4)  — velocity   vx       [m/s] */
  /*     x(5)  — velocity   vy       [m/s] */
  /*     x(6)  — yaw rate   ω        [rad/s] low-pass filtered (v33 FIX 2) */
  /*     x(7)  — accel bias bx       [m/s²] */
  /*     x(8)  — accel bias by       [m/s²] */
  /*  */
  /*   FIXED NOISE MATRICES (match fusion_v33.m §5f exactly): */
  /*     Q          = diag([1e-5, 1e-5, 1e-6, 0.02, 0.02, 0.01, 5e-5, 5e-5]) */
  /*     R_ndt_good = diag([0.10, 0.10, 0.03])   (rmse < 1.0 m) */
  /*     R_ndt_mid  = diag([0.25, 0.25, 0.08])   (rmse < 3.0 m) */
  /*     R_ndt_poor = diag([0.60, 0.60, 0.20])   (rmse ≥ 3.0 m) */
  /*     R_zupt     = diag([0.02, 0.02]) */
  /*  */
  /*   HOW TO GENERATE C CODE (run in MATLAB Command Window): */
  /*  */
  /*     cfg = coder.config('lib'); */
  /*     cfg.TargetLang = 'C'; */
  /*  */
  /*     % --- ekf_predict --- */
  /*     codegen ekf_predict ... */
  /*         -args {zeros(8,1), zeros(8,8), zeros(3,1), 0, 0, zeros(8,8), 0} ...
   */
  /*         -config cfg -o ekf_predict_lib */
  /*  */
  /*     % --- ekf_update --- */
  /*     codegen ekf_update ... */
  /*         -args {zeros(8,1), zeros(8,8), zeros(3,1), 0, 0, zeros(2,1), 0, 0,
   * false} ... */
  /*         -config cfg -o ekf_update_lib */
  /*  */
  /*     % --- ekf_zupt --- */
  /*     codegen ekf_zupt ... */
  /*         -args {zeros(8,1), zeros(8,8), zeros(2,2)} ... */
  /*         -config cfg -o ekf_zupt_lib */
  /*  */
  /*     % --- ekf_init --- */
  /*     codegen ekf_init ... */
  /*         -args {zeros(3,1), zeros(2,1)} ... */
  /*         -config cfg -o ekf_init_lib */
  /*  */
  /*  ==========================================================================
   */
  /*  --------------------------------------------------------------------------
   */
  /*   FUNCTION 1 — ekf_predict */
  /*  */
  /*   Propagates state and covariance forward by one IMU time-step. */
  /*  */
  /*   Inputs */
  /*     x         (8×1)  current state vector */
  /*     P         (8×8)  current covariance matrix */
  /*     acc_clean (3×1)  accelerometer reading with gravity already removed
   * [m/s²] */
  /*                      (gravity vector must be subtracted by the caller) */
  /*     gyro_z    (1×1)  yaw-rate from gyroscope [rad/s] */
  /*                      (calibrated bias must be subtracted by the caller) */
  /*     dt        (1×1)  elapsed time since last call [s] */
  /*     Q         (8×8)  process-noise covariance  (pass the constant defined
   * above) */
  /*     alpha     (1×1)  low-pass gain for yaw-rate state, 0 < alpha ≤ 1 */
  /*                      0.3 recommended (v33 FIX 2); 1.0 = old
   * direct-overwrite */
  /*  */
  /*   Outputs */
  /*     x_new     (8×1)  predicted state */
  /*     P_new     (8×8)  predicted covariance  (symmetrised + nugget) */
  /*  --------------------------------------------------------------------------
   */
  ax = fmax(fmin(acc_clean[0] - x[6], 5.0), -5.0);
  /*  clamp ±5 m/s² */
  ay = fmax(fmin(acc_clean[1] - x[7], 5.0), -5.0);
  /*  --- State propagation (constant-acceleration kinematics) --- */
  memcpy(&x_new[0], &x[0], 8U * sizeof(double));
  x_new[0] = (x[0] + x[3] * dt) + 0.5 * ax * dt * dt;
  x_new[1] = (x[1] + x[4] * dt) + 0.5 * ay * dt * dt;
  x_new[2] = x[2] + x[5] * dt;
  x_new[3] = x[3] + ax * dt;
  x_new[4] = x[4] + ay * dt;
  /*  FIX 2 (v33): low-pass blend instead of direct overwrite */
  /*  Keeps yaw-rate estimate smooth across noisy gyro samples. */
  x_new[5] = (1.0 - alpha) * x[5] + alpha * gyro_z;
  /*  Bias states are modelled as random-walk — no explicit propagation. */
  /*  x_new(7) = x(7);  (identity, left unchanged) */
  /*  x_new(8) = x(8); */
  /*  Wrap yaw to (−π, π] */
  x_new[2] = b_atan2(sin(x_new[2]), cos(x_new[2]));
  /*  --- Linearised state-transition Jacobian F (8×8) --- */
  for (i = 0; i < 64; i++) {
    F[i] = b[i];
  }
  F[24] = dt;
  /*  ∂X/∂vx */
  F[33] = dt;
  /*  ∂Y/∂vy */
  F[42] = dt;
  /*  ∂yaw/∂ω */
  F[51] = -dt;
  /*  ∂vx/∂bx  (bias correction) */
  F[60] = -dt;
  /*  ∂vy/∂by */
  ax = -0.5 * dt * dt;
  F[48] = ax;
  /*  ∂X/∂bx   (second-order bias effect) */
  F[57] = ax;
  /*  ∂Y/∂by */
  F[45] = 1.0 - alpha;
  /*  ∂ω_new/∂ω_old  (LP memory term) */
  /*  --- Covariance propagation --- */
  memset(&b_F[0], 0, sizeof(double) << 6);
  for (i = 0; i < 8; i++) {
    P_new_tmp = i << 3;
    for (i1 = 0; i1 < 8; i1++) {
      __m128d r;
      __m128d r1;
      __m128d r2;
      int i3;
      i3 = i1 << 3;
      r = _mm_loadu_pd(&F[i3]);
      r1 = _mm_loadu_pd(&b_F[P_new_tmp]);
      r2 = _mm_set1_pd(P[i1 + P_new_tmp]);
      _mm_storeu_pd(&b_F[P_new_tmp], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&F[i3 + 2]);
      r1 = _mm_loadu_pd(&b_F[P_new_tmp + 2]);
      _mm_storeu_pd(&b_F[P_new_tmp + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&F[i3 + 4]);
      r1 = _mm_loadu_pd(&b_F[P_new_tmp + 4]);
      _mm_storeu_pd(&b_F[P_new_tmp + 4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&F[i3 + 6]);
      r1 = _mm_loadu_pd(&b_F[P_new_tmp + 6]);
      _mm_storeu_pd(&b_F[P_new_tmp + 6], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i1 = 0; i1 < 8; i1++) {
    for (i2 = 0; i2 < 8; i2++) {
      ax = 0.0;
      for (i = 0; i < 8; i++) {
        P_new_tmp = i << 3;
        ax += b_F[i1 + P_new_tmp] * F[i2 + P_new_tmp];
      }
      P_new_tmp = i1 + (i2 << 3);
      P_new[P_new_tmp] = ax + Q[P_new_tmp];
    }
  }
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      P_new_tmp = i1 + (i << 3);
      F[P_new_tmp] =
          (P_new[P_new_tmp] + P_new[i + (i1 << 3)]) * 0.5 + y[P_new_tmp];
    }
  }
  memcpy(&P_new[0], &F[0], 64U * sizeof(double));
  /*  symmetrise + nugget */
}

/*
 * File trailer for ekf_fusion_codegen.c
 *
 * [EOF]
 */
