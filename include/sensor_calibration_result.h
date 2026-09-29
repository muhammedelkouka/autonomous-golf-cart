// sensor_calibration_result.h
// TEMPLATE — run sensor_calibration.exe to auto-fill real values.
// Place in: include/sensor_calibration_result.h
#pragma once

// ---- IMU Frame Notes ----
// Physical IMU: Y-forward, X-right, Z-up  (you physically rotated it)
// Correct rot_imu() for this orientation is IDENTITY:
//   static void rot_imu(const Vector3& in, Vector3& out) { out = in; }
// Your current rot_imu (out.x=in.y; out.y=-in.x; out.z=in.z) is WRONG for rotated IMU.
// It was correct when IMU was in DEFAULT orientation (X-forward).
// FIX: replace rot_imu in main.cpp with identity (see above).

static const float CALIB_IMU_ACC_SIGN_X  =  1.0f;
static const float CALIB_IMU_ACC_SIGN_Y  =  1.0f;
static const float CALIB_IMU_ACC_SIGN_Z  =  1.0f;
static const float CALIB_IMU_GYRO_SIGN_Z = +1.0f;  // no flip needed: left-turn (CCW) = positive, same as LiDAR/EKF frame
static const float CALIB_IMU_YAW_SIGN    = +1.0f;  // no flip needed: IMU yaw direction already matches LiDAR frame (CCW positive)
static const float CALIB_GYRO_BIAS_Z     =  0.0f;  // filled by calibration tool

// ---- LiDAR ----
static const float CALIB_LIDAR_Z_OFFSET      = 0.0f;   // mount height (optional correction)
static const float CALIB_LIDAR_VIBRATION_TOL = 0.05f;  // filled by calibration tool (3-sigma)
static const bool  CALIB_LIDAR_Y_IS_FORWARD  = true;