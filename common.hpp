#pragma once

// =============================================================================
// common.hpp  —  Shared types, constants, globals, math helpers
// All modules include this file. Only one .cpp (main.cpp) must also include
// the Windows socket headers before including this.
// =============================================================================

#ifndef UNICODE
#define UNICODE
#endif
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define NOGDI
#define NOUSER

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#undef near
#undef far
#undef Rectangle
#undef CloseWindow
#undef ShowCursor
#undef DrawText
#undef DrawTextEx

#include "raylib.h"
#include "raymath.h"
#include "sensor_calibration_result.h"

extern "C" {
#include "ekf_fusion_codegen.h"
#include "ekf_fusion_codegen_initialize.h"
#include "ekf_fusion_codegen_terminate.h"
}

#include <array>
#include <vector>
#include <map>
#include <utility>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <random>
#include <queue>
#include <chrono>
#include <iomanip>
#include <deque>
#include <ctime>
#include <filesystem>

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")

namespace fs = std::filesystem;

// =============================================================================
//  SHARED CONFIGURATION
// =============================================================================

// --- Fusion / LiDAR ---
#define VLP16_PORT   2368
#define IMU_PORT     "\\\\.\\COM11"
#define IMU_BAUD     CBR_115200
#define MIN_DIST     0.8f
#define SCAN_Z_MIN   0.2f
#define SCAN_Z_MAX   3.0f
static const int   MIN_ABS_INLIERS   = 120;
static const float IMU_YAW_SIGN      = CALIB_IMU_YAW_SIGN;
static const float IMU_GYRO_SIGN     = CALIB_IMU_GYRO_SIGN_Z;
static const float IMU_YAW_OFFSET    = 0.0f;
static const double GYRO_RATE_LP     = 0.3;
static const double ZUPT_ACC_THRESH  = 0.28;
static const double ZUPT_GYRO_THRESH = 0.07;
static const int    ZUPT_PERSIST     = 4;
static const float  ROI_INDOOR_MAX   = 20.0f;
static const float  ROI_CAMPUS_FWD   = 50.0f;
static const float  ROI_CAMPUS_SIDE  = 25.0f;
static const float  ROI_FWD_HALF_ANG = 1.047f;
static const float  TURN_GYRO_THRESH = 0.15f;
static const float  RMSE_FULL_TRUST  = 0.20f;
static const float  RMSE_REJECT      = 0.40f;
static const float  SEED_RADIUS_INDOOR  = 4.0f;
static const float  SEED_RADIUS_OUTDOOR = 12.0f;
static const int    TRAJ_AUTO_SAVE_INTERVAL = 50;
static const std::string TRAJ_CSV_FILE  = "trajectory_data.csv";
static const std::string TRAJ_JSON_FILE = "trajectory_data.json";
static const std::string POSE_FILE_OUT  = "localized_pose.txt";

// --- Path Planning ---
static const std::string CSV_REF_FILE    = "reference_path.csv";
static const std::string CSV_OUTPUT_FILE = "smoothed_path.csv";
// Snapshot of the plan BEFORE static-obstacle avoidance shifts it (pp_save_plan(),
// written every time regardless of whether a detour actually occurred) — lets a
// before/after comparison be read directly off disk instead of inferred by diffing
// two separate drive runs.
static const std::string CSV_PRE_AVOID_FILE = "smoothed_path_pre_avoid.csv";
static const float CORRIDOR_WIDTH_M      = 3.5f;
static const float RESAMPLE_STEP_PP      = 0.05f;
static const float FILTER_MIN_QUALITY    = 0.30f;
static const int   SMOOTH_PASSES         = 40;
static const float SMOOTH_ALPHA          = 0.35f;
static const float DOWNSAMPLE_STEP_M     = 0.5f;

// --- Control / ESP32 ---
static const char*  ESP32_IP         = "10.63.129.200";
static const int    ESP32_CMD_PORT   = 4210;
static const int    LOCAL_TELEM_PORT = 4211;

static const float  LOOKAHEAD_MIN_M  = 2.5f;
static const float  LOOKAHEAD_NOM_M  = 3.2f;
static const float  LOOKAHEAD_MAX_M  = 4.0f;
static const float  WHEELBASE_M      = 2.4f;
static const float  MAX_STEER_DEG    = 23.0f;
static const float  CRUISE_VOLTAGE   = 0.99f;
static const float  MIN_VOLTAGE      = 0.95f;
// Higher cruise voltage used only while the cart is physically inside the Zone 2 area
// (g_zone2_poly) AND not dead-reckoning through the danger zone — the open zone-2
// stretch felt too slow at the normal 0.99V. Tune this single value to change it.
static const float  ZONE2_CRUISE_VOLTAGE = 1.2f;
// Slower, fixed cruise voltage used while dead-reckoning (danger zone OR the
// Danger-Zone-Fwd sub-zone) — 2026-07-07 field request. Overrides both the normal
// CRUISE_VOLTAGE and any ZONE2_CRUISE_VOLTAGE boost while active.
static const float  DANGER_ZONE_CRUISE_VOLTAGE = 0.85f;
// Reduced cruise voltage while a pedestrian is anywhere in the camera's lane zone
// (2026-07-27 — real near-miss during lab testing: the cart's brake is a plain
// throttle-cutoff + coast, confirmed no strong active deceleration, so it needs real
// lead distance to actually slow before the STOP-distance threshold matters). Applies
// as soon as g_ped_caution is true (person detected in-lane at ANY distance up to the
// camera's range), not just once they're within --stop-distance — see control.cpp.
// Starting value matches the already-field-tried DANGER_ZONE_CRUISE_VOLTAGE; untested
// at this specific value, tune from the first real pedestrian-caution test.
static const float  PED_CAUTION_CRUISE_VOLTAGE = 0.85f;

static const float  GOAL_DECEL_M     = 5.0f;
static const float  GOAL_RADIUS_M    = 2.0f;

static const float  STARTUP_BLIND_M      = 0.1f;
static const float  STARTUP_RAMP_M       = 0.2f;  // reduced — 0.5m was too long for short paths, 0.1 blind + 0.2 ramp = 0.3m total startup zone
static const float  STARTUP_MAX_STEER_DEG = 6.0f;  // slight reduction to keep startup gentle with shorter ramp zone

static const float  GOAL_STEER_CLAMP_M   = 3.0f;
static const float  GOAL_MAX_STEER_DEG   = 12.0f; // raised from 6° — 6° was insufficient to correct CTE when entering goal zone with offset
static const float  GOAL_MIN_STEER_DEG   = 3.0f;

static const float  STEER_SPEED_THRESH_DEG = 4.0f;
static const float  STEER_SPEED_MIN_FRAC   = 1.0f;

static const float  CTE_NOISE_THRESH_M      = 0.05f;
static const float  STEER_CMD_DEADBAND      = 0.10f;
static const float  STEER_RATE_LIM_DEG = 5.0f;

static const float  CORRIDOR_HALF_M    = 1.9f;
static const float  WATCHDOG_GRACE_M   = 1.8f;

// ── Obstacle/allowable-area grid (2026-07-10, test feature) ──────────────────
// World-frame cells (NOT image/screen space — this grid is used for planning
// later, so it must live in the same meters/EKF frame as the path). A cell only
// "exists" (is clickable/relevant) within GRID_BAND_M of the currently loaded
// reference path (g_smooth) — this is a corridor-following grid, not a full-map
// raster, so it stays cheap regardless of campus map size.
static const float  GRID_CELL_M = 0.60f;
static const float  GRID_BAND_M = 3.5f;   // reachable range for grid cells on each side of the path
static const float  GRID_PED_STOP_RADIUS_M = 3.0f;  // robot-to-cell distance that triggers a stop
enum ObstacleCellType { OBST_NONE=0, OBST_PEDESTRIAN=1, OBST_STATIC=2 };

// Step-2 static-obstacle avoidance (2026-07-19): a STATIC cell placed/removed while
// planning triggers an automatic replan that detours the FINAL per-plan path around it
// (see pp_apply_obstacle_avoidance in planning.cpp) — the shared reference corridor
// g_smooth is left untouched. PEDESTRIAN cells deliberately do NOT trigger any replan —
// a moving/unpredictable person is handled by the existing camera+grid STOP/auto-resume
// flow, not a detour. Dynamic (while-driving) re-routing is a later phase.
static const float  OBST_AVOID_CLEARANCE_M = 0.9f;  // min path clearance from a STATIC cell center
// Two obstacles whose blocked stretches are within this path arc-length of each other
// are merged into ONE combined detour instead of two independent ones — independent
// detours can pick opposite left/right directions and fight each other into a wavy
// path (confirmed visually 2026-07-19 with two close-together static cells).
static const float  OBST_MERGE_GAP_M       = 3.0f;
// Ramp-in/ramp-out length is NOT fixed — it's computed per-detour in
// pp_apply_obstacle_avoidance() from the cart's actual kinematics (WHEELBASE_M,
// MAX_STEER_DEG) and the shift magnitude, so the resulting curve stays within a
// physically achievable (and comfortable) turning radius rather than an arbitrary
// distance. A fixed 3m ramp was tried first and found to demand a ~1.1m turning
// radius for a typical shift — about 5x tighter than the cart's ~5.65m minimum.

static const float  CONTROL_HZ         = 50.0f;
static const float  POSE_WRITE_HZ      = 20.0f;

static const float  CTRL_ACTUATOR_DELAY_S   = 0.020f;
static const float  CTRL_PREDICTION_MAX_S   = 0.050f;
static const float  CTRL_PREDICTION_DELAY_S = CTRL_ACTUATOR_DELAY_S;

static const float  SIM_MAX_SPEED      = 2.0f;
static const int    TRAIL_MAX          = 2000;

// =============================================================================
//  COLOUR PALETTE
// =============================================================================
static const Color C_BG         = {12,  14,  20,  255};
static const Color C_PANEL      = {18,  22,  34,  230};
static const Color C_PANEL_DARK = {10,  12,  18,  245};
static const Color C_BORDER     = {45,  75, 130,  100};
static const Color C_BORDER_HI  = {55, 130, 220,  160};
static const Color C_HEADER     = {74, 179, 244,  255};
static const Color C_GOOD       = {57, 217, 138,  255};
static const Color C_WARN       = {245,166,  35,  255};
static const Color C_BAD        = {232, 64,  64,  255};
static const Color C_DIM        = { 80, 95, 115,  255};
static const Color C_WHITE      = {220,228, 240,  255};

// =============================================================================
//  APPLICATION STEP STATE
// =============================================================================
enum UiStep { STEP_LOCALIZE = 1, STEP_PLAN = 2, STEP_DRIVE = 3 };
extern std::atomic<int> g_ui_step;
extern std::atomic<bool> g_show_range_rings;  // [G] toggle for Step 1 range rings/crosshair

// =============================================================================
//  MATH CONSTANTS & HELPERS
// =============================================================================
static const float kPI = 3.14159265f;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static inline float toDeg(float r)  { return r * 180.f / kPI; }
static inline float toRad(float d)  { return d * kPI / 180.f; }
static inline float wrapA(float a)  { while(a > kPI) a -= 2*kPI; while(a < -kPI) a += 2*kPI; return a; }

static inline double nowSec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
static inline double nowSecEpoch() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

// =============================================================================
//  LOCALIZE PANEL BACKGROUND TRANSFORM
//  Maps localize-panel background image pixels ↔ world (map) coordinates.
//  Per-map: saved to localize_bg_<mapname>.cfg.
//  Developer calibrates once by overlaying the PLY cloud on the campus photo.
//  worldToImg / imgToWorld operate in image-pixel space; caller applies bscale.
// =============================================================================
struct LocalizeBGTransform {
    bool  valid      = false;
    float offset_x   = 0.f;   // image pixel x where world origin (0,0) appears
    float offset_y   = 0.f;   // image pixel y where world origin (0,0) appears
    float scale      = 8.f;   // image pixels per world meter
    float rotation   = 0.f;   // radians: map +X relative to image right (CCW positive)
    float cos_rot    = 1.f;
    float sin_rot    = 0.f;

    void setRotation(float rad){ rotation=rad; cos_rot=cosf(rad); sin_rot=sinf(rad); }

    // World (m) → image pixel (before cover-fit bscale)
    // Y is flipped to match w2s() (ui.cpp) — world +Y is "up" (north), image +Y is down.
    // Without this flip the cloud renders mirrored top-to-bottom vs. every other map view
    // (Step 1/2/3), so no amount of rotation can align it to the background photo.
    Vector2 worldToImg(float wx, float wy) const {
        return { offset_x + scale*(cos_rot*wx - sin_rot*wy),
                 offset_y - scale*(sin_rot*wx + cos_rot*wy) };
    }
    // Image pixel → world (m)
    Vector2 imgToWorld(float ix, float iy) const {
        float dix=ix-offset_x, diy=-(iy-offset_y);
        return { ( cos_rot*dix + sin_rot*diy)/scale,
                 (-sin_rot*dix + cos_rot*diy)/scale };
    }
};
extern LocalizeBGTransform g_localize_bg_tf;
extern bool g_localize_cal_mode;   // developer calibration overlay on/off
extern bool g_localize_cal_from_technical;   // entered calibration from the technical Step 1 panel, not the passenger flow
void load_localize_bg_transform_for_map();
void save_localize_bg_transform();

// =============================================================================
//  CORE DATA TYPES
// =============================================================================

struct Point3D   { float x, y, z; unsigned char intensity; };
struct MapPoint  { float x, y, z; };

struct TrajectoryPoint {
    float x, y, z, yaw, rmse, inlier_ratio;
    double timestamp; bool is_turning;
};

struct DistanceMap {
    float min_x=0, max_x=0, min_y=0, max_y=0, resolution=0.15f;
    int width=0, height=0;
    std::vector<float> grid;
    float get_dist(float x, float y) const {
        int ix = (int)((x-min_x)/resolution), iy = (int)((y-min_y)/resolution);
        if(ix < 0 || ix >= width || iy < 0 || iy >= height) return 2.f;
        return grid[iy*width+ix];
    }
};

struct MapEntry {
    std::string name, filename;
    std::vector<MapPoint> cloud;
    DistanceMap dist_map;
    bool loaded=false, is_large=false;
    float inlier_dist=0.40f, seed_accept=0.60f, track_accept=0.35f;
};

struct Particle    { float x, y, yaw, weight; };
struct RelocResult { float x, y, yaw, score; int abs_inliers; };

enum AppState { STATE_CALIBRATING, STATE_ALIGNMENT_CHECK, STATE_GLOBAL_RELOC, STATE_FUSION_RUNNING };

struct EkfState {
    double x[8]={}, P[64]={}, Q[64]={}, R_zupt[4]={};
    bool seeded=false;
    int zupt_counter=0;
};

struct LockedPose { float x=0, y=0, yaw=0; bool valid=false; };

// --- Path Planning ---
struct PPPoint  { float x, y, yaw, curvature; };
struct PPPose   { float x, y, yaw; bool valid; };

// --- Control ---
struct CtrlPathPoint { float x, y, yaw, curvature; };
struct CtrlPose      { float x, y, yaw; bool valid; };

enum CtrlState { CS_IDLE, CS_LOADED, CS_RUNNING, CS_PAUSED, CS_DONE, CS_WATCHDOG };

// ── Camera pedestrian detection (2026-07-08, test feature; stereo rewrite 2026-07-19) ──
// A separate Python detector (tools/pedestrian_detect.py) publishes a STOP/CLEAR
// status file; main.cpp reads it (throttled) and sets g_ped_detected. When
// g_ped_enabled and the cart is driving and a person is in the stop zone, the
// control loop pauses the cart (g_ped_paused). Auto-resumes once the trigger has
// stayed clear for pedResumeConfirmCount ticks (see control.cpp) — the operator's
// manual Resume button (ui.cpp) still exists as a fallback/override, it just rarely
// needs pressing now. Detector as of 2026-07-19: YOLOv8n (GPU) + calibrated stereo
// distance (two cameras, cross-camera disparity) — accurate to ~7cm in testing, stops
// at <=2m. Camera-only, no LiDAR.
extern std::atomic<bool>   g_ped_enabled;    // Drive-panel ON/OFF toggle
extern std::atomic<bool>   g_ped_detected;   // person currently in a stop zone (from Python)
extern std::atomic<bool>   g_ped_caution;    // person in-lane at ANY distance (2026-07-27) — slows the cart, see PED_CAUTION_CRUISE_VOLTAGE
extern std::atomic<bool>   g_ped_paused;     // control loop paused the cart for a pedestrian
extern std::atomic<bool>   g_ped_stale;      // detector enabled but not reporting (dead/slow)
void ped_set_enabled(bool on);               // main.cpp — launch/stop the Python detector
extern std::string PED_STATUS_FILE;          // absolute path both sides use

struct CtrlVars {
    bool  running=false, goalReached=false, watchdogTrip=false;
    float cmdSteer=0, cmdSteerRaw=0, cmdSpeed=0, crossTrackErr=0;
    int   lookaheadIdx=0;
    float distToGoal=9999, distTravelled=0;
    float startX=0, startY=0;
    float steerEMA=0, lastSentSteer=0;
    int   watchdogFrames=0;
    int   lastCI=0;
    float lastCte=0;
    int   centerRetryTicks=0;
    float steerRampTarget=0.f;
    float steerFiltered=0.f;
    float lastSentSpeed=0.f;
    float thetaActual=0.f;
    bool  thetaValid=false;
    int   startupTick=0;
    int   goalConfirmCount=0;
    bool  steerSentThisTick=false;
    double locFailSince=0.0;
    int   pedResumeConfirmCount=0;  // consecutive clear ticks before pedestrian auto-resume (see control.cpp)
    float imuRawDeg=0.f;   // snapshot of imu_angle.z taken each tick for logging
    float ekfYawDeg=0.f;   // snapshot of g_ctrl_robot.yaw in degrees for logging
};

struct PoseLPF {
    float x=0, y=0, yaw=0;
    bool  init=false;
    static constexpr float ALPHA_XY         = 0.70f;
    static constexpr float ALPHA_XY_TURNING = 0.35f;  // lower alpha during turns — scan is motion-distorted
    static constexpr float ALPHA_YAW_SLOW   = 0.10f;  // was 0.55 — slow filter prevents scan-match heading drift feeding PP
    static constexpr float ALPHA_YAW_FAST   = 0.45f;  // was 0.85 — for jumps >2°
    static constexpr float ALPHA_YAW        = 0.65f;

    CtrlPose update(float rx, float ry, float ryaw, bool turning=false) {
        if(!init){ x=rx; y=ry; yaw=ryaw; init=true; }
        float axy = turning ? ALPHA_XY_TURNING : ALPHA_XY;
        x   = axy * rx   + (1.f-axy)  * x;
        y   = axy * ry   + (1.f-axy)  * y;
        float dy2 = wrapA(ryaw - yaw);
        float alpha_y = (fabsf(dy2) > toRad(2.0f)) ? ALPHA_YAW_FAST : ALPHA_YAW_SLOW;
        yaw = wrapA(yaw + alpha_y * dy2);
        return {x, y, yaw, true};
    }
    void reset(){ init=false; }
};

// UDP performance logger row
struct LogRow {
    double  time_s;
    double  pc_send_time_s;
    double  esp_ack_time_s;
    float   udp_rtt_ms;
    bool    udp_dropped;
    int     send_seq;
    int     ack_seq;
    float   steer_cmd_deg;
    float   steer_actual_deg;
    float   steer_raw_deg;       // Pure Pursuit output before EMA
    float   steer_filtered_deg;  // after EMA, before send gate
    float   steer_delta_deg;     // delta vs lastSentSteer (what triggered send)
    int     steer_sent;          // 1 if a STEER command was sent this tick, 0 if not
    int     tick_index;          // control loop tick counter since run start
    float   speed_cmd_v;
    float   speed_actual_v;
    float   wheel_mps;            // actual wheel speed from Hall sensor (0 if not yet received)
    float   cte_m;
    float   lookahead_m;
    float   dist_to_goal_m;
    float   robot_x, robot_y, robot_yaw_deg;
    int     path_index;
    double  lidar_timestamp_s;
    double  ctrl_timestamp_s;
    float   pipeline_delay_ms;
    float   imu_raw_deg;    // raw IMU angle.z — absolute heading from IMU internal fusion (degrees)
    float   ekf_yaw_deg;    // EKF/scan-match heading before Fix 2 substitution (degrees)
    // Planned path reference (for planned-vs-actual comparison)
    float   path_x;         // nearest planned path point x (at path_index)
    float   path_y;         // nearest planned path point y
    float   path_yaw_deg;   // planned heading at nearest path point (CW-positive — planning.cpp convention)
    float   lookahead_x;    // lookahead target point x (what PP is steering toward)
    float   lookahead_y;    // lookahead target point y
    float   scan_rmse;          // scan matcher RMSE at this tick (m) — quality of localization
    float   scan_inlier_ratio;  // scan matcher inlier ratio at this tick — quality of localization
    // Pedestrian detection + danger-zone crossing state (2026-07-28) — added so a
    // drive log is self-describing for post-run analysis instead of requiring
    // manual cross-referencing against eststart_*.cfg mtimes / coordinate ranges.
    int     ped_enabled;    // camera pedestrian detector ON/OFF this tick (g_ped_enabled)
    int     ped_stop;       // camera STOP trigger active (g_ped_detected)
    int     ped_caution;    // camera CAUTION (in-lane, pre-slowdown) active (g_ped_caution)
    int     grid_ped_stop;  // obstacle-grid manual pedestrian-cell stop active (g_grid_ped_stop)
    int     dzx_state;      // DzxState enum value — which leg of a danger-zone crossing, if any
    std::string map_name;   // active_map_ptr->name at this tick
};

struct CalibData {
    std::atomic<bool> is_calibrating{true};
    std::atomic<int>  samples{0};
    double sum_acc[3]={}, sum_gyro[3]={};
};

// =============================================================================
//  SHARED GLOBALS  (defined in main.cpp, declared extern everywhere else)
// =============================================================================

// EKF / fusion
extern EkfState         ekf;
extern std::mutex       ekf_mutex;

// LiDAR
extern std::vector<Point3D>  dense_cloud;
extern std::mutex            lidar_mutex;
extern std::atomic<double>   g_lidar_sweep_time;
extern std::atomic<int>      lidar_sweeps_total;
extern std::atomic<int>      imu_frames_total;

// IMU
extern std::mutex  imu_mutex;
extern Vector3     imu_acc, imu_gyro, imu_angle;
extern float       g_imu_yaw_at_localize;
extern float       g_yaw_imu_lidar_offset_rad;
extern float       g_localize_yaw_rad;  // particle-filter heading at localization confirm (radians, map frame)
extern CalibData   calib_state;
extern Vector3     g_to_subtract, gyro_bias_init;

// Reloc
extern RelocResult           g_reloc;
extern bool                  g_reloc_done, g_reloc_running;
extern float                 g_reloc_progress;
extern std::vector<Particle> g_reloc_particles;
extern std::mutex            g_reloc_mutex;
extern std::string           g_reloc_error_msg;

// Scan matcher
extern std::mutex            matcher_mutex;
extern std::vector<Point3D>  matcher_cloud;
extern std::atomic<bool>     matcher_request;

// Trajectory
extern std::vector<Vector3>         robot_trajectory;
extern std::vector<TrajectoryPoint> trajectory_data;
extern std::mutex                   trajectory_mutex;
extern std::atomic<float>  current_rmse;
extern std::atomic<float>  current_inlier_ratio;
extern std::atomic<bool>   g_is_turning;
extern std::atomic<bool>   g_is_stationary;

// Map
extern std::vector<MapEntry> map_library;
extern int                   active_map_index;
extern MapEntry*             active_map_ptr;

// Confirmed pose
extern LockedPose g_confirmed_start;

// Planning
extern std::vector<PPPoint>  g_smooth;
extern std::vector<PPPoint>  g_corridor_L, g_corridor_R;
extern std::vector<Vector2>  g_approach_path, g_corridor_path, g_goal_path;
extern PPPose g_pp_robot;
extern PPPose g_pp_goal;
extern std::atomic<bool>     g_path_smoothing_enabled;  // Step 2 toggle: ON = Laplacian+Catmull, OFF = raw recorded points
extern std::atomic<bool>     g_show_path_overlay;       // Step 2 toggle: show/hide reference-path corridor overlay (per-map path may be stale)

// ── Step 2 local path editing ──
// Select a stretch of the reference path (g_smooth) by two clicks, then nudge it
// perpendicular (left/right of travel) to pull it off a wall; ends taper smoothly.
// Edits apply live to g_smooth; "Save" writes it back to the active corner CSV.
extern bool  g_path_edit_mode;      // true while the Edit Path tool is active
extern int   g_path_edit_i0;        // first selected g_smooth index (-1 = unset)
extern int   g_path_edit_i1;        // second selected g_smooth index (-1 = unset)
extern float g_path_edit_offset;    // accumulated signed shift (m), for the readout
void pp_shift_path_segment(int lo, int hi, float delta_m);  // shift [lo,hi] sideways, taper ends
void pp_straighten_path_segment(int lo, int hi);            // replace [lo,hi] with a straight line between its endpoints
bool pp_save_edited_path();         // write g_smooth back to the active corner CSV

// ── Step 2 danger-zone selection (July 2026) ──
// Same two-click pattern as path editing, but searches BOTH zone-1 and zone-2's
// cached corridors regardless of which is currently armed — the danger stretch can
// straddle the zone-1/zone-2 boundary, and zone 1's corridor isn't loaded into
// g_smooth unless zone 1 happens to be the armed one. Each selected point records
// which corridor it snapped to (1 or 2) + its index within that corridor.
extern bool g_dz_edit_mode;    // true while the Set Danger Zone tool is active
extern int  g_dz_edit_zone0;   // corridor (1 or 2) of the first selected point (-1 = unset)
extern int  g_dz_edit_idx0;    // its index within that corridor
extern int  g_dz_edit_zone1;   // corridor (1 or 2) of the second selected point (-1 = unset)
extern int  g_dz_edit_idx1;    // its index within that corridor
// "Set Sub-Zone" tool (Danger Zone Fwd test map only) — see main.cpp click handler.
extern bool  g_dzsub_edit_mode;
extern bool  g_dzsub_edit_have0;
extern bool  g_dzsub_edit_have1;
extern float g_dzsub_edit_x0, g_dzsub_edit_y0;
extern float g_dzsub_edit_x1, g_dzsub_edit_y1;
// "Set Zone 2 Slow Segment" tool (top-LiDAR maps) — 2 clicks snapped to zone 2's
// recorded corridor (pp_zone_corridor(2), not g_smooth, so it works regardless of
// which zone is currently armed). See main.cpp click handler.
extern bool  g_z2slow_edit_mode;
extern bool  g_z2slow_edit_have0;
extern bool  g_z2slow_edit_have1;
extern float g_z2slow_edit_x0, g_z2slow_edit_y0;
extern float g_z2slow_edit_x1, g_z2slow_edit_y1;
// "Place Danger Path" tool — drag/rotate the old campus danger-zone curve into
// position on the new Danger Zone Fwd map, then splice it into g_smooth.
extern bool  g_dzplace_mode;
extern std::vector<PPPoint> g_dzplace_raw;
extern float g_dzplace_cx, g_dzplace_cy;
extern float g_dzplace_offset_x, g_dzplace_offset_y, g_dzplace_rotation;
bool load_dzplace_raw();                          // loads danger_zone_forward.csv into g_dzplace_raw
void dzplace_transformed_points(std::vector<PPPoint>& out);  // applies current offset/rotation
bool dzplace_save_into_smooth();                   // splices transformed curve into g_smooth + persists
// Nearest point to (x,y) across whichever zone corridors are cached for the active
// map; outZone is 1 or 2, outIdx is the index within that corridor. False if neither loaded.
bool pp_nearest_either_zone(float x, float y, int& outZone, int& outIdx, int avoidZone);
// World coords of corridor `zone`'s (1 or 2) point at `idx` — for drawing the selection.
bool pp_zone_corridor_point(int zone, int idx, float& x, float& y);
// Full corridor `zone`'s (1 or 2) points — for drawing both zones' curves at once
// while using the Set Danger Zone tool (the non-armed zone is otherwise invisible).
const std::vector<PPPoint>& pp_zone_corridor(int zone);
// Save the danger zone from two (zone,idx) picks — same corridor: a plain slice;
// different corridors: concatenated in actual recorded drive order for the active map.
bool pp_save_danger_zone(int zone0, int idx0, int zone1, int idx1);

// Zone-2 slow-speed segment (2026-07) — a sub-stretch of zone 2's recorded corridor
// where the ZONE2_CRUISE_VOLTAGE boost causes scan-matching to fail (found on the
// reverse map's zone 2). Two world-position markers (entry/exit) define it, set via
// Step 2's "Set Zone 2 Slow Segment" tool and persisted per-map to z2slow_<plyname>.cfg
// — only maps with a saved file actually have a segment, so this takes effect only
// where it's actually been configured (the reverse map), no map-name check needed.
extern bool  g_z2slow_valid;
extern float g_z2slow_entry_x, g_z2slow_entry_y;
extern float g_z2slow_exit_x,  g_z2slow_exit_y;
void load_z2slow_for_map();
void save_z2slow_for_map();
void clear_z2slow_for_map();
// True if (rx,ry) is within the marked slow segment (nearest-index membership between
// the entry/exit markers on zone 2's cached corridor). False if none is set for the
// active map. control.cpp uses this to suppress the zone-2 speed boost there.
bool pp_zone2_slow_active(float rx, float ry);

// Recorded-path file names (per-map; auto-loaded by active map in pp_run_pipeline)
static const std::string PATH_FORWARD_FILE = "path_forward.csv";
static const std::string PATH_REVERSE_FILE = "path_reverse.csv";
static const std::string PATH_FORWARD2_FILE = "path_forward2.csv";  // zone 2's forward corner
static const std::string PATH_REVERSE2_FILE = "path_reverse2.csv";  // zone 2's reverse corner
// Full continuous recorded drive (both corners + everything between). Used only for
// cross-zone routing, where a straight connector between two separately-cut corner
// corridors would cut across the un-recorded gap between them.
static const std::string PATH_FORWARD_FULL_FILE = "path_forward_full.csv";
static const std::string PATH_REVERSE_FULL_FILE = "path_reverse_full.csv";
// Danger-zone standalone test map (2026-07-07): its own recorded there-and-back
// trajectory split into the two driving directions. Auto-loaded by map name, same
// mechanism as the forward/reverse corner files (see pp_active_map_recorded_path).
static const std::string PATH_DANGERZONE_FWD_FILE = "path_dangerzone_fwd.csv";
static const std::string PATH_DANGERZONE_REV_FILE = "path_dangerzone_rev.csv";

// Intro/destination panel state
extern bool g_show_welcome;           // GIU branded welcome splash (shown first)
extern bool g_show_intro;
extern bool g_show_dest;
extern bool g_show_localize;          // passenger "find the cart" panel
extern bool g_show_map_select;        // passenger map-select question (near lab?)
extern bool g_show_journey_end;       // "Another journey?" card, shown on arrival (passenger flow only)
extern bool g_drive_via_passenger;    // true only if the current drive was started via the passenger flow

// Danger-zone CROSSING journey (2026-07-07) — see main.cpp for the full design comment.
enum DzxState { DZX_NONE=0, DZX_TO_CHECKPOINT, DZX_WAIT_ENTER_RELOC, DZX_IN_DZMAP, DZX_WAIT_EXIT_RELOC };
extern DzxState g_dzx_state;
extern float g_dzx_final_x, g_dzx_final_y;
extern std::string g_dzx_final_name;
extern bool g_dzx_pending_enter;
extern bool g_dzx_pending_exit;
// Name of the destination the user picked (any journey, crossing or direct) — the
// label on the friendly drive panel's destination pin. Cleared by pp_clear_path().
extern std::string g_journey_dest_name;
extern bool g_localize_request;       // panel → main: run global relocalization now
extern bool g_localize_panel_needs_init; // set true when panel becomes visible → reset circle
void switch_active_map_by_name(const std::string& name); // switch map + reload all per-map cfgs
extern float g_localize_seed_x;      // circle world position → particle-filter seed
extern float g_localize_seed_y;
extern float g_localize_seed_r;      // search radius (m)
extern bool  g_localize_use_custom_seed; // true → use seed_x/y/r, ignoring zone/EKF
extern bool g_intro_remote;
extern bool g_bg_show_3d;   // background: true=3D render, false=satellite

// Saved destinations (set interactively in Step 2, loaded at startup)
// NUM_DEST + g_dest_names give every destination a single map-independent
// identity: index i always means the same named place (e.g. "Labs") on every
// map — only its world (x,y) differs per map, from that map's dest_<ply>.cfg.
constexpr int NUM_DEST = 5;
extern const char* const g_dest_names[NUM_DEST];  // canonical names, shared across all maps
struct SavedDest { std::string name; float x=0.f,y=0.f; bool valid=false; };
extern std::array<SavedDest,NUM_DEST> g_destinations;
extern int  g_selected_dest;      // index chosen in destination panel (-1 = none)
// Destination setup (Step 2, "Set Destinations on Map") is a pick-preview-confirm flow:
// g_dest_setup_mode opens the picker list; picking a name (g_dest_setup_idx>=0) awaits a
// map click; the click sets g_pp_goal + previews the path (g_dest_setup_previewing=true);
// admin then Confirms (saves into g_destinations) or Retries/Cancels (nothing is saved
// until Confirm). g_dest_setup_prev_goal is the g_pp_goal snapshot restored on Cancel.
extern bool   g_dest_setup_mode;
extern int    g_dest_setup_idx;         // index being placed, -1 = showing the picker list
extern bool   g_dest_setup_previewing;  // true once a candidate point has a previewed path
extern PPPose g_dest_setup_prev_goal;   // g_pp_goal snapshot taken before placing began
void load_destinations(const char* path);
void save_destinations(const char* path);

// ── Zone 2's destinations — same identity/persistence pattern as above, own name
// list (zone 2 leads to physically different places than zone 1), own per-map file
// (dest2_<plyname>.cfg, no legacy-global-file fallback — zone 2 never had one).
// Indices 4,5 (Basketball Court, Food Court — 2026-07-07) are reached via the
// danger-zone-crossing route (Building A -> switch to Danger Zone Fwd -> switch back
// to Campus Top LiDAR — see g_dzx_state in main.cpp), NOT the normal zone-2 corner
// route the first 4 destinations use.
constexpr int NUM_DEST2 = 6;
extern const char* const g_dest_names2[NUM_DEST2];
extern std::array<SavedDest,NUM_DEST2> g_destinations2;
extern int    g_selected_dest2;
extern bool   g_dest_setup_mode2;
extern int    g_dest_setup_idx2;
extern bool   g_dest_setup_previewing2;
extern PPPose g_dest_setup_prev_goal2;
void load_destinations2(const char* path);
void save_destinations2(const char* path);

// Resolves a "combined" destination index — 0..NUM_DEST-1 means zone 1,
// NUM_DEST..NUM_DEST+NUM_DEST2-1 means zone 2 — into that destination's name and
// this map's world coords, and arms the zone it belongs to (so the corner-mode
// planner routes through the right corner). Returns false if the index is out of
// range or that destination isn't set for the active map. Used everywhere a stored
// int used to index g_destinations[] directly: g_pax_grid and g_dest_grid.
bool resolve_combined_dest(int combined_idx, const char*& name_out, SavedDest& out);
constexpr int NUM_DEST_TOTAL = NUM_DEST + NUM_DEST2;

// Intended visiting sequence across all NUM_DEST_TOTAL destinations (1-based rank,
// indexed by combined index). The actual "don't skip a stop" guarantee comes from
// pp_trigger_plan() always routing via the recorded corridor (never a straight-line
// shortcut) — since the corridor is one physically-driven, contiguous curve, following
// it from the robot's position to any destination's point on it necessarily passes
// every other destination's point that lies between them. This array documents the
// intended order (as given for the forward map) and is available for future use
// (e.g. UI display, validation) without itself driving routing.
// dest_order_rank() mirrors it for the reverse map: reverse-map driving visits stops
// in the opposite sequence, so rank_rev = NUM_DEST_TOTAL+1 - rank_fwd.
extern const int g_dest_order_fwd[NUM_DEST_TOTAL];  // rank per combined index, forward map
int dest_order_rank(int combined_idx);              // rank for the *active* map (fwd or mirrored)

// ── Localization zones (Phase 1: zone 1, per map) ──
// Drawn in Step 2 (2D view), saved per map to zone1_<plyname>.cfg.
// Selecting (arming) a zone in Step 1 focuses particle-filter seeding inside it;
// the user still presses Enter / Shift+Click to fire the reloc.
extern std::vector<Vector2> g_zone_poly;        // active map's zone-1 polygon (world x,y)
extern bool                 g_zone_valid;       // active map has a saved zone 1
extern bool                 g_zone_armed;       // zone 1 selected → seed reloc within it
extern bool                 g_zone_edit_mode;   // currently drawing zone 1 (Step 2)
extern std::vector<Vector2> g_zone_edit_verts;  // in-progress vertices while drawing
extern bool                 g_zone_focus_request; // set when arming → main loop centers view on the zone
std::string zone_file_for_map();
void load_zone_for_map();
void save_zone(const std::vector<Vector2>& verts);

// ── Localization zones — Phase 2: zone 2, per map ──
// A second, physically distinct corner/route on the same top-LiDAR maps. Mirrors the
// zone-1 fields/functions exactly (see above) — kept as an explicit parallel set rather
// than a generic Zone[] array so zone-1's hardware-validated code paths are only ever
// added to, never rewritten. Zone 1 and zone 2 are mutually exclusive: arming one always
// disarms the other (enforced at each arm site, not here).
extern std::vector<Vector2> g_zone2_poly;
extern bool                 g_zone2_valid;
extern bool                 g_zone2_armed;
extern bool                 g_zone2_edit_mode;
extern std::vector<Vector2> g_zone2_edit_verts;
extern bool                 g_zone2_focus_request;
std::string zone2_file_for_map();
void load_zone2_for_map();
void save_zone2(const std::vector<Vector2>& verts);

// ── Per-map estimated start pose (pre-localization) ──
// Where the cart roughly starts on this map. Used as the planning pose before
// localizing so previews begin at the real start, not the map origin.
extern PPPose g_est_start;       // x,y,yaw + valid
extern bool   g_est_start_mode;  // true while clicking the map to place it
std::string est_start_file_for_map();
void load_est_start_for_map();
void save_est_start();

// ── Destination grid cells (Step 2 tap-to-navigate) ──
// The map is divided into 5 m × 5 m world-space cells. Each cell can be
// painted as belonging to one of the saved destinations (zone 1 or zone 2). A
// normal user just clicks anywhere on a painted cell and the path generates
// immediately — no panel buttons or Ctrl+click needed.
constexpr float DEST_GRID_CELL_M = 5.0f;
// Key = (col, row) = (floor(wx/5), floor(wy/5)); value = combined dest_idx
// (0..NUM_DEST-1 = zone 1, NUM_DEST..NUM_DEST_TOTAL-1 = zone 2 — see resolve_combined_dest)
extern std::map<std::pair<int,int>, int> g_dest_grid;
extern bool g_dest_grid_edit;       // true while user is painting cells
extern int  g_dest_grid_paint_dest; // combined index 0..NUM_DEST_TOTAL-1 = paint that dest; -1 = eraser
void save_dest_grid_for_map();      // writes destgrid_<plyname>.cfg

// ── Obstacle/allowable-area grid (2026-07-10, test feature) ──────────────────
// Key = (col,row) = (floor(wx/GRID_CELL_M), floor(wy/GRID_CELL_M)), WORLD frame.
// Sparse: only cells actually clicked into a non-NONE state are stored. Runtime
// state only (not persisted to disk, not per-map) — real obstacles are
// transient; "Clear All" resets it. Step 1 scope only: mark + visualize +
// stop-for-pedestrian-cells. NOT yet consumed by the path planner (no
// inflation/avoidance — that's the next phase once obstacle scenarios are
// being tested).
extern std::map<std::pair<int,int>, ObstacleCellType> g_obstacle_grid;
extern bool g_obstacle_test_mode;   // true while clicking the map cycles cell state
extern std::atomic<bool> g_grid_ped_stop;  // true when the robot is near a PEDESTRIAN cell
                                            // (feeds the same pause/resume flow as the camera)
// Counts down from 3.5s in DrawMapViewport (ui.cpp) once set — drives a centred,
// fading "No path available" box in the Step 2/3 map panel (2026-07-19), same visual
// pattern as the passenger destination-unavailable message. Set by
// pp_apply_obstacle_avoidance() (planning.cpp) when a static obstacle has no valid
// detour within the safe corridor on either side.
extern float g_obst_blocked_msg_timer;
inline std::pair<int,int> grid_cell_of(float wx, float wy) {
    return {(int)floorf(wx / GRID_CELL_M), (int)floorf(wy / GRID_CELL_M)};
}
inline void grid_cell_center(int col, int row, float& cx, float& cy) {
    cx = (col + 0.5f) * GRID_CELL_M; cy = (row + 0.5f) * GRID_CELL_M;
}
// True if the cell (col,row) is within GRID_BAND_M of the currently loaded reference
// path (g_smooth) — defined in planning.cpp, where g_smooth lives.
bool grid_cell_in_band(int col, int row);
// Click handling: snaps to the nearest in-band cell to (wx,wy) and cycles its state
// NONE->PEDESTRIAN->STATIC->NONE. No-op if no cell is in band nearby.
void grid_click_cycle(float wx, float wy);
void grid_clear_all();
// Called from control.cpp each tick: true if the robot is within GRID_PED_STOP_RADIUS_M
// of any PEDESTRIAN-type cell.
bool grid_check_pedestrian_stop(float rx, float ry);

// ── Border/wall grid cells (2026-07-10) — auto-generated forbidden cells running
// along BOTH sides of the reference path at CORRIDOR_HALF_M (the watchdog-trip
// edge — past this the cart is hitting the wall). Separate from g_obstacle_grid
// (pedestrian/static, runtime-only test data): these are meant to be generated
// once per map, hand-adjusted, and SAVED per-map (bordercells_<plyname>.cfg).
// Same "step 1: mark it" scope as the obstacle grid — not yet consumed by the
// path planner, enforcement is a later phase.
extern std::map<std::pair<int,int>, bool> g_border_grid;   // true = forbidden border cell
extern bool g_border_edit_mode;   // true while clicking the map adds/erases a border cell
extern bool g_show_border_cells;  // 2026-07-27: draw/hide toggle, independent of the data itself
void border_auto_generate();      // populate from the current g_smooth, both sides at CORRIDOR_HALF_M
void border_click_toggle(float wx, float wy);  // add if absent, erase if present (nearest in-band cell)
void border_clear_all();
std::string border_file_for_map();
void save_border_grid();
void load_border_grid();

// ── Passenger destination panel zone grid ─────────────────────────────────
// FIXED grid of PAX_COLS × PAX_ROWS cells anchored to the campus IMAGE
// (normalized 0..1), NOT to screen pixels — so a cell always covers the same
// part of the campus no matter the window size. Admin paints each allowed
// cell with the destination (by combined index) it routes to; a passenger tap
// resolves the cell's index via resolve_combined_dest(), which also arms the
// correct zone, looks up that destination's world (x,y) for the currently
// active map, and plans directly there. Unpainted cell = not available.
// Stored in paxgrid.cfg (not per-map — the campus image is campus-wide; only
// the resolved world coords are per-map).
constexpr int PAX_COLS = 20;   // fixed columns across the image
constexpr int PAX_ROWS = 12;   // fixed rows down the image
// Grid cell values: 0..NUM_DEST_TOTAL-1 = a destination; PAX_UNREACHABLE = explicitly
// marked "the cart cannot reach here" (shows a specific passenger-facing message,
// distinct from an ordinary un-painted cell's generic "not available" message);
// absent from the map = un-painted (same generic message as before this existed).
constexpr int PAX_UNREACHABLE = -2;
extern std::map<std::pair<int,int>, int> g_pax_grid;  // value = combined dest_idx (0..NUM_DEST_TOTAL-1), or PAX_UNREACHABLE
extern bool g_pax_grid_edit;
extern int  g_pax_paint_dest;  // admin's current paint brush: 0..NUM_DEST_TOTAL-1, PAX_UNREACHABLE, or -1 = eraser
void save_pax_grid();
void load_pax_grid();

// Map segmentation (June 22, 2026) — polygon defines corridor region.
// Inside polygon → corridor mode (reference_path.csv).
// Outside polygon → free mode (direct Bézier from robot to goal).
extern std::vector<Vector2> g_segment_polygon;
extern bool                 g_segment_polygon_valid;
extern bool                 g_plan_free_mode;       // set each call to pp_trigger_plan()
extern bool                 g_plan_corner_mode;     // top-LiDAR corner maps: plan forced through the full recorded corner
extern bool                 g_polygon_edit_mode;
extern std::vector<Vector2> g_polygon_edit_verts;

static inline bool point_in_polygon(float px, float py, const std::vector<Vector2>& poly) {
    int n = (int)poly.size(); if(n < 3) return false;
    bool inside = false;
    for(int i = 0, j = n-1; i < n; j = i++) {
        if(((poly[i].y > py) != (poly[j].y > py)) &&
           (px < (poly[j].x-poly[i].x)*(py-poly[i].y)/(poly[j].y-poly[i].y)+poly[i].x))
            inside = !inside;
    }
    return inside;
}

// Centroid + covering radius (max vertex distance from centroid) of a zone polygon.
// Pure/read-only — shared by every zone-1 and zone-2 site that needs to seed
// localization or center the camera on a zone. Returns false (unchanged cx/cy/r) if
// the polygon has fewer than 3 vertices.
static inline bool zone_centroid_radius(const std::vector<Vector2>& poly, float& cx, float& cy, float& r) {
    if(poly.size() < 3) return false;
    cx = 0.f; cy = 0.f;
    for(auto& v : poly){ cx += v.x; cy += v.y; }
    cx /= (float)poly.size(); cy /= (float)poly.size();
    r = 0.f;
    for(auto& v : poly){
        float d = sqrtf((v.x-cx)*(v.x-cx) + (v.y-cy)*(v.y-cy));
        if(d > r) r = d;
    }
    return true;
}

void load_segment_polygon(const char* path);
void save_segment_polygon(const char* path);

// Known DANGER ZONE (July 2026): the stretch from "Building A" through the
// zone-1/zone-2 boundary to "Basketball Court Curve" has sparse/ambiguous features —
// scan-matching there converges wrong or loses lock, confirmed even stationary and
// under manual driving. danger_zone_forward.csv / danger_zone_reverse.csv (built
// once from the already-validated zone1/zone2 recordings) give the exact curve
// shape, which control.cpp's ctrl_splice_danger_zone() splices directly into the
// driven plan — the spliced range IS the danger zone (see dz_nearest/dz_lookup in
// planning.cpp and their use in control.cpp ctrl_step()).
// End map segmentation (June 22, 2026)

// Control
extern std::vector<CtrlPathPoint> g_ctrl_path;
extern CtrlPose  g_ctrl_start;
extern CtrlPose  g_ctrl_goal;
extern CtrlPose  g_ctrl_robot;
extern CtrlPose  g_ctrl_robot_filtered;
extern CtrlPose  g_ctrl_sim;
extern float     g_ctrl_simSpd;
extern float     g_lookaheadM;
extern float     g_dbg_dt_lidar_ms;
extern float     g_dbg_dt_actuator_ms;
extern float     g_dbg_total_pred_ms;
extern CtrlState g_ctrl_state;
extern CtrlVars  g_ctrl;
extern bool      g_simMode;
extern std::string g_statusMsg;
extern Color       g_statusCol;
extern std::deque<std::string> g_cmdLog;
extern std::mutex              g_cmdLog_mutex;  // guards all g_cmdLog read/write

// Guards g_ctrl_path (and the lastCI/lastCte re-anchor that goes with replacing it)
// across the control thread (ctrl_step(), 50Hz) and the render/UI thread (obstacle-
// triggered hot-swap, Load/Start/Resume) — 2026-07-19, added so a static obstacle
// placed in Step 3 can reroute the ACTIVELY-DRIVEN path live, safely, even mid-drive.
// Recursive because ctrl_step() may call ctrl_doStart() internally (pedestrian
// auto-resume) while already holding it, and ctrl_doStart() also locks on its own
// when called directly from ui.cpp (a different thread) — same mutex, no deadlock.
extern std::recursive_mutex g_ctrl_path_mutex;
// Thread-safe live path swap: reloads g_ctrl_path from CSV_OUTPUT_FILE and re-anchors
// g_ctrl.lastCI to the robot's current position in the NEW array — safe to call from
// any thread, any time (a no-op if nothing valid to swap in). See control.cpp.
void ctrl_hotswap_path();

// Thread-safe helper — use instead of g_cmdLog.push_front() directly
static inline void cmdlog_push(const std::string& msg) {
    std::lock_guard<std::mutex> lk(g_cmdLog_mutex);
    g_cmdLog.push_front(msg);
    if((int)g_cmdLog.size() > 18) g_cmdLog.pop_back();
}
extern std::string g_telemStr;
extern std::vector<Vector2> g_ctrl_trail;
extern float g_ctrlTimer;
extern PoseLPF g_pose_lpf;

// Danger-zone position dead-reckoning override (July 2026) — see DZ_ENTRY_RADIUS_M
// above. While inside the danger zone during driving, position is snapped onto the
// real recorded curve (danger_zone_forward/reverse.csv) at an arc-length (g_dr_s)
// advanced each tick by wheel speed; heading stays IMU-only throughout. Normal
// EKF/scan-match trust resumes once past the end of the danger-zone curve.
extern bool  g_dr_active;
extern float g_dr_x, g_dr_y;   // current resolved dead-reckoned position (for display/logging)
extern float g_dr_s;           // arc-length progress along the danger-zone curve
// Danger-zone exit: on exit, the cart stops and sets this so main.cpp jumps straight
// to the (already-proven) passenger Localize panel — re-localizing and picking a
// destination the same way as starting a fresh journey. A custom in-drive relocalize-
// then-replan flow was tried here first (particle filter launched directly from the
// exit handler, replanning back to the original destination) but hung the whole app
// during a live test (cause not isolated); removed in favour of this simpler,
// already-validated path.
extern bool  g_dz_return_to_localize;
// Danger-zone TEST MAP inner sub-zone (2026-07-07) — independent of g_dr_active above,
// see control.cpp. Straight-line-only dead reckoning within "Danger Zone Fwd"'s
// recorded path. Scan-matcher must bypass EKF correction here too (same reason as
// g_dr_active) or it fights the straight-line position write.
extern bool  g_dzsub_active;
extern float g_dzsub_s;   // distance travelled since entering the sub-zone (for HUD)
// Sub-zone entry/exit world-coordinate markers, user-settable via Step 2's "Set
// Sub-Zone" tool and persisted to dzsub_fwd.cfg.
extern float g_dzsub_entry_mk_x, g_dzsub_entry_mk_y;
extern float g_dzsub_exit_mk_x,  g_dzsub_exit_mk_y;
void load_dzsub_fwd();
void save_dzsub_fwd();
void ctrl_relocate_dzsub_now();  // push a just-saved marker change into the live plan immediately

// UDP sockets
extern SOCKET g_txSock, g_rxSock;
extern struct sockaddr_in g_esp32Addr;

// Logging
extern std::vector<LogRow>  g_log_rows;
extern double               g_log_run_start;
extern int                  g_log_send_seq;
extern int                  g_log_last_ack_seq;
extern double               g_log_last_send_time;
extern double               g_log_last_ack_time;
extern float                g_log_telem_steer;
extern float                g_log_telem_speed;
extern int                  g_log_telem_seq;
extern std::string          g_log_filename;

// Velocity estimation
extern float g_wheel_speed_mps;
extern bool     g_wheel_speed_valid;
extern uint64_t g_wheel_speed_lastMs;
extern float g_vel_est_ms;
extern float g_vel_est_prev_x;
extern float g_vel_est_prev_y;
extern bool  g_vel_est_init;

// IMU/drive ref
extern float g_imu_yaw_drive_start_deg;
extern float g_ekf_yaw_drive_start_rad;
extern bool  g_imu_drive_ref_valid;

// Control thread
extern std::atomic<bool> g_ctrl_thread_run;
extern std::atomic<bool> g_steer_test_running;

// VLP-16 vertical angles
extern const float v_angles[16];

// EKF helpers (defined in localization.cpp)
void fill8(double* M, const double d[8]);
void fill2(double* M, double d0, double d1);
void rot_imu(const Vector3& in, Vector3& out);

// Steer test harness (defined in steer_test.cpp)
void steer_test_run_A();
void steer_test_run_B();
void steer_test_run_A_repeat(int num_runs);
void steer_test_run_sweep();

// Localization test harness (defined in loc_test.cpp)
extern std::atomic<bool> g_loc_test_running;
void loc_test_run_stationary(int duration_sec);

// Fixed-sequence drive test (defined in control.cpp)
// Generates a path from current EKF pose: 1m straight + 1° left + 2m → loads as plan
void ctrl_load_fixed_test_path();

