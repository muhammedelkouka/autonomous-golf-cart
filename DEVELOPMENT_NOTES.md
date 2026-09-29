# Autonomous Golf Cart — Development Notes

## What this project is
A real-time autonomous driving stack for a **Marshell electric golf cart**, running natively
on Windows (C++17 + Raylib). The cart operates in a university robotics lab and on a campus
path. This is a master's thesis project by Mohammed Alaa Elkouka.

## Hardware
| Component | Details |
|-----------|---------|
| LiDAR | Velodyne VLP-16, UDP port 2368 |
| IMU | Serial COM11, 115200 baud |
| Steering actuator | NEMA stepper + CS-D508 driver, belt-drive on steering column, 1:4 ratio |
| Steering range | ±23° wheel lock, 9000 pulses per lock, 400 steps/degree (approx) |
| Drive | MCP4725 DAC (I2C) → 0–5V motor controller |
| Direction relays | FWD GPIO18 / REV GPIO5 |
| Brake | GPIO 19+23 |
| Wheel speed | Hall sensor GPIO13, 1 pulse/rev, tyre circumference 1.442m |
| Microcontroller | ESP32 DevKit V1, firmware: golf_cart_v28_smooth.ino |
| PC | Windows, VS Code, CMake + MinGW |
| ESP32 IP | 10.63.129.200 (changed 2026-07-27, was 10.84.43.200, before that 10.103.193.200), CMD port 4210, TELEM port 4211 |

## IMU Hardware Notes

**Model:** Hiwonder IM10A — 10-axis IMU (accel + gyro + mag + baro)
**Mount:** Flat on center console, Y-axis forward, X-axis right, Z-axis up
**Install Direction setting:** Horizontal (matches physical mounting)
**Algorithm setting:** 6-axis (accel + gyro only, no magnetometer for heading)
**Baud rate:** 115200, Output rate: 100Hz
**Device ID:** WT4100196522

**Packet format (3 packets per 100Hz burst, 11 bytes each):**
- 0x55 0x51 = acceleration (±16g)
- 0x55 0x52 = angular velocity / gyro (±2000 deg/s)
- 0x55 0x53 = angle / euler (±180°)
- Checksum = sum of bytes 0-9 masked to uint8

**Important behavior — gyro_z zero when stationary:**
The IM10A applies zero-rate suppression on the Z (heading) axis.
Gyro_z reads exactly 0.000 when truly stationary — this is CORRECT
sensor behavior, not a fault. It responds correctly to real yaw
rotation (confirmed: 0.18 rad/s peak during 17° cart rotation test).
Do NOT mistake stationary gyro_z=0 for a broken sensor.

**IMU configuration issue encountered (June 2026):**
After running the Hiwonder PC calibration app (accelerometer calibration
+ angle reference reset), the gyro output appeared frozen at zero across
all axes. Root cause: the app's calibration sequence temporarily affects
output register state. Attempted fix of adding WriteFile startup commands
to localization.cpp (GENERIC_READ|GENERIC_WRITE) broke the IMU thread
entirely (0 Hz). Fix was reverted. Actual resolution: the gyro was never
truly broken — stationary gyro_z=0 is correct. The calibration should
only be done when necessary and only Acceleration calibration should be
used routinely. Avoid "Reset Z-axis Angle" and "Angle Reference" buttons
unless intentionally resetting heading reference.

**Code note:**
localization.cpp opens COM11 as GENERIC_READ only — no startup
configuration commands are sent. The IMU must be pre-configured
via the Hiwonder PC app with Velocity+Acceleration+Angle outputs
enabled and saved to flash before running the autonomy stack.

**IMU-LiDAR physical offset (tape measured June 2026):**
- ΔX left-right:  0.00m (centered on same axis)
- ΔY fore-aft:    0.23m (IMU is 23cm behind LiDAR)
- ΔZ vertical:    0.41m (LiDAR is 41cm above IMU)
- Horizontal separation: 0.23m

**Lever arm effect at operating speeds:**
- Gentle turn (gz=0.05 rad/s): 0.00058 m/s² = 0.006% of g
- Normal turn (gz=0.10 rad/s): 0.00230 m/s² = 0.023% of g
- Sharp turn  (gz=0.18 rad/s): 0.00745 m/s² = 0.076% of g
- All negligible — absorbed by 20Hz scan matcher correction ✓

**Systematic position bias:**
- EKF x[0,1] represents LiDAR position (scan matcher locks to it)
- IMU is 23cm behind in vehicle Y — below Pure Pursuit lookahead
  minimum (3200mm) so no path tracking impact ✓
- Future constant to add if needed: IMU_LIDAR_OFFSET_Y = 0.23m

**Conclusion: no code changes needed for IMU-LiDAR offset at
current operating speeds. Localization module fully validated.**

## Software architecture — 8 source files (verified 2026-07-09: main/localization/planning/control/communication/ui/common.hpp + 2 standalone-thread test modules below), plus one standalone Python tool

### main.cpp
- Defines all shared globals
- Spawns 5 background threads: LiDAR, IMU, scan-matcher, control loop, pose-writer
- Runs 60 FPS Raylib render loop
- **Step 1 orbital camera (added June 2026):** `cam_azimuth`, `cam_elevation`, `cam_radius` — spherical coordinate orbit. Left-drag = orbit, right-drag = pan, scroll = zoom, arrow keys = orbit (requires window focus), Home = reset top-down. Camera position recomputed from spherical every frame; camera follow updates target only.

### localization.cpp
- VLP-16 packet parser → point cloud
- IMU reader (serial) → acc + gyro
- EKF sensor fusion (8-state, MATLAB-generated C code in `codegen/`)
- Particle filter global relocalization against PLY map
- **Scan matcher: brute-force distance-transform, NOT ICP, NOT NDT**
  - Pre-computes 2D distance map from PLY cloud (nearest-neighbour field)
  - score_pose(): transforms scan points to world frame, looks up distance for each → returns mean distance (RMSE)
  - Two-pass grid search: coarse ±1.0m / ±0.30–0.60 rad, then fine ±0.15m refinement
  - **2D only — scan[i].z is never used.** Ceiling returns affect heading via x,y projection noise
  - Fix 2 bypasses all heading noise: EKF yaw never reaches Pure Pursuit during driving
- Output: `g_ctrl_robot` pose (x, y, yaw)

### planning.cpp
- Reads raw trajectory CSV or reference_path.csv
- Catmull-Rom spline interpolation + Laplacian smoothing
- Corridor generation (±CORRIDOR_WIDTH_M)
- Goal planning: approach segment + corridor segment + goal segment
- Output: `smoothed_path.csv` with (x, y, yaw, curvature) per point

### control.cpp
- Pure Pursuit algorithm at 50 Hz
- Two-stage time alignment: LiDAR age compensation + actuator lag prediction
- Steer rate limiter, EMA filter, send deadband (1° min delta)
- Startup ramp (blind zone + angle clamp), goal deceleration
- Corridor watchdog (ESTOP if cart leaves path boundaries)
- Velocity estimator (dead-reckoning fallback when no wheel speed)
- IMU yaw blending during drive (speed-dependent alpha)

### communication.cpp
- Two Winsock UDP sockets (TX to ESP32, RX non-blocking for telemetry)
- `ctrl_sendCmd()`: sends ASCII command + timestamps for RTT measurement
- `pollTelemetry()`: parses STEER_DONE, WHEEL_MPS, STEER_DEG, SPEED_V, SEQ
- RTT fed back into control loop as actuator lag estimate
- CSV data logger: LogRow per tick, summary on run end

### ui.cpp
- Raylib 3D/2D map visualization
- LiDAR cloud overlay
- Step 1/2/3 workflow panels
- Live debug readouts (CTE, lookahead, RTT, pipeline delay)
- Map click handler for goal selection
- **Metric grid overlay (added June 2026):**
  - `DrawGrid2D()` — Steps 2/3: adaptive spacing (1/2/5/10/20/50m), axis labels on edges, scale bar bottom-right
  - `DrawGrid3D()` + `DrawGrid3DLabels()` — Step 1: ground-plane grid lines in 3D; labels projected via GetWorldToScreen after EndMode3D
- **3D map cloud height coloring (added June 2026):** map points colored by z height using ColorFromHSV — blue=floor, green=mid, red=ceiling. Reveals 3D structure of map in orbit view.

### common.hpp
- All shared types (Point3D, TrajectoryPoint, CtrlPose, LogRow, etc.)
- All extern globals
- All tuning constants
- Color palette

### loc_test.cpp (added June 2026)
- Standalone stationary localization diagnostic — logs raw IMU, EKF state, scan-match at 50 Hz to `loc_test_stationary_*.csv`. RAII-guarded, never blocks the control loop.

### steer_test.cpp
- Standalone steering-actuator test harness (RTT/latency, sweep tests) — same RAII-guard pattern as loc_test.cpp, safe alongside normal driving.

### tools/pedestrian_detect.py (test feature, added July 2026 — NOT part of the C++ build)
- Standalone Python process: OpenCV + MobileNet-SSD (cv2.dnn, CPU) person detection over one or two USB cameras.
- File-IPC only: writes `runtime_data/pedestrian_status.txt` (`STOP`/`CLEAR` + unix timestamp); never linked into the C++ binary.
- Launched/killed by the Drive-panel "Pedestrian Detection ON/OFF" button (`ped_set_enabled()` in main.cpp, `CreateProcessA`). See the July 8–9 log entry below for full detail and current caveats (depth-awareness unvalidated).

## ESP32 firmware (golf_cart_v28_smooth.ino)
- Non-blocking Arduino loop — no RTOS
- `handleUdpCommands()`: drains up to 8 UDP packets per loop
- `processCommand()`: AUTO gate → `handleMATLABCommands()`
- `updateStepper()`: software step-pulse generator, non-blocking, latched half-period
- `updateDriveRamp()`: voltage ramp + FWD/REV interlock state machine
- `sendUdpTelem()`: fires STEER_DONE + WHEEL_MPS back to PC
- DAC heartbeat every 3s → ESTOP on I2C failure
- UDP watchdog: no command 2s → speed zeroed, steering held
- Web UI: mobile HTML D-pad + JSON /STATE endpoint

## Key constants (common.hpp + firmware)
**Verified against source 2026-07-09** — this block is the single reference; if it ever disagrees
with a dated log entry below, this block wins (log entries are history, this is current state).
```
LOOKAHEAD_MIN/NOM/MAX = 2.5 / 3.2 / 4.0 m   (common.hpp — campus-confirmed June 22, 2026)
WHEELBASE_M           = 2.4 m
MAX_STEER_DEG         = 23.0°
CRUISE_VOLTAGE        = 0.99 V
MIN_VOLTAGE           = 0.95 V
ZONE2_CRUISE_VOLTAGE     = 1.2 V   (zone-2 speed boost; suppressed where z2slow_<ply>.cfg is set)
DANGER_ZONE_CRUISE_VOLTAGE = 0.85 V (danger-zone/sub-zone dead-reckoning speed override)
CONTROL_HZ            = 50 Hz
STEER_RATE_LIM_DEG    = 5.0 deg/s (PC side)
STEER_EMA_ALPHA       = 0.45 (control.cpp — reduced from 0.60 for campus noise tolerance)
STEER_SEND_MIN_DELTA  = 1.0° (PC side, from control.cpp)
GOAL_MAX_STEER_DEG    = 12.0° (raised from 6° — 6° was insufficient to correct CTE entering the goal zone offset)
STEER_SPEED_CRUISE    = 8.0 deg/s  (ESP32 side — bulk travel, remaining > 2°)
STEER_SPEED_CREEP     = 5.0 deg/s  (ESP32 side — final approach, remaining ≤ 2°)
STEER_CREEP_ZONE      = 2.0°       (switch threshold; real driving stays in creep zone)
STEER_CMD_DEADBAND    = 0.30° (ESP32 side)
STEPS_PER_DEGREE      ≈ 391.3 (9000 / 23.0)
CORRIDOR_HALF_M       = 1.9 m
WATCHDOG_GRACE_M      = 1.8 m
```

## 3-step workflow
1. **Localize** — particle filter global relocalization against pre-built PLY map
2. **Plan** — click goal on map → smoothed corridor path → smoothed_path.csv
3. **Drive** — Pure Pursuit tracks path, streams commands to ESP32, logs telemetry

## Maps
**Verified against source 2026-07-09.** `map_library` in localization.cpp holds **8 entries, 6 active + 2 commented out (hidden)**, visible in the `[M]` cycle (was 6/4 before the July 2026 Danger Zone maps were added — that older count is stale wherever else it's referenced in this file). Default map is selected by name ("Full Campus") in main.cpp, robust to list reordering. PLY files in runtime_data/.
- `mapwithme.ply` — Lab map
- `Master_Structural_Ref_Adaptive.ply` — Structural map **(hidden June 25, 2026 — commented out in map_library)**
- `Full_Campus_Map_Master.ply` — Full campus (original, default driving map — best feature density)
- `Full_Campus_Map_ZFiltered.ply` — Campus with z>2.0m removed (ceiling/skylight returns stripped); 76k pts vs 129k original **(hidden June 25, 2026)**
- `Campus_TopLidar_Forward.ply` — Top-mounted LiDAR (raised ~90cm to cart roof), forward-direction recording. Merged from 4 VTK PLY exports (Edge+Planer+Intensity+Trajectory) in `Fdirection map  top/`; 127,856 pts, no filtering. Auto-loads `path_forward.csv` (zone 1) when active.
- `Campus_TopLidar_Reverse.ply` — Same top LiDAR, end-to-start recording. Merged from 4 PLY exports in `Rdirection top map/`; 129,230 pts. Auto-loads `path_reverse.csv`. **Different coordinate frame from forward map** (each VTK recording sets origin at its own start point — a map and the path planned on it are a matched pair; do not mix).
- `DangerZone_Test.ply` — **Added July 2026.** Dense there-and-back recording of just the campus danger-zone stretch (37,387 pts, Z-cropped [-2.5, 6.0]m). **Two map_library entries share this one file** — "Danger Zone Fwd" and "Danger Zone Rev" — loading opposite directions of the same recorded trajectory (`path_dangerzone_fwd.csv` / `path_dangerzone_rev.csv`, the latter now the former reversed point-by-point — see the July 8–9 log entry). Unlike the campus Fwd/Rev pair, Danger Zone Fwd and Rev **share the same coordinate frame** (same file, no transform needed between them) — but that frame is **unrelated to and has no transform to** either campus map's frame. Not wired into corner-mode/zone/cross-zone routing on its own; used by the danger-zone crossing journey (see below) and standalone Step 1/2/3 testing.

## Current project status (as of June 2026)
### What was done
- System was originally monolithic; refactored into the 6-module structure above
- Steering speed was 18 deg/s → caused aggressive oscillation → reduced to 5 deg/s (v28)
- PC-side send deadband increased to 1° to prevent micro-commands causing vibration
- ESP32 cmd deadband set to 0.30° to filter residual noise
- Trapezoidal velocity profile attempted (creep=3, cruise=20, accel=250 deg/s²) — caused **more vibration and worse tracking** than flat profile; reverted
- Dual-speed profile added June 2026: cruise=8 deg/s (bulk travel) + creep=5 deg/s (final 2°) — mathematically smooth; real driving stays in creep zone
- LogRow extended June 2026 with 5 steering-pipeline fields: steer_raw_deg, steer_filtered_deg, steer_delta_deg, steer_sent, tick_index — written to ctrl_log_*.csv
- loc_test.cpp added June 2026: stationary diagnostic logger → loc_test_stationary_*.csv (raw IMU, EKF state, scan-match at 50 Hz)
- Localization module validated June 2026: IMU frame correct, gyro responsive (0.18 rad/s peak in rotation test), EKF fusion stable; scan RMSE 0.050 m, inlier ratio 0.965, position drift <2 cm stationary
- First real drive completed June 2026 — two issues found and fixed (see below)
- CRUISE_VOLTAGE reduced to 0.95 V (from 0.99) for first real drive — restore to 0.99 after
- MIN_VOLTAGE reduced to 0.92 V (from 0.95) for first real drive — restore to 0.95 after
- Pre-drive readiness indicator added to Step 3 Drive panel (LiDAR Hz, IMU Hz, RMSE, plan loaded, real mode, not at goal)
- All test flags (g_steer_test_running, g_loc_test_running) confirmed safe — RAII guards, never block control loop
- logger_write_csv() now called on all stop paths: goal reached, watchdog, pause, E-stop/reset
- **First drive finding 1:** EMA frozen ~1.5s due to yaw-gap startup wait (UI showed "Yaw gap waiting"); threshold is 2.0° (toRad(2.0f) at control.cpp:378); review pending
- **First drive finding 2:** Lookahead 4.2m > path length 3.1m — Pure Pursuit targeted goal directly, no path tracking; fixed by reducing to 1.5/2.5/3.5m
- **Root cause of 1.52s steering freeze:** startup ramp gate forces rawSteer=0 while distTravelled < BLIND+RAMP (was 0.6m total); EMA ran correctly but input was zero; fix: STARTUP_RAMP_M reduced 0.5→0.2m (total startup zone now 0.3m); STARTUP_MAX_STEER_DEG reduced 8→6° for gentler startup with shorter ramp
- **Telemetry bug found and fixed (June 2026):** `udpCmd.remoteIP()` resets to `0.0.0.0` after every `parsePacket()` call — async `STEER_DONE` packets from `_checkSteeringStop()` always got dropped silently. Fix: cache last valid PC IP in `g_pc_ip` global on every successful `parsePacket()`; `sendUdpTelem()` now uses `g_pc_ip` instead of `udpCmd.remoteIP()`
- **Telemetry still not received after g_pc_ip fix:** Root cause — PC used two separate sockets (txSock unbound ephemeral, rxSock on port 4211). Windows Firewall blocks inbound UDP 4211 because no outbound traffic ever came FROM port 4211. Fix: merged into single socket bound to LOCAL_TELEM_PORT (4211), so commands go out FROM port 4211 and replies are allowed back in. Firmware debug added to `sendUdpTelem()` to distinguish 0.0.0.0 drop from endPacket failure.
- **Scan matcher jump rejection added (June 2026):** CTE spiked to 0.552m due to scan matcher applying large sudden corrections. Previous code clamped raw dx/dy to ±0.5m then multiplied by blend (max 0.25) = 0.125m max jump. Changed to reject the entire update if raw displacement > 0.5m, preventing bad-frame corrections from perturbing the EKF during driving.
- **Steering overshoot root cause identified (June 2026):** Fixed-path test (1m + 1° arc + 1m) showed peak commanded steer of 4.71° vs ~1° geometric. Root cause: EMA (α=0.35) took ~7 ticks/140ms to approach target, combined with 1° send gate — PP accumulated CTE waiting for gate to fire and over-commanded. Fixes applied: (1) STEER_EMA_ALPHA 0.35→0.60 (filter reaches gate in ~2 ticks), (2) STEER_SEND_MIN_DELTA kept at 1.0° (decision 2026-06-03: 1° gate retained; ESP32 0.30° deadband handles sub-threshold noise), (3) curvature transition feedforward added to ctrl_step() — adds atan(κ_li·L) − atan(κ_ci·L) to rawSteer so the cart pre-steers at curve entry/exit without waiting for CTE; differential form ensures zero contribution during steady-state arc tracking (no double-counting with PP).
- **Fixed-path test (1m + 1° + 1m) scheduled for 2026-06-03** — will verify peak steer drops from ~4.71° to ~1–2°; also confirms curvature FF works correctly for larger curves on campus path.
- **Path planning overhaul (June 2026):** Four problems fixed in planning.cpp + ui.cpp: (1) Goal click snaps to nearest forward path point — eliminates off-path goal segment that caused bad steering at destination. (2) Forward-only constraint — goal search restricted to indices ≥ robot entry index; backward planning blocked. (3) Off-path goals supported via quadratic Bézier goal connector (`pp_bezier_goal_connector`) — follows reference path to nearest exit then smooth curve to actual clicked position; connector only added when click is >0.5m from path, otherwise snaps to path. (4) Cubic Bézier approach (`pp_bezier_approach`) replaces straight-line approach — departs in robot heading, arrives tangent to path, no kink at join. Heading arrow bug fixed (−sinf → +sinf in all DrawLineEx calls in ui.cpp, visualization only). Hover snap indicator added to Step 2 map — orange ring + heading tick shows where Ctrl+Click will land before clicking.
- **Rendering performance + camera follow (June 2026):** World-space viewport culling added to map cloud draw loop in ui.cpp — previously computed w2s transform for every cloud point then discarded off-screen ones; now pre-filters in world space with 4 comparisons before calling w2s. Eliminates most transform work on large campus map. Camera follow Lerp factor increased 0.08→0.18 in main.cpp so 2D view catches up to robot position ~2× faster during driving.
- **Drive visualization overhaul (June 2026):** Three changes to Step 3 map view in ui.cpp: (1) Bright green ahead-path segment now ends exactly at `lookaheadIdx` instead of a fixed 50-point window — green line and robot target always visually aligned. (2) Lookahead dot, lookahead line, and lookahead radius ring removed — single robot circle is now the only pose indicator, reducing clutter. (3) Robot circle drawn at last `robot_trajectory` point (scan-matched EKF position) instead of `g_ctrl_robot_filtered` — circle now sits exactly at the tip of the magenta trajectory line.
- **Auto-replan on Start (June 2026):** When Start Autonomous is pressed in Step 3, the approach path is automatically regenerated from the current EKF pose before loading the plan. This ensures the Bézier approach always departs from the cart's actual position and heading at drive time, not the position from when the plan was originally saved. Handles the lab test scenario where the cart is repositioned slightly between runs. Only active in real mode when `g_pp_goal` is valid; sim mode and missing-goal cases use the saved plan unchanged.
- **Starting position/heading sensitivity confirmed (June 2026):** Lab tests showed that when the cart is correctly positioned and aligned with the path, the driven trajectory (magenta) and planned path overlap cleanly and the robot stops near the goal. When the cart starts misaligned (especially heading), two offset lines appear on the map (trajectory vs planned path diverge at the start). The heading alignment at start is the dominant factor for short (3m) lab paths. The auto-replan fix addresses position offset; heading alignment must still be done manually before each run.
- **20m straight-line test mode added (June 2026):** `ctrl_load_straight_path(length_m)` in control.cpp generates a zero-curvature path in the current EKF heading direction without any reference_path.csv or Step 2 planning. Activated via "Load 20m Straight" button in Step 3. Intended for campus straight-line validation: do Step 1 (localize), skip Step 2, press the button, then Start Autonomous. The function auto-centres the path laterally using the active map's distance map — samples ±3.5m perpendicular to the heading, detects corridor walls, and shifts the path start toward the corridor centre to maximise clearance on both sides.
- **Scan matching degrades near walls on campus (June 2026):** During the 20m straight-line test the cart started close to the right corridor wall. As it moved, scan match inlier ratio dropped and localisation was eventually lost. Root cause: `Full_Campus_Map_Master.ply` was recorded from a specific path — map point density is lower near the walls than in the open corridor centre. When the cart is too close to one wall, the LiDAR scan becomes dominated by that wall and the scan matcher has too few distinctive features to lock on. Auto-centering via distance map was attempted but was insufficient when the map itself has poor coverage near walls. Current approach: **10m straight-line test** — shorter distance reduces the risk of full localisation loss. Advice: always position the cart ≥ 1.5 m from any wall before starting a campus drive, and ensure RMSE stays < 0.20 m and overlap > 50% throughout.
- **June 4, 2026 campus test — all 8 runs failed (0 GOAL_REACHED):** Max CTE 1.2–1.9m. One near-success: run 095727 (max CTE=0.381m, 1.02m from goal at a position with better map coverage). Full analysis in `SESSION_ANALYSIS_20260605.md`.
- **Root cause of June 4 failures identified June 5, 2026:** `PoseLPF::ALPHA_YAW_SLOW=0.55` passed scan-match heading drift (~1°/second) directly into Pure Pursuit. The campus corridor has a semi-transparent skylight ceiling — VLP-16 upper beams produce lighting-dependent returns that the ICP interprets as small heading corrections every tick. With ALPHA=0.55, each correction applied at 55% per tick, accumulating ~1°/second drift in the robot heading seen by PP. PP responded to heading drift as if the robot faced the wrong direction → steer grew from 0.6° to 23° on a nearly straight path. Proven from CSV: at tick 100 of run 093320, CTE=−0.019m (2cm off path) yet steer_raw=−4.17°. Additional factors: near walls ICP degeneracy (single flat plane = zero rotational constraint), low map density near walls.
- **Hardware upgrade evaluated June 5, 2026:** Moving LiDAR to cart top + adding camera. Decision: NOT needed. Higher LiDAR would scan more ceiling (worse). Camera is large scope increase. Current hardware sufficient with software fixes.
- **KISS-ICP map rebuild attempted June 5, 2026 — FAILED:** reading7.pcap processed with KISS-ICP (no loop closure). Over 8492 frames the accumulated drift produced a completely unusable smeared map. Approach abandoned; all related files deleted from project.
- **Four fixes applied and built June 5, 2026 (all in current GolfCartControl.exe):**
  - **Fix 1** `common.hpp` PoseLPF: `ALPHA_YAW_SLOW` 0.55→0.10, `ALPHA_YAW_FAST` 0.85→0.45 — heading filter now spreads each ICP correction over ~22 ticks instead of 3
  - **Fix 2** `control.cpp:232`: During driving, `g_pose_lpf.update()` now receives `imu_yaw_world` as the heading argument instead of `g_ctrl_robot.yaw`. **EKF/scan-match heading cannot reach Pure Pursuit at all during driving.** EKF is still used for position (x,y). Heading is purely IMU gyro integration. Eliminates all lighting/ceiling/map-density effects on steering.
  - **Fix 3** `control.cpp:408`: `yaw_converged` clamp time-limited to first 100 ticks (2 seconds) after startup gate. Previously ran indefinitely — with Fix 2 decoupling EKF from IMU heading the clamp would fire permanently without this limit.
  - **Fix 5** `control.cpp:298`: Watchdog uses `fabsf(g_ctrl.crossTrackErr) > CORRIDOR_HALF_M` instead of `ctrl_nearestDistWindowed()`. 4/8 June 4 runs exceeded corridor boundary without watchdog firing because `ctrl_nearestDistWindowed` is Euclidean distance to nearest path point, not perpendicular CTE.
- **Campus Z-Filtered map added June 5, 2026:** `Full_Campus_Map_ZFiltered.ply` — original campus map with z>2.0m removed (40.8% stripped = ceiling/skylight/upper-building). 76k pts vs 129k original. Added as Map slot 4. Use Map 3 (original full campus) for driving — has more features. Map 4 available as fallback.
- **IMU drift characterisation (June 2026, laptop+IMU only):** `tools/imu_drift_test.py` — standalone serial reader for IM10A; reads 60s of angle packets from COM11, reports total drift and rate. Results: stationary = +0.000° drift (PASS); with table tapping/vibration = ~0.27° during stable phase (vibration does NOT cause false yaw), ~0.9° final drift was from physically picking up the sensor (correct detection of real rotation). Fix 2 confirmed reliable — zero-rate bias is zero.
- **Scan matcher architecture clarified (June 2026):** localization.cpp scan matcher is NOT ICP and NOT NDT. It is a brute-force 2D distance-transform grid search. score_pose() ignores scan[i].z entirely — matching is always 2D. Ceiling returns cause heading noise only via their x,y projections. Fix 2 bypasses this completely.
- **Campus map rebuild evaluated (June 2026):** Current Full_Campus_Map_Master.ply was already built with LiDAR View + MATLAB from reading7.pcap. Rebuilding from the same PCAP produces the same result. Better map requires multi-pass recording (3 passes: left/center/right lane) when cart is available — not needed before next test.
- **UI visualization overhaul (June 2026):** (1) Metric grid overlays added to all steps — DrawGrid2D for Steps 2/3, DrawGrid3D+DrawGrid3DLabels for Step 1. Adaptive spacing, scale bar, world-coordinate labels. (2) Step 1 orbital camera: left-drag to orbit, right-drag to pan, scroll zoom, arrow keys, Home=top-down. Spherical coordinate math in main.cpp; camera up vector recomputed each frame to keep map Y-axis pointing screen-up at all tilt angles. (3) 3D map cloud colored by height (ColorFromHSV) to reveal vertical structure.
- **Deep code review + 11 bugs fixed (June 8, 2026):** Automated multi-agent review of all 6 source files + ESP32 firmware. All found bugs fixed; build verified clean. Key fixes:
  - **Bug 2 (control.cpp:435):** EMA seed condition `!startupTick` was always false (tick=1 at eval) → changed to `startupTick==1`. EMA now seeds from the first PP output instead of always starting from 0. This was the true root cause of the documented "EMA freeze" — the freeze was never fully fixed before.
  - **Bug 7 (planning.cpp):** `pp_save_plan()` re-scanned `g_smooth` to find corridor indices, which could produce `idx_end < idx_start` on near-loop paths → reversed corridor → completely wrong steering. Fixed by storing `g_corridor_idx_start/end` in `pp_trigger_plan()` and using them directly (always forward-ordered).
  - **Bug 4 (control.cpp):** `loc_fail_since` was a `static double` local that survived across drive runs — could trigger instant false "localisation lost" E-stop at start of next run. Moved into `CtrlVars.locFailSince` (reset by `g_ctrl={}` in ctrl_doStart()).
  - **Bug 3 (control.cpp):** `g_wheel_speed_valid` not reset to false in `ctrl_doStart()` — stale true flag let the 1.0 m/s seed corrupt velocity estimator for first 2s. Added `g_wheel_speed_valid=false` reset.
  - **Bug 1 (all files):** `g_cmdLog` deque written from 4+ threads (scan-match, steer-test, loc-test, control) and read from render thread with no mutex → crash risk. Added `cmdlog_push()` helper in common.hpp with `g_cmdLog_mutex`; all access sites updated.
  - **Bug 8 (planning.cpp):** Silent `return` on degenerate plan (< 2 points) replaced with `g_statusMsg` error.
  - **Bug 10 (firmware):** Hard stepper limit now sends `STEER_DONE` so PC steer-test and RTT estimator don't hang.
  - **Bug 11 (firmware):** `FWD_MODE` default voltage changed 2.5V → 0.0V — eliminated 1–2 ms voltage spike before SPEED command arrives.
  - **Bug 5 (communication.cpp):** `g_lidar_sweep_time` snapshotted once at start of `logger_tick()` — prevents negative `pipeline_delay_ms` from race with LiDAR thread.
  - **Bug 9 (communication.cpp):** `g_wheel_speed_lastMs` changed `unsigned long` → `uint64_t` — eliminates 49-day epoch ms wraparound.
  - **ESP32 firmware must be re-flashed** for Bugs 10 and 11 to take effect on the cart.
- **Additional fixes applied June 13, 2026:**
  - **cyaw_corrected algebraic bug (ui.cpp:237):** `cyaw_corrected = wrapA(cyaw_lidar + g_yaw_imu_lidar_offset_rad)` was algebraically equal to `imu_rad` (raw IMU angle) because `g_yaw_imu_lidar_offset_rad = imu_rad − cyaw_lidar`. The scan matcher heading was discarded at localization confirm and the EKF was seeded with the raw IMU angle instead. Fixed: `cyaw_corrected = cyaw_lidar` directly. `g_yaw_imu_lidar_offset_rad` still computed and stored for UI display only.
  - **Wrong calibration comment fixed (sensor_calibration_result.h:17):** Comment said "right-turn = positive Z" — physically wrong. Code and hardware are CCW-positive throughout. Comment corrected; values (+1.0f) were already correct.
  - **Scan distortion during turns mitigated (localization.cpp + common.hpp + control.cpp):** VLP-16 takes 100ms/scan; cart rotates during scan → geometrically distorted scan → bad position and heading corrections fed into EKF during turns. Three-layer fix: (1) `localization.cpp` scan_matching_thread: `blend *= 0.40f` when `g_is_turning` — position correction rate halved during turns; (2) `localization.cpp`: EKF heading update (`ekf.x[2]`) fully suppressed during turns — only position (x,y) updated from distorted scan; (3) `common.hpp` PoseLPF: `ALPHA_XY_TURNING = 0.35f` constant + `bool turning=false` parameter added to `update()` — position LPF uses slower alpha during turns. `control.cpp` ctrl_step() updated to pass `g_is_turning.load()` as 4th argument to all three `g_pose_lpf.update()` calls. Build verified clean June 13, zero warnings.
  - **Magnetometer evaluated and rejected:** IM10A has 10-axis (mag present) but motor current changes invalidate hard-iron calibration dynamically. Not reliable for ground vehicle with electric drivetrain. Decision: continue with Fix 2 (IMU delta + EKF anchor). For 50m+ paths, if heading drift confirmed from CSV, add velocity-derived heading from consecutive EKF positions as low-gain anchor correction.
  - **printf() in control loop replaced (control.cpp:388):** printf() was called every 100 ticks (2s) inside ctrl_step() — on Windows console this blocks 1–3ms, causing periodic timing jitter in the 50Hz loop. Replaced with cmdlog_push() which queues to the render thread.
  - **g_log_rows pre-allocated (communication.cpp):** logger_start_run() now calls g_log_rows.reserve(6000) before driving starts, eliminating all std::vector reallocation stalls mid-drive (previously reallocated at row 1024, 2048 etc. inside the control thread).
  - **Step 2 heading arrow sign fixed (ui.cpp:857, 867):** Robot pose arrows (g_pp_robot, live EKF) in Step 2 used `+sinf(yaw)` for the X screen offset — this is inverted for CCW-positive yaw and a Y-flipped map. Fixed to `-sinf(yaw)`. Hover snap indicator (line 884) correctly kept at `+sinf` because g_smooth[i].yaw uses `atan2f(dx, dy)` (CW-positive) — opposite convention from robot yaw.
  - **Path yaw sign inconsistency identified (planning.cpp:84):** `g_smooth[i].yaw = atan2f(dx, dy)` produces CW-positive angles (east = +π/2), while robot/EKF/IMU yaw is CCW-positive (east = −π/2). These are opposite signs. **No impact on current driving** — CTE uses path segment geometry not .yaw; PP uses robot yaw only; curvature feedforward is disabled; watchdog uses fabsf(CTE). **Future action required:** fix to `atan2f(-dx, dy)` BEFORE re-enabling curvature feedforward, or curvature FF will steer in the wrong direction (documented cause of previous wall-strike incidents).
  - **CSV logging expanded (common.hpp + communication.cpp + control.cpp):** Nine new columns added to `ctrl_log_*.csv` for complete post-run diagnosis. New columns: `imu_raw_deg` (raw IMU absolute angle for drift measurement), `ekf_yaw_deg` (scan-matcher heading before Fix 2, for divergence analysis), `path_x/path_y/path_yaw_deg` (nearest planned path point — planned vs actual overlay), `lookahead_x/lookahead_y` (PP lookahead target — shows what PP was steering toward), `scan_rmse` (scan match quality per tick), `scan_inlier_ratio` (scan match confidence per tick). All populated from existing globals; no new globals needed. Build verified clean.
- **Three CCW-positive sign fixes applied June 21, 2026 (all in current GolfCartControl.exe):**
  - **Bézier approach departure sign (planning.cpp ~line 161):** `pp_bezier_approach()` used `+sinf(start.yaw)` for the X control point — wrong for CCW-positive yaw (forward = −sin, +cos). Fixed to `-sinf(start.yaw)`. Caused S-curved planned path visible in Step 2 sim. `pp_bezier_goal_connector()` uses CW-positive path yaw so correctly kept `+sinf`.
  - **PP body-frame transform (control.cpp ~line 200):** PP used CW-positive rotation matrix `lx=sin*dx+cos*dy, ly=cos*dx−sin*dy`. For CCW-positive yaw the correct form is `lx=−sin*dx+cos*dy, ly=cos*dx+sin*dy`. Old code computed 16.2° steer at yaw=4.9° on a straight approach (CTE=0.002m). New code computes −0.14°. Proven from CSV: steer_raw dropped from 16.21° to −0.14° after fix.
  - **Prediction stage X sign (control.cpp ~lines 360, 377):** Both time-alignment stages had `synced.x += speed*sin(yaw)*dt` — wrong sign for CCW-positive (X decreases when moving at positive yaw). Fixed to `-=`. Stage A (LiDAR age) and Stage B (actuator lag) corrected.
- **Lab validation run June 21, 2026 (ctrl_log_20260621_110639.csv):** 7.2s, 360 ticks. PP fix confirmed — steer_raw at tick 0 = −0.14° (was 16.21° pre-fix). CTE stayed ≤ 3cm throughout. Localization: RMSE 0.025–0.042m, inlier 0.97–1.00. Cart reached 0.86m from goal (run manually stopped). Only 2 steer commands sent (1.02° at t=3.87s, 2.05° at t=5.07s) — 1° send gate blocked all others. Telemetry ack_seq=−1 (ESP32 not yet re-flashed). IMU drift: −0.32°/s (−2.3° over 7.2s) — Fix 2 correctly tracking via IMU delta.
- **Two more lookahead bugs fixed June 21-22, 2026 (campus test session):**
  - **Lookahead forward-check sign (control.cpp:192):** `dot = sinf(yaw)*dxi + cosf(yaw)*dyi` — CW-positive forward check. For CCW-positive yaw, forward = (−sin, cos), so fixed to `dot = −sinf(yaw)*dxi + cosf(yaw)*dyi`. For a west-facing robot (yaw≈80°), old code rejected ALL westward path points as "behind", causing PP to fall back to the goal (60m away).
  - **Lookahead fallback condition (control.cpp:195):** `if(i<N-5||d<la*1.5f) fallback=i` — the `i<N-5` clause updated fallback on every loop iteration, causing fallback=N-6≈goal on the campus path. Removed `i<N-5||` so fallback only updates within 1.5× lookahead radius.
  - Root cause: June 21 CCW sign session fixed the body-frame transform but missed the "is this point ahead?" forward-direction check. Lab unaffected (N-6≈goal≈3m away). Campus affected post-corner when robot faces west.
- **RMSE blend gate added June 22, 2026 (localization.cpp):** `if(best_rmse > 0.25f) blend *= 0.1f;` — when scan quality is degraded (window glass / inner wall proximity at corner), EKF position correction drops to near-zero. Cart dead-reckons on IMU through the bad scan zone instead of taking corrupted position updates. Active in the 0.25–0.40m RMSE range; `good=false` gate unchanged above 0.40m.
- **PreviewCurv window change reverted June 22, 2026:** `8.0f` change was reverted back to `3.0f` before the successful campus test — `3.0f` is the confirmed working value.
- **Campus test result June 22, 2026 (FINAL CONFIRMATION):** System confirmed **stable** on campus with all June 5 + June 13 + June 21 fixes applied. Smooth driving, goal reached. This is the validated working baseline — do not change parameters without CSV evidence.
- **Top-LiDAR remount + new maps + UI features (June 24–25, 2026 — UNTESTED on cart, pending drive validation):**
  - **LiDAR physically moved to cart roof (~+90cm height).** Two new campus maps recorded with the new mount and added to `map_library`: `Campus_TopLidar_Forward.ply` (127,856 pts) and `Campus_TopLidar_Reverse.ply` (129,230 pts). Each merged with no filtering from 4 VTK PLY exports (Edge/Planer/Intensity/Trajectory). NOTE: scan matcher is 2D (ignores scan z), so the height change needs no code change; `SCAN_Z_MIN/MAX` filter is in LiDAR-local frame and the maps were built with the new mount, so frames are self-consistent.
  - **PLY loader stride bug FIXED (`load_ply` in localization.cpp):** the loader counted *every* `property` line until `end_header`, including a `face` element's `property list uchar int vertex_indices`, inflating per-vertex stride 12→16 bytes and scrambling x/y/z (map appeared as 3 overlaid axis-swapped copies). Fix: added `in_vertex` flag — only count properties belonging to the `vertex` element (stop at next `element`). Vertex-only headers (old maps) unaffected. The two new merged PLYs are also written vertex-only as belt-and-suspenders.
  - **Two maps hidden** (commented out in `map_library`): "Master Structural" and "Campus Z-Filtered". Default-map selection in main.cpp now picks "Full Campus" **by name** (was hardcoded index 2) so hiding/reordering can't change the default.
  - **Per-map path record + reuse (planning.cpp + ui.cpp):** Step 1 has two snapshot buttons ("Record Forward/Reverse Path") that write the live localization trajectory to `path_forward.csv` / `path_reverse.csv`. `pp_run_pipeline()` auto-loads the matching file by active map name ("Campus Top LiDAR"→forward, "Campus Top (Rev)"→reverse), taking priority over `reference_path.csv`.
  - **Step 2 smoothing toggle** (`g_path_smoothing_enabled`, default ON): ON = Laplacian+Catmull; OFF = raw recorded points used directly as the driven path. Affects what is written to `smoothed_path.csv`.
  - **Step 2 path-overlay toggle** (`g_show_path_overlay`, default ON): show/hide the reference-path corridor curve (g_corridor_L/R + smoothed centerline) in Plan and Drive views. Display only.
  - **Clear-old-path on map switch:** `pp_clear_path()` (planning.cpp) clears g_smooth/corridors/planned segments and invalidates g_pp_goal; called from the `[M]` map-cycle handler in main.cpp so a path built in one map's frame doesn't linger on another.
  - **Range-rings toggle** (`[G]` key, `g_show_range_rings`, default ON): show/hide the Step-1 3D range rings + crosshair. Display only.
  - All built clean. **None of this is drive-tested yet** — maps display correctly (single coherent frame after the loader fix) but localization RMSE/inlier quality on the top-LiDAR maps is unproven.
- **Campus retest June 25, 2026 — OLD MAPS BASELINE RE-CONFIRMED:** After all the June 24–25 additions (new maps, loader fix, UI toggles, hidden maps), the system still drives well on the **original maps** (e.g. Full Campus) with good results. Planning, path building, and driving all still work as before. **The June 22 working baseline settings are intact and confirmed — do NOT change them.** The June 24–25 changes are additive and did not regress the old-map path.
- **Corner-path workflow + corner-mode planner (June 25, 2026):**
  - **Per-map corner paths:** each top-LiDAR map's reference path is a single recorded **corner curve** (extracted from that map's VTK `Trajectory path.ply` build trajectory, via offline scripts). `path_forward.csv` = forward route's **corner #1** (73 pts, ~30 m). `path_reverse.csv` = reverse route's **last corner** (74 pts, ~31 m, ~8 m radius), point order flipped so it runs in the cart's actual driving direction. These auto-load by active map name and are driven with the **Smoothing toggle OFF** so the pre-cleaned corner isn't re-rounded.
  - **`pp_trigger_plan()` corner mode (planning.cpp):** when a top-LiDAR map is active AND its zone is armed (`corner_active`), the plan is **forced through the full corner** — enter from whichever corner end is nearest the robot, traverse the whole corner to the far end, with straight connectors in (robot→corner) and out (corner→goal). This is direction-aware and deterministic, so it no longer matters where the cart localizes. **Why:** the old nearest-point + forward-only logic collapsed the corridor when the cart localized near the corner's far end (the "passes through if not localized, not after" bug).
  - **Free-mode gating:** free mode (direct Bézier robot→goal) is disabled for corner maps when the zone is armed, so a stale `segment_polygon.cfg` can't hijack the reverse plan into skipping the corner. Old maps keep their original free-mode/nearest-point behavior (corner maps gated by name only).
- **Localization zones — Phase 1 (June 25, 2026):**
  - Per-map polygon "zones" saved to `zone1_<plyname>.cfg` (auto-loaded on map switch). **Draw** in Step 2 ("Draw Zone 1" → click map → Save); **arm** in Step 1 ("Arm Zone 1").
  - Arming a zone: (1) **focuses particle-filter seeding** inside it — Enter seeds at the zone centroid + covering radius instead of the EKF pose (user still presses Enter / Shift+Click to fire); (2) **centers the view** (Step-1 camera + Step-2 2D) on the zone.
  - **Reference-path curves appear only when the zone is armed** (`g_show_path_overlay` + zone gate); and the **corner is used by the planner only when the zone is armed** (else a direct path). Maps with no zone fall back to the manual overlay toggle.
- **Forward map: CONFIRMED WORKING June 25** with corner-mode + zones (drives the corner #1 cleanly). **Old maps: working.**
- **Reverse map: still imperfect.** It localizes and follows the corner when starting **at/just before the curve on the straight**, but **over-steers and loses it on the ~8 m corner** when driven at cruise (5°/s steer-rate limit can't keep up → lookahead sticks at the apex → loops). Confirmed from `ctrl_log` (path_index froze at the apex, steer saturated −23°, CTE grew).
- **Reverse-map driving experiments REVERTED June 25 (per user):** the following were tried then **removed** to keep the forward-confirmed baseline clean — do not assume they exist: (1) curve **checkpoints** (stop + confirm at curve entry/exit), (2) **creep-through-curve** speed reduction, (3) **"exit = corner point nearest goal"** (reverted to always force the full corner), (4) **Taubin** non-shrinking smoothing (reverted to Laplacian). A curve-**widening** attempt (push points off the wall) also failed (made it jagged) and was reverted. **Open problem:** reliably driving the reverse ~8 m corner — likely needs either a genuinely wider re-recorded corner or a corner-speed reduction, but neither is currently in the code. (Harmless leftover: a 3-line orphaned "Taubin" comment in planning.cpp — no code.) (Top-LiDAR map behavior discussed separately — see next session notes.)

- **Project is now under git (June 25, 2026).** `git init` done; `.gitignore` (build/, logs) + `.gitattributes` (*.ply etc. = binary, no CRLF corruption). **Baseline commit `bb438fd`** = the validated state (top-LiDAR maps + zones + corner-mode, forward confirmed, reverse experiments reverted). Each enhancement after it is its own commit so any can be reverted independently (`git revert <hash>`) or roll fully back with `git checkout bb438fd -- .`.
- **Per-map config files (June 25, 2026).** Each map now has its own side-car cfgs, auto-loaded on map switch / startup and cleared between maps so different coordinate frames never bleed across:
  - `zone1_<plyname>.cfg` — zone-1 polygon (existing)
  - `dest_<plyname>.cfg` — the 3 destinations (NEW). Top-LiDAR maps do **not** fall back to the legacy global `destinations.cfg` (its old-frame coords landed far outside the zone — the "starts at map origin" symptom); old maps still use the global fallback. Reverse-map destination **names** = Swimming Pool / Security / Lab.
  - `eststart_<plyname>.cfg` — per-map **estimated start pose** x,y,yaw (NEW).
- **Step 2 enhancements (June 25, 2026):**
  - **Pick-to-generate-path:** each saved destination is a "Go" button that sets the goal and runs `pp_trigger_plan()` immediately (corner-mode if zone armed). Set the 3 points via "Set Destinations on Map".
  - **Estimated start point:** "Set Start Point" → click the map where the cart starts; heading auto-derived toward the nearest corner point (driving direction). Drawn as a green marker + heading tick. Before localizing, the planning pose uses the estimated start (preferred) or the zone centroid, so previews begin at the real start, not the map origin. After localizing, the real EKF pose takes over.
- **Corner-mode planner refined (June 25, 2026):** `pp_trigger_plan()` now forces the corner only when it is actually **between** robot and goal — it compares which corner END is nearest the robot vs the goal. Opposite ends → force the full corner (as before). **Same end → cart has already passed the corner → direct path to goal** (no more "loop back to drag it through the curve again"). The corner is still gated to top-LiDAR maps with zone 1 armed.
- **Corner-mode partial traversal (planning.cpp, late June 2026):** `pp_trigger_plan()` corner mode now finds the nearest corner **point** (any index, not just an end) to the destination. A mid-corner destination routes only to that midpoint and **stops there** (no overshoot to the far end + backtrack). Same-end-as-robot → direct path (unchanged). Still gated to top-LiDAR maps with zone 1 armed.

- **Operator/passenger UI overhaul (June 29–30, 2026 — UI only, no control-path changes):** Large redesign of the three front-end panels (intro, localize, destination). All purely presentational + a new pre-localization UX step; the autonomy stack (localization, planning, control) is untouched. **None of this changes driving behavior.**
  - **Smooth font everywhere:** Segoe UI loaded via `LoadFontEx` (`C:/Windows/Fonts/segoeui.ttf`, 72px atlas, bilinear) and passed into the intro/localize/destination panels. Replaces the pixelated Raylib bitmap font on those screens. `DrawTextEx`/`MeasureTextEx` helpers (`TW`/`TC`/`MW`) with 0.6px letter-spacing. NOTE: the loaded glyph set is ASCII-only — em-dash `—` and arrows `→`/`»` render as `?`; use plain `-`/`>` in panel strings (this was a real bug, now fixed).
  - **Intro panel:** professional cards (drop shadow, inner-shine line, rounded corners, accent top-stripe), plain-language copy aimed at non-engineers ("Drive the cart manually from your phone", "The cart drives itself to your destination", steps "Find cart location / Choose where to go / Sit back…"). Two primitive-drawn icons: Remote = phone + 3 WiFi arcs; Autonomous = the `cart_icon.png` image (white bg stripped + auto-cropped + bilinear at load). Cards 600×545, title 52 / card-title 44 / body 23 / button 27.
  - **NEW Localize panel (`DrawLocalizePanel` in ui.cpp):** passenger-friendly "find the cart" step. Same chrome as the destination panel (title bar + campus background + bottom bar) but **no image toggle**. Center status card with animated radar (pulse + sweep), headline/subtext, progress bar. States from `fus_state` + reloc globals: warming-up (CALIBRATING) / ready (ALIGNMENT_CHECK) / searching (GLOBAL_RELOC, shows `g_reloc_progress`) / located (FUSION_RUNNING, shows `g_reloc.score`%) / failed (`g_reloc_error_msg`). **"Find My Cart" runs the REAL particle-filter global relocalization** — the panel only sets `g_localize_request`; `main.cpp` consumes it and launches `particle_filter_reloc_thread` (same engine/seed logic as the Step-1 Enter key: zone centroid if armed, else EKF pos). **Temporary "Skip (test)" button** (amber, bottom-left) bypasses localization → destination panel; commented `TEMP … remove before deployment`.
  - **New panel flow:** Intro → **Localize** → Destination → main app (Step 1/2/3). Wiring: intro "Start Journey" sets `g_show_localize`; localize "Continue" (enabled only when located) sets `g_show_dest`; localize "Back" → intro; destination "Back" → localize. New globals in common.hpp: `g_show_localize`, `g_localize_request`. (If we later want Localize **after** Destination, it's a ~2-line swap.)
  - **Background: satellite ↔ 3D toggle.** Two textures loaded at startup: `g_bg_sat` (`campus_bg.png`) and `g_bg_3d` (`campus 3d image.png` — new aerial render in runtime_data). `g_bg_show_3d` (default true) selects `g_bg_active`, passed to all panels. Toggle button ("View: 3D / Satellite") lives **only** on the destination-panel title bar (top-left). Each falls back to the other if one file is missing; both bilinear-filtered.
  - **Passenger zone grid is now resize-stable (the important fix).** Was a fixed **pixel** size (`PAXGRID_PX=90`) → cells re-flowed and covered different campus spots when the window resized. Now a **fixed `PAX_COLS×PAX_ROWS` (20×12) grid anchored to the IMAGE** (normalized 0..1), drawn through the same cover-fit transform as the background (`bgX/bgY/cellW/cellH`, `cellRect(c,r)`). A given cell always covers the same campus location at any window size. Caveat: zones are tied to whichever background image is shown — if the satellite and 3D renders frame the campus differently, a painted cell sits over a slightly different spot when toggling views; paint zones on one view and keep passengers on it.
  - **Destination panel reorganized:** clean title bar (View toggle left / "Where would you like to go?" center / Admin right), single vertically-centered bottom row (Back · status · Begin Journey; admin: Cancel · allowed/blocked legend · Save & Exit), one shared rounded `Button()` helper, no special-char arrows.
  - **Assets added to runtime_data:** `cart_icon.png` (golf-cart glyph), `campus 3d image.png` (3D aerial background), `paxgrid.cfg` (passenger zone grid, campus-wide not per-map).
  - **Status: builds clean, NOT yet committed to git** (per user: commit deferred to a later session). UI not yet validated on the cart. Localize panel triggers real reloc but unproven end-to-end on hardware.

- **Zones 2 + cross-zone routing + destinations + path editing + welcome panel + perf cache (July 3–4, 2026 — builds clean, bench-tested UI only, NOT committed, real-cart test scheduled next):**
  - **Zone 2 added (per-map second corner):** full parallel duplicate of the zone-1 system (`g_zone2_*` in common.hpp/main.cpp, `zone2_<ply>.cfg`, "Draw Zone 2" in Step 2, "Arm Zone 2" in Step 1). Deliberately duplicated (not a generic `Zone[]` array) to avoid touching zone-1's validated corner-routing. Zones are mutually exclusive (arming one disarms the other) except the cross-zone case. Shared helper `zone_centroid_radius()` (common.hpp) dedups the centroid math. Arming a zone now reloads the pipeline + replans immediately (previously it didn't → stale-corner bug).
  - **Corner-mode is now corridor-based, not polygon-based:** `pp_trigger_plan()` decides which zone the robot/goal belong to by **which recorded corner corridor each is NEAREST to**, NOT `point_in_polygon` against the hand-drawn zone boundary (the polygons don't reliably contain the corridor points → caused "skipped zone 1" / "dragged through zone 2" mis-routes). The single-zone same-end "direct shortcut" was removed — it now ALWAYS follows the recorded corridor (a straight shortcut could cut through a building and skip stops).
  - **Cross-zone routing:** when robot is nearest one zone's corridor and goal the other's, route along the **full continuous recorded drive** (`path_forward_full.csv` / `path_reverse_full.csv` = the entire original `Trajectory path.ply`, both corners + the gap between) from robot's nearest point to goal's nearest point. Follows the real road across the ~75 m un-recorded gap between the two separately-cut corner corridors (a straight connector there cut through buildings). **Screenshot-confirmed working.** Extraction tool: `tools/extract_zone_path.cpp` (`--list` to eyeball indices, `--range <ply> <a> <b> <out>` to cut). Zone-2 corner files (`path_forward2.csv` etc.) were extracted this way — NOTE they may still be the long over-crop; verify before trusting zone-2 single-zone routing on hardware.
  - **Destinations now two-tier:** zone 1 = `NUM_DEST=5` (`g_dest_names`: Labs, Swimming Pool, Security, Building A, Between Building A and S) from `dest_<ply>.cfg`; zone 2 = `NUM_DEST2=4` (`g_dest_names2`: Basketball Court Curve, Gate, Curve 2 - Food Court, Green Area) from `dest2_<ply>.cfg`. `resolve_combined_dest(combined_idx,...)` maps a combined index (0..NUM_DEST-1 = z1, NUM_DEST..NUM_DEST_TOTAL-1 = z2) to coords + arms the right zone; used by the passenger pax-grid and Step-2 tap-grid. Step 2 has separate pick→click→**preview**→Confirm/Retry/Cancel setup flows per zone (nothing saved until Confirm). Visiting order stored in `g_dest_order_fwd[]`/`dest_order_rank()` (main.cpp) as documented intent — enforcement is geometric (corridor traversal passes every stop between robot and goal in physical order).
  - **Passenger flow finalized:** Welcome (GIU splash, `DrawWelcomePanel`, `runtime_data/giu_welcome.png`) → Intro → Map Select → Localize ("Find My Cart" = real particle-filter reloc) → Destination → **Step 3 (Drive), direct**. "Begin Journey" resolves the destination, seeds the start pose (live EKF → estimated start → world origin), builds+saves+loads the plan, and jumps to Step 3 with the path shown and Start Autonomous ready (does NOT auto-start the motor — safety). Works on the bench without sensors (fallback pose); on the cart it uses the live localized pose. Step-2 panel decluttered (hid old "Tap Zones" g_dest_grid section + the confirmed-start/path-stats readout; Edit Path controls moved to the top).
  - **Local path editing (Step 2 "Edit Path"):** select a stretch with two clicks, "Nudge Left/Right" slides it sideways (0.10 m/press) as a **rigid parallel offset** (single chord-perpendicular direction with cosine-ramped ends — a per-point normal wobbles into a zigzag on noisy paths), Save writes `g_smooth` back to the active corner CSV (`pp_shift_path_segment` / `pp_save_edited_path` in planning.cpp). Save is destructive to the corner CSV (git-restore to undo).
  - **PERF fix — corridor cache (planning.cpp):** `pp_trigger_plan()` was loading + 40-pass-smoothing both corner corridors (~3.6k pts) + the ~8k-pt full drive from disk on EVERY call, and it fires on every robot-move frame during Step 2 pre-localize tracking (main.cpp ~1118). Now cached by `active_map->name + smoothing-mode` (`corr_cache_refresh()` for the 2 corners, `corr_cache_ensure_full()` lazy-loads the full drive only for cross-zone trips); `pp_invalidate_corr_cache()` (called by `pp_save_edited_path`) forces reload after an edit. Single-threaded (pp_trigger_plan is UI-thread only). Per-plan cost during tracking is now just O(n) nearest-point scans, no file I/O — eliminates the stutter. Old non-top-LiDAR maps skip the whole zone block, unaffected.

- **Danger-zone crossing journey + friendly Step-3 view + pedestrian detection (July 8–9, 2026 — builds clean, supervisor demo PASSED July 9, NOT committed to git yet):**
  - **NUM_DEST2 = 4 → 6.** Added two zone-2 "crossing" destinations (`g_dest_names2` indices 4,5 = **Basketball Court**, **Food Court**) that lie beyond the danger-zone stretch and are reachable only via the full crossing sequence below. `dest2_Campus_TopLidar_Forward.cfg` now has 6 entries.
  - **Danger-zone CROSSING journey (`DzxState` state machine — common.hpp/main.cpp/control.cpp/ui.cpp):** picking Basketball Court/Food Court in the passenger flow drives: **leg 1** robot→Building A checkpoint (zone-1 corridor) → on arrival auto **switch map to "Danger Zone Fwd" + particle-filter relocalize** (seeded at the sub-zone entry marker) → **leg 2** drive the danger map's own recorded route; its existing `g_dzsub_active` sub-zone dead-reckoning handles the hard stretch → at the **sub-zone exit** auto **switch back to "Campus Top LiDAR" + relocalize** (seeded from the last point of `danger_zone_forward.csv`) → **leg 3** drive to the real destination (zone 2). States: `DZX_TO_CHECKPOINT / WAIT_ENTER_RELOC / IN_DZMAP / WAIT_EXIT_RELOC / NONE`. Each map switch is a hard stop (different, unrelated coordinate frames — no transform between Campus and DangerZone frames); operator/flow does NOT auto-drive between legs, Start Autonomous per leg. `g_drive_via_passenger` gates passenger-only transitions.
  - **KEY control.cpp fix:** the pre-existing "stop at sub-zone exit and demand manual relocalize" safety behavior (standalone Danger-Zone-map testing) now, **only during a crossing journey (`g_dzx_state==DZX_IN_DZMAP`)**, instead fires `g_dzx_pending_exit` to auto-hand-off. Hand-off is deliberately at the **sub-zone exit**, NOT the recorded route's own endpoint — `danger_zone_forward.csv` (the campus-frame relocalize seed) only spans the hard stretch (~24 m), so stopping further along would seed the search where there's no matching reference. Standalone testing (`DZX_NONE`) unchanged.
  - **`ekf.seeded` is STICKY — recurring bug class fixed in 3 more places this session.** It stays true from ANY past localization (even a stale pose from an earlier different-map test); it is NOT "currently localized." Correct live signal is `fus_state==STATE_FUSION_RUNNING`. Fixed: (1) friendly Step-3 panel follow-camera + car marker (`DrawDriveMapPanel`); (2) Step-2 pre-localize planning-pose sync (main.cpp `if(step==STEP_PLAN && fus_state==STATE_FUSION_RUNNING)`); (3) **the crossing "already past the danger zone → skip checkpoint" optimization** (`needsCrossing` in the Begin-Journey handler) — a stale zone-2 EKF pose on the bench made it silently cancel the ENTIRE crossing ("it was working all day, now gone"). Now gated on `fus_state==STATE_FUSION_RUNNING` (DrawDestinationPanel gained a `fus_state` param); on the bench / stale pose it always crosses.
  - **Auto-persist localized pose as estimated start:** the shared reloc success point (main.cpp) now writes the real localized pose into that map's `eststart_<ply>.cfg`. So wherever the cart is actually found becomes the next default start, instead of a hand-set point going stale. (An earlier session had a bad `-159,-5.9` value hand-saved into the forward map's eststart — corrected.)
  - **Step-3 split screen — friendly campus-photo panel (`DrawDriveMapPanel`, left half) beside the technical Step-3 view (right half).** Shows the planned route over the calibrated background photo + a live car marker; NO raw point cloud. Follow-camera centres on the live EKF pose (gated on fus_state), `VIEW_RADIUS_M = 60`. Route drawn as a translucent green **corridor band ±6 m each side** (perpendicular offset per point), path centre-line hidden. Car marker = plain **GOLD circle, r≈24 px** (no heading tick). **Destination pin** (red-pink, named) drawn at the picked destination's true location — during a crossing it's the stashed final destination, not the checkpoint; off-screen → an edge arrow points to it. During a crossing, remaining legs drawn as a **dashed** green line sliced from the full recorded drive (`pp_full_drive_slice`). Two-line title banner shows "Leg 1/3 / Leg 2/3 / Relocalizing…". `RenderTexture2D` note: functions with real mouse hit-testing (DrawPanelDrive, DrawStepBar) must draw to the real framebuffer at true coords — only the non-interactive DrawMapViewport goes through an offset texture.
  - **"Another journey?" card** (`DrawJourneyEndPanel`, `g_show_journey_end`) shown on arrival via the passenger flow; Yes → reopens Welcome (fresh flow), No → dismiss.
  - **Zone-2 slow-speed segment (per-map, `z2slow_<ply>.cfg`):** "Set Zone 2 Slow Segment" tool in Step 2 (2 clicks snapped to zone-2 corridor). Where set, control.cpp suppresses the `ZONE2_CRUISE_VOLTAGE` boost (`pp_zone2_slow_active()`), using `CRUISE_VOLTAGE` instead — for the reverse-map zone-2 stretch where high speed broke scan-matching. No-op on maps with no file.
  - **Reverse Danger-Zone path now reuses the forward route (reversed):** `path_dangerzone_rev.csv` rewritten as `path_dangerzone_fwd.csv` reversed point-by-point, `dzsub_rev.cfg` = same marker coords. Both Danger-Zone maps share `DangerZone_Test.ply` (same frame) so this is safe. Old rev files backed up as `*.bak_beforefwdreuse`. **Forward files never touched.**
  - **Unreachable-destination message wording:** "Please choose a reachable area." (passenger pax-grid, marked-unreachable cells). Diagnostic: a painted-but-invalid zone-2 cell now logs `[DIAG] zone2 dest #N rejected | map=… zone1_armed=… zone2_armed=… z2valid=…` to the command log.
  - **CAMERA PEDESTRIAN DETECTION (test feature — separate process, file-IPC):** `tools/pedestrian_detect.py` — standalone **MobileNet-SSD via cv2.dnn** (model in `tools/models/`, ~23 MB, CPU; HOG was too weak, PyTorch/CUDA download failed repeatedly so DNN was the pragmatic choice). Threaded per-camera, detection every Nth frame (`--detect-every 3`) so display stays smooth; single camera by default (`--cam1 -1`). Publishes `runtime_data/pedestrian_status.txt` = `STOP/CLEAR <unixtime>`. C++ side: Drive-panel "Pedestrian Detection ON/OFF" button (`ped_set_enabled()` in main.cpp `CreateProcessA`-launches/kills the Python), throttled 10 Hz status read in the render loop (stale >2 s = CLEAR, never strands the cart), atomics `g_ped_enabled/detected/paused/stale`. control.cpp: top-priority pause (`g_ctrl_state==CS_RUNNING && g_ped_detected`) — does NOT auto-resume; Drive panel shows a **Resume** button once the person clears. Requires opencv-python **4.10.0.84** (a 5.x preview build was missing `HOGDescriptor`/`CascadeClassifier`). Zone editing = `z`/`s` keys in the Python window.
    - **Depth awareness — CODE WRITTEN, NOT yet validated (deferred July 9, user didn't want to risk before the demo):** monocular box-height distance estimate (`f_px = (w/2)/tan(hfov/2)`, `dist_m = f_px·person_height / box_h`; args `--hfov` default 60°, `--person-height` 1.7 m, `--stop-distance` default **6.0 m**). STOP now = person **in-zone AND** `dist_m ≤ stop_distance`; each box shows `Xm YY%`. The C++ launch command does NOT pass `--stop-distance`, so it uses the 6 m default. **User wants "2 m only"** — trivial to set (change the Python default or add `--stop-distance 2` to the launch string, no rebuild needed for the Python default) BUT the distance is an uncalibrated monocular estimate: needs `--hfov` tuned to the real camera (stand at a known distance, check the label) and a person at 2 m may be partly out of frame (bad box height). **Lower-risk alternative recommended:** just shrink the stop-zone to the near ~2 m of ground (already-tested zone logic) instead of trusting metres. Not done yet.
  - **Estimated-start files reset to each map's start (data only, no code):** `eststart_Campus_TopLidar_Forward.cfg` = `0,0,~0` (lab/parking = `path_forward.csv` first point); `eststart_Campus_TopLidar_Reverse.cfg` = the reverse recording's first point `-158.2,-39.2` (its true start; later set to `0,0` at user request to eyeball it). NOTE the **auto-persist** (above) overwrites these on every successful localization — so before a demo the value may hold the last-found pose, not the map start; re-set the file (or re-pick the map, which reloads it) if a clean map-start is wanted. Also: the **reverse map's localize-panel photo calibration is cosmetically misaligned** — a correct world coord (e.g. `-158,-39`, the true recorded start) draws the search circle at the wrong spot on the photo. This is display-only and does NOT affect the particle-filter seed (which uses the world coord); do NOT drag the circle to "correct" it or the seed moves off the real start. Fix later via the reverse map's "Calibrate Map".
  - **COSMETIC BUG FIXED (identified July 9, reverted before the demo, re-applied and confirmed built July 10):** the friendly Step-3 green route band was rendering as a spraying fan instead of a clean corridor (confirmed on the reverse map). Cause: it offset the band perpendicular using each path point's stored `.yaw`, but the recorded top-LiDAR paths carry `yaw=0` (and path yaw elsewhere uses a different sign convention). Fix: compute the perpendicular from the actual segment direction `(b-a)` instead (convention-free, can't fan). Display-only; zero effect on driving/control/localization. Build verified clean.
  - **Test results:** **July 8 (cart):** forward crossing journey works well; reverse works up to Basketball Court; pedestrian detection fires (no depth yet). **July 9 (supervisor demo): PASSED / good.** Still NOT committed to git — commit deferred to a later session (per user; commit only at session end in logical, independently-revertable chunks).

- **Laptop-only dev week, day 1 (July 10, 2026 — no campus testing this week; builds clean, committed at session end):**
  - **Week plan (user-ordered):** (1) pedestrian depth/distance calibration [deferred — needs live camera + measured-distance point], (2) 60cm obstacle grid [DONE, step 1], (3) manual cell blocking [DONE], (4) dynamic re-routing around blocked cells while driving [NEXT], (5) thesis documentation [TOC done], (6) reverse-button auto-alignment [last].
  - **Obstacle/allowable-area grid (test feature — marking/visualization/stop only, NO avoidance planning yet):** 60 cm world-frame cells (`GRID_CELL_M`), existing only within `GRID_BAND_M = 3.5 m` of the loaded reference path (corridor-following sparse grid, not a full-map raster). "Obstacle Test Mode" (Step 2 panel; clicks work in Step 2 AND Step 3) cycles a cell none→pedestrian(orange)→static(red)→none. Pedestrian cells trigger the SAME top-priority pause/Resume flow as the camera detector when the robot is within `GRID_PED_STOP_RADIUS_M = 3 m` (`grid_check_pedestrian_stop()` in the 50 Hz loop → `g_grid_ped_stop`; Resume button now lives OUTSIDE the camera-enabled block since the triggers are independent). Static cells are forbidden-marked/visualized only. Runtime-only state + "Clear All Obstacles".
  - **Border/wall cells (separate concept, persisted):** "Auto-Generate Borders" drops forbidden cells along BOTH sides of the path at the band edge (`GRID_BAND_M − cell/2`; segment-direction perpendicular, NOT point .yaw — recorded paths carry yaw=0); "Edit: ON" click add/erase; saved per-map to `bordercells_<ply>.cfg`, auto-loaded on map switch/startup; purple render, distinct from orange/red obstacle cells. Mutually exclusive click-modes with Obstacle Test Mode. NOT yet consumed by the planner (enforcement = later phase with static-obstacle avoidance).
  - **Step-3 split-screen click-mapping fix (real bug):** every older map-click tool is Step-2-only; the obstacle/border handlers are the first to accept Step-3 clicks, where the technical map is an offset RenderTexture (leftW..leftW+rightMapW). Handlers now remap clicks into that texture-local space — plain `view2d.sw` math (correct in Step 2) resolves to wrong world coords in Step 3 and also wrongly accepts clicks on the left friendly panel.
  - **Pedestrian detector — YOLO backend added:** `--backend yolo` = YOLOv8n via ultralytics on the GTX 1650 (torch 1.13.1+cu116 + ultralytics installed and verified this session; yolov8n.pt auto-downloads, copy committed at repo root). Default remains `--backend mobilenet` (CPU, zero deps). Per-camera detector instances (no shared-model threading risk). Depth estimate still UNVALIDATED — first live run showed distances "totally wrong"; needs the one-point `--hfov` calibration (stand at measured distance, read the box label) before trusting `--stop-distance 2`.
  - **UI cleanup:** Step-2 zone-1+zone-2 destination "Go" lists collapsed behind one "Destinations >>" toggle (collapsed by default; auto-forced open during either Set-Destinations-on-Map flow); redundant "Build Path from Trajectory" button hidden (pp_run_pipeline already auto-fires on zone arming / Go buttons / map switch / Step-2 entry).
  - **Friendly-panel green route band fan bug — FIX APPLIED this time (was reverted before the demo):** perpendicular now from segment direction (b−a); confirmed built.
  - **Est-start files reset to 0,0** (both top-LiDAR maps) at user request; auto-persist behavior unchanged (they will drift to the last-found pose again after the next localization).
  - **Thesis work (thesis_project_V0.1/ now in repo):** user's LaTeX thesis draft imported; produced a researcher-grade suggested TOC (`suggested_table_of_contents.md/.html/.pdf`, Edge headless print pipeline since no LaTeX/pandoc on this machine). Framing: **teach-and-repeat campus road train** — the golf cart as leader + the BLDC robot as a **virtually hitched trailer** (supervisor's link): trail-following at arc-length offset reuses the Pure Pursuit framework, follower inherits the leader's corridor guarantee (perception-less), and the Hall-sensor quantization/bandwidth finding becomes the quantified bound on following precision (RQ5 + C6 + new Ch.7 + eval §9.7). Validation levels stated explicitly (coupling designed/simulated; follower actuation bench-validated; convoy field trial = future work). Commits us to producing the coupling-law design + simulation.

### Current focus: campus path tracking (module 3 — control loop)
The approach is to validate each module independently before full integration:
- Module 1 (steering) — **complete**
- Module 2 (localization) — **complete** (stationary + rotation tests passed, EKF fusion validated)
- **Module 3 (control loop) — COMPLETE** (campus stable June 22, 2026)
- Full integration — **complete**

### Confirmed working code state (June 22, 2026)
*(Note: line-number citations throughout this file's dated log entries drift as the file grows since June — verify by content/grep, not the cited line number, before trusting one.)*
- previewCurv window: `3.0f` (value still correct as of 2026-07-09; now at control.cpp:235, not :177 — the file has grown since this was written)
- LOOKAHEAD_MIN/NOM/MAX = 2.5/3.2/4.0m
- STEER_EMA_ALPHA = 0.45, STEER_SEND_MIN_DELTA = 1.0°
- RMSE blend gate: `blend *= 0.1f` when best_rmse > 0.25f
- Fix 2 active: IMU delta heading during driving (not EKF yaw)
- All CCW-positive sign fixes applied (PP body-frame, prediction stages, Bézier approach)
- All June 8 bug fixes applied

### Known issues / suspicions
- Many parameters likely still need tuning across all modules
- Pipeline delay (LiDAR age + actuator lag) needs measurement and tuning from first real drive log
- `CTRL_PREDICTION_DELAY_S` should be set to match measured avg pipeline delay from CSV logs

### Campus snake-wave oscillation (partially addressed — June 2026)
The system drives stably in the lab but oscillated with a snake-wave pattern on the campus path.
Root causes and status:

1. **EKF heading drift feeding PP (dominant cause) — FIXED June 5:**
   - `PoseLPF::ALPHA_YAW_SLOW` was 0.55 → now 0.10 (Fix 1)
   - PoseLPF now receives `imu_yaw_world` instead of EKF yaw during driving (Fix 2)
   - EKF/scan-match heading cannot reach PP at all — heading source is IMU only
   - **Status: resolved architecturally**

2. **Lookahead — partially restored (current: 2.5/3.2/4.0m):**
   - Was reduced to `1.5/2.5/3.5m` for lab (3m path); never intended for campus. Already partially restored to `2.5/3.2/4.0m` (common.hpp). June 4 CSV confirms lookahead never collapsed to 1.5m minimum during campus runs — not a cause of the June 4 failures.
   - If campus snaking persists after next test: consider raising `LOOKAHEAD_MIN=2.5→3.0, NOM=3.2→3.8, MAX=4.0→4.5`.

3. **Scan matcher position jitter — partially mitigated:**
   - EKF position (x,y) still from scan-match. PoseLPF `ALPHA_XY=0.70` absorbs 70% of each jump.
   - With heading now from IMU, position jitter causes CTE steps rather than heading steps — less harmful.
   - **Diagnosis if needed:** look for step-changes in `robot_x/robot_y` correlated with `steer_raw_deg` spikes.
   - **If still a problem:** reduce `ALPHA_XY` from 0.70 to 0.50.

4. **Curvature feedforward — DISABLED for normal campus driving; RE-ENABLED, but ONLY for danger-zone/sub-zone dead-reckoning replay (2026-07-07):**
   - Still disabled for normal (live-localized) campus driving — noisy live path curvature was adding parasitic steer / doubled steer / wall-strikes. Do not re-enable there until path curvature is smoothed or FF gain is attenuated.
   - **Exception:** control.cpp's `if(g_dr_active || g_dzsub_active)` branch (verified ~line 698–730) DOES apply curvature FF — safe there specifically because the danger-zone/sub-zone curve is a fixed, pre-recorded, deterministic replay (no live-path noise). Added to fix the actuator (5–8°/s) not keeping up with the recorded curve's heading-change rate, causing understeer wide on tight turns. Requires `g_ctrl_path[i].curvature` to be CCW-positive (fixed via `ctrl_splice_danger_zone`'s yaw computation) or it steers the wrong way. **Must live in the `g_dr_active/g_dzsub_active` branch specifically** — an earlier attempt placed near-identical code in the `else` branch and was dead code (that branch never runs while dead-reckoning is active).

5. **STEER_EMA_ALPHA — currently 0.45 (reduced from 0.60):**
   - Reduced from 0.60 for campus noise tolerance. If snaking persists after next test, reduce further to 0.35.

**How to confirm from CSV after campus test:**
- `steer_raw_deg` oscillating ±2–4° on a straight → lookahead too small
- `lookahead_m` collapsing to `LOOKAHEAD_MIN_M=1.5` → confirms lookahead is the problem
- `cte_m` cycling ±0.1–0.2m in sinusoidal pattern → snaking confirmed, not random noise
- Step-changes in `robot_x/robot_y` at ~20Hz → scan match pose jitter is a contributor
- `pipeline_delay_ms` average much larger than 20ms → prediction is off, update `CTRL_PREDICTION_DELAY_S`

**Parameters to tune after diagnosis (do not change until CSV confirms root cause):**
- `LOOKAHEAD_MIN/NOM/MAX` in `common.hpp` — primary lever
- `STEER_EMA_ALPHA` in `control.cpp` — secondary lever
- `STEER_SEND_MIN_DELTA` in `control.cpp` — consider 1.5° for campus
- `PoseLPF::ALPHA_XY` in `common.hpp` — if pose jitter confirmed as driver

### Key files to check for tuning
- `common.hpp` — all tuning constants in one place
- `control.cpp` → `ctrl_step()` — Pure Pursuit + prediction pipeline
- `communication.cpp` → `logger_write_csv()` — avg pipeline delay printed in CSV summary
- `golf_cart_v28_smooth.ino` → `STEER_SPEED_CRUISE`, `STEER_SPEED_CREEP`, `STEER_CREEP_ZONE`, `STEER_CMD_DEADBAND`, `STEER_DEADBAND_DEG`
- `tools/analyze_drive.py` — 4-subplot drive-run analysis (steer pipeline, tracking, CTE, send gate); detects new-format logs by `steer_raw_deg` column
- `tools/analyze_steer.py` — steering-module unit tests (Test A, B, sweep)
- `tools/analyze_loc.py` — localization stationary diagnostic; reads loc_test_stationary_*.csv; 3×2 figure + flagged report
- `tools/check_june4.py` — batch analysis of all 8 June 4 CSV runs; reports yaw_drift, peak_steer, lookahead stats, max CTE per run
- `tools/imu_drift_test.py` — standalone IMU drift test (laptop+IMU only, no cart/LiDAR); reads COM11 60s, reports total yaw drift. Run: `C:\Users\bios\AppData\Local\Programs\Python\Python38\python.exe tools/imu_drift_test.py`

## Build
```
cmake -B build -G "MinGW Makefiles"
cmake --build build
# exe auto-copies to runtime_data/GolfCartControl.exe
```
Run from `runtime_data/` directory (needs reference_path.csv, PLY maps, etc.)

## Notes for developers
- Never edit codegen/ — MATLAB-generated C, treat as read-only
- common.hpp is the single source of truth for constants — change them there, not inline
- The control loop thread runs at exactly 50 Hz using sleep_until — don't add blocking calls
- Serial prints on ESP32 are suppressed while stepper is moving (blocks ~1.5ms each at 115200)
- STEER command negates the angle on ESP32 side to reconcile PC convention vs hardware polarity
- **When reviewing or auditing code: trust only the actual code logic, never comments.**
  Comments in this codebase are sometimes wrong or outdated (confirmed example: sensor_calibration_result.h
  said "right-turn = positive Z" but the actual physical behavior and code are CCW-positive throughout).
  Always verify behavior by tracing the code path, not by reading descriptions above functions.

## Angle / sign convention (verified against hardware June 2026)
- **Left turn (CCW) = POSITIVE yaw** everywhere: IMU angle.z, EKF x[2], scan-matcher output, path .yaw
- **Right turn (CW) = NEGATIVE yaw** everywhere — confirmed physically on the cart
- Pure Pursuit steer command: **positive = steer LEFT**, **negative = steer RIGHT** (PC side)
- ESP32 negates the steer angle: positive PC steer → negative steps → physically LEFT turn ✓
- `wrapA()` is for RADIANS only (wraps at ±π). Never call it on a degrees value.
- IMU angle is always in DEGREES. EKF, path yaw, robot pose are always in RADIANS.
- Steer command pipeline (PP output → EMA → send gate → UDP) is always in DEGREES.
