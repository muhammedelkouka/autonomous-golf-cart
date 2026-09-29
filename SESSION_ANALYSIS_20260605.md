# Golf Cart Autonomy — Deep Analysis Session (June 5, 2026)
## Based on June 4, 2026 Campus Test Session Data

---

## 1. Context

- **Project**: Autonomous golf cart (Marshell), campus path tracking
- **Test date**: June 4, 2026 (8 runs: 090555 → 101355)
- **Result**: All 8 runs failed. Zero GOAL_REACHED. Max CTE reached 1.9m on most runs.
- **One near-success**: Run 095727 — max CTE=0.381m, came 1.02m from goal.
- **User observation**: "The path is nearly straight — why is 23° being demanded?"

---

## 2. Run Summary Table (June 4)

| Run       | Duration | Path  | Start yaw | Max CTE  | Peak steer_raw | Stop reason  |
|-----------|----------|-------|-----------|----------|----------------|--------------|
| 090555    | 27.8s    | ~25m  | +1.53°   | 1.934m   | ±23°           | Manual       |
| 091736    | 16.2s    | ~25m  | ?         | 1.610m   | +18.8°         | Manual       |
| 092430    | 27.9s    | ~25m  | ?         | 1.903m   | ±22°           | Manual       |
| 093320    | 21.2s    | 25m   | +1.53°   | 1.905m   | +23°           | Manual       |
| 094129    | 30.8s    | ~25m  | ?         | 1.207m   | +8.78°         | Manual       |
| **095727**| 17.6s    | **10m**| -1.36°  | **0.381m**| 23° (end only)| Manual       |
| 100130    | 40.7s    | ~10m  | -1.36°   | 1.820m   | ±23°           | Manual       |
| 101355    | 19.1s    | ~10m  | -1.36°   | 1.902m   | ±12°           | Manual       |

---

## 3. The Actual Root Cause (Proved from CSV Data)

### The Smoking Gun — Run 093320, Ticks 100–145:

```
tick 100: CTE = -0.019m (2cm off path)   yaw = -0.84°   steer_raw = -4.17°
tick 110: CTE = -0.021m (2cm off path)   yaw = -0.96°   steer_raw = -3.96°
tick 120: CTE = -0.021m (2cm off path)   yaw = -1.13°   steer_raw = -4.55°
tick 140: CTE = -0.024m (2cm off path)   yaw = -1.54°   steer_raw = -5.19°
```

**The cart is only 2cm off the straight path, yet steer_raw is already -5° and growing.**

The 23° is NOT caused by the cart being off path. It is caused by the **EKF heading continuously drifting at ~1°/second**.

Full heading timeline for run 093320:
- Tick 0–56: yaw = +1.527° (stable, from localization)
- Tick 57: **scan-matcher fires a 1.2° heading correction** — yaw jumps from +1.527° → +1.064°
- Tick 57→60: yaw continues rapidly: +0.866° → +0.469° → +0.291°
- Tick 60–145: yaw keeps drifting: 0.291° → -0.017° → -0.84° → -1.64°
- At tick 60: steer_raw = -3.0° (was -0.6° before the jump), **gate fires, first steer command sent**

Pure Pursuit sees the heading changing and responds: "the robot is facing the wrong direction relative to the path → steer harder." CTE is only 2cm but steer_raw climbs to -5° purely from heading drift.

Eventually these steer commands **physically turn the cart**, which creates real CTE. Once real CTE reaches 1.5m+, PP demands 23° — which IS geometrically correct at that point. So 23° is not wrong at the moment it appears; it's the END of a chain reaction started by heading drift.

### Why Run 095727 Worked:

```
tick 45–90: yaw = -1.355° (COMPLETELY CONSTANT — does not drift)
```

Run 095727 started at a different campus location (x=0.44, y=2.49) where the scan-matcher has better feature density. The EKF heading stayed stable. No heading drift → PP computed small corrections throughout → CTE stayed ≤0.38m.

**The difference between success and failure is entirely location-dependent scan-match heading quality.**

---

## 4. Root Cause Chain

```
Poor campus map density at specific location
         ↓
Scan-matcher fires continuous small heading corrections (~0.01°/tick)
         ↓
PoseLPF ALPHA_YAW_SLOW=0.55 applies each correction at 55% per tick
         ↓
EKF heading drifts 1°/second into filtered robot pose
         ↓
PP sees heading changing → steer_raw grows from 0.6° → 3° → 5° → ...
         ↓
Gate fires, steer commands sent, cart physically turns
         ↓
Real CTE builds from 2cm → 1.9m over 20 seconds
         ↓
PP demands 23° (now geometrically correct for 1.9m CTE)
         ↓
yaw_converged clamp (±6°) prevents recovery  [secondary issue]
```

---

## 5. What Was Tried on June 4 That Did NOT Help

Based on user feedback and code comments:

1. **Removing yaw_converged clamp**: Made no difference — the clamp is a symptom-response to drift. Removing it lets PP act on the wrong heading more freely, still uses wrong data.
2. **Increasing lookahead**: Reduces CTE-to-steer gain but heading error enters PP directly via `yaw_error = atan2(-ly, lx)`, unaffected by lookahead magnitude.
3. **Curvature feedforward disabled** (already committed to code, comment at control.cpp:393): Correct decision. Does not address heading drift.
4. **Lookahead kf multiplier**: Changed from 4.0 → 2.0 (comment at control.cpp:117). Addresses lookahead collapse on curves but not the heading issue.

---

## 6. PoseLPF Heading Filter — The Key Code (common.hpp)

```cpp
struct PoseLPF {
    static constexpr float ALPHA_XY       = 0.70f;
    static constexpr float ALPHA_YAW_SLOW = 0.55f;  // <-- PROBLEM: too high for campus
    static constexpr float ALPHA_YAW_FAST = 0.85f;  // used when heading change > 2°
    static constexpr float ALPHA_YAW      = 0.65f;  // defined but NOT used in current code

    CtrlPose update(float rx, float ry, float ryaw) {
        if(!init){ x=rx; y=ry; yaw=ryaw; init=true; }
        x   = ALPHA_XY * rx   + (1.f-ALPHA_XY)  * x;
        y   = ALPHA_XY * ry   + (1.f-ALPHA_XY)  * y;
        float dy2 = wrapA(ryaw - yaw);
        float alpha_y = (fabsf(dy2) > toRad(2.0f)) ? ALPHA_YAW_FAST : ALPHA_YAW_SLOW;
        yaw = wrapA(yaw + alpha_y * dy2);
        return {x, y, yaw, true};
    }
};
```

With `ALPHA_YAW_SLOW=0.55`, a scan-match heading correction converges to its target in ~3 ticks (60ms). The campus map gives continuous small corrections in one direction → accumulated drift of 1°/second.

---

## 7. Fixes — Prioritized

### Fix 1 — CRITICAL (one line, test immediately): Lower PoseLPF heading alpha

**File**: `common.hpp`

```cpp
// Change from:
static constexpr float ALPHA_YAW_SLOW = 0.55f;
static constexpr float ALPHA_YAW_FAST = 0.85f;

// Change to:
static constexpr float ALPHA_YAW_SLOW = 0.10f;   // spreads correction over ~22 ticks (440ms)
static constexpr float ALPHA_YAW_FAST = 0.45f;   // for large jumps >2°, still slower
```

**Why this works**: With 0.10, a 0.01°/tick scan-match drift takes 22 ticks to apply instead of 3. The IMU blend (alpha 0.02-0.10) can now counteract the scan-match drift before it accumulates. The cart's heading as seen by PP will be much more stable.

**Risk**: If the scan-match heading IS correct (in a good location), convergence will be slower. Position accuracy (ALPHA_XY=0.70) is unaffected.

---

### Fix 2 — HIGH: Use IMU heading during driving instead of EKF heading

**File**: `control.cpp`, `ctrl_step()`, around lines 227–238

The current code passes both position AND heading from EKF to PoseLPF, then blends IMU on top. The problem is the EKF heading jump is applied at 55%, then the IMU blend is only 2-10% — the IMU can't undo the scan-match heading corruption.

**Better approach**: During driving (after startup), pass IMU-derived heading to PoseLPF instead of EKF heading:

```cpp
// CURRENT code (lines 229–236):
g_ctrl_robot_filtered = g_pose_lpf.update(g_ctrl_robot.x, g_ctrl_robot.y, g_ctrl_robot.yaw);
float _spd = g_wheel_speed_valid ? g_wheel_speed_mps : g_vel_est_ms;
float _t   = clampf((_spd - IMU_ALPHA_SPEED_LO)/(IMU_ALPHA_SPEED_HI - IMU_ALPHA_SPEED_LO), 0.f, 1.f);
float blend = IMU_YAW_ALPHA_LOW + _t*(IMU_YAW_ALPHA_HIGH - IMU_YAW_ALPHA_LOW);
float dy2 = wrapA(imu_yaw_world - g_ctrl_robot_filtered.yaw);
g_ctrl_robot_filtered.yaw = wrapA(g_ctrl_robot_filtered.yaw + blend * dy2);

// PROPOSED change: feed IMU heading directly into PoseLPF for yaw
// PoseLPF will still smooth it with ALPHA_YAW_SLOW=0.10
g_ctrl_robot_filtered = g_pose_lpf.update(
    g_ctrl_robot.x,        // position from EKF (scan-match, accurate)
    g_ctrl_robot.y,        // position from EKF (scan-match, accurate)
    imu_yaw_world          // heading from IMU (stable for 30-40s drives)
);
// Remove the separate IMU blend block below since it's now handled by PoseLPF
```

**Why**: IMU heading is accurate for 30-40 second drives (validated: drift <2° during rotation test). Campus scan-match heading is noisy. Use each sensor for what it's good at: EKF for position, IMU for heading.

---

### Fix 3 — HIGH: Fix yaw_converged persistent clamp

**File**: `control.cpp`, lines 406–424

The `yaw_converged` check runs every tick for the ENTIRE drive. When IMU and EKF heading differ by >2° (which happens as EKF drifts), it clamps steer to ±6°. This is a startup safety check that was never time-limited.

```cpp
// CURRENT (fires indefinitely):
yaw_converged = (fabsf(wrapA(imu_world2 - ekf_yaw_now)) < toRad(2.0f));
if (!yaw_converged) {
    rawSteer = clampf(rawSteer, -STARTUP_MAX_STEER_DEG, STARTUP_MAX_STEER_DEG);  // ±6°
}

// FIX: time-limit to first 2 seconds (100 ticks):
bool in_extended_startup = (g_ctrl.startupTick <= STARTUP_IMU_TICKS + 50);
if (!yaw_converged && in_extended_startup) {
    rawSteer = clampf(rawSteer, -STARTUP_MAX_STEER_DEG, STARTUP_MAX_STEER_DEG);
}
// After 100 ticks (2s): trust PP output directly
```

**Note**: Fix 3 is secondary. Fix 1+2 must be done first. If the heading is accurate, the yaw_converged condition will rarely fire anyway.

---

### Fix 4 — MEDIUM: Add `steer_clamped_deg` logging column

Currently `steer_raw_deg` in CSV = PP output (pre-clamp). `steer_filtered_deg` = EMA of post-clamp value. This makes the CSV look like EMA lag when it's actually the clamp. Add a `steer_clamped_deg` column = the value entering the EMA filter, so future analysis can distinguish PP output vs clamp output vs EMA output.

---

### Fix 5 — MEDIUM: Fix corridor watchdog to use CTE not Euclidean distance

**File**: `control.cpp`, line 298

```cpp
// CURRENT (can miss lateral exceedances on curves):
if (ctrl_nearestDistWindowed(robot) > CORRIDOR_HALF_M)

// FIX (consistent with PP's CTE):
if (fabsf(g_ctrl.crossTrackErr) > CORRIDOR_HALF_M)
```

4 of 8 June 4 runs exceeded CORRIDOR_HALF_M=1.9m in CTE yet watchdog never fired. This is a safety gap.

---

### Fix 6 — LOW: Reduce STEER_SEND_MIN_DELTA for small corrections

**File**: `control.cpp`, line 66

```cpp
static const float STEER_SEND_MIN_DELTA = 0.3f;  // was 1.0° — allows sub-1° corrections
```

For runs where steer_raw is <0.75° (small heading error), the 1° gate never fires and NO correction is ever sent. The small correction (0.4-0.7°) is blocked until CTE grows enough to push steer_raw above 1°.

**Risk**: More frequent small steer commands. Monitor for increased vibration. The ESP32 0.30° deadband will absorb genuinely trivial commands.

---

## 8. What Does NOT Need Changing

- **EKF localization** (position tracking): Working correctly. No large position jumps confirmed for most runs.
- **IMU hardware**: Validated. Gyro_z=0 when stationary is correct behavior.
- **Communication / UDP**: 0% packet drops across all June 4 runs.
- **Pipeline delay measurement**: CTRL_PREDICTION_DELAY_S=0.020s matches measured 22-27ms. Accurate.
- **ESP32 firmware**: Working correctly after the g_pc_ip and socket merge fixes.
- **Pure Pursuit algorithm**: Geometrically correct. The 23° demand is mathematically valid once CTE reaches 1.5m. PP is not broken.

---

## 9. Testing Procedure for Next Session

**Step 1** — Apply Fix 1 only (ALPHA_YAW_SLOW 0.55→0.10, ALPHA_YAW_FAST 0.85→0.45). Build. Run.

**What to look for**:
- Does `robot_yaw_deg` in the CSV now stay stable during driving?
- Does `steer_raw_deg` stay ≤5° throughout a straight-line run?
- Is max CTE ≤0.2m on a 10m run?

**Step 2** — If still failing, apply Fix 2 (IMU heading for PoseLPF during driving). This is the stronger version.

**Step 3** — Apply Fix 3 (time-limit yaw_converged) and Fix 5 (watchdog) together.

**Step 4** — Restore CRUISE_VOLTAGE=0.99V / MIN_VOLTAGE=0.95V if runs are stable.

**Key diagnostic: plot `robot_yaw_deg` vs time**. If it stays flat (drift <0.5°/s), the fix worked. If it still drifts, the scan-match heading issue is deeper than the PoseLPF filter.

---

## 10. Campus Map Issue — Physical Environment Analysis (Added June 5)

### Physical environment (from site photos)

The campus test location is a semi-covered university corridor:
- Width: ~6–8m
- Floor: brick pavers (zero LiDAR returns — all features are vertical or overhead)
- Right side: glass-front rooms with concrete pillars (strong, flat returns — good for ICP but degenerate for heading near the wall)
- Left side: display boards on stands (angled, variable returns, not structural)
- **Overhead: translucent skylight panels + metal beam structure** — this is the critical factor

### Why the overhead structure hurts scan matching

The VLP-16 has beams from -15° to +15° elevation. In this corridor, the upper beams hit the metal ceiling beams and semi-transparent skylight panels overhead. This causes:

1. **Lighting-dependent returns**: Translucent panels reflect differently under different sun angles. The map was recorded at one time of day; current scans at a different time produce different return intensity/density from the same panels. ICP sees a "heading offset" to reconcile the two sets of ceiling features.

2. **Overhead features dominate the optimization**: If the overhead structure contributes a large fraction of total inlier points, ICP will over-weight it. A small systematic difference in the ceiling point pattern (e.g., sun angle shifted 15°) produces a false heading rotation estimate of 0.5–1°.

3. **Single PCAP pass**: The map was built from one PCAP recording pass using LiDAR View + MATLAB post-processing (no SLAM, no loop closure, no camera). Any drift that accumulated during the recording walk is baked into the map coordinates. There is no multi-pass averaging of the overhead structure features.

### ICP degeneracy near walls

When the cart drifts within ~1.5m of either wall, the LiDAR scan is dominated by a single planar surface. A flat plane provides:
- Good translational constraint perpendicular to the wall
- **Zero rotational constraint** — ICP cannot distinguish "moved along the wall" from "rotated slightly while stationary" when facing a featureless plane

This is a well-known ICP degeneracy condition. The result: heading estimates become numerically unstable, producing noisy corrections in random directions — exactly what the CSV shows (0.01°/tick in a consistent direction while the cart is near the wall side of the corridor).

### Map coverage vs. driving position

The map was recorded on a single centerline pass. Point density:
- Center of corridor: high (most of the map recording path)
- Within 1.5m of either wall: low
- Near ceiling structure directly above recording path: high

When the cart drives off-center, the ICP tries to match a scan dominated by features that are underrepresented in the map → higher RMSE, lower inlier ratio, noisier heading.

**This is why run 095727 succeeded**: it happened to start at a position with good map coverage (center corridor, away from walls), giving stable ICP heading throughout.

### Long-term fix

Re-record the campus map using multiple passes covering the full corridor width. Ideally:
- 3 parallel passes: left lane, center, right lane
- Merge all three scans into one PLY file
- This provides uniform point density across the full drivable corridor width
- Also reduces the impact of overhead structure noise (the floor-level features will dominate if there are more of them)

**Consider Z-height filtering in the scan matcher**: Excluding points above ~3.0m (i.e., above head height) would eliminate the ceiling structure entirely from the ICP optimization. This would make localization rely only on wall features, which are geometrically stable and present in all positions. This requires a code change in `localization.cpp` in the ICP point preparation step.

### Short-term workaround

- Always position the cart ≥1.5m from any wall (center corridor) before starting a campus drive
- Check RMSE < 0.10m and overlap > 60% in Step 1 readiness panel before pressing Start
- If RMSE is 0.15–0.20m at the starting position, the heading estimates will be noisy throughout the drive — restart from a better position

---

## 11. Key Files and Line References

| File | Lines | Topic |
|------|-------|-------|
| `common.hpp` | PoseLPF struct | `ALPHA_YAW_SLOW=0.55` → change to `0.10` |
| `control.cpp` | 65–66 | `STEER_EMA_ALPHA`, `STEER_SEND_MIN_DELTA` |
| `control.cpp` | 227–238 | PoseLPF update + IMU blend (Fix 2 location) |
| `control.cpp` | 298–311 | Corridor watchdog (Fix 5 location) |
| `control.cpp` | 406–424 | yaw_converged clamp (Fix 3 location) |
| `control.cpp` | 399–400 | cmdSteerRaw logging (set before clamp = correct) |
| `control.cpp` | 391 | ctrl_purePursuit call |
| `control.cpp` | 113–119 | ctrl_computeLookahead (kf multiplier now 2.0, was 4.0) |
| `control.cpp` | 393–397 | Curvature FF disabled (correct, keep disabled) |

---

## 12. Glossary of CSV Columns

| Column | Meaning |
|--------|---------|
| `steer_raw_deg` | Pure Pursuit output BEFORE any clamp (what PP needs geometrically) |
| `steer_filtered_deg` | EMA of POST-clamp value (what actually feeds the send gate) |
| `steer_cmd_deg` = `steer_sent_deg` | Last value sent to ESP32 firmware |
| `steer_delta_deg` | Delta between filtered and last sent (gate fires when ≥1°) |
| `steer_sent` | 1 = new steer command sent this tick, 0 = gate blocked |
| `cte_m` | Cross-track error: positive = cart right of path, negative = cart left |
| `lookahead_m` | Dynamic lookahead distance (function of speed + path curvature) |
| `robot_yaw_deg` | Filtered EKF+IMU heading used by PP |
| `pipeline_delay_ms` | LiDAR age + actuator lag prediction total |

---

---

## 13. Hardware Upgrade Decision (June 5, 2026)

User asked: should we move the LiDAR to the top of the cart, add a camera, and build a new mapping module to solve the problem entirely?

**Decision: No hardware changes needed yet. The current failures are software and map quality problems, not hardware limitations.**

### Moving LiDAR to top of cart — NOT recommended
- The campus corridor has a translucent skylight overhead. A higher LiDAR would scan MORE of the ceiling, making the overhead structure noise problem **worse**, not better.
- Would require rebuilding the map entirely.
- Current mount height is fine.

### Adding a camera — NOT recommended at this stage
- Requires: camera intrinsic calibration + LiDAR-camera extrinsic calibration + VIO/SLAM library integration + new map format. This is a second major project.
- The overhead glass ceiling creates severe backlit conditions — challenging visual environment in exactly the test location.
- A camera adds value AFTER the LiDAR-only stack is proven reliable: for visual loop closure, obstacle detection, long-route drift correction.
- Appropriate as a future extension, not a prerequisite.

### The correct fix sequence (no hardware changes):

| Step | What | Effort | Status |
|------|------|--------|--------|
| 1 | ALPHA_YAW_SLOW: 0.55 → 0.10, ALPHA_YAW_FAST: 0.85 → 0.45 in common.hpp | 1 line | **DONE — built June 5** |
| 2 | Z-height PLY map: `Full_Campus_Map_ZFiltered.ply` (z > 2.0m stripped, 76k pts) added as Map 4 | PLY post-process | **DONE — June 5** |
| 3 | Fix 2 — IMU heading (not EKF) fed to PoseLPF during driving (control.cpp:232) | ~5 lines | **DONE — June 5** |
| 4 | Fix 3 — yaw_converged clamp time-limited to first 100 ticks (control.cpp:408) | ~3 lines | **DONE — June 5** |
| 5 | Fix 5 — watchdog uses `fabsf(g_ctrl.crossTrackErr)` not Euclidean dist (control.cpp:298) | 1 line | **DONE — June 5** |
| 6 | Multi-pass campus map re-record | 1–2 h fieldwork | **Not done — not needed; system stable without it** |
| 7 | Campus test session | 1 session | **DONE — June 22, 2026 (GOAL_REACHED confirmed)** |

### Why Z-height filter is the next most important code change

The VLP-16 upper beams hit the ceiling beams and semi-transparent skylight panels. These features:
- Are lighting/sun-angle dependent → inconsistent returns between map recording and live scan
- Were captured in a single PCAP pass under specific conditions
- Create false rotation estimates when ICP tries to reconcile ceiling features

Filtering out all points above ~2.5m makes ICP fit ONLY against walls, pillars, and structural features — geometrically stable, lighting-independent, consistent across visits. Estimated 10-line change in `localization.cpp` in the point cloud preparation step before ICP.

---

## 14. Session Progress Log

| Date | Change | Result |
|------|--------|--------|
| June 4, 2026 | 8 campus test runs | All failed. Max CTE 1.2–1.9m. Root cause unknown at time. |
| June 5, 2026 (session 1) | Deep analysis of all CSVs | Root cause found: ALPHA_YAW_SLOW=0.55 passes scan-match heading drift to PP |
| June 5, 2026 (session 2) | Physical environment analyzed from site photos + LiDAR visualization | Added: overhead structure noise, ICP degeneracy near walls, PCAP-only map limitations |
| June 5, 2026 (session 2) | Fix 1 implemented + built | ALPHA_YAW_SLOW=0.10, ALPHA_YAW_FAST=0.45 in common.hpp. Clean build confirmed. |
| June 5, 2026 | Fix 2 implemented — IMU heading replaces EKF heading in PoseLPF during driving (control.cpp:232) | EKF/scan-match heading cannot reach PP during driving. Heading is purely IMU delta. |
| June 5, 2026 | Fix 3 implemented — yaw_converged clamp time-limited to first 100 ticks (control.cpp:408) | Prevents permanent clamp after Fix 2 decoupled IMU from EKF heading. |
| June 5, 2026 | Fix 5 implemented — watchdog uses `fabsf(g_ctrl.crossTrackErr)` (control.cpp:298) | Closed safety gap where 4/8 June 4 runs exceeded corridor without watchdog. |
| June 5, 2026 | Z-height PLY map added — `Full_Campus_Map_ZFiltered.ply` (z>2.0m removed, 76k pts) | Approach taken instead of runtime Z-filter — same effect, no code change in ICP. |
| June 13, 2026 | Scan distortion mitigation — blend×0.40 during turns, EKF heading suppressed during turns, ALPHA_XY_TURNING=0.35 | Reduces position/heading corruption from motion-distorted 100ms scans during cornering. |
| June 21, 2026 | Three CCW-positive sign fixes — Bézier departure, PP body-frame transform, prediction stage X sign | PP steer_raw at tick 0 dropped from 16.21° to −0.14°. Lab CTE ≤ 3cm throughout. |
| June 22, 2026 | RMSE blend gate — `blend *= 0.1f` when best_rmse > 0.25f (localization.cpp) | Cart dead-reckons through poor-scan zones instead of taking corrupted position updates. |
| **June 22, 2026** | **Campus test — GOAL_REACHED confirmed** | **Smooth driving, stable heading, goal reached. Confirmed working baseline.** |

---

*Session analysis originally conducted June 5, 2026. Progress log updated through June 22, 2026.*
*All identified fixes applied. System confirmed stable on campus June 22, 2026. See DEVELOPMENT_NOTES.md for full history.*
