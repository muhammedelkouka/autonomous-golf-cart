#!/usr/bin/env python3
"""
Localization stationary diagnostic tool.

Usage:
    python analyze_loc.py loc_test_stationary_20260601_120000.csv [...]

Reads loc_test_stationary_*.csv files produced by loc_test_run_stationary().
The first line is a '#' comment carrying calibration values; it is parsed
separately and excluded from the data frame.

Diagnostic report covers:
  - Calibration: gyro bias per axis, accel gravity axis detection
  - IMU noise:   per-axis stddev of accel and gyro
  - EKF stability: position drift, yaw drift
  - Velocity: max |ekf_vx|, |ekf_vy| (should be ~0)
  - Scan match: mean/std RMSE, mean inlier ratio
  - Streaming rates: LiDAR Hz, IMU Hz
  - Stationary flag coverage

Saves a 3x2 figure as <csvname>_loc_analysis.png.
"""

import sys
import re
import math
import pathlib
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _flag(cond: bool, msg: str) -> str:
    return f"  *** FLAG: {msg}" if cond else ""


def _parse_calib_comment(line: str) -> dict:
    """Extract key=value pairs from the # comment line."""
    result = {}
    for m in re.finditer(r'(\w+)=([-\d.eE+]+)', line):
        result[m.group(1)] = float(m.group(2))
    return result


# ---------------------------------------------------------------------------
#  Core analysis
# ---------------------------------------------------------------------------

def analyze_loc(path: str) -> None:
    name = pathlib.Path(path).name
    stem = pathlib.Path(path).stem

    # Read raw lines to extract the # comment before loading with pandas
    calib = {}
    with open(path, encoding="utf-8") as fh:
        first = fh.readline()
        if first.startswith("#"):
            calib = _parse_calib_comment(first)

    # Load data (skip the comment line)
    df = pd.read_csv(path, comment="#")

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    t        = df["time_s"].values
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    n        = len(df)

    print(f"\n=== {name}  [LOC STATIONARY] ===")
    print(f"  Duration: {duration:.1f} s   Rows: {n}   Rate: {n/duration:.1f} Hz"
          if duration > 0 else f"  Duration: 0 s   Rows: {n}")

    # ── Calibration ──────────────────────────────────────────────────────────
    print("\n  -- Calibration --")
    if calib:
        gb = [calib.get("gyro_bias_x", float("nan")),
              calib.get("gyro_bias_y", float("nan")),
              calib.get("gyro_bias_z", float("nan"))]
        am = [calib.get("accel_mean_x", float("nan")),
              calib.get("accel_mean_y", float("nan")),
              calib.get("accel_mean_z", float("nan"))]
        for i, axis in enumerate("XYZ"):
            flag = _flag(abs(gb[i]) > 0.01, f"gyro_bias_{axis} |{gb[i]:.4f}| > 0.01 rad/s")
            print(f"  gyro_bias_{axis}: {gb[i]:+.4f} rad/s{flag}")
        print()
        grav_idx = int(np.argmax(np.abs(am)))
        grav_axis = "XYZ"[grav_idx]
        for i, axis in enumerate("XYZ"):
            print(f"  accel_mean_{axis}: {am[i]:+.4f} m/s²")
        if grav_idx != 2:
            print(f"  *** FLAG: gravity on axis {grav_axis} (expected Z) — "
                  "check IMU orientation or calibration")
        else:
            print(f"  Gravity detected on Z axis ({am[2]:+.2f} m/s²) — OK")
    else:
        print("  (no # comment line found — calibration values unavailable)")

    # ── IMU noise ────────────────────────────────────────────────────────────
    print("\n  -- IMU noise (stddev) --")
    for col, label, thresh, unit in [
        ("imu_acc_x",  "acc_x",  None,  "m/s²"),
        ("imu_acc_y",  "acc_y",  None,  "m/s²"),
        ("imu_acc_z",  "acc_z",  None,  "m/s²"),
        ("imu_gyro_x", "gyro_x", 0.02, "rad/s"),
        ("imu_gyro_y", "gyro_y", 0.02, "rad/s"),
        ("imu_gyro_z", "gyro_z", 0.02, "rad/s"),
    ]:
        std = float(df[col].std()) if col in df.columns else float("nan")
        flag = _flag(thresh is not None and std > thresh,
                     f"{label} std {std:.4f} > {thresh} {unit} — NOISY")
        print(f"  std({label}): {std:.4f} {unit}{flag}")

    # ── EKF stability ─────────────────────────────────────────────────────
    print("\n  -- EKF stability --")
    ekf_x_std = float(df["ekf_x"].std())
    ekf_y_std = float(df["ekf_y"].std())
    yaw_first = float(df["ekf_yaw_deg"].iloc[0])  if n > 0 else float("nan")
    yaw_last  = float(df["ekf_yaw_deg"].iloc[-1]) if n > 0 else float("nan")
    yaw_drift = abs(yaw_last - yaw_first)

    print(f"  std(ekf_x): {ekf_x_std:.4f} m"
          + _flag(ekf_x_std > 0.05, f"ekf_x std {ekf_x_std:.4f} m > 0.05 m — drifting"))
    print(f"  std(ekf_y): {ekf_y_std:.4f} m"
          + _flag(ekf_y_std > 0.05, f"ekf_y std {ekf_y_std:.4f} m > 0.05 m — drifting"))
    print(f"  yaw drift (first→last): {yaw_drift:.3f}°"
          + _flag(yaw_drift > 2.0, f"yaw drift {yaw_drift:.2f}° > 2° over run"))

    # ── Velocity ──────────────────────────────────────────────────────────
    print("\n  -- EKF velocity (should be ~0) --")
    max_vx = float(df["ekf_vx"].abs().max())
    max_vy = float(df["ekf_vy"].abs().max())
    print(f"  max |ekf_vx|: {max_vx:.4f} m/s"
          + _flag(max_vx > 0.1, f"max |vx| {max_vx:.3f} > 0.1 m/s"))
    print(f"  max |ekf_vy|: {max_vy:.4f} m/s"
          + _flag(max_vy > 0.1, f"max |vy| {max_vy:.3f} > 0.1 m/s"))

    # ── Scan match ────────────────────────────────────────────────────────
    print("\n  -- Scan match --")
    rmse_mean = float(df["scan_rmse"].mean())
    rmse_std  = float(df["scan_rmse"].std())
    ir_mean   = float(df["inlier_ratio"].mean())
    print(f"  scan_rmse:    mean={rmse_mean:.4f} m  std={rmse_std:.4f} m")
    print(f"  inlier_ratio: mean={ir_mean:.3f}")

    # ── Streaming rates ───────────────────────────────────────────────────
    print("\n  -- Streaming rates --")
    if duration > 0:
        lidar_delta = int(df["lidar_sweeps"].iloc[-1]) - int(df["lidar_sweeps"].iloc[0])
        imu_delta   = int(df["imu_frames"].iloc[-1])   - int(df["imu_frames"].iloc[0])
        lidar_hz    = lidar_delta / duration
        imu_hz      = imu_delta   / duration
        print(f"  LiDAR: {lidar_hz:.1f} Hz"
              + _flag(lidar_hz < 3.0, f"LiDAR {lidar_hz:.1f} Hz — too slow or stalled"))
        print(f"  IMU:   {imu_hz:.1f} Hz"
              + _flag(imu_hz < 50.0, f"IMU {imu_hz:.1f} Hz — too slow or stalled"))
    else:
        print("  (cannot compute rates — duration=0)")

    # ── Stationary flag ───────────────────────────────────────────────────
    print("\n  -- Stationary flag --")
    stat_pct = 100.0 * df["is_stationary"].sum() / n if n > 0 else 0.0
    print(f"  is_stationary=1 for {stat_pct:.1f}% of ticks"
          + _flag(stat_pct < 90.0, f"only {stat_pct:.0f}% stationary — check ZUPT thresholds"))

    # ── 3×2 Figure ────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), tight_layout=True)
    fig.suptitle(f"{name}  [localization stationary]", fontsize=11)

    # 1. Accel x/y/z
    ax = axes[0, 0]
    ax.plot(t, df["imu_acc_x"], linewidth=1.0, label="acc_x")
    ax.plot(t, df["imu_acc_y"], linewidth=1.0, label="acc_y")
    ax.plot(t, df["imu_acc_z"], linewidth=1.0, label="acc_z")
    ax.set_title("IMU accelerometer (m/s²)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m/s²")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    # 2. Gyro x/y/z
    ax = axes[0, 1]
    ax.plot(t, df["imu_gyro_x"], linewidth=1.0, label="gyro_x")
    ax.plot(t, df["imu_gyro_y"], linewidth=1.0, label="gyro_y")
    ax.plot(t, df["imu_gyro_z"], linewidth=1.0, label="gyro_z")
    ax.axhline( 0.02, color="orange", linewidth=0.8, linestyle="--", label="±0.02 noise threshold")
    ax.axhline(-0.02, color="orange", linewidth=0.8, linestyle="--")
    ax.set_title("IMU gyroscope (rad/s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("rad/s")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    # 3. IMU yaw + EKF yaw overlaid
    ax = axes[1, 0]
    ax.plot(t, df["imu_yaw_deg"], linewidth=1.2, label="imu_yaw_deg", color="steelblue")
    ax.plot(t, df["ekf_yaw_deg"], linewidth=1.2, label="ekf_yaw_deg", color="darkorange",
            linestyle="--")
    ax.set_title("Yaw — IMU vs EKF (deg)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("deg")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    # 4. EKF x and y positions
    ax = axes[1, 1]
    ax.plot(t, df["ekf_x"], linewidth=1.2, label="ekf_x (m)", color="steelblue")
    ax.plot(t, df["ekf_y"], linewidth=1.2, label="ekf_y (m)", color="darkorange")
    ax.set_title("EKF position (should be constant)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    # 5. Scan RMSE
    ax = axes[2, 0]
    ax.plot(t, df["scan_rmse"], linewidth=1.2, color="crimson", label="scan_rmse")
    ax.axhline(0.20, color="orange", linewidth=0.8, linestyle="--", label="0.20 m (full-trust)")
    ax.axhline(0.40, color="red",    linewidth=0.8, linestyle="--", label="0.40 m (reject)")
    ax.set_title("Scan-match RMSE (m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    # 6. Inlier ratio
    ax = axes[2, 1]
    ax.plot(t, df["inlier_ratio"], linewidth=1.2, color="limegreen", label="inlier_ratio")
    ax.axhline(0.35, color="orange", linewidth=0.8, linestyle="--", label="0.35 track accept")
    ax.set_ylim(0, 1.05)
    ax.set_title("Scan-match inlier ratio")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("ratio")
    ax.legend(fontsize=8)
    ax.grid(True, linewidth=0.3)

    out_path = path.replace(".csv", "_loc_analysis.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\n  Saved plot: {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python analyze_loc.py loc_test_stationary_*.csv [...]")
        sys.exit(1)

    for path in sys.argv[1:]:
        try:
            analyze_loc(path)
        except Exception as exc:
            print(f"ERROR processing {path}: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
