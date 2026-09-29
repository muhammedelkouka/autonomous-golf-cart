#!/usr/bin/env python3
"""
Smartphone IMU vs dedicated IMU (Hiwonder IM10A) comparison figures.

Inputs (this folder):
  imu_all_data_2025-12-12_21-19-57.csv  - smartphone IMU log, 50 Hz nominal
  ctrl_log_20260728_162951.csv          - golf cart drive log (dedicated IMU
                                           heading logged once per 50 Hz
                                           control tick in imu_raw_deg)

Outputs (this folder):
  imu_gyro_performance.png     - raw gyro rate (GX/GY/GZ) time series + noise
  imu_heading_angles.png       - raw yaw vs fused yaw/pitch/roll (own scales)
  imu_latency_update_rate.png  - sample-interval / update-rate / latency
"""
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = pathlib.Path(__file__).parent
OUT = HERE

SURFACE, INK_PRIMARY, INK_SECOND, INK_MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE = "#e1e0d9", "#c3c2b7"
BLUE, ORANGE, RED, AQUA, VIOLET = "#2a78d6", "#eb6834", "#e34948", "#1baf7a", "#4a3aa7"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "axes.edgecolor": BASELINE, "axes.labelcolor": INK_SECOND,
    "text.color": INK_PRIMARY, "xtick.color": INK_MUTED, "ytick.color": INK_MUTED,
    "axes.titlecolor": INK_PRIMARY, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.grid": True, "axes.axisbelow": True, "legend.frameon": False,
    "font.size": 10.5,
})

def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(BASELINE)
    ax.tick_params(length=0)

# ── Load ─────────────────────────────────────────────────────────────────
phone = pd.read_csv(HERE / "imu_all_data_2025-12-12_21-19-57.csv")
cart = pd.read_csv(HERE / "ctrl_log_20260728_162951.csv", comment="#")

t_ph = phone["Timestamp"].to_numpy()
dt_ph = np.diff(t_ph)
hz_ph = 1.0 / np.median(dt_ph)

t_cart = cart["time_s"].to_numpy()
dt_cart = np.diff(t_cart)
hz_cart = 1.0 / np.median(dt_cart)

# ── Figure 1: gyro performance (phone GX/GY/GZ) ─────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(8.6, 8.0), sharex=True)
labels = [("GX", "Roll rate GX (deg/s)", BLUE),
          ("GY", "Pitch rate GY (deg/s)", ORANGE),
          ("GZ", "Yaw rate GZ (deg/s)", AQUA)]
for ax, (col, ylabel, color) in zip(axes, labels):
    sig = phone[col].to_numpy()
    ax.plot(t_ph, sig, color=color, lw=0.9)
    mu, sd = np.mean(sig), np.std(sig)
    ax.axhline(mu, color=BASELINE, lw=1.0, ls="--")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{col}: mean={mu:+.3f} deg/s, std={sd:.3f} deg/s, "
                 f"peak={np.max(np.abs(sig)):.2f} deg/s", fontsize=9.5, loc="left")
    style(ax)
axes[-1].set_xlabel("Time (s)")
fig.suptitle("Smartphone gyroscope raw output (3-axis)", y=0.995)
fig.tight_layout()
fig.savefig(OUT / "imu_gyro_performance.png", dpi=200)
plt.close(fig)

# ── Figure 2: heading / angles (raw vs fused, own scales) ──────────────
fig, axes = plt.subplots(4, 1, figsize=(8.6, 10.2), sharex=True)

ax = axes[0]
ax.plot(t_ph, phone["Yaw_Device"], color=INK_MUTED, lw=1.0)
ax.set_ylabel("Yaw raw (deg)")
ax.set_title("Raw device yaw (0-360, magnetometer/compass heading)", fontsize=9.5, loc="left")
style(ax)

ax = axes[1]
yf = phone["Yaw_Fused"].to_numpy()
ax.plot(t_ph, yf, color=BLUE, lw=1.1)
ax.set_ylabel("Yaw fused (deg)")
ax.set_title(f"Fused yaw (gyro-integrated): range [{yf.min():.2f}, {yf.max():.2f}] deg, "
             f"total drift={yf[-1]-yf[0]:+.3f} deg over {t_ph[-1]:.0f}s", fontsize=9.5, loc="left")
style(ax)

ax = axes[2]
pf = phone["Pitch_Fused"].to_numpy()
ax.plot(t_ph, pf, color=ORANGE, lw=1.1)
ax.set_ylabel("Pitch fused (deg)")
ax.set_title(f"Fused pitch: range [{pf.min():.2f}, {pf.max():.2f}] deg", fontsize=9.5, loc="left")
style(ax)

ax = axes[3]
rf = phone["Roll_Fused"].to_numpy()
ax.plot(t_ph, rf, color=AQUA, lw=1.1)
ax.set_ylabel("Roll fused (deg)")
ax.set_title(f"Fused roll: range [{rf.min():.2f}, {rf.max():.2f}] deg", fontsize=9.5, loc="left")
ax.set_xlabel("Time (s)")
style(ax)

fig.suptitle("Smartphone heading/attitude: raw compass vs. gyro-fused Euler angles", y=0.995)
fig.tight_layout()
fig.savefig(OUT / "imu_heading_angles.png", dpi=200)
plt.close(fig)

# ── Figure 3: latency / update-rate characterization ────────────────────
fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.2))

ax = axes[0, 0]
ax.plot(t_ph[1:], dt_ph * 1000.0, color=BLUE, lw=0.8)
ax.axhline(1000.0 / hz_ph, color=BASELINE, lw=1.0, ls="--")
ax.set_ylim(15, 25)
ax.set_ylabel("Sample interval (ms)")
ax.set_xlabel("Time (s)")
ax.set_title(f"Smartphone: dt over time (jitter std={np.std(dt_ph)*1000:.3f}ms\n"
             f"-> looks like a synthetic/rounded timestamp, not measured arrival time)",
             fontsize=9.5, loc="left")
style(ax)

ax = axes[0, 1]
bins = np.linspace(0, max(dt_ph.max(), dt_cart.max()) * 1000.0 * 1.05, 60)
ax.hist(dt_ph * 1000.0, bins=bins, color=BLUE, alpha=0.65, density=True,
        label=f"Smartphone (med={np.median(dt_ph)*1000:.1f}ms, jitter std={np.std(dt_ph)*1000:.2f}ms)")
ax.hist(dt_cart * 1000.0, bins=bins, color=AQUA, alpha=0.55, density=True,
        label=f"Cart control tick (med={np.median(dt_cart)*1000:.1f}ms, jitter std={np.std(dt_cart)*1000:.2f}ms)")
ax.set_xlabel("Sample interval (ms)")
ax.set_ylabel("Density")
ax.set_title("Sample-interval distribution", fontsize=9.5, loc="left")
ax.legend(loc="upper right", fontsize=8.5)
style(ax)

ax = axes[1, 0]
ax.plot(t_cart[1:], dt_cart * 1000.0, color=AQUA, lw=0.8)
ax.axhline(1000.0 / hz_cart, color=BASELINE, lw=1.0, ls="--")
ax.set_ylabel("Tick interval (ms)")
ax.set_xlabel("Time (s)")
ax.set_title(f"Golf cart control loop: dt over time (eff. {hz_cart:.1f} Hz)", fontsize=9.5, loc="left")
style(ax)

ax = axes[1, 1]
win = 2.0  # seconds
n_ph = max(1, int(win / np.median(dt_ph)))
n_cart = max(1, int(win / np.median(dt_cart)))
roll_sum_ph = pd.Series(dt_ph).rolling(n_ph).sum().to_numpy()
roll_sum_cart = pd.Series(dt_cart).rolling(n_cart).sum().to_numpy()
roll_hz_ph = n_ph / roll_sum_ph
roll_hz_cart = n_cart / roll_sum_cart
ax.plot(t_ph[1:], roll_hz_ph, color=BLUE, lw=1.3, label="Smartphone", alpha=0.9)
ax.plot(t_cart[1:], roll_hz_cart, color=AQUA, lw=1.3, label="Cart control loop", alpha=0.9)
ax.axhline(50, color=BASELINE, lw=1.0, ls="--")
ax.set_ylim(0, 60)
ax.set_ylabel("Rolling effective rate (Hz)")
ax.set_xlabel("Time (s)")
ax.set_title(f"{win:.0f}s rolling update rate", fontsize=9.5, loc="left")
ax.legend(loc="lower right", fontsize=8.5)
style(ax)

fig.suptitle("Update-rate stability / latency jitter: smartphone UDP stream vs. wired dedicated IMU", y=0.995)
fig.tight_layout()
fig.savefig(OUT / "imu_latency_update_rate.png", dpi=200)
plt.close(fig)

# ── Console summary ──────────────────────────────────────────────────────
print(f"[phone gyro]  GX std={phone['GX'].std():.4f}  GY std={phone['GY'].std():.4f}  GZ std={phone['GZ'].std():.4f} deg/s")
print(f"[phone yaw]   raw range=[{phone['Yaw_Device'].min():.1f},{phone['Yaw_Device'].max():.1f}]  "
      f"fused range=[{yf.min():.2f},{yf.max():.2f}]  fused drift={yf[-1]-yf[0]:+.3f} deg / {t_ph[-1]:.0f}s")
print(f"[phone dt]    median={np.median(dt_ph)*1000:.2f}ms  std={np.std(dt_ph)*1000:.2f}ms  "
      f"max_gap={dt_ph.max()*1000:.1f}ms  eff_hz={hz_ph:.2f}")
print(f"[cart dt]     median={np.median(dt_cart)*1000:.2f}ms  std={np.std(dt_cart)*1000:.2f}ms  "
      f"max_gap={dt_cart.max()*1000:.1f}ms  eff_hz={hz_cart:.2f}")
print("Saved:")
print(" ", OUT / "imu_gyro_performance.png")
print(" ", OUT / "imu_heading_angles.png")
print(" ", OUT / "imu_latency_update_rate.png")
