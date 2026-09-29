#!/usr/bin/env python3
"""
Thesis-grade figure set for a single ctrl_log_*.csv drive run.

Produces six print-ready PNGs covering data acquisition/latency, localization
(scan-matching) quality, sensor fusion (IMU vs scan-match heading), control
execution performance, the driven trajectory, and a whole-system KPI summary.

Design rules (see dataviz skill): one axis per panel — never a dual-axis /
twin-y chart, two related measures become two stacked panels instead; fixed
categorical color order; status colors (good/warning/critical) reserved for
threshold reference lines, never reused as a series color; light print
surface, hairline recessive gridlines, legend whenever 2+ series share a panel.

Usage:
    python thesis_figures.py ctrl_log_20260728_162951.csv [out_dir]
"""

import sys
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# ── Palette (validated default, light/print surface) ───────────────────────
SURFACE      = "#fcfcfb"
INK_PRIMARY  = "#0b0b0b"
INK_SECOND   = "#52514e"
INK_MUTED    = "#898781"
GRID         = "#e1e0d9"
BASELINE     = "#c3c2b7"

BLUE    = "#2a78d6"   # categorical slot 1
ORANGE  = "#eb6834"   # slot 2
AQUA    = "#1baf7a"   # slot 3
YELLOW  = "#eda100"   # slot 4
MAGENTA = "#e87ba4"   # slot 5
GREEN   = "#008300"   # slot 6
VIOLET  = "#4a3aa7"   # slot 7
RED     = "#e34948"   # slot 8

GOOD     = "#0ca30c"
WARNING  = "#fab219"
SERIOUS  = "#ec835a"
CRITICAL = "#d03b3b"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": BASELINE,
    "axes.labelcolor": INK_SECOND,
    "text.color": INK_PRIMARY,
    "xtick.color": INK_MUTED,
    "ytick.color": INK_MUTED,
    "axes.titlecolor": INK_PRIMARY,
    "grid.color": GRID,
    "grid.linewidth": 0.7,
    "axes.grid": True,
    "axes.axisbelow": True,
    "legend.frameon": False,
    "font.size": 10.5,
})

LW_DATA = 1.6
LW_REF  = 1.1


def style_axis(ax):
    ax.grid(True, linewidth=0.7, color=GRID)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(BASELINE)
    ax.tick_params(length=0)


def load(path):
    df = pd.read_csv(path, comment="#")
    numeric_cols = [c for c in df.columns if c not in ("map_name",)]
    for c in numeric_cols:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


# ── Figure 1: data acquisition & latency ────────────────────────────────────
def fig_latency(df, stem, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), tight_layout=True)
    fig.suptitle("Data acquisition timing — pipeline delay & command RTT", fontsize=13, y=1.02)

    t = df["time_s"].values
    delay = df["pipeline_delay_ms"].values
    rtt_all = df["udp_rtt_ms"].values
    rtt = df.loc[df["udp_rtt_ms"] > 0, ["time_s", "udp_rtt_ms"]]

    ax = axes[0, 0]
    ax.plot(t, delay, color=BLUE, linewidth=LW_DATA)
    ax.axhline(delay.mean(), color=INK_MUTED, linewidth=LW_REF, linestyle="--",
               label=f"mean {delay.mean():.1f} ms")
    ax.set_title("LiDAR→control pipeline delay")
    ax.set_ylabel("delay (ms)")
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    ax = axes[0, 1]
    ax.plot(rtt["time_s"], rtt["udp_rtt_ms"], color=ORANGE, linewidth=1.1, alpha=0.85)
    ax.axhline(rtt["udp_rtt_ms"].mean(), color=INK_MUTED, linewidth=LW_REF, linestyle="--",
               label=f"mean {rtt['udp_rtt_ms'].mean():.0f} ms")
    ax.set_title("ESP32 command round-trip time")
    ax.set_ylabel("RTT (ms)")
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    ax = axes[1, 0]
    ax.hist(delay, bins=40, color=BLUE, alpha=0.85)
    med = np.median(delay)
    ax.axvline(med, color=INK_MUTED, linewidth=LW_REF, linestyle="--", label=f"median {med:.1f} ms")
    ax.set_title("Pipeline delay distribution")
    ax.set_xlabel("delay (ms)"); ax.set_ylabel("ticks")
    ax.legend(fontsize=8)
    style_axis(ax)

    ax = axes[1, 1]
    ax.hist(rtt["udp_rtt_ms"], bins=40, color=ORANGE, alpha=0.85)
    med = rtt["udp_rtt_ms"].median()
    ax.axvline(med, color=INK_MUTED, linewidth=LW_REF, linestyle="--", label=f"median {med:.0f} ms")
    ax.set_title("RTT distribution (commands actually sent)")
    ax.set_xlabel("RTT (ms)"); ax.set_ylabel("commands")
    ax.legend(fontsize=8)
    style_axis(ax)

    drop_rate = 100.0 * df["udp_dropped"].sum() / len(df)
    fig.text(0.01, -0.02,
              f"{stem}  |  {len(df)} ticks over {t.max():.1f}s  |  UDP drop rate {drop_rate:.2f}%",
              fontsize=8, color=INK_MUTED)

    out = out_dir / f"{stem}_fig1_latency.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# ── Figure 2: localization / scan-matching quality ──────────────────────────
def fig_localization(df, stem, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, tight_layout=True)
    fig.suptitle("Localization quality — scan-matcher RMSE & inlier ratio", fontsize=13, y=1.02)

    t = df["time_s"].values

    ax = axes[0]
    ax.plot(t, df["scan_rmse"], color=BLUE, linewidth=LW_DATA)
    ax.axhline(0.20, color=WARNING, linewidth=LW_REF, linestyle="--", label="0.20 m nominal target")
    ax.axhline(0.40, color=CRITICAL, linewidth=LW_REF, linestyle="--", label="0.40 m degraded gate")
    ax.set_ylabel("scan RMSE (m)")
    ax.legend(fontsize=8, loc="center right")
    style_axis(ax)

    ax = axes[1]
    ax.plot(t, df["scan_inlier_ratio"], color=AQUA, linewidth=LW_DATA)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("inlier ratio")
    ax.set_xlabel("time (s)")
    style_axis(ax)

    fig.text(0.01, -0.02,
              f"{stem}  |  mean RMSE {df['scan_rmse'].mean():.3f} m, "
              f"max {df['scan_rmse'].max():.3f} m  |  mean inlier {df['scan_inlier_ratio'].mean():.3f}, "
              f"min {df['scan_inlier_ratio'].min():.3f}",
              fontsize=8, color=INK_MUTED)

    out = out_dir / f"{stem}_fig2_localization.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# ── Figure 3: sensor fusion — IMU vs scan-match heading ─────────────────────
def fig_fusion(df, stem, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, tight_layout=True)
    fig.suptitle("Heading sensor fusion — IMU vs scan-matcher (Fix 2: IMU drives control heading)",
                 fontsize=12.5, y=1.02)

    t = df["time_s"].values
    imu = df["imu_raw_deg"].values
    ekf = df["ekf_yaw_deg"].values

    ax = axes[0]
    ax.plot(t, imu, color=BLUE, linewidth=LW_DATA, label="IMU raw heading (used for control)")
    ax.plot(t, ekf, color=ORANGE, linewidth=1.2, alpha=0.85, label="scan-matcher heading (not used)")
    ax.set_ylabel("heading (deg)")
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    ax = axes[1]
    diff = imu - ekf
    ax.plot(t, diff, color=VIOLET, linewidth=1.2)
    ax.axhline(0, color=BASELINE, linewidth=0.8)
    ax.set_ylabel("IMU - scan-match (deg)")
    ax.set_xlabel("time (s)")
    style_axis(ax)

    out = out_dir / f"{stem}_fig3_fusion.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out, float(np.mean(np.abs(diff))), float(np.max(np.abs(diff)))


# ── Figure 4: control execution performance ─────────────────────────────────
def fig_execution(df, stem, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, tight_layout=True)
    fig.suptitle("Control execution — steering pipeline & path-tracking error", fontsize=13, y=1.02)

    t = df["time_s"].values

    ax = axes[0]
    ax.plot(t, df["steer_raw_deg"], color=BLUE, linewidth=1.0, alpha=0.7, label="raw (Pure Pursuit)")
    ax.plot(t, df["steer_filtered_deg"], color=ORANGE, linewidth=LW_DATA, label="filtered (post-EMA)")
    ax.axhline(0, color=BASELINE, linewidth=0.8)
    ax.set_ylabel("steer (deg)")
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    ax = axes[1]
    ax.plot(t, df["cte_m"], color=VIOLET, linewidth=LW_DATA)
    ax.axhline(0, color=BASELINE, linewidth=0.8)
    ax.axhline(0.5, color=WARNING, linewidth=LW_REF, linestyle="--", label="±0.5 m")
    ax.axhline(-0.5, color=WARNING, linewidth=LW_REF, linestyle="--")
    ax.axhline(1.9, color=CRITICAL, linewidth=LW_REF, linestyle="--", label="corridor limit ±1.9 m")
    ax.axhline(-1.9, color=CRITICAL, linewidth=LW_REF, linestyle="--")
    ax.set_ylabel("CTE (m)")
    ax.set_ylim(-2.1, 2.1)
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    sent = df[df["steer_sent"] == 1]
    pct_sent = 100.0 * len(sent) / len(df)
    ax = axes[2]
    ax.plot(t, df["wheel_mps"], color=AQUA, linewidth=LW_DATA, label="wheel speed")
    ax.set_ylabel("speed (m/s)")
    ax.set_xlabel("time (s)")
    ax.legend(fontsize=8, loc="upper right")
    style_axis(ax)

    fig.text(0.01, -0.02,
              f"{stem}  |  mean|CTE| {df['cte_m'].abs().mean():.3f} m, max {df['cte_m'].abs().max():.3f} m"
              f"  |  steer commands sent on {pct_sent:.1f}% of ticks",
              fontsize=8, color=INK_MUTED)

    out = out_dir / f"{stem}_fig4_execution.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# ── Figure 5: trajectory overview ───────────────────────────────────────────
def fig_trajectory(df, stem, out_dir):
    fig, ax = plt.subplots(figsize=(8, 9), tight_layout=True)
    ax.plot(df["path_x"], df["path_y"], color=ORANGE, linewidth=1.3, linestyle="--",
            alpha=0.9, label="planned path")
    ax.plot(df["robot_x"], df["robot_y"], color=BLUE, linewidth=1.8, label="driven trajectory")
    ax.scatter(df["robot_x"].iloc[0], df["robot_y"].iloc[0], color=GOOD, s=70, zorder=5,
               label="start")
    ax.scatter(df["robot_x"].iloc[-1], df["robot_y"].iloc[-1], color=CRITICAL, s=70, zorder=5,
               marker="X", label=f"end (dist_to_goal={df['dist_to_goal_m'].iloc[-1]:.2f} m)")
    ax.axis("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"{stem} — driven trajectory\n"
                 f"max|CTE|={df['cte_m'].abs().max():.3f} m, mean|CTE|={df['cte_m'].abs().mean():.3f} m")
    ax.legend(fontsize=9)
    style_axis(ax)
    out = out_dir / f"{stem}_fig5_trajectory.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# ── Figure 6: whole-system KPI summary panel ────────────────────────────────
def fig_summary(df, stem, out_dir, fusion_mean, fusion_max):
    dist_m = float(np.sum(np.hypot(df["robot_x"].diff(), df["robot_y"].diff()).fillna(0)))
    dur = float(df["time_s"].max())
    sent = df[df["steer_sent"] == 1]
    rtt = df.loc[df["udp_rtt_ms"] > 0, "udp_rtt_ms"]

    rows = [
        ("Run duration",              f"{dur:.1f} s"),
        ("Distance driven",           f"{dist_m:.1f} m"),
        ("Average speed",             f"{dist_m/dur:.2f} m/s"),
        ("Mean / max |CTE|",          f"{df['cte_m'].abs().mean():.3f} m / {df['cte_m'].abs().max():.3f} m"),
        ("Final distance to goal",    f"{df['dist_to_goal_m'].iloc[-1]:.2f} m"),
        ("Mean scan RMSE",            f"{df['scan_rmse'].mean():.3f} m"),
        ("Min scan inlier ratio",     f"{df['scan_inlier_ratio'].min():.3f}"),
        ("IMU vs scan-match heading", f"mean {fusion_mean:.2f}°, max {fusion_max:.2f}°"),
        ("Mean pipeline delay",       f"{df['pipeline_delay_ms'].mean():.1f} ms"),
        ("Mean command RTT",          f"{rtt.mean():.0f} ms" if len(rtt) else "n/a"),
        ("UDP drop rate",             f"{100.0*df['udp_dropped'].sum()/len(df):.2f}%"),
        ("Steer commands sent",       f"{len(sent)} ({100.0*len(sent)/len(df):.1f}% of ticks)"),
        ("Wheel-speed data valid",    f"{100.0*(df['wheel_mps']>0).mean():.1f}% of ticks"),
        ("Max steer command",         f"{df['steer_raw_deg'].abs().max():.1f}°"),
    ]

    fig, ax = plt.subplots(figsize=(8, 6.2))
    ax.axis("off")
    fig.suptitle(f"{stem} — whole-system performance summary", fontsize=13, y=0.97)

    n = len(rows)
    for i, (label, value) in enumerate(rows):
        y = 1 - (i + 0.5) / n
        ax.text(0.02, y, label, fontsize=10.5, color=INK_SECOND, va="center", transform=ax.transAxes)
        ax.text(0.98, y, value, fontsize=10.5, color=INK_PRIMARY, va="center", ha="right",
                fontweight="bold", transform=ax.transAxes)
        if i < n - 1:
            ax.axhline(1 - (i + 1) / n, color=GRID, linewidth=0.7, xmin=0.02, xmax=0.98)

    out = out_dir / f"{stem}_fig6_summary.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def main():
    if len(sys.argv) < 2:
        print("Usage: python thesis_figures.py ctrl_log_*.csv [out_dir]")
        sys.exit(1)

    path = sys.argv[1]
    stem = pathlib.Path(path).stem
    out_dir = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else pathlib.Path(path).parent
    out_dir.mkdir(parents=True, exist_ok=True)

    df = load(path)

    outputs = []
    outputs.append(fig_latency(df, stem, out_dir))
    outputs.append(fig_localization(df, stem, out_dir))
    f3, fmean, fmax = fig_fusion(df, stem, out_dir)
    outputs.append(f3)
    outputs.append(fig_execution(df, stem, out_dir))
    outputs.append(fig_trajectory(df, stem, out_dir))
    outputs.append(fig_summary(df, stem, out_dir, fmean, fmax))

    print(f"\n{stem}: {len(df)} ticks, {df['time_s'].max():.1f}s")
    for o in outputs:
        print("  saved:", o)


if __name__ == "__main__":
    main()
