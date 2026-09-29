#!/usr/bin/env python3
"""
Thesis-figure generator for ctrl_log_*.csv drive runs.

Produces two figures per file, separate from analyze_drive.py's 4-panel
diagnostic plot:
  1. <stem>_trajectory.png     — top-down driven path vs. nearest planned
                                  path point, start/end markers, goal distance
  2. <stem>_localization.png   — scan_rmse + scan_inlier_ratio vs time,
                                  with the project's documented quality
                                  thresholds (0.20 m nominal, 0.40 m "bad")

Usage:
    python plot_trajectory.py ctrl_log_20260728_131526.csv [file2.csv ...]
"""

import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt


def _numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


def plot_trajectory(path: str) -> None:
    df = pd.read_csv(path, comment="#")
    name = pathlib.Path(path).name
    _numeric(df, ["robot_x", "robot_y", "path_x", "path_y", "time_s",
                   "cte_m", "dist_to_goal_m", "scan_rmse",
                   "scan_inlier_ratio"])

    # ── Figure 1: top-down trajectory ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 8), tight_layout=True)
    ax.plot(df["path_x"], df["path_y"], color="seagreen", linewidth=1.6,
             linestyle="--", alpha=0.85, label="planned path (nearest point)")
    ax.plot(df["robot_x"], df["robot_y"], color="magenta", linewidth=1.6,
             label="driven trajectory (EKF pose)")
    ax.scatter(df["robot_x"].iloc[0], df["robot_y"].iloc[0], color="blue",
               s=70, zorder=5, marker="o", label="start")
    ax.scatter(df["robot_x"].iloc[-1], df["robot_y"].iloc[-1], color="black",
               s=90, zorder=5, marker="X",
               label=f"end (dist_to_goal={df['dist_to_goal_m'].iloc[-1]:.2f} m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{name}\nDriven trajectory vs. planned path "
                 f"(max |CTE|={df['cte_m'].abs().max():.3f} m, "
                 f"mean |CTE|={df['cte_m'].abs().mean():.3f} m)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3)
    ax.legend(fontsize=9)

    out1 = path.replace(".csv", "_trajectory.png")
    fig.savefig(out1, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out1}")

    # ── Figure 2: localization quality over time ───────────────────────
    fig2, ax1 = plt.subplots(figsize=(11, 5), tight_layout=True)
    ax1.plot(df["time_s"], df["scan_rmse"], color="steelblue", linewidth=1.3,
              label="scan RMSE (m)")
    ax1.axhline(0.20, color="orange", linewidth=0.8, linestyle="--",
                 alpha=0.7, label="0.20 m nominal quality target")
    ax1.axhline(0.40, color="crimson", linewidth=0.8, linestyle="--",
                 alpha=0.7, label="0.40 m 'bad' gate")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("scan RMSE (m)", color="steelblue")
    ax1.tick_params(axis="y", labelcolor="steelblue")

    ax2 = ax1.twinx()
    ax2.plot(df["time_s"], df["scan_inlier_ratio"], color="darkorange",
              linewidth=1.1, alpha=0.85, label="inlier ratio")
    ax2.set_ylabel("scan inlier ratio", color="darkorange")
    ax2.tick_params(axis="y", labelcolor="darkorange")
    ax2.set_ylim(0, 1.05)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="lower right")
    ax1.set_title(f"{name}\nLocalization quality over time "
                   f"(mean RMSE={df['scan_rmse'].mean():.3f} m, "
                   f"mean inlier={df['scan_inlier_ratio'].mean():.3f})")
    ax1.grid(True, linewidth=0.3)

    out2 = path.replace(".csv", "_localization.png")
    fig2.savefig(out2, dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print(f"Saved: {out2}")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python plot_trajectory.py ctrl_log_*.csv [...]")
        sys.exit(1)
    for path in sys.argv[1:]:
        try:
            plot_trajectory(path)
        except Exception as exc:
            print(f"ERROR processing {path}: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
