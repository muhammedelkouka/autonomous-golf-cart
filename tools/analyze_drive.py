#!/usr/bin/env python3
"""
Drive-run analysis tool for ctrl_log_*.csv files.

Detects new-format logs by the presence of the 'steer_raw_deg' column
(added June 2026 with dual-speed steering pipeline logging).

Usage:
    python analyze_drive.py ctrl_log_20260601_120000.csv [file2.csv ...]

Produces a 4-subplot figure per file:
  1. Steer pipeline  — steer_raw vs steer_filtered vs steer_cmd over time
  2. Steer tracking  — steer_cmd vs steer_actual over time
  3. CTE             — cross-track error over time with path_index transitions
  4. Send gate       — tick_index vs steer_delta_deg, green=sent, red=filtered

Saves figure as <csvname>_drive_analysis.png and prints a summary.
"""

import sys
import math
import pathlib
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _isnan(v) -> bool:
    try:
        return math.isnan(float(v))
    except (TypeError, ValueError):
        return True


def _safe(v, default=float("nan")) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
#  Core analysis
# ---------------------------------------------------------------------------

def analyze_drive(path: str) -> None:
    df = pd.read_csv(path, comment="#")

    if "steer_raw_deg" not in df.columns:
        print(f"ERROR: {path} does not contain 'steer_raw_deg' — "
              "re-run with a log from the dual-speed build (June 2026+).")
        return

    name = pathlib.Path(path).name
    stem = pathlib.Path(path).stem

    # Numeric coercion for all columns we touch
    for col in ("time_s", "steer_cmd_deg", "steer_actual_deg",
                "steer_raw_deg", "steer_filtered_deg", "steer_delta_deg",
                "steer_sent", "tick_index", "cte_m", "path_index"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    t = df["time_s"].values

    # ── Summary stats ────────────────────────────────────────────────────────
    cte      = df["cte_m"].dropna()
    sent     = df[df["steer_sent"] == 1]
    n_ticks  = len(df)
    run_dur  = float(df["time_s"].max()) if not df["time_s"].isna().all() else float("nan")

    mean_cte = round(float(cte.abs().mean()), 4) if not cte.empty else float("nan")
    max_cte  = round(float(cte.abs().max()),  4) if not cte.empty else float("nan")

    cmds_per_sec = (len(sent) / run_dur) if (not _isnan(run_dur) and run_dur > 0) else float("nan")

    sent_deltas   = sent["steer_delta_deg"].dropna().abs()
    mean_delta_sent = round(float(sent_deltas.mean()), 4) if not sent_deltas.empty else float("nan")

    pct_sent = round(100.0 * len(sent) / n_ticks, 2) if n_ticks > 0 else float("nan")

    print(f"\n=== {name}  [DRIVE ANALYSIS] ===")
    print(f"  Run duration:           {run_dur:.1f} s")
    print(f"  Total ticks:            {n_ticks}")
    print(f"  Mean |CTE|:             {mean_cte:.4f} m")
    print(f"  Max  |CTE|:             {max_cte:.4f} m")
    print(f"  Commands sent/s:        {cmds_per_sec:.2f}")
    print(f"  Mean |delta| when sent: {mean_delta_sent:.4f} deg")
    print(f"  Ticks with cmd sent:    {pct_sent:.1f}%  ({len(sent)}/{n_ticks})")

    # ── Figure ───────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(13, 14), tight_layout=True)
    fig.suptitle(f"{name}  [drive analysis]", fontsize=11)

    # ── Subplot 1: Steer pipeline ─────────────────────────────────────────
    ax1 = axes[0]
    ax1.plot(t, df["steer_raw_deg"],      color="steelblue",   linewidth=1.2,
             label="raw (Pure Pursuit)", alpha=0.75)
    ax1.plot(t, df["steer_filtered_deg"], color="darkorange",  linewidth=1.5,
             label="filtered (post-EMA)")
    ax1.plot(t, df["steer_cmd_deg"],      color="limegreen",   linewidth=1.0,
             label="cmd (last sent)", linestyle="--", alpha=0.9)
    ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax1.set_ylabel("Angle (deg)")
    ax1.set_title("Steer pipeline — raw vs filtered vs last-sent command")
    ax1.legend(fontsize=8)
    ax1.grid(True, linewidth=0.3)

    # ── Subplot 2: Steer tracking ─────────────────────────────────────────
    ax2 = axes[1]
    ax2.plot(t, df["steer_cmd_deg"],    color="steelblue",  linewidth=1.5,
             label="cmd (PC)")
    ax2.plot(t, df["steer_actual_deg"], color="crimson",    linewidth=1.2,
             label="actual (ESP32 telem)", alpha=0.85)
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Angle (deg)")
    ax2.set_title("Steer tracking — commanded vs actual wheel angle")
    ax2.legend(fontsize=8)
    ax2.grid(True, linewidth=0.3)

    # ── Subplot 3: CTE + path_index transitions ──────────────────────────
    ax3 = axes[2]
    ax3.plot(t, df["cte_m"], color="darkorchid", linewidth=1.3, label="CTE")
    ax3.axhline( 0,    color="gray",   linewidth=0.5, linestyle="--")
    ax3.axhline( 0.5,  color="orange", linewidth=0.7, linestyle="--", alpha=0.6,
                 label="±0.5 m")
    ax3.axhline(-0.5,  color="orange", linewidth=0.7, linestyle="--", alpha=0.6)

    # Mark path_index transitions with thin vertical dashed lines
    if "path_index" in df.columns:
        pidx = df["path_index"].dropna().astype(int)
        transitions = df.index[pidx.diff().fillna(0) != 0].tolist()
        for ti in transitions:
            if ti < len(t):
                ax3.axvline(t[ti], color="teal", linewidth=0.6, linestyle=":",
                            alpha=0.6)
        # Label the first occurrence so the legend isn't cluttered
        if transitions:
            ax3.axvline(t[transitions[0]], color="teal", linewidth=0.6,
                        linestyle=":", alpha=0.6, label="path_index change")

    ax3.set_ylabel("CTE (m)")
    ax3.set_title("Cross-track error over time  (teal dashes = path segment change)")
    ax3.legend(fontsize=8)
    ax3.grid(True, linewidth=0.3)

    # ── Subplot 4: Send-gate scatter ─────────────────────────────────────
    ax4 = axes[3]

    not_sent = df[df["steer_sent"] == 0]
    ax4.scatter(
        not_sent["tick_index"].values,
        not_sent["steer_delta_deg"].values,
        s=8, color="crimson", alpha=0.45, label="filtered out (not sent)",
        zorder=2,
    )
    ax4.scatter(
        sent["tick_index"].values,
        sent["steer_delta_deg"].values,
        s=14, color="limegreen", alpha=0.80, label="sent",
        zorder=3,
    )
    ax4.axhline( 1.0, color="gray", linewidth=0.8, linestyle="--",
                 label="±1° send gate")
    ax4.axhline(-1.0, color="gray", linewidth=0.8, linestyle="--")
    ax4.axhline( 0,   color="gray", linewidth=0.4, linestyle="--")
    ax4.set_xlabel("Tick index")
    ax4.set_ylabel("steer_delta_deg (deg)")
    ax4.set_title(
        f"Send-gate scatter — green=sent ({pct_sent:.1f}% of ticks), "
        f"red=filtered  |  mean Δ when sent={mean_delta_sent:.3f}°"
    )
    ax4.legend(fontsize=8)
    ax4.grid(True, linewidth=0.3)

    # Share x-axis time label on lower subplots
    for ax in (ax1, ax2, ax3):
        ax.set_xlabel("")
        ax.tick_params(labelbottom=False)
    ax3.set_xlabel("")  # path_index subplot — x handled by CTE label
    axes[-1].set_xlabel("Tick index")

    out_path = path.replace('.csv', '_drive_analysis.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python analyze_drive.py ctrl_log_*.csv [...]")
        sys.exit(1)

    for path in sys.argv[1:]:
        try:
            analyze_drive(path)
        except Exception as exc:
            print(f"ERROR processing {path}: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
