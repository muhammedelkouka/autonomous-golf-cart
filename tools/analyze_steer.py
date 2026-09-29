#!/usr/bin/env python3
"""
Steering test analysis tool.

Usage:
    python analyze_steer.py steer_test_A_20240601_120000.csv [file2.csv ...]
    python analyze_steer.py steer_test_repeat_20240601_120000.csv

Single-run CSVs (no run_index column):
  - Prints steps, drop count, mean/max error, latency stats.
  - Flags steps where |error_deg| > 0.5 deg.
  - Saves a two-subplot PNG (commanded vs actual + latency).

Repeat CSVs (run_index column present):
  - Reports center_offset per run (zero-drift visibility).
  - Per-step consistency table: mean_actual, std, spread across runs.
  - Flags steps where spread > 1.0 deg as INCONSISTENT.
  - Saves a scatter PNG: actual_deg vs cmd_deg, one point per run.
  - Falls back to the single-run comparison table when multiple repeat
    CSVs are supplied.

Test B CSVs (lag_deg column present — steer_test_B_*.csv):
  - Plots commanded vs actual angle over time + lag_deg over time.
  - Prints max lag, mean lag, settled angle after 3 s.
  - Flags max lag > 3° (STEER_SPEED may be too slow for corners).
"""

import sys
import math
import pathlib
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _safe_float(v) -> float:
    """Convert value to float; return NaN on failure."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _isnan(v) -> bool:
    try:
        return math.isnan(float(v))
    except (TypeError, ValueError):
        return True


# ---------------------------------------------------------------------------
#  Single-run analysis  (no run_index column)
# ---------------------------------------------------------------------------

def analyze_single(df: pd.DataFrame, path: str) -> dict:
    """Print stats and save PNG for a single-run CSV. Returns summary dict."""
    df["actual_deg"] = pd.to_numeric(df["actual_deg"], errors="coerce")
    df["error_deg"]  = pd.to_numeric(df["error_deg"],  errors="coerce")

    n_steps    = len(df)
    drop_count = int(df["dropped"].sum())
    valid      = df[df["dropped"] == 0].copy()

    mean_abs_err = round(float(valid["error_deg"].abs().mean()), 3) if not valid.empty else float("nan")
    max_abs_err  = round(float(valid["error_deg"].abs().max()),  3) if not valid.empty else float("nan")
    mean_lat     = round(float(df["latency_ms"].mean()), 3)
    std_lat      = round(float(df["latency_ms"].std()),  3)

    print(f"\n=== {pathlib.Path(path).name} ===")
    print(f"  Steps:        {n_steps}")
    print(f"  Drops:        {drop_count}")
    print(f"  Mean |error|: {mean_abs_err} deg")
    print(f"  Max  |error|: {max_abs_err} deg")
    print(f"  Mean latency: {mean_lat} ms")
    print(f"  Std  latency: {std_lat} ms")

    inaccurate = df[df["error_deg"].abs() > 0.5]
    if not inaccurate.empty:
        print("  Inaccurate steps (|error_deg| > 0.5 deg):")
        for _, row in inaccurate.iterrows():
            act_str = f"{row['actual_deg']:.3f}" if not _isnan(row["actual_deg"]) else "NaN"
            print(f"    step {int(row['step_index'])}: "
                  f"cmd={row['cmd_deg']:.3f}  actual={act_str}  "
                  f"error={row['error_deg']:.3f}")

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), tight_layout=True)
    fig.suptitle(pathlib.Path(path).name, fontsize=11)

    steps   = df["step_index"].tolist()
    cmd_deg = df["cmd_deg"].tolist()
    xs      = list(range(len(steps)))
    act_deg = [
        float(v) if not _isnan(v) else 0.0
        for v in df["actual_deg"].tolist()
    ]

    ax1   = axes[0]
    width = 0.35
    ax1.bar([x - width / 2 for x in xs], cmd_deg, width, label="Commanded", color="steelblue")
    ax1.bar([x + width / 2 for x in xs], act_deg, width, label="Actual",    color="darkorange", alpha=0.85)
    for i, row in df.iterrows():
        if row["dropped"]:
            ax1.annotate("DROP", xy=(xs[i] + width / 2, 0), ha="center", va="bottom",
                         fontsize=7, color="red", rotation=90)
    ax1.set_xticks(xs)
    ax1.set_xticklabels([str(s) for s in steps])
    ax1.set_xlabel("Step index")
    ax1.set_ylabel("Angle (deg)")
    ax1.set_title("Commanded vs Actual Steering Angle")
    ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax1.legend()

    ax2      = axes[1]
    latency  = df["latency_ms"].tolist()
    bar_cols = ["red" if bool(d) else "teal" for d in df["dropped"].tolist()]
    ax2.bar(xs, latency, color=bar_cols)
    ax2.set_xticks(xs)
    ax2.set_xticklabels([str(s) for s in steps])
    ax2.set_xlabel("Step index")
    ax2.set_ylabel("Latency (ms)")
    ax2.set_title("Latency per Step  (red = dropped / timeout)")

    out_png = pathlib.Path(path).with_suffix(".png")
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  Saved plot: {out_png}")

    return {
        "filename":     pathlib.Path(path).name,
        "mean_abs_err": mean_abs_err,
        "max_abs_err":  max_abs_err,
        "drop_count":   drop_count,
        "mean_latency": mean_lat,
    }


# ---------------------------------------------------------------------------
#  Repeat-run analysis  (run_index column present)
# ---------------------------------------------------------------------------

def analyze_repeat(df: pd.DataFrame, path: str) -> dict:
    """Print consistency stats and save PNG for a repeat-run CSV."""
    df["actual_deg"] = pd.to_numeric(df["actual_deg"], errors="coerce")
    df["error_deg"]  = pd.to_numeric(df["error_deg"],  errors="coerce")
    if "center_offset_deg" in df.columns:
        df["center_offset_deg"] = pd.to_numeric(df["center_offset_deg"], errors="coerce")

    num_runs = int(df["run_index"].max())
    name     = pathlib.Path(path).name

    print(f"\n=== {name}  [REPEAT x{num_runs}] ===")

    # ── Center offset per run ────────────────────────────────────────────────
    center_rows = df[df["step_index"] == -1].copy()
    if not center_rows.empty:
        print("  Center offset per run (wheel return to 0°):")
        for _, row in center_rows.iterrows():
            offset_col = (
                row["center_offset_deg"]
                if "center_offset_deg" in row.index and not _isnan(row["center_offset_deg"])
                else row["actual_deg"]
            )
            dropped_str = " [TIMEOUT]" if bool(row["dropped"]) else ""
            offset_str  = f"{offset_col:.3f}" if not _isnan(offset_col) else "NaN"
            print(f"    run {int(row['run_index'])}: center_offset={offset_str} deg{dropped_str}")

    # ── Per-step consistency across runs ────────────────────────────────────
    step_rows = df[df["step_index"] >= 0].copy()

    # Unique (step_index, cmd_deg) pairs in original order
    step_order = (
        step_rows[["step_index", "cmd_deg"]]
        .drop_duplicates(subset="step_index")
        .sort_values("step_index")
    )

    print()
    print("  Per-step consistency table:")
    hdr = (f"  {'step':>4}  {'cmd_deg':>8}  {'mean_actual':>11}  "
           f"{'std':>7}  {'min':>7}  {'max':>7}  {'spread':>7}  "
           f"{'mean_lat':>9}  {'drops':>5}  flag")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    any_inconsistent = False
    for _, srow in step_order.iterrows():
        si   = int(srow["step_index"])
        cmd  = float(srow["cmd_deg"])
        mask = step_rows["step_index"] == si
        grp  = step_rows[mask]
        valid = grp[grp["dropped"] == 0]["actual_deg"].dropna()
        drops = int(grp["dropped"].sum())

        mean_a  = round(float(valid.mean()), 3) if not valid.empty else float("nan")
        std_a   = round(float(valid.std()),  3) if not valid.empty else float("nan")
        min_a   = round(float(valid.min()),  3) if not valid.empty else float("nan")
        max_a   = round(float(valid.max()),  3) if not valid.empty else float("nan")
        spread  = round(float(valid.max() - valid.min()), 3) if not valid.empty else float("nan")
        mean_l  = round(float(grp["latency_ms"].mean()), 3)

        flag = ""
        if not _isnan(spread) and spread > 1.0:
            flag = "INCONSISTENT"
            any_inconsistent = True

        def _fs(v): return f"{v:>7.3f}" if not _isnan(v) else f"{'NaN':>7}"

        print(f"  {si:>4}  {cmd:>8.2f}  {_fs(mean_a)}  "
              f"{_fs(std_a)}  {_fs(min_a)}  {_fs(max_a)}  {_fs(spread)}  "
              f"{mean_l:>9.3f}  {drops:>5}  {flag}")

    if not any_inconsistent:
        print("  → All steps within spread ≤ 1.0 deg")

    # Summary stats (treating all valid step rows)
    valid_all    = step_rows[step_rows["dropped"] == 0]
    mean_abs_err = round(float(valid_all["error_deg"].abs().mean()), 3) if not valid_all.empty else float("nan")
    max_abs_err  = round(float(valid_all["error_deg"].abs().max()),  3) if not valid_all.empty else float("nan")
    drop_total   = int(step_rows["dropped"].sum())
    mean_lat     = round(float(step_rows["latency_ms"].mean()), 3)

    print()
    print(f"  Overall  mean|err|={mean_abs_err} deg  max|err|={max_abs_err} deg  "
          f"drops={drop_total}  mean_latency={mean_lat} ms")

    # ── Scatter plot: actual_deg vs cmd_deg, one point per run ──────────────
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), tight_layout=True)
    fig.suptitle(f"{name}  [repeat x{num_runs}]", fontsize=11)

    ax1 = axes[0]
    # Reference line y = x
    cmd_range = sorted(step_rows["cmd_deg"].dropna().unique())
    ax1.plot(cmd_range, cmd_range, "k--", linewidth=0.8, label="ideal (y=x)")

    cmap   = plt.get_cmap("tab10")
    n_runs = int(step_rows["run_index"].max())
    for ri in sorted(step_rows["run_index"].unique()):
        run_data = step_rows[
            (step_rows["run_index"] == ri) & (step_rows["dropped"] == 0)
        ]
        ax1.scatter(
            run_data["cmd_deg"],
            run_data["actual_deg"],
            s=40,
            color=cmap(int(ri - 1) % 10),
            label=f"run {int(ri)}",
            zorder=3,
            alpha=0.85,
        )

    ax1.set_xlabel("Commanded angle (deg)")
    ax1.set_ylabel("Actual angle (deg)")
    ax1.set_title("Scatter: actual vs commanded  (one point per run per step)")
    ax1.legend(fontsize=8, ncol=min(n_runs, 5))
    ax1.grid(True, linewidth=0.4)

    # Subplot 2: spread (max−min) per step_index as bar chart
    ax2 = axes[1]
    si_list     = []
    spread_list = []
    cmd_labels  = []
    for _, srow in step_order.iterrows():
        si  = int(srow["step_index"])
        cmd = float(srow["cmd_deg"])
        grp = step_rows[(step_rows["step_index"] == si) & (step_rows["dropped"] == 0)]["actual_deg"].dropna()
        sp  = float(grp.max() - grp.min()) if len(grp) > 1 else float("nan")
        si_list.append(si)
        spread_list.append(sp if not _isnan(sp) else 0)
        cmd_labels.append(f"{si}\n({cmd:.0f}°)")

    bar_colors = ["red" if s > 1.0 else "steelblue" for s in spread_list]
    ax2.bar(range(len(si_list)), spread_list, color=bar_colors)
    ax2.axhline(1.0, color="red", linestyle="--", linewidth=0.8, label="1.0° threshold")
    ax2.set_xticks(range(len(si_list)))
    ax2.set_xticklabels(cmd_labels, fontsize=8)
    ax2.set_xlabel("Step (cmd angle)")
    ax2.set_ylabel("Spread (max − min) deg")
    ax2.set_title("Run-to-run spread per step  (red = INCONSISTENT > 1.0°)")
    ax2.legend(fontsize=8)

    out_png = pathlib.Path(path).with_suffix(".png")
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  Saved plot: {out_png}")

    return {
        "filename":     name,
        "mean_abs_err": mean_abs_err,
        "max_abs_err":  max_abs_err,
        "drop_count":   drop_total,
        "mean_latency": mean_lat,
    }


# ---------------------------------------------------------------------------
#  Test B analysis  (lag_deg column present — continuous tracking)
# ---------------------------------------------------------------------------

def analyze_test_b(df: pd.DataFrame, path: str) -> dict:
    """Analyze a Test B (continuous tracking / corner simulation) CSV."""
    df["actual_deg"] = pd.to_numeric(df["actual_deg"], errors="coerce")
    df["lag_deg"]    = pd.to_numeric(df["lag_deg"],    errors="coerce")

    name = pathlib.Path(path).name

    # Split: last row (large timestamp gap) is the post-stream settle measurement
    df = df.sort_values("timestamp_ms").reset_index(drop=True)
    stream_df = df.iloc[:-1].copy() if len(df) > 1 else df.copy()
    settle_row = df.iloc[-1]  if len(df) > 1 else None

    # Stats (stream rows only)
    valid_lag = stream_df["lag_deg"].dropna()
    max_lag   = round(float(valid_lag.abs().max()),  3) if not valid_lag.empty else float("nan")
    mean_lag  = round(float(valid_lag.abs().mean()), 3) if not valid_lag.empty else float("nan")
    settled   = (round(float(settle_row["actual_deg"]), 3)
                 if settle_row is not None and not _isnan(settle_row["actual_deg"])
                 else float("nan"))
    settle_lag = (round(float(settle_row["lag_deg"]), 3)
                  if settle_row is not None and not _isnan(settle_row["lag_deg"])
                  else float("nan"))

    print(f"\n=== {name}  [TEST B — Continuous Tracking] ===")
    print(f"  Stream commands:  {len(stream_df)}")
    print(f"  Max |lag|:        {max_lag} deg")
    print(f"  Mean |lag|:       {mean_lag} deg")
    print(f"  Settled actual:   {settled} deg  (lag = {settle_lag} deg after 3 s)")

    if not _isnan(max_lag) and max_lag > 3.0:
        print(f"  FLAG: max lag {max_lag:.3f}° > 3° — STEER_SPEED may be too slow for corners")

    # ── Figure ─────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), tight_layout=True)
    fig.suptitle(f"{name}  [tracking test]", fontsize=11)

    ts  = stream_df["timestamp_ms"].values

    # Subplot 1: commanded vs actual over time
    ax1 = axes[0]
    ax1.plot(ts, stream_df["cmd_deg"],    color="steelblue",  linewidth=1.5, label="Commanded")
    ax1.plot(ts, stream_df["actual_deg"], color="darkorange", linewidth=1.5, label="Actual", alpha=0.85)
    if settle_row is not None and not _isnan(settle_row["actual_deg"]):
        ax1.scatter([settle_row["timestamp_ms"]], [settle_row["actual_deg"]],
                    color="limegreen", zorder=5, s=60, label="Settled")
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Angle (deg)")
    ax1.set_title("Commanded vs Actual Steering Angle")
    ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax1.legend()
    ax1.grid(True, linewidth=0.4)

    # Subplot 2: lag_deg over time (+ ±3° threshold lines)
    ax2 = axes[1]
    ax2.plot(ts, stream_df["lag_deg"], color="crimson", linewidth=1.5, label="lag (actual − cmd)")
    ax2.axhline( 0.0, color="gray",   linewidth=0.5, linestyle="--")
    ax2.axhline( 3.0, color="orange", linewidth=0.8, linestyle="--", label="±3° threshold")
    ax2.axhline(-3.0, color="orange", linewidth=0.8, linestyle="--")
    ax2.fill_between(ts, stream_df["lag_deg"], 0,
                     where=stream_df["lag_deg"].abs() > 3.0,
                     color="orange", alpha=0.25, label=">3° zone")
    ax2.set_xlabel("Time (ms)")
    ax2.set_ylabel("Lag (deg)")
    ax2.set_title("Tracking Lag  (red = actual − commanded)")
    ax2.legend()
    ax2.grid(True, linewidth=0.4)

    out_png = pathlib.Path(path).with_suffix(".png")
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  Saved plot: {out_png}")

    return {
        "filename":     name,
        "mean_abs_err": mean_lag,
        "max_abs_err":  max_lag,
        "drop_count":   0,
        "mean_latency": float("nan"),
    }


# ---------------------------------------------------------------------------
#  Sweep analysis  (interval_ms column present — steer_sweep_*.csv)
#
#  actual_deg in the CSV is the raw ESP32 value (negated from PC convention
#  because the firmware negates the angle on input).  Sign-correcting it
#  (negating again) recovers the physical angle in PC convention, which lets
#  us compute a meaningful real_lag = corrected_actual − cmd_deg.
#
#  Table columns: interval_ms | max_lag | mean_lag | settled | verdict
#    GOOD     — max |real_lag| < 2.0°
#    MARGINAL — 2.0° ≤ max |real_lag| ≤ 3.5°
#    POOR     — max |real_lag| > 3.5°
#  settled: final command (return to 0°) lands within 1° of target.
# ---------------------------------------------------------------------------

def analyze_sweep(df: pd.DataFrame, path: str) -> dict:
    """Analyze a sweep CSV and return a summary dict."""
    for col in ("actual_deg", "lag_deg", "cmd_deg", "cmd_index", "interval_ms"):
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Sign-correct actual_deg → PC convention; compute real_lag
    df["actual_corr"] = -df["actual_deg"]
    df["real_lag"]    = df["actual_corr"] - df["cmd_deg"]

    name = pathlib.Path(path).name
    stem = pathlib.Path(path).stem  # steer_sweep_YYYYMMDD_HHMMSS

    intervals = sorted(df["interval_ms"].dropna().unique().astype(int).tolist())

    print(f"\n=== {name}  [SWEEP] ===")
    print(f"  Intervals tested: {intervals}")
    print()
    hdr = f"  {'interval_ms':>12}  {'max_lag':>9}  {'mean_lag':>9}  {'settled':>8}  verdict"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    interval_stats = []
    for ivl in intervals:
        sub      = df[df["interval_ms"] == ivl].sort_values("cmd_index")
        real_lags = sub["real_lag"].dropna()

        max_lag  = round(float(real_lags.abs().max()),  3) if not real_lags.empty else float("nan")
        mean_lag = round(float(real_lags.abs().mean()), 3) if not real_lags.empty else float("nan")

        # settled: last staircase command (return to 0°) within 1° of target
        last_row = sub[sub["cmd_index"] == sub["cmd_index"].max()]
        last_rl  = last_row["real_lag"].dropna()
        final_lag = float(last_rl.abs().mean()) if not last_rl.empty else float("nan")
        settled  = not _isnan(final_lag) and final_lag < 1.0

        if _isnan(max_lag):
            verdict = "NO DATA"
        elif max_lag < 2.0:
            verdict = "GOOD"
        elif max_lag <= 3.5:
            verdict = "MARGINAL"
        else:
            verdict = "POOR"

        max_s    = f"{max_lag:.3f}" if not _isnan(max_lag) else " NaN"
        mean_s   = f"{mean_lag:.3f}" if not _isnan(mean_lag) else " NaN"
        settled_s = "YES" if settled else "NO"
        print(f"  {ivl:>12}  {max_s:>9}  {mean_s:>9}  {settled_s:>8}  {verdict}")

        interval_stats.append({
            "interval_ms": ivl,
            "max_lag":     max_lag,
            "mean_lag":    mean_lag,
            "settled":     settled,
            "verdict":     verdict,
        })

    # Recommended interval
    good_ivls     = [s for s in interval_stats if s["verdict"] == "GOOD"]
    marginal_ivls = [s for s in interval_stats if s["verdict"] == "MARGINAL"]
    if good_ivls:
        rec = min(good_ivls, key=lambda s: s["interval_ms"])
        print(f"\n  Recommended interval: {rec['interval_ms']} ms  "
              f"(max_lag={rec['max_lag']:.3f}°  GOOD)")
    elif marginal_ivls:
        rec = min(marginal_ivls, key=lambda s: s["interval_ms"])
        print(f"\n  Recommended interval: {rec['interval_ms']} ms  "
              f"(max_lag={rec['max_lag']:.3f}°  MARGINAL — best available)")
    else:
        print("\n  No GOOD or MARGINAL interval found — "
              "reduce STEER_SPEED or increase command rate")

    # ── Plot: real_lag per cmd_index, one colored line per interval ──────────
    fig, ax = plt.subplots(figsize=(12, 6), tight_layout=True)
    fig.suptitle(f"{name}  [sweep — real lag per interval]", fontsize=11)

    cmap = plt.get_cmap("tab10")
    for ki, ivl in enumerate(intervals):
        sub = df[df["interval_ms"] == ivl].sort_values("cmd_index")
        ax.plot(
            sub["cmd_index"].values,
            sub["real_lag"].values,
            marker="o", markersize=4,
            color=cmap(ki % 10),
            linewidth=1.5,
            label=f"{ivl} ms",
        )

    ax.axhline( 2.0, color="red",  linewidth=1.2, linestyle="--", label="±2° threshold")
    ax.axhline(-2.0, color="red",  linewidth=1.2, linestyle="--")
    ax.axhline( 0.0, color="gray", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Command index")
    ax.set_ylabel("Real lag (deg)  =  corrected_actual − cmd")
    ax.set_title("Steering lag per interval  (GOOD threshold ±2°, red dashed)")
    ax.legend(fontsize=9, title="Interval")
    ax.grid(True, linewidth=0.4)

    out_png = pathlib.Path(path).parent / f"{stem}.png"
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  Saved plot: {out_png}")

    best_max = min((s["max_lag"] for s in interval_stats if not _isnan(s["max_lag"])),
                   default=float("nan"))
    return {
        "filename":     name,
        "mean_abs_err": float("nan"),
        "max_abs_err":  best_max,
        "drop_count":   0,
        "mean_latency": float("nan"),
    }


# ---------------------------------------------------------------------------
#  Dispatcher
# ---------------------------------------------------------------------------

def analyze_file(path: str) -> dict:
    """Auto-detect single / repeat / sweep / Test-B CSV and dispatch accordingly."""
    df = pd.read_csv(path)
    if "interval_ms" in df.columns:
        return analyze_sweep(df, path)
    if "run_index" in df.columns:
        return analyze_repeat(df, path)
    if "lag_deg" in df.columns:
        return analyze_test_b(df, path)
    return analyze_single(df, path)


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python analyze_steer.py <steer_test_*.csv> [...]")
        sys.exit(1)

    summaries = []
    for path in sys.argv[1:]:
        try:
            s = analyze_file(path)
            summaries.append(s)
        except Exception as exc:
            print(f"ERROR processing {path}: {exc}", file=sys.stderr)

    if len(summaries) > 1:
        col_w  = max(len(s["filename"]) for s in summaries) + 2
        header = (f"{'Filename':<{col_w}} {'MeanAbsErr(deg)':>16} "
                  f"{'MaxAbsErr(deg)':>15} {'Drops':>6} {'MeanLat(ms)':>12}")
        print(f"\n{'=' * len(header)}")
        print("Comparison Table")
        print("=" * len(header))
        print(header)
        print("-" * len(header))
        for s in summaries:
            print(f"{s['filename']:<{col_w}} "
                  f"{s['mean_abs_err']:>16.3f} "
                  f"{s['max_abs_err']:>15.3f} "
                  f"{s['drop_count']:>6} "
                  f"{s['mean_latency']:>12.3f}")


if __name__ == "__main__":
    main()
