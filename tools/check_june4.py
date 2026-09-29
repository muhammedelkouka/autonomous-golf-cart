import csv, glob, os

data_dir = r"c:\Users\bios\Desktop\isolated_version\runtime_data"
files = sorted(glob.glob(os.path.join(data_dir, "ctrl_log_20260604_*.csv")))

for path in files:
    run = os.path.basename(path).replace("ctrl_log_","").replace(".csv","")
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r.get("steer_raw_deg") not in (None,"")]
    if not rows:
        print(f"{run}: no data rows")
        continue

    cols = list(rows[0].keys())
    steer = [float(r["steer_raw_deg"]) for r in rows]
    peak  = max(abs(min(steer)), abs(max(steer)))

    yaw_drift = "N/A"
    if "robot_yaw_deg" in cols:
        yaw = [float(r["robot_yaw_deg"]) for r in rows]
        yaw_drift = f"{yaw[-1]-yaw[0]:+.2f} deg ({yaw[0]:.2f}->{yaw[-1]:.2f})"

    la_info = "N/A"
    if "lookahead_m" in cols:
        la = [float(r["lookahead_m"]) for r in rows]
        pct_min = 100*sum(1 for v in la if v<=1.51)/len(la)
        la_info = f"avg={sum(la)/len(la):.2f}m  min={min(la):.2f}m  @min={pct_min:.0f}%"

    cte_info = "N/A"
    if "cte_m" in cols:
        cte = [float(r["cte_m"]) for r in rows]
        cte_info = f"max={max(abs(v) for v in cte):.3f}m"

    print(f"{run}  ticks={len(rows):3d}  peak_steer={peak:5.1f}deg  yaw_drift={yaw_drift}  lookahead={la_info}  CTE={cte_info}")
