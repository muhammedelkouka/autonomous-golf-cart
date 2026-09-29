# -*- coding: utf-8 -*-
"""Conference-paper figure: GPS/IMU EKF fusion scenarios.
Each scenario turns exactly one knob relative to the baseline.
Data source: thesis_final_draft/ch3_problem_analysis_system_design.tex, Sec. 3.3.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

NL  = chr(10)
SIG = "$" + chr(92) + "sigma$"   # rendered as a proper Greek sigma

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8,
    "mathtext.fontset": "stix",
    "axes.linewidth": 0.7,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
})

# (name, distinguishing knob, Position RMSE [m], Max position error [m])
scen = [
    ("Best case",        SIG + " = 1 m, 5 fixes/s",   7.088,  9.282),
    ("Noisy start",      SIG + " = 10 m, first 30 m", 11.402, 26.552),
    ("Frequent outages", "25% of fixes, up to 5 s",   12.606, 18.008),
    ("Slow fix rate",    "1 fix/s instead of 5",       9.981, 15.614),
    ("No signal at start", "no fix for first 30 m",   12.688, 28.097),
]

labels = [s[0] + NL + "(" + s[1] + ")" for s in scen]
rmse   = np.array([s[2] for s in scen])
maxerr = np.array([s[3] for s in scen])
y = np.arange(len(scen))[::-1]          # first scenario at top

BAR  = "#2A6EA6"
WHIS = "#5A6570"

fig, ax = plt.subplots(figsize=(3.5, 2.8))

ax.hlines(y, rmse, maxerr, color=WHIS, lw=0.8, zorder=2)
ax.plot(maxerr, y, marker="|", ls="none", ms=4.5, mew=0.9, color=WHIS, zorder=2)
ax.barh(y, rmse, height=0.62, color=BAR, edgecolor="white", lw=0.5, zorder=1)

for yi, r in zip(y, rmse):
    ax.text(r - 0.45, yi, "%.1f" % r, va="center", ha="right",
            fontsize=7, color="white", zorder=4)

ax.set_yticks(y)
ax.set_yticklabels(labels, fontsize=6.9)
ax.set_title("GPS/IMU fusion (EKF) vs. ground truth" + NL +
             "721.1 m campus route, 1.5 m/s",
             fontsize=7.8, pad=5, loc="left")
ax.set_xlabel("Position error of fused estimate" + NL +
              "w.r.t. ground-truth trajectory (m)", fontsize=8)
ax.set_xlim(0, 30)
ax.set_xticks([0, 5, 10, 15, 20, 25, 30])
ax.tick_params(axis="both", length=2.5, pad=2)
ax.set_ylim(-0.6, len(scen) - 0.4)

for s in ("top", "right", "left"):
    ax.spines[s].set_visible(False)
ax.grid(axis="x", color="#D8DCE0", lw=0.5, zorder=0)
ax.set_axisbelow(True)

ax.legend(handles=[Patch(facecolor=BAR, edgecolor="white", label="RMSE over run"),
                   Line2D([0], [0], color=WHIS, lw=0.8, marker="|", ms=4.5,
                          label="max error")],
          loc="upper right", fontsize=6.8, frameon=False,
          handlelength=1.4, borderpad=0.2, labelspacing=0.3)

fig.tight_layout(pad=0.4)
out = "thesis_final_draft/images/gps_imu_scenario_summary.png"
fig.savefig(out, dpi=400)
fig.savefig(out.replace(".png", ".pdf"))
print("saved:", out)
