# -*- coding: utf-8 -*-
"""Paper figure: the localization-control coupling failure.

Source: Paper_Preparation/data/03_validation_runs/failure_round_8/
        ctrl_log_20260604_093320.csv  (pre-fix campus run)

Shows the mechanism in one image: heading walks monotonically, the commanded
steering angle grows to the actuator limit, yet cross-track error stays small
throughout. Position was fine; heading was not.
"""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC = (r"C:\Users\bios\Desktop\Paper_Preparation\data\03_validation_runs"
       r"\failure_round_8\ctrl_log_20260604_093320.csv")
OUT = (r"C:\Users\bios\Desktop\Isolated version V2\thesis_final_draft"
       r"\paper 1\F05_failure_mechanism.png")
LOCK = 23.0          # deg, mechanical steering limit

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 7.5,
    "mathtext.fontset": "stix",
    "axes.linewidth": 0.7,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
})

def _num(r):
    """Logs carry a trailing '# SUMMARY' block after the data rows."""
    try:
        float(r["time_s"]); return True
    except (TypeError, ValueError):
        return False

rows = [r for r in csv.DictReader(open(SRC)) if _num(r)]
g = lambda k: np.array([float(r[k]) for r in rows])
t, cte, steer, yaw = g("time_s"), g("cte_m"), g("steer_raw_deg"), g("robot_yaw_deg")
t = t - t[0]

BLUE, RED, GREY = "#2A6EA6", "#C0392B", "#5A6570"

# the diagnostic instant: steering already substantial while CTE is ~2 cm
k = int(np.argmin(np.abs(t - 2.0)))

fig, ax = plt.subplots(3, 1, figsize=(3.4, 3.6), sharex=True)

# (a) cross-track error
ax[0].axhline(0, color="#C8CDD2", lw=0.5)
ax[0].plot(t, cte, color=BLUE, lw=0.9)
ax[0].plot(t[k], cte[k], marker="o", ms=3, color=BLUE)
ax[0].annotate("%.0f cm" % (abs(cte[k]) * 100), xy=(t[k], cte[k]),
               xytext=(t[k] + 1.8, cte[k] + 0.55), fontsize=6.4, color=BLUE,
               arrowprops=dict(arrowstyle="-", lw=0.6, color=BLUE))
ax[0].set_ylabel("CTE (m)")
_m = 0.15 * (cte.max() - cte.min())
ax[0].set_ylim(cte.min() - _m, cte.max() + _m)

# (b) commanded steering
ax[1].axhspan(LOCK, 30, color=RED, alpha=0.09, lw=0)
ax[1].axhspan(-30, -LOCK, color=RED, alpha=0.09, lw=0)
ax[1].axhline(-LOCK, color=RED, ls="--", lw=0.8)
ax[1].axhline(LOCK, color=RED, ls="--", lw=0.8)
ax[1].plot(t, steer, color=RED, lw=0.9)
ax[1].plot(t[k], steer[k], marker="o", ms=3, color=RED)
ax[1].annotate("%.1f° already" % steer[k], xy=(t[k], steer[k]),
               xytext=(t[k] + 1.6, steer[k] - 15), fontsize=6.4, color=RED,
               arrowprops=dict(arrowstyle="-", lw=0.6, color=RED))
ax[1].text(0.985, 0.06, "mechanical lock", transform=ax[1].transAxes,
           ha="right", va="bottom", fontsize=6.2, color=RED)
ax[1].set_ylabel("steer cmd (deg)")
ax[1].set_ylim(-30, 30)

# (c) heading
ax[2].plot(t, yaw, color=GREY, lw=0.9)
ax[2].set_ylabel("heading (deg)")
ax[2].set_xlabel("time (s)")
ax[2].text(0.985, 0.90, "drifts %.1f° in %.0f s" % (yaw[0] - yaw[-1], t[-1]),
           transform=ax[2].transAxes, ha="right", va="top",
           fontsize=6.6, color=GREY)

for a in ax:
    a.grid(color="#E8EBED", lw=0.4)
    a.set_axisbelow(True)
    a.tick_params(labelsize=6.8, length=2.5, pad=2)
    for sp in ("top", "right"):
        a.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        a.spines[sp].set_color("#C8CDD2")
    a.axvline(t[k], color="#B0B6BC", lw=0.5, ls=":", zorder=0)

ax[2].set_xlim(t.min(), t.max())
fig.align_ylabels(ax)
fig.tight_layout(pad=0.3, h_pad=0.4)
fig.savefig(OUT, dpi=600, bbox_inches="tight", pad_inches=0.02)
fig.savefig(OUT.replace(".png", ".pdf"), bbox_inches="tight", pad_inches=0.02)

print("diagnostic instant t=%.2f s: CTE %.4f m, steer %.2f deg" % (t[k], cte[k], steer[k]))
i = int(np.argmax(np.abs(steer)))
print("run length %.1f s | %d ticks" % (t[-1], len(t)))
print("|CTE| max %.3f m, mean %.3f m" % (np.abs(cte).max(), np.abs(cte).mean()))
print("steer peak %.2f deg at t=%.1f s, CTE there = %.4f m" % (steer[i], t[i], cte[i]))
print("heading span %.1f deg (%.2f -> %.2f)" % (yaw.max() - yaw.min(), yaw[0], yaw[-1]))
print("saved:", OUT)
