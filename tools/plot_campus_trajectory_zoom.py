"""Two-panel driven-vs-planned trajectory figure for the thesis.

The single-panel version of this plot is misleading in the good direction: at
full-route scale the driven trajectory sits so close to the planned path that
the two render as one line, so the reader cannot see that there are two traces
at all, let alone how far apart they get.  This script draws the same run as a
route overview with the zoom window marked, plus a magnified panel centred on
the run's worst cross-track excursion, where the separation is actually
resolvable.

Usage:
    python tools/plot_campus_trajectory_zoom.py <ctrl_log.csv> <out.png>
"""

import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

PLANNED_C = "#e8833a"
DRIVEN_C = "#2f6fb5"
ZOOM_HALF_M = 6.0

# Panels are stacked vertically, not side by side, so each one gets the full
# width of the page and the detail view is as large as the route view. The
# figure is placed at ~0.92\textwidth (about 6.0in wide) against a 9in figure
# width here, so type is scaled down by roughly 0.66 on the page; sizes below
# are set to land near 9pt in the thesis.
matplotlib.rcParams.update({
    "font.size": 13,
    "axes.titlesize": 14,
    "axes.labelsize": 13,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12,
})


def load(path):
    rows = []
    with open(path, newline="", encoding="utf8", errors="ignore") as fh:
        rdr = csv.reader(fh)
        hdr = next(rdr)
        idx = {n: i for i, n in enumerate(hdr)}
        for r in rdr:
            if not r or r[0].startswith("#") or len(r) < len(hdr):
                continue
            try:
                rows.append((float(r[idx["robot_x"]]), float(r[idx["robot_y"]]),
                             float(r[idx["path_x"]]), float(r[idx["path_y"]]),
                             abs(float(r[idx["cte_m"]]))))
            except ValueError:
                continue
    return rows


def main():
    src, dst = sys.argv[1], sys.argv[2]
    rows = load(src)
    rx = [r[0] for r in rows]
    ry = [r[1] for r in rows]
    px = [r[2] for r in rows]
    py = [r[3] for r in rows]
    cte = [r[4] for r in rows]

    worst = max(range(len(cte)), key=lambda i: cte[i])
    cx, cy = rx[worst], ry[worst]
    mean_cte = sum(cte) / len(cte)

    # Stacked, not side by side: each panel then spans the full page width, so
    # the detail view is drawn as large as the route view rather than at half
    # the width, and both still land on a single page.
    fig, (ax, axz) = plt.subplots(
        2, 1, figsize=(9.0, 10.6),
        gridspec_kw=dict(height_ratios=[1.12, 1.0], hspace=0.28))

    for a in (ax, axz):
        a.plot(px, py, "--", color=PLANNED_C, lw=2.0, label="planned path")
        a.plot(rx, ry, "-", color=DRIVEN_C, lw=1.7, label="driven trajectory")
        a.set_xlabel("X (m)")
        a.set_ylabel("Y (m)")
        a.grid(alpha=0.3, lw=0.6)
        a.set_aspect("equal", adjustable="datalim")
    ax.plot(rx[0], ry[0], "o", color="#2e9e4f", ms=9, label="start", zorder=5)
    ax.plot(rx[-1], ry[-1], "X", color="#cc2b2b", ms=10, label="end", zorder=5)
    ax.add_patch(Rectangle((cx - ZOOM_HALF_M, cy - ZOOM_HALF_M),
                           2 * ZOOM_HALF_M, 2 * ZOOM_HALF_M,
                           fill=False, ec="0.25", lw=1.2, ls="-", zorder=6))
    ax.annotate("detail", xy=(cx + ZOOM_HALF_M, cy + ZOOM_HALF_M),
                xytext=(12, 12), textcoords="offset points",
                fontsize=13, color="0.25")
    ax.set_title("Full route (252.4 m)")
    ax.legend(loc="upper right", framealpha=0.95)

    axz.set_xlim(cx - ZOOM_HALF_M, cx + ZOOM_HALF_M)
    axz.set_ylim(cy - ZOOM_HALF_M, cy + ZOOM_HALF_M)
    axz.plot([cx, px[worst]], [cy, py[worst]], "-", color="0.35", lw=1.0)
    axz.plot(cx, cy, "o", color=DRIVEN_C, ms=5, zorder=5)
    axz.annotate("max |CTE| = %.3f m" % cte[worst],
                 xy=(cx, cy), xytext=(10, -22), textcoords="offset points",
                 fontsize=13, color="0.15",
                 arrowprops=dict(arrowstyle="->", color="0.35", lw=0.9))
    # No window size is quoted in this title: the equal-aspect adjustment
    # widens the view beyond the requested +/-ZOOM_HALF_M, so any figure
    # stated here would be wrong for one of the two axes.
    axz.set_title("Detail at the worst deviation")

    # No suptitle: a line long enough to carry both CTE figures overflows the
    # figure width and gets clipped at both ends. The caption states them.
    # tight_layout() is also avoided here -- it cannot handle equal-aspect
    # axes and warns; explicit margins are predictable instead.
    fig.subplots_adjust(left=0.11, right=0.97, top=0.96, bottom=0.06, hspace=0.30)
    fig.savefig(dst, dpi=200, facecolor="white")
    print("wrote %s  (worst CTE %.3f m at index %d)" % (dst, cte[worst], worst))


if __name__ == "__main__":
    main()
