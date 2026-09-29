# -*- coding: utf-8 -*-
"""Conference-paper figure: 3D LiDAR point-cloud map + recorded reference path.

Map  : runtime_data/Campus_TopLidar_Forward.ply   (top-mounted VLP-16, 127856 pts,
       merged from the Edge / Planer / Intensity / Trajectory VTK exports)
Path : Fdirection map  top/Trajectory path.ply    (the build trajectory, 7995 pts)

The path is read from the trajectory PLY rather than path_forward_full.csv:
the two are identical in XY (verified elementwise, max |dx|=|dy|=0.000 m), but
the CSV flattens z to a constant 0.2 m, whereas the PLY keeps the true sensor
height. Since the trajectory PLY is one of the four clouds merged into the map,
drawing it at its own z makes the red line land exactly on the trajectory that
is visible inside the point cloud. Any other height (a flat value, or a
ground-following offset) projects to a visible sideways displacement.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.lines import Line2D

MAP  = "runtime_data/Campus_TopLidar_Forward.ply"
PATH = "Fdirection map  top/Trajectory path.ply"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8,
    "mathtext.fontset": "stix",
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
})


def load_ply(path):
    """Minimal PLY reader. Counts properties of the *vertex* element only
    (a face element's property list would otherwise corrupt the stride)."""
    f = open(path, "rb")
    nvert, fmt, in_vertex, props = 0, None, False, 0
    while True:
        line = f.readline().decode("ascii", "ignore").strip()
        if line.startswith("format"):
            fmt = line.split()[1]
        elif line.startswith("element"):
            parts = line.split()
            in_vertex = (parts[1] == "vertex")
            if in_vertex:
                nvert = int(parts[2])
        elif line.startswith("property") and in_vertex:
            props += 1
        if line == "end_header":
            break
    if fmt == "binary_little_endian":
        raw = np.frombuffer(f.read(nvert * props * 4), dtype="<f4")
        pts = raw.reshape(nvert, props)[:, :3]
    else:
        pts = np.loadtxt(f, max_rows=nvert)[:, :3]
    f.close()
    return np.asarray(pts, dtype=float)


cloud = load_ply(MAP)
path  = load_ply(PATH)
px, py, pz = path[:, 0], path[:, 1], path[:, 2]
route_len = np.hypot(np.diff(px), np.diff(py)).sum()

# drop sparse height outliers (stray returns) for display only
zlo, zhi = np.percentile(cloud[:, 2], [0.5, 99.5])
cloud = cloud[(cloud[:, 2] >= zlo) & (cloud[:, 2] <= zhi)]

fig = plt.figure(figsize=(3.6, 3.0))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                c=cloud[:, 2], cmap="viridis", s=0.035, alpha=0.75,
                linewidths=0, rasterized=True, zorder=1)

ax.plot(px, py, pz, color="#D62728", lw=0.75, zorder=6)
ax.scatter([px[0]], [py[0]], [pz[0]], color="#111111", s=7,
           marker="o", depthshade=False, zorder=7)

# crop to where the cloud actually is, so sparse stragglers don't pad the box
x0, x1 = np.percentile(cloud[:, 0], [0.3, 99.7])
y0, y1 = np.percentile(cloud[:, 1], [0.3, 99.7])
mx, my = 0.02 * (x1 - x0), 0.02 * (y1 - y0)
ax.set_xlim(x0 - mx, x1 + mx)
ax.set_ylim(y0 - my, y1 + my)

dx, dy = x1 - x0, y1 - y0
dz = cloud[:, 2].max() - cloud[:, 2].min()
ax.set_box_aspect((dx, dy, dz * 3.0))     # exaggerate height so structure reads
ax.view_init(elev=58, azim=-60)

ax.set_xlabel("x (m)", fontsize=7.5, labelpad=-4)
ax.set_ylabel("y (m)", fontsize=7.5, labelpad=-2)
ax.set_zlabel("")
ax.set_zticks([])                          # height is read from the colorbar
ax.tick_params(labelsize=6.5, pad=-2)

for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
    axis.pane.set_facecolor("white")
    axis.pane.set_edgecolor("white")
    axis.pane.set_alpha(1.0)
    axis.line.set_color("#B8BEC4")
    axis._axinfo["grid"].update(color="#E8EBED", linewidth=0.4)

cb = fig.colorbar(sc, ax=ax, shrink=0.42, aspect=16, pad=0.06)
cb.set_label("height z (m)", fontsize=7)
cb.ax.tick_params(labelsize=6.5, length=2)
cb.solids.set_alpha(1.0)

ax.legend(handles=[Line2D([0], [0], color="#D62728", lw=1.2,
                          label="reference path (%.0f m)" % route_len),
                   Line2D([0], [0], color="#111111", lw=0, marker="o", ms=3.5,
                          label="start")],
          loc="upper left", fontsize=6.8, frameon=False,
          handlelength=1.4, borderpad=0.1, labelspacing=0.25,
          bbox_to_anchor=(0.02, 0.97))

ax.set_title("LiDAR point-cloud map with reference path",
             fontsize=7.8, pad=-14)

fig.tight_layout(pad=0.1)
out = "thesis_final_draft/images/lidar_map_reference_path.png"
fig.savefig(out, dpi=600, bbox_inches="tight", pad_inches=0.02)
fig.savefig(out.replace(".png", ".pdf"), dpi=600, bbox_inches="tight", pad_inches=0.02)
print("map pts: %d | path pts: %d | route %.1f m | path z [%.2f, %.2f]"
      % (len(cloud), len(px), route_len, pz.min(), pz.max()))
print("saved:", out)


# ---------------------------------------------------------------------------
# Second figure: true top-down (orthographic) plan view of the same data.
# A 2D axes rather than a 3D view from above, so there is no perspective at all.
# ---------------------------------------------------------------------------
fig2, ax2 = plt.subplots(figsize=(3.4, 3.3))

sc2 = ax2.scatter(cloud[:, 0], cloud[:, 1], c=cloud[:, 2], cmap="viridis",
                  s=0.05, alpha=0.8, linewidths=0, rasterized=True, zorder=1)
ax2.plot(px, py, color="#D62728", lw=0.9, zorder=5)
ax2.plot(px[0], py[0], marker="o", color="#111111", ms=3.5, zorder=6)

ax2.set_xlim(x0 - mx, x1 + mx)
ax2.set_ylim(y0 - my, y1 + my)
ax2.set_aspect("equal")
ax2.set_xlabel("x (m)", fontsize=8)
ax2.set_ylabel("y (m)", fontsize=8)
ax2.tick_params(labelsize=6.8, length=2.5, pad=2)
ax2.grid(color="#E8EBED", lw=0.4, zorder=0)
ax2.set_axisbelow(True)
for sp in ax2.spines.values():
    sp.set_color("#C8CDD2")
    sp.set_linewidth(0.6)

cb2 = fig2.colorbar(sc2, ax=ax2, shrink=0.75, aspect=22, pad=0.03)
cb2.set_label("height z (m)", fontsize=7)
cb2.ax.tick_params(labelsize=6.5, length=2)
cb2.solids.set_alpha(1.0)

ax2.legend(handles=[Line2D([0], [0], color="#D62728", lw=1.2,
                           label="reference path (%.0f m)" % route_len),
                    Line2D([0], [0], color="#111111", lw=0, marker="o", ms=3.5,
                           label="start")],
           loc="lower left", fontsize=6.8, frameon=False,
           handlelength=1.4, borderpad=0.6, labelspacing=0.3)

ax2.set_title("Map and reference path, top view", fontsize=7.8, pad=4)

fig2.tight_layout(pad=0.2)
out2 = "thesis_final_draft/images/lidar_map_reference_path_top.png"
fig2.savefig(out2, dpi=600, bbox_inches="tight", pad_inches=0.02)
fig2.savefig(out2.replace(".png", ".pdf"), dpi=600, bbox_inches="tight", pad_inches=0.02)
print("saved:", out2)
