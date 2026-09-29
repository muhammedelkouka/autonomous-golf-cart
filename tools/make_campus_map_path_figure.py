# -*- coding: utf-8 -*-
"""Conference-paper figure: 3D campus LiDAR map + smoothed reference path.
Map  : runtime_data/Full_Campus_Map_Master.ply   (VLP-16, 129142 pts, ASCII)
Path : planned/smoothed path recorded in the validated 2026-06-22 campus run
       (ctrl_log_20260622_111227.csv, path_x/path_y, reconstructed by path_index)
Both are in the same map frame -- verified: ~169 map points lie within 3 m of
every sampled path point.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.lines import Line2D

RT   = "runtime_data/"
MAP  = RT + "Full_Campus_Map_Master.ply"
LOG  = RT + "ctrl_log_20260622_111227.csv"
PATH_Z = 1.0          # draw the path just above ground so it reads over the cloud

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

# reconstruct the planned path polyline: one point per unique path_index, in order
d = np.genfromtxt(LOG, delimiter=",", names=True, invalid_raise=False)
idx = d["path_index"].astype(int)
keep = np.concatenate(([True], np.diff(idx) != 0))
px, py = d["path_x"][keep], d["path_y"][keep]
order = np.argsort(idx[keep])
px, py = px[order], py[order]
route_len = np.hypot(np.diff(px), np.diff(py)).sum()

# drop sparse height outliers (stray returns) for display only
zlo, zhi = np.percentile(cloud[:, 2], [0.5, 99.0])
cloud = cloud[(cloud[:, 2] >= zlo) & (cloud[:, 2] <= zhi)]

rng = np.random.default_rng(0)
N = 60000
if len(cloud) > N:
    cloud = cloud[rng.choice(len(cloud), N, replace=False)]

fig = plt.figure(figsize=(3.6, 2.9))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                c=cloud[:, 2], cmap="viridis", s=0.05, alpha=0.5,
                linewidths=0, rasterized=True, zorder=1)

pz = np.full_like(px, PATH_Z)
ax.plot(px, py, pz, color="#D62728", lw=1.7, zorder=6)
ax.scatter([px[0]], [py[0]], [PATH_Z], color="#111111", s=11,
           marker="o", depthshade=False, zorder=7)

dx = cloud[:, 0].max() - cloud[:, 0].min()
dy = cloud[:, 1].max() - cloud[:, 1].min()
dz = cloud[:, 2].max() - cloud[:, 2].min()
ax.set_box_aspect((dx, dy, dz * 3.0))
ax.view_init(elev=58, azim=-60)

ax.set_xlabel("x (m)", fontsize=7.5, labelpad=-4)
ax.set_ylabel("y (m)", fontsize=7.5, labelpad=-2)
ax.set_zlabel("")
ax.set_zticks([])          # height is read from the colorbar
ax.tick_params(labelsize=6.5, pad=-2)

for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
    axis.pane.set_facecolor("white")
    axis.pane.set_edgecolor("white")
    axis.pane.set_alpha(1.0)
    axis.line.set_color("#B8BEC4")
    axis._axinfo["grid"].update(color="#E8EBED", linewidth=0.4)

cb = fig.colorbar(sc, ax=ax, shrink=0.5, aspect=14, pad=0.02)
cb.set_label("height z (m)", fontsize=7)
cb.ax.tick_params(labelsize=6.5, length=2)
cb.solids.set_alpha(1.0)

ax.legend(handles=[Line2D([0], [0], color="#D62728", lw=1.7,
                          label="reference path (%.0f m)" % route_len),
                   Line2D([0], [0], color="#111111", lw=0, marker="o", ms=3.5,
                          label="start")],
          loc="upper left", fontsize=6.8, frameon=False,
          handlelength=1.4, borderpad=0.1, labelspacing=0.25,
          bbox_to_anchor=(0.02, 0.97))

ax.set_title("Campus LiDAR map with smoothed reference path",
             fontsize=7.8, pad=-14)

fig.tight_layout(pad=0.1)
out = "thesis_final_draft/images/campus_map_reference_path.png"
fig.savefig(out, dpi=400, bbox_inches="tight", pad_inches=0.02)
fig.savefig(out.replace(".png", ".pdf"), dpi=400, bbox_inches="tight", pad_inches=0.02)
print("map pts drawn: %d | path pts: %d | route: %.1f m" % (len(cloud), len(px), route_len))
print("saved:", out)
