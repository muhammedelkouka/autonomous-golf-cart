# -*- coding: utf-8 -*-
"""Top-down (orthographic) figure of the front-LiDAR full-campus map, no path.

Map : runtime_data/Full_Campus_Map_Master.ply  (VLP-16, 129142 pts, ASCII)
Same styling as the top view in make_map_path_figure.py. No mapping trajectory
was saved for this map, so only the point cloud is drawn.
Run from the project root.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MAP = "runtime_data/Full_Campus_Map_Master.ply"

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

# drop sparse height outliers (stray returns) for display only
zlo, zhi = np.percentile(cloud[:, 2], [0.5, 99.5])
cloud = cloud[(cloud[:, 2] >= zlo) & (cloud[:, 2] <= zhi)]

# crop to where the cloud actually is, so sparse stragglers don't pad the box
x0, x1 = np.percentile(cloud[:, 0], [0.3, 99.7])
y0, y1 = np.percentile(cloud[:, 1], [0.3, 99.7])
mx, my = 0.02 * (x1 - x0), 0.02 * (y1 - y0)

fig, ax = plt.subplots(figsize=(3.4, 3.3))

sc = ax.scatter(cloud[:, 0], cloud[:, 1], c=cloud[:, 2], cmap="viridis",
                s=0.05, alpha=0.8, linewidths=0, rasterized=True, zorder=1)

ax.set_xlim(x0 - mx, x1 + mx)
ax.set_ylim(y0 - my, y1 + my)
ax.set_aspect("equal")
ax.set_xlabel("x (m)", fontsize=8)
ax.set_ylabel("y (m)", fontsize=8)
ax.tick_params(labelsize=6.8, length=2.5, pad=2)
ax.grid(color="#E8EBED", lw=0.4, zorder=0)
ax.set_axisbelow(True)
for sp in ax.spines.values():
    sp.set_color("#C8CDD2")
    sp.set_linewidth(0.6)

cb = fig.colorbar(sc, ax=ax, shrink=0.75, aspect=22, pad=0.03)
cb.set_label("height z (m)", fontsize=7)
cb.ax.tick_params(labelsize=6.5, length=2)
cb.solids.set_alpha(1.0)

ax.set_title("Full campus map, top view", fontsize=7.8, pad=4)

fig.tight_layout(pad=0.2)
out = "thesis_final_draft/images/full_campus_map_top.png"
fig.savefig(out, dpi=600, bbox_inches="tight", pad_inches=0.02)
fig.savefig(out.replace(".png", ".pdf"), dpi=600, bbox_inches="tight", pad_inches=0.02)
print("map pts: %d | z [%.2f, %.2f]" % (len(cloud), zlo, zhi))
print("saved:", out)
