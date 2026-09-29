"""
merge_ply_maps.py - build a localisation map from LiDAR-mapping PLY exports.

The LiDAR mapping software exports one recording as several VTK PLY files
(Edge map, Planer map, Intensity map, Trajectory path). This tool merges them
into the single point-cloud map that the autonomy stack loads from runtime_data/.

Output format (what localization.cpp's load_ply() expects):
    binary little-endian PLY, one vertex element, float x/y/z, NO face element.
The VTK exports carry an empty "element face 0 / property list ..." block; it is
dropped on purpose (an extra face property once broke the loader's stride).

Points are written in input order: all points of the first file, then the second,
and so on. No filtering is applied unless --zmin/--zmax is given.

Usage
-----
  python merge_ply_maps.py OUT.ply IN1.ply [IN2.ply ...] [--zmin Z] [--zmax Z]
                           [--preview IMAGE.png] [--compare REFERENCE.ply]

  --preview   also save a picture of the result: each input in its own colour
              (left) and the merged map coloured by height (right)
  --compare   check that OUT.ply is byte-identical to an existing map

Quick demo (inputs included in this folder; run from tools/map_merge/):

  python merge_ply_maps.py output/Campus_TopLidar_Forward_merged.ply ^
      "forward_map_exports/Edge map.ply" "forward_map_exports/Planer map.ply" ^
      "forward_map_exports/Intensity map.ply" "forward_map_exports/Trajectory path.ply" ^
      --preview output/merged_map_preview.png ^
      --compare ../../runtime_data/Campus_TopLidar_Forward.ply

Exact commands for the maps in runtime_data/ (run from the repository root).
Each reproduces the shipped map byte for byte (verified 2026-09-29).

  # Forward top-LiDAR campus map: 127,856 points, no filtering
  python tools/map_merge/merge_ply_maps.py runtime_data/Campus_TopLidar_Forward.ply ^
      "Fdirection map  top/Edge map.ply" "Fdirection map  top/Planer map.ply" ^
      "Fdirection map  top/Intensity map.ply" "Fdirection map  top/Trajectory path.ply"

  # Reverse top-LiDAR campus map: 129,230 points, no filtering
  python tools/map_merge/merge_ply_maps.py runtime_data/Campus_TopLidar_Reverse.ply ^
      "Rdirection top map/Edge map.ply" "Rdirection top map/Intensity edge map.ply" ^
      "Rdirection top map/Planer map.ply" "Rdirection top map/Trajectory path.ply"

  # Danger-zone map: 61,311 points cropped to 37,387 with -2.5 m <= z <= 6.0 m
  python tools/map_merge/merge_ply_maps.py runtime_data/DangerZone_Test.ply ^
      "danger zone map files/Edge map.ply" "danger zone map files/Planer map.ply" ^
      "danger zone map files/Intenisty edge map.ply" ^
      "danger zone map files/trajectory path map.ply" --zmin -2.5 --zmax 6.0

The point order only affects the file bytes, not localisation: the scan matcher
builds a distance field from the points, which is independent of their order.
"""
import argparse
import struct
import sys

import numpy as np


def read_ply_xyz(path):
    """Return an (N, 3) float32 array of the x/y/z vertices of a binary PLY."""
    with open(path, "rb") as f:
        data = f.read()
    end = data.find(b"end_header")
    if end < 0:
        raise ValueError(f"{path}: no end_header")
    body_start = data.index(b"\n", end) + 1
    header = data[:body_start].decode("ascii", "replace").splitlines()

    if not any(l.startswith("format binary_little_endian") for l in header):
        raise ValueError(f"{path}: only binary_little_endian PLY is supported")

    # Collect the vertex element's properties only (stop at the next element).
    n_vertex, props, in_vertex = None, [], False
    for line in header:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "element":
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                n_vertex = int(parts[2])
        elif parts[0] == "property" and in_vertex:
            if parts[1] == "list":
                raise ValueError(f"{path}: list property inside vertex element")
            props.append((parts[2], parts[1]))
    if n_vertex is None:
        raise ValueError(f"{path}: no vertex element")

    type_map = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
                "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
                "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
                "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}
    dtype = np.dtype([(name, type_map[t]) for name, t in props])
    verts = np.frombuffer(data, dtype=dtype, count=n_vertex, offset=body_start)
    return np.stack([verts["x"], verts["y"], verts["z"]], axis=1).astype(np.float32)


def write_ply_xyz(path, pts):
    header = ("ply\n"
              "format binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "end_header\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


def save_preview(path, named_parts, merged):
    """Top-view picture: inputs in separate colours (left), merged map by height (right)."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7.5), constrained_layout=True)
    colours = ["#1f77b4", "#2ca02c", "#ff7f0e", "#d62728", "#9467bd", "#8c564b"]
    for (name, p), c in zip(named_parts, colours):
        is_traj = "traj" in name.lower()
        ax1.scatter(p[:, 0], p[:, 1], s=4 if is_traj else 0.3, c=c, linewidths=0,
                    label=f"{os.path.splitext(os.path.basename(name))[0]} ({len(p):,d})",
                    zorder=3 if is_traj else 2)
    ax1.set_title("Input exports (one colour per file)")
    leg = ax1.legend(loc="best", fontsize=9)
    for h in getattr(leg, "legend_handles", None) or leg.legendHandles:
        h.set_sizes([40])

    order = np.argsort(merged[:, 2])
    sc = ax2.scatter(merged[order, 0], merged[order, 1], s=0.3, c=merged[order, 2],
                     cmap="viridis", linewidths=0)
    fig.colorbar(sc, ax=ax2, label="height z (m)", shrink=0.8)
    ax2.set_title(f"Merged map ({len(merged):,d} points, coloured by height)")

    for ax in (ax1, ax2):
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(True, alpha=0.3)
    fig.savefig(path, dpi=130)
    print(f"preview   -> {path}")


def main():
    ap = argparse.ArgumentParser(description="Merge VTK PLY exports into one localisation map.")
    ap.add_argument("output")
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--zmin", type=float, default=None, help="drop points with z below this (m)")
    ap.add_argument("--zmax", type=float, default=None, help="drop points with z above this (m)")
    ap.add_argument("--preview", default=None, help="save a PNG picture of the inputs and the result")
    ap.add_argument("--compare", default=None, help="check the output is byte-identical to this PLY")
    a = ap.parse_args()

    parts = []
    for p in a.inputs:
        pts = read_ply_xyz(p)
        print(f"  {len(pts):>8,d} pts  {p}")
        parts.append(pts)
    pts = np.concatenate(parts)

    if a.zmin is not None or a.zmax is not None:
        keep = np.ones(len(pts), bool)
        if a.zmin is not None:
            keep &= pts[:, 2] >= a.zmin
        if a.zmax is not None:
            keep &= pts[:, 2] <= a.zmax
        print(f"  z-crop kept {keep.sum():,d} of {len(pts):,d} points")
        pts = pts[keep]

    write_ply_xyz(a.output, pts)
    print(f"wrote {len(pts):,d} points -> {a.output}")

    if a.preview:
        save_preview(a.preview, list(zip(a.inputs, parts)), pts)

    if a.compare:
        with open(a.output, "rb") as f1, open(a.compare, "rb") as f2:
            same = f1.read() == f2.read()
        print(f"compare   -> {'IDENTICAL to' if same else 'DIFFERENT from'} {a.compare}")
        return 0 if same else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
