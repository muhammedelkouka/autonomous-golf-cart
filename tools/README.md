# Tools

Offline and helper tools that support the autonomy stack. None of them are part of the C++ build.
Run Python tools with Python 3.8 (see [REQUIREMENTS.md](../REQUIREMENTS.md)).

> Files stay in this folder rather than in sub-folders on purpose: the GUI launches
> `tools/pedestrian_detect.py` by path, and that script loads its models and calibration files
> from here.

## Mapping: building the maps the cart localises on

| Tool | Purpose |
|---|---|
| `map_merge/` | `merge_ply_maps.py` merges the PLY exports of one LiDAR recording (Edge / Planer / Intensity / Trajectory) into a single map for `runtime_data/`. Its header lists the exact command for each shipped map; each reproduces the file byte for byte. Includes a runnable demo with the forward-map exports and a preview image. |
| `extract_zone_path.cpp` | Cuts a sub-segment out of a recorded `Trajectory path.ply` to make a zone route (`--list` to inspect indices, `--range <ply> <a> <b> <out>` to cut). |
| `fwd_list*.csv`, `rev_list*.csv`, `*_extended.csv` | Index lists and extended route segments produced while cutting the zone routes. |

Raw map sources used by `map_merge/merge_ply_maps.py` are kept at the repository root:
`Fdirection map  top/`, `Rdirection top map/`, `danger zone map files/`.

## Pedestrian detection

| Tool | Purpose |
|---|---|
| `pedestrian_detect.py` | The live detector. Launched and stopped by the GUI's "Pedestrian Detection" button; writes `runtime_data/pedestrian_status.txt` (`STOP` / `CLEAR`). YOLOv8 (GPU) or MobileNet-SSD (CPU) backends, stereo distance estimation. |
| `PEDESTRIAN_DETECTION.md` | Detailed usage notes for the detector. |
| `models/`, `yolov8n.pt` | Detection models (MobileNet-SSD, YOLOv8n). |
| `stereo_calibrate.py` | Full OpenCV stereo calibration of the camera pair → `stereo_calib.npz`. |
| `stereo_distance_calib.py` | Empirical distance calibration using the checkerboard as a target. |
| `gen_checkerboard.py`, `checkerboard.pdf` | Printable calibration checkerboard (A3). |
| `calib_*.csv`, `stereo_*.csv`, `yolo_fps_test.csv` | Recorded calibration and benchmark data. |

## Drive-log analysis

Every autonomous run writes a 50 Hz log `ctrl_log_*.csv` to `runtime_data/`.

| Tool | Purpose |
|---|---|
| `analyze_drive.py` | Four-panel diagnostic of a drive run: steering pipeline, tracking, cross-track error, send gate. |
| `thesis_figures.py` | Six print-ready figures for one run: latency, localisation, tracking, steering and more. |
| `plot_trajectory.py` | Driven-vs-planned trajectory figures for a run. |
| `plot_campus_trajectory_zoom.py` | Two-panel trajectory figure with a magnified view of the worst deviation. |
| `check_june4.py` | Batch analysis of the June 4, 2026 campus runs (the heading-drift failure round). |

## Steering and localisation tests

| Tool | Purpose |
|---|---|
| `analyze_steer.py` | Analyses the steering test logs (`steer_test_*.csv`) from `steer_test.cpp`. |
| `analyze_loc.py` | Analyses the stationary localisation logs (`loc_test_stationary_*.csv`) from `loc_test.cpp`. |

## IMU

| Tool | Purpose |
|---|---|
| `imu_drift_test.py` | 60 s stationary yaw-drift test of the IM10A on COM11 (laptop + IMU only). |
| `imu_capture_im10a.py` | Logs the IM10A to CSV with real packet arrival times. |
| `imu_analysis/` | Smartphone IMU vs. dedicated IM10A comparison (`analyze_imu_comparison.py`, with its two data files). |

## Figures

| Tool | Purpose |
|---|---|
| `make_map_path_figure.py` | 3D top-LiDAR campus map with the recorded reference path. |
| `make_campus_map_path_figure.py` | 3D full-campus map with the smoothed reference path. |
| `make_full_campus_map_top_figure.py` | Top-down view of the full-campus map. |
| `make_gps_scenario_chart.py` | GPS/IMU fusion scenario comparison chart. |
| `make_failure_mechanism_figure.py` | The localisation–control coupling failure (heading drift). |
