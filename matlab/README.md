# MATLAB

The MATLAB work behind the system: the GPS/IMU study that ruled out GPS, the offline LiDAR mapping
that produced the first campus maps, and early offline localisation experiments. None of it is
needed to build or run the autonomy stack. The EKF used on the vehicle is the separate,
MATLAB-generated C code in [`../codegen/`](../codegen).

## `gps_imu_simulation/`: why the final system does not use GPS

A GPS/IMU Extended Kalman Filter simulated on the GIU campus route under five GPS-reception
scenarios, from open sky to heavily blocked. It showed that the position error is bounded by GPS
quality, not by the filter, which motivated the LiDAR-based localisation.

| File | Purpose |
|---|---|
| `run_all_scenarios.m` | **Entry point.** Runs every scenario over several noise seeds and prints a comparison table (with filter-consistency checks). |
| `giu_ekf_fusion.m` | One scenario end to end: route, simulated sensors, EKF, plots. Set `scenario = N` at the top to run one case. |
| `ExtendedKalmanFilter.m` | The EKF class used by the simulation. |
| `export_thesis_figures.m` | Saves the per-scenario figures and numbers after a sweep. |
| `GPS_IMU_realistic_scenario_V2_6.m` | Earlier self-contained version: A* path planning + GPS/IMU EKF on the campus map. |
| `kml2struct.m`, `kml2struct_LICENSE.txt` | Third-party KML reader (© 2012 James Slegers, BSD licence). |
| `route_cache.mat` | Cached campus route, so the simulation runs without the original KMZ file. |

Usage: put the folder on the MATLAB path and run `run_all_scenarios`.

## `lidar_mapping/`: building a map from a LiDAR recording

Offline scan-to-map NDT SLAM on Velodyne VLP-16 recordings (`.pcap`), producing a point-cloud map
and the vehicle trajectory.

| File | Purpose |
|---|---|
| `lidar_basicSLAM_workingversion.m` | Basic NDT SLAM on a PCAP recording. |
| `lidar_basicSLAM_workingrobustversionwithtrajectory.m` | More robust version that also saves the trajectory (`slam_results.mat`). |
| `lidar_basicSLAM_outlier_remove.m` | Refined scan-to-map NDT SLAM with outlier removal. |
| `segementation_of_the_map.m` | Mapping with IMU data and map segmentation. |

The raw `.pcap` recordings are not included (hundreds of MB each). Set `pcapFile` at the top of
each script to your recording.

## `lidar_localization_offline/`: early localisation experiments

| File | Purpose |
|---|---|
| `loc_pcapVsfilteredMap_V1.m` | Localises a recorded PCAP against a filtered map, with reference path and live scan views. |
| `loc_withloopclosure_refinment.m` | Hybrid localisation (consecutive scans + map reference) with loop-closure refinement. |

These prototypes preceded the real-time C++ scan matcher in `localization.cpp`.
