#!/usr/bin/env python3
"""
Empirical stereo distance calibration using the checkerboard as the target,
instead of a person (tools/pedestrian_detect.py --auto-calib) or full camera
intrinsics (tools/stereo_calibrate.py).

Same underlying method/model as the original person-based calibration that gave
the validated, walk-tested 3m point this morning — distance_m = stereo_a /
(disparity_px - stereo_b), fitted by holding the target at known tape-measured
distances and logging the cross-camera disparity — but using the checkerboard's
corner CENTROID (mean of all detected corners in each camera) instead of a
person's bounding-box center as the disparity reference point. Two real
advantages over the person-based version:
  1. Precision: cornerSubPix corners average over ~54 points, far less jitter
     than one YOLO/MobileNet box edge.
  2. The centroid is immune to the corner-ORDER ambiguity that broke
     tools/stereo_calibrate.py's full intrinsics calibration at 45-degree tilts
     (found 2026-07-27) — averaging a set of points doesn't care what order
     they're summed in. Board should be held roughly FLAT/facing the camera at
     each distance (no tilting needed here, we're not fitting lens distortion).

Board: generate with tools/gen_checkerboard.py (A3 PDF by default).
--square-size-mm is not required here (no metric board geometry is used, just
pixel disparity vs the ground-truth walked distance) — pass --cols/--rows only
if you changed the board's corner count from the 9x6 default.

Hands-free: beep-cued walk/hold sequence, same UX as pedestrian_detect.py's
--auto-calib. Stand at each --distances value in turn, board held flat facing
the cameras, centered; hold still through the hold phase.

Run:
    python tools/stereo_distance_calib.py --distances 1,1.5,2,2.5,3,3.5,4

Output: prints the fitted stereo_a/stereo_b to paste into main.cpp's
ped_set_enabled() launch string, plus per-point residuals, and a CSV log.
"""
import argparse
import time
import winsound

import cv2
import numpy as np


def open_camera(index, width, height, fourcc):
    cap = None
    for attempt in range(4):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            break
        cap.release()
        time.sleep(0.5)
    if not cap.isOpened():
        raise RuntimeError(f"camera index {index} failed to open after 4 attempts")
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def show(label, frame, text, color):
    d = frame.copy()
    cv2.rectangle(d, (0, 0), (d.shape[1], 46), (20, 20, 20), -1)
    cv2.putText(d, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.imshow(f"stereo_distance_calib {label}", d)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cam0", type=int, default=0)
    ap.add_argument("--cam1", type=int, default=1)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fourcc", type=str, default="MJPG")
    ap.add_argument("--cols", type=int, default=9, help="internal corners across")
    ap.add_argument("--rows", type=int, default=6, help="internal corners down")
    ap.add_argument("--distances", type=str, default="1,1.5,2,2.5,3,3.5,4",
                     help="comma-separated ground-truth metres to walk through, tape-measured")
    ap.add_argument("--prep-s", type=float, default=8.0, help="walk time before each hold")
    ap.add_argument("--hold-s", type=float, default=6.0, help="seconds to log samples at each distance")
    ap.add_argument("--out-csv", type=str, default="tools/stereo_distance_calib.csv")
    ap.add_argument("--no-beep", action="store_true")
    args = ap.parse_args()

    beep = (lambda f, d: None) if args.no_beep else winsound.Beep
    board_size = (args.cols, args.rows)
    distances = [float(x) for x in args.distances.split(",") if x.strip()]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    print(f"Opening cameras (cam0={args.cam0}, cam1={args.cam1})...")
    cap_a = open_camera(args.cam0, args.width, args.height, args.fourcc)
    cap_b = open_camera(args.cam1, args.width, args.height, args.fourcc)

    rows_out = []  # (distance_m, disparity_px)
    print(f"Distances to walk (tape-measured from the lens plane): {distances}")
    print("Hold the board FLAT, facing the cameras, centered. Beeps: countdown = walk time, "
          "high tone = start logging (hold still), low tone = stop/move to next.\n")

    try:
        for d in distances:
            print(f"[calib] walk to {d:.2f}m — you have {args.prep_s:.0f}s", flush=True)
            t_end = time.time() + args.prep_s
            countdown_beeps = 0 if args.no_beep else min(3, int(args.prep_s))
            while time.time() < t_end:
                ret_a, frame_a = cap_a.read()
                ret_b, frame_b = cap_b.read()
                remaining = max(0.0, t_end - time.time())
                if ret_a:
                    show("cam0", frame_a, f"WALK TO {d:.2f}m [{remaining:0.1f}s]", (0, 165, 255))
                if ret_b:
                    show("cam1", frame_b, f"WALK TO {d:.2f}m [{remaining:0.1f}s]", (0, 165, 255))
                key = cv2.waitKey(15) & 0xFF
                if key in (ord('q'), 27):
                    print("aborted by user.")
                    cv2.destroyAllWindows()
                    return
            beep(1800, 300)

            samples = []
            t_end = time.time() + args.hold_s
            while time.time() < t_end:
                ret_a, frame_a = cap_a.read()
                ret_b, frame_b = cap_b.read()
                if not (ret_a and ret_b):
                    continue
                gray_a = cv2.cvtColor(frame_a, cv2.COLOR_BGR2GRAY)
                gray_b = cv2.cvtColor(frame_b, cv2.COLOR_BGR2GRAY)
                found_a, corners_a = cv2.findChessboardCorners(gray_a, board_size, flags=find_flags)
                found_b, corners_b = cv2.findChessboardCorners(gray_b, board_size, flags=find_flags)
                remaining = max(0.0, t_end - time.time())
                disp_a, disp_b = frame_a.copy(), frame_b.copy()
                if found_a:
                    cv2.drawChessboardCorners(disp_a, board_size, corners_a, found_a)
                if found_b:
                    cv2.drawChessboardCorners(disp_b, board_size, corners_b, found_b)
                col = (0, 200, 0) if (found_a and found_b) else (0, 0, 200)
                show("cam0", disp_a, f"LOGGING {d:.2f}m [{remaining:0.1f}s] n={len(samples)}", col)
                show("cam1", disp_b, f"LOGGING {d:.2f}m [{remaining:0.1f}s] n={len(samples)}", col)

                if found_a and found_b:
                    corners_a = cv2.cornerSubPix(gray_a, corners_a, (11, 11), (-1, -1), criteria)
                    corners_b = cv2.cornerSubPix(gray_b, corners_b, (11, 11), (-1, -1), criteria)
                    cx_a = float(np.mean(corners_a[:, 0, 0]))
                    cx_b = float(np.mean(corners_b[:, 0, 0]))
                    samples.append(abs(cx_a - cx_b))

                key = cv2.waitKey(15) & 0xFF
                if key in (ord('q'), 27):
                    print("aborted by user.")
                    cv2.destroyAllWindows()
                    return

            beep(500, 400)
            if samples:
                med = float(np.median(samples))
                print(f"   -> {len(samples)} samples, median disparity {med:.2f}px", flush=True)
                rows_out.append((d, med))
            else:
                print(f"   -> NO samples (board never detected in both cameras) — skipped", flush=True)
    finally:
        cap_a.release()
        cap_b.release()
        cv2.destroyAllWindows()

    for f in (1000, 1300, 1600):
        beep(f, 150)

    if len(rows_out) < 3:
        print(f"\nOnly {len(rows_out)} usable distance(s) — need at least 3 to fit. Aborting.")
        return

    with open(args.out_csv, "w") as f:
        f.write("distance_m,median_disparity_px\n")
        for d, disp in rows_out:
            f.write(f"{d:.3f},{disp:.3f}\n")
    print(f"\nWrote {args.out_csv}")

    # distance_m = A / (disparity - B)  <=>  1/distance = disparity/A - B/A  (linear in disparity)
    xs = np.array([disp for _, disp in rows_out])
    ys = np.array([1.0 / d for d, _ in rows_out])
    slope, intercept = np.polyfit(xs, ys, 1)
    stereo_a = 1.0 / slope
    stereo_b = -intercept * stereo_a

    print(f"\nFitted: stereo_a={stereo_a:.2f}  stereo_b={stereo_b:.2f}")
    print("Residuals (predicted vs ground truth):")
    for d, disp in rows_out:
        pred = stereo_a / (disp - stereo_b) if disp > stereo_b else float("nan")
        print(f"  {d:.2f}m: disparity={disp:.2f}px -> predicted {pred:.3f}m "
              f"(residual {pred - d:+.3f}m)")

    print(f"\nPaste into main.cpp's ped_set_enabled(): "
          f"--stereo-a {stereo_a:.2f} --stereo-b {stereo_b:.2f}")


if __name__ == "__main__":
    main()
