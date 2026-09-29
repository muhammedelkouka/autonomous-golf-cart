#!/usr/bin/env python3
"""
Real OpenCV stereo camera calibration for the pedestrian-detection camera pair.

Replaces the empirical distance_m = stereo_a/(disparity_px - stereo_b) curve-fit
(fitted from only 4 tape-measured points, see PEDESTRIAN_DETECTION.md) with real
per-camera intrinsics + lens distortion (cv2.calibrateCamera) and real stereo
geometry (cv2.stereoCalibrate + cv2.stereoRectify) measured from a checkerboard —
the standard, professional way to calibrate a stereo rig. This fixes the off-axis
distance error at its source (undistorting each detection point before computing
disparity) instead of just excluding off-axis people via the lane-zone.

Board: generate with tools/gen_checkerboard.py first (A3-sized PDF by default).
--square-size-mm MUST be the value you physically measured with a ruler on the
actual printed board, not the value gen_checkerboard.py targeted.

GUIDED capture (2026-07-27 rewrite): three prior free-roaming attempts ("move the
board around") all diverged wildly between runs (rectified focal length 335px /
1772px / 37356px across three tries on the same rig) — the signature of
insufficient/inconsistent pose diversity, confirmed by the user reporting the
"varied" movement was actually fairly random. This version walks through a FIXED
list of 20 explicit poses (distance + tilt + frame position all specified) one at
a time: shows an on-screen countdown to get into position, then a capture window
that grabs the pose as soon as both cameras see it cleanly. Guarantees the actual
diversity a stereo calibration needs instead of relying on memory/judgement while
also watching a video feed.

Run:
    python tools/stereo_calibrate.py --square-size-mm 39
    (needs both cameras connected; live preview windows show pose instructions +
    detected corners; 'q'/Esc aborts without saving)

Output: tools/stereo_calib.npz — camera matrices, distortion coefficients,
R/T (stereo rotation/translation, i.e. calibrated baseline), rectification
maps. Loaded by pedestrian_detect.py via --calib-file to replace the old
empirical formula.
"""
import argparse
import time
import winsound

import cv2
import numpy as np

# Each entry: on-screen/console instruction. Deliberately fixed and explicit — NOT
# "move around" — covering near/mid/far distance, all 4 tilt directions, and all
# frame quadrants/edges, so pose diversity is guaranteed rather than hoped for.
GUIDED_POSES = [
    "1/20  FLAT, CENTER, ~50cm (near)",
    "2/20  FLAT, CENTER, ~150cm (far)",
    "3/20  TILT LEFT ~45deg, CENTER, ~70cm",
    "4/20  TILT RIGHT ~45deg, CENTER, ~70cm",
    "5/20  TILT UP ~45deg (top edge away), CENTER, ~70cm",
    "6/20  TILT DOWN ~45deg (bottom edge away), CENTER, ~70cm",
    "7/20  FLAT, UPPER-LEFT of frame, ~80cm",
    "8/20  FLAT, UPPER-RIGHT of frame, ~80cm",
    "9/20  FLAT, LOWER-LEFT of frame, ~80cm",
    "10/20 FLAT, LOWER-RIGHT of frame, ~80cm",
    "11/20 FLAT, CENTER, ~100cm",
    "12/20 TILT LEFT ~45deg, UPPER-LEFT of frame, ~90cm",
    "13/20 TILT RIGHT ~45deg, UPPER-RIGHT of frame, ~90cm",
    "14/20 TILT UP ~45deg, LOWER area, ~90cm",
    "15/20 TILT DOWN ~45deg, UPPER area, ~90cm",
    "16/20 FLAT, LEFT EDGE of frame, ~90cm",
    "17/20 FLAT, RIGHT EDGE of frame, ~90cm",
    "18/20 FLAT, TOP EDGE of frame, ~90cm",
    "19/20 FLAT, BOTTOM EDGE of frame, ~90cm",
    "20/20 FLAT, CENTER, ~180cm (far)",
]


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
    cv2.imshow(f"stereo_calibrate {label}", d)


def run_pose(pose_desc, args, cap_a, cap_b, board_size, criteria, find_flags, beep, prep_s):
    """Prep countdown, then a capture window that grabs the pose as soon as both
    cameras see it cleanly. Returns ('ok', corners_a, corners_b, img_size),
    ('missed', None, None, None), or ('abort', None, None, None)."""
    t_end = time.time() + prep_s
    while time.time() < t_end:
        ret_a, frame_a = cap_a.read()
        ret_b, frame_b = cap_b.read()
        remaining = max(0.0, t_end - time.time())
        if ret_a:
            show("cam0", frame_a, f"GET READY [{remaining:0.1f}s]: {pose_desc}", (0, 165, 255))
        if ret_b:
            show("cam1", frame_b, f"GET READY [{remaining:0.1f}s]: {pose_desc}", (0, 165, 255))
        key = cv2.waitKey(15) & 0xFF
        if key in (ord('q'), 27):
            return "abort", None, None, None

    beep(1800, 150)
    t_end = time.time() + args.capture_window_s
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
        show("cam0", disp_a, f"CAPTURING [{remaining:0.1f}s]: {pose_desc}", col)
        show("cam1", disp_b, f"CAPTURING [{remaining:0.1f}s]: {pose_desc}", col)

        if found_a and found_b:
            corners_a = cv2.cornerSubPix(gray_a, corners_a, (11, 11), (-1, -1), criteria)
            corners_b = cv2.cornerSubPix(gray_b, corners_b, (11, 11), (-1, -1), criteria)
            beep(1500, 120)
            return "ok", corners_a, corners_b, (gray_a.shape[1], gray_a.shape[0])

        key = cv2.waitKey(15) & 0xFF
        if key in (ord('q'), 27):
            return "abort", None, None, None

    return "missed", None, None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cam0", type=int, default=0)
    ap.add_argument("--cam1", type=int, default=1)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fourcc", type=str, default="MJPG")
    ap.add_argument("--cols", type=int, default=9, help="internal corners across")
    ap.add_argument("--rows", type=int, default=6, help="internal corners down")
    ap.add_argument("--square-size-mm", type=float, required=True,
                     help="MEASURED (with a ruler) square size of the actual printed board — "
                          "do not pass the --square-size-mm you asked gen_checkerboard.py for, "
                          "pass what you actually measured")
    ap.add_argument("--prep-s", type=float, default=6.0,
                     help="seconds shown to get into each pose before the capture window opens")
    ap.add_argument("--capture-window-s", type=float, default=10.0,
                     help="seconds to wait for a clean dual-camera detection of each pose "
                          "before giving up on it and moving to the next")
    ap.add_argument("--lead-in-s", type=float, default=12.0,
                     help="extra prep time added ONLY to pose 1, so there's time to walk over "
                          "and get set up before anything starts (2026-07-27: all 6 attempted "
                          "poses were missed on a run using only the normal --prep-s for pose 1)")
    ap.add_argument("--min-good-poses", type=int, default=10,
                     help="abort calibration (don't save) if fewer than this many of the "
                          "20 poses were successfully captured")
    ap.add_argument("--out", type=str, default="tools/stereo_calib.npz")
    ap.add_argument("--no-beep", action="store_true")
    args = ap.parse_args()

    beep = (lambda f, d: None) if args.no_beep else winsound.Beep
    board_size = (args.cols, args.rows)
    square_m = args.square_size_mm / 1000.0

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * square_m

    print(f"Opening cameras (cam0={args.cam0}, cam1={args.cam1})...")
    cap_a = open_camera(args.cam0, args.width, args.height, args.fourcc)
    cap_b = open_camera(args.cam1, args.width, args.height, args.fourcc)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    objpoints, imgpoints_a, imgpoints_b = [], [], []
    img_size = None
    missed = []
    print(f"Guided capture: {len(GUIDED_POSES)} fixed poses. Get into each pose during the "
          "orange countdown; it captures automatically once both cameras see the board "
          "cleanly (turns green). 'q'/Esc aborts without saving.\n")

    try:
        for i, pose_desc in enumerate(GUIDED_POSES):
            prep_s = args.prep_s + (args.lead_in_s if i == 0 else 0.0)
            print(pose_desc, flush=True)
            status, ca, cb, sz = run_pose(pose_desc, args, cap_a, cap_b, board_size,
                                           criteria, find_flags, beep, prep_s)
            if status == "abort":
                print("aborted by user.")
                cv2.destroyAllWindows()
                return
            if status == "missed":
                print("   -> MISSED (no clean detection in time), skipping this pose")
                missed.append(pose_desc)
                continue
            objpoints.append(objp.copy())
            imgpoints_a.append(ca)
            imgpoints_b.append(cb)
            img_size = sz
            print(f"   -> captured ({len(objpoints)} good so far)")
    finally:
        cap_a.release()
        cap_b.release()
        cv2.destroyAllWindows()

    for f in (1000, 1300, 1600):
        beep(f, 150)
    print(f"\nCaptured {len(objpoints)}/{len(GUIDED_POSES)} poses "
          f"({len(missed)} missed: {missed if missed else 'none'})")
    if len(objpoints) < args.min_good_poses:
        print(f"Only {len(objpoints)} good poses (< --min-good-poses {args.min_good_poses}) — "
              "not enough for a trustworthy calibration. Aborting without saving.")
        return

    print("Running calibration...")
    rms_a, mtx_a, dist_a, _, _ = cv2.calibrateCamera(objpoints, imgpoints_a, img_size, None, None)
    rms_b, mtx_b, dist_b, _, _ = cv2.calibrateCamera(objpoints, imgpoints_b, img_size, None, None)
    print(f"cam0 solo reprojection RMS error: {rms_a:.3f}px  fx={mtx_a[0,0]:.1f} fy={mtx_a[1,1]:.1f}")
    print(f"cam1 solo reprojection RMS error: {rms_b:.3f}px  fx={mtx_b[0,0]:.1f} fy={mtx_b[1,1]:.1f}")
    if rms_a > 1.0 or rms_b > 1.0:
        print("WARNING: reprojection error > 1.0px is high for a calibration this size — "
              "consider recollecting with more varied poses / better lighting / a flatter board.")

    stereo_rms, mtx_a, dist_a, mtx_b, dist_b, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_a, imgpoints_b, mtx_a, dist_a, mtx_b, dist_b, img_size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    baseline_m = float(np.linalg.norm(T))
    print(f"\nStereo reprojection RMS error: {stereo_rms:.3f}px")
    print(f"Calibrated baseline: {baseline_m*100:.2f}cm  (physically measured should be close to this — "
          "large disagreement means a bad capture set or the board size is wrong)")

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtx_a, dist_a, mtx_b, dist_b, img_size, R, T, alpha=0)
    map1_a, map2_a = cv2.initUndistortRectifyMap(mtx_a, dist_a, R1, P1, img_size, cv2.CV_16SC2)
    map1_b, map2_b = cv2.initUndistortRectifyMap(mtx_b, dist_b, R2, P2, img_size, cv2.CV_16SC2)

    np.savez(args.out,
             mtx_a=mtx_a, dist_a=dist_a, mtx_b=mtx_b, dist_b=dist_b, R=R, T=T, E=E, F=F,
             R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
             map1_a=map1_a, map2_a=map2_a, map1_b=map1_b, map2_b=map2_b,
             img_size=np.array(img_size), baseline_m=baseline_m,
             rectified_fx=P1[0, 0])
    print(f"\nSaved calibration to {args.out}")
    print(f"Rectified common focal length: {P1[0,0]:.1f}px  "
          f"(true physical depth = {baseline_m:.4f} * {P1[0,0]:.1f} / disparity_px_rectified)")
    print("Next: pass --calib-file tools/stereo_calib.npz to pedestrian_detect.py to use this "
          "instead of the empirical stereo-a/stereo-b formula.")


if __name__ == "__main__":
    main()
