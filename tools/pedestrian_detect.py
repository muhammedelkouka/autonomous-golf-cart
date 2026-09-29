#!/usr/bin/env python3
"""
Standalone camera-only pedestrian detection for the golf cart.

WIRED INTO THE C++ CONTROL STACK as of 2026-07-19: main.cpp's ped_set_enabled()
launches this exact script (see that function for the flags) via CreateProcessA,
and reads its --status-file output at 10Hz to drive control.cpp's top-priority
pedestrian pause (and auto-resume — see control.cpp's pedResumeConfirmCount).
File-IPC only, no other coupling — this process can still be run and tested fully
standalone (see tools/PEDESTRIAN_DETECTION.md for the full session history/findings).

Camera-only, no LiDAR fusion (per requirement).

Detector (2026-07-08): MobileNet-SSD via OpenCV's built-in cv2.dnn module.
Replaces the earlier HOG attempt, which only detects full-body standing
pedestrians from a distance and completely missed a close / seated / partial
person at a desk. MobileNet-SSD is a real neural detector: it finds people at
any range including close-up and partial-body, is far more reliable, and still
runs on CPU at a usable frame rate at 300x300 input. cv2.dnn is already inside
opencv-python; only the ~23MB model weights were downloaded (tools/models/).

Detector option (2026-07-10): --backend yolo uses YOLOv8n via ultralytics on
CUDA (this machine has a GTX 1650) — more accurate and much faster than
MobileNet-SSD/CPU, giving headroom for two cameras at higher resolution and
for validating the depth-distance numbers live. Requires torch+cu116 and
ultralytics (see Setup below); falls back to --backend mobilenet (the
default, zero extra installs) if those aren't present. yolov8n.pt weights
(~6MB) auto-download from ultralytics on first run.

Threading: each camera runs capture+detect in its OWN thread so a slow detect
pass on one never blocks the other or the display loop (cv2.dnn releases the
GIL during forward()). The display loop shows whichever annotated frame each
camera thread most recently finished.

Detection logic: for each detected person, take the bottom-center of the box
(their feet / ground contact — the single-camera distance heuristic, no stereo
depth here) and test whether it's inside the "stop zone" trapezoid in front of
the camera. Any person with feet in the zone => STOP.

Model files (already downloaded to tools/models/):
    MobileNetSSD_deploy.prototxt      (~29 KB)
    MobileNetSSD_deploy.caffemodel    (~23 MB)

Setup for --backend yolo (already done on this machine 2026-07-10):
    python -m pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 --index-url https://download.pytorch.org/whl/cu116
    python -m pip install ultralytics

Run:
    C:\\Users\\bios\\AppData\\Local\\Programs\\Python\\Python38\\python.exe tools/pedestrian_detect.py
    (--cam1 -1 to test with one camera; --conf 0.4 to tune sensitivity; --backend yolo for GPU)

Keys: 'q'/Esc quit, 's' toggle stop-zone overlay, 'z' cycle stop-zone presets,
      '0'-'9' mark current ground-truth distance in metres for --log-file calibration
      (0 clears the mark). Stand at a measured distance, press the matching digit,
      hold still a couple seconds, move to the next distance, press its digit, etc.

Hands-free calibration (2026-07-17): --auto-calib runs a beep-timed sequence so you
can be standing away from the PC/keyboard the whole time — no video-watching or key-
pressing needed. Sequence per distance in --calib-distances: a few beeps to count
down the walk time (--calib-prep seconds), a high "start" beep + it starts logging,
--calib-hold seconds later a low "stop/move on" beep. Three rising beeps at the end.
Run ONE camera at a time (pass -1 for the other) — cleaner ground truth than trying
to stand centred on two cameras 60cm apart at once, and calibrates each separately.
Example: --cam0 0 --cam1 -1 --auto-calib --calib-distances 1,2,3,5 --log-file calib_cam0.csv
"""

import argparse
import math
import os
import sys
import threading
import time
import winsound
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROTOTXT = os.path.join(HERE, "models", "MobileNetSSD_deploy.prototxt")
CAFFEMODEL = os.path.join(HERE, "models", "MobileNetSSD_deploy.caffemodel")

# MobileNet-SSD (VOC) class list — index 15 is "person", 9 is "chair".
VOC_CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle",
               "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
               "horse", "motorbike", "person", "pottedplant", "sheep", "sofa",
               "train", "tvmonitor"]
PERSON_IDX = 15
VOC_CHAIR_IDX = 9
# COCO (YOLO) class ids — 0 is "person", 56 is "chair".
COCO_PERSON_IDX = 0
COCO_CHAIR_IDX = 56

# ─── Stop-zone presets (normalized 0..1, y=1 = bottom/closest) ─────────────
# 'z' cycles these live so you can find one that matches the camera mount
# without editing the file. Corners: bottom-left, bottom-right, top-right, top-left.
STOP_ZONE_PRESETS = [
    # 0: wide trapezoid, most of the lower frame (good default for a forward cam)
    [(0.05, 1.00), (0.95, 1.00), (0.75, 0.35), (0.25, 0.35)],
    # 1: full frame (trigger on a person anywhere in view — good for first tests)
    [(0.00, 1.00), (1.00, 1.00), (1.00, 0.00), (0.00, 0.00)],
    # 2: narrow central lane (only stop for someone directly ahead). Bottom edge
    # widened 0.30-0.70 -> 0.10-0.90 on 2026-07-27, then -> FULL WIDTH (0.0-1.0) same
    # day after live testing still showed small uncovered triangles at the bottom
    # corners — a person standing close to the camera swings much farther across the
    # frame for the same real-world sideways position (perspective), so anything less
    # than the full width at the closest range can clip someone genuinely on the path.
    # Far edge (0.40-0.60 at y=0.45) left unchanged — confirmed fine at range.
    [(0.00, 1.00), (1.00, 1.00), (0.60, 0.45), (0.40, 0.45)],
]


def build_mobilenet_detector(args, conf, static_conf):
    """MobileNet-SSD via cv2.dnn (CPU). Returns detect(frame) ->
    [(x1,y1,x2,y2,conf,kind), ...] where kind is 'person' (dynamic) or, when
    --detect-static is passed, also 'chair' (static).

    conf/static_conf (2026-07-28) are passed in explicitly, resolved PER-CAMERA by the
    caller — see build_detector_for() — rather than read from args.conf/args.static_conf
    directly, so each camera can get a different threshold."""
    if not (os.path.exists(PROTOTXT) and os.path.exists(CAFFEMODEL)):
        print("Model files missing under tools/models/. Expected:\n"
              f"  {PROTOTXT}\n  {CAFFEMODEL}")
        sys.exit(1)
    net = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    tracked = {PERSON_IDX: "person"}
    conf_of = {PERSON_IDX: conf}
    if args.detect_static:
        tracked[VOC_CHAIR_IDX] = "chair"
        conf_of[VOC_CHAIR_IDX] = static_conf

    def detect(frame):
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        det = net.forward()
        out = []
        for i in range(det.shape[2]):
            conf = float(det[0, 0, i, 2])
            cls_idx = int(det[0, 0, i, 1])
            if cls_idx not in tracked or conf < conf_of[cls_idx]:
                continue
            x1 = max(0, min(int(round(float(det[0, 0, i, 3]) * w)), w - 1))
            y1 = max(0, min(int(round(float(det[0, 0, i, 4]) * h)), h - 1))
            x2 = max(0, min(int(round(float(det[0, 0, i, 5]) * w)), w - 1))
            y2 = max(0, min(int(round(float(det[0, 0, i, 6]) * h)), h - 1))
            out.append((x1, y1, x2, y2, conf, tracked[cls_idx]))
        return out

    return detect


def build_yolo_detector(args, conf, static_conf):
    """YOLOv8n via ultralytics, CUDA if available. Same return shape as above:
    [(x1,y1,x2,y2,conf,kind), ...], kind 'person' or (with --detect-static) 'chair'.

    conf/static_conf (2026-07-28) — see build_mobilenet_detector's docstring, same
    per-camera resolution."""
    try:
        import torch
        from ultralytics import YOLO
    except ImportError:
        print("--backend yolo requires torch+ultralytics. Install with:\n"
              "  python -m pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 "
              "--index-url https://download.pytorch.org/whl/cu116\n"
              "  python -m pip install ultralytics\n"
              "Falling back to --backend mobilenet instead, or install the above.")
        sys.exit(1)
    device = "cpu" if args.device == "cpu" else (0 if torch.cuda.is_available() else "cpu")
    model = YOLO("yolov8n.pt")
    dev_name = torch.cuda.get_device_name(0) if device == 0 else "CPU"
    print(f"YOLOv8n loaded, running on: {dev_name}")
    class_ids = [COCO_PERSON_IDX] + ([COCO_CHAIR_IDX] if args.detect_static else [])
    kind_of = {COCO_PERSON_IDX: "person", COCO_CHAIR_IDX: "chair"}
    conf_of = {COCO_PERSON_IDX: conf, COCO_CHAIR_IDX: static_conf}
    # ultralytics predict() only takes one global conf floor — pass the lower of the
    # two so nothing valid gets dropped at inference, then apply the real per-class
    # threshold ourselves below.
    min_conf = min(conf_of[c] for c in class_ids)

    def detect(frame):
        results = model.predict(frame, classes=class_ids, conf=min_conf, iou=args.nms_iou,
                                 device=device, verbose=False)[0]
        out = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if conf < conf_of[cls_id]:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append((int(x1), int(y1), int(x2), int(y2), conf, kind_of[cls_id]))
        return out

    return detect


def merge_duplicate_boxes(dets, min_area_frac=0.5):
    """Collapse same-kind boxes that are really one object seen as two candidate
    boxes (e.g. 'whole chair' + 'just the backrest') — a failure mode standard IoU-
    based NMS can miss entirely. IoU divides by the UNION area, so it stays low
    whenever one box is much bigger than the other, no matter how much the smaller
    box sits inside the bigger one. This instead measures overlap as a fraction of
    the SMALLER box's own area — catches nested/partial duplicates regardless of
    size mismatch. Applied backend-agnostically (both YOLO and MobileNet-SSD can
    produce this pattern), keeping the highest-confidence box of each cluster."""
    def area(b):
        return max(0, b[2] - b[0]) * max(0, b[3] - b[1])

    def overlap_area(b1, b2):
        ix1, iy1 = max(b1[0], b2[0]), max(b1[1], b2[1])
        ix2, iy2 = min(b1[2], b2[2]), min(b1[3], b2[3])
        return max(0, ix2 - ix1) * max(0, iy2 - iy1)

    kept = []
    for d in sorted(dets, key=lambda b: b[4], reverse=True):  # highest confidence first
        is_dup = False
        for k in kept:
            if d[5] != k[5]:  # different kind (person vs chair) — never merge across
                continue
            smaller = min(area(d), area(k))
            if smaller > 0 and overlap_area(d, k) / smaller > min_area_frac:
                is_dup = True
                break
        if not is_dup:
            kept.append(d)
    return kept


def log_calib_row(args, shared, label, box_h_px, dist_m, conf):
    """Append one calibration sample. Locked — cam0/cam1 threads write concurrently."""
    if not args.log_file or shared["marked_distance"] <= 0:
        return
    with shared["log_lock"]:
        with open(args.log_file, "a") as f:
            f.write(f"{time.time():.3f},{label},{shared['marked_distance']:.2f},"
                    f"{box_h_px},{dist_m:.3f},{conf:.3f}\n")


def run_stereo_logger(args, shared, worker_a, worker_b):
    """Cross-camera disparity logger for stereo depth calibration. Every ~100ms, takes
    the highest-confidence person box from each camera and logs the horizontal pixel
    disparity between them against the ground-truth distance (shared['marked_distance'],
    same marker used by the monocular calibration). Unlike the monocular box-height
    method, this needs NO assumed person height — depth = baseline_m * f_px / disparity_px,
    with f_px solved offline from these logged (distance, disparity) pairs. Requires the
    two cameras to be level and roughly parallel (confirmed for this rig 2026-07-17)."""
    while worker_a.running and worker_b.running:
        time.sleep(0.1)
        if shared["marked_distance"] <= 0 or not args.stereo_log_file:
            continue
        boxes_a = [b for b in worker_a.get_latest_boxes() if b[5] == "person"]
        boxes_b = [b for b in worker_b.get_latest_boxes() if b[5] == "person"]
        if not boxes_a or not boxes_b:
            continue
        # highest-confidence person box per camera — valid for a single test subject;
        # multi-person correspondence would need a matching step, not needed for calibration.
        # Static objects (chairs) are excluded — this calibration is pedestrian-specific.
        ba = max(boxes_a, key=lambda b: b[4])
        bb = max(boxes_b, key=lambda b: b[4])
        xa = (ba[0] + ba[2]) / 2.0
        xb = (bb[0] + bb[2]) / 2.0
        disparity_px = abs(xa - xb)
        with shared["log_lock"]:
            with open(args.stereo_log_file, "a") as f:
                f.write(f"{time.time():.3f},{shared['marked_distance']:.2f},"
                        f"{disparity_px:.2f},{ba[4]:.3f},{bb[4]:.3f}\n")


def run_fps_monitor(active_workers, duration_s):
    """Unconditional throughput print, independent of whether a person is in frame —
    self.fps already measures the real sustained capture+detect pipeline rate (with
    --detect-every 1, every captured frame is detected, so this is the true cost of
    running the detector, not just camera capture speed). No positioning/timing
    dependency, unlike the marked-distance calibration logging."""
    t_end = time.time() + duration_s
    while time.time() < t_end:
        time.sleep(1.0)
        parts = [f"{w.label}={w.fps:.1f}fps" for w in active_workers]
        print(f"[fps-monitor] {'  '.join(parts)}")
    print("[fps-monitor] done.")


class DistanceFilter:
    """Rolling-median filter over the last few valid stereo distance readings —
    rejects single-frame outliers (e.g. a transient occlusion-shifted box) without
    the lag a moving average would add, since one spike can't drag a median the way
    it drags a mean. Confirmed necessary 2026-07-17: a single frame where a person's
    head touched a chair's box in only one camera swung the reported distance from
    ~2.1m to 4.8m — a real, disparity-corrupting occlusion artifact, not sensor
    noise, but the fix is the same either way. Clears itself after too many
    consecutive misses so a genuinely-gone object doesn't keep reporting stale data.

    min_samples guards a second failure mode found 2026-07-17: a BRAND-NEW track's
    first reading is a median of one value — itself — so it passes through completely
    unfiltered. When a person tracker re-spawns a new track (lost/reacquired mid-test),
    that first raw reading could be a bad single-frame cross-camera mismatch reported
    with zero smoothing. Withholding output until the track has >= min_samples means a
    new track's opening reading is silent for a couple frames rather than possibly wrong."""
    def __init__(self, window=5, max_misses=8, min_samples=2):
        self.values = deque(maxlen=window)
        self.max_misses = max_misses
        self.min_samples = min_samples
        self.misses = 0

    def update(self, dist_m):
        if dist_m is None:
            self.misses += 1
            if self.misses > self.max_misses:
                self.values.clear()
            return self.median()
        self.misses = 0
        self.values.append(dist_m)
        return self.median()

    def median(self):
        if len(self.values) < self.min_samples:
            return None
        s = sorted(self.values)
        n = len(s)
        mid = n // 2
        return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def stereo_match_all(args, boxes_a, boxes_b, kind, width_a=None, width_b=None):
    """Match ALL `kind` boxes between the two cameras by left-to-right x-position RANK
    — NOT by independent per-camera confidence. With more than one object of the same
    kind in frame, picking 'highest confidence in each camera separately' can silently
    pair up two DIFFERENT physical objects (e.g. two different people), producing a
    meaningless disparity/distance — confirmed 2026-07-17 with two people in frame.
    Rank-by-x-position is far more stable across two closely-spaced forward-facing
    cameras: confidence varies with per-camera lighting/angle, but left-right ORDER
    essentially doesn't (barring people crossing paths at very close range, an
    accepted edge case for now). Returns [(dist_m, ba, bb), ...] sorted closest-first
    — for a safety decision the closest valid match should always be the one used.
    Same stereo_a/stereo_b calibration reused for every kind (see module notes) — the
    dominant physics is camera geometry, not object-specific, so it generalizes.

    EDGE-CLIP FIX (2026-07-27): a box touching its frame's left/right edge means the
    true extent is cut off — the person is close enough that part of them is out of
    frame. Plain box-CENTER disparity silently breaks here: both cameras' boxes clamp
    toward their own edge, so center-to-center disparity shrinks toward whatever the
    edge geometry happens to be, NOT the person's true position — confirmed live
    2026-07-27, a person <1m away (filling most of the frame) read as farther than they
    actually were. Uncorrected this fails in the DANGEROUS direction (close reads as
    far, could suppress STOP/CAUTION).

    Real fix, not just a safety clamp: disparity math is valid for ANY consistent
    physical point seen by both cameras, not specifically the box center — stereo_a/
    stereo_b encode camera geometry (baseline, focal length, principal-point offset),
    not anything about "center vs edge". So if one side of the box is clipped but the
    OTHER side is a real, unclipped edge of the person in BOTH cameras, that edge is
    used as the disparity reference instead of the center — still the same calibrated
    formula, just anchored to whichever boundary is actually trustworthy. Falls back to
    a monocular box-HEIGHT distance estimate (varies with how large the person appears,
    unlike a fixed constant) only in the genuinely ambiguous residual case: a box
    clipped on BOTH sides in one camera (spans the whole frame — no unclipped edge
    exists at all), or clipped on OPPOSITE sides between the two cameras (no single
    shared reference point). That fallback is itself capped at args.clip_safe_dist_m as
    a ceiling — clipping only happens at genuinely close range by definition, so even a
    rough estimate should not be trusted past a conservative "close" bound; "uncertain"
    must always resolve to "close", never "far"."""
    CLIP_MARGIN_PX = 2

    def clip_state(b, w):
        if w is None:
            return (False, False)
        return (b[0] <= CLIP_MARGIN_PX, b[2] >= w - 1 - CLIP_MARGIN_PX)

    def mono_fallback_dist(b, w):
        # Monocular box-height distance, used ONLY as a last-resort estimate when no
        # valid stereo disparity reference exists at all — capped at clip_safe_dist_m
        # since anything reaching this branch is, by construction, close enough to be
        # clipping the frame.
        if w is None:
            return args.clip_safe_dist_m
        f_px = (w * 0.5) / math.tan(math.radians(args.hfov) * 0.5)
        box_h = max(1, b[3] - b[1])
        est = f_px * args.person_height / box_h
        return min(est, args.clip_safe_dist_m)

    # ── APPARENT-SIZE CROSS-CHECK (2026-07-27 campus bug) ──────────────────────
    # Catches wrong cross-camera pairings that every other guard misses. Campus test
    # showed three people visible in BOTH cameras (people:3 == people:3, so the
    # earlier "unequal counts" theory does NOT explain it) with one clearly-distant
    # person (small box) reported at 1.4m and triggering a false STOP —
    # --template-stereo-people was already active and did not save it (a ~20x50px
    # patch of a distant person can find a false NCC peak above --template-min-score
    # among repetitive corridor structure and similar-looking people).
    #
    # Root cause: equal COUNTS does not mean the same SET of people. The 40cm
    # baseline gives the two cameras slightly different fields of view, so cam0 can
    # see people {A,B,C} while cam1 sees {B,C,D} — A outside cam1's view, D outside
    # cam0's. Rank-by-x then pairs A<->B, B<->C, C<->D: every pair wrong, counts
    # matching perfectly, disparity computed between two DIFFERENT bodies.
    #
    # The check: a person's apparent SIZE gives their distance independently of any
    # pairing at all. f_px comes from the stereo calibration itself — distance =
    # baseline*f_px/disparity is the same equation as stereo_a/(disparity-stereo_b),
    # so f_px = stereo_a/baseline_m (631px for today's validated 40cm fit, which
    # back-solves to a ~54deg HFOV — a sane webcam value, and notably NOT dependent
    # on the uncalibrated --hfov guess that was found "totally wrong" earlier).
    #
    # ASYMMETRIC ON PURPOSE — this is a safety device, so it may only ever reject in
    # the false-ALARM direction. Rejecting whenever the two disagree would be
    # dangerous: a crouching or partly-occluded person at close range has a SHORT box,
    # so size-distance reads far while stereo correctly reads close, and rejecting
    # that would suppress a legitimate STOP. So we reject ONLY when stereo claims
    # much CLOSER than the apparent size can support (the false-STOP case). The
    # opposite disagreement is left alone. A genuinely close person who somehow trips
    # this still has the single-camera near-field trigger as an independent backstop,
    # which uses no distance or pairing at all.
    def size_implausible(dist_m, ba, bb, real_height_m):
        if args.size_check_ratio <= 0:
            return False
        f_px = args.stereo_a / max(1e-6, args.baseline_m)
        # Taller of the two boxes = the less-occluded/clipped view of that object, so
        # it gives the least-biased size estimate (a shorter box only ever pushes the
        # estimate farther away, i.e. toward rejecting — use the safer one).
        box_h = max(1, ba[3] - ba[1], bb[3] - bb[1])
        mono_dist = f_px * real_height_m / box_h
        # Only the "stereo says closer than size allows" direction rejects.
        return dist_m * args.size_check_ratio < mono_dist

    ka = sorted((b for b in boxes_a if b[5] == kind), key=lambda b: (b[0] + b[2]) / 2.0)
    kb = sorted((b for b in boxes_b if b[5] == kind), key=lambda b: (b[0] + b[2]) / 2.0)
    matches = []
    for ba, bb in zip(ka, kb):
        la, ra = clip_state(ba, width_a)
        lb, rb = clip_state(bb, width_b)
        if not (la or ra or lb or rb):
            xa, xb = (ba[0] + ba[2]) / 2.0, (bb[0] + bb[2]) / 2.0
        elif (la or lb) and not (ra or rb):
            xa, xb = ba[2], bb[2]  # left clipped somewhere -> both RIGHT edges still valid
        elif (ra or rb) and not (la or lb):
            xa, xb = ba[0], bb[0]  # right clipped somewhere -> both LEFT edges still valid
        else:
            # No shared unclipped reference (clipped both sides in one camera, or
            # opposite sides between the two) — fall back to whichever camera's box is
            # less clipped (taller/more of it visible) for the height estimate.
            ha, hb = ba[3] - ba[1], bb[3] - bb[1]
            dist_m = mono_fallback_dist(ba, width_a) if ha >= hb else mono_fallback_dist(bb, width_b)
            matches.append((dist_m, ba, bb))
            continue
        disparity_px = abs(xa - xb)
        if disparity_px <= args.stereo_b:
            continue
        dist_m = args.stereo_a / (disparity_px - args.stereo_b)
        # distance_m = A/(disparity-B) is numerically unstable as disparity approaches
        # B — a tiny disparity error (bad rank match during movement, marginal
        # detection) explodes into a physically-impossible reading (confirmed
        # 2026-07-19: 74m/88m spikes in a 2.8m room). Reject outright rather than
        # cap/clamp — treated the same as no-match, never fed to the median filter,
        # so it can't even partially bias a track.
        if dist_m > args.stereo_max_dist:
            continue
        # Apparent-size cross-check — rejects a cross-paired match whose disparity
        # claims an object is much closer than its on-screen size can support.
        # See size_implausible() above for why this is deliberately one-directional.
        # Applied to chairs too (2026-07-28): the same cross-pairing failure was
        # reported live with TWO chairs in frame (a 3.1m reading the operator judged
        # wrong). Chair height is a rougher assumption than person height (real chairs
        # vary ~0.8-1.1m, and the detected box may or may not include the backrest),
        # but the check only ever fires on gross disagreement at the 2.5x ratio, so a
        # loose height assumption is still enough to catch a cross-pair while leaving
        # honest readings alone.
        exp_h = args.person_height if kind == "person" else args.static_height
        if size_implausible(dist_m, ba, bb, exp_h):
            continue
        matches.append((dist_m, ba, bb))
    matches.sort(key=lambda m: m[0])
    return matches


class PersonTracker:
    """Keeps each matched person's identity across frames (greedy nearest cam-A box
    center) so each person can be median-filtered INDEPENDENTLY — a single-frame
    occlusion spike on one person can't swing anyone's reading, and it works with any
    number of people at once. Tracks unseen for a few frames are dropped.

    COAST-THROUGH (2026-07-27): a missed cycle (no fresh stereo match — e.g. the
    frame-skew guard rejected this poll, or a single-frame detection dropout in one
    camera) used to just vanish from the output entirely, even though the track
    itself survived internally (misses < max_misses). That silently flipped the
    live STOP/CLEAR decision to CLEAR for that instant whenever a real in-lane
    person happened to be missed for even one 100ms poll — reported as "no stereo
    match toggles too much", made worse by the frame-skew guard rejecting more
    polls than before. Now a missed-but-not-yet-dropped track still outputs its
    last known smoothed distance + box position, so brief gaps coast through
    smoothly instead of flickering; the person only actually disappears (CLEAR)
    after max_misses consecutive real misses (~0.6s at the 10Hz poll rate this is
    used at), same timeout as before — this only fixes the flicker, not how long
    it takes to notice someone is genuinely gone."""
    def __init__(self, max_move_px=100, max_misses=6, window=5):
        self.tracks = {}          # id -> {"cx","cy","filt","misses","payload"}
        self.next_id = 0
        self.max_move2 = max_move_px ** 2
        self.max_misses = max_misses
        self.window = window

    def update(self, dets):
        """dets: [(raw_dist, cx, cy, payload), ...]. Returns [(smoothed_dist, payload), ...]."""
        assigned = {}
        for raw, cx, cy, payload in dets:
            best_id, best_d2 = None, self.max_move2
            for tid, tr in self.tracks.items():
                if tid in assigned:
                    continue
                d2 = (tr["cx"] - cx) ** 2 + (tr["cy"] - cy) ** 2
                if d2 < best_d2:
                    best_d2, best_id = d2, tid
            if best_id is None:
                best_id = self.next_id
                self.next_id += 1
                self.tracks[best_id] = {"cx": cx, "cy": cy, "payload": payload,
                                        "filt": DistanceFilter(self.window, self.max_misses),
                                        "misses": 0}
            assigned[best_id] = (raw, cx, cy, payload)
        out = []
        for tid in list(self.tracks.keys()):
            tr = self.tracks[tid]
            if tid in assigned:
                raw, cx, cy, payload = assigned[tid]
                tr["cx"], tr["cy"], tr["misses"], tr["payload"] = cx, cy, 0, payload
                sd = tr["filt"].update(raw)
                if sd is not None:
                    out.append((sd, payload))
            else:
                tr["misses"] += 1
                if tr["misses"] > self.max_misses:
                    del self.tracks[tid]
                else:
                    sd = tr["filt"].median()  # coast: last known smoothed value, no new sample
                    if sd is not None:
                        out.append((sd, tr["payload"]))
        return out


def template_match_disparity(args, frame_a, frame_b, box_a):
    """True stereo disparity by matching the object's actual PIXELS between the two
    cameras, instead of subtracting bounding-box centers. Box-center disparity fails
    for wide/asymmetric objects (e.g. a chair seen at an angle): each camera frames a
    different extent, so the centers don't track the same physical point — confirmed
    2026-07-17, a chair at a tape-measured 2.1m read 4.7m because box-center disparity
    (~56px) was ~half the true value (~119px).

    Cameras are level & parallel (this rig), so corresponding points lie on the SAME
    image row — we take the object's patch from frame_a and slide it horizontally
    across the identical rows of frame_b (cv2.matchTemplate, normalized cross-corr),
    the offset of best match IS the true disparity. Same stereo_a/stereo_b calibration
    then applies, because that formula is really baseline*focal geometry (object-
    independent). Returns (disparity_px, match_score) or None if no confident match.

    Caveat: at close range with a 60cm baseline the two views differ by ~16deg, so the
    patch isn't identical between cameras — NCC tolerates small differences but a
    low match_score (< --template-min-score) is rejected rather than trusted."""
    if frame_a is None or frame_b is None:
        return None
    h, w = frame_a.shape[:2]
    x1, y1, x2, y2 = (int(box_a[0]), int(box_a[1]), int(box_a[2]), int(box_a[3]))
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 - x1 < 12 or y2 - y1 < 12:
        return None
    template = frame_a[y1:y2, x1:x2]
    strip = frame_b[y1:y2, :]  # same rows, full width — cameras are row-aligned
    if strip.shape[0] != template.shape[0] or strip.shape[1] < template.shape[1]:
        return None
    res = cv2.matchTemplate(strip, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    if max_val < args.template_min_score:
        return None
    center_a = (x1 + x2) / 2.0
    center_b = max_loc[0] + (x2 - x1) / 2.0
    return abs(center_a - center_b), max_val


def run_stereo_live(args, shared, worker_a, worker_b):
    """Prints a real-time calibrated STOP/CLEAR decision + distance (~2/sec) so accuracy
    can be validated by console without watching the video window. Distance model:
    distance_m = stereo_a / (disparity_px - stereo_b) — a 2-parameter fit (inverse
    relationship plus a constant disparity offset) empirically calibrated 2026-07-17
    against 1.0/2.0/2.8m ground truth, residuals < 4cm across that range. Extrapolation
    beyond ~2.8m or below 1.0m is unverified.

    This is the AUTHORITATIVE distance/decision — separate from (and more accurate than)
    each camera's own on-screen banner, which still uses the old uncalibrated per-camera
    monocular formula. Also naturally immune to single-camera false positives (a mirror
    reflection, poster, or screen showing a person) since a stereo match requires
    agreement from BOTH independent camera views — confirmed 2026-07-17 when a mirror
    reflection visible only in cam1 correctly produced no stereo reading at all.

    Also matches static objects (chairs, with --detect-static) the same way — distance
    matters for a static obstacle too, just no STOP/zone decision is computed for it yet
    (visual distance readout only; not wired into any avoidance logic).

    Drives the on-screen box color/label (shared['stereo_dist_m' / 'stereo_static_dist_m']
    etc., read by CameraWorker.run()) at the full 10Hz update rate — decoupled from the
    2Hz console print so the video overlay stays smooth.

    The DISTANCE reported/decided-on is median-filtered (DistanceFilter, ~0.5s window
    at 10Hz) — a single bad frame (occlusion, misdetection) can't swing it. Box
    highlighting still tracks the raw current-frame position so the outline stays
    visually responsive; only the number and the STOP decision are smoothed."""
    def resolve_dist(box_center_dist, ba, use_template):
        """Distance for one matched pair. When use_template, recompute disparity by
        pixel-matching box `ba`'s patch across the ENTIRE width of the other camera's
        raw frame (template_match_disparity doesn't use the paired box `bb` at all) —
        this independently REDISCOVERS the true correspondence for this specific
        person, regardless of whatever stereo_match_all's rank-based pairing guessed.

        2026-07-27 campus bug fix: template-match failure now REJECTS the pair
        (returns None) instead of falling back to box_center_dist. Found live: when
        the two cameras detect a DIFFERENT NUMBER of people (cam0 saw 1, cam1 saw 3),
        stereo_match_all's left-to-right rank pairing (zip()) can pair the WRONG two
        people together — e.g. cam0's one visible person paired with cam1's leftmost
        of three unrelated people — producing a nonsense box-center disparity between
        two different physical people that can compute as falsely CLOSE even though
        every individual box is visibly small/far. The old fall-back-to-box-center-on-
        template-failure let exactly that wrong pairing still produce a confidently-
        wrong number and trigger STOP. Since template matching is the only thing here
        that can actually tell two different people apart, a failed/low-score match
        means the PAIR itself is untrustworthy, not just its disparity refinement —
        same 'a skipped reading beats a confidently-wrong one' philosophy already used
        for the frame-skew guard and --stereo-max-dist rejection."""
        if not use_template:
            return box_center_dist, "box"
        fa, fb = worker_a.get_latest_raw(), worker_b.get_latest_raw()
        tm = template_match_disparity(args, fa, fb, ba)
        if tm is None:
            return None, "reject(tpl-miss)"
        disparity, score = tm
        if disparity <= args.stereo_b:
            return None, "reject(tpl-range)"
        return args.stereo_a / (disparity - args.stereo_b), f"tpl{score:.2f}"

    last_print = 0.0
    person_tracker = PersonTracker()
    static_filter = DistanceFilter()
    while worker_a.running and worker_b.running:
        time.sleep(0.1)
        boxes_a = worker_a.get_latest_boxes()
        boxes_b = worker_b.get_latest_boxes()
        # Raw, pre-skew-guard copies for the near-field check below — that check is
        # SINGLE-camera (no cross-camera correspondence needed at all), so the frame-
        # skew guard's reasoning (protects DISPARITY, which needs simultaneous frames)
        # doesn't apply to it and it must not be starved by the same guard.
        raw_boxes_a, raw_boxes_b = boxes_a, boxes_b
        now = time.time()
        do_print = (now - last_print) >= 0.5

        # ── Frame-skew guard (2026-07-27) ── cam0/cam1 run at different native FPS
        # (observed 13 vs 24 FPS — cameras don't honor cap.set(CAP_PROP_FPS), no sync
        # between them) so "latest boxes" from each camera can be tens of ms apart in
        # actual capture time. While everything is stationary that's harmless (the
        # scene hasn't changed between those two moments); found 2026-07-27 that once
        # the CART itself is moving, the whole scene shifts between cam0's and cam1's
        # capture instants, corrupting the disparity geometry — reported as "distance
        # sometimes wrong for a while" specifically while driving. Rather than trying
        # to force-sync two independent, unsynced USB cameras (not achievable in
        # software with this hardware), reject the pair outright when they're too far
        # apart in time — same fail-safe philosophy as the existing --stereo-max-dist
        # outlier rejection: a skipped reading (falls back to CLEAR/no-match) is far
        # safer than a confidently-wrong one.
        skew_s = abs(worker_a.get_latest_boxes_ts() - worker_b.get_latest_boxes_ts())
        if skew_s > (args.max_frame_skew_ms / 1000.0):
            boxes_a, boxes_b = [], []

        # Real per-camera frame size — width for the edge-clip safety check in
        # stereo_match_all (see its docstring), height for the near-field check below;
        # fetched once here, before matching.
        fw_a, fh_a = worker_a.get_frame_size()
        fw_b, fh_b = worker_b.get_frame_size()

        # ── NEAR-FIELD trigger (2026-07-27) — closes a real gap: STOP/CAUTION above
        # only ever fire from a matched PAIR across both cameras. But someone close
        # enough can fall completely out of ONE camera's field of view (the two lenses
        # are offset by the baseline), so no cross-camera match exists at all — the
        # system would report "CLEAR - no matching person" for someone dangerously
        # close. This check needs only ONE camera: if a person's feet (box bottom) are
        # in the bottom --near-field-y fraction of EITHER camera's frame, that alone is
        # close enough to force STOP — no distance calculation, no stereo match, no
        # cross-camera agreement required. Ignores lateral (x) position deliberately:
        # the lane zone's own bottom edge is already full-width (0.0-1.0), so anything
        # this close to the bottom edge is already "in the path" by the existing zone
        # shape anyway.
        def max_foot_frac(boxes, fh):
            if not fh:
                return None
            fracs = [b[3] / fh for b in boxes if b[5] == "person"]
            return max(fracs) if fracs else None

        # 2026-07-27 TUNING: the Y-fraction-to-real-distance mapping depends entirely on
        # this camera's mount height/tilt, which the 0.85 default did NOT account for —
        # found live to trigger on essentially any distance the person could stand at.
        # foot_frac_a/b (the RAW measured value) is surfaced in the console print below
        # so the right --near-field-y can be picked from real numbers instead of guessed.
        foot_frac_a = max_foot_frac(raw_boxes_a, fh_a)
        foot_frac_b = max_foot_frac(raw_boxes_b, fh_b)
        near_field_trigger = (foot_frac_a is not None and foot_frac_a >= args.near_field_y_a) or \
                              (foot_frac_b is not None and foot_frac_b >= args.near_field_y_b)

        # ── MULTI-person: match every person across both cameras, distance for each ──
        # By x-position RANK (leftmost-with-leftmost etc.) — proven reliable for this
        # camera layout; height/y-position was tried and REJECTED 2026-07-19: it can't
        # tell apart two similarly-tall, similarly-positioned people and cross-matched
        # them, giving each camera opposite (wrong) distances for the same two people.
        person_matches = stereo_match_all(args, boxes_a, boxes_b, "person", fw_a, fw_b)
        dets = []
        for box_dist, ba, bb in person_matches:
            d, _ = resolve_dist(box_dist, ba, args.template_stereo_people)
            if d is None:
                continue  # rejected pair (2026-07-27) — don't feed a bad match into the tracker
            cx, cy = (ba[0] + ba[2]) / 2.0, (ba[1] + ba[3]) / 2.0
            dets.append((d, cx, cy, (tuple(ba[:4]), tuple(bb[:4]))))
        # per-person identity + independent median filtering (noise/occlusion robust)
        tracked = person_tracker.update(dets)
        zone = STOP_ZONE_PRESETS[shared["zone_idx"]]
        # Normalize against camera A's ACTUAL captured frame size, not args.width/height —
        # cap.set(CAP_PROP_FRAME_WIDTH/HEIGHT) is only a request; if the camera doesn't
        # support that exact mode, cv2 silently falls back to its own native resolution.
        # Dividing by the wrong (requested-but-not-actual) size put every foot_x/foot_y
        # outside 0..1, so point_in_poly() was FALSE for every person regardless of their
        # real position — found 2026-07-27 when --zone 2 stopped triggering STOP at all,
        # even for someone standing dead-center at 0.4m. ba4 is always camera A's box (the
        # match loop below iterates camera-A-anchored pairs), so camera A's own frame size
        # is the correct one to use here.
        fw, fh = worker_a.get_frame_size()
        fw, fh = fw or args.width, fh or args.height  # fall back before the first frame lands
        people = []
        for sdist, (ba4, bb4) in tracked:
            foot_x, foot_y = (ba4[0] + ba4[2]) / 2.0, ba4[3]
            # --no-zone: ignore lateral position, distance alone decides
            in_zone = True if args.no_zone else point_in_poly(foot_x / fw, foot_y / fh, zone)
            people.append({"dist": sdist, "box_a": ba4, "box_b": bb4, "in_zone": in_zone})
        people.sort(key=lambda p: p["dist"])
        # STOP if ANY person is both in-lane and within stop distance (nearest such one),
        # OR the near-field trigger fired (single-camera, no distance/match needed at
        # all — see above). CAUTION (2026-07-27): ANY in-lane person at all, regardless
        # of distance — the cart's brake is a plain throttle-cutoff + coast (confirmed
        # no strong active deceleration), so it needs real lead distance to slow down
        # BEFORE it's close enough to require a full stop. See PED_CAUTION_CRUISE_VOLTAGE
        # in control.cpp.
        stop = near_field_trigger or any(p["in_zone"] and p["dist"] <= args.stop_distance for p in people)
        caution = near_field_trigger or any(p["in_zone"] for p in people)
        shared["stereo_people"] = people
        shared["stereo_ts"] = now
        shared["stereo_stop"] = stop
        shared["stereo_caution"] = caution
        shared["stereo_near_field"] = near_field_trigger
        shared["stereo_dist_m"] = people[0]["dist"] if people else None
        # TUNING readout (2026-07-27): raw foot-position fraction per camera (None if no
        # person in that camera), always shown when either camera sees anyone — this is
        # the number to correlate against a tape-measured distance to pick the right
        # --near-field-y, not the 0.85 guess.
        frac_str = ""
        if foot_frac_a is not None or foot_frac_b is not None:
            fa = f"{foot_frac_a:.2f}" if foot_frac_a is not None else "--"
            fb = f"{foot_frac_b:.2f}" if foot_frac_b is not None else "--"
            frac_str = f"  [foot-y a={fa} (t={args.near_field_y_a:.2f}) b={fb} (t={args.near_field_y_b:.2f})]"

        if near_field_trigger:
            person_msg = f"STOP  NEAR-FIELD (person very close, single-camera trigger)  " \
                         f"[{len(people)} matched person(s) also]{frac_str}"
        elif not people:
            person_msg = f"CLEAR — no matching person in both cameras{frac_str}"
        else:
            tag = "STOP" if stop else "CLEAR"
            dists = ", ".join((f"{p['dist']:.2f}m*" if p["in_zone"] else "out-of-lane") for p in people)
            person_msg = f"{tag}  {len(people)} person(s): {dists}  (*=in-lane){frac_str}"

        static_msg = ""
        if args.detect_static:
            static_matches = stereo_match_all(args, boxes_a, boxes_b, "chair", fw_a, fw_b)
            if static_matches:
                s_box_dist, s_ba, s_bb = static_matches[0]
                raw_static, s_method = resolve_dist(s_box_dist, s_ba, args.template_stereo)
                shared["stereo_static_box_a"] = (s_ba[0], s_ba[1], s_ba[2], s_ba[3])
                shared["stereo_static_box_b"] = (s_bb[0], s_bb[1], s_bb[2], s_bb[3])
            else:
                raw_static, s_method = None, ""
            s_dist_m = static_filter.update(raw_static)
            if s_dist_m is None:
                shared["stereo_static_dist_m"] = None
            else:
                shared["stereo_static_dist_m"] = s_dist_m
                shared["stereo_static_ts"] = now
                static_msg = f"  |  chair distance={s_dist_m:.2f}m  ({s_method}, raw={raw_static})"

            # ── Static-obstacle OBSERVATIONS: distance + BEARING (2026-07-28) ──────
            # Groundwork for auto-placing detected static obstacles into the C++
            # obstacle grid. Distance alone is not enough to place a world point — we
            # also need WHERE ACROSS the field of view the object sits. Bearing is
            # computed from the same calibrated focal length the size-check uses
            # (f_px = stereo_a/baseline_m, ~631px), NOT the uncalibrated --hfov.
            #
            # SIGN CONVENTION (must be verified physically before the C++ side trusts
            # it — see PEDESTRIAN_DETECTION.md): POSITIVE bearing = object appears to
            # the RIGHT of camera A's centre. This is published and printed precisely
            # so it can be checked against a chair placed at a known left/right offset
            # BEFORE any world-frame transform is written, rather than discovering a
            # sign error later through wrong grid placements. This project has a
            # documented history of exactly that class of bug (three separate
            # CCW-yaw sign-fix sessions), so the sign gets tested, not assumed.
            #
            # ALL matches are published (not just the nearest) and deliberately
            # UNFILTERED per-object: multi-object median filtering would duplicate the
            # PersonTracker machinery, and the consumer needs its own multi-frame
            # confirmation anyway (a falsely-placed STATIC cell is permanent and can
            # block the corridor, so it must never be committed from one frame).
            f_px_static = args.stereo_a / max(1e-6, args.baseline_m)
            obs = []
            for s_dist, s_ba, s_bb in static_matches:
                d, _ = resolve_dist(s_dist, s_ba, args.template_stereo)
                if d is None:
                    continue
                cx = (s_ba[0] + s_ba[2]) / 2.0
                bearing = math.atan2(cx - (fw_a or args.width) * 0.5, f_px_static)
                obs.append((d, bearing, s_ba[4]))
            shared["static_obs"] = obs
            if obs:
                static_msg += "  | obs: " + ", ".join(
                    f"{d:.2f}m@{math.degrees(b):+.0f}deg" for d, b, _ in obs)

        if do_print:
            last_print = now
            print(f"[stereo-live] {person_msg}{static_msg}")


def nearest_box_idx(boxes, idxs, target):
    """Index (from idxs) whose box center is closest to target's center. Used to find
    which of THIS camera's own boxes corresponds to a cross-camera stereo match stored
    by coordinate — safer than re-picking 'highest confidence' locally, which can name
    a different box than the one actually used for the match when multiple objects of
    the same kind are in frame."""
    if not idxs or target is None:
        return None
    tx, ty = (target[0] + target[2]) / 2.0, (target[1] + target[3]) / 2.0

    def dist2(i):
        b = boxes[i]
        bx, by = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
        return (bx - tx) ** 2 + (by - ty) ** 2

    return min(idxs, key=dist2)


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def draw_stop_zone(frame, poly, color):
    h, w = frame.shape[:2]
    pts = np.array([[int(px * w), int(py * h)] for px, py in poly], dtype=np.int32)
    overlay = frame.copy()
    cv2.fillPoly(overlay, [pts], color)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)


def draw_near_field_zone(frame, poly, near_field_y, color):
    """Bottom slice of the lane-zone polygon, from near_field_y down to the bottom edge
    — visualizes the single-camera near-field trigger zone (see run_stereo_live).
    Clipped to the ACTUAL zone shape (2026-07-27 fix) rather than drawn as a full-width
    rectangle: the zone trapezoid narrows going up (e.g. preset 2 is already down to
    ~11%-89% width by y=0.85), so a full-width box stuck out past the zone's own white
    outline and made it look like the trigger was firing far more often than it
    actually was. `poly` is (bottom-left, bottom-right, top-right, top-left), matching
    STOP_ZONE_PRESETS' convention."""
    h, w = frame.shape[:2]
    bl, br, tr, tl = poly
    y_bottom, y_top = bl[1], tl[1]

    def interp_x(p_bottom, p_top, y):
        if p_top[1] == p_bottom[1]:
            return p_bottom[0]
        t = (y - p_bottom[1]) / (p_top[1] - p_bottom[1])
        t = max(0.0, min(1.0, t))
        return p_bottom[0] + t * (p_top[0] - p_bottom[0])

    y_line = near_field_y
    lo, hi = (y_top, y_bottom) if y_top < y_bottom else (y_bottom, y_top)
    y_line = max(lo, min(hi, y_line))  # clamp within the zone's own y-range
    left_x = interp_x(bl, tl, y_line)
    right_x = interp_x(br, tr, y_line)
    slice_poly = [bl, br, (right_x, y_line), (left_x, y_line)]
    pts = np.array([[int(px * w), int(py * h)] for px, py in slice_poly], dtype=np.int32)

    overlay = frame.copy()
    cv2.fillPoly(overlay, [pts], color)
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)
    cv2.line(frame, (int(left_x * w), int(y_line * h)), (int(right_x * w), int(y_line * h)), color, 2)


class CameraWorker(threading.Thread):
    """Owns one camera: capture -> detect (backend-agnostic detect_fn) -> annotate ->
    publish latest frame + stop flag. Its own thread so cameras never block each other."""

    def __init__(self, index, width, height, label, detect_fn, args, shared):
        super().__init__(daemon=True)
        self.index = index
        self.label = label
        self.detect_fn = detect_fn
        self.args = args
        self.shared = shared          # dict holding live 'zone_idx' / 'show_zone'
        self.ok = False
        self.running = True
        self.lock = threading.Lock()
        self.latest = None
        self.latest_raw = None     # unannotated frame copy, for cross-camera template-match stereo
        self.latest_stop = False   # is a person currently inside this cam's stop zone
        self.latest_caution = False  # 2026-07-27: person in-lane at ANY distance (pre-stop slowdown)
        self.latest_boxes = []     # last detection result, for cross-camera stereo matching
        self.latest_boxes_ts = 0.0 # wall-clock time these boxes were captured (see run_stereo_live)
        self.frame_w = 0           # ACTUAL captured frame size (may differ from requested
        self.frame_h = 0           # width/height if the camera doesn't honor cap.set()) —
                                    # use this, not args.width/height, to normalize box coords
        self.stereo_key = None     # "a" or "b" — which half of the stereo pair this camera is (set in main())
        self.fps = 0.0
        self.cap = None
        if index < 0:
            return
        # Retry loop (2026-07-27): opening two USB cameras via DSHOW back-to-back in
        # the same process is intermittently flaky on this rig — cam0 or cam1 (never
        # a consistent one) has failed to open on 3 separate launches this session,
        # each time recovering fine on a plain retry a moment later. Not a fixed
        # "not enough time since the last process closed" issue (seen even with a 2s+
        # gap already), looks like genuine driver-level flakiness rather than
        # something a delay reliably prevents — so retry a few times instead of
        # guessing the right wait.
        cap = None
        for attempt in range(4):
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            if cap.isOpened():
                break
            cap.release()
            time.sleep(0.5)
        if args.fourcc:
            # USB BANDWIDTH FIX (2026-07-27 lab session): running both cameras
            # together at 640x480 RAW (YUY2) collapsed cam0 13.3fps->1.1fps and
            # cam1 26.9fps->4.1fps solo-vs-together — each camera's SOLO ceiling
            # was fine (matches the old documented 13/27fps baseline), only
            # running BOTH concurrently starved them, confirming USB bus
            # bandwidth contention (2x raw YUY2 streams over one controller),
            # not GPU/compute (detect-every 1000 showed the same collapse with
            # YOLO barely running at all). Forcing MJPG (camera-side compressed,
            # far fewer bytes/frame over the wire) measured 1.1/4.1fps -> a
            # clean ~12.5fps each running together (quick round-robin bench,
            # real per-camera threads should do at least as well). Must be set
            # BEFORE width/height — changing pixel format can reset other props.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if args.fps:
            # Explicitly request a matched frame rate on BOTH cameras (2026-07-27,
            # user request) — these two cameras run at very different native rates
            # (observed 13 vs 15-24 FPS) with no sync, the root cause behind the
            # frame-skew guard added earlier. A camera can't be pushed past its own
            # hardware ceiling (cam0 appears capped near 13), but a faster camera CAN
            # be told to throttle down to match — cv2.CAP_PROP_FPS is a request, not
            # a guarantee, some DSHOW drivers ignore it entirely, so this reduces the
            # mismatch on cameras that honor it without breaking ones that don't
            # (self.fps below is measured live either way, not assumed).
            cap.set(cv2.CAP_PROP_FPS, args.fps)
        if cap.isOpened():
            self.cap = cap
            self.ok = True
        else:
            print(f"[WARN] camera {label} (index {index}) failed to open after 4 attempts — skipping it.")

    def run(self):
        last_t = time.time()
        frame_i = 0
        boxes = []          # last detection result: list of (x1,y1,x2,y2,conf,in_zone)
        boxes_ts = 0.0      # wall-clock time `boxes` was actually (re)detected
        stop = False
        n_people = 0
        while self.running and self.ok:
            ret, frame = self.cap.read()
            if not ret:
                continue
            now = time.time()
            dt = now - last_t
            last_t = now
            inst = 1.0 / dt if dt > 0 else 0.0
            self.fps = self.fps * 0.9 + inst * 0.1 if self.fps > 0 else inst

            # Snapshot the UNANNOTATED frame now, before any boxes/zone/text are drawn
            # onto `frame` in place below — template-match stereo needs clean pixels.
            raw_frame = frame.copy() if self.args.template_stereo else None
            h, w = frame.shape[:2]
            zone = STOP_ZONE_PRESETS[self.shared["zone_idx"]]
            # f_px/dist_m here feeds ONLY the optional --log-file monocular calibration
            # logger below — it is NOT used for any live color/STOP decision anymore.
            # The live decision is exclusively the calibrated cross-camera stereo
            # distance (run_stereo_live) — the old single-camera monocular estimate
            # was proven unreliable (up to ~43% error) and is no longer trusted live.
            f_px = (w * 0.5) / math.tan(math.radians(self.args.hfov) * 0.5)

            # ── Detection runs only every Nth captured frame (--detect-every) so the
            # video stays smooth at the camera's own rate while the expensive CPU
            # forward-pass runs less often. Between detections we just redraw the
            # last known boxes — a person can't cross the frame in a few tens of ms,
            # so the stop decision stays valid. This decouples display FPS from the
            # detector's speed, which was the cause of the low FPS.
            if frame_i % max(1, self.args.detect_every) == 0:
                raw = merge_duplicate_boxes(self.detect_fn(frame))
                boxes_ts = now
                boxes = []
                n_people = 0
                n_static = 0
                for (x1, y1, x2, y2, conf, kind) in raw:
                    if kind == "person":
                        n_people += 1
                        box_h = max(1, y2 - y1)
                        dist_m_mono = f_px * self.args.person_height / box_h
                        log_calib_row(self.args, self.shared, self.label, box_h, dist_m_mono, conf)
                    else:
                        n_static += 1
                    boxes.append((x1, y1, x2, y2, conf, kind))
            frame_i += 1

            # EVERY matched person gets their OWN calibrated distance + color tier (from
            # run_stereo_live's per-person tracker, 10Hz, <0.3s fresh). Each drawn person
            # box is paired to a matched person by nearest box center in THIS camera —
            # so with several people in view, each shows their own distance, not one
            # shared number. A person box with no match (seen in only one camera) is
            # drawn neutral grey with no distance claim. Chairs stay magenta, presence-only.
            # These three are needed for self.latest_stop below regardless of whether
            # the window is visible — everything else in this section is pure display.
            fresh = (time.time() - self.shared.get("stereo_ts", 0)) < 0.3
            stereo_stop = self.shared.get("stereo_stop", False)
            stereo_caution = self.shared.get("stereo_caution", False)
            stereo_near_field = self.shared.get("stereo_near_field", False)
            people = self.shared.get("stereo_people") or []

            # ── Draw (every frame) — SKIPPED entirely when the window is hidden
            # (--auto-hide-window, 2026-07-27): zone overlay, box outlines, labels,
            # and the banner are pure display cost with zero bearing on the actual
            # STOP/CLEAR decision computed above/below — skipping them when nobody's
            # looking saves real per-frame CPU work. This does NOT touch YOLO
            # inference (the dominant cost), which keeps running unconditionally —
            # detection itself never stops, only drawing does.
            if self.shared.get("window_visible", True):
                if self.shared["show_zone"]:
                    draw_stop_zone(frame, zone, (255, 255, 255))
                    # Per-camera threshold (2026-07-27) — self.stereo_key is "a"/"b",
                    # set in main() when the stereo pair is assigned; falls back to the
                    # shared value if this worker isn't part of an active stereo pair.
                    my_near_field_y = (self.args.near_field_y_a if self.stereo_key == "a"
                                        else self.args.near_field_y_b if self.stereo_key == "b"
                                        else self.args.near_field_y)
                    draw_near_field_zone(frame, zone, my_near_field_y, (0, 0, 255))
                static_d = self.shared.get("stereo_static_dist_m")
                static_fresh = static_d is not None and (time.time() - self.shared.get("stereo_static_ts", 0)) < 0.3
                chair_idxs = [i for i, b in enumerate(boxes) if b[5] == "chair"]
                if static_fresh and self.stereo_key:
                    static_target = self.shared.get(f"stereo_static_box_{self.stereo_key}")
                    best_chair_idx = nearest_box_idx(boxes, chair_idxs, static_target)
                else:
                    best_chair_idx = max(chair_idxs, key=lambda i: boxes[i][4]) if chair_idxs else None
                # this camera's matched-person targets: (center_x, center_y, distance,
                # in_zone, display_trusted)
                ptargets = []
                if fresh and self.stereo_key:
                    for p in people:
                        b = p["box_a"] if self.stereo_key == "a" else p["box_b"]
                        cx_px = (b[0] + b[2]) / 2.0
                        # DISPLAY_TRUST band (2026-07-27, real concern: "if an examiner
                        # sees a wrong distance I'm in trouble") — deliberately NARROWER
                        # than in_zone/the safety STOP zone. in_zone's bottom edge is
                        # full-width (needed so the near-field trigger has corner
                        # coverage — see earlier fix), so a person can be off-center AND
                        # still "in_zone" if they're close. The near-field trigger
                        # doesn't use distance at all, so the SAFETY decision is fine
                        # either way — but the displayed NUMBER for an off-axis person is
                        # still lens-distortion-unreliable regardless of why they're
                        # in_zone. This band only gates what's shown on screen, never the
                        # STOP/CAUTION decision (that stays on the wider in_zone, computed
                        # independently in run_stereo_live).
                        display_trusted = (0.35 * w) <= cx_px <= (0.65 * w)
                        ptargets.append((cx_px, (b[1] + b[3]) / 2.0, p["dist"], p["in_zone"], display_trusted))

                def person_dist(cx, cy, thresh2=100 * 100):
                    """Returns (dist, in_zone, display_trusted) for the nearest matched
                    person, or None. Caller shows a precise number ONLY when
                    display_trusted — see that flag's definition above for why this is a
                    stricter/narrower check than in_zone (which only gates the actual
                    STOP/CAUTION safety decision, computed elsewhere)."""
                    best, best_d2 = None, thresh2
                    for tx, ty, d, in_zone, display_trusted in ptargets:
                        d2 = (tx - cx) ** 2 + (ty - cy) ** 2
                        if d2 < best_d2:
                            best_d2, best = d2, (d, in_zone, display_trusted)
                    return best

                for i, (x1, y1, x2, y2, conf, kind) in enumerate(boxes):
                    if kind == "chair":
                        color = (200, 0, 200)  # magenta — static object
                        if static_fresh and i == best_chair_idx:
                            label = f"STATIC chair {static_d:.1f}m {conf*100:.0f}%"
                        else:
                            label = f"STATIC chair {conf*100:.0f}%"
                    else:  # person
                        match = person_dist((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                        if match is not None and match[2]:
                            # Centered enough to trust the number (display_trusted) —
                            # NOT just in_zone, see that flag's definition above.
                            # 3-tier distance color: orange <1m, yellow 1-2m, green >2m
                            d = match[0]
                            color = (0, 140, 255) if d < 1.0 else (0, 255, 255) if d < 2.0 else (0, 200, 0)
                            label = f"{d:.1f}m {conf*100:.0f}%"
                        elif match is not None:
                            # Matched but off-center (display_trusted False) — distance
                            # for an off-axis person is lens-distortion-unreliable (found
                            # 2026-07-27: an off-center person ~4m away read as 2.4m).
                            # Still SHOW the number (2026-07-27, user preference over
                            # suppressing it entirely) but flag it as low-confidence
                            # rather than claiming the same precision as a centered
                            # reading — dimmer grey-blue, distinct from the 3-tier
                            # trusted colors above. Covers BOTH "outside the safety zone
                            # entirely" and "inside the (wider) safety zone but still
                            # off-center" — the STOP/CAUTION decision is computed
                            # independently and unaffected either way (see
                            # run_stereo_live / the near-field trigger, which doesn't use
                            # distance at all).
                            d = match[0]
                            color = (150, 120, 90)
                            label = f"{d:.1f}m (low trust) {conf*100:.0f}%"
                        else:
                            color = (150, 150, 150)
                            label = f"{conf*100:.0f}%"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.circle(frame, (int((x1 + x2) / 2), y2), 5, color, -1)
                    cv2.putText(frame, label, (x1, max(y1 - 6, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # Near-field (2026-07-27) is a single-camera trigger — checked FIRST and
                # independent of `fresh`/`people` (a cross-camera match/stereo pair),
                # since the whole point is to catch someone too close for a match to
                # exist at all. Everything else below is unchanged, still gated on a
                # fresh stereo pair as before.
                if stereo_stop and stereo_near_field:
                    banner, banner_col = "STOP - PERSON VERY CLOSE (near-field)", (0, 0, 255)
                elif fresh and people:
                    if stereo_stop:
                        banner, banner_col = f"STOP - PERSON WITHIN {self.args.stop_distance:.0f}m", (0, 0, 255)
                    elif stereo_caution:
                        banner, banner_col = "CAUTION - PERSON IN LANE, SLOWING", (0, 165, 255)
                    else:
                        banner, banner_col = "CLEAR", (0, 200, 0)
                else:
                    banner = "NO STEREO MATCH"
                    banner_col = (120, 120, 120)
                cv2.rectangle(frame, (0, 0), (w, 40), (20, 20, 20), -1)
                cv2.putText(frame, banner, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, banner_col, 2)
                static_txt = f"  static:{n_static}" if self.args.detect_static else ""
                cv2.putText(frame, f"{self.label}  {self.fps:4.1f} FPS  people:{n_people}{static_txt}",
                            (max(0, w - 340), 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            with self.lock:
                self.latest = frame
                self.latest_raw = raw_frame
                # Gated on `fresh` alone, NOT `fresh and people` (2026-07-27) — a
                # near-field trigger can be True with `people` empty (no cross-camera
                # match exists at all, that's the whole point of that check), and
                # `stereo_stop`/`stereo_caution` already correctly encode that. The old
                # "and people" requirement would have silently swallowed a near-field-
                # only STOP right here. `fresh` alone is still a sufficient staleness
                # guard — stereo_ts is updated every loop iteration regardless.
                self.latest_stop = stereo_stop if fresh else False
                self.latest_caution = stereo_caution if fresh else False
                self.latest_boxes = boxes
                self.latest_boxes_ts = boxes_ts
                self.frame_w, self.frame_h = w, h

    def get_frame_size(self):
        with self.lock:
            return self.frame_w, self.frame_h

    def get_latest_boxes_ts(self):
        with self.lock:
            return self.latest_boxes_ts

    def get_latest(self):
        with self.lock:
            return self.latest

    def get_latest_raw(self):
        with self.lock:
            return self.latest_raw

    def get_stop(self):
        with self.lock:
            return self.latest_stop

    def get_caution(self):
        with self.lock:
            return self.latest_caution

    def get_latest_boxes(self):
        with self.lock:
            return self.latest_boxes

    def stop_worker(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()


def run_auto_calib(args, shared, distances, prep_s, hold_s):
    """Hands-free timed sequence — sets shared['marked_distance'] on a timer so the
    operator never has to touch the keyboard while standing at each distance. Cues are
    audible beeps by default, or silent (--no-beep) when an external narrator (e.g.
    a helper calling out cues timed to prep_s/hold_s) is telling the operator when to
    move instead."""
    beep = (lambda f, d: None) if args.no_beep else winsound.Beep
    time.sleep(2.0)  # let camera windows settle before the first cue
    for d in distances:
        print(f"[calib] walk to {d:.0f}m — you have {prep_s:.0f}s")
        countdown_beeps = 0 if args.no_beep else min(3, int(prep_s))
        time.sleep(max(0.0, prep_s - countdown_beeps))
        for _ in range(countdown_beeps):
            beep(1200, 120)
            time.sleep(1.0 - 0.12)
        beep(1800, 300)  # start-logging cue
        shared["marked_distance"] = float(d)
        print(f"[calib] LOGGING at {d:.0f}m for {hold_s:.0f}s — hold still")
        time.sleep(hold_s)
        shared["marked_distance"] = 0.0
        beep(500, 400)  # stop / move-on cue
    for f in (1000, 1300, 1600):
        beep(f, 150)
    print("[calib] sequence complete.")
    if args.auto_quit:
        shared["quit_event"].set()
    else:
        print("[calib] press 'q' in the video window to quit.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cam0", type=int, default=0)
    ap.add_argument("--cam1", type=int, default=1, help="second camera index (-1 to disable)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fourcc", type=str, default="MJPG",
                     help="capture codec requested from each camera before width/height "
                          "(2026-07-27) — default MJPG (camera-side compressed) fixes a real "
                          "USB-bandwidth collapse seen running two raw-format 640x480 cameras "
                          "together (each camera's SOLO fps was fine — 13/27fps, matching the "
                          "documented baseline — but running both TOGETHER on raw format "
                          "starved them to 1.1/4.1fps; MJPG measured a clean ~12.5fps each "
                          "together). Pass '' to leave the camera's own default format alone.")
    ap.add_argument("--fps", type=float, default=0.0,
                     help="request this capture FPS on both cameras (2026-07-27) — "
                          "TESTED AND CONFIRMED INEFFECTIVE on this rig's DSHOW "
                          "drivers (cam1 ran FASTER, 26.2 FPS, after requesting 13 — "
                          "the request was silently ignored). Left in as a no-cost "
                          "option in case different camera hardware honors it later; "
                          "default 0 = don't bother asking. The real fix for the FPS "
                          "mismatch is the frame-skew guard (--max-frame-skew-ms), "
                          "which doesn't need the cameras to actually match speed")
    ap.add_argument("--conf", type=float, default=0.45,
                     help="min detection confidence (lower = more sensitive)")
    ap.add_argument("--conf0", type=float, default=0.25,
                     help="(2026-07-28) per-camera override of --conf for cam0 only — "
                          "live testing found cam0 systematically less sensitive than "
                          "cam1 (missed a plainly unoccluded chair cam1 caught at 74%%, "
                          "and the original false-STOP screenshot showed people:1 in "
                          "cam0 vs people:3 in cam1). Default here is an empirical first "
                          "trial (roughly halved from --conf's 0.45), not independently "
                          "derived — tune from a live retest. Set equal to --conf to "
                          "disable this override if cam0 turns out not to need it.")
    ap.add_argument("--conf1", type=float, default=None,
                     help="per-camera override of --conf for cam1 only — see --conf0. "
                          "Default None = use --conf (cam1 hasn't shown the weakness).")
    ap.add_argument("--detect-static", action="store_true",
                     help="also detect chairs as a stand-in 'static object' class, "
                          "drawn/labeled separately from the 'person' dynamic-object "
                          "pipeline (no stereo distance/STOP wiring yet — visual only)")
    ap.add_argument("--static-conf", type=float, default=0.25,
                     help="min confidence for static-object (chair) detections — lower "
                          "than --conf by default since COCO training data skews heavily "
                          "toward 'person', so furniture classes score less confidently")
    ap.add_argument("--static-conf0", type=float, default=0.15,
                     help="(2026-07-28) per-camera override of --static-conf for cam0 "
                          "only — see --conf0 for why (same cam0 weak-detection finding, "
                          "confirmed directly on chairs: cam0 detected NOTHING on a "
                          "plainly unoccluded chair cam1 caught at 74%%, twice in a row). "
                          "Empirical first trial, tune from a live retest.")
    ap.add_argument("--static-conf1", type=float, default=None,
                     help="per-camera override of --static-conf for cam1 only — see "
                          "--static-conf0. Default None = use --static-conf.")
    ap.add_argument("--nms-iou", type=float, default=0.45,
                     help="--backend yolo only: duplicate-box suppression threshold "
                          "(ultralytics default 0.7 is permissive — irregular shapes like "
                          "chairs can produce two overlapping partial boxes that survive "
                          "it; lower merges more aggressively)")
    ap.add_argument("--detect-every", type=int, default=3,
                     help="run the detector every Nth frame (higher = smoother video, "
                          "less frequent detection). Video still displays every frame.")
    ap.add_argument("--stop-distance", type=float, default=6.0,
                     help="only STOP when a person in the zone is within this many metres")
    ap.add_argument("--near-field-y", type=float, default=0.85,
                     help="(2026-07-27) SINGLE-camera safety net, independent of the stereo "
                          "distance calc: if a person's feet (box bottom) are below this "
                          "fraction of EITHER camera's frame height (0..1, 1=bottom edge), "
                          "force STOP immediately — no cross-camera match or distance value "
                          "needed at all. Closes a real gap where someone close enough falls "
                          "out of one camera's FOV entirely, so no matched pair (and thus no "
                          "distance) exists. Lower = triggers farther out / more sensitive. "
                          "Default for BOTH cameras unless overridden per-camera below.")
    ap.add_argument("--near-field-y0", type=float, default=None,
                     help="per-camera override of --near-field-y for cam0 only (the two "
                          "cameras' mounting/framing can differ enough that the same "
                          "fraction doesn't mean the same real distance in each — observed "
                          "live 2026-07-27, cam1 read consistently higher than cam0 for the "
                          "same real position). Defaults to --near-field-y if not given.")
    ap.add_argument("--near-field-y1", type=float, default=None,
                     help="per-camera override of --near-field-y for cam1 only — see "
                          "--near-field-y0. Defaults to --near-field-y if not given.")
    ap.add_argument("--hfov", type=float, default=60.0,
                     help="camera horizontal field of view in degrees (for distance estimate)")
    ap.add_argument("--person-height", type=float, default=1.7,
                     help="assumed real person height in metres (for distance estimate)")
    ap.add_argument("--zone", type=int, default=0,
                     help="initial stop-zone preset index (0=wide,1=full frame,2=narrow lane)")
    ap.add_argument("--no-zone", action="store_true",
                     help="hide the stop-zone (ROI) trapezoid AND make STOP distance-only "
                          "— any matched person within --stop-distance triggers it, "
                          "regardless of lateral position. The zone (lateral path filter) "
                          "stays in the code for on-cart use; this just turns it off now.")
    ap.add_argument("--auto-hide-window", action="store_true",
                     help="(2026-07-27) keep camera windows hidden during normal silent "
                          "monitoring, auto-show them only when a STOP triggers, hide "
                          "again --window-hide-delay seconds after it clears. Skips the "
                          "per-frame drawing (boxes/zone/banner) and cv2.imshow() "
                          "compositor cost while hidden — does NOT reduce YOLO inference "
                          "load, which always keeps running (it's what detects the STOP "
                          "in the first place). Off by default so manual testing/"
                          "calibration still shows continuous live video.")
    ap.add_argument("--window-hide-delay", type=float, default=3.0,
                     help="--auto-hide-window: seconds to keep the window open after "
                          "the last STOP clears, before hiding it again")
    ap.add_argument("--status-file", type=str, default="",
                     help="if set, continuously write 'STOP <unixtime>' / 'CLEAR <unixtime>' "
                          "here so the C++ app can read the pedestrian state")
    ap.add_argument("--log-file", type=str, default="",
                     help="if set, append one CSV row per detection (timestamp, camera, "
                          "marked ground-truth distance, box height px, estimated distance, "
                          "confidence) for offline hfov calibration — see '0'-'9' keys")
    ap.add_argument("--auto-calib", action="store_true",
                     help="hands-free beep-timed calibration sequence — see module docstring")
    ap.add_argument("--calib-distances", type=str, default="1,2,3,5",
                     help="comma-separated metres to step through for --auto-calib")
    ap.add_argument("--calib-prep", type=float, default=6.0,
                     help="--auto-calib: seconds to walk to each distance before it logs")
    ap.add_argument("--calib-hold", type=float, default=4.0,
                     help="--auto-calib: seconds spent logging at each distance")
    ap.add_argument("--no-beep", action="store_true",
                     help="--auto-calib: skip audible cues (timing is still exact — pair "
                          "with an external narrator instead, e.g. a helper telling you when "
                          "to move, timed to match --calib-prep/--calib-hold)")
    ap.add_argument("--auto-quit", action="store_true",
                     help="--auto-calib: close the window(s) automatically once the "
                          "sequence finishes instead of waiting for 'q'")
    ap.add_argument("--stereo-log-file", type=str, default="",
                     help="if set (requires both cameras active), log cross-camera "
                          "disparity samples for stereo depth calibration — see "
                          "run_stereo_logger docstring. Gated by the same '0'-'9' / "
                          "--auto-calib ground-truth marker as --log-file.")
    ap.add_argument("--baseline-m", type=float, default=0.60,
                     help="physical distance between the two camera centres, metres "
                          "(measured 0.60m for this rig) — used for stereo depth math")
    ap.add_argument("--stereo-live", action="store_true",
                     help="print a real-time calibrated distance reading to the console "
                          "(requires both cameras active) — see run_stereo_live docstring")
    ap.add_argument("--stereo-a", type=float, default=199.86,
                     help="stereo calibration constant A in distance_m = A/(disparity_px-B) "
                          "(fitted 2026-07-17 from 1.0/2.0/2.8m ground truth, this rig only)")
    ap.add_argument("--stereo-b", type=float, default=41.67,
                     help="stereo calibration constant B (disparity offset), see --stereo-a")
    ap.add_argument("--stereo-max-dist", type=float, default=6.0,
                     help="reject any stereo match reporting farther than this (metres) "
                          "— A/(disparity-B) is unstable near disparity=B, a tiny error "
                          "there explodes into a physically-impossible reading")
    ap.add_argument("--static-height", type=float, default=0.85,
                     help="assumed real height (m) of a static/chair detection, used by "
                          "the --size-check-ratio cross-check (people use "
                          "--person-height). Rough by nature — real chairs vary ~0.8-1.1m "
                          "and the box may or may not include the backrest — but the "
                          "check only fires on gross disagreement, so this is sufficient.")
    ap.add_argument("--size-check-ratio", type=float, default=2.5,
                     help="(2026-07-27) reject a stereo person-match when its disparity "
                          "claims a distance more than this factor CLOSER than the "
                          "person's on-screen box height can physically support — catches "
                          "wrong cross-camera pairings that produce false STOPs on distant "
                          "people (campus test: a clearly-far person reported at 1.4m). "
                          "Uses f_px = stereo_a/baseline_m from the stereo calibration "
                          "itself, NOT the uncalibrated --hfov. Deliberately ONE-"
                          "DIRECTIONAL: never rejects when stereo reads FARTHER than size "
                          "suggests, since a crouching/occluded person legitimately looks "
                          "small-boxed while genuinely being close. Default 2.5 chosen by "
                          "working the numbers, not guessed: 2.0 was verified to wrongly "
                          "reject a heavily-occluded (~150px box) person at 3m, while 2.5 "
                          "keeps every realistic real-person case (full-body, partly and "
                          "heavily occluded, 2-5m) and still rejects the observed campus "
                          "false pairings by a 4-7x margin. 0 disables the check entirely.")
    ap.add_argument("--clip-safe-dist-m", type=float, default=0.6,
                     help="(2026-07-27) ceiling used ONLY for the rare edge-clip residual "
                          "case in stereo_match_all where no valid disparity reference "
                          "exists at all (box clipped both sides, or opposite sides between "
                          "cameras) — see that function's docstring. Anything reaching this "
                          "path is close enough to be clipping the frame by definition, so "
                          "this should stay a genuinely conservative 'assume close' value")
    ap.add_argument("--max-frame-skew-ms", type=float, default=60.0,
                     help="reject a cam0/cam1 box pair if their actual capture times are "
                          "more than this many ms apart (cameras run at different native "
                          "FPS, unsynced) — harmless while stationary, but corrupts "
                          "disparity once the cart itself is moving between the two "
                          "cameras' capture instants; 60ms default keeps scene-shift error "
                          "small at typical cart/walking speeds")
    ap.add_argument("--template-stereo", action="store_true",
                     help="compute stereo disparity by matching object PIXELS between the "
                          "two cameras (cv2.matchTemplate) instead of subtracting box "
                          "centers — fixes wide/asymmetric objects like chairs where box "
                          "centers don't track the same physical point. Applied to static "
                          "objects; also to people with --template-stereo-people.")
    ap.add_argument("--template-stereo-people", action="store_true",
                     help="also use template-match disparity for people (default: people "
                          "keep box-center, already validated <7cm; static uses template)")
    ap.add_argument("--template-min-score", type=float, default=0.40,
                     help="min normalized-cross-correlation score to trust a template "
                          "stereo match (0-1); below this the reading is rejected")
    ap.add_argument("--fps-monitor", type=float, default=0.0,
                     help="print per-camera throughput every second for this many seconds, "
                          "then exit — no person/positioning needed, pure pipeline benchmark")
    ap.add_argument("--backend", choices=["mobilenet", "yolo"], default="mobilenet",
                     help="mobilenet = cv2.dnn, CPU, zero extra installs (default). "
                          "yolo = YOLOv8n via ultralytics, CUDA if available (more accurate/faster; "
                          "requires torch+ultralytics, see module docstring for install command)")
    ap.add_argument("--device", choices=["auto", "cpu"], default="auto",
                     help="--backend yolo only: force CPU even if a GPU is available")
    args = ap.parse_args()
    # Resolve per-camera near-field thresholds once, here — everywhere else just reads
    # these two, never args.near_field_y0/1/near_field_y directly (see those args' help).
    args.near_field_y_a = args.near_field_y0 if args.near_field_y0 is not None else args.near_field_y
    args.near_field_y_b = args.near_field_y1 if args.near_field_y1 is not None else args.near_field_y

    if args.backend == "mobilenet":
        print("Using MobileNet-SSD person detector (cv2.dnn, CPU).")
    build_detector = build_yolo_detector if args.backend == "yolo" else build_mobilenet_detector

    shared = {"zone_idx": max(0, min(args.zone, len(STOP_ZONE_PRESETS) - 1)),
              "show_zone": not args.no_zone, "marked_distance": 0.0, "log_lock": threading.Lock(),
              "quit_event": threading.Event(), "stereo_dist_m": None, "stereo_ts": 0.0,
              "stereo_people": [], "stereo_caution": False, "stereo_near_field": False,
              "static_obs": [],
              "stereo_static_dist_m": None, "stereo_static_ts": 0.0,
              "stereo_static_box_a": None, "stereo_static_box_b": None,
              "window_visible": not args.auto_hide_window}

    if args.log_file:
        if not os.path.exists(args.log_file):
            with open(args.log_file, "w") as f:
                f.write("timestamp,camera,marked_distance_m,box_h_px,dist_est_m,confidence\n")
        else:
            print(f"[calib] appending to existing {args.log_file} (won't overwrite prior runs)")
        print(f"Calibration logging to: {args.log_file} "
              "(press '0'-'9' to mark ground-truth distance in metres, 0=clear)")

    # Each camera gets its OWN detector instance (not a shared one) — calling YOLO's
    # predict() concurrently from two threads on the same model object risks internal
    # state clashes; a separate instance per thread is small (YOLOv8n) and avoids that
    # entirely. Skipped for a disabled camera (index<0) to avoid an unused GPU load.
    #
    # PER-CAMERA confidence (2026-07-28, root-cause fix): live campus/lab testing found
    # cam0 repeatedly missing objects cam1 detected confidently — a chair cam1 saw at
    # 74% confidence, cam0 saw NOT AT ALL, twice in a row, on a plainly unoccluded
    # object. Also matches the original campus false-STOP screenshot (people:1 in cam0
    # vs people:3 in cam1). This asymmetry is the likely ROOT cause of the cross-camera
    # pairing failures chased earlier — when the two cameras' detected object COUNTS
    # differ because one camera is simply less sensitive, no amount of downstream
    # matching-logic patching (template-match rejection, size cross-check — both kept,
    # they remain valid defense-in-depth for whatever still slips through) fully closes
    # a gap that starts at the detection stage. --conf0/--conf1 and
    # --static-conf0/--static-conf1 (default None = fall back to --conf/--static-conf)
    # let cam0 run a lower, more permissive threshold to close that gap. Values are a
    # first empirical trial (halved), not independently derived — tune from a live
    # re-test of the same single-chair scene that exposed the asymmetry.
    def resolve_conf(base, override0, override1, label):
        if label == "cam0" and override0 is not None:
            return override0
        if label == "cam1" and override1 is not None:
            return override1
        return base

    def worker(index, label):
        if index < 0:
            return CameraWorker(index, args.width, args.height, label, None, args, shared)
        conf = resolve_conf(args.conf, args.conf0, args.conf1, label)
        static_conf = resolve_conf(args.static_conf, args.static_conf0, args.static_conf1, label)
        return CameraWorker(index, args.width, args.height, label,
                             build_detector(args, conf, static_conf), args, shared)

    t_init_start = time.time()
    workers = [worker(args.cam0, "cam0"), worker(args.cam1, "cam1")]
    print(f"[startup] camera+model init took {time.time()-t_init_start:.1f}s")
    active = [w for w in workers if w.ok]
    if not active:
        print("No cameras opened successfully. Check indices with --cam0/--cam1.")
        sys.exit(1)
    if len(active) == 2:
        active[0].stereo_key = "a"
        active[1].stereo_key = "b"
    for w in active:
        w.start()

    if args.fps_monitor > 0:
        run_fps_monitor(active, args.fps_monitor)
        for w in active:
            w.stop_worker()
        sys.exit(0)

    if args.stereo_log_file:
        if len(active) < 2:
            print("[stereo] WARNING: --stereo-log-file needs both cameras active — skipping.")
        else:
            if not os.path.exists(args.stereo_log_file):
                with open(args.stereo_log_file, "w") as f:
                    f.write("timestamp,marked_distance_m,disparity_px,conf_a,conf_b\n")
            else:
                print(f"[stereo] appending to existing {args.stereo_log_file}")
            print(f"Stereo disparity logging to: {args.stereo_log_file} "
                  f"(baseline={args.baseline_m:.2f}m)")
            threading.Thread(target=run_stereo_logger, args=(args, shared, active[0], active[1]),
                              daemon=True).start()

    if args.stereo_live:
        if len(active) < 2:
            print("[stereo-live] WARNING: needs both cameras active — skipping.")
        else:
            print(f"[stereo-live] using distance_m = {args.stereo_a:.2f} / "
                  f"(disparity_px - {args.stereo_b:.2f})")
            threading.Thread(target=run_stereo_live, args=(args, shared, active[0], active[1]),
                              daemon=True).start()

    print("Keys: 'q'/Esc quit | 's' toggle zone overlay | 'z' cycle zone preset")
    if args.status_file:
        print(f"Publishing STOP/CAUTION/CLEAR status to: {args.status_file}")

    if args.auto_calib:
        if not args.log_file:
            print("[calib] WARNING: --auto-calib with no --log-file — nothing will be saved.")
        distances = [float(x) for x in args.calib_distances.split(",") if x.strip()]
        threading.Thread(target=run_auto_calib, args=(args, shared, distances, args.calib_prep,
                                                        args.calib_hold), daemon=True).start()

    def write_status(stop, caution):
        if not args.status_file:
            return
        # CAUTION (2026-07-27): person in-lane at any distance, not yet close enough to
        # STOP — read by main.cpp to slow the cart down (PED_CAUTION_CRUISE_VOLTAGE)
        # BEFORE it's close enough to need a full stop. See control.cpp for why: the
        # cart's brake is a plain throttle-cutoff + coast, no strong active deceleration.
        state = "STOP" if stop else ("CAUTION" if caution else "CLEAR")
        try:
            # write to a temp file then replace, so the C++ reader never sees a
            # half-written line (atomic on the same volume on Windows).
            tmp = args.status_file + ".tmp"
            with open(tmp, "w") as f:
                f.write(f"{state} {time.time():.3f}\n")
            os.replace(tmp, args.status_file)
        except OSError:
            pass

    # --auto-hide-window state (2026-07-27): window_shown mirrors shared["window_visible"]
    # (the CameraWorker threads read the shared copy to decide whether to bother
    # drawing at all — see CameraWorker.run()). Starts hidden (last_stop_seen=0.0
    # makes the very first "time since last stop" huge, so want_visible is False
    # until an actual STOP happens).
    window_shown = not args.auto_hide_window
    last_stop_seen = 0.0
    try:
        while True:
            any_stop = False
            any_caution = False
            for w in active:
                any_stop = any_stop or w.get_stop()
                any_caution = any_caution or w.get_caution()

            if args.auto_hide_window:
                # Show the window on CAUTION too (2026-07-27), not just STOP — that's
                # exactly the lead-up phase where seeing what triggered the slowdown
                # is most useful.
                if any_stop or any_caution:
                    last_stop_seen = time.time()
                want_visible = any_stop or any_caution or (time.time() - last_stop_seen) < args.window_hide_delay
                if want_visible != window_shown:
                    window_shown = want_visible
                    shared["window_visible"] = window_shown
                    if not window_shown:
                        for w in active:
                            cv2.destroyWindow(f"Pedestrian Detect - {w.label}")

            if window_shown:
                for w in active:
                    f = w.get_latest()
                    if f is not None:
                        cv2.imshow(f"Pedestrian Detect - {w.label}", f)

            write_status(any_stop, any_caution)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord('q'), 27) or shared["quit_event"].is_set():
                break
            if key == ord('s'):
                shared["show_zone"] = not shared["show_zone"]
            if key == ord('z'):
                shared["zone_idx"] = (shared["zone_idx"] + 1) % len(STOP_ZONE_PRESETS)
                print("stop-zone preset ->", shared["zone_idx"])
            if ord('0') <= key <= ord('9'):
                shared["marked_distance"] = float(key - ord('0'))
                if shared["marked_distance"] > 0:
                    print(f"marked distance -> {shared['marked_distance']:.0f}m (logging)")
                else:
                    print("marked distance cleared (not logging)")
    finally:
        write_status(False, False)   # leave a CLEAR behind so a stale STOP/CAUTION can't strand or slow the cart
        for w in active:
            w.stop_worker()
        for w in active:
            w.join(timeout=1.0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
