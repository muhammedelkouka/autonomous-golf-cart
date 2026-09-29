# Pedestrian Detection — Session Log & Reference (2026-07-17 → 2026-07-28)

## ROOT CAUSE found: cam0 is systematically less sensitive than cam1 (2026-07-28)

Chased the campus false-STOP bug and a chair-distance bug (both below) into their
matching-logic symptoms first, but live testing then exposed something more
fundamental: **cam0 detects the same real, unoccluded objects at meaningfully lower
confidence than cam1 — sometimes missing them entirely.**

Evidence: two separate single-chair tests (chair centered, plainly unoccluded, good
lighting) both showed cam1 detecting it (74% confidence) while **cam0 detected nothing
at all** (`static:0`). This is not occlusion or edge-clipping — the object was cleanly
visible in both frames. And the SAME pattern was already present in the original
campus false-STOP screenshot: `people:1` in cam0 vs `people:3` in cam1.

This reframes both bugs below: when the two cameras' detected-object COUNTS differ
because one camera is simply less sensitive, `stereo_match_all`'s rank-based pairing
has no correct answer to find — no amount of downstream matching-logic patching
(template-match rejection, apparent-size cross-check) fully closes a gap that starts at
the detection stage itself. Those two downstream fixes remain valid defense-in-depth,
but this is the fix that addresses the actual mechanism.

**Fix: per-camera confidence thresholds.** `build_mobilenet_detector()`/
`build_yolo_detector()` now take explicit `conf`/`static_conf` parameters (previously
read `args.conf`/`args.static_conf` directly, identically for both cameras) resolved
per-camera in `main()` via `resolve_conf()`. New flags: `--conf0`/`--conf1` (person),
`--static-conf0`/`--static-conf1` (chair) — `None` (the cam1 default) falls back to the
shared `--conf`/`--static-conf`. cam0 defaults: `--conf0 0.25` (was effectively 0.45),
`--static-conf0 0.15` (was effectively 0.25) — roughly halved as a first empirical
trial, explicitly NOT independently derived; needs live retest on the exact scene that
exposed the asymmetry to confirm cam0 now catches what cam1 catches, and to tune from
there. `py_compile` clean. **Not yet live-validated.**

## CAMPUS TEST 2026-07-28: false STOPs on distant people — two fixes

**Result of the run:** forward map drove cleanly with pedestrian detection ON and good
stopping distance. Two problems reported: (a) reverse-map localization (see the campus
memory file — investigated, traced to the known steering/corner limitation, NOT a
regression from this session's work), and (b) **false STOPs triggered by people who were
clearly far away** — screenshots showed people whose boxes are visibly small/distant
being reported at 1.4m and firing "STOP - PERSON WITHIN 3m".

**First hypothesis (WRONG, recorded because it shaped the first fix):** one screenshot
showed `people:1` in cam0 vs `people:3` in cam1, suggesting unequal detection counts
caused `stereo_match_all`'s left-to-right rank pairing (`zip()`) to pair different
physical people. Fix applied: `resolve_dist()` now REJECTS the pair (returns `None`)
when template matching fails/scores low, instead of silently falling back to the
rank-based box-center distance. Rationale: template matching (which searches the full
width of the other camera's frame and so independently rediscovers the true
correspondence) is the only mechanism that can tell two different people apart, so its
failure means the PAIR is untrustworthy, not just its refinement. Call site skips
`None`. This is still a correct, valuable fix — but user then supplied a second
screenshot showing `people:3` in BOTH cameras with the same false 1.4m reading, proving
unequal counts is not the (only) cause.

**Actual root cause:** equal COUNTS does not mean the same SET of people. The 40cm
baseline gives the cameras slightly different fields of view, so cam0 can see people
{A,B,C} while cam1 sees {B,C,D} — rank pairing then yields A↔B, B↔C, C↔D: every pair
wrong, counts matching perfectly. `--template-stereo-people` was already active in
production and did NOT catch it: a ~20x50px patch of a distant person can find a false
NCC peak above `--template-min-score` among repetitive corridor structure and
similar-looking people.

**Second fix — apparent-size cross-check** (`size_implausible()` in `stereo_match_all`):
a person's on-screen box height gives their distance *independently of any pairing*, so
it can veto a disparity that is physically impossible for that apparent size. Key
detail: `f_px` is derived from the stereo calibration itself — `distance =
baseline*f_px/disparity` is the same equation as `stereo_a/(disparity-stereo_b)`, so
`f_px = stereo_a/baseline_m` = **631px** for today's validated 40cm fit, which
back-solves to a ~54° HFOV (a sane webcam value). This deliberately does NOT use the
`--hfov` guess, which was found "totally wrong" in earlier testing.

**Deliberately ONE-DIRECTIONAL** (this is a safety device, so it may only reject in the
false-ALARM direction): rejects only when stereo claims *much closer* than apparent size
supports. It never rejects the opposite disagreement, because a crouching or
partly-occluded person legitimately has a short box while genuinely being close —
rejecting that would suppress a real STOP. The single-camera near-field trigger (no
pairing, no distance) remains as an independent backstop.

**Threshold chosen by working the numbers, not guessed** — verified across realistic
scenarios: ratio 2.0 wrongly rejected a heavily-occluded (~150px box) person at 3m; 2.5
keeps every realistic real-person case (full-body / partly / heavily occluded, 2–5m)
while still rejecting the observed campus false pairings by a 4–7x margin. Default
`--size-check-ratio 2.5`; `0` disables.

Both fixes `py_compile` clean. **NEITHER IS LIVE-TESTED YET.** Watch specifically that
genuine close-range detections still fire reliably (the new rejections make the system
more conservative; the existing coast-through tracker should absorb brief gaps, but that
interaction is unverified).

## Display-only "off-center" distance suppression (2026-07-27)

User's concern testing off-axis positions: distance is correct near center, wrong when
standing left/right — a known, already-explained limitation (uncorrected wide-angle lens
distortion off-axis; the earlier checkerboard intrinsics-calibration attempt this session
was meant to fix this properly but didn't converge). Real risk identified: the near-field
trigger fix earlier today widened the safety zone's bottom edge to full-width (needed for
corner coverage), which means a person can be off-center AND still `in_zone` if they're
close — so a lens-distortion-inaccurate NUMBER could now display for them, even though
the actual STOP/CAUTION decision is unaffected (the near-field trigger doesn't use
distance at all, and the safety decision was never wrong). But a wrong-looking number on
screen during a demo is its own problem ("if an examiner sees a wrong distance...").

**Fix: separate display trust from the safety zone**, in `CameraWorker.run()`'s per-box
label logic only — added a `display_trusted` flag, a NARROW fixed x-band (35%-65% of
frame width) independent of and stricter than `in_zone`. When NOT `display_trusted`, the
number is still shown (not suppressed — user preference: confidence-flagging reads more
sophisticated than hiding data) but qualified as `"{d:.1f}m (low trust) NN%"` in a
distinct dimmer color, instead of the 3-tier trusted-color scheme. Covers both "outside
the safety zone entirely" and "inside the wider safety zone but still off-center" — same
low-trust treatment either way. The actual STOP/CAUTION decision (`run_stereo_live`,
`in_zone`, the near-field trigger) is completely unaffected — this is purely what gets
drawn on screen. `py_compile` clean. No new CLI flag, no `main.cpp` change needed.

## Pre-campus safety fixes: control-loop focus starvation + GPU heat (2026-07-27)

User reported, ahead of the campus test: (1) the cart sometimes lost steering control
mid-turn specifically when the pedestrian-camera windows had OS focus instead of the
main UI, and (2) the laptop getting warmer over a sustained run.

**Control-loss root cause:** Windows grants a scheduling/priority boost to whichever
process owns the currently-FOCUSED window. `tools/pedestrian_detect.py` is launched at
`BELOW_NORMAL_PRIORITY_CLASS` specifically so it can't starve the control loop (see
`main.cpp` `ped_set_enabled()`) — but that's only a base priority; the focus boost can
still tip CPU scheduling toward it over the (unfocused, background) `GolfCartControl.exe`
under contention, exactly during a turn when the control loop is already doing more work
(steering calc, curvature feedforward) and can least afford a missed/delayed 50Hz tick.
**Fix:** `control_loop_thread()` (`control.cpp`) now calls
`SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL)` at startup —
elevates just that one thread to the highest user-mode priority, above what a normal
foreground-boosted process reaches, protecting its timing regardless of which window has
focus. Deliberately thread-level, not process-level, so it doesn't affect LiDAR/IMU/
render thread scheduling. Rebuilt clean. **Not yet live-tested** — needs a real turn
with the camera window focused to confirm the fix holds.

**GPU heat:** `--detect-every` 1->**2** in the production launch string — YOLO/CUDA
shares the GPU with Raylib rendering (flagged as an unmeasured risk earlier this
session); running inference on every captured frame from both cameras continuously is
real sustained load, the likely main heat source. Detection still runs ~7-13x/sec (was
~13-27x/sec) — still far above what the stopping-distance margins established earlier
today need; frames between detections just redraw the last known box position, so
display stays smooth either way. Rebuilt clean.

## Near-field trigger tuned live + wired into production (2026-07-27, continued)

Following up on the near-field trigger added earlier this session: first live test at
the code default (`--near-field-y 0.85`) found it firing far too readily — user
reported "it always detects me as near." Added a live diagnostic readout (`foot-y a=..
b=..` in the console) to tune from real numbers instead of guessing again. Findings:
- The default 0.85 didn't account for the specific camera mount/tilt — foot-y for a
  person at normal/moderate distance (~2.5-3.4m) was already 0.75-0.80, uncomfortably
  close to the 0.85 trigger.
- The two cameras read DIFFERENTLY for the same real position — cam1 consistently
  ~3-4 percentage points higher than cam0 (e.g. a=0.87 vs b=0.90 at the same distance).
  Added per-camera override args `--near-field-y0`/`--near-field-y1` (fall back to
  `--near-field-y` if not given) to handle this properly instead of one shared value.
- Also fixed the on-screen visualization: initially drawn as a full-width rectangle,
  but the lane-zone trapezoid narrows going up (already ~11%-89% width by y=0.85), so
  the full-width box stuck out past the zone's own white outline — `draw_near_field_zone()`
  rewritten to clip to the actual bottom slice of the zone polygon instead.
- Iteratively tuned live (0.85 -> 0.90 -> 0.93 -> 0.94 -> 0.95 -> 0.96 for cam0; cam1 set
  20% thinner at 0.968 per request) while checking real close-approach data stayed
  reliable — genuinely-close foot-y readings consistently reached ~0.99-1.00 on both
  cameras throughout, comfortably clearing both final thresholds. Flagged to user at
  each step that pushing past the observed 0.95-1.00 close-range cluster trades away
  reliability margin; user confirmed the direction each time.
- **Wired into production**: `main.cpp`'s launch string now passes `--near-field-y 0.96
  --near-field-y1 0.968` (was previously relying on the untested 0.85 code default —
  the earlier "on by default" note was misleading, since tuning had never actually
  reached `main.cpp`). Rebuilt clean.

## Recalibrated at 40cm baseline + NEAR-FIELD single-camera safety net added (2026-07-27)

**Recalibration:** user chose to remount the cameras closer (42.5cm -> 40cm) even after
the reflection/template-match fix resolved the specific bug that originally prompted it
— wanted the extra margin regardless. Redid the full `tools/stereo_distance_calib.py`
walk (1.0-4.0m). Best fit of the whole day: all 7 points used, no outliers dropped,
residuals -0.2cm@1m, -0.4cm@1.5m, +3.5cm@2m, +3.4cm@2.5m, **-4.9cm@3m (actual STOP
threshold)**, -10.1cm@3.5m, +5.6cm@4m (only 1 sample at 4m, treat with a bit more
caution than the rest). `main.cpp`: `stereo_a` 261.72->**252.53**, `stereo_b`
-17.13->**-21.43**, `--baseline-m` 0.425->**0.40**. Rebuilt clean.

**NEAR-FIELD trigger (new safety layer, not just a tuning change):** user identified a
real architectural gap — STOP/CAUTION only ever fired from a matched pair across BOTH
cameras. Someone close enough can fall completely out of ONE camera's field of view
(the two lenses are offset by the baseline), meaning no cross-camera match — and
therefore no distance — would ever exist for them, silently reporting "CLEAR - no
matching person" for a dangerously close pedestrian. Fixed with a SINGLE-camera check
independent of all stereo machinery: if a person's feet (box bottom) are in the bottom
`--near-field-y` fraction of EITHER camera's frame (default 0.85, i.e. bottom 15%),
force STOP immediately — no distance calc, no cross-camera match, no frame-skew-guard
dependency (that guard exists to protect disparity math specifically, which this check
doesn't use at all — computed from raw pre-skew-guard boxes so it can't be starved by
it). Ignores lateral position deliberately: the lane zone's own bottom edge is already
full-width, so anything this close to the bottom edge is already "in the path" by the
existing zone shape regardless. On by default, no launch-string flag needed. Two related
bugs caught and fixed while wiring this in: the per-camera `latest_stop`/`latest_caution`
gate was `fresh and people`, which would have silently swallowed a near-field-only STOP
(no matched `people` exists in that case, by definition) — changed to gate on `fresh`
alone. On-screen banner now shows a distinct "STOP - PERSON VERY CLOSE (near-field)"
message so it's visually obvious which trigger fired during testing. `py_compile` clean.
**Not yet live-tested** — needs a real close-approach test to confirm it fires correctly,
ideally including a case where the person is close enough to actually exit one camera's
FOV entirely (the scenario it exists for).

## Glass reflection caused a real bad-distance bug — fixed with template matching (2026-07-27)

After the edge-clip fix, user retested and found a NEW specific failure: standing
CENTERED at a tape-measured 1.8m (facing a glass door in the test room), the system read
2.5m+ — moving slightly left or right immediately corrected it. That symptom (wrong only
when dead-center, correct off-center) pointed away from a calibration/geometry problem
and toward a **duplicate detection**: the log for that test showed a spurious
`2 person(s)` even though only one real person was present — the glass reflection was
being detected as a second person. `stereo_match_all()` matches boxes between cameras by
left-to-right **x-position rank**, so with two candidates (real person + reflection) in
play, it can pair the real body in one camera with the reflection in the other,
computing disparity between two different physical things entirely.

**Fix: `--template-stereo-people`** (already existed in the script for static objects,
just not enabled for people) — correlates actual pixel patches between the two cameras
(`cv2.matchTemplate`) instead of box-position rank, so a real body and its mirrored/
lit-differently reflection don't falsely correlate. Retested live at the same 1.8m
position, moving deliberately left/right and centered: **zero duplicate-person
detections** in the whole retest log, readings stayed in a consistent ~1.6-2.7m band
(normal stereo jitter, no more catastrophic misreads). **Wired into production**
(`main.cpp`'s `ped_set_enabled()` launch string) and rebuilt clean.

## Lane-zone bottom edge widened to full frame width (2026-07-27, continued)

Live testing (screenshot) showed small uncovered triangles at the bottom corners of the
frame even after the earlier 0.30-0.70 -> 0.10-0.90 widening. `STOP_ZONE_PRESETS[2]`'s
bottom edge is now the full frame width (0.0-1.0 at y=1.0); far edge (0.40-0.60 at
y=0.45) unchanged, still confirmed fine at range.

## Close-range distance bug fixed: edge-clipped boxes read as FAR instead of close (2026-07-27)

Live-tested the widened ROI (see next section) standing very close (<1m, filling most of
the frame). User caught it live: at that range the distance reading did not reflect how
close they actually were — the opposite of safe. Root cause: a bounding box touching the
frame's left/right edge means the person's true extent is cut off. Box-**center**
disparity silently breaks here — both cameras' boxes clamp toward their own edge, so the
computed center-to-center disparity shrinks toward whatever the edge geometry happens to
be, not the person's real position. Fails in the dangerous direction: close reads as far,
which could suppress STOP/CAUTION exactly when it matters most.

**Real geometric fix, not a band-aid clamp** (`stereo_match_all()` in
`tools/pedestrian_detect.py`): `stereo_a`/`stereo_b` encode camera geometry (baseline,
focal length, principal-point offset) — nothing about the disparity reference having to
be the box *center* specifically. So when one side of a box is clipped, the still-valid
**opposite edge** (real, unclipped boundary of the person, present in both cameras) is
used as the disparity reference instead — same calibrated formula, correct anchor point.
Falls back to a monocular box-**height** distance estimate (varies with how large the
person appears, unlike a fixed constant) only for the genuinely ambiguous residual case:
a box clipped on both sides in one camera (spans the whole frame, no unclipped edge left
at all), or clipped on opposite sides between the two cameras (no shared reference
point). That fallback is capped at `--clip-safe-dist-m` (default 0.6m) — reaching this
branch already implies close range by construction, so "uncertain" must resolve to
"close", never "far". Compiles clean (`py_compile`). **Live-testing in progress** to
confirm the fix tracks correctly at close range.

## SAFETY: near-miss (cart touched a person) — CAUTION pre-slowdown added (2026-07-27)

User reported that in real testing the cart sometimes stopped right at/touching a person
instead of with clearance. Root cause investigation:
- `BRAKE,1` (`golf_cart_v28_smooth.ino` `updateDriveRamp()`) is a plain throttle-cutoff —
  `setDAC(0.0f)` — plus energizing `BRAKE_PIN_1/2`. **There is no measured/quantified
  active deceleration force** — user confirmed live it "looks like creep", i.e. the cart
  coasts down rather than braking hard.
- User confirmed real driving speed is ~3-5 km/h (0.83-1.39 m/s).
- Today's own `ctrl_log` CSV shows avg UDP round-trip = 241ms — real, measured comms
  latency between "STOP decided" and the ESP32 receiving the command, on top of whatever
  the coast-down itself takes.
- At the old 3.0m trigger distance, this reaction+coast budget plausibly consumes most or
  all of the margin — consistent with the reported near-touch.
- **The CSV logger still doesn't record anything once the cart is paused** (flagged
  earlier this session, still not fixed) — so real stopping distance has never actually
  been measured, only estimated. Flagged again as still open.

**Fix implemented: a CAUTION pre-slowdown, not just a bigger STOP number.** User's
explicit design call: slow down only when a person is actually detected (not just
whenever the pedestrian-detection feature is toggled on), but far enough out to give the
cart real lead distance to decelerate before it's close enough to need a full stop —
since braking distance scales with speed², slowing down early is a much stronger lever
than pushing the STOP trigger farther into less-reliable detection range.
- `tools/pedestrian_detect.py`: new **CAUTION** state — true when a person is in-lane at
  **any** distance (not gated by `--stop-distance`), separate from STOP (the existing
  in-lane + within-stop-distance trigger). `write_status()` now writes `STOP`/`CAUTION`/
  `CLEAR` (was `STOP`/`CLEAR`) to the status file. On-screen banner shows
  "CAUTION - PERSON IN LANE, SLOWING" (orange) as a third state; `--auto-hide-window`
  now also shows the camera window on CAUTION, not just STOP.
- `common.hpp`: new `PED_CAUTION_CRUISE_VOLTAGE = 0.85f` (same starting value as the
  already-field-tried `DANGER_ZONE_CRUISE_VOLTAGE` — untested at this specific value,
  tune from the first real test) + new `g_ped_caution` atomic.
- `main.cpp`: status-file poll parses the third state, sets `g_ped_caution` (true for
  CAUTION or STOP) alongside the existing `g_ped_detected` (true only for STOP, unchanged
  meaning — still the actual pause trigger).
- `control.cpp`: cruise-voltage selection — `g_ped_enabled && g_ped_caution` selects
  `PED_CAUTION_CRUISE_VOLTAGE`, overriding the zone-2 speed boost but still overridden by
  danger-zone dead-reckoning if both are somehow active. STOP (full pause) logic
  unchanged — still keyed on `g_ped_detected` only.
- Rebuilt clean (all files — `common.hpp` change touches everything).
- **NOT YET tested** — needs a real run to confirm the cart actually visibly slows on
  CAUTION before the STOP point. The stop-DISTANCE threshold itself (3.0m) was
  deliberately left unchanged this pass — user selected only the speed-reduction fix from
  a 3-option menu (raise stop-distance, fix CSV-logging-through-pause were the other two,
  both still open/available if the speed reduction alone isn't enough after testing).

## Distance calibration REFITTED with checkerboard-centroid method (2026-07-27, continued)

User reported the person-based distance readings "seemed inaccurate" during real testing.
After the full checkerboard intrinsics/stereoCalibrate route failed 4 times (see the
"Checkerboard calibration ATTEMPTED and NOT ADOPTED" section below for that history),
built `tools/stereo_distance_calib.py` — same tape-measured-distance empirical method as
the original person-based fit, but using the A3 checkerboard's corner **centroid** (mean
of all ~54 cornerSubPix corners) as the disparity reference instead of a YOLO person box.
Two real advantages: far less jitter than one box edge, and the centroid is immune to the
corner-ORDER ambiguity that broke the full intrinsics calibration attempts (averaging a
point set doesn't care what order they're summed in — no tilting needed either, board
just held flat/facing the camera at each distance).

Walked 1.0/1.5/2.0/2.5/3.0/3.5/4.0m (tape-measured, `tools/stereo_distance_calib.csv`).
The 2.5m sample was a **physically-impossible outlier** — its disparity (116.82px) was
*higher* than the 2.0m sample's (115.39px) despite being farther away, i.e. a real
stance/position error, not noise — excluded from the fit. Least-squares over the
remaining 6 points: residuals +1.5cm@1m, -4.3cm@1.5m, -2.5cm@2m, -6.5cm@3m, +21cm@3.5m,
+4.9cm@4m. The 3.5m residual is expected (disparity-based depth error amplifies with
distance for a fixed baseline) and 3.5m isn't the operating distance anyway — **residual
at the actual 3.0m STOP threshold (-6.5cm) matches the original walk-tested fit's
quality.** `main.cpp` updated: `stereo_a` 167.45→**261.72**, `stereo_b` 16.54→**-17.13**.
Rebuilt clean. **NOT YET live-walk-tested with a real person post-refit** — validate
before fully trusting, same as the original fit was validated the morning of 2026-07-27.

## USB bandwidth collapse found and fixed (2026-07-27, continued lab session)

User asked for a "more professional" review of the stereo setup + a performance check.
Measured live with `--fps-monitor`: each camera **alone** ran at its expected rate
(cam0=13.3fps, cam1=26.9fps — matches the FPS-mismatch already documented below), but
running **both together** collapsed to cam0=1.1fps / cam1=4.1fps. Confirmed compute
(GPU/YOLO) was NOT the cause — the collapse was identical with `--detect-every 1000`
(detection barely running at all). Root cause: two raw-format (YUY2) 640x480 streams
saturating the shared USB bus/controller — a well-known consumer-webcam limitation, not
a code bug. **Fix:** force MJPG (camera-side compressed, far fewer bytes/frame) via a new
`--fourcc` flag (default `"MJPG"`, set on `cap` before width/height in `CameraWorker.__init__`,
`tools/pedestrian_detect.py`). Verified with the real threaded pipeline + YOLO running:
both cameras now sustain their full **solo** ceiling running together (cam0≈13.3fps,
cam1≈25-26fps) — the contention is completely gone. No `main.cpp` change needed (the
default already applies to the production launch string). Pass `--fourcc ""` to go back
to the old raw-format behavior if ever needed for comparison.

**Calibration model assessment (same session):** the existing
`distance_m = stereo_a/(disparity_px - stereo_b)` formula is not an arbitrary curve fit —
structurally it IS the real stereo depth equation (`baseline_m * f_px / disparity_px`,
with the `-B` term corresponding to the principal-point offset between the two cameras).
The uneven residuals (-17cm@2m, +47cm@4m from the 4-point fit below) come from `f_px`/`B`
being *estimated* from only 4 noisy tape-measured points instead of *measured* directly —
a proper OpenCV checkerboard calibration (`cv2.calibrateCamera` per camera, standard,
no new installs needed) would fix the off-axis error at its source (currently just
gated around via the lane-zone) instead of curve-fitting around it.

**Checkerboard calibration ATTEMPTED and NOT ADOPTED (same session).** Built
`tools/gen_checkerboard.py` (printable/phone-displayable 9x6-internal-corner pattern,
7mm squares as displayed on the test phone) + `tools/stereo_calibrate.py` (hands-free
auto-capture on dual clean detection, same beep-cued philosophy as `--auto-calib`, then
real `cv2.calibrateCamera`/`stereoCalibrate`/`stereoRectify`). Two 20-pose capture runs:
- **Run 1** (handheld phone): reprojection RMS 3.5px (cam0) / 4.1px (cam1) / 5.2px
  (stereo) — target is <1px. Calibrated baseline 40.0cm vs physical 42.5cm (close-ish).
- **Run 2** (phone braced against a rigid surface, slower pace): RMS still high
  (4.2/4.1/5.0px) AND **diverged sharply from Run 1** — rectified focal length
  335px -> 1772px, baseline 40.0cm -> 31.2cm, cam0 fx/fy badly mismatched (219 vs 152,
  should be close for a normal sensor). Large disagreement between independent runs
  (not just noise) is the signature of **insufficient tilt diversity** in the captured
  poses — Zhang's calibration method needs the board significantly TILTED across poses,
  not just moved/translated fronto-parallel, to numerically separate focal length from
  scene depth. Bracing removed hand-shake but didn't fix this — the poses were still
  likely too close to fronto-parallel.
- **Decision (2026-07-27, user call):** stop here, keep the field-validated empirical
  `stereo_a=167.45`/`stereo_b=16.54` fit rather than chase a third capture attempt —
  reasonable given two divergent results and limited on-site time with tasks 3/4 still
  pending. `tools/stereo_calib.npz` was saved from Run 2 but is **NOT wired into
  `pedestrian_detect.py`** (no `--calib-file` flag exists; the file is inert output only).
  If revisited later: the fix is a **structured, explicit tilt sequence** (flat -> tilt
  left 45° -> right 45° -> up 45° -> down 45°, at 2+ distances) rather than open-ended
  "move it around", which is hard to self-enforce in real time while also keeping the
  small board in both cameras' frames at once.
- **Performance fix from the same session stands independently and IS in production
  use** — see the USB/MJPG section above; unaffected by this decision.

## Lane-zone bug found and fixed (2026-07-27, in-lab live app test)

Live app testing found the cart STOPping for a person ~4m away and off to the side —
not actually in its driving path. `--no-zone` (added 2026-07-19) had disabled ALL
lateral filtering, so any detected person anywhere in frame counted. Fixed: `main.cpp`
now uses `--zone 2` (`STOP_ZONE_PRESETS[2]`, the "narrow central lane" trapezoid)
instead of `--no-zone`.

**That fix immediately exposed a real, pre-existing bug**: with `--zone 2` active, STOP
never fired at all, even for someone standing centered at 0.4m. Root cause in
`run_stereo_live()`: it normalized detection coordinates by `args.width`/`args.height`
(the requested 640x480 capture resolution) instead of the camera's actual captured
frame size. `cv2.VideoCapture.set(CAP_PROP_FRAME_WIDTH/HEIGHT)` is only a request — these
cameras don't honor it — so `foot_x/args.width` and `foot_y/args.height` always landed
outside 0..1, and `point_in_poly()` returned `False` for every detection, unconditionally.
**This bug has existed since the zone feature was written; it was invisible because
`--no-zone` bypassed it in every real launch since 2026-07-19.** Fixed by tracking each
`CameraWorker`'s actual per-frame size (`self.frame_w/frame_h`, set from `frame.shape`
each capture) and using that instead of the args values. Also changed the zone ROI
overlay color from yellow to white (user request).

**Verified fixed** with a live standalone walk test: off-center person stayed CLEAR
down to 1.13m; in-lane/centered person correctly triggered STOP at 1.67m/1.46m and
CLEAR at 2.08m+. Both the lateral zone filter and the 2.0m distance gate now work
together correctly.

> ## ⚠ STATUS: CALIBRATED + LIVE-WALK-TESTED ON-CART 2026-07-27 — AUTO-RESUME TIMING STILL UNVALIDATED
> The multi-person tracking, stereo math, and full C++ integration described below were
> originally built and tested only indoors on a temporary hand-held rig (max 2.8m) — that
> part is history, not current state. **On 2026-07-27 the cameras were rigidly mounted on
> the actual cart (42.5cm baseline) and recalibrated on-site** (see "On-cart recalibration"
> below) — `main.cpp` now carries the fitted `stereo_a=167.45`/`stereo_b=16.54`. The
> STOP/CLEAR decision was live-walk-tested the same day at a 3.0m threshold (every
> transition landed on the correct side of 3m, see calibration history) before the
> threshold was set back to **`--stop-distance 2.0`** (final, user request) — the 2m
> operating point itself was NOT re-walk-tested after the revert, and its fit residual
> (-17cm) is looser than 3m's was (+5cm), so treat the exact 2m boundary with a bit more
> caution than the validated 3m one. Camera indices `--cam0 0 --cam1 1` are confirmed
> correct on the real hardware. **What's still unconfirmed:** the 3-second auto-resume
> debounce (`control.cpp`, raised from 1s same day) — needs the full driving stack to
> observe end-to-end, not yet done.

Python detector (`tools/pedestrian_detect.py`) — **wired into the C++ control stack as of
2026-07-19** (see Integration section below), **but unconfirmed end-to-end — see status
banner above**. Two-camera (60cm baseline) USB rig, tested on the laptop that will run the
cart (GTX 1650 GPU confirmed). This file documents what was built, what was tried and
rejected, and what's still open — read this before touching the script or the integration
again.

## Integration (2026-07-19)

Replaces the old single-camera monocular pedestrian logic entirely. Two explicit
requirements from this integration pass:
1. **STOP at any distance <= 2m** — `--stop-distance 2.0` in the launch command.
2. **Auto-resume** when the person/obstacle clears — previously required a manual
   "Resume" button press every time; now resumes on its own.

**Files changed:**
- **`main.cpp` `ped_set_enabled()`** — launch command rewritten: `--cam0 0 --cam1 1`
  (was single-camera `--cam1 -1`), `--backend yolo`, `--detect-every 1`, `--stereo-live`
  (**required** — this is what actually computes the stop decision written to the status
  file; without it the status silently stays CLEAR forever), `--stereo-a 240.72
  --stereo-b 4.78` (YOLO-backend calibration constants), `--stop-distance 2.0`,
  `--no-zone` (the stop-lane trapezoid — a separate, pre-existing lateral filter,
  unrelated to today's stereo rewrite — was still active by default and would have
  ignored a close person standing just outside it; removed so STOP fires at any
  distance <=2m regardless of lateral position, matching the literal requirement).
- **`control.cpp`** — added an auto-resume block right after the existing pedestrian-pause
  trigger: while `CS_PAUSED` with `g_ped_paused` true, once the trigger condition
  (`camPedStop || gridPedStop`) has been clear for **50 consecutive ticks (~1s @ 50Hz)**,
  calls `ctrl_doStart()` — the exact same function the manual Resume button already used,
  so all the existing CS_PAUSED-resume special-casing (danger-zone state preservation etc.)
  is reused unchanged. The 1s debounce is a second, independent smoothing layer on top of
  the Python side's own per-person median filter — prevents rapid stop/go if someone
  lingers right at the 2m boundary. New `CtrlVars.pedResumeConfirmCount` field, reset
  automatically inside `ctrl_doStart()` alongside the rest of `CtrlVars`.
- **`common.hpp`** — updated the stale "will NOT auto-resume" comment; added the
  `pedResumeConfirmCount` field.
- **`tools/pedestrian_detect.py`** — updated the module docstring (was "NOT wired into the
  C++ control stack yet").
- The manual Resume button (`ui.cpp`) is **left in place as a fallback/override** — it
  still works if auto-resume is ever disabled or something goes wrong, it just rarely
  needs pressing now.

**Verified:** `write_status()` in the Python script already correctly reflects the new
multi-person stereo logic (`any_stop = OR of both cameras' get_stop()`, which read the
shared per-person `stereo_stop` — confirmed no Python-side change was needed). Full C++
rebuild completed clean (zero errors/warnings) after all changes.

**Not yet done / explicitly out of scope for this pass:**
- Not tested on the actual cart hardware/cameras — `--cam0 0 --cam1 1` assumes the same
  USB enumeration order as the dev machine; verify on the real cart PC.
- Cold-start warm-up (~10s to full detection speed) still unfixed — see main findings below.
- Camera mount rigidity + accurate multi-point recalibration still pending (lab session).

### Isolation from the main control system (2026-07-19)

Explicit requirement: the detector must never lag or interrupt the main app, especially
the 50Hz control loop. What was verified/already correct, and the one thing fixed:

- **Already correct (pre-existing design, confirmed by code read, not changed):** the
  detector runs as a separate OS **process** (not a thread) — a Python crash/hang can't
  corrupt C++ memory. The status-file read happens in the 60fps **render loop**, throttled
  to 10Hz, explicitly NOT in the 50Hz control thread (there's a pre-existing comment saying
  exactly this). Cross-thread communication is lock-free atomics only
  (`g_ped_detected` etc.) — no mutex shared with the control loop for this feature. The
  status file write is atomic (temp file + rename) so the reader can never see a torn write.
- **Fixed this pass:** the detector process had no priority hint — under CPU contention it
  could compete equally with the control loop's thread for scheduling. Now launched with
  `BELOW_NORMAL_PRIORITY_CLASS` (`main.cpp`) so it can never starve the main app. Not
  `IDLE_PRIORITY_CLASS` deliberately — that risks the detector itself stalling under
  routine background load and silently going stale (fails open to CLEAR after 2s, so an
  over-throttled detector is a silent loss of pedestrian protection, not a crash).
- **NOT fixed, cannot be from a code-only change — needs real hardware verification:**
  GPU contention. Both processes share the ONE physical GPU — Raylib/OpenGL rendering in
  the C++ app, YOLO/CUDA inference in the detector. Windows WDDM time-slices the GPU
  between processes, but a sustained ~26 FPS/camera CUDA workload could still cause
  render-loop stutter in the C++ UI. This should NOT reach the control thread itself
  (separate thread, no GPU work in the control loop, localization is CPU-based
  distance-transform matching — not GPU-dependent), but it hasn't been measured on real
  hardware. **If UI stutter is observed once both run together:** the lever is
  `--detect-every 2` (or higher) in the `main.cpp` launch command — halves the GPU
  inference load at the cost of detection latency; still very responsive for a slow-moving
  cart at 13 FPS. Not applied preemptively since it wasn't a measured problem.

## Current validated state

- **Backend: YOLOv8n on GPU** (`--backend yolo`) — ~26 FPS per camera, both cameras
  simultaneously, unbatched (two separate model instances). MobileNet-SSD (CPU) still works
  as the zero-dependency fallback/default but is less accurate and was mostly superseded.
- **Pedestrian distance: stereo (cross-camera disparity), NOT monocular.** The monocular
  box-height method (assumes a fixed 1.7m person height) was proven unreliable — up to ~43%
  error — and is no longer used for any live decision. It's fully removed from the on-screen
  banner/box color; the underlying math is only kept to feed the optional `--log-file`
  calibration logger.
- **Stereo formula:** `distance_m = stereo_a / (disparity_px - stereo_b)`. This is the real
  geometric stereo relationship (baseline × focal / disparity) with a small constant offset
  absorbing a bounding-box-centering bias — fitted from tape-measured 1.0/2.0/2.8m ground
  truth, residuals < 4cm across that range (YOLO backend: `stereo-a=240.72`, `stereo-b=4.78`).
- **Multi-person support:** every detected person gets matched and given their own distance
  simultaneously (not just the closest one). Matching is by **left-to-right x-position rank**
  between the two cameras — proven far more reliable than confidence-based or height/y-based
  matching (see Rejected Approaches below).
- **Per-person tracking + median filtering:** each person is tracked frame-to-frame
  (nearest-position association) and independently median-filtered (5-sample rolling window,
  min 2 samples before reporting) — rejects single-frame outliers/occlusion spikes without
  much lag, and one person's bad frame can't corrupt another's reading.
- **Outlier rejection:** any stereo reading beyond `--stereo-max-dist` (default 6m) is
  discarded outright, not clamped — treated exactly like "no match." Necessary because
  `A/(disparity-B)` is numerically unstable as disparity→B; a tiny disparity error there
  can explode into a physically-impossible distance (confirmed: 74m/88m spikes in a 2.8m room).
- **Mirror/reflection immunity:** a person's reflection visible in only ONE camera can never
  produce a stereo match (needs agreement from both), so it's automatically ignored — no
  special-case code needed, it's a natural consequence of the stereo requirement.
- **Duplicate-box merge:** `merge_duplicate_boxes()` collapses same-object nested/partial
  detections (e.g. a chair detected as both "whole chair" and "just the backrest") using
  overlap-over-smaller-box-area, not standard IoU (which fails when the two boxes are very
  different sizes — union-based IoU stays low even when one is contained in the other).

## What did NOT work — read before re-attempting

### Static-object (chair) distance — abandoned for now
Tried to extend the same stereo pipeline to "chair" as a stand-in static-object class
(`--detect-static`). Two independent stereo methods were tried and **both failed**:

1. **Box-center disparity** (same method that's accurate for people): wrong by 2-3x for a
   chair, because chairs are wide/asymmetric — each camera frames a different extent at a
   different viewing angle, so the box centers don't track the same physical point. A person
   works because they're front-facing and roughly symmetric; a chair does not.
2. **Template-matching disparity** (`--template-stereo`, matches actual pixels via
   `cv2.matchTemplate` instead of box centers) — built specifically to fix #1, and *still*
   failed. Root cause found: the test chair had a semi-transparent mesh backrest, so each
   camera sees *different background* through it — worst case for pixel matching. Also
   struggles with duplicate/uncertain detections and edge-of-frame cropping.

**Conclusion (documented decision, not just an in-progress note):** camera-based stereo
distance is reliable for people, fundamentally unreliable for arbitrary static objects
(wide/asymmetric shapes, transparency, reflections all break it). **Recommendation: use
LiDAR for static-obstacle geometry** (already on the cart, class-agnostic, no box-centering
assumption) and keep the camera focused on pedestrians, where it's proven strong. Chair
detection code (`--detect-static`, `template_match_disparity()`) is left in the script,
default OFF, in case a well-behaved (opaque, roughly symmetric) object needs it later — but
it is NOT part of the validated/trusted path.

### Cross-camera person matching by height + y-position — reverted
First multi-person implementation matched people between cameras by comparing apparent box
height and vertical center (reasoning: viewpoint-invariant, unlike x-position which changes
by design). **This was wrong.** Confirmed via screenshot: two similarly-tall, similarly-
positioned people got cross-matched (beige-shirt person paired with blue-shirt person's box
in the other camera), causing each camera to report *opposite* distances for the same two
people. Reverted to x-position rank matching (same function `stereo_match_all()`, reused —
the earlier single/closest-match code was already correct, no need for a separate matcher).

### Confidence-based ("highest confidence in each camera") matching — reverted earlier
Original single-person stereo picked the highest-confidence box independently in each
camera. Broke as soon as 2 people were in frame: confidence varies with per-camera
lighting/angle, so each camera could pick a *different* physical person as "the" match,
corrupting the cross-camera disparity. Replaced by x-position rank matching.

## On-cart recalibration — DONE 2026-07-27

**Resolved.** Cameras remounted rigid at a 42.5cm baseline (was the temporary 60cm hand-held
indoor rig described below, now stale). Multi-point calibration redone directly on the cart
in the lab: 1/2/3/4m, each with a tape-measured mark (not paced/eyeballed — the first two
attempts at the 1m point were off by 35-70cm before switching to an actual tape from the
lens plane, see `tools/stereo_calib_20260727.csv` for the full raw log including the
rejected earlier attempts). Least-squares fit gave `stereo_a=167.45`, `stereo_b=16.54`
(`main.cpp`'s `ped_set_enabled()` updated to match, rebuilt clean). Residuals were uneven
across the 4 points (+1.5cm@1m, -17cm@2m, +5cm@3m, +47cm@4m) — only 4 points were taken, and
2m/4m didn't fit the same curve as cleanly as 1m/3m did. 5m/6m points were skipped (user
judged 1-4m sufficient).

**Live-walk-tested same day at `--stop-distance 3.0`** (sat right at the well-fit 3m
sample, +5cm residual) — every STOP/CLEAR transition in the walk log landed on the correct
side of 3m, no bias either direction. **Then reverted to `--stop-distance 2.0`** (final,
user request) — this is the looser-fit point (-17cm residual) and was NOT independently
re-walk-tested at 2m after the revert; if 2m accuracy ever seems off in practice, that's
why. The far end (4m+) should not be trusted for precise distance either way, only rough
outlier rejection (`--stereo-max-dist`). If the stop distance changes again, or better
accuracy at whatever the new operating point is matters, redo the calibration with more
points spanning that range.

**Still not yet validated:** the 3-second pedestrian auto-resume debounce (raised from 1s
same day, `control.cpp`) — needs the full driving stack running to observe end-to-end,
not yet done.

<details>
<summary>Original finding (2026-07-17 to 07-19, superseded by the above)</summary>

**Camera mount is not rigid.** A camera got bumped mid-session; a true 2.1m chair distance
read as 4.7-5.4m and a true 2m person distance read as 3.7m, both traced to the physical
shift (confirmed once the camera was manually repositioned and readings recovered close to
~1.6m, though not exactly — the manual "put it back" wasn't pixel-perfect). The whole stereo
calibration assumes a fixed, measured baseline with both cameras level and parallel — any
drift silently invalidates `stereo-a`/`stereo-b` with no way for the system to detect it.

The `stereo-a=240.72`/`stereo-b=4.78` constants that were hardcoded in `main.cpp`'s launch
command were placeholders from this temporary, since-disturbed setup — not a trustworthy
final calibration. They were good enough to prove the pipeline works, not accurate enough
to deploy as-is. (Now superseded — see above.)

</details>

## Cold-start / warm-up latency (not yet fixed, but understood)

Measured: ~4s to load both YOLO models + open both cameras, then a further **~6 seconds of
~1 FPS** while CUDA/cuDNN does first-call kernel autotuning, before settling into steady
~26 FPS. Total ~10s from launch to reliable detection. **Not yet fixed** — the planned fix
(run a few throwaway inference calls immediately after model load, before relying on the
detector for anything real) was explicitly deferred to the integration phase, not done in
this session.

## Reliability / integration notes for later

- Fail-open by design: the C++ side already treats a stale `pedestrian_status.txt` (>2s) as
  `CLEAR` so a frozen/crashed detector can't strand the cart — but this means a GPU
  hang/crash silently removes pedestrian protection rather than blocking the drive. Accepted
  tradeoff already in the existing design; worth remembering when reasoning about failure modes.
- Two separate YOLO model instances (one per camera thread) is simple and already meets the
  FPS requirement (~26 FPS each, both simultaneously) — batching into one shared model +
  one call per cycle would be a legitimate efficiency improvement but was not necessary and
  not implemented.
- Detection is genuinely GPU-viable on this hardware for the real deployment PC (GTX 1650
  confirmed to be the same machine that will run on the cart, not a separate weaker unit).

## Key script flags (current)

| Flag | Default | Purpose |
|---|---|---|
| `--backend` | `mobilenet` | `mobilenet` (CPU, zero deps) or `yolo` (GPU, more accurate) |
| `--cam0` / `--cam1` | `0` / `1` | camera indices (`-1` disables one) |
| `--conf` | `0.45` | person detection confidence floor |
| `--detect-static` | off | also detect chairs as a static-object stand-in (visual only, see above) |
| `--static-conf` | `0.25` | lower confidence floor for chairs (COCO training skews toward "person") |
| `--nms-iou` | `0.45` | YOLO duplicate-box suppression threshold |
| `--no-zone` | off | hide the stop-lane trapezoid, STOP becomes distance-only (used for today's testing) |
| `--stereo-live` | off | print + drive on-screen color from calibrated stereo distance (the trustworthy path) |
| `--stereo-a` / `--stereo-b` | `240.72` / `4.78` (YOLO) | calibration constants — **backend-specific**, MobileNet has its own (`199.86`/`41.67`) |
| `--stereo-max-dist` | `6.0` | reject any reading beyond this (outlier rejection) |
| `--stop-distance` | `6.0` | STOP trigger threshold, metres |
| `--template-stereo` / `--template-stereo-people` | off | pixel-matching disparity instead of box-center (chairs: tried, failed; people: not needed, already accurate) |
| `--auto-calib` | off | hands-free beep-timed calibration sequence (walk to marked distances) |
| `--fps-monitor <seconds>` | `0` | pure throughput benchmark, no person needed |

Full flag list and docstrings are in the script itself — this table covers what's relevant
to picking up where this session left off, not everything.

## Calibration history

| Date | Backend | Points | Result |
|---|---|---|---|
| 2026-07-17 | MobileNet | 1.0/2.0/2.8m | `stereo-a=199.86`, `stereo-b=41.67`, residuals <4cm (indoor, temporary rig, superseded) |
| 2026-07-17 | YOLO | 1.0/2.0/2.8m | `stereo-a=240.72`, `stereo-b=4.78`, residuals <4cm (indoor, temporary rig, superseded) |
| 2026-07-27 | YOLO | 1/2/3/4m, on-cart, 42.5cm baseline, rigid mount | `stereo_a=167.45`, `stereo_b=16.54`, residuals +1.5cm/-17cm/+5cm/+47cm (uneven, see note above) — **current live values in `main.cpp`** |

## Next steps (in priority order, per user)

1. ~~Rigid camera mount + accurate multi-point recalibration~~ — **done 2026-07-27** on-cart,
   42.5cm baseline, 1-4m (5m/6m skipped, judged sufficient).
2. ~~Live STOP/CLEAR + distance-accuracy validation with a real walking person~~ — **done
   2026-07-27** at the 3.0m threshold (all transitions correct). Final `--stop-distance`
   reverted to 2.0 afterward — not independently re-walked at 2m, see status banner.
3. 3-second pedestrian auto-resume debounce — needs the full driving stack running to
   validate end-to-end, not yet done.
4. Cold-start warm-up fix (throwaway inference calls at launch) — deferred, still open.
5. ~~Wire validated pedestrian stereo distance into the real `pedestrian_status.txt` /~~
   ~~C++ control-loop pause logic~~ — **done 2026-07-19, see Integration section above.**
   Camera indices (`--cam0 0 --cam1 1`) confirmed correct on the actual cart 2026-07-27.
6. Static-obstacle detection: pursue via LiDAR, not camera (per the conclusion above).
