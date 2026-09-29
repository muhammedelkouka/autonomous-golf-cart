// =============================================================================
// main.cpp  —  System init, thread startup, main Raylib loop
// Only heavy logic allowed: thread spawning and UI event dispatch.
// All subsystem logic lives in its own .cpp file.
// =============================================================================
#include "common.hpp"

// ── Global that is only needed in main.cpp ──
std::atomic<int> g_ui_step{STEP_LOCALIZE};
std::atomic<bool> g_show_range_rings{true};  // [G] toggles Step 1 range rings/crosshair

// ── Intro/destination panel state ──
bool g_show_welcome = false;
bool g_show_intro   = false;
bool g_show_dest    = false;
bool g_show_localize        = false;
bool g_show_map_select      = false;
bool g_show_journey_end     = false;   // "Another journey?" card, shown on arrival (passenger flow only)
// True only when the current drive was started via the passenger flow (the
// Destination panel's "Begin Journey" button) — set false at every technical Step
// 1/2/3 drive-start entry point instead. Lets a danger-zone/sub-zone exit return to
// wherever the operator actually came from, instead of always forcing the passenger
// Localize panel (which is jarring when testing directly from Step 1/2/3, as if the
// passenger flow was never used at all).
bool g_drive_via_passenger  = false;

// ── Danger-zone CROSSING journey (2026-07-07) ──────────────────────────────────
// Destinations beyond the danger-zone stretch (Basketball Court, Food Court — the two
// new zone-2 destinations) are reached via a 3-leg journey on TWO different maps:
//   1. Campus Top LiDAR: drive to "Building A" (checkpoint) — normal zone-1 routing.
//   2. Switch to Danger Zone Fwd, relocalize, drive its own recorded route end-to-end
//      (that map's own g_dzsub_active sub-zone mechanism handles the hard stretch,
//      completely unchanged — this journey layer never touches it).
//   3. Switch back to Campus Top LiDAR, relocalize, drive to the real final destination
//      — normal zone-2 routing, exactly as if picked directly.
// Each map-switch happens only while stopped (CS_PAUSED) — see the goal-reached
// branch in control.cpp and the orchestration in the main loop below. The operator
// still presses Start Autonomous at each leg, same as every other stop in this
// codebase — no auto-continue.
// (DzxState enum is declared in common.hpp, alongside the rest of these externs.)
DzxState g_dzx_state = DZX_NONE;
float g_dzx_final_x=0.f, g_dzx_final_y=0.f;
std::string g_dzx_final_name;
std::string g_journey_dest_name;    // picked-destination label for the friendly panel pin
bool g_dzx_pending_enter = false;   // control.cpp -> main.cpp: perform the entry map-switch now
bool g_dzx_pending_exit  = false;   // control.cpp -> main.cpp: perform the exit map-switch now

// ── Camera pedestrian detection (2026-07-08, test feature) — see common.hpp ──
std::atomic<bool> g_ped_enabled{false};
std::atomic<bool> g_ped_detected{false};
std::atomic<bool> g_ped_caution{false};
std::atomic<bool> g_ped_paused{false};
std::atomic<bool> g_ped_stale{false};
std::string PED_STATUS_FILE = "pedestrian_status.txt";  // in runtime_data/ (cwd at run time)
static PROCESS_INFORMATION g_ped_proc = {};
static bool g_ped_proc_running = false;
// Launch (on=true) or terminate (on=false) the standalone Python detector, which
// publishes STOP/CLEAR to PED_STATUS_FILE. Launching is best-effort: if the process
// can't start (Python missing, etc.) the toggle still flips on and the operator can
// run the script by hand — the file-based link works either way.
void ped_set_enabled(bool on) {
    if(on){
        if(!g_ped_proc_running){
            // tools/pedestrian_detect.py lives one level up from runtime_data/ (cwd).
            // 2026-07-19 rewrite: TWO cameras (0/1) + YOLOv8n/GPU + calibrated
            // cross-camera stereo distance — replaces the old single-camera monocular
            // estimate (proven unreliable, up to ~43% error). --stereo-live is REQUIRED
            // here, not optional: it's what actually computes/writes the per-person
            // stereo distance that drives the status file; without it every reading
            // silently stays CLEAR. --cam0/--cam1 indices confirmed correct on the
            // actual cart PC 2026-07-27 (both cameras produced a sane, monotonic
            // disparity-vs-distance trend during calibration).
            //
            // 2026-07-27 on-cart recalibration: camera mount was remounted rigid at a
            // 42.5cm baseline (was a temporary 60cm hand-held rig, now stale/invalid).
            // --stereo-a/-b originally fitted from 1/2/3/4m tape-measured PERSON ground
            // truth (see tools/PEDESTRIAN_DETECTION.md calibration history +
            // tools/stereo_calib_20260727.csv). Residuals were uneven (+1.5cm@1m,
            // -17cm@2m, +5cm@3m, +47cm@4m) — only 4 points, and a YOLO person box is a
            // noisier disparity reference than a rigid target.
            //
            // REFITTED same day (continued lab session) using tools/stereo_distance_calib.py
            // — same tape-measured-distance method, but disparity taken from the A3
            // checkerboard's corner CENTROID (mean of all ~54 cornerSubPix corners)
            // instead of a person's bounding box: far less jitter, and immune to the
            // corner-ordering issue that separately broke a full intrinsics/stereoCalibrate
            // attempt (see tools/PEDESTRIAN_DETECTION.md — that approach was abandoned).
            // 7 distances walked (1.0-4.0m); the 2.5m sample was a physically-impossible
            // outlier (disparity computed HIGHER than the 2.0m sample despite being
            // farther away — a real stance/position error, not noise) and was excluded.
            // Least-squares fit over the remaining 6 points: residuals +1.5cm@1m,
            // -4.3cm@1.5m, -2.5cm@2m, -6.5cm@3m, +21cm@3.5m, +4.9cm@4m — the 3.5m outlier
            // is expected (disparity error amplifies with distance for a fixed baseline,
            // not a data problem) and 3.5m isn't the operating point. **Residual at the
            // actual 3.0m STOP threshold (-6.5cm) is as good as the original walk-tested
            // fit.** New values: stereo_a 167.45->261.72, stereo_b 16.54->-17.13.
            // NOT YET live-walk-tested with a real person post-refit — do that before
            // fully trusting it, same as the original fit was validated this morning.
            // If the camera mount is ever changed again, this MUST be recalibrated —
            // see tools/PEDESTRIAN_DETECTION.md.
            //
            // --stop-distance 3.0 (2026-07-27, user request — history same day: 2.0 ->
            // 3.0 (live-walk-tested clean, every transition correct) -> 2.0 -> 2.5 ->
            // 3.0 final. This lands back on the point with the best calibration fit
            // (+5cm residual, tightest of the four calibrated points) AND the one
            // that was actually live-walk-tested earlier today — see calibration
            // history in tools/PEDESTRIAN_DETECTION.md) is the cart's stop trigger
            // (<=3.0m, any matched person IN the lane zone below).
            //
            // 2026-07-27 lane-zone re-enabled (removed --no-zone, added --zone 2):
            // live testing found off-center people (standing near the edge of the
            // frame, not in the cart's actual driving path) got wildly inaccurate
            // stereo distance — box-center disparity + visible wide-angle lens
            // distortion don't hold up off-axis the way they do for a centered person
            // (the calibration itself was done with the subject centered). A person
            // ~4m away off to the side read as 2.4m and would've wrongly triggered
            // STOP. Root fix is lens undistortion before disparity (not done — bigger
            // task). Practical fix: only evaluate distance for people actually walking
            // in the cart's path — --zone 2 is the "narrow central lane" preset
            // (tools/pedestrian_detect.py STOP_ZONE_PRESETS, x=0.30-0.70 at the bottom
            // of frame narrowing to 0.40-0.60 mid-frame), so someone off to the side is
            // excluded from the STOP decision regardless of their (unreliable) distance
            // reading. Someone walking straight toward the cart stays centered in-frame
            // at any distance, so this doesn't weaken protection on the actual path.
            //
            // --auto-hide-window (2026-07-27, user request): camera windows stay hidden
            // during normal silent monitoring, auto-show only when a STOP fires, hide
            // again 3s after it clears (default --window-hide-delay). Saves per-frame
            // drawing + cv2.imshow() compositor cost while idle; does NOT reduce YOLO
            // inference load (still runs continuously — that's what detects the STOP).
            //
            // --template-stereo-people (2026-07-27, real bug found live): standing
            // CENTERED at a tape-measured 1.8m in front of a glass door read as 2.5m+ —
            // moving slightly left/right fixed it immediately. Root cause: the glass
            // reflection was getting detected as a second "person", and the rank-by-x-
            // position box matching paired the real body in one camera with the
            // REFLECTION in the other, computing disparity between two different things.
            // Confirmed live: with plain box-center matching the log showed a spurious
            // "2 person(s)" (only one real person present); switching to template-match
            // disparity (correlates actual pixel patches between cameras instead of box
            // rank) eliminated the ghost detection entirely and the reading stayed
            // consistent (~1.6-2.7m jitter, normal stereo noise) throughout a live
            // centered + side-to-side retest at 1.8m. See tools/PEDESTRIAN_DETECTION.md.
            //
            // 2026-07-27 REMOUNTED to 40cm baseline (was 42.5cm) + recalibrated with
            // tools/stereo_distance_calib.py (checkerboard-centroid method, same as the
            // 42.5cm fit above) — user's call, made after the reflection/template-match
            // fix already resolved the specific bug that prompted it. Best fit of the
            // day: all 7 walked distances (1.0-4.0m) used with NO outliers dropped,
            // residuals -0.2cm@1m, -0.4cm@1.5m, +3.5cm@2m, +3.4cm@2.5m, -4.9cm@3m (the
            // actual STOP threshold), -10.1cm@3.5m, +5.6cm@4m (only 1 sample at 4m —
            // board was likely too small/far for consistently reliable detection during
            // the hold, so treat that single point with a bit more caution than the
            // others, though it still fits the trend well). stereo_a 261.72->252.53,
            // stereo_b -17.13->-21.43, baseline-m 0.425->0.40.
            //
            // NEAR-FIELD trigger (2026-07-27, user request): tools/pedestrian_detect.py
            // also force-STOPs from a SINGLE camera alone (no cross-camera match or
            // distance calc required) if a person's feet are far enough down either
            // camera's frame. Closes a real gap: someone close enough can fall
            // completely out of ONE camera's FOV (baseline offset between the lenses),
            // so no matched pair — and thus no distance — would otherwise ever exist for
            // them. Thresholds TUNED live (not the 0.85 code default) — the two cameras
            // read differently for the same real position (cam1 consistently ~3-4
            // points higher than cam0), confirmed from live foot-y logging: cam0
            // --near-field-y 0.96, cam1 --near-field-y1 0.968 (~20% thinner trigger band
            // than cam0's, by request). Both validated live: real close-approach tests
            // reliably reached foot-y ~0.99-1.00 on both cameras, comfortably past
            // either threshold. See PEDESTRIAN_DETECTION.md.
            //
            // --detect-every 2 (2026-07-27, user request — laptop getting warm over a
            // sustained run): YOLO/CUDA now shares the GPU with Raylib rendering
            // (already flagged as unmeasured risk earlier this session); running
            // inference on every single captured frame from BOTH cameras continuously
            // is real sustained GPU load, the likely main heat source. detect-every 2
            // halves that load — detection still runs ~7-13x/sec (was ~13-27x/sec),
            // far above what the established stopping-distance margins need; frames
            // between detections just redraw the last known box, so display stays
            // smooth either way.
            std::string cmd = "\"C:\\Users\\bios\\AppData\\Local\\Programs\\Python\\Python38\\python.exe\" "
                              "\"..\\tools\\pedestrian_detect.py\" --cam0 0 --cam1 1 "
                              "--backend yolo --detect-every 2 --zone 2 --stereo-live "
                              "--template-stereo-people "
                              "--baseline-m 0.40 --stereo-a 252.53 --stereo-b -21.43 "
                              "--stop-distance 3.0 --near-field-y 0.96 --near-field-y1 0.968 "
                              "--auto-hide-window "
                              "--status-file \"" + PED_STATUS_FILE + "\"";
            STARTUPINFOA si = {}; si.cb = sizeof(si);
            std::vector<char> mut(cmd.begin(), cmd.end()); mut.push_back('\0');
            // BELOW_NORMAL_PRIORITY_CLASS (2026-07-19): the detector must never be able
            // to starve the main app's threads — especially the 50Hz control loop —
            // under CPU contention. Deliberately not IDLE: that could make the detector
            // itself stall/go stale under routine background load, silently dropping
            // pedestrian protection (staleness fails open to CLEAR after 2s). This is
            // a floor under normal load, not a hard guarantee under heavy load — see
            // tools/PEDESTRIAN_DETECTION.md for the GPU-contention caveat, which this
            // flag does not address (CPU scheduling only).
            if(CreateProcessA(NULL, mut.data(), NULL, NULL, FALSE,
                              CREATE_NEW_CONSOLE | BELOW_NORMAL_PRIORITY_CLASS,
                              NULL, NULL, &si, &g_ped_proc)){
                g_ped_proc_running = true;
            } else {
                g_statusMsg = "Pedestrian: couldn't launch detector - run the Python script manually";
                g_statusCol = C_WARN;
            }
        }
        g_ped_enabled.store(true);
    } else {
        if(g_ped_proc_running){
            TerminateProcess(g_ped_proc.hProcess, 0);
            CloseHandle(g_ped_proc.hProcess);
            CloseHandle(g_ped_proc.hThread);
            g_ped_proc = {};
            g_ped_proc_running = false;
        }
        g_ped_enabled.store(false);
        g_ped_detected.store(false);
        g_ped_caution.store(false);
        g_ped_paused.store(false);
        g_ped_stale.store(false);
    }
}

bool g_localize_request     = false;
bool g_localize_panel_needs_init = false;
float g_localize_seed_x     = 0.f;
float g_localize_seed_y     = 0.f;
float g_localize_seed_r     = SEED_RADIUS_OUTDOOR;
bool  g_localize_use_custom_seed = false;
bool g_intro_remote = false;
bool g_bg_show_3d   = true;   // default to the new 3D campus render
LocalizeBGTransform g_localize_bg_tf;
bool g_localize_cal_mode = false;
// True only when calibration mode was entered from the technical Step 1 panel (not
// the passenger flow) — see DrawPanelLocalize's "Calibrate This Map" button. Used to
// auto-return to the technical view once calibration finishes, since the passenger
// Localize panel's own "Back"/"Done" only knows how to go to the passenger Intro
// screen, not back to whichever technical step the operator was actually on.
bool g_localize_cal_from_technical = false;

// ── Map segmentation globals (June 22, 2026) ──
std::vector<Vector2> g_segment_polygon;
bool                 g_segment_polygon_valid = false;
bool                 g_plan_free_mode        = false;
bool                 g_plan_corner_mode      = false;  // top-LiDAR corner maps: force full corner through plan
bool                 g_polygon_edit_mode     = false;
std::vector<Vector2> g_polygon_edit_verts;

void load_segment_polygon(const char* path) {
    g_segment_polygon.clear(); g_segment_polygon_valid = false;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line; float vx, vy;
    while(std::getline(f, line)){
        if(line.empty() || line[0]=='#') continue;
        if(sscanf(line.c_str(), "%f %f", &vx, &vy) == 2)
            g_segment_polygon.push_back({vx, vy});
    }
    g_segment_polygon_valid = ((int)g_segment_polygon.size() >= 3);
}

void save_segment_polygon(const char* path) {
    std::ofstream f(path);
    f << "# Corridor region polygon (map world x y) — June 22, 2026\n"
      << std::fixed << std::setprecision(3);
    for(auto& v : g_segment_polygon)
        f << v.x << " " << v.y << "\n";
}
// ── End map segmentation globals (June 22, 2026) ──

// ── Localization zones (per map) ──
std::vector<Vector2> g_zone_poly;
bool                 g_zone_valid     = false;
bool                 g_zone_armed     = false;
bool                 g_zone_edit_mode = false;
std::vector<Vector2> g_zone_edit_verts;
bool                 g_zone_focus_request = false;

std::string zone_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.');
    if(dot != std::string::npos) fn = fn.substr(0, dot);
    return "zone1_" + fn + ".cfg";
}
void load_zone_for_map() {
    g_zone_poly.clear(); g_zone_valid = false; g_zone_armed = false; g_zone_edit_mode = false;
    std::string path = zone_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line; float vx, vy;
    while(std::getline(f, line)){
        if(line.empty() || line[0]=='#') continue;
        if(sscanf(line.c_str(), "%f %f", &vx, &vy) == 2) g_zone_poly.push_back({vx, vy});
    }
    g_zone_valid = ((int)g_zone_poly.size() >= 3);
}
void save_zone(const std::vector<Vector2>& verts) {
    std::string path = zone_file_for_map();
    if(path.empty() || (int)verts.size() < 3) return;
    std::ofstream f(path);
    f << "# Zone 1 polygon (map world x y) for "
      << (active_map_ptr ? active_map_ptr->name : "") << "\n"
      << std::fixed << std::setprecision(3);
    for(auto& v : verts) f << v.x << " " << v.y << "\n";
    g_zone_poly = verts; g_zone_valid = true;
}

// ── Localization zones — zone 2 (per map) ──
std::vector<Vector2> g_zone2_poly;
bool                  g_zone2_valid     = false;
bool                  g_zone2_armed     = false;
bool                  g_zone2_edit_mode = false;
std::vector<Vector2>  g_zone2_edit_verts;
bool                  g_zone2_focus_request = false;

std::string zone2_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.');
    if(dot != std::string::npos) fn = fn.substr(0, dot);
    return "zone2_" + fn + ".cfg";
}
void load_zone2_for_map() {
    g_zone2_poly.clear(); g_zone2_valid = false; g_zone2_armed = false; g_zone2_edit_mode = false;
    std::string path = zone2_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line; float vx, vy;
    while(std::getline(f, line)){
        if(line.empty() || line[0]=='#') continue;
        if(sscanf(line.c_str(), "%f %f", &vx, &vy) == 2) g_zone2_poly.push_back({vx, vy});
    }
    g_zone2_valid = ((int)g_zone2_poly.size() >= 3);
}
void save_zone2(const std::vector<Vector2>& verts) {
    std::string path = zone2_file_for_map();
    if(path.empty() || (int)verts.size() < 3) return;
    std::ofstream f(path);
    f << "# Zone 2 polygon (map world x y) for "
      << (active_map_ptr ? active_map_ptr->name : "") << "\n"
      << std::fixed << std::setprecision(3);
    for(auto& v : verts) f << v.x << " " << v.y << "\n";
    g_zone2_poly = verts; g_zone2_valid = true;
}

// ── Zone-2 slow-speed segment ── see common.hpp comment. Unlike g_dzsub's hardcoded
// defaults (that mechanism always needs SOME sub-zone on its dedicated test maps),
// this defaults to unset/inactive everywhere until explicitly configured for a map —
// most maps (and zone 1) must never be affected by it.
bool  g_z2slow_valid = false;
float g_z2slow_entry_x=0.f, g_z2slow_entry_y=0.f;
float g_z2slow_exit_x=0.f,  g_z2slow_exit_y=0.f;
std::string z2slow_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn = fn.substr(0,dot);
    return "z2slow_" + fn + ".cfg";
}
void load_z2slow_for_map() {
    g_z2slow_valid=false;
    std::string path = z2slow_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    float ex,ey,xx,xy;
    if(f>>ex>>ey>>xx>>xy){
        g_z2slow_entry_x=ex; g_z2slow_entry_y=ey; g_z2slow_exit_x=xx; g_z2slow_exit_y=xy;
        g_z2slow_valid=true;
    }
}
void save_z2slow_for_map() {
    std::string path = z2slow_file_for_map();
    if(path.empty()) return;
    std::ofstream f(path);
    f << std::fixed << std::setprecision(4)
      << g_z2slow_entry_x << " " << g_z2slow_entry_y << "\n"
      << g_z2slow_exit_x  << " " << g_z2slow_exit_y  << "\n";
    g_z2slow_valid=true;
}
void clear_z2slow_for_map() {
    g_z2slow_valid=false;
    std::string path = z2slow_file_for_map();
    if(path.empty()) return;
    std::ofstream f(path, std::ios::trunc);   // empty file -> load_z2slow_for_map() finds nothing
}

// ── Step 2 local path editing ──
bool  g_path_edit_mode   = false;
int   g_path_edit_i0     = -1;
int   g_path_edit_i1     = -1;
float g_path_edit_offset = 0.f;

bool  g_dz_edit_mode  = false;
int   g_dz_edit_zone0 = -1;
int   g_dz_edit_idx0  = -1;
int   g_dz_edit_zone1 = -1;
int   g_dz_edit_idx1  = -1;

// "Set Sub-Zone" tool (Danger Zone Fwd test map only) — 2 clicks snapped to nearest
// point on g_smooth (the plain recorded path; no zone corridors on this map).
bool  g_dzsub_edit_mode = false;
bool  g_dzsub_edit_have0 = false;
bool  g_dzsub_edit_have1 = false;
float g_dzsub_edit_x0=0.f, g_dzsub_edit_y0=0.f;
float g_dzsub_edit_x1=0.f, g_dzsub_edit_y1=0.f;

bool  g_z2slow_edit_mode = false;
bool  g_z2slow_edit_have0 = false;
bool  g_z2slow_edit_have1 = false;
float g_z2slow_edit_x0=0.f, g_z2slow_edit_y0=0.f;
float g_z2slow_edit_x1=0.f, g_z2slow_edit_y1=0.f;

// "Place Danger Path" tool (2026-07-07) — drag/rotate the OLD campus danger-zone
// curve (danger_zone_forward.csv, a different coordinate frame) into position on the
// new "Danger Zone Fwd" map by eye, then splice it into g_smooth to replace the
// sub-zone stretch. g_dzplace_raw is loaded once (raw coords, untouched); rotation is
// applied around its own centroid so it spins in place instead of swinging wildly.
bool  g_dzplace_mode = false;
std::vector<PPPoint> g_dzplace_raw;
float g_dzplace_cx=0.f, g_dzplace_cy=0.f;   // centroid of g_dzplace_raw (rotation pivot)
float g_dzplace_offset_x=0.f, g_dzplace_offset_y=0.f, g_dzplace_rotation=0.f;

// ── Per-map estimated start pose ──
PPPose g_est_start     = {0,0,0,false};
bool   g_est_start_mode = false;
std::string est_start_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn = fn.substr(0,dot);
    return "eststart_" + fn + ".cfg";
}
void load_est_start_for_map() {
    g_est_start = {0,0,0,false}; g_est_start_mode = false;
    std::string path = est_start_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line; float x,y,yaw;
    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        std::istringstream ss(line); char c;
        if(ss>>x>>c>>y>>c>>yaw){ g_est_start={x,y,yaw,true}; }
    }
    // Apply immediately as the planning pose so path previews start from the
    // correct place as soon as the map loads — no need to arm the zone first.
    if(g_est_start.valid && !g_confirmed_start.valid)
        g_pp_robot = g_est_start;
}
void save_est_start() {
    std::string path = est_start_file_for_map();
    if(path.empty() || !g_est_start.valid) return;
    std::ofstream f(path);
    f << "# Estimated start pose: x,y,yaw(rad)\n" << std::fixed << std::setprecision(4);
    f << g_est_start.x << "," << g_est_start.y << "," << g_est_start.yaw << "\n";
}

// ── Localize-panel background calibration (per map) ──
// Each map gets its own localize_bg_<basename>.cfg.
// Format: offset_x  offset_y  scale  rotation_deg  (key-value, # = comment)
// Developer calibrates once by overlaying the PLY cloud on the campus photo
// and adjusting until the dots align. File is written by the in-panel Save button.
std::string localize_bg_tf_file_for_map(){
    if(!active_map_ptr) return "";
    std::string fn=active_map_ptr->filename;
    size_t dot=fn.rfind('.'); if(dot!=std::string::npos) fn=fn.substr(0,dot);
    return "localize_bg_"+fn+".cfg";
}
void load_localize_bg_transform_for_map(){
    g_localize_bg_tf={};
    std::string path=localize_bg_tf_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    float ox=0,oy=0,sc=8.f,rot=0.f; bool got=false;
    std::string line;
    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        char key[64]; float val;
        if(sscanf(line.c_str(),"%63s %f",key,&val)!=2) continue;
        std::string k(key);
        if     (k=="offset_x")    ox=val;
        else if(k=="offset_y")    oy=val;
        else if(k=="scale")     { sc=val; got=true; }
        else if(k=="rotation_deg") rot=val;
    }
    if(!got) return;
    g_localize_bg_tf.offset_x=ox; g_localize_bg_tf.offset_y=oy;
    g_localize_bg_tf.scale=sc;
    g_localize_bg_tf.setRotation(rot*kPI/180.f);
    g_localize_bg_tf.valid=true;
}
void save_localize_bg_transform(){
    std::string path=localize_bg_tf_file_for_map();
    if(path.empty()) return;
    std::ofstream f(path);
    f<<"# Localize-panel background calibration for "
     <<(active_map_ptr?active_map_ptr->name:"")<<"\n"
     <<"# offset: image pixel where world origin (0,0) appears\n"
     <<"# scale: image pixels per world metre\n"
     <<"# rotation_deg: map +X relative to image right (CCW positive)\n"
     <<std::fixed<<std::setprecision(3);
    f<<"offset_x      "<<g_localize_bg_tf.offset_x<<"\n";
    f<<"offset_y      "<<g_localize_bg_tf.offset_y<<"\n";
    f<<"scale         "<<g_localize_bg_tf.scale<<"\n";
    f<<"rotation_deg  "<<(g_localize_bg_tf.rotation*180.f/kPI)<<"\n";
    g_localize_bg_tf.valid=true;
}

// ── Saved destinations globals ──
// Canonical, map-independent destination identity: index i is always the
// same named place, on every map. Only the (x,y) differs per map, loaded
// from that map's dest_<plyname>.cfg.
const char* const g_dest_names[NUM_DEST] =
    {"Labs", "Swimming Pool", "Security", "Building A", "Between Building A and S"};
std::array<SavedDest,NUM_DEST> g_destinations = {{
    {"Labs",                     0.f, 0.f, false},
    {"Swimming Pool",            0.f, 0.f, false},
    {"Security",                 0.f, 0.f, false},
    {"Building A",               0.f, 0.f, false},
    {"Between Building A and S", 0.f, 0.f, false},
}};
int  g_selected_dest   = -1;
bool   g_dest_setup_mode       = false;
int    g_dest_setup_idx        = -1;
bool   g_dest_setup_previewing = false;
PPPose g_dest_setup_prev_goal  = {0.f,0.f,0.f,false};

void load_destinations(const char* path) {
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line;
    while(std::getline(f, line)) {
        if(line.empty()||line[0]=='#') continue;
        std::istringstream ss(line); int idx; char c;
        float x, y;
        if(ss >> idx >> c >> x >> c >> y && idx>=0 && idx<NUM_DEST) {
            g_destinations[idx].x = x;
            g_destinations[idx].y = y;
            g_destinations[idx].valid = true;
        }
    }
}

void save_destinations(const char* path) {
    std::ofstream f(path);
    f << "# Destination world coords — index,x,y\n" << std::fixed << std::setprecision(3);
    for(int i=0;i<NUM_DEST;i++)
        if(g_destinations[i].valid)
            f << i << "," << g_destinations[i].x << "," << g_destinations[i].y << "\n";
}

// Per-map destinations: each map has its own dest_<plyname>.cfg (NUM_DEST points).
std::string dest_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn = fn.substr(0,dot);
    return "dest_" + fn + ".cfg";
}
void load_destinations_for_map() {
    for(auto& d : g_destinations) d.valid=false;
    g_selected_dest = -1;
    std::string path = dest_file_for_map();
    std::ifstream chk(path);
    if(chk.is_open()){ chk.close(); load_destinations(path.c_str()); return; }
    // Fallback to the legacy global file ONLY for maps in the original coordinate
    // frame. The top-LiDAR maps have their own frames, so the old global coords
    // would land in the wrong place — they start empty until set per map.
    bool top_lidar = active_map_ptr &&
        (active_map_ptr->name=="Campus Top LiDAR" || active_map_ptr->name=="Campus Top (Rev)");
    if(!top_lidar) load_destinations("destinations.cfg");
}
void save_destinations_for_map() {
    std::string path = dest_file_for_map();
    if(!path.empty()) save_destinations(path.c_str());
}

// ── Zone 2's destinations (own identity list — different physical places than zone 1) ──
const char* const g_dest_names2[NUM_DEST2] =
    {"Basketball Court Curve", "Gate", "Curve 2 - Food Court", "Green Area",
     "Basketball Court", "Food Court"};
std::array<SavedDest,NUM_DEST2> g_destinations2 = {{
    {"Basketball Court Curve", 0.f, 0.f, false},
    {"Gate",                   0.f, 0.f, false},
    {"Curve 2 - Food Court",   0.f, 0.f, false},
    {"Green Area",             0.f, 0.f, false},
    {"Basketball Court",       0.f, 0.f, false},
    {"Food Court",             0.f, 0.f, false},
}};
int    g_selected_dest2        = -1;
bool   g_dest_setup_mode2      = false;
int    g_dest_setup_idx2       = -1;
bool   g_dest_setup_previewing2 = false;
PPPose g_dest_setup_prev_goal2  = {0.f,0.f,0.f,false};

void load_destinations2(const char* path) {
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line;
    while(std::getline(f, line)) {
        if(line.empty()||line[0]=='#') continue;
        std::istringstream ss(line); int idx; char c;
        float x, y;
        if(ss >> idx >> c >> x >> c >> y && idx>=0 && idx<NUM_DEST2) {
            g_destinations2[idx].x = x;
            g_destinations2[idx].y = y;
            g_destinations2[idx].valid = true;
        }
    }
}
void save_destinations2(const char* path) {
    std::ofstream f(path);
    f << "# Zone 2 destination world coords — index,x,y\n" << std::fixed << std::setprecision(3);
    for(int i=0;i<NUM_DEST2;i++)
        if(g_destinations2[i].valid)
            f << i << "," << g_destinations2[i].x << "," << g_destinations2[i].y << "\n";
}
std::string dest2_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn = fn.substr(0,dot);
    return "dest2_" + fn + ".cfg";
}
void load_destinations2_for_map() {
    for(auto& d : g_destinations2) d.valid=false;
    g_selected_dest2 = -1;
    std::string path = dest2_file_for_map();
    if(path.empty()) return;
    std::ifstream chk(path);
    if(chk.is_open()){ chk.close(); load_destinations2(path.c_str()); }
    // No legacy-file fallback — zone 2 only exists on top-LiDAR maps and never had
    // a pre-per-map global destinations file.
}
void save_destinations2_for_map() {
    std::string path = dest2_file_for_map();
    if(!path.empty()) save_destinations2(path.c_str());
}

// Combined index 0..NUM_DEST-1 = zone 1, NUM_DEST..NUM_DEST_TOTAL-1 = zone 2.
// Arms whichever zone the destination belongs to (mutually exclusive with the other)
// and reloads/replans if the armed zone actually changed, so the corner-mode planner
// routes through the correct corner before the caller sets g_pp_goal.
bool pp_run_pipeline(const std::string&);   // forward decl (planning.cpp) — used below
bool resolve_combined_dest(int combined_idx, const char*& name_out, SavedDest& out) {
    if(combined_idx>=0 && combined_idx<NUM_DEST){
        if(!g_zone_armed){ g_zone_armed=true; g_zone2_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
        name_out = g_dest_names[combined_idx];
        out = g_destinations[combined_idx];
        return out.valid;
    }
    int i2 = combined_idx - NUM_DEST;
    if(i2>=0 && i2<NUM_DEST2){
        if(!g_zone2_armed){ g_zone2_armed=true; g_zone_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
        name_out = g_dest_names2[i2];
        out = g_destinations2[i2];
        return out.valid;
    }
    return false;
}

// Forward-map visiting order (1-based rank), indexed by combined destination index:
// 0=Labs 1=Swimming Pool 2=Security 3=Building A 4=Between Building A and S
// 5=Basketball Court Curve 6=Gate 7=Curve 2 - Food Court 8=Green Area
// Sequence as given: Labs, Security, Swimming Pool, Between Building A and S,
// Building A, Basketball Court Curve, Gate, Green Area, Curve 2 - Food Court.
const int g_dest_order_fwd[NUM_DEST_TOTAL] = {1,3,2,5,4,6,7,9,8};
int dest_order_rank(int combined_idx) {
    if(combined_idx<0 || combined_idx>=NUM_DEST_TOTAL) return -1;
    int r = g_dest_order_fwd[combined_idx];
    // Reverse map drives the same stops in the opposite sequence.
    if(active_map_ptr && active_map_ptr->name=="Campus Top (Rev)") r = NUM_DEST_TOTAL+1-r;
    return r;
}
// ── End saved destinations globals ──

// ── Destination grid cells (per-map, Step 2 tap-to-navigate) ──
std::map<std::pair<int,int>, int> g_dest_grid;
bool g_dest_grid_edit      = false;
int  g_dest_grid_paint_dest = 0;

std::string dest_grid_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn=fn.substr(0,dot);
    return "destgrid_" + fn + ".cfg";
}
void load_dest_grid_for_map() {
    g_dest_grid.clear();
    std::string path = dest_grid_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line;
    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        int col,row,di;
        if(sscanf(line.c_str(),"%d,%d,%d",&col,&row,&di)==3)
            g_dest_grid[{col,row}] = di;
    }
}
void save_dest_grid_for_map() {
    std::string path = dest_grid_file_for_map();
    if(path.empty()) return;
    std::ofstream f(path);
    f << "# Destination grid cells: col,row,dest_idx\n";
    for(auto& kv : g_dest_grid)
        f << kv.first.first << "," << kv.first.second << "," << kv.second << "\n";
}
// ── End destination grid ──

// ── Passenger zone grid (paxgrid.cfg, campus-wide) ───────────────────────
std::map<std::pair<int,int>, int> g_pax_grid;
bool g_pax_grid_edit = false;
int  g_pax_paint_dest = 0;

void load_pax_grid() {
    g_pax_grid.clear();
    std::ifstream f("paxgrid.cfg");
    if(!f.is_open()) return;
    std::string line;
    while(std::getline(f, line)) {
        if(line.empty() || line[0]=='#') continue;
        int col, row, di;
        if(sscanf(line.c_str(), "%d,%d,%d", &col, &row, &di)==3 &&
           (di==PAX_UNREACHABLE || (di>=0 && di<NUM_DEST_TOTAL)))
            g_pax_grid[{col,row}] = di;
    }
}
void save_pax_grid() {
    std::ofstream f("paxgrid.cfg");
    f << "# Passenger zone grid: col,row,combined_dest_idx (0.." << (NUM_DEST_TOTAL-1) << ")\n";
    for(auto& kv : g_pax_grid)
        f << kv.first.first << "," << kv.first.second << "," << kv.second << "\n";
}

// ── Forward declarations: threads ──
void lidar_receiver_thread();
void imu_receiver_thread();
void scan_matching_thread();
void pose_writer_thread();
void control_loop_thread();

// ── Forward declarations: localization.cpp ──
bool load_ply(const std::string&, std::vector<MapPoint>&);
DistanceMap build_distance_map(const std::vector<MapPoint>&, bool);
void particle_filter_reloc_thread(std::vector<Point3D>, const DistanceMap*, bool, float, bool, float, float, float);

// ── Forward declarations: planning.cpp ──
bool pp_run_pipeline(const std::string&);
bool pp_load_smoothed();
void pp_trigger_plan();
void pp_save_plan();
void pp_clear_path();
int  pp_zone_for_point(float x, float y, float max_dist_m);

// ── Forward declarations: control.cpp ──
void ctrl_doLoad();
void ctrl_doStart();
void ctrl_doStop();
void ctrl_doReset();

// ── Forward declarations: communication.cpp ──
bool initNetwork();
void ctrl_sendCmd(const std::string& cmd);

// ── Forward declarations: ui.cpp ──
void DrawPanelLocalize(int,int,int,int,AppState&,float,int,int);
void DrawPanelPlan(int,int,int,int);
void DrawPanelDrive(int,int,int,int);
void DrawStepBar(int,int);
void DrawIntroPanel(int,int,Texture2D,Texture2D,Texture2D,Font,bool&,bool&,bool&);
void DrawWelcomePanel(int,int,Texture2D,Font,bool&,bool&);
void DrawDestinationPanel(int,int,Texture2D,Font,bool&,AppState);
void DrawLocalizePanel(int,int,Texture2D,Font,AppState,bool&);
void DrawMapSelectPanel(int,int,Texture2D,Font,bool&);
void DrawDriveMapPanel(int,int,int,int,Texture2D,Font,AppState);
void DrawJourneyEndPanel(int,int,Texture2D,Font,bool&,bool&,bool&,bool&,bool&,bool&);

struct View2D { float cx, cy, scale; int sw, sh; };
static Vector2 w2s(float wx, float wy, const View2D& v) {
    return {v.sw*0.5f+(wx-v.cx)*v.scale, v.sh*0.5f-(wy-v.cy)*v.scale};
}

void DrawMapViewport(int mapW, int mapH, int TITLE_H, int PANEL_W, int BAR_H,
                     int step, Camera3D& camera, View2D& view2d, bool camera_follow,
                     AppState fus_state);

// Switch the active map by name and reload all per-map config files.
// Same housekeeping as the [M] key handler in the render loop.
void switch_active_map_by_name(const std::string& name) {
    for(int i=0;i<(int)map_library.size();i++){
        if(map_library[i].name==name && map_library[i].loaded){
            active_map_index=i;
            active_map_ptr=&map_library[i];
            {std::lock_guard<std::mutex>lk(trajectory_mutex);
             robot_trajectory.clear(); trajectory_data.clear();}
            pp_clear_path();
            g_confirmed_start={0,0,0,false};
            g_pp_robot={0,0,0,false};
            load_zone_for_map();
            load_zone2_for_map();
            load_destinations_for_map();
            load_destinations2_for_map();
            load_est_start_for_map();
            load_dest_grid_for_map();
            load_localize_bg_transform_for_map();
            load_z2slow_for_map();
            load_border_grid();
            g_localize_panel_needs_init=true;
            g_dest_grid_edit=false;
            g_path_edit_mode=false; g_path_edit_i0=-1; g_path_edit_i1=-1;  // stale vs new g_smooth
            g_reloc_error_msg="";  // clear any stale error from a previous map's reloc attempt
            return;
        }
    }
}

int main() {
    ekf_fusion_codegen_initialize();
    g_statusCol = C_DIM;

    // ── Load maps ──
    std::cout << "[MAP] Loading map library...\n";
    for(auto& e : map_library){
        e.loaded = load_ply(e.filename, e.cloud);
        if(e.loaded){
            e.dist_map = build_distance_map(e.cloud, e.is_large);
            std::cout << "  Loaded: " << e.name << " (" << e.cloud.size() << " pts)\n";
        } else {
            std::cout << "  Not found: " << e.filename << "\n";
        }
    }
    // Default to Full Campus (by name, robust to map list reordering); fall back to first loaded map
    active_map_index = -1;
    for(size_t i = 0; i < map_library.size(); i++)
        if(map_library[i].name == "Full Campus" && map_library[i].loaded){ active_map_index=(int)i; break; }
    if(active_map_index < 0)
        for(size_t i = 0; i < map_library.size(); i++)
            if(map_library[i].loaded){ active_map_index=(int)i; break; }
    if(active_map_index >= 0) active_map_ptr = &map_library[active_map_index];
    load_zone_for_map();      // load this map's zone 1 (if any)
    load_zone2_for_map();     // load this map's zone 2 (if any)
    load_est_start_for_map();
    load_dest_grid_for_map(); // load tap-to-navigate grid
    load_pax_grid();          // load passenger panel zone grid
    load_localize_bg_transform_for_map(); // background calibration for localize panel (optional)
    load_z2slow_for_map();    // zone-2 slow-speed segment (optional)
    load_border_grid();       // border/wall forbidden cells (optional)

    // ── EKF init ──
    {
        const double Qd[8]={1e-5,1e-5,1e-6,0.02,0.02,0.01,5e-5,5e-5};
        fill8(ekf.Q,Qd);
        fill2(ekf.R_zupt,0.02,0.02);
        const double Pd[8]={1,1,0.5,1,1,0.5,0.1,0.1};
        fill8(ekf.P,Pd);
    }

    pp_load_smoothed();
    load_segment_polygon("segment_polygon.cfg");
    load_destinations_for_map();
    load_destinations2_for_map();
    load_dzsub_fwd();
    initNetwork();

    // ── Start background threads ──
    std::thread(lidar_receiver_thread).detach();
    std::thread(imu_receiver_thread).detach();
    std::thread(scan_matching_thread).detach();
    std::thread(pose_writer_thread).detach();
    std::thread(control_loop_thread).detach();

    // ── Raylib window ──
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1600, 900, "Autonomous Golf Cart System");
    SetExitKey(KEY_NULL);
    SetTargetFPS(60);

    // ── Intro screen assets ──
    // Two backgrounds the operator can toggle between: satellite photo and 3D render.
    auto loadBG = [](std::initializer_list<const char*> names)->Texture2D{
        for(const char* f : names){
            if(FileExists(f)){
                Image im = LoadImage(f);
                if(IsImageReady(im)){
                    Texture2D t = LoadTextureFromImage(im);
                    UnloadImage(im);
                    if(t.id>0) SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
                    return t;
                }
            }
        }
        return Texture2D{};
    };
    Texture2D g_bg_sat = loadBG({"campus_bg.png","campus_bg.jpg"});
    Texture2D g_bg_3d  = loadBG({"campus 3d image.png","campus_bg_3d.png","campus_bg_3d.jpg"});
    if(g_bg_3d.id==0)  g_bg_3d  = g_bg_sat;   // fall back if 3D image missing
    if(g_bg_sat.id==0) g_bg_sat = g_bg_3d;
    Image _logo = LoadImage("giu_logo.png");
    Texture2D g_intro_logo = (IsImageReady(_logo) ? LoadTextureFromImage(_logo) : Texture2D{});
    UnloadImage(_logo);
    Texture2D g_welcome_img = loadBG({"giu_welcome.png","GIU welcome panal.png"});
    // UI font — Segoe UI for clean modern look; fallback to default
    Font g_ui_font = {};
    const char* font_path = "C:/Windows/Fonts/segoeui.ttf";
    if(FileExists(font_path)) g_ui_font = LoadFontEx(font_path, 72, NULL, 0);
    if(g_ui_font.texture.id == 0) g_ui_font = GetFontDefault();
    SetTextureFilter(g_ui_font.texture, TEXTURE_FILTER_BILINEAR);

    Image _cart = LoadImage("cart_icon.png");
    if(IsImageReady(_cart)){
        ImageFormat(&_cart, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        Color* px = (Color*)_cart.data;
        int n = _cart.width * _cart.height;
        // Remove white / near-white background
        for(int i=0;i<n;i++)
            if(px[i].r>215 && px[i].g>215 && px[i].b>215) px[i].a=0;
        // Auto-crop transparent padding so cart fills the full draw rect
        int minX=_cart.width, maxX=0, minY=_cart.height, maxY=0;
        for(int y=0;y<_cart.height;y++)
            for(int x=0;x<_cart.width;x++)
                if(px[y*_cart.width+x].a>10){
                    if(x<minX) minX=x; if(x>maxX) maxX=x;
                    if(y<minY) minY=y; if(y>maxY) maxY=y;
                }
        if(maxX>minX && maxY>minY)
            ImageCrop(&_cart,{(float)minX,(float)minY,
                              (float)(maxX-minX+2),(float)(maxY-minY+2)});
    }
    Texture2D g_cart_icon = (IsImageReady(_cart) ? LoadTextureFromImage(_cart) : Texture2D{});
    if(g_cart_icon.id>0) SetTextureFilter(g_cart_icon, TEXTURE_FILTER_BILINEAR);
    UnloadImage(_cart);
    g_show_welcome = true;    // GIU welcome splash first
    g_show_intro  = false;
    g_show_dest   = false;
    g_intro_remote = false;

    Camera3D camera = {};
    camera.position  = {0,60,0};
    camera.target    = {0,0,0};
    camera.up        = {0,0,-1};
    camera.fovy      = 60;
    camera.projection = CAMERA_PERSPECTIVE;

    float cam_azimuth   = 0.f;
    float cam_elevation = 1.5707963f;   // PI/2 = top-down (matches startup position)
    float cam_radius    = 60.f;

    AppState fus_state   = STATE_CALIBRATING;
    float calib_timer    = 0, hz_timer = 0;
    int   cur_lidar_hz   = 0, cur_imu_hz = 0;
    int   last_sweep     = 0, last_imu  = 0, last_processed_sweep = 0;
    bool  camera_follow  = true;

    View2D view2d = {0, 0, 15, 1600, 900};

    // Step 3 split-screen (2026-07-08): the technical map viewport (STEP_DRIVE only)
    // renders into this off-screen texture at its OWN local width (not the full
    // window), then gets blitted into the right half of the screen — same coordinate
    // conventions DrawMapViewport already uses, just narrower, so nothing distorts.
    // (Re)created on demand if the required size changes (window resize, etc).
    // DrawPanelDrive is drawn directly to the real framebuffer, NOT through this
    // texture — it has real Start/Stop button hit-tests against GetMousePosition(),
    // which would silently break if compared against texture-local coordinates
    // instead of the real screen position they're actually drawn at.
    RenderTexture2D step3MapRT = {0};

    // ── MAIN LOOP ──
    while(!WindowShouldClose()){
        float dt = GetFrameTime();
        hz_timer += dt;
        if(hz_timer >= 1.f){
            int cs=lidar_sweeps_total.load(), ci=imu_frames_total.load();
            cur_lidar_hz=cs-last_sweep; cur_imu_hz=ci-last_imu;
            last_sweep=cs; last_imu=ci; hz_timer=0;
        }

        // ── Pedestrian detector status poll (2026-07-08) — throttled to ~10 Hz so file
        // I/O stays off the 50 Hz control thread (which only reads the resulting atomics).
        // The Python detector writes "STOP <ts>" / "CAUTION <ts>" / "CLEAR <ts>" (CAUTION
        // added 2026-07-27: person in-lane at ANY distance, not just within --stop-distance
        // — see PED_CAUTION_CRUISE_VOLTAGE in control.cpp). A status older than 2s means
        // the detector isn't reporting (dead/slow) -> treat as CLEAR (never let a stale
        // STOP/CAUTION strand or slow the cart) but flag g_ped_stale so the UI can warn.
        {
            static float ped_poll_timer = 0.f;
            ped_poll_timer += dt;
            if(g_ped_enabled.load() && ped_poll_timer >= 0.1f){
                ped_poll_timer = 0.f;
                bool detected=false, caution=false, stale=true;
                std::ifstream pf(PED_STATUS_FILE);
                if(pf.is_open()){
                    std::string state; double ts=0;
                    if(pf>>state>>ts){
                        double age = (double)time(nullptr) - ts;
                        stale = (age > 2.0);
                        detected = (!stale && state=="STOP");
                        caution  = (!stale && (state=="STOP" || state=="CAUTION"));
                    }
                }
                g_ped_detected.store(detected);
                g_ped_caution.store(caution);
                g_ped_stale.store(stale);
            } else if(!g_ped_enabled.load()){
                ped_poll_timer = 0.f;
            }
        }

        // Danger-zone exit: ctrl_step() sets g_dz_return_to_localize and stops the cart.
        if(g_dz_return_to_localize){
            g_dz_return_to_localize=false;
            if(g_drive_via_passenger){
                // Jump to the passenger Localize panel — same already-proven path as
                // starting a fresh journey — instead of the custom in-drive relocalize-
                // then-replan flow this replaced (it hung the app during a live test).
                g_show_welcome=false; g_show_intro=false; g_show_map_select=false; g_show_dest=false;
                g_show_localize=true;
                // fus_state is still STATE_FUSION_RUNNING from before the danger zone —
                // DrawLocalizePanel's "located" check is exactly (fus_state==
                // STATE_FUSION_RUNNING), so without resetting it the panel would open
                // straight to "Cart found!" using the stale pre-danger-zone lock, giving no
                // way to drag the ring and search fresh. STATE_ALIGNMENT_CHECK is the
                // "ready to search, not yet located" state the panel expects to start from.
                fus_state=STATE_ALIGNMENT_CHECK;
            } else {
                // Drive was started from the technical Step 1/2/3 view directly — stay
                // there instead of bouncing into the passenger panels, which were never
                // used this run. Land back on Step 1 to re-localize.
                g_ui_step.store(STEP_LOCALIZE);
                fus_state=STATE_ALIGNMENT_CHECK;
            }
        }

        int sw=GetScreenWidth(), sh=GetScreenHeight();
        const int PANEL_W=310, BAR_H=50, TITLE_H=46;
        int mapW=sw-PANEL_W, mapH=sh-BAR_H;
        int step=g_ui_step.load();

        view2d.sw=mapW;
        view2d.sh=mapH;

        // ── KEYBOARD INPUT ──
        if(IsKeyPressed(KEY_TAB)){int s=g_ui_step.load();if(s<3)g_ui_step.store(s+1);}
        if(IsKeyPressed(KEY_F)&&step!=STEP_PLAN) camera_follow=!camera_follow;
        if(IsKeyPressed(KEY_G)) g_show_range_rings.store(!g_show_range_rings.load());
        if(IsKeyPressed(KEY_M)){
            int start=active_map_index;
            do{active_map_index=(active_map_index+1)%(int)map_library.size();}
            while(!map_library[active_map_index].loaded&&active_map_index!=start);
            active_map_ptr=&map_library[active_map_index];
            {std::lock_guard<std::mutex>lk(trajectory_mutex);robot_trajectory.clear();trajectory_data.clear();}
            pp_clear_path();  // drop path/corridor built for the previous map's frame
            // Each map has its own coordinate frame — clear previous localization
            // so the estimated start (not the old pose) drives pre-localize previews.
            g_confirmed_start={0,0,0,false};
            g_pp_robot={0,0,0,false};
            load_zone_for_map();                     // load the new map's zone 1
            load_zone2_for_map();                    // load the new map's zone 2
            load_destinations_for_map();             // load the new map's zone-1 destinations
            load_destinations2_for_map();             // load the new map's zone-2 destinations
            load_est_start_for_map();                // sets g_pp_robot to est start if valid
            load_dest_grid_for_map();                // load tap-to-navigate cell grid
            load_localize_bg_transform_for_map();    // localize-panel bg calibration
            g_localize_panel_needs_init = true;      // reposition search circle for new map
            g_dest_grid_edit = false;    // never carry edit mode across maps
            // Danger-zone test maps have no zone to arm (which is what loads g_smooth
            // for the campus corner maps), so load their recorded path now on switch.
            // Scoped to these two names only — the confirmed campus maps are untouched.
            if(active_map_ptr && (active_map_ptr->name=="Danger Zone Fwd" ||
                                  active_map_ptr->name=="Danger Zone Rev")){
                pp_run_pipeline(TRAJ_CSV_FILE);
                load_dzsub_fwd();   // per-map sub-zone markers (dzsub_fwd.cfg / dzsub_rev.cfg)
            }
            fus_state=STATE_ALIGNMENT_CHECK; g_reloc_error_msg="";
        }
        if(IsKeyPressed(KEY_E)&&step==STEP_DRIVE){
            ctrl_sendCmd("ESTOP");
            if(g_ctrl_state==CS_RUNNING){g_ctrl_state=CS_WATCHDOG;g_statusMsg="Manual ESTOP";g_statusCol=C_BAD;}
        }
        if(IsKeyPressed(KEY_S)){
            if(step==STEP_LOCALIZE){
                std::lock_guard<std::mutex>lk(trajectory_mutex);
                if(!trajectory_data.empty()){
                    std::ofstream f(TRAJ_CSV_FILE);
                    if(f.is_open()){
                        f<<"x,y,z,yaw,rmse,inlier_ratio,timestamp,is_turning\n"<<std::fixed<<std::setprecision(6);
                        for(auto& p:trajectory_data)
                            f<<p.x<<","<<p.y<<","<<p.z<<","<<p.yaw<<","<<p.rmse<<","
                             <<p.inlier_ratio<<","<<p.timestamp<<","<<(p.is_turning?1:0)<<"\n";
                    }
                }
            }
            else if(step==STEP_PLAN){ pp_save_plan(); ctrl_doLoad(); }
            else if(step==STEP_DRIVE){ g_simMode=!g_simMode; if(g_ctrl_state==CS_RUNNING)ctrl_doStop(); }
        }
        if(IsKeyPressed(KEY_L)&&step==STEP_PLAN){ pp_run_pipeline("reference_path.csv"); pp_trigger_plan(); }
        if(IsKeyPressed(KEY_R)){
            if(step==STEP_LOCALIZE&&fus_state==STATE_FUSION_RUNNING){
                fus_state=STATE_ALIGNMENT_CHECK; g_reloc_error_msg="";
                std::lock_guard<std::mutex>lk(trajectory_mutex);
                robot_trajectory.clear(); trajectory_data.clear();
            } else if(step==STEP_PLAN){ pp_run_pipeline(TRAJ_CSV_FILE); pp_trigger_plan(); }
        }
        if(IsKeyPressed(KEY_F)&&step==STEP_PLAN){
            std::ifstream pf(POSE_FILE_OUT);
            if(pf.is_open()){
                float px2,py2,pyaw2;
                if(pf>>px2>>py2>>pyaw2){
                    g_pp_robot={px2,py2,pyaw2,true};
                    g_confirmed_start={px2,py2,pyaw2,true};
                    pp_trigger_plan();
                }
            }
        }

        // ── MOUSE / CAMERA ──
        float wheel=GetMouseWheelMove();
        if(step==STEP_LOCALIZE){
            // 3D orbital camera — shared between developer Step 1 and operator 3D mode
            if(wheel!=0){ cam_radius=Clamp(cam_radius-wheel*5.f,5.f,500.f); }
            if(IsMouseButtonDown(MOUSE_BUTTON_RIGHT)&&GetMousePosition().x<mapW){
                Vector2 d=GetMouseDelta();
                float ps=0.05f*(cam_radius/20.f);
                camera.target.x-=d.x*ps; camera.target.z-=d.y*ps;
            }
            if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)&&!IsKeyDown(KEY_LEFT_SHIFT)&&GetMousePosition().x<mapW){
                Vector2 d=GetMouseDelta();
                cam_azimuth   += d.x*0.005f;
                cam_elevation  = Clamp(cam_elevation - d.y*0.005f, 0.12f, 1.5707f);
            }
            float orb=1.5f*dt;
            if(IsKeyDown(KEY_LEFT))  cam_azimuth-=orb;
            if(IsKeyDown(KEY_RIGHT)) cam_azimuth+=orb;
            if(IsKeyDown(KEY_UP))    cam_elevation=Clamp(cam_elevation+orb,0.12f,1.5707f);
            if(IsKeyDown(KEY_DOWN))  cam_elevation=Clamp(cam_elevation-orb,0.12f,1.5707f);
            if(IsKeyPressed(KEY_HOME)||IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE))
                { cam_elevation=1.5707f; cam_azimuth=0.f; }
        } else {
            if(wheel!=0&&GetMousePosition().x<mapW){view2d.scale+=wheel*1.5f;if(view2d.scale<1)view2d.scale=1;}
            // Right-drag is repurposed for rotation while placing the danger path (below) —
            // suppress the normal view-pan binding so they don't fight over the same input.
            if(!g_dzplace_mode && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)&&GetMousePosition().x<mapW){
                Vector2 d=GetMouseDelta();view2d.cx-=d.x/view2d.scale;view2d.cy+=d.y/view2d.scale;
            }
        }

        // ── PLACE DANGER PATH drag/rotate (Step 2, while the tool is active) ──
        // Left-drag moves the curve (world-space delta, screen Y flipped vs world Y like
        // the pan binding above); right-drag rotates it about its own centroid. Mirrors
        // the map-calibration tool's drag=pan/right-drag=rotate pattern.
        if(g_dzplace_mode && step==STEP_PLAN && GetMousePosition().x<mapW){
            if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
                Vector2 d=GetMouseDelta();
                g_dzplace_offset_x += d.x/view2d.scale;
                g_dzplace_offset_y -= d.y/view2d.scale;
            }
            if(IsMouseButtonDown(MOUSE_BUTTON_RIGHT)){
                Vector2 d=GetMouseDelta();
                g_dzplace_rotation += d.x*0.006f;
            }
        }

        // ── SEED CLICK (Shift+LMB in Step 1) ──
        float cur_seed_r=(active_map_ptr&&active_map_ptr->is_large)?SEED_RADIUS_OUTDOOR:SEED_RADIUS_INDOOR;
        if(step==STEP_LOCALIZE&&IsKeyDown(KEY_LEFT_SHIFT)&&active_map_ptr&&active_map_ptr->loaded){
            if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&GetMousePosition().x<mapW){
                Ray ray=GetMouseRay(GetMousePosition(),camera);
                float t=-ray.position.y/ray.direction.y;
                if(t>0){
                    float sx=ray.position.x+ray.direction.x*t;
                    float sy=-(ray.position.z+ray.direction.z*t);
                    std::lock_guard<std::mutex>lk(g_reloc_mutex);
                    if(!g_reloc_running){
                        g_reloc_done=false;g_reloc_running=true;g_reloc_progress=0;g_reloc_error_msg="";
                        std::vector<Point3D>snap;{std::lock_guard<std::mutex>lk2(lidar_mutex);snap=dense_cloud;}
                        std::thread(particle_filter_reloc_thread,std::move(snap),&active_map_ptr->dist_map,
                                    active_map_ptr->is_large,active_map_ptr->inlier_dist,true,sx,sy,cur_seed_r).detach();
                        fus_state=STATE_GLOBAL_RELOC;
                    }
                }
            }
        }

        // ── DESTINATION SETUP CLICK (plain LMB while awaiting a candidate point for the
        // destination currently being placed, Step 2). Previews the path immediately —
        // nothing is saved until the admin hits Confirm in the panel (see ui.cpp).
        // Ensures zone 1 is armed first so the preview routes through zone 1's corner —
        // necessary now that zone 2 can be armed instead.
        if(g_dest_setup_mode && g_dest_setup_idx>=0 && !g_dest_setup_previewing &&
           step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mp=GetMousePosition();
            float wx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
            float wy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
            if(!g_zone_armed){ g_zone_armed=true; g_zone2_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
            g_pp_goal = {wx, wy, 0, true};
            pp_trigger_plan();
            g_dest_setup_previewing = true;
        }

        // ── ZONE 2 DESTINATION SETUP CLICK — mirrors the block above for zone 2's
        // destination picker, arming zone 2 (not zone 1) before previewing. ──
        if(g_dest_setup_mode2 && g_dest_setup_idx2>=0 && !g_dest_setup_previewing2 &&
           step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mp2=GetMousePosition();
            float wx2=view2d.cx+(mp2.x-view2d.sw*0.5f)/view2d.scale;
            float wy2=view2d.cy-(mp2.y-view2d.sh*0.5f)/view2d.scale;
            if(!g_zone2_armed){ g_zone2_armed=true; g_zone_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
            g_pp_goal = {wx2, wy2, 0, true};
            pp_trigger_plan();
            g_dest_setup_previewing2 = true;
        }

        // ── GOAL CLICK (Ctrl+LMB in Step 2, only when not in dest-setup mode) ──
        if(!g_dest_setup_mode&&step==STEP_PLAN&&IsKeyDown(KEY_LEFT_CONTROL)&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)&&GetMousePosition().x<mapW){
            Vector2 mp=GetMousePosition();
            float wx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
            float wy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
            g_pp_goal={wx,wy,0,true}; pp_trigger_plan();
        }

        // ── POLYGON VERTEX CLICK (plain LMB in Step 2 polygon-edit mode) — June 22, 2026 ──
        if(g_polygon_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mp2 = GetMousePosition();
            float vwx = view2d.cx + (mp2.x - view2d.sw*0.5f) / view2d.scale;
            float vwy = view2d.cy - (mp2.y - view2d.sh*0.5f) / view2d.scale;
            g_polygon_edit_verts.push_back({vwx, vwy});
        }
        // ── End polygon vertex click (June 22, 2026) ──

        // ── ZONE VERTEX CLICK (plain LMB in Step 2 while drawing zone 1) ──
        if(g_zone_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mpz = GetMousePosition();
            float zwx = view2d.cx + (mpz.x - view2d.sw*0.5f) / view2d.scale;
            float zwy = view2d.cy - (mpz.y - view2d.sh*0.5f) / view2d.scale;
            g_zone_edit_verts.push_back({zwx, zwy});
        }

        // ── ZONE 2 VERTEX CLICK (plain LMB in Step 2 while drawing zone 2) ──
        if(g_zone2_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mpz2 = GetMousePosition();
            float zwx2 = view2d.cx + (mpz2.x - view2d.sw*0.5f) / view2d.scale;
            float zwy2 = view2d.cy - (mpz2.y - view2d.sh*0.5f) / view2d.scale;
            g_zone2_edit_verts.push_back({zwx2, zwy2});
        }

        // ── ESTIMATED START CLICK (plain LMB in Step 2 while setting start point) ──
        if(g_est_start_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mps = GetMousePosition();
            float swx = view2d.cx + (mps.x - view2d.sw*0.5f) / view2d.scale;
            float swy = view2d.cy - (mps.y - view2d.sh*0.5f) / view2d.scale;
            // heading toward the nearest reference-path (corner) point — the cart
            // drives from the start into the curve. CCW-positive: forward=(-sin,cos).
            float yaw = 0.f;
            if(!g_smooth.empty()){
                int bi=0; float bd=1e30f;
                for(size_t i=0;i<g_smooth.size();i++){
                    float dx=g_smooth[i].x-swx, dy=g_smooth[i].y-swy, d=dx*dx+dy*dy;
                    if(d<bd){bd=d;bi=(int)i;}
                }
                float dx=g_smooth[bi].x-swx, dy=g_smooth[bi].y-swy;
                yaw = atan2f(-dx, dy);
            }
            g_est_start = {swx, swy, yaw, true};
            save_est_start();
            g_est_start_mode = false;
            if(!g_confirmed_start.valid) g_pp_robot = g_est_start;   // use it now (pre-localize)
        }

        // ── PATH EDIT SELECTION (plain LMB in Step 2 while the Edit Path tool is on) ──
        // Two clicks pick the [i0,i1] stretch of g_smooth to nudge; a 3rd click restarts.
        if(g_path_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT) &&
           !g_smooth.empty()){
            Vector2 mpe = GetMousePosition();
            float ewx = view2d.cx + (mpe.x - view2d.sw*0.5f) / view2d.scale;
            float ewy = view2d.cy - (mpe.y - view2d.sh*0.5f) / view2d.scale;
            int best=0; float bd=1e30f;
            for(int i=0;i<(int)g_smooth.size();i++){
                float dx=g_smooth[i].x-ewx, dy=g_smooth[i].y-ewy, d=dx*dx+dy*dy;
                if(d<bd){bd=d;best=i;}
            }
            if(g_path_edit_i0<0 || (g_path_edit_i0>=0 && g_path_edit_i1>=0)){
                g_path_edit_i0=best; g_path_edit_i1=-1; g_path_edit_offset=0.f;  // start / restart
            } else {
                g_path_edit_i1=best;
            }
        }

        // ── DANGER ZONE SELECTION (plain LMB in Step 2 while the Set Danger Zone tool is on) ──
        // Two clicks pick the start/end of the danger zone; a 3rd click restarts. Snaps
        // to the nearest point across BOTH zone 1 and zone 2's corridors (not just
        // whichever is currently armed/loaded into g_smooth) — the danger stretch can
        // straddle the zone boundary, so zone 1 must be pickable even while zone 2 is active.
        if(g_dz_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mpz = GetMousePosition();
            float zwx = view2d.cx + (mpz.x - view2d.sw*0.5f) / view2d.scale;
            float zwy = view2d.cy - (mpz.y - view2d.sh*0.5f) / view2d.scale;
            int zone, idx;
            bool pickingSecond = (g_dz_edit_idx0>=0 && g_dz_edit_idx1<0);
            int avoidZone = pickingSecond ? g_dz_edit_zone0 : 0;
            if(pp_nearest_either_zone(zwx, zwy, zone, idx, avoidZone)){
                if(g_dz_edit_idx0<0 || (g_dz_edit_idx0>=0 && g_dz_edit_idx1>=0)){
                    g_dz_edit_zone0=zone; g_dz_edit_idx0=idx; g_dz_edit_zone1=-1; g_dz_edit_idx1=-1;  // start / restart
                } else {
                    g_dz_edit_zone1=zone; g_dz_edit_idx1=idx;
                }
            }
        }

        // ── SET SUB-ZONE (either danger-zone test map — plain LMB in Step 2 while tool is
        // on) ── Two clicks snapped to the nearest point on g_smooth (whichever test map's
        // plain recorded path is currently loaded — no zone corridors here); a 3rd click
        // restarts, mirroring the danger-zone tool above.
        if(g_dzsub_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT) && !g_smooth.empty()){
            Vector2 mpz = GetMousePosition();
            float zwx = view2d.cx + (mpz.x - view2d.sw*0.5f) / view2d.scale;
            float zwy = view2d.cy - (mpz.y - view2d.sh*0.5f) / view2d.scale;
            int bi=0; float bd=1e18f;
            for(size_t i=0;i<g_smooth.size();i++){
                float dx=g_smooth[i].x-zwx, dy=g_smooth[i].y-zwy, d=dx*dx+dy*dy;
                if(d<bd){bd=d;bi=(int)i;}
            }
            if(!g_dzsub_edit_have0 || g_dzsub_edit_have1){
                g_dzsub_edit_x0=g_smooth[bi].x; g_dzsub_edit_y0=g_smooth[bi].y;  // start / restart
                g_dzsub_edit_have0=true; g_dzsub_edit_have1=false;
            } else {
                g_dzsub_edit_x1=g_smooth[bi].x; g_dzsub_edit_y1=g_smooth[bi].y;
                g_dzsub_edit_have1=true;
            }
        }

        // ── SET ZONE 2 SLOW SEGMENT (top-LiDAR maps — plain LMB in Step 2 while tool
        // is on) ── Two clicks snapped to zone 2's recorded corridor (pp_zone_corridor(2),
        // NOT g_smooth — so this works no matter which zone is currently armed/loaded);
        // a 3rd click restarts, mirroring the sub-zone tool above.
        if(g_z2slow_edit_mode && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            const std::vector<PPPoint>& z2c = pp_zone_corridor(2);
            if(!z2c.empty()){
                Vector2 mpz3 = GetMousePosition();
                float zwx3 = view2d.cx + (mpz3.x - view2d.sw*0.5f) / view2d.scale;
                float zwy3 = view2d.cy - (mpz3.y - view2d.sh*0.5f) / view2d.scale;
                int bi3=0; float bd3=1e18f;
                for(size_t i=0;i<z2c.size();i++){
                    float dx=z2c[i].x-zwx3, dy=z2c[i].y-zwy3, d=dx*dx+dy*dy;
                    if(d<bd3){bd3=d;bi3=(int)i;}
                }
                if(!g_z2slow_edit_have0 || g_z2slow_edit_have1){
                    g_z2slow_edit_x0=z2c[bi3].x; g_z2slow_edit_y0=z2c[bi3].y;
                    g_z2slow_edit_have0=true; g_z2slow_edit_have1=false;
                } else {
                    g_z2slow_edit_x1=z2c[bi3].x; g_z2slow_edit_y1=z2c[bi3].y;
                    g_z2slow_edit_have1=true;
                }
            }
        }

        // ── OBSTACLE TEST MODE + BORDER CELL EDIT (plain LMB, Step 2 OR Step 3) ──
        // Step 3 is a split screen: the technical map is rendered into an offset
        // RenderTexture from leftW to leftW+rightMapW (see the STEP_DRIVE draw block
        // below), narrower than and offset from the full-width mapW/view2d.sw used
        // for Step 1/2 and set earlier this frame (line ~899). A click handler using
        // the plain view2d.sw/mapW math (as all the OTHER map click handlers in this
        // file correctly do — they're all Step-2-only, so they never hit this) would
        // silently resolve to the wrong world position in Step 3, or accept clicks on
        // the LEFT friendly panel too since that's also < mapW. Recompute the same
        // leftW/rightMapW/rMapH the draw block uses and remap the click into that
        // texture-local space before converting to world coordinates.
        if((g_obstacle_test_mode || g_border_edit_mode) && (step==STEP_PLAN||step==STEP_DRIVE) &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mpo = GetMousePosition();
            bool inMapArea; float localX, localY, effSw, effSh;
            if(step==STEP_DRIVE){
                int leftW3 = sw/2;
                int rightMapW3 = std::max(200, sw - leftW3 - PANEL_W);
                int rMapH3 = sh - BAR_H;
                inMapArea = (mpo.x>=leftW3 && mpo.x<leftW3+rightMapW3 && mpo.y>=0 && mpo.y<rMapH3);
                localX = mpo.x - leftW3; localY = mpo.y;
                effSw = (float)rightMapW3; effSh = (float)rMapH3;
            } else {
                inMapArea = (mpo.x < mapW);
                localX = mpo.x; localY = mpo.y;
                effSw = view2d.sw; effSh = view2d.sh;
            }
            if(inMapArea){
                float owx = view2d.cx + (localX - effSw*0.5f) / view2d.scale;
                float owy = view2d.cy - (localY - effSh*0.5f) / view2d.scale;
                // !other_mode is defensive — the two toggle buttons already enforce
                // mutual exclusion, but a click handler shouldn't rely solely on UI state.
                if(g_obstacle_test_mode && !g_border_edit_mode) grid_click_cycle(owx, owy);
                else if(g_border_edit_mode && !g_obstacle_test_mode) border_click_toggle(owx, owy);
            }
        }

        // ── DEST GRID CELL PAINT (plain LMB in Step 2 while in grid edit mode) ──
        if(g_dest_grid_edit && step==STEP_PLAN &&
           IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mp=GetMousePosition();
            float wx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
            float wy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
            int col=(int)floorf(wx/DEST_GRID_CELL_M);
            int row=(int)floorf(wy/DEST_GRID_CELL_M);
            auto key=std::make_pair(col,row);
            if(g_dest_grid_paint_dest<0){
                g_dest_grid.erase(key);  // eraser
            } else {
                auto it=g_dest_grid.find(key);
                // Toggle off if same dest already painted, else paint/repaint
                if(it!=g_dest_grid.end()&&it->second==g_dest_grid_paint_dest)
                    g_dest_grid.erase(key);
                else
                    g_dest_grid[key]=g_dest_grid_paint_dest;
            }
        }

        // ── DEST GRID CELL TAP (plain LMB in normal Step 2 — one tap to navigate) ──
        // Only fires when no edit/setup/draw mode is active and click lands on a painted cell.
        if(!g_dest_grid_edit && !g_dest_setup_mode && !g_polygon_edit_mode &&
           !g_zone_edit_mode && !g_zone2_edit_mode && !g_est_start_mode && !g_path_edit_mode &&
           !g_dz_edit_mode &&
           step==STEP_PLAN && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
           GetMousePosition().x<mapW &&
           !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_LEFT_SHIFT)){
            Vector2 mp=GetMousePosition();
            float wx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
            float wy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
            int col=(int)floorf(wx/DEST_GRID_CELL_M);
            int row=(int)floorf(wy/DEST_GRID_CELL_M);
            auto it=g_dest_grid.find({col,row});
            if(it!=g_dest_grid.end()){
                const char* dname; SavedDest resolved;
                if(resolve_combined_dest(it->second, dname, resolved)){
                    g_pp_goal={resolved.x,resolved.y,0,true};
                    pp_trigger_plan();
                }
            }
        }

        // ── GLOBAL RELOC (Enter in Step 1) ──
        // Seeds at current EKF position (where the LiDAR is scanning in world frame).
        if(step==STEP_LOCALIZE&&IsKeyPressed(KEY_ENTER)&&fus_state==STATE_ALIGNMENT_CHECK){
            std::lock_guard<std::mutex>lk(g_reloc_mutex);
            if(!g_reloc_running&&active_map_ptr&&active_map_ptr->loaded&&fus_state==STATE_ALIGNMENT_CHECK){
                g_reloc_done=false;g_reloc_running=true;g_reloc_progress=0;g_reloc_error_msg="";
                std::vector<Point3D>snap;{std::lock_guard<std::mutex>lk2(lidar_mutex);snap=dense_cloud;}
                float seed_r=(active_map_ptr->is_large)?SEED_RADIUS_OUTDOOR:SEED_RADIUS_INDOOR;
                float sx=0.f, sy=0.f;
                float zcx,zcy,zr;
                if(g_zone_armed && g_zone_valid && zone_centroid_radius(g_zone_poly,zcx,zcy,zr)){
                    // Focus the seed inside the armed zone: centroid + covering radius.
                    sx=zcx; sy=zcy; seed_r=zr+2.f;
                } else if(g_zone2_armed && g_zone2_valid && zone_centroid_radius(g_zone2_poly,zcx,zcy,zr)){
                    sx=zcx; sy=zcy; seed_r=zr+2.f;
                } else {
                    std::lock_guard<std::mutex>lk_ekf(ekf_mutex); sx=(float)ekf.x[0]; sy=(float)ekf.x[1];
                }
                std::thread(particle_filter_reloc_thread,std::move(snap),&active_map_ptr->dist_map,
                            active_map_ptr->is_large,active_map_ptr->inlier_dist,true,sx,sy,seed_r).detach();
                fus_state=STATE_GLOBAL_RELOC;
            }
        }

        // ── DANGER-ZONE CROSSING: entry/exit map-switch + relocalize orchestration ──
        // (2026-07-07) See g_dzx_state's declaration above for the full journey design.
        // Both branches below only run while the cart is stopped (control.cpp only sets
        // these pending flags from the CS_PAUSED goal-reached branch), so there's no
        // control-loop activity mid-switch — matches how [M] map-switching already
        // works, just triggered by reaching a checkpoint instead of a keypress.
        if(g_dzx_pending_enter){
            g_dzx_pending_enter=false;
            switch_active_map_by_name("Danger Zone Fwd");
            // Danger-zone test maps have no zone to arm (that's what loads g_smooth for
            // the campus corner maps) — load the recorded route + its sub-zone markers
            // explicitly, same as the [M] handler does for these two maps.
            pp_run_pipeline(TRAJ_CSV_FILE);
            load_dzsub_fwd();
            // Seed the relocalization search near the sub-zone entry marker — the stale
            // EKF position from Campus Top LiDAR is a DIFFERENT, unrelated coordinate
            // frame (no known transform) and would misdirect a default-seeded search.
            g_localize_use_custom_seed=true;
            g_localize_seed_x=g_dzsub_entry_mk_x; g_localize_seed_y=g_dzsub_entry_mk_y;
            g_localize_seed_r=SEED_RADIUS_OUTDOOR*1.5f;
            fus_state=STATE_ALIGNMENT_CHECK;
            g_localize_request=true;
            g_dzx_state=DZX_WAIT_ENTER_RELOC;
            g_statusMsg="Switched to danger-zone map - relocalizing...";
            g_statusCol=C_WARN;
        }
        if(g_dzx_pending_exit){
            g_dzx_pending_exit=false;
            switch_active_map_by_name("Campus Top LiDAR");
            // Seed near the danger zone's actual EXIT point in THIS map's own frame —
            // the last point of danger_zone_forward.csv (the recorded curve's own
            // coordinates here, unrelated to the Danger Zone Fwd map's frame it was
            // just driven on). Without this the search fell back to zone 1's broad
            // centroid (or worse, the stale EKF pose left over from the OTHER map's
            // frame, since switch_active_map_by_name doesn't touch ekf.x) — too
            // imprecise a seed, which is almost certainly why relocalization never
            // actually completed (confirmed: zone 2 never got armed downstream,
            // meaning the "relocalization finished" branch below never ran at all).
            {
                std::ifstream f("danger_zone_forward.csv");
                std::string line, lastLine;
                if(f.is_open()){
                    std::getline(f,line); // header
                    while(std::getline(f,line)) if(!line.empty()) lastLine=line;
                }
                float ex=0.f, ey=0.f; bool haveExit=false;
                if(!lastLine.empty()){
                    std::stringstream ss(lastLine); char c;
                    if(ss>>ex>>c>>ey) haveExit=true;
                }
                if(haveExit){
                    g_localize_use_custom_seed=true;
                    g_localize_seed_x=ex; g_localize_seed_y=ey;
                    g_localize_seed_r=SEED_RADIUS_OUTDOOR*1.5f;
                }
            }
            g_zone_armed=true; g_zone2_armed=false;
            fus_state=STATE_ALIGNMENT_CHECK;
            g_localize_request=true;
            g_dzx_state=DZX_WAIT_EXIT_RELOC;
            g_statusMsg="Switched back to main map - relocalizing...";
            g_statusCol=C_WARN;
        }
        // Once either leg's relocalization completes (fus_state reaches FUSION_RUNNING),
        // auto-build the NEXT leg's plan and load it — the operator still presses Start
        // Autonomous to actually move, same as every other stop in this codebase (no
        // auto-continue).
        if((g_dzx_state==DZX_WAIT_ENTER_RELOC || g_dzx_state==DZX_WAIT_EXIT_RELOC) &&
           fus_state==STATE_FUSION_RUNNING){
            if(ekf.seeded){
                std::lock_guard<std::mutex> lk(ekf_mutex);
                g_pp_robot={(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],true};
                g_confirmed_start={g_pp_robot.x,g_pp_robot.y,g_pp_robot.yaw,true};
            }
            if(g_dzx_state==DZX_WAIT_ENTER_RELOC){
                // Drive the danger-zone map's own recorded route end-to-end — its own
                // g_dzsub_active sub-zone mechanism (unchanged) handles the hard stretch
                // internally; this journey layer just needs SOME goal to plan+drive
                // toward, so it uses that map's own recorded endpoint.
                if(!g_smooth.empty()){
                    g_pp_goal={g_smooth.back().x, g_smooth.back().y, 0, true};
                    pp_trigger_plan();
                    if(!g_approach_path.empty()||!g_corridor_path.empty()){ pp_save_plan(); ctrl_doLoad(); }
                    g_statusMsg="Located on danger-zone map - press Start Autonomous to cross.";
                    g_statusCol=C_GOOD;
                    g_dzx_state=DZX_IN_DZMAP;
                } else {
                    g_statusMsg="Danger-zone map has no recorded path to drive - crossing aborted.";
                    g_statusCol=C_BAD;
                    g_dzx_state=DZX_NONE;
                }
            } else {
                // Final leg: the passenger's real destination, normal zone-2 routing.
                g_zone_armed=false; g_zone2_armed=true;
                pp_run_pipeline(TRAJ_CSV_FILE);
                g_pp_goal={g_dzx_final_x, g_dzx_final_y, 0, true};
                pp_trigger_plan();
                if(!g_approach_path.empty()||!g_corridor_path.empty()){ pp_save_plan(); ctrl_doLoad(); }
                g_statusMsg=TextFormat("Located - press Start Autonomous to continue to %s.", g_dzx_final_name.c_str());
                g_statusCol=C_GOOD;
                g_dzx_state=DZX_NONE;
            }
        }

        // ── LOCALIZE PANEL REQUEST (passenger "Find My Cart" button) ──
        // Same global relocalization as Enter, but triggered from the friendly panel
        // and independent of the current step.
        if(g_localize_request){
            g_localize_request=false;
            std::lock_guard<std::mutex>lk(g_reloc_mutex);
            if(!g_reloc_running && active_map_ptr && active_map_ptr->loaded && fus_state==STATE_ALIGNMENT_CHECK){
                g_reloc_done=false;g_reloc_running=true;g_reloc_progress=0;g_reloc_error_msg="";
                std::vector<Point3D>snap;{std::lock_guard<std::mutex>lk2(lidar_mutex);snap=dense_cloud;}
                float seed_r=(active_map_ptr->is_large)?SEED_RADIUS_OUTDOOR:SEED_RADIUS_INDOOR;
                float sx=0.f, sy=0.f;
                if(g_localize_use_custom_seed){
                    // Circle position set by the passenger in the localize panel
                    sx=g_localize_seed_x; sy=g_localize_seed_y; seed_r=g_localize_seed_r;
                    g_localize_use_custom_seed=false;
                } else {
                    float zcx,zcy,zr;
                    if(g_zone_armed && g_zone_valid && zone_centroid_radius(g_zone_poly,zcx,zcy,zr)){
                        sx=zcx; sy=zcy; seed_r=zr+2.f;
                    } else if(g_zone2_armed && g_zone2_valid && zone_centroid_radius(g_zone2_poly,zcx,zcy,zr)){
                        sx=zcx; sy=zcy; seed_r=zr+2.f;
                    } else {
                        std::lock_guard<std::mutex>lk_ekf(ekf_mutex); sx=(float)ekf.x[0]; sy=(float)ekf.x[1];
                    }
                }
                std::thread(particle_filter_reloc_thread,std::move(snap),&active_map_ptr->dist_map,
                            active_map_ptr->is_large,active_map_ptr->inlier_dist,true,sx,sy,seed_r).detach();
                fus_state=STATE_GLOBAL_RELOC;
            }
        }

        // ── FUSION STATE MACHINE ──
        if(fus_state==STATE_CALIBRATING){
            calib_timer+=dt;
            if(calib_timer>=3.f){
                std::lock_guard<std::mutex>lk(imu_mutex);
                calib_state.is_calibrating=false;
                if(calib_state.samples>0){
                    g_to_subtract.x=(float)(calib_state.sum_acc[0]/calib_state.samples);
                    g_to_subtract.y=(float)(calib_state.sum_acc[1]/calib_state.samples);
                    g_to_subtract.z=(float)(calib_state.sum_acc[2]/calib_state.samples);
                    gyro_bias_init.x=(float)(calib_state.sum_gyro[0]/calib_state.samples);
                    gyro_bias_init.y=(float)(calib_state.sum_gyro[1]/calib_state.samples);
                    gyro_bias_init.z=(float)(calib_state.sum_gyro[2]/calib_state.samples);
                    std::lock_guard<std::mutex>lk2(ekf_mutex); ekf.x[6]=0; ekf.x[7]=0;
                }
                fus_state=STATE_ALIGNMENT_CHECK;
            }
        } else if(fus_state==STATE_GLOBAL_RELOC){
            bool done=false; RelocResult res={};
            {std::lock_guard<std::mutex>lk(g_reloc_mutex);done=g_reloc_done;res=g_reloc;}
            if(done){
                float accept=active_map_ptr?active_map_ptr->seed_accept:0.55f;
                if(res.score>=accept&&res.abs_inliers>=MIN_ABS_INLIERS){
                    std::lock_guard<std::mutex>lk(ekf_mutex);
                    ekf.x[0]=res.x;ekf.x[1]=res.y;ekf.x[2]=res.yaw;
                    ekf.x[3]=ekf.x[4]=0;ekf.seeded=true;
                    last_processed_sweep=lidar_sweeps_total.load();
                    {std::lock_guard<std::mutex>lk2(trajectory_mutex);
                     robot_trajectory.clear();
                     robot_trajectory.push_back({(float)ekf.x[0],0.2f,-(float)ekf.x[1]});}
                    {std::lock_guard<std::mutex>lk3(imu_mutex);g_imu_yaw_at_localize=imu_angle.z;}
                    // Auto-persist the real localized pose as this map's estimated start
                    // point. If the cart is found somewhere different next time (moved,
                    // parked elsewhere, etc.), that new spot becomes the default — instead
                    // of trusting a manually-set point that silently goes stale. Same
                    // success point covers every localization on every map (technical
                    // Confirm Pose, passenger Find My Cart, danger-zone entry/exit legs).
                    g_est_start = {res.x, res.y, res.yaw, true};
                    save_est_start();
                    // Auto-arm whichever zone the real localized pose lands inside — this
                    // is the ONE success point shared by both the technical "Confirm Pose"
                    // flow AND the passenger "Find My Cart" flow, so arming now happens
                    // regardless of which one localized. Previously this only lived in the
                    // Confirm-Pose button handler (ui.cpp), so the passenger flow never
                    // armed a zone at all → corner_active stayed false → planning fell back
                    // to a raw direct line instead of following the recorded corridor.
                    bool zoneArmedNow=false;
                    if(g_zone_valid && !g_zone_poly.empty() && point_in_polygon((float)res.x,(float)res.y,g_zone_poly)){
                        g_zone_armed = true; g_zone2_armed = false; zoneArmedNow=true;
                    } else if(g_zone2_valid && !g_zone2_poly.empty() && point_in_polygon((float)res.x,(float)res.y,g_zone2_poly)){
                        g_zone2_armed = true; g_zone_armed = false; zoneArmedNow=true;
                    } else {
                        // Fallback: the hand-drawn zone polygon doesn't reliably contain
                        // the actual recorded corridor points — a pose genuinely on/near
                        // the road can fall just outside it and silently arm nothing. If
                        // that happens, arm by which corridor the pose is actually
                        // nearest to (within 15 m).
                        int cz = pp_zone_for_point((float)res.x, (float)res.y, 15.f);
                        if(cz==0 && g_zone_valid){ g_zone_armed=true; g_zone2_armed=false; zoneArmedNow=true; }
                        else if(cz==1 && g_zone2_valid){ g_zone2_armed=true; g_zone_armed=false; zoneArmedNow=true; }
                    }
                    // g_smooth (the actual driven corridor) only updates via
                    // pp_run_pipeline() — without this, a newly-armed zone's corner file
                    // never gets loaded and single-zone routing would use a stale/empty
                    // g_smooth. The technical "Confirm Pose" button already did this
                    // itself; this success point needs its own call for the passenger flow.
                    if(zoneArmedNow) pp_run_pipeline(TRAJ_CSV_FILE);
                    fus_state=STATE_FUSION_RUNNING;
                } else {
                    g_reloc_error_msg=TextFormat("FAILED: %d%% overlap, %d inliers",(int)(res.score*100),res.abs_inliers);
                    fus_state=STATE_ALIGNMENT_CHECK;
                }
            }
        } else if(fus_state==STATE_FUSION_RUNNING){
            int cur_sw=lidar_sweeps_total.load();
            if(cur_sw>last_processed_sweep&&active_map_ptr){
                last_processed_sweep=cur_sw;
                if(!matcher_request.load()){
                    std::lock_guard<std::mutex>lk(matcher_mutex);
                    std::lock_guard<std::mutex>lk2(lidar_mutex);
                    matcher_cloud.clear();
                    for(size_t i=0;i<dense_cloud.size();i+=2) matcher_cloud.push_back(dense_cloud[i]);
                    matcher_request.store(true);
                }
            }
        }

        // ── ZONE FOCUS (arming a zone centers the view on it) ──
        if(g_zone_focus_request && g_zone_valid){
            float cx,cy,r;
            if(zone_centroid_radius(g_zone_poly,cx,cy,r)){
                camera.target = {cx, 0.f, -cy};           // Step 1 3D view (map y → -z)
                cam_radius = r*2.2f + 10.f;                 // zoom to roughly fit the zone
                view2d.cx = cx; view2d.cy = cy;            // Step 2 2D view
                view2d.scale = clampf((float)mapH/(2.f*r+8.f), 2.f, 60.f);
            }
            g_zone_focus_request = false;
        }
        if(g_zone2_focus_request && g_zone2_valid){
            float cx,cy,r;
            if(zone_centroid_radius(g_zone2_poly,cx,cy,r)){
                camera.target = {cx, 0.f, -cy};
                cam_radius = r*2.2f + 10.f;
                view2d.cx = cx; view2d.cy = cy;
                view2d.scale = clampf((float)mapH/(2.f*r+8.f), 2.f, 60.f);
            }
            g_zone2_focus_request = false;
        }

        // ── CAMERA FOLLOW ──
        if(camera_follow&&(fus_state==STATE_FUSION_RUNNING||fus_state==STATE_GLOBAL_RELOC)){
            double ex2=0,ey2=0;{std::lock_guard<std::mutex>lk(ekf_mutex);ex2=ekf.x[0];ey2=ekf.x[1];}
            camera.target.x=Lerp(camera.target.x,(float)ex2,0.1f);
            camera.target.z=Lerp(camera.target.z,-(float)ey2,0.1f);
            // position is recomputed from spherical coords below
        }

        // ── 3D CAMERA POSITION FROM SPHERICAL COORDS ──
        {
            float se=sinf(cam_elevation), ce=cosf(cam_elevation);
            float sa=sinf(cam_azimuth),   ca=cosf(cam_azimuth);
            camera.position.x=camera.target.x+cam_radius*ce*sa;
            camera.position.y=cam_radius*se;
            camera.position.z=camera.target.z+cam_radius*ce*ca;
            // Up vector: keep world map +Y direction (Raylib -Z) as screen-up at all angles
            Vector3 view=Vector3Normalize({camera.target.x-camera.position.x,
                                           camera.target.y-camera.position.y,
                                           camera.target.z-camera.position.z});
            Vector3 wup={0,0,-1};
            float dot=Vector3DotProduct(wup,view);
            Vector3 up_raw={wup.x-dot*view.x, wup.y-dot*view.y, wup.z-dot*view.z};
            camera.up=Vector3Normalize(up_raw);
        }
        if(camera_follow&&step==STEP_DRIVE&&fus_state==STATE_FUSION_RUNNING){
            double ex2=0,ey2=0;{std::lock_guard<std::mutex>lk(ekf_mutex);ex2=ekf.x[0];ey2=ekf.x[1];}
            view2d.cx=Lerp(view2d.cx,(float)ex2,0.18f);
            view2d.cy=Lerp(view2d.cy,(float)ey2,0.18f);
        }

        // ── PLAN STEP: keep the planning pose synced to the LIVE localized EKF pose ──
        // Whenever the cart is actually localized RIGHT NOW (fus_state==FUSION_RUNNING)
        // and we're in Step 2, track the real pose and replan on any real move.
        // NOTE: this used to gate on "ekf.seeded", which is STICKY — it stays true from
        // any earlier localization (e.g. an earlier Danger Zone map test) even after
        // switching to a different map/session where nothing has relocalized yet. That
        // dragged the pre-localize preview marker (g_pp_robot) onto the stale old EKF
        // position instead of leaving it at the manually-set map start (g_est_start).
        // fus_state==STATE_FUSION_RUNNING is the correct "currently live" signal (same
        // fix as DrawDriveMapPanel's follow-camera/marker gating). The 0.10 m / 0.05 rad
        // threshold keeps it from replanning on EKF jitter while stationary, and replans
        // are cheap now (cached corridors). If not currently localized, the manually-set
        // start pose (or g_est_start) is left untouched.
        if(step==STEP_PLAN && fus_state==STATE_FUSION_RUNNING){
            std::lock_guard<std::mutex>lk(ekf_mutex);
            float nx=(float)ekf.x[0], ny=(float)ekf.x[1], nyaw=(float)ekf.x[2];
            float dx=nx-g_pp_robot.x, dy=ny-g_pp_robot.y;
            bool moved=!g_pp_robot.valid||sqrtf(dx*dx+dy*dy)>0.10f||fabsf(wrapA(nyaw-g_pp_robot.yaw))>0.05f;
            g_pp_robot={nx,ny,nyaw,true};
            if(moved&&g_pp_goal.valid) pp_trigger_plan();
        }

        // ── DRAW ──
        BeginDrawing();
        Texture2D g_bg_active = g_bg_show_3d ? g_bg_3d : g_bg_sat;
        // Reset localize-panel circle when the panel opens fresh
        { static bool was_loc=false;
          if(!was_loc && g_show_localize) g_localize_panel_needs_init=true;
          was_loc=g_show_localize; }
        // "Another journey?" card — fires once when the cart reaches its final
        // destination via the passenger flow (not on technical-view test drives,
        // same g_drive_via_passenger gate used for the danger-zone-exit panel
        // routing above). Edge-triggered so it only pops up the moment CS_DONE is
        // first reached, not every frame afterward.
        { static bool was_done=false;
          bool nowDone = (g_ctrl_state==CS_DONE);
          if(!was_done && nowDone && g_drive_via_passenger) g_show_journey_end=true;
          was_done=nowDone; }
        if(g_show_welcome){
            ClearBackground(BLACK);
            DrawWelcomePanel(sw, sh, g_welcome_img, g_ui_font, g_show_welcome, g_show_intro);
            EndDrawing();
            continue;
        }
        if(g_show_intro){
            ClearBackground(BLACK);
            DrawIntroPanel(sw, sh, g_bg_active, g_intro_logo, g_cart_icon, g_ui_font, g_show_intro, g_intro_remote, g_show_dest);
            EndDrawing();
            continue;
        }
        if(g_show_map_select){
            ClearBackground(BLACK);
            DrawMapSelectPanel(sw, sh, g_bg_active, g_ui_font, g_show_map_select);
            EndDrawing();
            continue;
        }
        if(g_show_localize){
            // Calibration entered from the technical panel: once cal mode ends (Done or
            // Save Calibration inside DrawLocalizePanel sets g_localize_cal_mode=false),
            // drop straight back to the technical view instead of sitting in the
            // passenger Localize panel — this path never went through the passenger
            // flow, so its "Back" button (-> Intro) wouldn't get the operator anywhere
            // useful.
            if(g_localize_cal_from_technical && !g_localize_cal_mode){
                g_show_localize=false; g_localize_cal_from_technical=false;
            } else {
                ClearBackground(BLACK);
                DrawLocalizePanel(sw, sh, g_bg_active, g_ui_font, fus_state, g_show_localize);
                EndDrawing();
                continue;
            }
        }
        if(g_show_dest){
            ClearBackground(BLACK);
            DrawDestinationPanel(sw, sh, g_bg_active, g_ui_font, g_show_dest, fus_state);
            EndDrawing();
            continue;
        }
        if(g_show_journey_end){
            ClearBackground(BLACK);
            DrawJourneyEndPanel(sw, sh, g_bg_active, g_ui_font, g_show_journey_end,
                                 g_show_welcome, g_show_intro, g_show_map_select,
                                 g_show_localize, g_show_dest);
            EndDrawing();
            continue;
        }
        ClearBackground(C_BG);

        if(step==STEP_DRIVE){
            // ── Split screen: friendly campus-photo view (left half) + technical Step
            // 3 view (right half) ── replaces the old full-screen Step 3, so both are
            // visible together while driving, per the 2026-07-08 request.
            int leftW = sw/2;
            int rightMapW = std::max(200, sw - leftW - PANEL_W);
            int rMapH = sh - BAR_H;

            if(step3MapRT.texture.width != rightMapW || step3MapRT.texture.height != rMapH){
                if(step3MapRT.id != 0) UnloadRenderTexture(step3MapRT);
                step3MapRT = LoadRenderTexture(rightMapW, rMapH);
            }

            // Render the map viewport at ITS OWN local width (rightMapW), same
            // conventions DrawMapViewport already uses when that width is the whole
            // window — just narrower, so nothing scales/distorts. No interactive
            // elements live inside DrawMapViewport (all click handling happens earlier
            // in the frame, in the input section, not here), so rendering it off-
            // screen like this can't break any hit-testing.
            float saved_sw=view2d.sw, saved_sh=view2d.sh;
            view2d.sw=(float)rightMapW; view2d.sh=(float)rMapH;
            BeginTextureMode(step3MapRT);
            ClearBackground(C_BG);
            DrawMapViewport(rightMapW, rMapH, TITLE_H, PANEL_W, BAR_H, step, camera, view2d, camera_follow, fus_state);
            EndTextureMode();
            view2d.sw=saved_sw; view2d.sh=saved_sh;

            // Left half: friendly campus-photo view + live car marker.
            DrawDriveMapPanel(0, TITLE_H, leftW, sh-BAR_H-TITLE_H, g_bg_active, g_ui_font, fus_state);

            // Right half: blit the map-viewport texture (RenderTexture2D content is
            // Y-flipped in raylib — negative source height corrects it), then the
            // drive panel drawn DIRECTLY to the real framebuffer (not through the
            // texture) so its Start/Stop button hit-tests, which compare against real
            // GetMousePosition(), land on the same real screen coordinates the
            // buttons are actually drawn at.
            DrawTextureRec(step3MapRT.texture, {0,0,(float)rightMapW,-(float)rMapH}, {(float)leftW,0}, WHITE);
            DrawPanelDrive(leftW+rightMapW, TITLE_H, PANEL_W, rMapH-TITLE_H);
            DrawRectangle(leftW-2, TITLE_H, 4, sh-BAR_H-TITLE_H, C_BORDER_HI); // divider

            // Title bar drawn AFTER the blit (on top) — the texture clears its own
            // y<TITLE_H strip to C_BG, which would otherwise paint over an
            // already-drawn title bar in that region.
            DrawRectangle(0,0,sw,TITLE_H,C_PANEL_DARK);
            DrawRectangle(0,TITLE_H-1,sw,1,C_BORDER_HI);
            DrawText("AUTONOMOUS GOLF CART",16,14,20,C_HEADER);
            {const char* sn="STEP 3 — DRIVE"; int tw=MeasureText(sn,16); DrawText(sn,sw/2-tw/2,16,16,C_GOOD);}
            DrawText(TextFormat("LiDAR %d Hz  IMU %d Hz",cur_lidar_hz,cur_imu_hz),
                     sw-220,16,13,cur_lidar_hz>5?C_GOOD:C_BAD);
            DrawFPS(sw-60,4);
        } else {
            // Title bar
            DrawRectangle(0,0,sw,TITLE_H,C_PANEL_DARK);
            DrawRectangle(0,TITLE_H-1,sw,1,C_BORDER_HI);
            DrawText("AUTONOMOUS GOLF CART",16,14,20,C_HEADER);
            const char* stepNames[]={"STEP 1 — LOCALIZE","STEP 2 — PLAN ROUTE","STEP 3 — DRIVE"};
            Color stepCols[]={C_HEADER,C_WARN,C_GOOD};
            {int si=step-1; int tw=MeasureText(stepNames[si],16); DrawText(stepNames[si],sw/2-tw/2,16,16,stepCols[si]);}
            DrawText(TextFormat("LiDAR %d Hz  IMU %d Hz",cur_lidar_hz,cur_imu_hz),
                     sw-220,16,13,cur_lidar_hz>5?C_GOOD:C_BAD);
            DrawFPS(sw-60,4);

            // Map viewport
            DrawMapViewport(mapW, mapH, TITLE_H, PANEL_W, BAR_H, step, camera, view2d, camera_follow, fus_state);

            // Seed radius overlay in 3D
            if(step==STEP_LOCALIZE&&IsKeyDown(KEY_LEFT_SHIFT)){
                BeginMode3D(camera);
                Ray ray=GetMouseRay(GetMousePosition(),camera);
                float t2=-ray.position.y/ray.direction.y;
                if(t2>0){
                    float sx=ray.position.x+ray.direction.x*t2, sy2=-(ray.position.z+ray.direction.z*t2);
                    DrawCircle3D({sx,0.2f,-sy2},cur_seed_r,{1,0,0},90,Fade(LIME,0.3f));
                    DrawCylinderWires({sx,0.2f,-sy2},cur_seed_r,cur_seed_r,0.05f,32,LIME);
                }
                EndMode3D();
            }

            // Right panel
            if(step==STEP_LOCALIZE)
                DrawPanelLocalize(mapW,TITLE_H,PANEL_W,mapH-TITLE_H,fus_state,calib_timer,cur_lidar_hz,cur_imu_hz);
            else
                DrawPanelPlan(mapW,TITLE_H,PANEL_W,mapH-TITLE_H);
        }

        DrawStepBar(sh-BAR_H, sw);
        EndDrawing();
    }

    // ── Cleanup ──
    ped_set_enabled(false);   // kill the Python detector process if it's running
    if(g_txSock!=INVALID_SOCKET) closesocket(g_txSock);
    if(g_rxSock!=INVALID_SOCKET) closesocket(g_rxSock);
    WSACleanup();
    ekf_fusion_codegen_terminate();
    if(step3MapRT.id!=0) UnloadRenderTexture(step3MapRT);
    if(g_bg_sat.id>0) UnloadTexture(g_bg_sat);
    if(g_bg_3d.id>0 && g_bg_3d.id!=g_bg_sat.id) UnloadTexture(g_bg_3d);
    if(g_welcome_img.id>0) UnloadTexture(g_welcome_img);
    UnloadTexture(g_intro_logo);
    UnloadTexture(g_cart_icon);
    if(g_ui_font.texture.id != GetFontDefault().texture.id) UnloadFont(g_ui_font);
    CloseWindow();
    return 0;
}