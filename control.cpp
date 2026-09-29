// =============================================================================
// control.cpp  —  Pure Pursuit, steering, speed, watchdog, prediction, 
//                 control loop thread
// =============================================================================
#include "common.hpp"

// forward declarations from communication.cpp
void ctrl_sendCmd(const std::string& cmd);
void logger_start_run();
void logger_write_csv();
void logger_tick(const CtrlPose& robot);
void pollTelemetry();

// forward declarations from planning.cpp
void pp_load_smoothed();
void pp_trigger_plan();
bool dz_nearest(float px, float py, float& outArc, float& outDist);
bool dz_lookup(float arc, float& outX, float& outY);
const std::vector<PPPoint>& pp_danger_zone_pts();

// forward declaration — defined later in this file, used earlier by the pedestrian
// auto-resume check below
void ctrl_doStart();

// =============================================================================
//  GLOBALS (owned here)
// =============================================================================
std::vector<CtrlPathPoint> g_ctrl_path;
CtrlPose  g_ctrl_start = {0,0,0,false};
CtrlPose  g_ctrl_goal  = {0,0,0,false};
CtrlPose  g_ctrl_robot = {0,0,0,false};
CtrlPose  g_ctrl_robot_filtered = {0,0,0,false};
CtrlPose  g_ctrl_sim   = {0,0,0,true};
float     g_ctrl_simSpd = 0.f;
float     g_lookaheadM = LOOKAHEAD_NOM_M;
float     g_dbg_dt_lidar_ms    = 0.f;
float     g_dbg_dt_actuator_ms = 0.f;
float     g_dbg_total_pred_ms  = 0.f;
CtrlState g_ctrl_state = CS_IDLE;
CtrlVars  g_ctrl;
bool      g_simMode = false;
std::string g_statusMsg = "Load plan when ready.";
Color       g_statusCol;
std::deque<std::string> g_cmdLog;
std::mutex              g_cmdLog_mutex;
std::recursive_mutex    g_ctrl_path_mutex;
std::string g_telemStr;
std::vector<Vector2> g_ctrl_trail;
float g_ctrlTimer = 0;
PoseLPF g_pose_lpf;

bool  g_dr_active = false;
float g_dr_x = 0.f, g_dr_y = 0.f;
float g_dr_s = 0.f;
static bool g_dr_was_active = false;   // previous tick's g_dr_active, to detect entry/exit
bool g_dz_return_to_localize = false;  // set on danger-zone exit; main.cpp jumps to the Localize panel

// ── DANGER ZONE TEST MAP inner sub-zone (2026-07-07) ──────────────────────────
// Independent of g_dr_active/the campus curve-replay system above (different maps,
// different behavior) — kept fully separate on purpose so this can't regress the
// just-validated campus danger-zone code. Marks a stretch WITHIN the "Danger Zone
// Fwd" test map's recorded path where even the denser map still loses lock: inside
// it the cart drives dead straight (heading frozen at entry, zero steer, wheel-
// distance only) and ignores the curve entirely; on exit it stops and sends the
// operator back to the Localize panel to manually reacquire, same as the campus
// system's exit — no auto-continue.
bool  g_dzsub_active = false;
float g_dzsub_s = 0.f;   // arc-length progress along the sub-zone stretch (for HUD)
static bool  g_dzsub_was_active = false;
static int   g_dzsub_lo = -1, g_dzsub_hi = -1;
// Sub-zone markers — user-settable via Step 2's "Set Sub-Zone" tool (2 clicks on the
// recorded path), persisted per-map (dzsub_fwd.cfg / dzsub_rev.cfg — one shared pair
// of in-memory globals is fine since only one test map drives at a time, but the file
// must be per-map or switching maps would clobber the other's saved markers). Default
// = 35%/65% along the recorded FWD trajectory (rough "mid of path" per initial field
// report) until set/loaded explicitly; meaningless on Rev until its own file exists.
float g_dzsub_entry_mk_x=-13.30f, g_dzsub_entry_mk_y=30.24f;
float g_dzsub_exit_mk_x = 2.02f,  g_dzsub_exit_mk_y = 47.76f;
static std::string dzsub_cfg_path_for_active_map() {
    if(!active_map_ptr) return "";
    if(active_map_ptr->name=="Danger Zone Fwd") return "dzsub_fwd.cfg";
    if(active_map_ptr->name=="Danger Zone Rev") return "dzsub_rev.cfg";
    return "";
}
void load_dzsub_fwd() {
    std::string path = dzsub_cfg_path_for_active_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    float ex,ey,xx,xy;
    if(f>>ex>>ey>>xx>>xy){ g_dzsub_entry_mk_x=ex; g_dzsub_entry_mk_y=ey; g_dzsub_exit_mk_x=xx; g_dzsub_exit_mk_y=xy; }
}
void save_dzsub_fwd() {
    std::string path = dzsub_cfg_path_for_active_map();
    if(path.empty()) return;
    std::ofstream f(path); if(!f.is_open()) return;
    f << std::fixed << std::setprecision(4)
      << g_dzsub_entry_mk_x << " " << g_dzsub_entry_mk_y << "\n"
      << g_dzsub_exit_mk_x  << " " << g_dzsub_exit_mk_y  << "\n";
}

// Danger-zone plan-index range (which g_ctrl_path indices fall inside the danger
// zone) — set directly by ctrl_splice_danger_zone() (the exact range it just spliced
// in), used by ctrl_step() to gate entry/exit off plan progress instead of live position.
static int   g_dz_path_lo = -1, g_dz_path_hi = -1;
static float g_dz_path_lo_arc = 0.f;

float         g_vel_est_ms     = 1.0f;
float         g_vel_est_prev_x = 0.f;
float         g_vel_est_prev_y = 0.f;
bool          g_vel_est_init   = false;

float g_imu_yaw_drive_start_deg = 0.f;
float g_ekf_yaw_drive_start_rad = 0.f;
bool  g_imu_drive_ref_valid     = false;

std::atomic<bool> g_ctrl_thread_run{true};

// Internal constants
static const int   STARTUP_IMU_TICKS        = 50;
static const float VEL_EST_ALPHA            = 0.30f;
static const float VEL_EST_MAX_MS           = 3.0f;
static const float STEER_TURN_THRESH_DEG    = 4.0f;
static const float STEER_SPEED_HOLD_FRAC    = 1.0f;
static const int   STEER_HOLD_TICKS         = 0;
static const float IMU_YAW_ALPHA_LOW        = 0.02f;
static const float IMU_YAW_ALPHA_HIGH       = 0.10f;
static const float IMU_ALPHA_SPEED_LO       = 0.30f;
static const float IMU_ALPHA_SPEED_HI       = 1.20f;
static const float STEER_EMA_ALPHA          = 0.45f;
static const float STEER_SEND_MIN_DELTA     = 1.0f;

static int g_steer_hold_ticks_remaining = 0;

// =============================================================================
//  PATH LOAD HELPERS
// =============================================================================
static bool ctrl_loadPath() {
    // Read into a local first, THEN swap in under lock (2026-07-19) — keeps the
    // critical section to one assignment instead of a clear()+push_back() loop
    // running while the control thread could be mid-read.
    std::vector<CtrlPathPoint> fresh;
    std::ifstream f(CSV_OUTPUT_FILE); if(!f.is_open()) return false;
    std::string line; std::getline(f,line);
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); CtrlPathPoint p; char c;
        if(ss>>p.x>>c>>p.y>>c>>p.yaw>>c>>p.curvature) fresh.push_back(p);
    }
    {
        std::lock_guard<std::recursive_mutex> lk(g_ctrl_path_mutex);
        g_ctrl_path = std::move(fresh);
    }
    return !g_ctrl_path.empty();
}
static bool ctrl_loadPlan() {
    std::ifstream f("plan_result.txt"); if(!f.is_open()) return false;
    std::string key; float val;
    g_ctrl_start = {};
    while(f>>key>>val){
        if(key=="START_X") g_ctrl_start.x=val;
        if(key=="START_Y") g_ctrl_start.y=val;
        if(key=="START_YAW") g_ctrl_start.yaw=val;
    }
    g_ctrl_start.valid=true; return true;
}

// =============================================================================
//  VELOCITY ESTIMATOR
// =============================================================================
static void vel_est_update(float rx, float ry, float dt_s) {
    if(g_wheel_speed_valid) return;
    if(!g_vel_est_init){ g_vel_est_prev_x=rx; g_vel_est_prev_y=ry; g_vel_est_init=true; return; }
    float dx=rx-g_vel_est_prev_x, dy=ry-g_vel_est_prev_y;
    float raw_v = sqrtf(dx*dx+dy*dy) / std::max(dt_s, 0.001f);
    raw_v = clampf(raw_v, 0.0f, VEL_EST_MAX_MS);
    if(raw_v < 2.5f)
        g_vel_est_ms = VEL_EST_ALPHA * raw_v + (1.f-VEL_EST_ALPHA) * g_vel_est_ms;
    g_vel_est_prev_x=rx; g_vel_est_prev_y=ry;
}

// =============================================================================
//  LOOKAHEAD + PURE PURSUIT
// =============================================================================
static float ctrl_computeLookahead(float /*cte*/, float curv) {
    const float k_v = 0.50f;
    float speed = g_wheel_speed_valid ? g_wheel_speed_mps : g_vel_est_ms;
    float vel_term = k_v * clampf(speed, 0.0f, VEL_EST_MAX_MS);
    float kf = clampf(1.f - fabsf(curv) * 2.0f, 0.0f, 1.f);  // was 4.0: collapsed LA to min for any curve >0.25m⁻¹
    float la = LOOKAHEAD_MIN_M + kf * (LOOKAHEAD_NOM_M - LOOKAHEAD_MIN_M) + vel_term;
    return clampf(la, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M);
}

static float ctrl_nearestDistWindowed(const CtrlPose& robot) {
    if(g_ctrl_path.empty()) return 1e9f;
    int N=(int)g_ctrl_path.size();
    int lo=std::max(0, g_ctrl.lastCI-50);
    int hi=std::min(N-1, g_ctrl.lastCI+50);
    float md=1e9f;
    for(int i=lo;i<=hi;i++){
        float d=sqrtf((robot.x-g_ctrl_path[i].x)*(robot.x-g_ctrl_path[i].x)+
                      (robot.y-g_ctrl_path[i].y)*(robot.y-g_ctrl_path[i].y));
        if(d<md) md=d;
    }
    return md;
}

static float ctrl_purePursuit(const CtrlPose& robot, int& outLA, float& outCTE) {
    if(g_ctrl_path.empty()) return 0;
    int N=(int)g_ctrl_path.size();
    int ci = g_ctrl.lastCI;
    {
        auto fdist=[](float ax,float ay,float bx,float by){
            float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
        float md = fdist(robot.x,robot.y,g_ctrl_path[ci].x,g_ctrl_path[ci].y);
        float walked=0;
        for(int i=ci+1;i<N;i++){
            walked += fdist(g_ctrl_path[i-1].x,g_ctrl_path[i-1].y,g_ctrl_path[i].x,g_ctrl_path[i].y);
            if(walked>20.0f) break;
            float d=fdist(robot.x,robot.y,g_ctrl_path[i].x,g_ctrl_path[i].y);
            if(d<md){md=d;ci=i;}
        }
        if(md>3.0f){
            for(int i=g_ctrl.lastCI;i<N;i++){
                float d=fdist(robot.x,robot.y,g_ctrl_path[i].x,g_ctrl_path[i].y);
                if(d<md){md=d;ci=i;}
            }
        }
    }
    if(ci < g_ctrl.lastCI) ci = g_ctrl.lastCI;
    g_ctrl.lastCI = ci;

    if(ci < N-1){
        float px2=g_ctrl_path[ci+1].x-g_ctrl_path[ci].x, py2=g_ctrl_path[ci+1].y-g_ctrl_path[ci].y;
        float qx=robot.x-g_ctrl_path[ci].x, qy=robot.y-g_ctrl_path[ci].y;
        float len=sqrtf(px2*px2+py2*py2);
        if(len>1e-5f) outCTE = -(px2*qy-py2*qx)/len;
        else outCTE=0.0f;
    } else outCTE=0.0f;

    float previewCurv = fabsf(g_ctrl_path[ci].curvature);
    {
        auto fdist2=[](float ax,float ay,float bx,float by){
            float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
        float walked3=0.f;
        for(int i=ci+1;i<N;i++){
            walked3+=fdist2(g_ctrl_path[i-1].x,g_ctrl_path[i-1].y,g_ctrl_path[i].x,g_ctrl_path[i].y);
            if(walked3>3.0f) break;
            previewCurv=std::max(previewCurv,fabsf(g_ctrl_path[i].curvature));
        }
    }
    float la = ctrl_computeLookahead(outCTE, previewCurv);
    g_lookaheadM = la;

    auto fdist3=[](float ax,float ay,float bx,float by){
        float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
    int li=-1;
    // Normal lookahead: first path point at least `la` ahead of the robot. The danger
    // zone is handled by splicing its curve INTO g_ctrl_path (see ctrl_splice_danger_zone),
    // so Pure Pursuit just follows it like any other path here — no special aim-at-exit
    // override (that older hack made the cart aim straight at the far exit and ignore
    // the curve in between, so it wouldn't steer around the bend). If the danger curve
    // is too tight, straighten it with the Step 2 Edit Path tool instead.
    {
        int fallback=(ci+1<N)?ci+1:N-1;
        for(int i=ci+1;i<N;i++){
            float d=fdist3(robot.x,robot.y,g_ctrl_path[i].x,g_ctrl_path[i].y);
            if(d>=la){
                float dxi=g_ctrl_path[i].x-robot.x, dyi=g_ctrl_path[i].y-robot.y;
                float dot=-sinf(robot.yaw)*dxi+cosf(robot.yaw)*dyi;
                if(dot>=-0.3f){li=i;break;}
            }
            if(d<la*1.5f) fallback=i;
        }
        if(li<0) li=fallback;
    }
    outLA=li;

    float dx2=g_ctrl_path[li].x-robot.x, dy2=g_ctrl_path[li].y-robot.y;
    // CCW-positive yaw (north=0, west=+π/2): forward=(-sin,cos), right=(cos,sin)
    float lx=-sinf(robot.yaw)*dx2+cosf(robot.yaw)*dy2;
    float ly= cosf(robot.yaw)*dx2+sinf(robot.yaw)*dy2;
    float L=sqrtf(lx*lx+ly*ly); if(L<0.01f) return 0;
    float yaw_error=atan2f(-ly,lx);
    float kappa=2.f*sinf(yaw_error)/L;
    return clampf(toDeg(atanf(kappa*WHEELBASE_M)), -MAX_STEER_DEG, MAX_STEER_DEG);
}

// Arc length along g_ctrl_path between two indices (order-independent).
static float ctrl_path_arc_between(int a, int b) {
    int N=(int)g_ctrl_path.size();
    if(a>b) std::swap(a,b);
    a=std::max(0,a); b=std::min(N-1,b);
    float s=0.f;
    for(int i=a;i<b;i++){
        float dx=g_ctrl_path[i+1].x-g_ctrl_path[i].x, dy=g_ctrl_path[i+1].y-g_ctrl_path[i].y;
        s+=sqrtf(dx*dx+dy*dy);
    }
    return s;
}
// Interpolated position on g_ctrl_path at arc length `s` measured forward from
// `startIdx`, not past `endIdx`. reachedEnd=true (and clamps to endIdx) once s runs
// off the end of the [startIdx,endIdx] stretch.
static bool ctrl_path_pos_at_arc(int startIdx, int endIdx, float s, float& outX, float& outY, bool& reachedEnd) {
    int N=(int)g_ctrl_path.size();
    reachedEnd=false;
    if(startIdx<0||startIdx>=N){ reachedEnd=true; return false; }
    if(endIdx>=N) endIdx=N-1;
    if(s<0.f) s=0.f;
    float acc=0.f;
    for(int i=startIdx;i<endIdx;i++){
        float dx=g_ctrl_path[i+1].x-g_ctrl_path[i].x, dy=g_ctrl_path[i+1].y-g_ctrl_path[i].y;
        float seg=sqrtf(dx*dx+dy*dy);
        if(acc+seg>=s){
            float t=(seg>1e-6f)?(s-acc)/seg:0.f;
            outX=g_ctrl_path[i].x+t*dx; outY=g_ctrl_path[i].y+t*dy;
            return true;
        }
        acc+=seg;
    }
    outX=g_ctrl_path[endIdx].x; outY=g_ctrl_path[endIdx].y;
    reachedEnd=true;
    return true;
}

// Full pose (position + tangent heading) on g_ctrl_path at arc length `s`. The tangent
// is the direction of travel along the curve, in the same CCW-positive convention the
// rest of control uses (forward = (-sin(yaw), cos(yaw)), so yaw = atan2(-dx, dy)).
// Used to drive the WHOLE pose from the recorded curve during danger-zone dead-
// reckoning — position AND heading — so the traversal is a fully deterministic replay
// of the fixed curve, independent of IMU/scan-match (which are untrustworthy there).
static bool ctrl_path_pose_at_arc(int startIdx, int endIdx, float s, float& outX, float& outY, float& outYaw, bool& reachedEnd) {
    if(!ctrl_path_pos_at_arc(startIdx, endIdx, s, outX, outY, reachedEnd)) return false;
    // sample a short step ahead for the tangent; if at the very end, sample behind
    float ax, ay; bool re2=false;
    ctrl_path_pos_at_arc(startIdx, endIdx, s+0.20f, ax, ay, re2);
    float dx=ax-outX, dy=ay-outY;
    if(dx*dx+dy*dy < 1e-8f){
        float bx, by; bool re3=false;
        ctrl_path_pos_at_arc(startIdx, endIdx, s-0.20f, bx, by, re3);
        dx=outX-bx; dy=outY-by;
    }
    outYaw = atan2f(-dx, dy);
    return true;
}

// =============================================================================
//  CONTROL STEP  (called from control_loop_thread at CONTROL_HZ)
// =============================================================================
void ctrl_step(float dt) {
    if (g_steer_test_running.load()) return;

    // ── Pedestrian auto-resume (2026-07-19, MOVED HERE 2026-07-27 — real bug fix) ──
    // This must run during CS_PAUSED, but it used to live further down in this
    // function, after the "if(g_ctrl_state != CS_RUNNING) return;" guard below —
    // which made it permanently unreachable dead code. It could NEVER execute while
    // paused, so the cart never auto-resumed and always needed the manual Resume
    // button, ever since 2026-07-19. Found 2026-07-27 from a live test where the
    // cart sat paused indefinitely after the pedestrian cleared. Moved above the
    // CS_RUNNING guard and given its own early return so the rest of this function
    // (which genuinely only makes sense while actively driving) is untouched.
    //
    // Once a pedestrian-triggered pause's cause has cleared, continue automatically
    // instead of waiting for the operator to press Resume — same resume path
    // (ctrl_doStart() preserves danger-zone state across CS_PAUSED). Requires
    // pedResumeConfirmCount consecutive CLEAR ticks (~3s at 50Hz) before actually
    // resuming — so a person lingering right at the stop-distance boundary can't
    // cause rapid stop/go flicker; the Python detector's own per-person median
    // filter already smooths the distance reading, this is a second, independent
    // layer on the C++ side. Only fires for a PEDESTRIAN pause specifically
    // (g_ped_paused) — a manual pause or any other CS_PAUSED reason (goal reached,
    // danger-zone checkpoint, etc.) is untouched.
    if(g_ctrl_state==CS_PAUSED && g_ped_paused.load()){
        CtrlPose& robotNow = g_simMode ? g_ctrl_sim : (g_ctrl_robot.valid ? g_ctrl_robot_filtered : g_ctrl_robot);
        bool gridPedStop = grid_check_pedestrian_stop(robotNow.x, robotNow.y);
        g_grid_ped_stop.store(gridPedStop);
        bool camPedStop = g_ped_enabled.load() && g_ped_detected.load();
        if(camPedStop || gridPedStop){
            g_ctrl.pedResumeConfirmCount = 0;
        } else {
            g_ctrl.pedResumeConfirmCount++;
            if(g_ctrl.pedResumeConfirmCount >= 150){
                ctrl_doStart();   // takes g_ctrl_path_mutex itself (recursive-safe)
            }
        }
        return;
    }

    if(g_ctrl_state != CS_RUNNING) return;
    // Held for the whole tick (2026-07-19) — g_ctrl_path can now be hot-swapped live
    // from the render/UI thread (Step 3 obstacle-triggered reroute, ctrl_hotswap_path()
    // in this file) while the cart is actively driving. Recursive: the pedestrian
    // auto-resume path below calls ctrl_doStart(), which also takes this lock.
    std::lock_guard<std::recursive_mutex> ctrlPathLock(g_ctrl_path_mutex);
    g_ctrl.steerSentThisTick = false;

    g_ctrl.startupTick++;  // must increment in both real and sim so EMA initialises correctly
    if(!g_simMode && g_ctrl_robot.valid){
        bool in_startup = (g_ctrl.startupTick <= STARTUP_IMU_TICKS);
        bool turning = g_is_turning.load();

        g_ctrl.ekfYawDeg = toDeg(g_ctrl_robot.yaw);
        if(g_imu_drive_ref_valid){
            float imu_deg_now;
            { std::lock_guard<std::mutex> lk(imu_mutex); imu_deg_now = imu_angle.z; }
            g_ctrl.imuRawDeg = imu_deg_now;
            float imu_delta_rad = wrapA((imu_deg_now - g_imu_yaw_drive_start_deg) * (kPI/180.f));
            float imu_yaw_world = wrapA(g_ekf_yaw_drive_start_rad + imu_delta_rad);

            // ── DANGER ZONE dead-reckoning (July 2026) ──
            // Scan-match position is unreliable through the Building-A -> Basketball
            // Court Curve stretch — confirmed wrong/lost even stationary and under
            // manual driving. In that stretch, position is SNAPPED onto the PLANNED
            // PATH being driven (g_ctrl_path — the exact same green line Pure Pursuit
            // steers toward), advanced by wheel speed. Because the dead-reckoned
            // position and the PP target are now the SAME curve, they can never
            // disagree (an earlier version snapped onto a SEPARATE frozen danger-zone
            // file, which diverged from the planned path once the path was edited/
            // straightened — the cart then chased a phantom cross-track error and
            // followed neither line). A speed error now just shifts the estimate
            // forward/back ALONG the driven path, never off to the side of it. Heading
            // stays IMU-only. Entry/exit gates on g_ctrl.lastCI (plan-progress index,
            // set by ctrl_splice_danger_zone); the danger-zone file is used ONLY to
            // mark which path indices are in the region, not to drive position.
            float posX = g_ctrl_robot.x, posY = g_ctrl_robot.y;
            float dr_yaw = imu_yaw_world;   // overwritten with the curve tangent while dead-reckoning
            bool  dr_pose_valid = false;    // true once posX/posY/dr_yaw hold a curve pose this tick
            {
                float speed_dr = g_wheel_speed_valid ? g_wheel_speed_mps : g_vel_est_ms;
                if(!g_dr_active && g_dz_path_lo>=0 &&
                   g_ctrl.lastCI >= g_dz_path_lo && g_ctrl.lastCI <= g_dz_path_hi){
                    g_dr_active = true;
                    // start arc-length at the robot's current progress into the stretch
                    g_dr_s = ctrl_path_arc_between(g_dz_path_lo, g_ctrl.lastCI);
                }
                if(g_dr_active){
                    g_dr_s += speed_dr * dt;
                    float lx, ly, lyaw; bool reachedEnd=false;
                    if(ctrl_path_pose_at_arc(g_dz_path_lo, g_dz_path_hi, g_dr_s, lx, ly, lyaw, reachedEnd) && !reachedEnd){
                        // Deterministic curve replay: BOTH position and heading come from
                        // the recorded curve at the current arc-length (advanced only by
                        // wheel distance). Heading is the curve tangent, NOT the IMU —
                        // with position on the curve but heading from a drifting IMU, Pure
                        // Pursuit saw a phantom heading error and steered wrong (worse the
                        // longer the curve → reverse map "got lost"). Now the pose is
                        // exactly on the curve facing along it, so PP's steer is just the
                        // curve's own geometry: a fixed, repeatable path out of the zone.
                        g_dr_x = lx; g_dr_y = ly;
                        posX = g_dr_x; posY = g_dr_y; dr_yaw = lyaw; dr_pose_valid = true;
                    } else {
                        g_dr_active = false;   // reached/passed the end of the danger-zone stretch
                    }
                }
                // ── DANGER ZONE TEST MAP inner sub-zone (2026-07-07) ── independent of
                // g_dr_active above; only meaningful on "Danger Zone Fwd" (g_dzsub_lo/hi
                // set by ctrl_locate_dzsub(), -1 elsewhere so this never fires on other
                // maps). Curve-replay, same mechanism as g_dr_active above — NOT a
                // straight line (that was the first attempt; switched back to curve replay
                // so PP keeps steering the actual recorded shape and can recover instead of
                // blindly ignoring it): position AND heading come from g_ctrl_path itself
                // at the current arc-length, advanced by wheel distance. Steering is NOT
                // force-zeroed for this version — PP steers normally against the curve.
                if(!dr_pose_valid){
                    if(!g_dzsub_active && g_dzsub_lo>=0 &&
                       g_ctrl.lastCI >= g_dzsub_lo && g_ctrl.lastCI <= g_dzsub_hi){
                        g_dzsub_active = true;
                        g_dzsub_s = ctrl_path_arc_between(g_dzsub_lo, g_ctrl.lastCI);
                    }
                    if(g_dzsub_active){
                        g_dzsub_s += speed_dr * dt;
                        float lx, ly, lyaw; bool reachedEnd=false;
                        if(ctrl_path_pose_at_arc(g_dzsub_lo, g_dzsub_hi, g_dzsub_s, lx, ly, lyaw, reachedEnd) && !reachedEnd){
                            posX = lx; posY = ly; dr_yaw = lyaw; dr_pose_valid = true;
                        } else {
                            g_dzsub_active = false;   // reached/passed the end of the sub-zone
                        }
                    }
                }
            }
            // While dead-reckoning (either mechanism), actively hold the EKF's position in
            // sync with the known-good dead-reckoned estimate — the scan-matcher thread
            // bypasses its own correction here (see localization.cpp) specifically so this
            // write is the only thing setting ekf.x[0,1] during the zone. Without this, the
            // EKF sits frozen at whatever it last held right before entry (or, before this
            // fix, kept silently drifting on untrustworthy scan-matches), so by exit
            // time it's badly wrong — exactly why localization struggled right after
            // the curve. Continuously syncing it means scan-matching resumes from a
            // known-good position the instant the zone ends, and Step 3's visualization
            // (which reads robot_trajectory, fed from the EKF) shows the real dead-
            // reckoned path through the zone instead of a frozen/wrong position.
            if(g_dr_active || g_dzsub_active){
                {std::lock_guard<std::mutex> lk(ekf_mutex); ekf.x[0]=posX; ekf.x[1]=posY;}
                std::lock_guard<std::mutex> lk2(trajectory_mutex);
                Vector3 np={posX,0.2f,-posY};
                if(robot_trajectory.empty()||Vector3Distance(robot_trajectory.back(),np)>0.05f)
                    robot_trajectory.push_back(np);
            }

            // ── DANGER ZONE entry/exit safety stop (July 2026) ──
            // Stop and alert at both the start and end of the danger zone so whoever
            // is watching knows localization may be unreliable there ("keep safe").
            // On EXIT, request a full particle-filter global RELOCALIZATION (seeded at
            // the dead-reckoned exit position, which is roughly right since we followed
            // the known path) — dead-reckoning ran open-loop through the zone, so the
            // EKF must be re-anchored to the map before scan-matching is trusted again.
            // The operator still presses Start to resume (no auto-continue).
            if(g_dr_active && !g_dr_was_active){
                ctrl_sendCmd("SPEED,0.00"); ctrl_sendCmd("BRAKE,1"); ctrl_sendCmd("STEER,0.00");
                g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
                g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
                g_ctrl.centerRetryTicks=25;
                g_ctrl_state = CS_PAUSED;
                g_statusMsg = "DANGER ZONE - stopped for safety. Localization may be unreliable here. Press Start to continue.";
                g_statusCol = C_BAD;
                g_dr_was_active = g_dr_active;
                return;
            }
            if(!g_dr_active && g_dr_was_active){
                ctrl_sendCmd("SPEED,0.00"); ctrl_sendCmd("BRAKE,1"); ctrl_sendCmd("STEER,0.00");
                g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
                g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
                g_ctrl.centerRetryTicks=25;
                g_ctrl_state = CS_PAUSED;
                g_statusMsg = "Exited danger zone - please relocalize to continue.";
                g_statusCol = C_WARN;
                // The custom in-drive relocalize-then-replan flow (particle filter
                // launched directly from here, replan back to the original destination)
                // hung the whole app during a live test — cause not yet isolated. Removed
                // in favour of just sending the operator back to the normal, already-
                // proven Localize panel (main.cpp picks up g_dz_return_to_localize),
                // where they re-localize and pick a destination the same way as starting
                // a fresh journey. Slower, but uses only paths already validated on the cart.
                g_dz_return_to_localize = true;
                g_dr_was_active = g_dr_active;
                return;
            }
            g_dr_was_active = g_dr_active;

            // ── DANGER ZONE TEST MAP sub-zone exit stop ── No entry stop (per field
            // request: drive through uninterrupted, no pause on the way in) — only stop
            // on exit. Same stop mechanics as the g_dr exit above.
            // What happens AFTER the stop depends on context (2026-07-08):
            //   - Standalone/manual test (g_dzx_state==DZX_NONE): send the operator to
            //     Localize to reacquire manually, same as always
            //     (g_dz_return_to_localize).
            //   - Automated crossing journey (g_dzx_state==DZX_IN_DZMAP): hand off
            //     automatically via g_dzx_pending_exit — the SAME point the crossing
            //     journey's exit-relocalization already uses. This is deliberately the
            //     sub-zone exit, NOT the map's own recording endpoint further ahead:
            //     danger_zone_forward.csv (the campus-frame seed used to relocalize back
            //     on the main map) represents only the hard/dead-reckoned stretch itself
            //     (confirmed ~24m of arc length, matching the sub-zone marker), not the
            //     ~50m of extra recorded road beyond it. An earlier version of this fix
            //     let the cart keep driving to the recording's own endpoint, which took
            //     it somewhere with no matching campus-frame seed — the relocalization
            //     search had nothing correct to converge on and the destination was
            //     unreachable. Stopping HERE, exactly where danger_zone_forward.csv's
            //     endpoint actually corresponds to, is what makes the seed valid.
            if(!g_dzsub_active && g_dzsub_was_active){
                ctrl_sendCmd("SPEED,0.00"); ctrl_sendCmd("BRAKE,1"); ctrl_sendCmd("STEER,0.00");
                g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
                g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
                g_ctrl.centerRetryTicks=25;
                g_ctrl_state = CS_PAUSED;
                if(g_dzx_state==DZX_IN_DZMAP){
                    g_statusMsg = "Crossed danger zone - relocalizing on main map...";
                    g_statusCol = C_WARN;
                    g_dzx_pending_exit = true;
                } else {
                    g_statusMsg = "Exited sub-zone - please relocalize to continue.";
                    g_statusCol = C_WARN;
                    g_dz_return_to_localize = true;
                }
                g_dzsub_was_active = g_dzsub_active;
                return;
            }
            g_dzsub_was_active = g_dzsub_active;

            if(dr_pose_valid){
                // Danger-zone replay: use the exact curve pose, NOT the smoothing filter
                // — the curve is already smooth and deterministic, and running it through
                // the position LPF (tuned for noisy live pose) would only lag it behind
                // the true arc-length. Keep the filter seeded so normal driving resumes
                // cleanly at the exit.
                g_ctrl_robot_filtered = {posX, posY, dr_yaw, true};
                g_pose_lpf.update(posX, posY, dr_yaw, turning);
            } else if(in_startup){
                g_ctrl_robot_filtered = g_pose_lpf.update(posX, posY, g_ctrl_robot.yaw, turning);
                g_ctrl_robot_filtered.yaw = imu_yaw_world;
            } else {
                // Use IMU heading as PoseLPF input — EKF used for position only.
                // Scan-match heading drift (lighting/ceiling dependent) cannot reach PP.
                g_ctrl_robot_filtered = g_pose_lpf.update(posX, posY, imu_yaw_world, turning);
            }
        } else {
            g_ctrl_robot_filtered = g_pose_lpf.update(g_ctrl_robot.x, g_ctrl_robot.y, g_ctrl_robot.yaw, turning);
        }

        vel_est_update(g_ctrl_robot_filtered.x, g_ctrl_robot_filtered.y, dt);
        if(g_wheel_speed_valid){
            // Expire stale wheel speed after 2 s — Hall sensor gives ~1 pulse/1.44 s at 1 m/s,
            // so 2 s covers one full revolution gap before trusting EKF estimator instead.
            uint64_t age_ms = (uint64_t)(nowSecEpoch()*1000.0) - g_wheel_speed_lastMs;
            if(age_ms > 2000UL) g_wheel_speed_valid = false;
            else g_vel_est_ms = 0.55f * g_wheel_speed_mps + 0.45f * g_vel_est_ms;
        }
    }
    CtrlPose& robot = g_simMode ? g_ctrl_sim : (g_ctrl_robot.valid ? g_ctrl_robot_filtered : g_ctrl_robot);

    if(!robot.valid){
        if(g_ctrl_state == CS_RUNNING){
            g_statusMsg = "ERROR: Lost pose tracking - pausing safely";
            g_statusCol = C_BAD;
            ctrl_sendCmd("SPEED,0.00");
            ctrl_sendCmd("BRAKE,1");
            ctrl_sendCmd("STEER,0.00");
            g_ctrl.cmdSteer          = 0.0f;
            g_ctrl.lastSentSteer     = 0.0f;
            g_ctrl.steerEMA          = 0.0f;
            g_ctrl.steerRampTarget   = 0.0f;
            g_ctrl.steerFiltered     = 0.0f;
            g_ctrl.lastSentSpeed     = 0.0f;
            g_ctrl.centerRetryTicks  = 25;
            g_ctrl_state = CS_PAUSED;
        }
        return;
    }

    // ── Pedestrian stop (2026-07-08 camera + 2026-07-10 obstacle-grid) ── Top-priority
    // safety pause: if either the camera detector sees a person in its stop zone, OR
    // the robot is near a grid cell manually marked "pedestrian" (Step 2/3 Obstacle
    // Test Mode — a stand-in for real detection, same trigger semantics), stop
    // immediately while driving. Sets g_ped_paused so the Drive panel can offer a
    // manual Resume button (fallback/override) — but see the auto-resume block right
    // below, which normally handles this without the operator needing to press
    // anything. Placed before goal/watchdog/PP so it always wins. Works alongside
    // danger-zone dead-reckoning: ctrl_doStart() preserves g_dr_active/g_dzsub_active
    // across a CS_PAUSED resume.
    bool gridPedStop = grid_check_pedestrian_stop(robot.x, robot.y);
    g_grid_ped_stop.store(gridPedStop);
    bool camPedStop = g_ped_enabled.load() && g_ped_detected.load();
    if(g_ctrl_state==CS_RUNNING && (camPedStop || gridPedStop)){
        ctrl_sendCmd("SPEED,0.00"); ctrl_sendCmd("BRAKE,1"); ctrl_sendCmd("STEER,0.00");
        g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
        g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
        g_ctrl.centerRetryTicks=25;
        g_ctrl_state=CS_PAUSED;
        g_ped_paused.store(true);
        g_statusMsg = camPedStop ? "PEDESTRIAN DETECTED - cart stopped"
                                  : "OBSTACLE GRID: PEDESTRIAN CELL - cart stopped";
        g_statusCol=C_BAD;
        return;
    }
    // (Pedestrian auto-resume moved to the top of this function, 2026-07-27 — see there.)

    auto fdist_ctrl=[](float ax,float ay,float bx,float by){
        float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
    g_ctrl.distToGoal = fdist_ctrl(robot.x,robot.y,g_ctrl_goal.x,g_ctrl_goal.y);

    bool distCheck  = (g_ctrl.distToGoal < GOAL_RADIUS_M);
    bool indexCheck = (g_ctrl.lastCI >= (int)g_ctrl_path.size() - 20);

    if(distCheck && indexCheck){
        g_ctrl.goalConfirmCount++;
        if(g_ctrl.goalConfirmCount >= 5){
            ctrl_sendCmd("SPEED,0.00");
            ctrl_sendCmd("BRAKE,1");
            ctrl_sendCmd("STEER,0.00");
            g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
            g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
            g_ctrl.centerRetryTicks=25;
            logger_write_csv();
            // Danger-zone CROSSING journey (2026-07-07): this "goal" may be an
            // intermediate checkpoint (Building A, before switching to the danger-zone
            // map) or the danger-zone map's own end point (before switching back) —
            // not the passenger's real final destination. Pause and hand off to
            // main.cpp for the map-switch+relocalize instead of declaring done.
            if(g_dzx_state==DZX_TO_CHECKPOINT){
                g_ctrl_state=CS_PAUSED;
                g_statusMsg="Reached checkpoint - switching to danger-zone map...";
                g_statusCol=C_WARN;
                g_dzx_pending_enter=true;
            } else if(g_dzx_state==DZX_IN_DZMAP){
                g_ctrl_state=CS_PAUSED;
                g_statusMsg="Crossed danger zone - relocalizing on main map...";
                g_statusCol=C_WARN;
                g_dzx_pending_exit=true;
            } else {
                g_ctrl_state=CS_DONE;
                g_statusMsg=TextFormat("GOAL REACHED - Final dist: %.2fm", g_ctrl.distToGoal);
                g_statusCol=C_GOOD;
            }
            return;
        }
    } else {
        g_ctrl.goalConfirmCount=0;
    }

    // Corridor watchdog is suppressed while dead-reckoning through the danger zone or
    // sub-zone (g_dr_active / g_dzsub_active). The whole point of that stretch is
    // "trust the recorded curve instead of live position sensing" — enforcing a strict
    // CTE-based safety check over the SAME window fights that: even the dead-reckoned
    // position itself can show a brief CTE spike right at entry (snapping from the
    // last EKF estimate to the recorded curve's start, which the position filter
    // takes a few ticks to smooth over), tripping the watchdog mid-zone for no real
    // fault. Confirmed from a live test: watchdog fired while correctly dead-reckoning
    // through the curve.
    if(fdist_ctrl(robot.x,robot.y,g_ctrl.startX,g_ctrl.startY) > WATCHDOG_GRACE_M && !g_dr_active && !g_dzsub_active){
        if(fabsf(g_ctrl.crossTrackErr) > CORRIDOR_HALF_M){
            g_ctrl.watchdogFrames++;
            if(g_ctrl.watchdogFrames>=5){
                ctrl_sendCmd("ESTOP"); ctrl_sendCmd("SPEED,0.00"); ctrl_sendCmd("STEER,0.00");
                g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
                g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
                g_ctrl.centerRetryTicks=25;
                g_ctrl_state=CS_WATCHDOG;
                g_statusMsg="WATCHDOG TRIP — Cart left path boundaries!";
                g_statusCol=C_BAD;
                logger_write_csv();
                return;
            }
        } else { g_ctrl.watchdogFrames=0; }
    }

    // ── Localization health watchdog ─────────────────────────────────────────
    // If inlier ratio stays below 0.30 for 3 s during driving (total localization
    // loss — scan matcher cannot find the robot), pause and stop the drive.
    // All other stop mechanisms (goal, corridor watchdog) rely on EKF pose, so
    // they cannot trigger when localisation is lost.
    // SUPPRESSED while dead-reckoning through the danger zone or sub-zone (g_dr_active
    // / g_dzsub_active): both are DEFINED by scan-matching failing there, so inlier
    // ratio is expected to sit below 0.30 — after ~3 s inside the zone (common on the
    // longer reverse curve) this watchdog would fire "localisation lost" and pause
    // mid-traversal, which is the "stops more than once / fails in the danger zone"
    // behaviour. Dead-reckoning does not use scan-matching at all here, so a bad
    // inlier ratio is not a fault — that's the whole reason it's dead-reckoning.
    {
        // g_ctrl.locFailSince is in CtrlVars — reset to 0.0 on every ctrl_doStart()
        // via g_ctrl={}, so a stale timestamp from a prior run cannot trigger an
        // instant false watchdog at the start of the next drive.
        float ir_now = current_inlier_ratio.load();
        if(ir_now < 0.30f && !g_simMode && !g_dr_active && !g_dzsub_active){
            if(g_ctrl.locFailSince == 0.0) g_ctrl.locFailSince = nowSecEpoch();
            if(nowSecEpoch() - g_ctrl.locFailSince > 3.0){
                ctrl_sendCmd("SPEED,0.00");
                ctrl_sendCmd("BRAKE,1");
                g_statusMsg = "PAUSED: localisation lost (inlier<30% for >3s)";
                g_statusCol = C_BAD;
                g_ctrl_state = CS_PAUSED;
                g_ctrl.running = false;
                logger_write_csv();
                g_ctrl.locFailSince = 0.0;
                return;
            }
        } else {
            g_ctrl.locFailSince = 0.0;
        }
    }

    float rawSteer=0, cte=0;
    if(g_dr_active || g_dzsub_active){
        // Danger-zone / sub-zone replay: steer straight off the exact curve pose, with
        // NO sensor time-alignment prediction. The prediction stages extrapolate the
        // pose by speed·(-sin,cos)(yaw)·dt using IMU angular rate — that would shove
        // the pose off the fixed curve again, re-introducing exactly the drift this
        // replay exists to avoid. `robot` is already the deterministic curve pose (set
        // above, from whichever of the two mechanisms is currently active — they're
        // mutually exclusive by map, never both true at once).
        rawSteer = ctrl_purePursuit(robot, g_ctrl.lookaheadIdx, cte);

        // Curvature feedforward — danger-zone/sub-zone-only tuning (2026-07-07 field
        // test). Disabled campus-wide elsewhere (noisy live path curvature caused
        // doubled steer / wall-strikes), but this curve is a fixed, pre-recorded,
        // deterministic replay, so that noise risk doesn't apply here. Targets the
        // physical steering actuator (5-8 deg/s) not keeping up with how fast a curve's
        // heading changes, so PP's reactive geometric steer arrives correct but late and
        // the cart understeers wide. FF lets PP pre-steer using the curve's own
        // curvature ahead of the lookahead point, giving the rate-limited actuator a
        // head start. Differential form (lookahead curvature minus current-point
        // curvature) contributes ~0 during steady arc tracking (no double-counting with
        // PP's own geometric steer) and only kicks in at curvature transitions.
        // Requires g_ctrl_path[i].curvature to be CCW-positive (fixed in
        // ctrl_splice_danger_zone's yaw computation) or this steers the wrong way.
        // NOTE: must live in THIS branch, not the else-branch below — a near-identical
        // block placed in the else-branch on 2026-07-07 was dead code (that branch
        // never runs while g_dr_active/g_dzsub_active is true) and never actually ran.
        if(g_ctrl.lastCI>=0 && g_ctrl.lastCI<(int)g_ctrl_path.size() &&
           g_ctrl.lookaheadIdx>=0 && g_ctrl.lookaheadIdx<(int)g_ctrl_path.size()){
            float kappa_ci = g_ctrl_path[g_ctrl.lastCI].curvature;
            float kappa_li = g_ctrl_path[g_ctrl.lookaheadIdx].curvature;
            float ff_deg = toDeg(atanf(kappa_li*WHEELBASE_M) - atanf(kappa_ci*WHEELBASE_M));
            rawSteer = clampf(rawSteer+ff_deg, -MAX_STEER_DEG, MAX_STEER_DEG);
        }
    } else if(distCheck && indexCheck){
        rawSteer = g_ctrl.lastSentSteer;
        cte      = g_ctrl.lastCte;
    } else {
        // TIME ALIGNMENT: bring sensor inputs to same instant
        float speed_now = g_wheel_speed_valid ? g_wheel_speed_mps : g_vel_est_ms;
        float omega_now = 0.f;
        { std::lock_guard<std::mutex> lk(imu_mutex); omega_now = imu_gyro.z; }

        // Stage A: compensate LiDAR age
        double t_lidar = g_lidar_sweep_time.load();
        double t_now   = nowSecEpoch();
        float dt_lidar = (t_lidar>0.0) ? clampf((float)(t_now-t_lidar),0.f,0.080f) : 0.f;

        CtrlPose synced = robot;
        {
            float yaw0=robot.yaw, yaw_mid=wrapA(yaw0+0.5f*omega_now*dt_lidar);
            synced.x  -= speed_now*sinf(yaw_mid)*dt_lidar;
            synced.y  += speed_now*cosf(yaw_mid)*dt_lidar;
            synced.yaw = wrapA(yaw0+omega_now*dt_lidar);
        }

        // Stage B: compensate actuator lag
        double est_rtt_s=0.0;
        if(g_log_last_send_time>0.0 && g_log_last_ack_time>g_log_last_send_time)
            est_rtt_s = g_log_last_ack_time - g_log_last_send_time;
        float dt_actuator = (est_rtt_s>0.001 && est_rtt_s<0.200)
                            ? clampf((float)est_rtt_s*0.5f, 0.005f, CTRL_PREDICTION_MAX_S)
                            : CTRL_ACTUATOR_DELAY_S;
        dt_actuator = clampf(dt_actuator, 0.f, CTRL_PREDICTION_MAX_S);

        CtrlPose predicted_robot = synced;
        {
            float yaw_mid2=wrapA(synced.yaw+0.5f*omega_now*dt_actuator);
            predicted_robot.x  -= speed_now*sinf(yaw_mid2)*dt_actuator;
            predicted_robot.y  += speed_now*cosf(yaw_mid2)*dt_actuator;
            predicted_robot.yaw = wrapA(synced.yaw+omega_now*dt_actuator);
        }

        g_dbg_dt_lidar_ms    = dt_lidar    * 1000.f;
        g_dbg_dt_actuator_ms = dt_actuator * 1000.f;
        g_dbg_total_pred_ms  = (dt_lidar+dt_actuator) * 1000.f;
        {
            static int delay_log_tick=0;
            if(++delay_log_tick >= 100){
                delay_log_tick=0;
                // Use cmdlog_push instead of printf — printf on Windows console can block
                // the control thread for 1-3 ms, causing timing jitter at 50 Hz.
                std::ostringstream _oss;
                _oss << std::fixed << std::setprecision(1)
                     << "[SYNC] lidar_age=" << g_dbg_dt_lidar_ms
                     << " ms  actuator_lag=" << g_dbg_dt_actuator_ms
                     << " ms  total_pred=" << g_dbg_total_pred_ms << " ms";
                cmdlog_push(_oss.str());
            }
        }
        rawSteer = ctrl_purePursuit(predicted_robot, g_ctrl.lookaheadIdx, cte);

        // Curvature feedforward disabled for campus driving — campus CSV paths have
        // per-point curvature noise that the ±3-point average cannot suppress.
        // On a curved path PP already computes correct steer geometrically; the FF
        // was doubling the steer in the same direction and causing wall-strikes.
        // Re-enable only after path curvature is smoothed or FF gain is attenuated.
        // (This branch only runs when !g_dr_active/!g_dzsub_active — both danger-zone
        // systems now use curve-replay + PP steers normally, handled in the g_dr_active
        // branch above via ctrl_purePursuit(robot,...) where robot is already the
        // dead-reckoned curve pose for either mechanism. No steer override needed here.)
    }
    g_ctrl.crossTrackErr = cte;
    g_ctrl.cmdSteerRaw   = rawSteer;

    // Startup gate + goal clamp. SKIPPED entirely while dead-reckoning the danger zone
    // or sub-zone: resuming from the entry-stop resets startupTick (g_ctrl={} in
    // ctrl_doStart), so the blind-zone + steer clamp would fire again right at the
    // start of the curve and force rawSteer toward 0 — under-steering the entry and
    // drifting off. The replay pose is exact and the curve steer is trusted, so no
    // ramp is wanted here. Also matters for g_dzsub: the yaw_converged check below
    // compares live IMU heading against ekf.x[2], which is frozen (not written) during
    // dead-reckoning — it would read stale/wrong and could wrongly clamp steer.
    if(!g_dr_active && !g_dzsub_active){
        float d = g_ctrl.distTravelled;
        bool in_startup_gate = (g_ctrl.startupTick <= STARTUP_IMU_TICKS);
        bool yaw_converged   = true;
        bool in_yaw_check_window = (g_ctrl.startupTick <= STARTUP_IMU_TICKS + 100);
        if(!g_simMode && g_imu_drive_ref_valid && !in_startup_gate && in_yaw_check_window){
            float imu_deg_now2;
            { std::lock_guard<std::mutex> lk(imu_mutex); imu_deg_now2=imu_angle.z; }
            float imu_delta2=wrapA((imu_deg_now2-g_imu_yaw_drive_start_deg)*(kPI/180.f));
            float imu_world2=wrapA(g_ekf_yaw_drive_start_rad+imu_delta2);
            float ekf_yaw_now=0.f;
            { std::lock_guard<std::mutex> lk(ekf_mutex); ekf_yaw_now=(float)ekf.x[2]; }
            yaw_converged=(fabsf(wrapA(imu_world2-ekf_yaw_now))<toRad(2.0f));
        }
        if(d < STARTUP_BLIND_M || (in_startup_gate && d < STARTUP_BLIND_M+STARTUP_RAMP_M)){
            rawSteer=0.f;
        } else if(!yaw_converged){
            rawSteer=clampf(rawSteer,-STARTUP_MAX_STEER_DEG,STARTUP_MAX_STEER_DEG);
        } else if(d < STARTUP_BLIND_M+STARTUP_RAMP_M){
            float ramp_frac=(d-STARTUP_BLIND_M)/STARTUP_RAMP_M;
            float startup_clamp=STARTUP_MAX_STEER_DEG+ramp_frac*(MAX_STEER_DEG-STARTUP_MAX_STEER_DEG);
            rawSteer=clampf(rawSteer,-startup_clamp,startup_clamp);
        }
    }

    if(g_ctrl.distToGoal < GOAL_STEER_CLAMP_M){
        float t=clampf((g_ctrl.distToGoal-GOAL_RADIUS_M)/(GOAL_STEER_CLAMP_M-GOAL_RADIUS_M),0.f,1.f);
        float goal_clamp=GOAL_MIN_STEER_DEG+t*(GOAL_MAX_STEER_DEG-GOAL_MIN_STEER_DEG);
        rawSteer=clampf(rawSteer,-goal_clamp,goal_clamp);
    }

    g_ctrl.lastCte=cte;
    float steerFF=clampf(rawSteer,-MAX_STEER_DEG,MAX_STEER_DEG);

    // Stage 1: EMA filter — seed on tick 1 so EMA starts from the first PP output, not zero
    if(g_ctrl.startupTick == 1) g_ctrl.steerFiltered=steerFF;
    g_ctrl.steerFiltered=STEER_EMA_ALPHA*steerFF+(1.f-STEER_EMA_ALPHA)*g_ctrl.steerFiltered;

    // Stage 2: send gate
    g_ctrl.steerRampTarget=g_ctrl.steerFiltered;
    float steerOut=g_ctrl.lastSentSteer;
    float delta=g_ctrl.steerRampTarget-g_ctrl.lastSentSteer;
    if(fabsf(delta)>=STEER_SEND_MIN_DELTA){
        steerOut=clampf(g_ctrl.steerRampTarget,-MAX_STEER_DEG,MAX_STEER_DEG);
        g_ctrl.lastSentSteer=steerOut;
        g_ctrl.steerSentThisTick=true;
        std::ostringstream ss; ss<<std::fixed<<std::setprecision(2)<<"STEER,"<<steerOut;
        ctrl_sendCmd(ss.str());
        if(fabsf(delta)>=STEER_TURN_THRESH_DEG) g_steer_hold_ticks_remaining=STEER_HOLD_TICKS;
    }
    g_ctrl.cmdSteer=steerOut;

    // Speed: pick the active cruise voltage, then decelerate near the goal only.
    // Higher cruise (ZONE2_CRUISE_VOLTAGE) applies ONLY when ALL of these hold:
    //   - physically inside the Zone 2 area (live robot pose, so it turns off the moment
    //     the cart leaves zone 2 — not gated on which zone was armed at localization);
    //   - NOT also inside Zone 1 (the two hand-drawn polygons overlap at the boundary,
    //     and the higher speed must never apply in zone 1 per the operator);
    //   - NOT dead-reckoning the danger zone (that stretch stays at the slow speed);
    //   - settled ON the path: past the startup window AND cross-track error is small.
    //     Boosting while the cart is still steering itself back onto the path (e.g. right
    //     after Start when it localized slightly off the line) makes those corrections
    //     happen at high speed → the wavy motion the operator saw. Waiting for a small
    //     CTE means it only speeds up once it's actually tracking the path cleanly.
    float cruise = CRUISE_VOLTAGE;
    bool inZone2   = g_zone2_valid && !g_zone2_poly.empty() && point_in_polygon(robot.x, robot.y, g_zone2_poly);
    bool inZone1   = g_zone_valid  && !g_zone_poly.empty()  && point_in_polygon(robot.x, robot.y, g_zone_poly);
    bool pastStartup = (g_ctrl.startupTick > STARTUP_IMU_TICKS) && (g_ctrl.distTravelled > 2.0f);
    bool onPath      = (fabsf(cte) < 0.35f);
    // Zone-2 slow segment (2026-07): a marked sub-stretch (currently only set on the
    // reverse map) where the boosted speed below causes scan-matching to fail. Checked
    // live off the robot's position — same reasoning as inZone2/inZone1 — so the boost
    // cuts out right at the marked entry point and resumes right after the exit point.
    bool inSlowSeg = pp_zone2_slow_active(robot.x, robot.y);
    if(!g_dr_active && inZone2 && !inZone1 && pastStartup && onPath && !inSlowSeg){
        cruise = ZONE2_CRUISE_VOLTAGE;
    }
    // Pedestrian caution (2026-07-27): slow down as soon as a person is detected
    // ANYWHERE in the camera's lane zone (g_ped_caution), not just once they're within
    // --stop-distance (g_ped_detected, the actual STOP trigger below). Real near-miss
    // during lab testing showed the cart's brake is a plain throttle-cutoff + coast, no
    // strong active deceleration — it needs real lead distance to actually slow before
    // the stop threshold matters, not just a bigger stop-distance number on its own.
    // Overrides the zone-2 boost (safety over convenience); danger-zone dead-reckoning
    // below still takes final priority if both are somehow active at once.
    bool pedCaution = g_ped_enabled.load() && g_ped_caution.load();
    if(pedCaution){
        cruise = PED_CAUTION_CRUISE_VOLTAGE;
    }
    // Dead-reckoning (danger zone or sub-zone): fixed slower speed, takes priority
    // over everything above including the zone-2 boost and pedestrian caution.
    if(g_dr_active || g_dzsub_active){
        cruise = DANGER_ZONE_CRUISE_VOLTAGE;
    }
    // MIN_VOLTAGE (0.95) is higher than DANGER_ZONE_CRUISE_VOLTAGE (0.85) — floor must
    // track whichever is actually lower or clampf(spd,MIN_VOLTAGE,cruise) below would
    // silently clamp the danger-zone speed back UP to 0.95 (lo>hi degenerate range,
    // clampf's lower bound wins), defeating the override entirely.
    float speedFloor = fminf(MIN_VOLTAGE, cruise);
    float spd=cruise;
    if(g_ctrl.distToGoal < GOAL_DECEL_M){
        float t=clampf((g_ctrl.distToGoal-GOAL_RADIUS_M)/(GOAL_DECEL_M-GOAL_RADIUS_M),0.f,1.f);
        spd=speedFloor+sinf(t*kPI*0.5f)*(cruise-speedFloor);
    }
    spd=clampf(spd,speedFloor,cruise);
    g_ctrl.cmdSpeed=spd;

    static const float SPEED_SEND_HYSTERESIS=0.02f;
    static int speed_keepalive_ticks=0;
    speed_keepalive_ticks++;
    bool keepalive=(speed_keepalive_ticks>=(int)(CONTROL_HZ*1.0f));
    if(keepalive) speed_keepalive_ticks=0;
    if(fabsf(spd-g_ctrl.lastSentSpeed)>=SPEED_SEND_HYSTERESIS||keepalive){
        std::ostringstream sv; sv<<std::fixed<<std::setprecision(2)<<"SPEED,"<<spd;
        ctrl_sendCmd(sv.str());
        g_ctrl.lastSentSpeed=spd;
    }

    // Simulate kinematics in sim mode
    if(g_simMode){
        float sr=toRad(g_ctrl.cmdSteer); float spms=(g_ctrl.cmdSpeed/5)*SIM_MAX_SPEED;
        g_ctrl_sim.x+=sinf(g_ctrl_sim.yaw)*spms*dt;
        g_ctrl_sim.y+=cosf(g_ctrl_sim.yaw)*spms*dt;
        g_ctrl_sim.yaw=wrapA(g_ctrl_sim.yaw-(spms/WHEELBASE_M)*tanf(sr)*dt);
        g_ctrl_simSpd=spms; g_ctrl.distTravelled+=spms*dt;
        g_ctrl_trail.push_back({g_ctrl_sim.x,g_ctrl_sim.y});
        if((int)g_ctrl_trail.size()>TRAIL_MAX) g_ctrl_trail.erase(g_ctrl_trail.begin());
    } else {
        if(g_ctrl_robot.valid){
            static float prev_rx=0,prev_ry=0;
            if(!g_ctrl_trail.empty()){
                float step=sqrtf((g_ctrl_robot.x-prev_rx)*(g_ctrl_robot.x-prev_rx)+
                                 (g_ctrl_robot.y-prev_ry)*(g_ctrl_robot.y-prev_ry));
                // ignore steps below 1 mm — EKF jitter is ~0.5 mm; real motion at
                // even 0.05 m/s produces 1 mm per 20 ms tick and is correctly counted
                if(step > 0.001f) g_ctrl.distTravelled += step;
            }
            prev_rx=g_ctrl_robot.x; prev_ry=g_ctrl_robot.y;
            g_ctrl_trail.push_back({g_ctrl_robot.x,g_ctrl_robot.y});
            if((int)g_ctrl_trail.size()>TRAIL_MAX) g_ctrl_trail.erase(g_ctrl_trail.begin());
        }
    }
    logger_tick(robot);
}

// ── DANGER ZONE splice into the driven plan (July 2026) ─────────────────────
// Through the danger region the cart must follow the exact curve the user drew in
// Step 2 (the known-good recorded shape) — localization can't be trusted there.
// Rather than keep that curve in a SEPARATE file that can visually/behaviourally
// diverge from the plan (the source of "Step 3 differs from Step 2"), splice the
// danger-zone curve directly INTO g_ctrl_path, replacing whatever plan segment spans
// the same region. After this, Pure Pursuit, dead-reckoning, the red overlay and the
// watchdog ALL use the one identical curve — Step 3 matches Step 2 by construction.
// Sets g_dz_path_lo/hi directly from the exact splice range (see below) — this is now
// the ONE place that decides where the danger zone lives on g_ctrl_path. Previously a
// SEPARATE function (dz_precompute_path_range) re-found this range by scanning for any
// path point within a radius of the danger curve, which could disagree with where the
// splice actually inserted the curve (e.g. the untouched approach path already ran
// close-and-parallel to the danger curve for a few metres before the exact splice
// point, so the radius scan flagged "entry" a few metres earlier than the splice did —
// confirmed from screenshots: the on-map DANGER ZONE marker sat ~3m apart between the
// live drive view and the Step-2 preview). One computation, used everywhere, cannot
// disagree with itself.
static void ctrl_splice_danger_zone() {
    g_dz_path_lo = -1; g_dz_path_hi = -1; g_dz_path_lo_arc = 0.f;
    const std::vector<PPPoint>& dz = pp_danger_zone_pts();
    int M=(int)dz.size(), N=(int)g_ctrl_path.size();
    if(M<2 || N<2) return;

    auto nearestIdx=[&](float x,float y,float& outDist){
        int bi=0; float bd=1e18f;
        for(int i=0;i<N;i++){
            float dx=g_ctrl_path[i].x-x, dy=g_ctrl_path[i].y-y, d=dx*dx+dy*dy;
            if(d<bd){bd=d;bi=i;}
        }
        outDist=sqrtf(bd);
        return bi;
    };
    // Where the plan is nearest each end of the danger-zone curve.
    float distA, distB;
    int iA=nearestIdx(dz[0].x,dz[0].y,distA);
    int iB=nearestIdx(dz[M-1].x,dz[M-1].y,distB);
    // Bail if this plan doesn't actually pass near the danger curve — e.g. after
    // relocalizing past the zone and replanning the remaining leg to the destination,
    // that new leg generally does NOT run back through the danger curve's coordinates,
    // but nearestIdx always returns SOME "closest" point even if it's tens of metres
    // away. Without this guard, pressing Start again after a successful exit-replan
    // (state is CS_LOADED, not CS_PAUSED, so the re-splice-on-fresh-start logic still
    // runs) could spuriously re-attach the danger curve to an unrelated stretch of the
    // new leg and re-arm dead-reckoning where it doesn't belong.
    const float MAX_SPLICE_DIST_M = 3.0f;
    if(distA > MAX_SPLICE_DIST_M || distB > MAX_SPLICE_DIST_M) return;
    int entry=std::min(iA,iB), exit=std::max(iA,iB);
    if(exit-entry < 1) return;
    // Orient the danger curve to run the same direction as the plan.
    bool forward = (iA<=iB);

    std::vector<CtrlPathPoint> out;
    out.reserve(entry + M + (N-exit));
    for(int i=0;i<entry;i++) out.push_back(g_ctrl_path[i]);
    if(forward) for(int i=0;i<M;i++)      out.push_back({dz[i].x, dz[i].y, 0.f, 0.f});
    else        for(int i=M-1;i>=0;i--)   out.push_back({dz[i].x, dz[i].y, 0.f, 0.f});
    for(int i=exit+1;i<N;i++) out.push_back(g_ctrl_path[i]);

    // Recompute yaw (from neighbours) + light curvature for the whole spliced path so
    // the new points have valid headings. Uses atan2f(-dx,dy) — CCW-positive, matching
    // robot/EKF/IMU yaw and ctrl_path_pose_at_arc's own tangent calc (NOT the CW-positive
    // atan2f(dx,dy) pp_save_plan uses elsewhere). This spliced path's curvature now feeds
    // the danger-zone-only curvature feedforward below; using the CW convention here was
    // flagged as "the documented cause of previous wall-strike incidents" (see planning.cpp
    // path-yaw-sign note) if curvature FF were ever re-enabled without fixing it first.
    int P=(int)out.size();
    for(int i=0;i<P;i++){
        int a=std::max(0,i-1), b=std::min(P-1,i+1);
        float dx=out[b].x-out[a].x, dy=out[b].y-out[a].y;
        out[i].yaw = atan2f(-dx, dy);
    }
    for(int i=1;i<P-1;i++){
        float d=sqrtf((out[i+1].x-out[i].x)*(out[i+1].x-out[i].x)+(out[i+1].y-out[i].y)*(out[i+1].y-out[i].y));
        float dyaw=wrapA(out[i+1].yaw-out[i].yaw);
        out[i].curvature = (d>1e-6f)? dyaw/d : 0.f;
    }
    g_ctrl_path.swap(out);

    // The spliced danger curve now occupies exactly [entry, entry+M-1] in the NEW
    // g_ctrl_path — this IS the danger-zone range, no separate search needed.
    g_dz_path_lo = entry;
    g_dz_path_hi = entry + M - 1;
    g_dz_path_lo_arc = ctrl_path_arc_between(0, g_dz_path_lo);
}

// Locates the g_dzsub_lo/hi index range for either danger-zone test map's ("Danger
// Zone Fwd" or "Danger Zone Rev") inner sub-zone — simple nearest-point lookup of the
// two world-coordinate markers against the currently loaded g_ctrl_path (no curve
// splice needed, unlike the campus system: this sub-zone is curve-replayed straight
// from g_ctrl_path itself, not a separately recorded shape). No-op on any other map.
static void ctrl_locate_dzsub() {
    g_dzsub_lo = -1; g_dzsub_hi = -1;
    bool onTestMap = active_map_ptr &&
        (active_map_ptr->name=="Danger Zone Fwd" || active_map_ptr->name=="Danger Zone Rev");
    if(!onTestMap) return;
    int N=(int)g_ctrl_path.size();
    if(N<2) return;
    auto nearestIdx=[&](float x,float y)->int{
        int bi=0; float bd=1e18f;
        for(int i=0;i<N;i++){
            float dx=g_ctrl_path[i].x-x, dy=g_ctrl_path[i].y-y, d=dx*dx+dy*dy;
            if(d<bd){bd=d;bi=i;}
        }
        return bi;
    };
    int a = nearestIdx(g_dzsub_entry_mk_x, g_dzsub_entry_mk_y);
    int b = nearestIdx(g_dzsub_exit_mk_x,  g_dzsub_exit_mk_y);
    g_dzsub_lo = std::min(a,b); g_dzsub_hi = std::max(a,b);
}

// Public wrapper — called from Step 2's "Save Sub-Zone" button so a marker change
// takes effect on whatever plan is already loaded immediately, mirroring
// ctrl_resplice_danger_zone_now(). No-op if no plan is loaded yet.
void ctrl_relocate_dzsub_now() {
    if(g_ctrl_path.empty()) return;
    ctrl_locate_dzsub();
}

// The danger-zone index range on the loaded plan (g_ctrl_path). Lets the map overlay
// draw the danger stretch AS PART OF the driven path, so red is always a subset of
// the green planned line (never a separate diverging curve). False if no plan loaded
// or the plan doesn't pass through the danger zone.
bool ctrl_danger_path_range(int& lo, int& hi) {
    if(g_dz_path_lo<0 || g_dz_path_hi<0) return false;
    lo=g_dz_path_lo; hi=g_dz_path_hi; return true;
}

// Re-splice the danger-zone curve into whatever plan is already loaded, right now —
// called from Step 2's "Save Danger Zone" button so Step 3 updates the moment you
// save, instead of waiting for the next Start press. No-op if no plan is loaded yet
// (nothing to splice into; the next ctrl_doLoad()/ctrl_doStart() will pick it up then).
void ctrl_resplice_danger_zone_now() {
    if(g_ctrl_path.empty()) return;
    ctrl_splice_danger_zone();
}

// =============================================================================
//  HIGH-LEVEL CONTROL ACTIONS
// =============================================================================
void ctrl_doLoad() {
    bool pathOk = ctrl_loadPath();
    ctrl_loadPlan();
    if(!pathOk){
        g_statusMsg="smoothed_path.csv not found — complete Step 2 first";
        g_statusCol=C_BAD; g_ctrl_state=CS_IDLE; return;
    }
    // Splice the user-drawn danger-zone curve into the plan (before goal/start are read
    // from it) so the driven path IS the danger-zone shape through the bad region.
    ctrl_splice_danger_zone();
    ctrl_locate_dzsub();
    g_ctrl_goal = {g_ctrl_path.back().x, g_ctrl_path.back().y, g_ctrl_path.back().yaw, true};
    if(g_confirmed_start.valid)
        g_ctrl_start={g_confirmed_start.x,g_confirmed_start.y,g_confirmed_start.yaw,true};
    else
        g_ctrl_start={g_ctrl_path.front().x,g_ctrl_path.front().y,g_ctrl_path.front().yaw,true};
    // Sim always starts with path's heading — g_ctrl_start.yaw from Skip Localization
    // is 0 (north) regardless of path direction, causing PP to see the goal behind it.
    float sim_yaw0 = !g_ctrl_path.empty() ? g_ctrl_path.front().yaw : g_ctrl_start.yaw;
    g_ctrl_sim={g_ctrl_start.x,g_ctrl_start.y,sim_yaw0,true};
    {std::lock_guard<std::mutex>lk(ekf_mutex);
     g_ctrl_robot={(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],ekf.seeded};}
    if(!g_ctrl_robot.valid) g_ctrl_robot={g_ctrl_start.x,g_ctrl_start.y,g_ctrl_start.yaw,true};
    g_ctrl_trail.clear();g_ctrl={};
    g_ctrl.distToGoal=sqrtf((g_ctrl_start.x-g_ctrl_goal.x)*(g_ctrl_start.x-g_ctrl_goal.x)+
                             (g_ctrl_start.y-g_ctrl_goal.y)*(g_ctrl_start.y-g_ctrl_goal.y));
    g_ctrl_state=CS_LOADED;
    g_statusMsg="Plan loaded — press Start Autonomous"; g_statusCol=C_GOOD;
}

// Thread-safe LIVE path swap (2026-07-19) — reloads g_ctrl_path from CSV_OUTPUT_FILE
// (smoothed_path.csv, just rewritten by pp_save_plan()'s obstacle-avoidance pass) and
// re-anchors g_ctrl.lastCI to the robot's current position in the NEW array, so Pure
// Pursuit doesn't search from an index that pointed somewhere else in the old array's
// geometry. Unlike ctrl_doLoad()/ctrl_doStart(), this does NOT touch g_ctrl_goal,
// g_ctrl_start, or reset g_ctrl — it's a path-array-only swap, safe to call while the
// cart is actively driving (CS_RUNNING): a Step-3 static obstacle placement can
// reroute the cart live without stopping, resetting progress, or needing a confirm
// step. Called from grid_click_cycle() (planning.cpp) on the render/UI thread; safe
// against the control thread's concurrent reads via g_ctrl_path_mutex (also held by
// ctrl_step() for the whole tick). No-op if the file is missing/degenerate or the
// robot's pose isn't known yet — leaves the cart on its current path rather than risk
// swapping in something worse.
void ctrl_hotswap_path() {
    std::vector<CtrlPathPoint> fresh;
    {
        std::ifstream f(CSV_OUTPUT_FILE);
        if(!f.is_open()) return;
        std::string line; std::getline(f,line);
        while(std::getline(f,line)){
            if(line.empty()) continue;
            std::stringstream ss(line); CtrlPathPoint p; char c;
            if(ss>>p.x>>c>>p.y>>c>>p.yaw>>c>>p.curvature) fresh.push_back(p);
        }
    }
    if(fresh.size() < 2) return;

    const CtrlPose& rp = g_simMode ? g_ctrl_sim : g_ctrl_robot;
    if(!rp.valid) return;

    // Nearest/forward-preferring point search — same formula as ctrl_doStart()'s
    // resume-anchor logic, kept identical for consistency (that function is the
    // existing, validated reference for "where is the robot on this path array now").
    auto fdist_hs=[](float ax,float ay,float bx,float by){float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
    float md=1e9f; int best_any=0, best_fwd=-1; float md_fwd=1e9f;
    for(int i=0;i<(int)fresh.size();i++){
        float d=fdist_hs(rp.x,rp.y,fresh[i].x,fresh[i].y);
        if(d<md){md=d;best_any=i;}
        float dxi=fresh[i].x-rp.x, dyi=fresh[i].y-rp.y;
        if(sinf(rp.yaw)*dxi+cosf(rp.yaw)*dyi>0.0f&&d<md_fwd){md_fwd=d;best_fwd=i;}
    }
    int newCI = (best_fwd>=0 && md_fwd<=md*2.0f) ? best_fwd : best_any;

    {
        std::lock_guard<std::recursive_mutex> lk(g_ctrl_path_mutex);
        g_ctrl_path = std::move(fresh);
        g_ctrl.lastCI = newCI;
        // lastCte is recomputed naturally by ctrl_step() every tick from lastCI, same
        // as always — no need to duplicate that math here.
    }
}

void ctrl_doStart() {
    // Recursive lock (2026-07-19): safe whether called directly from ui.cpp (the
    // render/UI thread — Start/Resume buttons) or from within ctrl_step() itself on
    // the control thread (pedestrian auto-resume), which already holds this lock —
    // protects every g_ctrl_path read below against a concurrent Step-3 hot-swap.
    std::lock_guard<std::recursive_mutex> ctrlPathLock(g_ctrl_path_mutex);
    if(g_ctrl_path.empty()||!g_ctrl_goal.valid) return;
    // Re-splice the danger-zone curve fresh from disk on a genuinely FRESH start
    // (CS_LOADED — Autonomous AND Simulation, real AND sim mode), so a Step-2 edit is
    // picked up the moment Start is pressed without needing ctrl_doLoad() to have run
    // first. NOT done when RESUMING from a pause (CS_PAUSED — includes the danger-zone
    // entry-stop itself): re-splicing rebuilds g_ctrl_path and recomputes
    // g_dz_path_lo/hi from scratch, and if the recomputed boundary shifts by even one
    // index relative to the robot's current g_ctrl.lastCI, the "am I still in the
    // zone" check below can momentarily read false right as the cart resumes — then
    // read true again a moment later as it moves forward, re-triggering the entry-stop
    // a second time. Confirmed from a live test: the cart stopped twice at the same
    // entry. Skipping the re-splice on resume keeps g_dz_path_lo/hi exactly as they
    // were when the pause first fired, so the in-zone check can't flip.
    // Same resume-preservation reasoning applies to the dzsub sub-zone range — recompute
    // fresh on a real Start, but leave it (and g_dzsub_active/was_active, set below)
    // untouched across a CS_PAUSED resume so its own entry/exit edge check can't flip.
    if(g_ctrl_state != CS_PAUSED){ ctrl_splice_danger_zone(); ctrl_locate_dzsub(); }
    float sim_yaw0 = !g_ctrl_path.empty() ? g_ctrl_path.front().yaw : g_ctrl_start.yaw;
    g_ctrl_sim={g_ctrl_start.x,g_ctrl_start.y,sim_yaw0,true};
    g_ctrl_trail.clear();g_ctrl={};g_ctrl.running=true;
    g_ctrl.goalConfirmCount=0;
    g_ped_paused.store(false);   // resuming clears any pedestrian-pause flag

    const CtrlPose& rp = g_simMode ? g_ctrl_sim : g_ctrl_robot;
    g_ctrl.startX = rp.valid ? rp.x : g_ctrl_start.x;
    g_ctrl.startY = rp.valid ? rp.y : g_ctrl_start.y;

    auto fdist_s=[](float ax,float ay,float bx,float by){
        float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
    g_ctrl.distToGoal=fdist_s(g_ctrl_start.x,g_ctrl_start.y,g_ctrl_goal.x,g_ctrl_goal.y);

    {
        float md=1e9f; int best_any=0, best_fwd=-1; float md_fwd=1e9f;
        for(int i=0;i<(int)g_ctrl_path.size();i++){
            float d=fdist_s(rp.x,rp.y,g_ctrl_path[i].x,g_ctrl_path[i].y);
            if(d<md){md=d;best_any=i;}
            float dxi=g_ctrl_path[i].x-rp.x, dyi=g_ctrl_path[i].y-rp.y;
            if(sinf(rp.yaw)*dxi+cosf(rp.yaw)*dyi>0.0f&&d<md_fwd){md_fwd=d;best_fwd=i;}
        }
        g_ctrl.lastCI=(best_fwd>=0&&md_fwd<=md*2.0f)?best_fwd:best_any;
        if(g_ctrl.lastCI<(int)g_ctrl_path.size()-1){
            float px2=g_ctrl_path[g_ctrl.lastCI+1].x-g_ctrl_path[g_ctrl.lastCI].x;
            float py2=g_ctrl_path[g_ctrl.lastCI+1].y-g_ctrl_path[g_ctrl.lastCI].y;
            float qx=rp.x-g_ctrl_path[g_ctrl.lastCI].x, qy=rp.y-g_ctrl_path[g_ctrl.lastCI].y;
            float len=sqrtf(px2*px2+py2*py2);
            if(len>1e-5f) g_ctrl.lastCte=-(px2*qy-py2*qx)/len;
        }
        float distFromPlanStart=fdist_s(rp.x,rp.y,g_ctrl_start.x,g_ctrl_start.y);
        if(distFromPlanStart>1.5f){
            g_statusMsg=TextFormat("WARNING: %.1fm from plan start — approach path gently",distFromPlanStart);
            g_statusCol=C_WARN;
        }
    }

    g_pose_lpf.reset();
    g_vel_est_ms=1.0f; g_vel_est_init=false;
    g_steer_hold_ticks_remaining=0;
    g_wheel_speed_mps=1.0f;
    g_wheel_speed_valid=false;  // clear stale valid flag so 1.0m/s seed is never used
    // Danger-zone dead-reckoning state: on a genuinely FRESH start (CS_LOADED),
    // derive it from plan progress (g_ctrl.lastCI, just computed above) in case the
    // robot already sits inside the danger zone at the very start of the drive. On a
    // RESUME from a pause (CS_PAUSED — includes the entry-stop itself), do NOT
    // recompute: g_ctrl.lastCI is a fresh nearest-point search against the current
    // (possibly slightly noisy) position estimate, and even a one-index jitter right
    // at the zone boundary can make "am I in the zone" flip from true to false purely
    // from that noise — even though the cart hasn't actually moved and g_dr_active was
    // correctly true when the pause fired. That false reset made g_dr_was_active
    // disagree with reality, and the entry-stop fired a second time a moment later as
    // the cart resumed moving forward and lastCI ticked back into range. Confirmed
    // from a live test: pressed Start after the entry-stop, it stopped again almost
    // immediately. Preserving g_dr_active/g_dr_was_active exactly as they were when
    // paused avoids the flip entirely.
    if(g_ctrl_state != CS_PAUSED){
        bool nowInDZ = g_dz_path_lo>=0 && g_ctrl.lastCI>=g_dz_path_lo && g_ctrl.lastCI<=g_dz_path_hi;
        g_dr_active = nowInDZ;
        g_dr_was_active = nowInDZ;
        // arc-length progress ALONG the planned path from the danger-zone start index
        g_dr_s = nowInDZ ? ctrl_path_arc_between(g_dz_path_lo, g_ctrl.lastCI) : 0.f;

        // Same fresh-start-already-inside-the-zone handling for the dzsub sub-zone —
        // now curve-replay like g_dr_active above, same arc-length semantics.
        bool nowInDZsub = g_dzsub_lo>=0 && g_ctrl.lastCI>=g_dzsub_lo && g_ctrl.lastCI<=g_dzsub_hi;
        g_dzsub_active = nowInDZsub;
        g_dzsub_was_active = nowInDZsub;
        g_dzsub_s = nowInDZsub ? ctrl_path_arc_between(g_dzsub_lo, g_ctrl.lastCI) : 0.f;
    }

    {std::lock_guard<std::mutex> lk(imu_mutex); g_imu_yaw_drive_start_deg=imu_angle.z;}
    {std::lock_guard<std::mutex> lk(ekf_mutex); g_ekf_yaw_drive_start_rad=(float)ekf.x[2];}
    g_imu_drive_ref_valid=true;
    logger_start_run();
    ctrl_sendCmd("AUTO");
    ctrl_sendCmd("BRAKE,0");
    ctrl_sendCmd("FWD_MODE");
    std::ostringstream sv; sv<<std::fixed<<std::setprecision(2)<<"SPEED,"<<MIN_VOLTAGE;
    ctrl_sendCmd(sv.str());
    g_ctrl.lastSentSpeed=0.0f;
    g_ctrl_state=CS_RUNNING;
    if(g_statusMsg.find("WARNING")==std::string::npos){
        g_statusMsg=g_simMode?"Simulation running":"Autonomous running — UDP active";
        g_statusCol=g_simMode?C_WARN:C_GOOD;
    }
}

void ctrl_doStop() {
    ctrl_sendCmd("SPEED,0.00");
    ctrl_sendCmd("BRAKE,1");
    ctrl_sendCmd("STEER,0.00");
    g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
    g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
    g_ctrl.centerRetryTicks=25;
    g_imu_drive_ref_valid=false;
    g_ctrl.running=false; g_ctrl_state=CS_PAUSED;
    g_statusMsg="Paused manually"; g_statusCol=C_WARN;
    logger_write_csv();
}

void ctrl_doReset() {
    ctrl_sendCmd("ESTOP");
    ctrl_sendCmd("SPEED,0.00");
    ctrl_sendCmd("STEER,0.00");
    g_ctrl.cmdSteer=0; g_ctrl.lastSentSteer=0; g_ctrl.steerEMA=0;
    g_ctrl.steerRampTarget=0; g_ctrl.steerFiltered=0; g_ctrl.lastSentSpeed=0;
    g_ctrl.centerRetryTicks=25;
    auto fdist_r=[](float ax,float ay,float bx,float by){
        float dx=ax-bx,dy=ay-by;return sqrtf(dx*dx+dy*dy);};
    g_ctrl.distToGoal=fdist_r(g_ctrl_start.x,g_ctrl_start.y,g_ctrl_goal.x,g_ctrl_goal.y);
    g_ctrl_sim={g_ctrl_start.x,g_ctrl_start.y,g_ctrl_start.yaw,true};
    g_ctrl_robot={g_ctrl_start.x,g_ctrl_start.y,g_ctrl_start.yaw,true};
    g_ctrl_robot_filtered={g_ctrl_start.x,g_ctrl_start.y,g_ctrl_start.yaw,true};
    g_ctrl_trail.clear(); g_pose_lpf.reset();
    g_dz_return_to_localize=false; g_dr_active=false; g_dr_was_active=false;
    g_ped_paused.store(false);
    g_ctrl_state=g_ctrl_path.empty()?CS_IDLE:CS_LOADED;
    g_statusMsg="Reset to start"; g_statusCol=C_WHITE;
    logger_write_csv();
}

// =============================================================================
//  DEDICATED CONTROL THREAD  (deterministic 50 Hz)
// =============================================================================
void control_loop_thread() {
    // THREAD_PRIORITY_TIME_CRITICAL (2026-07-27, real bug found live): user reported
    // the cart losing control mid-turn specifically when the pedestrian-detector's
    // camera windows had OS focus instead of the main UI. Root cause: Windows grants a
    // scheduling/priority boost to whichever process owns the FOCUSED window. The
    // python detector is deliberately launched at BELOW_NORMAL_PRIORITY_CLASS
    // (main.cpp's ped_set_enabled()) specifically so it can't starve this loop — but
    // that's only a BASE priority; the focus boost can still tip the balance toward it
    // over this (unfocused, background) process under CPU contention, exactly when a
    // turn is already doing more work (steering calc, curvature feedforward) and can
    // least afford a missed/delayed 50Hz tick. Elevating THIS thread specifically to
    // TIME_CRITICAL (the highest user-mode thread priority, above what a normal
    // foreground-boosted process reaches) protects the control loop's timing
    // regardless of which window has focus, without changing process-wide priority
    // (which would also affect the LiDAR/IMU/render threads unnecessarily).
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    using namespace std::chrono;
    const auto period = duration_cast<nanoseconds>(duration<double>(1.0/CONTROL_HZ));
    auto next_tick = steady_clock::now();

    while(g_ctrl_thread_run.load()){
        next_tick += period;
        const float dt_fixed = 1.0f / CONTROL_HZ;

        if(g_ctrl_state == CS_RUNNING) pollTelemetry();

        if(!g_simMode && g_ctrl_state != CS_DONE){
            std::lock_guard<std::mutex> lk(ekf_mutex);
            g_ctrl_robot={(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],ekf.seeded};
        }

        ctrl_step(dt_fixed);

        if(g_ctrl_state != CS_RUNNING && g_ctrl.centerRetryTicks > 0){
            if(g_ctrl.centerRetryTicks % 5 == 0) ctrl_sendCmd("STEER,0.00");
            g_ctrl.cmdSteer=0.0f; g_ctrl.lastSentSteer=0.0f;
            if(g_ctrl.centerRetryTicks==1) ctrl_sendCmd("MANUAL");
            g_ctrl.centerRetryTicks--;
        }

        std::this_thread::sleep_until(next_tick);
    }
}

// =============================================================================
//  FIXED TEST PATH  (1 m straight → 1 m left 1° arc → 1 m buffer)
//  Builds g_ctrl_path directly from live EKF pose so the full control loop
//  (Pure Pursuit + EKF + wheel encoder + steering) runs without a map click.
//  Goal trigger fires at ~2 m travelled (after the 1m+1m manoeuvre) because
//  indexCheck (ci≥N-20) and distCheck (<1.8m) both become true at that point.
// =============================================================================
void ctrl_load_fixed_test_path() {
    float sx, sy, syaw;
    {
        std::lock_guard<std::mutex> lk(ekf_mutex);
        if(ekf.seeded) {
            sx = (float)ekf.x[0];
            sy = (float)ekf.x[1];
            syaw = (float)ekf.x[2];
        } else {
            sx = 0.f; sy = 0.f; syaw = 0.f;
        }
    }

    g_ctrl_path.clear();

    const float step  = RESAMPLE_STEP_PP;              // 0.05 m
    const float kappa = tanf(toRad(1.0f)) / WHEELBASE_M; // ~0.00727 m⁻¹, left turn

    float x = sx, y = sy, yaw = syaw;

    // Segment 1: 1 m straight
    for(float d = 0.f; d < 1.0f; d += step) {
        g_ctrl_path.push_back({x, y, yaw, 0.0f});
        x += step * sinf(yaw);
        y += step * cosf(yaw);
    }

    // Segment 2: 1 m left arc at 1° steer curvature
    for(float d = 0.f; d < 1.0f; d += step) {
        g_ctrl_path.push_back({x, y, yaw, kappa});
        x += step * sinf(yaw);
        y += step * cosf(yaw);
        yaw = wrapA(yaw - kappa * step);
    }

    // Segment 3: 1 m buffer — goal is here; ensures both goal conditions fire together
    for(float d = 0.f; d < 1.0f; d += step) {
        g_ctrl_path.push_back({x, y, yaw, 0.0f});
        x += step * sinf(yaw);
        y += step * cosf(yaw);
    }

    if(g_ctrl_path.empty()) {
        g_statusMsg = "Fixed test: path gen failed"; g_statusCol = C_BAD; return;
    }

    g_ctrl_goal  = {x, y, yaw, true};
    g_ctrl_start = {sx, sy, syaw, true};
    g_ctrl_sim   = {sx, sy, g_ctrl_path.front().yaw, true};

    {
        std::lock_guard<std::mutex> lk(ekf_mutex);
        g_ctrl_robot = {(float)ekf.x[0], (float)ekf.x[1], (float)ekf.x[2], ekf.seeded};
    }
    if(!g_ctrl_robot.valid) g_ctrl_robot = g_ctrl_start;

    g_ctrl_trail.clear();
    g_ctrl = {};
    g_ctrl.distToGoal = sqrtf((sx-x)*(sx-x) + (sy-y)*(sy-y));
    g_ctrl_state = CS_LOADED;
    g_statusMsg  = TextFormat("Fixed test: %d pts  1m fwd + 1deg left  — press Start",
                              (int)g_ctrl_path.size());
    g_statusCol  = C_GOOD;
}

// =============================================================================
//  STRAIGHT-LINE PATH  (N metres in current EKF heading)
//  Builds g_ctrl_path directly from the live EKF pose.
//  No reference_path.csv or Step 2 needed — just Localize then drive.
//  Useful for testing straight-line tracking on campus from any start position.
// =============================================================================
void ctrl_load_straight_path(float length_m) {
    float sx, sy, syaw;
    {
        std::lock_guard<std::mutex> lk(ekf_mutex);
        if(ekf.seeded) {
            sx = (float)ekf.x[0];
            sy = (float)ekf.x[1];
            syaw = (float)ekf.x[2];
        } else {
            sx = 0.f; sy = 0.f; syaw = 0.f;
        }
    }

    // Auto-centre the path in the corridor using the active distance map.
    // Samples laterally left/right to find the nearest wall on each side,
    // then shifts the path start toward the corridor centre.  Keeps the cart
    // away from walls where scan matching quality is poor (low map density
    // near corridor edges).
    float centering_shift = 0.f;
    if(active_map_ptr && active_map_ptr->loaded) {
        const float rwx = cosf(syaw), rwy = sinf(syaw);  // right-lateral direction
        const float STEP = 0.15f, MAX_D = 3.5f, WALL_T = 0.3f;
        float min_r = MAX_D, min_l = MAX_D;
        for(float d = 0.3f; d <= MAX_D; d += STEP) {
            if(active_map_ptr->dist_map.get_dist(sx + rwx*d, sy + rwy*d) < WALL_T)
                { min_r = d; break; }
        }
        for(float d = 0.3f; d <= MAX_D; d += STEP) {
            if(active_map_ptr->dist_map.get_dist(sx - rwx*d, sy - rwy*d) < WALL_T)
                { min_l = d; break; }
        }
        if(min_r < MAX_D && min_l < MAX_D && fabsf(min_r - min_l) > 0.3f) {
            centering_shift = (min_r - min_l) * 0.5f;  // negative = shift left
            centering_shift = clampf(centering_shift, -2.f, 2.f);
            sx += centering_shift * rwx;
            sy += centering_shift * rwy;
        }
    }

    g_ctrl_path.clear();
    const float step = RESAMPLE_STEP_PP;
    float x = sx, y = sy;
    for(float d = 0.f; d <= length_m; d += step) {
        g_ctrl_path.push_back({x, y, syaw, 0.0f});
        x += step * sinf(syaw);
        y += step * cosf(syaw);
    }

    if(g_ctrl_path.empty()) {
        g_statusMsg = "Straight path: gen failed"; g_statusCol = C_BAD; return;
    }

    g_ctrl_goal  = {x, y, syaw, true};
    g_ctrl_start = {sx, sy, syaw, true};
    g_ctrl_sim   = {sx, sy, syaw, true};

    {
        std::lock_guard<std::mutex> lk(ekf_mutex);
        g_ctrl_robot = {(float)ekf.x[0], (float)ekf.x[1], (float)ekf.x[2], ekf.seeded};
    }
    if(!g_ctrl_robot.valid) g_ctrl_robot = g_ctrl_start;

    g_ctrl_trail.clear();
    g_ctrl = {};
    g_ctrl.distToGoal = length_m;
    g_ctrl_state = CS_LOADED;
    if(fabsf(centering_shift) > 0.1f)
        g_statusMsg = TextFormat("Straight %.0fm  centred %.1fm %s — press Start",
                                 length_m, fabsf(centering_shift),
                                 centering_shift < 0.f ? "left" : "right");
    else
        g_statusMsg = TextFormat("Straight %.0fm  (%d pts) — press Start",
                                 length_m, (int)g_ctrl_path.size());
    g_statusCol = C_GOOD;
}
