// =============================================================================
// planning.cpp  —  Path smoothing, interpolation, trajectory loading,
//                  corridor generation, plan saving
// =============================================================================
#include "common.hpp"

// =============================================================================
//  GLOBALS (owned here)
// =============================================================================
std::vector<PPPoint>  g_smooth;
std::vector<PPPoint>  g_corridor_L, g_corridor_R;
std::vector<Vector2>  g_approach_path, g_corridor_path, g_goal_path;
PPPose g_pp_robot = {0,0,0,false};
PPPose g_pp_goal  = {0,0,0,false};
std::atomic<bool>     g_path_smoothing_enabled{true};  // Step 2 toggle
std::atomic<bool>     g_show_path_overlay{true};        // Step 2 toggle

// ── Obstacle/allowable-area grid (2026-07-10) — see common.hpp comment ──
std::map<std::pair<int,int>, ObstacleCellType> g_obstacle_grid;
bool g_obstacle_test_mode = false;
std::atomic<bool> g_grid_ped_stop{false};
float g_obst_blocked_msg_timer = 0.f;

// Saved by pp_trigger_plan() so pp_save_plan() can use them directly
// without re-scanning g_smooth (re-scan can produce backward idx if path near-loops)
static int g_corridor_idx_start = 0;
static int g_corridor_idx_end   = 0;

LockedPose g_confirmed_start;

// =============================================================================
//  INTERNAL HELPERS
// =============================================================================
static float pp_fdist(float ax, float ay, float bx, float by) {
    float dx=ax-bx, dy=ay-by; return sqrtf(dx*dx+dy*dy);
}
static float pp_fwrap(float a) {
    while(a > kPI) a -= 2*kPI; while(a < -kPI) a += 2*kPI; return a;
}

static void pp_downsample(std::vector<float>& xs, std::vector<float>& ys, float step_m) {
    if(xs.size() < 2) return;
    std::vector<float> dxs, dys;
    dxs.push_back(xs[0]); dys.push_back(ys[0]);
    float accum = 0.0f;
    for(size_t i = 1; i < xs.size(); i++){
        float dx = xs[i]-xs[i-1], dy = ys[i]-ys[i-1];
        accum += sqrtf(dx*dx+dy*dy);
        if(accum >= step_m){ dxs.push_back(xs[i]); dys.push_back(ys[i]); accum=0.0f; }
    }
    if(dxs.empty() || pp_fdist(dxs.back(),dys.back(),xs.back(),ys.back()) > 1e-4f){
        dxs.push_back(xs.back()); dys.push_back(ys.back());
    }
    xs=dxs; ys=dys;
}

static void pp_laplacian(std::vector<float>& xs, std::vector<float>& ys, int passes, float alpha) {
    int n=(int)xs.size(); if(n<3) return;
    std::vector<float> nx(n), ny(n);
    for(int it=0;it<passes;it++){
        nx[0]=xs[0];ny[0]=ys[0];nx[n-1]=xs[n-1];ny[n-1]=ys[n-1];
        for(int i=1;i<n-1;i++){
            float mx=0.5f*(xs[i-1]+xs[i+1]),my=0.5f*(ys[i-1]+ys[i+1]);
            nx[i]=xs[i]+alpha*(mx-xs[i]);ny[i]=ys[i]+alpha*(my-ys[i]);
        }
        xs=nx;ys=ny;
    }
}

// Taubin λ/μ smoothing: a shrinking pass (λ>0) followed by an expanding pass
// (μ<-λ) per iteration. Removes jitter WITHOUT shrinking the curve toward the
// inside wall (plain Laplacian pulls corners inward; this cancels that).

static std::vector<PPPoint> pp_catmull(const std::vector<float>& xs, const std::vector<float>& ys, float sp) {
    std::vector<PPPoint> out; int n=(int)xs.size(); if(n<2) return out;
    out.push_back({xs[0],ys[0],0,0}); float accum=0;
    for(int seg=1;seg<n;seg++){
        int p0=(seg>1)?seg-2:seg-1,p1=seg-1,p2=seg,p3=(seg<n-1)?seg+1:seg;
        float sl=pp_fdist(xs[p1],ys[p1],xs[p2],ys[p2]); if(sl<1e-6f) continue;
        int sub=(int)(sl/0.01f)+2; float prev_cx=xs[p1],prev_cy=ys[p1];
        for(int s=1;s<=sub;s++){
            float t=(float)s/sub,t2=t*t,t3=t2*t;
            float cx=0.5f*((-xs[p0]+3*xs[p1]-3*xs[p2]+xs[p3])*t3+(2*xs[p0]-5*xs[p1]+4*xs[p2]-xs[p3])*t2+(-xs[p0]+xs[p2])*t+2*xs[p1]);
            float cy=0.5f*((-ys[p0]+3*ys[p1]-3*ys[p2]+ys[p3])*t3+(2*ys[p0]-5*ys[p1]+4*ys[p2]-ys[p3])*t2+(-ys[p0]+ys[p2])*t+2*ys[p1]);
            float step=pp_fdist(cx,cy,prev_cx,prev_cy);accum+=step;prev_cx=cx;prev_cy=cy;
            if(accum>=sp){out.push_back({cx,cy,0,0});accum-=sp;}
        }
    }
    out.push_back({xs[n-1],ys[n-1],0,0}); return out;
}

static void pp_annotate() {
    if(g_smooth.size()<2) return;
    for(size_t i=0;i<g_smooth.size();i++){
        size_t a=(i>0)?i-1:0, b=(i<g_smooth.size()-1)?i+1:g_smooth.size()-1;
        g_smooth[i].yaw=atan2f(g_smooth[b].x-g_smooth[a].x, g_smooth[b].y-g_smooth[a].y);
    }
    for(size_t i=1;i<g_smooth.size()-1;i++)
        g_smooth[i].curvature=pp_fwrap(g_smooth[i+1].yaw-g_smooth[i].yaw)/
                              (pp_fdist(g_smooth[i].x,g_smooth[i].y,g_smooth[i+1].x,g_smooth[i+1].y)+1e-6f);
    if(g_smooth.size() >= 3){
        std::vector<float> sc(g_smooth.size(), 0.f);
        sc[0] = g_smooth[0].curvature;
        sc[g_smooth.size()-1] = g_smooth.back().curvature;
        for(size_t i=1;i<g_smooth.size()-1;i++)
            sc[i] = (g_smooth[i-1].curvature + g_smooth[i].curvature + g_smooth[i+1].curvature) / 3.0f;
        for(size_t i=0;i<g_smooth.size();i++) g_smooth[i].curvature = sc[i];
    }
    g_corridor_L.clear(); g_corridor_R.clear();
    for(auto& p : g_smooth){
        float nx=-cosf(p.yaw)*CORRIDOR_WIDTH_M, ny=sinf(p.yaw)*CORRIDOR_WIDTH_M;
        g_corridor_L.push_back({p.x+nx, p.y+ny, 0, 0});
        g_corridor_R.push_back({p.x-nx, p.y-ny, 0, 0});
    }
}

static int pp_nearest(float qx, float qy) {
    int best=0; float bd=1e9f;
    for(size_t i=0;i<g_smooth.size();i++){
        float d=pp_fdist(g_smooth[i].x,g_smooth[i].y,qx,qy);
        if(d<bd){bd=d;best=(int)i;}
    }
    return best;
}

static std::vector<Vector2> pp_linear_interpolate(const PPPose& start, const PPPoint& end) {
    std::vector<Vector2> pts;
    float dist=pp_fdist(start.x,start.y,end.x,end.y); if(dist<0.1f) return pts;
    int steps=std::max(10,(int)(dist/RESAMPLE_STEP_PP));
    for(int i=0;i<=steps;i++){
        float t=(float)i/steps;
        pts.push_back({start.x + t*(end.x-start.x), start.y + t*(end.y-start.y)});
    }
    return pts;
}

// Straight-line goal connector: goes directly from path exit to goal with no curve.
static std::vector<Vector2> pp_bezier_goal_connector(const PPPoint& start, float gx, float gy) {
    std::vector<Vector2> pts;
    float dist = pp_fdist(start.x, start.y, gx, gy);
    if(dist < 0.1f) return pts;
    int steps = std::max(2, (int)(dist / RESAMPLE_STEP_PP));
    for(int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        pts.push_back({start.x + t*(gx - start.x),
                       start.y + t*(gy - start.y)});
    }
    return pts;
}

// Cubic Bézier approach: departs in robot heading, arrives tangent to path direction.
// Eliminates the heading kink that a straight-line approach creates at the join.
static std::vector<Vector2> pp_bezier_approach(const PPPose& start, const PPPoint& end) {
    std::vector<Vector2> pts;
    float dist = pp_fdist(start.x, start.y, end.x, end.y);
    if(dist < 0.1f) return pts;
    float ctrl = dist * 0.4f;
    // Robot yaw is CCW-positive (north=0): forward direction = (-sin(yaw), cos(yaw))
    // Path yaw uses atan2f(dx,dy) = CW-positive: forward direction = (sin(yaw), cos(yaw))
    float p1x = start.x - ctrl * sinf(start.yaw);
    float p1y = start.y + ctrl * cosf(start.yaw);
    float p2x = end.x   - ctrl * sinf(end.yaw);
    float p2y = end.y   - ctrl * cosf(end.yaw);
    int steps = std::max(10, (int)(dist / RESAMPLE_STEP_PP));
    for(int i = 0; i <= steps; i++) {
        float t = (float)i / steps, u = 1.f - t;
        pts.push_back({u*u*u*start.x + 3*u*u*t*p1x + 3*u*t*t*p2x + t*t*t*end.x,
                       u*u*u*start.y + 3*u*u*t*p1y + 3*u*t*t*p2y + t*t*t*end.y});
    }
    return pts;
}

// =============================================================================
//  RECORDED-PATH SNAPSHOT  (Step 1 record buttons)
// =============================================================================
// Snapshot the live localization trajectory to a per-map file (trajectory CSV
// format, matching pp_run_pipeline's reader). Returns points saved, or -1 on fail.
int pp_record_path_snapshot(const std::string& fn) {
    std::lock_guard<std::mutex> lk(trajectory_mutex);
    if(trajectory_data.size() < 3) return -1;
    std::ofstream f(fn);
    if(!f.is_open()) return -1;
    f << "x,y,z,yaw,rmse,inlier_ratio,timestamp,is_turning\n" << std::fixed << std::setprecision(6);
    for(const auto& p : trajectory_data)
        f << p.x << "," << p.y << "," << p.z << "," << p.yaw << "," << p.rmse << ","
          << p.inlier_ratio << "," << p.timestamp << "," << (p.is_turning?1:0) << "\n";
    return (int)trajectory_data.size();
}

// Shift a stretch [lo,hi] of the reference path g_smooth perpendicular to travel by
// delta_m metres (signed: + = left of travel, - = right), tapering the outer ~25% at
// each end with a raised-cosine ramp so the shifted section blends smoothly back into
// the unshifted path (no kink). Applied incrementally to the current g_smooth; caller
// re-triggers the plan preview. Refreshes corridors/yaw via pp_annotate().
void pp_shift_path_segment(int lo, int hi, float delta_m) {
    int n = (int)g_smooth.size();
    if(n < 3 || lo<0 || hi<0) return;
    if(lo>hi) std::swap(lo,hi);
    lo = std::max(0,lo); hi = std::min(n-1,hi);
    int span = hi - lo;
    if(span < 1) return;

    // ONE shift direction for the whole stretch = perpendicular to its overall chord
    // (g_smooth[lo]→g_smooth[hi]), so the section slides sideways as a rigid parallel
    // offset. A per-point local normal wobbles on any non-perfectly-smooth path and,
    // over repeated nudges, amplifies into a zigzag/fishbone — a single direction can't.
    float cx=g_smooth[hi].x-g_smooth[lo].x, cy=g_smooth[hi].y-g_smooth[lo].y;
    float cl=sqrtf(cx*cx+cy*cy);
    float nx, ny;
    if(cl > 1e-3f){ nx=-cy/cl; ny=cx/cl; }   // left-normal of the chord
    else {                                     // degenerate chord → local tangent at lo
        int b=std::min(n-1,lo+1);
        float tx=g_smooth[b].x-g_smooth[lo].x, ty=g_smooth[b].y-g_smooth[lo].y;
        float tl=sqrtf(tx*tx+ty*ty); if(tl<1e-6f) return;
        nx=-ty/tl; ny=tx/tl;
    }
    int ramp = std::max(1, span/4);   // outer 25% on each side ramps the magnitude 0→1
    for(int j=lo;j<=hi;j++){
        int fromLo=j-lo, fromHi=hi-j;
        float w=1.f;   // raised-cosine window: 0 at the two ends, flat 1 in the middle
        if(fromLo<ramp)      w=0.5f*(1.f-cosf(kPI*(float)fromLo/(float)ramp));
        else if(fromHi<ramp) w=0.5f*(1.f-cosf(kPI*(float)fromHi/(float)ramp));
        g_smooth[j].x += nx*delta_m*w;
        g_smooth[j].y += ny*delta_m*w;
    }
    pp_annotate();
}

// Replace g_smooth[lo..hi] with a straight line between its current endpoints
// (so it still connects exactly where the real curve did), evenly re-spacing the
// same number of points along that line. For a stretch where the real curve is
// tighter than needed and smoother/less aggressive steering is preferred over
// precisely tracing it. Applied to g_smooth same as pp_shift_path_segment; use
// pp_save_edited_path() afterward to persist to the corridor CSV.
void pp_straighten_path_segment(int lo, int hi) {
    int n = (int)g_smooth.size();
    if(n < 3 || lo<0 || hi<0) return;
    if(lo>hi) std::swap(lo,hi);
    lo = std::max(0,lo); hi = std::min(n-1,hi);
    int span = hi - lo;
    if(span < 1) return;
    float ax=g_smooth[lo].x, ay=g_smooth[lo].y;
    float bx=g_smooth[hi].x, by=g_smooth[hi].y;
    for(int j=lo;j<=hi;j++){
        float t = (float)(j-lo)/(float)span;
        g_smooth[j].x = ax + t*(bx-ax);
        g_smooth[j].y = ay + t*(by-ay);
    }
    pp_annotate();
}

// Write the (possibly edited) reference path g_smooth back to the active map's recorded
// corner CSV, in the same trajectory format pp_run_pipeline reads, so the correction
// persists and reloads on the next Build/plan. Returns false if there's no active
// recorded file or g_smooth is empty.
std::string pp_active_map_recorded_path();   // fwd decl (defined below)
void pp_invalidate_corr_cache();             // fwd decl (defined below)
bool pp_save_edited_path() {
    if(g_smooth.size() < 2) return false;
    std::string path = pp_active_map_recorded_path();
    if(path.empty()) return false;
    std::ofstream f(path);
    if(!f.is_open()) return false;
    f << "x,y,z,yaw,rmse,inlier_ratio,timestamp,is_turning\n" << std::fixed << std::setprecision(6);
    int ts=0;
    for(const auto& p : g_smooth)
        f << p.x << "," << p.y << ",0.200000,0.000000,0.000000,1.000000," << ts++ << ".000000,0\n";
    pp_invalidate_corr_cache();   // the on-disk corridor changed → drop cached copy
    return true;
}

// ── "Place Danger Path" tool (2026-07-07) ──────────────────────────────────────
// Lets the operator drag/rotate the OLD campus danger-zone curve (danger_zone_
// forward.csv or _reverse.csv, matching whichever test map is active — a DIFFERENT
// map's coordinate frame in both cases, no known transform to this one) into position
// on the new map by eye, then splices the result into g_smooth. Same idea as
// pp_shift_path_segment/pp_save_edited_path above, but placing an entirely different
// recorded curve rather than nudging the existing one.

// Loads the raw (untransformed) curve once, from whichever source file matches the
// active test map. The raw coordinates are the OLD campus map's frame (~100m away
// numerically from this map's own frame), so starting the offset at identity would
// spawn it far off-screen with no way to find it — instead the initial offset snaps
// the curve's centroid onto the current sub-zone entry marker (g_dzsub_entry_mk_x/y),
// which is roughly where the operator has already been working, so it's visible
// immediately and just needs fine-tuning by drag.
bool load_dzplace_raw() {
    g_dzplace_raw.clear();
    std::string src;
    if(active_map_ptr && active_map_ptr->name=="Danger Zone Rev") src="danger_zone_reverse.csv";
    else                                                          src="danger_zone_forward.csv";
    std::ifstream f(src);
    if(!f.is_open()) return false;
    std::string line; std::getline(f,line);   // header
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); float x,y; char c;
        if(ss>>x>>c>>y) g_dzplace_raw.push_back({x,y,0.f,0.f});
    }
    if(g_dzplace_raw.size()<2) return false;
    double sx=0,sy=0;
    for(auto& p:g_dzplace_raw){ sx+=p.x; sy+=p.y; }
    g_dzplace_cx=(float)(sx/g_dzplace_raw.size());
    g_dzplace_cy=(float)(sy/g_dzplace_raw.size());
    g_dzplace_offset_x = g_dzsub_entry_mk_x - g_dzplace_cx;
    g_dzplace_offset_y = g_dzsub_entry_mk_y - g_dzplace_cy;
    g_dzplace_rotation=0.f;
    return true;
}

// Applies the current drag offset + rotation-about-own-centroid to g_dzplace_raw.
// Rotating about the curve's own centroid (not the world origin, which sits nowhere
// near this curve's raw coordinates) means it spins in place under the cursor instead
// of swinging wildly — same reasoning as the map-calibration tool's anchor-and-reanchor
// approach, simplified since there's no background image to keep registered here.
void dzplace_transformed_points(std::vector<PPPoint>& out) {
    out.clear();
    float c=cosf(g_dzplace_rotation), s=sinf(g_dzplace_rotation);
    for(auto& p : g_dzplace_raw){
        float dx=p.x-g_dzplace_cx, dy=p.y-g_dzplace_cy;
        float rx=dx*c-dy*s, ry=dx*s+dy*c;
        out.push_back({g_dzplace_cx+rx+g_dzplace_offset_x, g_dzplace_cy+ry+g_dzplace_offset_y, 0.f, 0.f});
    }
}

// Splices the transformed curve into g_smooth — finds the nearest g_smooth point to
// each of the curve's own endpoints (mirrors ctrl_splice_danger_zone's nearestIdx
// approach) and replaces everything between them, oriented to match g_smooth's own
// direction. Persists via pp_save_edited_path() and keeps the sub-zone markers in
// sync with the newly placed curve's endpoints so entry/exit tracking still lines up.
bool dzplace_save_into_smooth() {
    if(g_dzplace_raw.size()<2 || g_smooth.size()<2) return false;
    std::vector<PPPoint> xf;
    dzplace_transformed_points(xf);

    int N=(int)g_smooth.size();
    auto nearestIdx=[&](float x,float y)->int{
        int bi=0; float bd=1e18f;
        for(int i=0;i<N;i++){
            float dx=g_smooth[i].x-x, dy=g_smooth[i].y-y, d=dx*dx+dy*dy;
            if(d<bd){bd=d;bi=i;}
        }
        return bi;
    };
    int a = nearestIdx(xf.front().x, xf.front().y);
    int b = nearestIdx(xf.back().x,  xf.back().y);
    int lo = std::min(a,b), hi = std::max(a,b);
    if(hi-lo < 1) return false;
    bool curveForward = (a<=b);   // orient the placed curve to run lo->hi like g_smooth

    std::vector<PPPoint> spliced;
    spliced.reserve(lo + (int)xf.size() + (N-hi));
    for(int i=0;i<lo;i++) spliced.push_back(g_smooth[i]);
    if(curveForward) for(auto& p:xf) spliced.push_back(p);
    else              for(auto it=xf.rbegin(); it!=xf.rend(); ++it) spliced.push_back(*it);
    for(int i=hi+1;i<N;i++) spliced.push_back(g_smooth[i]);
    g_smooth.swap(spliced);
    pp_annotate();

    g_dzsub_entry_mk_x=xf.front().x; g_dzsub_entry_mk_y=xf.front().y;
    g_dzsub_exit_mk_x =xf.back().x;  g_dzsub_exit_mk_y =xf.back().y;
    save_dzsub_fwd();

    return pp_save_edited_path();
}

// pp_save_danger_zone() and its helpers (pp_nearest_either_zone, pp_zone_corridor_point)
// live further down, right after the zone-corridor cache they depend on.

// Clear all planned-path state — called on map switch so a path built for one
// map's coordinate frame doesn't linger (stale corridor curve) on another map.
void pp_clear_path() {
    g_smooth.clear();
    g_corridor_L.clear(); g_corridor_R.clear();
    g_approach_path.clear(); g_corridor_path.clear(); g_goal_path.clear();
    g_pp_goal.valid = false;
    g_journey_dest_name.clear();   // pin label follows the goal it labelled
}

// ── Obstacle/allowable-area grid (2026-07-10; static-avoidance 2026-07-19) ──────
bool grid_cell_in_band(int col, int row) {
    if(g_smooth.empty()) return false;
    float cx, cy; grid_cell_center(col, row, cx, cy);
    for(size_t i=0;i<g_smooth.size();i++){
        if(pp_fdist(cx,cy,g_smooth[i].x,g_smooth[i].y) <= GRID_BAND_M) return true;
    }
    return false;
}

// fwd decls — grid_click_cycle below triggers a fresh replan when a STATIC cell is
// added/removed; both are defined later in this file.
void pp_trigger_plan();
void pp_save_plan();

void grid_click_cycle(float wx, float wy) {
    // Snap to the nearest in-band cell within one cell's radius of the click, same
    // "search a small neighborhood" pattern as the other click-to-grid tools in this
    // file — a raw floor(wx/CELL) would force exact-cell precision, unforgiving to click.
    auto [c0,r0] = grid_cell_of(wx, wy);
    int bestC=0, bestR=0; float bestD=1e18f; bool found=false;
    for(int dc=-2; dc<=2; dc++) for(int dr=-2; dr<=2; dr++){
        int c=c0+dc, r=r0+dr;
        if(!grid_cell_in_band(c,r)) continue;
        float cx,cy; grid_cell_center(c,r,cx,cy);
        float d = pp_fdist(wx,wy,cx,cy);
        if(d<bestD){ bestD=d; bestC=c; bestR=r; found=true; }
    }
    if(!found) return;
    auto key = std::make_pair(bestC,bestR);
    auto it = g_obstacle_grid.find(key);
    ObstacleCellType cur = (it!=g_obstacle_grid.end()) ? it->second : OBST_NONE;
    ObstacleCellType next = (cur==OBST_NONE) ? OBST_PEDESTRIAN : (cur==OBST_PEDESTRIAN) ? OBST_STATIC : OBST_NONE;
    if(next==OBST_NONE) g_obstacle_grid.erase(key);
    else g_obstacle_grid[key] = next;

    // Replan only when a STATIC cell was just ADDED or REMOVED (cur/next==OBST_STATIC
    // covers both transitions) — the actual detour happens inside pp_save_plan() via
    // pp_apply_obstacle_avoidance(). PEDESTRIAN-only transitions (NONE<->PEDESTRIAN)
    // deliberately do NOT replan — path stays exactly as it was, per spec: a dynamic
    // (pedestrian) marker is a stop trigger, not a routing input. No-ops harmlessly if
    // no plan exists yet (both functions already guard on a valid robot/goal).
    //
    // ctrl_hotswap_path() (2026-07-19) then reloads the just-rewritten CSV straight
    // into the ACTIVELY-DRIVEN g_ctrl_path — thread-safe (see g_ctrl_path_mutex in
    // control.cpp), so a Step-3 obstacle placed mid-drive reroutes the cart live, no
    // save/confirm step. Harmless to call in Step 2 too / before any drive starts —
    // it no-ops until the robot pose and a valid path both exist.
    if(cur==OBST_STATIC || next==OBST_STATIC){
        // g_pp_robot (the planning-side start pose) is normally only kept synced to
        // the live EKF pose while in STEP_PLAN (main.cpp) — without this, a Step-3
        // mid-drive replan would build a route starting from wherever the cart was
        // last time Step 2 was open, not from where it actually is right now. Sync
        // it here explicitly so this trigger works correctly from either step.
        // Left untouched if there's no valid live pose yet (e.g. not localized).
        const CtrlPose& live = g_simMode ? g_ctrl_sim : g_ctrl_robot;
        if(live.valid) g_pp_robot = {live.x, live.y, live.yaw, true};

        pp_trigger_plan();
        pp_save_plan();
        ctrl_hotswap_path();
    }
}

void grid_clear_all() { g_obstacle_grid.clear(); }

bool grid_check_pedestrian_stop(float rx, float ry) {
    for(auto& kv : g_obstacle_grid){
        if(kv.second != OBST_PEDESTRIAN) continue;
        float cx,cy; grid_cell_center(kv.first.first, kv.first.second, cx, cy);
        if(pp_fdist(rx,ry,cx,cy) <= GRID_PED_STOP_RADIUS_M) return true;
    }
    return false;
}

// ── Border/wall grid cells (2026-07-10) ──────────────────────────────────────
std::map<std::pair<int,int>, bool> g_border_grid;
bool g_border_edit_mode = false;
bool g_show_border_cells = false;

void border_auto_generate() {
    // Perpendicular from the actual SEGMENT direction (b-a), NOT point.yaw — same
    // reasoning as the friendly-panel route-band fix: recorded top-LiDAR paths carry
    // yaw=0 on many points, which would scatter these badly if trusted.
    // Placed at the LAST (outermost) in-band cell, i.e. right at GRID_BAND_M — not at
    // CORRIDOR_HALF_M — per 2026-07-10 request. Offset by half a cell short of
    // GRID_BAND_M so the resulting point's cell reliably resolves as in-band (a point
    // placed at exactly the band edge could round into the next cell out, which
    // grid_cell_in_band() would then reject).
    if(g_smooth.size()<2) return;
    const float edgeOffset = GRID_BAND_M - GRID_CELL_M*0.5f;
    for(size_t i=0;i<g_smooth.size();i++){
        float dx,dy;
        if(i+1<g_smooth.size()){ dx=g_smooth[i+1].x-g_smooth[i].x; dy=g_smooth[i+1].y-g_smooth[i].y; }
        else { dx=g_smooth[i].x-g_smooth[i-1].x; dy=g_smooth[i].y-g_smooth[i-1].y; }
        float len=sqrtf(dx*dx+dy*dy);
        if(len<1e-5f) continue;
        float nx=-dy/len*edgeOffset, ny=dx/len*edgeOffset;
        g_border_grid[grid_cell_of(g_smooth[i].x+nx, g_smooth[i].y+ny)] = true;
        g_border_grid[grid_cell_of(g_smooth[i].x-nx, g_smooth[i].y-ny)] = true;
    }
}

void border_click_toggle(float wx, float wy) {
    auto [c0,r0] = grid_cell_of(wx, wy);
    int bestC=0, bestR=0; float bestD=1e18f; bool found=false;
    for(int dc=-2; dc<=2; dc++) for(int dr=-2; dr<=2; dr++){
        int c=c0+dc, r=r0+dr;
        if(!grid_cell_in_band(c,r)) continue;
        float cx,cy; grid_cell_center(c,r,cx,cy);
        float d = pp_fdist(wx,wy,cx,cy);
        if(d<bestD){ bestD=d; bestC=c; bestR=r; found=true; }
    }
    if(!found) return;
    auto key = std::make_pair(bestC,bestR);
    if(g_border_grid.count(key)) g_border_grid.erase(key);
    else g_border_grid[key] = true;
}

void border_clear_all() { g_border_grid.clear(); }

std::string border_file_for_map() {
    if(!active_map_ptr) return "";
    std::string fn = active_map_ptr->filename;
    size_t dot = fn.rfind('.'); if(dot!=std::string::npos) fn = fn.substr(0,dot);
    return "bordercells_" + fn + ".cfg";
}
void save_border_grid() {
    std::string path = border_file_for_map();
    if(path.empty()) return;
    std::ofstream f(path);
    f << "# Border/wall grid cells (col,row) — forbidden, near corridor edge\n";
    for(auto& kv : g_border_grid) if(kv.second) f << kv.first.first << "," << kv.first.second << "\n";
}
void load_border_grid() {
    g_border_grid.clear();
    std::string path = border_file_for_map();
    if(path.empty()) return;
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line;
    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        std::istringstream ss(line); int c,r; char comma;
        if(ss>>c>>comma>>r) g_border_grid[{c,r}]=true;
    }
}

// Recorded-path file matching the active map (and whichever zone is armed), or ""
// if the active map has none. Zone 2 armed -> its corner file; otherwise (zone 1
// armed, or neither armed) -> the original zone-1 file, unchanged from before zone 2
// existed.
std::string pp_active_map_recorded_path() {
    if(!active_map_ptr) return "";
    // Danger-zone standalone test map: one recorded path per direction, no zones.
    if(active_map_ptr->name == "Danger Zone Fwd") return PATH_DANGERZONE_FWD_FILE;
    if(active_map_ptr->name == "Danger Zone Rev") return PATH_DANGERZONE_REV_FILE;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    bool rev = active_map_ptr->name == "Campus Top (Rev)";
    if(!fwd && !rev) return "";
    if(g_zone2_armed) return fwd ? PATH_FORWARD2_FILE : PATH_REVERSE2_FILE;
    return fwd ? PATH_FORWARD_FILE : PATH_REVERSE_FILE;
}

// Restores zone 1's or zone 2's corridor file (for whichever top-LiDAR map is active)
// from the "golden_paths/" backup — a known-good snapshot taken outside of any live
// editing/save path. Recovery for corruption like the July 2026 incident where a
// corner file got silently truncated: with no known-good copy on disk there was no
// way back except re-deriving it by hand. zone: 1 or 2.
bool pp_run_pipeline(const std::string& raw_file);   // fwd decl (defined below)
void pp_trigger_plan();                              // fwd decl (defined below)
bool pp_reset_zone_path(int zone) {
    if(!active_map_ptr) return false;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    bool rev = active_map_ptr->name == "Campus Top (Rev)";
    if(!fwd && !rev) return false;
    std::string fname = (zone==2) ? (fwd ? PATH_FORWARD2_FILE : PATH_REVERSE2_FILE)
                                   : (fwd ? PATH_FORWARD_FILE  : PATH_REVERSE_FILE);
    std::string golden = "golden_paths/" + fname;
    std::ifstream src(golden, std::ios::binary);
    if(!src.is_open()) return false;
    std::ofstream dst(fname, std::ios::binary);
    if(!dst.is_open()) return false;
    dst << src.rdbuf();
    src.close(); dst.close();
    // Drop the cached copy so cross-zone routing re-reads the restored file.
    // If the reset zone isn't the one currently armed, this reload/replan is a
    // harmless no-op on the still-active zone's own (unaffected) file — the
    // restored data takes effect next time that zone is armed.
    pp_invalidate_corr_cache();
    pp_run_pipeline(TRAJ_CSV_FILE);
    if(g_pp_goal.valid) pp_trigger_plan();
    return true;
}

// Loads and smooths a recorded corner trajectory CSV into a caller-supplied vector,
// independent of the global g_smooth / active-map / armed-zone state. Duplicates
// pp_run_pipeline's "Priority 3: trajectory CSV" branch on purpose (rather than
// refactoring it to share code) so it can never change pp_run_pipeline's validated
// behavior. Used by pp_trigger_plan()'s cross-zone chaining, which needs BOTH zones'
// corridors loaded at once to (a) decide which corridor the robot/goal are nearest to
// and (b) stitch them together.
static bool load_and_smooth_trajectory(const std::string& path, std::vector<PPPoint>& out) {
    struct TrajRow{float x,y,z,yaw,rmse,ir;double ts;int turn;};
    std::vector<TrajRow> raw;
    std::ifstream f(path); if(!f.is_open()) return false;
    std::string line; std::getline(f,line);
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); TrajRow p; char c;
        ss>>p.x>>c>>p.y>>c>>p.z>>c>>p.yaw>>c>>p.rmse>>c>>p.ir>>c>>p.ts>>c>>p.turn;
        if(p.ir>=FILTER_MIN_QUALITY) raw.push_back(p);
    }
    if(raw.size()<3) return false;
    std::vector<float>xs(raw.size()),ys(raw.size());
    for(size_t i=0;i<raw.size();i++){xs[i]=raw[i].x;ys[i]=raw[i].y;}
    out.clear();
    if(g_path_smoothing_enabled.load()){
        pp_downsample(xs, ys, DOWNSAMPLE_STEP_M);
        pp_laplacian(xs, ys, SMOOTH_PASSES, SMOOTH_ALPHA);
        out = pp_catmull(xs, ys, RESAMPLE_STEP_PP);
    } else {
        pp_downsample(xs, ys, DOWNSAMPLE_STEP_M);
        for(size_t i=0;i<xs.size();i++) out.push_back({xs[i], ys[i], 0.f, 0.f});
    }
    if(out.empty()) return false;
    // Fill .yaw from neighboring points (CW-positive atan2f(dx,dy), matching the
    // existing path-yaw convention) — only g_pp_goal.yaw reads this at the chain end.
    for(size_t i=0;i<out.size();i++){
        size_t j = (i+1<out.size()) ? i+1 : i;
        size_t k = (i>0) ? i-1 : i;
        out[i].yaw = atan2f(out[j].x-out[k].x, out[j].y-out[k].y);
    }
    return true;
}

// Nearest point index in a corridor to (qx,qy) + that distance.
static int corr_nearest(const std::vector<PPPoint>& corr, float qx, float qy, float& outDist) {
    int best=0; float bd=1e30f;
    for(size_t i=0;i<corr.size();i++){
        float d=pp_fdist(corr[i].x,corr[i].y,qx,qy);
        if(d<bd){bd=d;best=(int)i;}
    }
    outDist=bd; return best;
}

// ── Cross-zone corridor cache ────────────────────────────────────────────────
// pp_trigger_plan() fires on every robot-move frame during Step 2 tracking (and on
// every goal pick / nudge). Loading + 40-pass-smoothing the two corner corridors
// (up to ~3600 pts) plus the full recorded drive (~8000 pts) on every one of those
// calls was the dominant planning cost. Cache them, keyed by active map + smoothing
// mode; reload only when that key changes or a path edit invalidates it.
static std::string          g_corr_key, g_corr_full_key;
static std::vector<PPPoint> g_corr_c1, g_corr_c2, g_corr_full;
static bool                 g_corr_c1_ok=false, g_corr_c2_ok=false, g_corr_full_ok=false;

// Loads the two SHORT corner corridors (needed just to decide which zone the robot /
// goal are nearest to). Cheap-ish; refreshed on map / smoothing change.
static void corr_cache_refresh() {
    if(!active_map_ptr){ g_corr_c1_ok=g_corr_c2_ok=false; g_corr_key.clear(); return; }
    std::string key = active_map_ptr->name + (g_path_smoothing_enabled.load()?"|S":"|R");
    if(key == g_corr_key) return;   // cache still valid
    g_corr_key = key;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    g_corr_c1_ok = load_and_smooth_trajectory(fwd?PATH_FORWARD_FILE :PATH_REVERSE_FILE,  g_corr_c1);
    g_corr_c2_ok = load_and_smooth_trajectory(fwd?PATH_FORWARD2_FILE:PATH_REVERSE2_FILE, g_corr_c2);
}
// Loads the big full recorded drive (~8000 pts) — only when a cross-zone trip actually
// needs it, so same-zone planning never pays for it.
static void corr_cache_ensure_full() {
    if(!active_map_ptr){ g_corr_full_ok=false; return; }
    std::string key = active_map_ptr->name + (g_path_smoothing_enabled.load()?"|S":"|R");
    if(key == g_corr_full_key) return;
    g_corr_full_key = key;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    g_corr_full_ok = load_and_smooth_trajectory(fwd?PATH_FORWARD_FULL_FILE:PATH_REVERSE_FULL_FILE, g_corr_full);
}
// Force the next refresh to reload (call when a corridor CSV changes on disk while the
// map/smoothing key stays the same — e.g. after a path edit is saved).
void pp_invalidate_corr_cache() { g_corr_key.clear(); g_corr_full_key.clear(); }

// Which zone's recorded corridor a world point is nearest to — 0/1, or -1 if neither
// corridor is within max_dist_m. Used instead of point_in_polygon() for auto-arm-on-
// localize: the hand-drawn zone boundary doesn't reliably contain the actual corridor
// points (same fragility fixed for cross-zone routing), so a position genuinely on/near
// the recorded road could fall just outside the polygon and silently arm nothing.
int pp_zone_for_point(float x, float y, float max_dist_m) {
    corr_cache_refresh();
    float d1=1e9f, d2=1e9f;
    if(g_corr_c1_ok) corr_nearest(g_corr_c1, x, y, d1);
    if(g_corr_c2_ok) corr_nearest(g_corr_c2, x, y, d2);
    if(d1>max_dist_m && d2>max_dist_m) return -1;
    return (d1<=d2) ? 0 : 1;
}

// Is (rx,ry) within the marked zone-2 slow segment? Nearest-index membership between
// the entry/exit markers' nearest points on zone 2's cached corridor. The [lo,hi]
// index range is cached (keyed by map+smoothing, same pattern as the corridor cache
// itself) since this runs every control tick and must stay cheap; the range only
// changes when the map switches or the saved markers change (pp_invalidate_corr_cache()
// already gets called on both — same invalidation the rest of the corridor cache uses).
bool pp_zone2_slow_active(float rx, float ry) {
    if(!g_z2slow_valid) return false;
    corr_cache_refresh();
    if(!g_corr_c2_ok || g_corr_c2.empty()) return false;
    // Re-derive [lo,hi] whenever the corridor identity OR the saved marker values
    // change — comparing the markers directly (not just g_corr_key) means a re-save
    // of the segment on the SAME map takes effect immediately, no invalidation call
    // needed from the save path.
    static std::string s_key;
    static float s_ex=1e18f, s_ey=0.f, s_xx=0.f, s_xy=0.f;
    static int s_lo=-1, s_hi=-1;
    if(g_corr_key!=s_key || s_ex!=g_z2slow_entry_x || s_ey!=g_z2slow_entry_y ||
       s_xx!=g_z2slow_exit_x || s_xy!=g_z2slow_exit_y){
        s_key=g_corr_key; s_ex=g_z2slow_entry_x; s_ey=g_z2slow_entry_y;
        s_xx=g_z2slow_exit_x; s_xy=g_z2slow_exit_y;
        float d;
        int ia = corr_nearest(g_corr_c2, g_z2slow_entry_x, g_z2slow_entry_y, d);
        int ib = corr_nearest(g_corr_c2, g_z2slow_exit_x,  g_z2slow_exit_y,  d);
        s_lo = std::min(ia,ib); s_hi = std::max(ia,ib);
    }
    if(s_lo<0) return false;
    float d; int ir = corr_nearest(g_corr_c2, rx, ry, d);
    return ir>=s_lo && ir<=s_hi;
}

// Slice of the FULL recorded drive between the nearest points to (ax,ay) and (bx,by),
// in a→b order. Display-only helper (2026-07-08): the friendly Step 3 panel uses it to
// preview the REMAINING legs of a danger-zone crossing journey (checkpoint → across
// the danger stretch → final destination) — the loaded plan (g_ctrl_path) only covers
// the CURRENT leg, so without this the passenger sees a route that just stops at
// Building A. Never fed into the controller. Corner (top-LiDAR) maps only — the full
// drive files are recorded in those maps' frames.
bool pp_full_drive_slice(float ax, float ay, float bx, float by, std::vector<PPPoint>& out) {
    out.clear();
    if(!active_map_ptr ||
       (active_map_ptr->name!="Campus Top LiDAR" && active_map_ptr->name!="Campus Top (Rev)"))
        return false;
    corr_cache_ensure_full();
    if(!g_corr_full_ok) return false;
    float d;
    int ia=corr_nearest(g_corr_full, ax, ay, d);
    int ib=corr_nearest(g_corr_full, bx, by, d);
    if(ia<=ib) for(int i=ia;i<=ib;i++) out.push_back(g_corr_full[i]);
    else       for(int i=ia;i>=ib;i--) out.push_back(g_corr_full[i]);
    return out.size()>=2;
}

// Nearest point to (x,y) across BOTH zone-1 and zone-2's cached corridors, regardless
// of which is currently armed/loaded into g_smooth. Used by Step 2's "Set Danger
// Zone" tool so zone 1 can be picked even while zone 2 is the active/displayed one.
// Zone membership is decided by whichever zone's hand-drawn POLYGON contains the
// click, not by raw nearest-corridor-point distance — zone 2's corridor is far
// longer than zone 1's short one, so distance-only would almost always resolve to
// zone 2 even for a click clearly meant for zone 1.
// avoidZone (0/1/2): when the click is ambiguous (falls inside BOTH zone polygons —
// happens right at the zone-1/zone-2 boundary, exactly where a danger-zone bridge
// needs to be clicked), prefer the OTHER zone from the one already picked for the
// first point, so two clicks in that same small overlap sliver still resolve to
// different zones instead of both collapsing onto zone 2.
bool pp_nearest_either_zone(float x, float y, int& outZone, int& outIdx, int avoidZone) {
    corr_cache_refresh();
    bool inZ1 = g_zone_valid  && !g_zone_poly.empty()  && point_in_polygon(x,y,g_zone_poly);
    bool inZ2 = g_zone2_valid && !g_zone2_poly.empty() && point_in_polygon(x,y,g_zone2_poly);
    float dtmp;
    if(inZ1 && inZ2){
        if(avoidZone==1 && g_corr_c2_ok){ outZone=2; outIdx=corr_nearest(g_corr_c2,x,y,dtmp); return true; }
        if(avoidZone==2 && g_corr_c1_ok){ outZone=1; outIdx=corr_nearest(g_corr_c1,x,y,dtmp); return true; }
        // No preference -> default to zone 1 (a raw-distance tiebreak would otherwise
        // always favor zone 2's much longer corridor).
        if(g_corr_c1_ok){ outZone=1; outIdx=corr_nearest(g_corr_c1,x,y,dtmp); return true; }
    }
    if(inZ1 && !inZ2 && g_corr_c1_ok){ outZone=1; outIdx=corr_nearest(g_corr_c1,x,y,dtmp); return true; }
    if(inZ2 && !inZ1 && g_corr_c2_ok){ outZone=2; outIdx=corr_nearest(g_corr_c2,x,y,dtmp); return true; }
    // Inside neither polygon -> fall back to whichever corridor is nearest overall.
    float bd1=1e18f, bd2=1e18f;
    int bi1=-1, bi2=-1;
    if(g_corr_c1_ok) bi1 = corr_nearest(g_corr_c1, x, y, bd1);
    if(g_corr_c2_ok) bi2 = corr_nearest(g_corr_c2, x, y, bd2);
    if(bi1<0 && bi2<0) return false;
    if(bi2<0 || (bi1>=0 && bd1<=bd2)) { outZone=1; outIdx=bi1; }
    else                              { outZone=2; outIdx=bi2; }
    return true;
}

// World coords of zone corridor `zone`'s (1 or 2) point at `idx` — for drawing the
// live selection highlight while using the Set Danger Zone tool.
bool pp_zone_corridor_point(int zone, int idx, float& x, float& y) {
    corr_cache_refresh();
    if(zone==1 && g_corr_c1_ok && idx>=0 && idx<(int)g_corr_c1.size()){ x=g_corr_c1[idx].x; y=g_corr_c1[idx].y; return true; }
    if(zone==2 && g_corr_c2_ok && idx>=0 && idx<(int)g_corr_c2.size()){ x=g_corr_c2[idx].x; y=g_corr_c2[idx].y; return true; }
    return false;
}

// Full corridor `zone`'s (1 or 2) points — for drawing BOTH zones' curves at once
// while using the Set Danger Zone tool, since only the currently-armed zone's
// corridor is normally shown as g_smooth (the other one is invisible, making it
// impossible to see where to click without arming it first).
const std::vector<PPPoint>& pp_zone_corridor(int zone) {
    corr_cache_refresh();
    static const std::vector<PPPoint> empty;
    if(zone==1 && g_corr_c1_ok) return g_corr_c1;
    if(zone==2 && g_corr_c2_ok) return g_corr_c2;
    return empty;
}

// Save the danger zone from two (zone,idx) picks. Same corridor -> a plain slice.
// Different corridors -> concatenated in actual recorded drive order for the active
// map (forward: zone1 then zone2; reverse: zone2 then zone1 — matches how each map's
// full trajectory was actually driven). The two corridors may not join up perfectly
// (separately recorded) — the saved curve is shown as the red DANGER ZONE overlay so
// any bad jump at the join is visible immediately; re-pick closer to the boundary if so.
void dz_invalidate();   // fwd decl (defined below) — drop the cached copy after saving
bool pp_save_danger_zone(int zone0, int idx0, int zone1, int idx1) {
    if(!active_map_ptr) return false;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    bool rev = active_map_ptr->name == "Campus Top (Rev)";
    if(!fwd && !rev) return false;
    if(zone0<1 || zone1<1 || idx0<0 || idx1<0) return false;
    corr_cache_refresh();
    std::vector<PPPoint> out;
    if(zone0==zone1){
        const std::vector<PPPoint>& c = (zone0==1) ? g_corr_c1 : g_corr_c2;
        bool ok = (zone0==1) ? g_corr_c1_ok : g_corr_c2_ok;
        if(!ok) return false;
        int lo=std::min(idx0,idx1), hi=std::max(idx0,idx1);
        if(lo<0 || hi>=(int)c.size() || hi-lo<1) return false;
        for(int i=lo;i<=hi;i++) out.push_back(c[i]);
    } else {
        if(!g_corr_c1_ok || !g_corr_c2_ok) return false;
        int z1idx = (zone0==1) ? idx0 : idx1;
        int z2idx = (zone0==2) ? idx0 : idx1;
        if(z1idx<0 || z1idx>=(int)g_corr_c1.size() || z2idx<0 || z2idx>=(int)g_corr_c2.size()) return false;
        if(fwd){
            for(int i=z1idx;i<(int)g_corr_c1.size();i++) out.push_back(g_corr_c1[i]);
            for(int i=0;i<=z2idx;i++) out.push_back(g_corr_c2[i]);
        } else {
            for(int i=z2idx;i<(int)g_corr_c2.size();i++) out.push_back(g_corr_c2[i]);
            for(int i=0;i<=z1idx;i++) out.push_back(g_corr_c1[i]);
        }
    }
    if(out.size()<2) return false;
    std::string path = fwd ? "danger_zone_forward.csv" : "danger_zone_reverse.csv";
    std::ofstream f(path);
    if(!f.is_open()) return false;
    f << "x,y\n" << std::fixed << std::setprecision(6);
    for(auto& p : out) f << p.x << "," << p.y << "\n";
    dz_invalidate();
    return true;
}

// ── DANGER ZONE corridor (July 2026) ────────────────────────────────────────
// danger_zone_forward.csv / danger_zone_reverse.csv mark the stretch where
// scan-matching is known to fail (converges wrong or loses lock, even stationary).
// Editable from Step 2's "Set Danger Zone" tool (pp_save_danger_zone, two clicks on
// the loaded corridor); falls back to whatever was last saved otherwise. Simple
// x,y-only format — no smoothing/yaw needed, this is a position lookup table only.
static std::string          g_dz_key;
static std::vector<PPPoint> g_dz_pts;
static std::vector<float>   g_dz_cum;   // cumulative arc length, same size as g_dz_pts
static bool                 g_dz_ok = false;

// Force the next dz_ensure_loaded() to reload from disk (call after pp_save_danger_zone).
void dz_invalidate() { g_dz_key.clear(); }

static void dz_ensure_loaded() {
    if(!active_map_ptr){ g_dz_ok=false; g_dz_key.clear(); return; }
    if(active_map_ptr->name == g_dz_key) return;
    g_dz_key = active_map_ptr->name;
    g_dz_pts.clear(); g_dz_cum.clear(); g_dz_ok = false;
    bool fwd = active_map_ptr->name == "Campus Top LiDAR";
    bool rev = active_map_ptr->name == "Campus Top (Rev)";
    if(!fwd && !rev) return;
    std::string path = fwd ? "danger_zone_forward.csv" : "danger_zone_reverse.csv";
    std::ifstream f(path); if(!f.is_open()) return;
    std::string line; std::getline(f,line);   // header
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); float x,y; char c;
        if(ss>>x>>c>>y) g_dz_pts.push_back({x,y,0.f,0.f});
    }
    if(g_dz_pts.size()<2) return;
    g_dz_cum.assign(g_dz_pts.size(), 0.f);
    for(size_t i=1;i<g_dz_pts.size();i++)
        g_dz_cum[i] = g_dz_cum[i-1] + pp_fdist(g_dz_pts[i-1].x,g_dz_pts[i-1].y,g_dz_pts[i].x,g_dz_pts[i].y);
    g_dz_ok = true;
}

// Nearest point on the danger-zone curve to (px,py) -> its arc-length + distance.
// Used only to decide zone ENTRY — a tight radius, since this is exact curve
// geometry rather than a broad approximation.
bool dz_nearest(float px, float py, float& outArc, float& outDist) {
    dz_ensure_loaded();
    if(!g_dz_ok) return false;
    float bd=1e18f; int bi=0;
    for(size_t i=0;i<g_dz_pts.size();i++){
        float d=pp_fdist(g_dz_pts[i].x,g_dz_pts[i].y,px,py);
        if(d<bd){bd=d;bi=(int)i;}
    }
    outArc = g_dz_cum[bi]; outDist = bd;
    return true;
}

// Interpolated position at a given arc-length along the danger-zone curve. Returns
// false once arc exceeds the curve's length — caller resumes trusting the normal
// EKF/scan-match pose once the cart has driven past the danger stretch.
bool dz_lookup(float arc, float& outX, float& outY) {
    dz_ensure_loaded();
    if(!g_dz_ok) return false;
    if(arc < 0.f || arc > g_dz_cum.back()) return false;
    for(size_t i=1;i<g_dz_cum.size();i++){
        if(g_dz_cum[i] >= arc){
            float seg = g_dz_cum[i]-g_dz_cum[i-1];
            float t = (seg>1e-6f) ? (arc-g_dz_cum[i-1])/seg : 0.f;
            outX = g_dz_pts[i-1].x + t*(g_dz_pts[i].x-g_dz_pts[i-1].x);
            outY = g_dz_pts[i-1].y + t*(g_dz_pts[i].y-g_dz_pts[i-1].y);
            return true;
        }
    }
    outX=g_dz_pts.back().x; outY=g_dz_pts.back().y;
    return true;
}

// For drawing the danger-zone curve on the map (ui.cpp) — same points used for the
// dead-reckoning lookup above, so the overlay always matches what's actually active.
const std::vector<PPPoint>& pp_danger_zone_pts() {
    dz_ensure_loaded();
    return g_dz_pts;
}

// =============================================================================
//  PUBLIC PLANNING API
// =============================================================================
bool pp_run_pipeline(const std::string& raw_file) {
    // Priority 1: per-map recorded path (forward/reverse). Processed through the
    // trajectory branch below so the Step 2 smoothing toggle applies to it.
    std::string mapPath = pp_active_map_recorded_path();
    bool haveMapPath = false;
    if(!mapPath.empty()){ std::ifstream c(mapPath); haveMapPath = c.is_open(); }

    // Priority 2: reference_path.csv — pre-smoothed, load directly (legacy behavior).
    if(!haveMapPath){
        std::ifstream ref_check(CSV_REF_FILE);
        if(ref_check.is_open()){
            ref_check.close();
            g_smooth.clear();
            std::ifstream rf(CSV_REF_FILE);
            std::string line; std::getline(rf, line);
            while(std::getline(rf, line)){
                if(line.empty()) continue;
                std::stringstream ss(line); PPPoint p; char c;
                if(ss>>p.x>>c>>p.y>>c>>p.yaw>>c>>p.curvature) g_smooth.push_back(p);
            }
            if(!g_smooth.empty()){
                pp_annotate();
                std::ofstream out(CSV_OUTPUT_FILE);
                out<<"x,y,yaw,curvature\n"<<std::fixed<<std::setprecision(6);
                for(auto& sp : g_smooth) out<<sp.x<<","<<sp.y<<","<<sp.yaw<<","<<sp.curvature<<"\n";
                return true;
            }
        }
    }

    // Priority 3: trajectory CSV (per-map recorded path if present, else raw_file).
    const std::string src = haveMapPath ? mapPath : raw_file;
    struct TrajRow{float x,y,z,yaw,rmse,ir;double ts;int turn;};
    std::vector<TrajRow> raw;
    std::ifstream f(src); if(!f.is_open()) return false;
    std::string line; std::getline(f,line);
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); TrajRow p; char c;
        ss>>p.x>>c>>p.y>>c>>p.z>>c>>p.yaw>>c>>p.rmse>>c>>p.ir>>c>>p.ts>>c>>p.turn;
        if(p.ir>=FILTER_MIN_QUALITY) raw.push_back(p);
    }
    if(raw.size()<3) return false;
    std::vector<float>xs(raw.size()),ys(raw.size());
    for(size_t i=0;i<raw.size();i++){xs[i]=raw[i].x;ys[i]=raw[i].y;}
    if(g_path_smoothing_enabled.load()){
        // Smoothed: downsample → Laplacian → Catmull-Rom resample.
        pp_downsample(xs, ys, DOWNSAMPLE_STEP_M);
        pp_laplacian(xs, ys, SMOOTH_PASSES, SMOOTH_ALPHA);
        g_smooth=pp_catmull(xs, ys, RESAMPLE_STEP_PP);
    } else {
        // Raw: light downsample only (drop near-duplicate points), no smoothing.
        pp_downsample(xs, ys, DOWNSAMPLE_STEP_M);
        g_smooth.clear();
        for(size_t i=0;i<xs.size();i++) g_smooth.push_back({xs[i], ys[i], 0.f, 0.f});
    }
    pp_annotate();
    std::ofstream out(CSV_OUTPUT_FILE);
    out<<"x,y,yaw,curvature\n"<<std::fixed<<std::setprecision(6);
    for(auto& sp : g_smooth) out<<sp.x<<","<<sp.y<<","<<sp.yaw<<","<<sp.curvature<<"\n";
    return !g_smooth.empty();
}

bool pp_load_smoothed() {
    g_smooth.clear();
    std::ifstream f(CSV_OUTPUT_FILE); if(!f.is_open()) return false;
    std::string line; std::getline(f,line);
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::stringstream ss(line); PPPoint p; char c;
        if(ss>>p.x>>c>>p.y>>c>>p.yaw>>c>>p.curvature) g_smooth.push_back(p);
    }
    pp_annotate();
    return !g_smooth.empty();
}

void pp_trigger_plan() {
    g_approach_path.clear(); g_corridor_path.clear(); g_goal_path.clear();
    g_plan_corner_mode = false;
    if(!g_pp_robot.valid||!g_pp_goal.valid) return;

    // Top-LiDAR corner maps ("zones"): the recorded corner is used ONLY when a zone
    // (zone 1 or zone 2) is armed. Armed → corner_mode forces the full corner. Not
    // armed → the corner is ignored and the cart plans a direct path. Which corner
    // curve is actually in g_smooth is decided by pp_active_map_recorded_path()
    // (zone-2-aware) — this flag only decides whether to route through it at all.
    bool corner_map = active_map_ptr &&
        (active_map_ptr->name == "Campus Top LiDAR" || active_map_ptr->name == "Campus Top (Rev)");

    // ── Cross-zone routing (order-preserving, follows the real recorded road) ──
    // When the robot sits on one zone's corridor and the goal on the OTHER zone's,
    // route along the FULL continuous recorded drive (which physically drove through
    // both corners and everything between) from the robot's nearest point to the
    // goal's nearest point. This passes every destination in between in real driven
    // order and follows the actual road across the un-recorded gap between the two
    // separately-cut corner corridors — a straight connector there would cut across
    // buildings.
    //
    // Which zone the robot / goal belong to is decided by which SHORT corner corridor
    // each is NEAREST to — NOT by point-in-polygon against the hand-drawn zone
    // boundaries, which don't reliably contain the exact corridor points (that
    // fragility caused earlier "skipped zone 1" / "dragged through zone 2" mis-routes).
    if(corner_map && g_zone_valid && g_zone2_valid){
        corr_cache_refresh();
        if(g_corr_c1_ok && g_corr_c2_ok){
            const std::vector<PPPoint>& c1 = g_corr_c1;   // zone-1 corner (cached)
            const std::vector<PPPoint>& c2 = g_corr_c2;   // zone-2 corner (cached)
            float rd1,rd2,gd1,gd2;
            corr_nearest(c1, g_pp_robot.x, g_pp_robot.y, rd1);
            corr_nearest(c2, g_pp_robot.x, g_pp_robot.y, rd2);
            corr_nearest(c1, g_pp_goal.x,  g_pp_goal.y,  gd1);
            corr_nearest(c2, g_pp_goal.x,  g_pp_goal.y,  gd2);
            int robot_zone = (rd1<=rd2)?0:1;
            int goal_zone  = (gd1<=gd2)?0:1;
            if(robot_zone!=goal_zone) corr_cache_ensure_full();   // lazy-load the full drive
            if(robot_zone!=goal_zone && g_corr_full_ok){
                const std::vector<PPPoint>& full = g_corr_full;   // full recorded drive (cached)
                float df; int fi_r = corr_nearest(full, g_pp_robot.x, g_pp_robot.y, df);
                int      fi_g = corr_nearest(full, g_pp_goal.x,  g_pp_goal.y,  df);
                g_plan_corner_mode = true;
                float gx=g_pp_goal.x, gy=g_pp_goal.y;

                // approach: robot → its nearest point on the full trajectory (straight
                // line — reverted from a Bezier tangent-departure curve suspected of
                // steering the forward-map danger-zone approach closer to a wall;
                // straight-line is the confirmed-safe baseline).
                if(pp_fdist(g_pp_robot.x,g_pp_robot.y,full[fi_r].x,full[fi_r].y) > 0.5f)
                    g_approach_path = pp_linear_interpolate(g_pp_robot, {full[fi_r].x,full[fi_r].y,0,0});
                // full trajectory: robot's point → goal's point (whole road, in order)
                if(fi_r<=fi_g) for(int i=fi_r;i<=fi_g;i++) g_corridor_path.push_back({full[i].x,full[i].y});
                else           for(int i=fi_r;i>=fi_g;i--) g_corridor_path.push_back({full[i].x,full[i].y});
                // goal connector: full-traj goal-point → actual destination point
                if(pp_fdist(gx,gy,full[fi_g].x,full[fi_g].y) > 0.5f)
                    g_goal_path = pp_bezier_goal_connector(full[fi_g], gx, gy);
                g_pp_goal.yaw = full[fi_g].yaw;
                return;
            }
        }
        // Same zone (or a file failed to load) → fall through to single-zone below.
    }
    // ── End cross-zone routing ────────────────────────────────────────────────

    bool corner_active = corner_map &&
        ((g_zone_valid && g_zone_armed) || (g_zone2_valid && g_zone2_armed));

    // Danger-zone test maps: always follow the recorded path (never free-mode). The
    // global segment_polygon.cfg is loaded once at startup in the OLD campus frame and
    // never cleared per-map, so on a different-frame test map the robot can land
    // outside it and wrongly trigger free-mode (direct Bézier), ignoring the recorded
    // path entirely. Force it off so the nearest-point corridor-following below runs.
    bool dz_test_map = active_map_ptr &&
        (active_map_ptr->name == "Danger Zone Fwd" || active_map_ptr->name == "Danger Zone Rev");

    // ── Map segmentation ────────────────────────────────────────────────────
    // Free mode = direct Bézier robot→goal:
    //   corner maps: when zone 1 is NOT armed (corner used only when armed)
    //   old maps:    when robot is outside the segment polygon (existing behavior)
    bool free_mode = dz_test_map ? false
                   : corner_map  ? !corner_active
                                 : (g_segment_polygon_valid &&
                                    !point_in_polygon(g_pp_robot.x, g_pp_robot.y, g_segment_polygon));
    g_plan_free_mode = free_mode;
    if(free_mode){
        float dist = pp_fdist(g_pp_robot.x, g_pp_robot.y, g_pp_goal.x, g_pp_goal.y);
        if(dist < 0.1f) return;
        float ctrl = dist * 0.4f;
        float p1x = g_pp_robot.x - ctrl * sinf(g_pp_robot.yaw);
        float p1y = g_pp_robot.y + ctrl * cosf(g_pp_robot.yaw);
        int steps = std::max(10, (int)(dist / RESAMPLE_STEP_PP));
        for(int i = 0; i <= steps; i++){
            float t = (float)i / steps, u = 1.f - t;
            g_corridor_path.push_back({
                u*u*g_pp_robot.x + 2.f*u*t*p1x + t*t*g_pp_goal.x,
                u*u*g_pp_robot.y + 2.f*u*t*p1y + t*t*g_pp_goal.y
            });
        }
        return;
    }
    // ── End map segmentation (June 22, 2026) ────────────────────────────────

    if(g_smooth.empty()) return;

    float click_x = g_pp_goal.x, click_y = g_pp_goal.y;

    // ── Recorded-corner maps: ALWAYS route through the full corner ──────────
    // On the top-LiDAR maps the path file is a single corner curve. The nearest-
    // point + forward-only logic below collapses the corridor depending on which
    // end the cart localizes nearest (skip-localize lands near the start → works;
    // real localize lands near the other end → corner skipped, plan bows around).
    // Fix: enter from whichever corner END is nearest the robot, traverse the
    // WHOLE corner toward the far end, with straight-line connectors in and out.
    if(corner_active){
        int last = (int)g_smooth.size() - 1;
        // Entry point: if the robot is already ON (or very near) the recorded corridor
        // — the normal case once localized mid-route, not just at the corner's
        // recording start — enter from its actual nearest point on the corridor, so
        // the plan doesn't skip past reference points between the robot and that
        // point. Only fall back to "nearest END" (the original fix, preserved) when
        // the robot is genuinely far from the whole corridor — approaching it from
        // outside — where forcing a mid-corridor entry would cut the corner short.
        float dr0 = pp_fdist(g_smooth[0].x,   g_smooth[0].y,    g_pp_robot.x, g_pp_robot.y);
        float drL = pp_fdist(g_smooth[last].x,g_smooth[last].y, g_pp_robot.x, g_pp_robot.y);
        int robot_end = (dr0 <= drL) ? 0 : last;
        float robot_near_d = 1e9f; int robot_near_idx = robot_end;
        for(int i=0;i<=last;i++){
            float d=pp_fdist(g_smooth[i].x,g_smooth[i].y,g_pp_robot.x,g_pp_robot.y);
            if(d<robot_near_d){robot_near_d=d;robot_near_idx=i;}
        }
        static const float ON_CORRIDOR_M = 8.f;   // within this of some corridor point = "on it"
        if(robot_near_d <= ON_CORRIDOR_M) robot_end = robot_near_idx;
        // Find the nearest corner POINT (any index) to the destination.
        // Using any index (not just an end) allows partial traversal: a mid-corner
        // destination routes only to that midpoint, not to the far end + backtrack.
        int goal_idx = robot_end;
        { float bd = 1e9f;
          for(int i=0;i<=last;i++){
              float d=pp_fdist(g_smooth[i].x,g_smooth[i].y,click_x,click_y);
              if(d<bd){bd=d;goal_idx=i;}
          }
        }
        // Always traverse via the recorded corridor from robot_end to goal_idx, even
        // when they're the same point (degenerate 1-point "corridor" + short approach/
        // goal-connector hops). Previously, goal_idx==robot_end took a direct
        // straight/Bezier shortcut between the robot's and goal's raw coordinates —
        // since that line isn't checked against the map at all, it could cut straight
        // through a building or wall that the recorded (physically-driven, so
        // guaranteed-clear) corridor routes around. Routing through the corridor here
        // also guarantees any other destination whose nearest corridor point lies
        // between robot_end and goal_idx is passed along the way, never shortcut past.
        g_plan_corner_mode = true;
        int s = robot_end, e = goal_idx;
        // Straight-line approach — reverted from a Bezier tangent-departure curve that
        // was suspected of steering the forward-map danger-zone approach closer to a
        // wall. Straight-line is the confirmed-safe baseline; a Bezier bulge here can
        // overshoot toward one side when localizing off-track, which straight-line
        // doesn't.
        if(pp_fdist(g_pp_robot.x,g_pp_robot.y,g_smooth[s].x,g_smooth[s].y) > 0.5f)
            g_approach_path = pp_linear_interpolate(g_pp_robot, g_smooth[s]);
        if(s <= e) for(int i=s;i<=e;i++) g_corridor_path.push_back({g_smooth[i].x, g_smooth[i].y});
        else       for(int i=s;i>=e;i--) g_corridor_path.push_back({g_smooth[i].x, g_smooth[i].y});
        if(pp_fdist(click_x,click_y,g_smooth[e].x,g_smooth[e].y) > 0.5f)
            g_goal_path = pp_bezier_goal_connector(g_smooth[e], click_x, click_y);
        g_pp_goal.yaw = g_smooth[e].yaw;
        return;
    }
    // ── End recorded-corner handling ───────────────────────────────────────

    int entry = pp_nearest(g_pp_robot.x, g_pp_robot.y);

    // Forward-only: find nearest path point to the click among indices >= entry.
    // Prevents backward path planning if user clicks behind the cart.
    int g_idx = -1;
    {
        float bd = 1e9f;
        for(int i = entry; i < (int)g_smooth.size(); i++){
            float d = pp_fdist(g_smooth[i].x, g_smooth[i].y, click_x, click_y);
            if(d < bd){ bd = d; g_idx = i; }
        }
    }
    if(g_idx < 0){ g_pp_goal.valid = false; return; }

    // Bézier approach: robot → path entry, smooth tangent departure.
    if(pp_fdist(g_pp_robot.x,g_pp_robot.y,g_smooth[entry].x,g_smooth[entry].y)>0.5f)
        g_approach_path = pp_bezier_approach(g_pp_robot, g_smooth[entry]);

    // Corridor: follow reference path entry → g_idx, no skipping.
    g_corridor_idx_start = entry;
    g_corridor_idx_end   = g_idx;
    for(int i = entry; i <= g_idx; i++)
        g_corridor_path.push_back({g_smooth[i].x, g_smooth[i].y});

    // Goal connector: if click is >0.5 m off path, add Bézier from path exit to actual goal.
    // If click is on/near path, snap goal to path point (no connector needed).
    float off_path = pp_fdist(click_x, click_y, g_smooth[g_idx].x, g_smooth[g_idx].y);
    if(off_path > 0.5f){
        g_goal_path = pp_bezier_goal_connector(g_smooth[g_idx], click_x, click_y);
        g_pp_goal.x   = click_x;
        g_pp_goal.y   = click_y;
        g_pp_goal.yaw = g_smooth[g_idx].yaw;
    } else {
        g_pp_goal.x   = g_smooth[g_idx].x;
        g_pp_goal.y   = g_smooth[g_idx].y;
        g_pp_goal.yaw = g_smooth[g_idx].yaw;
    }
}

// Shift the FINAL per-plan path (not the shared reference corridor g_smooth) around any
// STATIC obstacle-grid cells within OBST_AVOID_CLEARANCE_M — a detour for one specific
// robot->goal plan shouldn't distort the corridor other plans reuse. PEDESTRIAN cells
// are deliberately ignored here (see grid_click_cycle) — that's a stop trigger, not a
// routing input. Reuses the same cosine-ramp tapering technique as pp_shift_path_segment
// (blend smoothly back to the base path, no kink) but computes its own direction (away
// from the obstacle), magnitude (just enough clearance), and a KINEMATICALLY-SIZED ramp
// length per detour, instead of taking a caller-specified delta and a fixed ramp.
static void pp_apply_obstacle_avoidance(std::vector<PPPoint>& path) {
    int n = (int)path.size();
    if(n < 3 || g_obstacle_grid.empty()) return;

    std::vector<std::pair<float,float>> statics;
    for(auto& kv : g_obstacle_grid){
        if(kv.second != OBST_STATIC) continue;
        float cx,cy; grid_cell_center(kv.first.first, kv.first.second, cx, cy);
        statics.push_back({cx,cy});
    }
    if(statics.empty()) return;

    // Cumulative arc-length — turns every "N metres from index i" / "gap between i
    // and j" question below into O(1) arithmetic instead of a per-use walking loop.
    std::vector<float> arc(n, 0.f);
    for(int i=1;i<n;i++) arc[i] = arc[i-1] + pp_fdist(path[i-1].x,path[i-1].y,path[i].x,path[i].y);

    // Per-point local normal, from the ORIGINAL (pre-shift) path — needed because a
    // single straight-line "sideways" direction for an entire multi-metre detour cuts
    // across a curving corridor instead of following it (confirmed visually 2026-07-19
    // on a corner-mode map: the detour visibly sliced across the curve rather than
    // swinging with it). Each point shifts along its OWN local perpendicular instead.
    // Computed once here, before any shift is applied, so later points in the same
    // (or a later) stretch never read an already-shifted neighbour when estimating
    // direction. Safe against the zigzag risk that rules this out for the manual
    // "Edit Path" nudge tool (pp_shift_path_segment) — that operates on a possibly
    // still-raw g_smooth; this operates on the already Laplacian+Catmull-Rom-smoothed
    // (or corner-mode) per-plan path, which has no high-frequency noise to amplify.
    std::vector<float> nrmX(n,0.f), nrmY(n,0.f);
    for(int i=0;i<n;i++){
        int a=std::max(0,i-2), b=std::min(n-1,i+2);
        float tx=path[b].x-path[a].x, ty=path[b].y-path[a].y;
        float tl=sqrtf(tx*tx+ty*ty);
        if(tl>1e-4f){ nrmX[i]=-ty/tl; nrmY[i]=tx/tl; }
    }

    std::vector<float> minDist(n, 1e9f);
    std::vector<int> nearestObst(n, -1);
    for(int i=0;i<n;i++){
        for(size_t k=0;k<statics.size();k++){
            float d = pp_fdist(path[i].x, path[i].y, statics[k].first, statics[k].second);
            if(d < minDist[i]){ minDist[i]=d; nearestObst[i]=(int)k; }
        }
    }

    // Contiguous blocked stretches, MERGED with the previous one when the gap between
    // them is under OBST_MERGE_GAP_M — two close-together obstacles must produce ONE
    // combined swing, not two independent detours that can pick opposite left/right
    // directions and fight each other into a wavy path (confirmed visually 2026-07-19).
    struct Stretch { int lo, hi; };
    std::vector<Stretch> stretches;
    {
        int i = 0;
        while(i < n){
            if(minDist[i] >= OBST_AVOID_CLEARANCE_M){ i++; continue; }
            int lo = i, hi = i;
            while(hi+1 < n && minDist[hi+1] < OBST_AVOID_CLEARANCE_M) hi++;
            if(!stretches.empty() && arc[lo] - arc[stretches.back().hi] < OBST_MERGE_GAP_M)
                stretches.back().hi = hi;
            else
                stretches.push_back({lo, hi});
            i = hi + 1;
        }
    }
    if(stretches.empty()) return;

    // Cart kinematics: absolute minimum physical turning radius from wheelbase + max
    // steer angle, with a 2x comfort/safety margin — the steering RATE limit (5deg/s)
    // means the cart can't instantly reach any target curvature either, so staying
    // well inside the physical minimum avoids both an infeasible ask and a harsh-
    // feeling one. (2026-07-19: a fixed 3m ramp was tried first and found to demand
    // a ~1.1m turning radius for a typical shift — ~5x tighter than achievable.)
    float rMinPhysical = WHEELBASE_M / tanf(toRad(MAX_STEER_DEG));
    float rTarget = 2.0f * rMinPhysical;

    float capMag = CORRIDOR_HALF_M - 0.3f;
    bool anyBlocked = false;
    for(auto& st : stretches){
        int lo = st.lo, hi = st.hi;

        // Preferred side: which side of the chord (path[lo]->path[hi]) the blocking
        // obstacle(s) sit on, on average — try shifting AWAY from them first.
        float ox=0, oy=0; int cnt=0;
        for(int j=lo;j<=hi;j++){
            if(nearestObst[j]<0) continue;
            auto& s = statics[nearestObst[j]];
            ox+=s.first; oy+=s.second; cnt++;
        }
        if(cnt==0) continue;
        ox/=cnt; oy/=cnt;
        float cx=path[hi].x-path[lo].x, cy=path[hi].y-path[lo].y;
        float cl=sqrtf(cx*cx+cy*cy);
        if(cl<=1e-3f) continue;                // degenerate chord — skip this stretch
        float cnx=-cy/cl, cny=cx/cl;
        float side = (ox-path[lo].x)*cnx + (oy-path[lo].y)*cny;
        float preferredSign = (side >= 0.f) ? -1.f : 1.f;

        // Find the SMALLEST magnitude (up to the corridor cap) in either direction
        // that clears EVERY static obstacle for EVERY point in the blocked stretch.
        // Sizing the magnitude from just the single nearest obstacle isn't enough
        // for a CLUSTER of several cells: clearing the nearest one doesn't guarantee
        // clearing a second cell elsewhere in the cluster (confirmed 2026-07-19: a
        // 3-cell cluster in an otherwise wide-open corridor was wrongly reported
        // unavoidable because the old single-nearest-cell estimate under-sized the
        // shift and failed verification on both sides). Binary search is correct
        // regardless of how the cluster is shaped, and only 12 clearance passes.
        auto clearsAt = [&](float sign, float testMag)->bool{
            for(int j=lo;j<=hi;j++){
                float sx = path[j].x + nrmX[j]*testMag*sign;
                float sy = path[j].y + nrmY[j]*testMag*sign;
                for(auto& s : statics)
                    if(pp_fdist(sx,sy,s.first,s.second) < OBST_AVOID_CLEARANCE_M) return false;
            }
            return true;
        };
        auto findMag = [&](float sign)->float{
            if(!clearsAt(sign, capMag)) return -1.f;   // doesn't clear even at the limit
            float loM=0.f, hiM=capMag;
            for(int iter=0; iter<12; iter++){
                float mid=0.5f*(loM+hiM);
                if(clearsAt(sign, mid)) hiM=mid; else loM=mid;
            }
            return std::min(hiM + 0.1f, capMag);       // small margin, still capped
        };

        float mag, shiftSign;
        float mPref = findMag(preferredSign);
        if(mPref >= 0.f){ shiftSign = preferredSign; mag = mPref; }
        else {
            float mOther = findMag(-preferredSign);
            if(mOther >= 0.f){ shiftSign = -preferredSign; mag = mOther; }
            else {
                // No valid detour within the safe corridor on EITHER side even at
                // the full cap — do not apply a partial/insufficient shift. Leave
                // this stretch untouched and surface the failure clearly instead of
                // silently producing a path that still cuts through the blockage.
                anyBlocked = true;
                continue;
            }
        }
        float delta = mag * shiftSign;

        // Ramp length that keeps this swing's peak curvature within 1/rTarget, for a
        // raised-cosine lateral offset profile y(s)=mag*0.5*(1-cos(pi*s/L)), whose
        // peak second derivative (curvature, for a locally near-straight base path)
        // is 0.5*mag*(pi/L)^2 — solved here for L given a target radius R=1/curvature.
        float rampLen = sqrtf(rTarget * mag * kPI*kPI * 0.5f);
        int elo = lo; while(elo > 0 && arc[lo]-arc[elo] < rampLen) elo--;
        int ehi = hi; while(ehi < n-1 && arc[ehi]-arc[hi] < rampLen) ehi++;

        // Full magnitude (w=1) held across lo..hi so clearance is maintained across
        // the whole (possibly merged, multi-obstacle) blocked stretch, not just its
        // center; ramps taper over the kinematically-sized elo->lo / hi->ehi extents.
        // Each point shifts along its OWN local normal (nrmX/nrmY, precomputed above
        // from the unshifted path) rather than the single chord direction — follows
        // a curving corridor instead of cutting across it.
        int rampLo = std::max(1, lo - elo), rampHi = std::max(1, ehi - hi);
        for(int j=elo;j<=ehi;j++){
            float w=1.f;
            if(j<lo)      w=0.5f*(1.f-cosf(kPI*(float)(j-elo)/(float)rampLo));
            else if(j>hi) w=0.5f*(1.f-cosf(kPI*(float)(ehi-j)/(float)rampHi));
            path[j].x += nrmX[j]*delta*w;
            path[j].y += nrmY[j]*delta*w;
        }
    }
    if(anyBlocked){
        g_statusMsg = "No path available - a static obstacle blocks the corridor with no valid route around it";
        g_statusCol = C_BAD;
        g_obst_blocked_msg_timer = 3.5f;   // drives the centred fade popup in DrawMapViewport
        std::cout << "[PLAN] No path available - obstacle blocks corridor, no valid detour on either side\n";
    }
}

void pp_save_plan() {
    // June 22, 2026: free mode bypasses g_smooth check
    if(!g_plan_free_mode && g_smooth.size()<2) return;
    if(!g_pp_robot.valid||!g_pp_goal.valid) return;
    if(g_approach_path.empty()&&g_corridor_path.empty()) return;

    std::vector<PPPoint> saved;

    if(!g_approach_path.empty()){
        float total = 0;
        for(size_t i=1;i<g_approach_path.size();i++)
            total += pp_fdist(g_approach_path[i-1].x,g_approach_path[i-1].y,
                              g_approach_path[i].x,  g_approach_path[i].y);
        int steps = std::max(2,(int)(total/RESAMPLE_STEP_PP));
        for(int s=0;s<=steps;s++){
            float t=(float)s/steps;
            float target = t * total;
            float walked = 0;
            Vector2 pt = g_approach_path[0];
            for(size_t i=1;i<g_approach_path.size();i++){
                float seg = pp_fdist(g_approach_path[i-1].x,g_approach_path[i-1].y,
                                     g_approach_path[i].x,  g_approach_path[i].y);
                if(walked+seg >= target || i==g_approach_path.size()-1){
                    float f = (seg>1e-5f) ? (target-walked)/seg : 0.f;
                    pt = {g_approach_path[i-1].x + f*(g_approach_path[i].x-g_approach_path[i-1].x),
                          g_approach_path[i-1].y + f*(g_approach_path[i].y-g_approach_path[i-1].y)};
                    break;
                }
                walked += seg;
            }
            saved.push_back({pt.x, pt.y, 0.f, 0.f});
        }
    }

    if(!g_corridor_path.empty()){
        if(g_plan_free_mode || g_plan_corner_mode){
            // free mode: Bézier pts already in g_corridor_path.
            // corner mode: full corner already in g_corridor_path in traversal order
            // (may be reversed vs g_smooth index order — must save as-is, not by index).
            for(auto& pt : g_corridor_path)
                saved.push_back({pt.x, pt.y, 0.f, 0.f});
        } else {
            // Use the indices stored by pp_trigger_plan() — guaranteed forward order
            // (g_corridor_idx_start <= g_corridor_idx_end).  Re-scanning g_smooth here
            // was Bug 7: it could return idx_end < idx_start on near-loop paths and walk
            // the corridor backward, producing reversed steering in Pure Pursuit.
            for(int i = g_corridor_idx_start; i <= g_corridor_idx_end; i++)
                saved.push_back(g_smooth[i]);
        }
    }

    if(!g_goal_path.empty()){
        for(const auto& pt : g_goal_path)
            saved.push_back({pt.x, pt.y, 0.f, 0.f});
    }

    if(saved.size()<2){
        g_statusMsg="Plan save failed: degenerate path (< 2 points) — re-click goal";
        g_statusCol=C_BAD;
        return;
    }

    std::vector<PPPoint> deduped;
    deduped.push_back(saved[0]);
    for(size_t i=1;i<saved.size();i++){
        if(pp_fdist(saved[i].x,saved[i].y,deduped.back().x,deduped.back().y)>0.02f)
            deduped.push_back(saved[i]);
    }
    saved = deduped;

    // Snapshot the pre-avoidance path (2026-07-28) — x,y only meaningful here (yaw/
    // curvature are filled in below, after avoidance); written unconditionally so
    // "before" is always on disk next to smoothed_path.csv, whether or not this
    // particular plan actually had an obstacle to detour around.
    {std::ofstream pre(CSV_PRE_AVOID_FILE);
     pre<<"x,y,yaw,curvature\n"<<std::fixed<<std::setprecision(6);
     for(auto& p : saved) pre<<p.x<<","<<p.y<<","<<p.yaw<<","<<p.curvature<<"\n";}

    // Step-2 static-obstacle avoidance (2026-07-19) — before yaw/curvature below, so
    // that existing recompute naturally produces correct values for the detoured points.
    pp_apply_obstacle_avoidance(saved);

    int M = (int)saved.size();
    for(int i=0;i<M;i++){
        int a=std::max(0,i-2), b=std::min(M-1,i+2);
        saved[i].yaw = atan2f(saved[b].x-saved[a].x, saved[b].y-saved[a].y);
    }
    saved[0].curvature=0; saved.back().curvature=0;
    for(int i=1;i<M-1;i++)
        saved[i].curvature=pp_fwrap(saved[i+1].yaw-saved[i].yaw)/
                           (pp_fdist(saved[i].x,saved[i].y,saved[i+1].x,saved[i+1].y)+1e-6f);
    if(M>=3){
        std::vector<float> sc(M,0.f);
        for(int i=0;i<M;i++){
            int lo=std::max(0,i-2),hi=std::min(M,i+3);
            float s=0;for(int j=lo;j<hi;j++)s+=saved[j].curvature;
            sc[i]=s/(hi-lo);
        }
        for(int i=0;i<M;i++) saved[i].curvature=sc[i];
    }

    {std::ofstream pf(CSV_OUTPUT_FILE);
     pf<<"x,y,yaw,curvature\n"<<std::fixed<<std::setprecision(6);
     for(auto& p : saved) pf<<p.x<<","<<p.y<<","<<p.yaw<<","<<p.curvature<<"\n";}

    float totalDist=0;
    for(size_t i=1;i<saved.size();i++)
        totalDist+=pp_fdist(saved[i-1].x,saved[i-1].y,saved[i].x,saved[i].y);
    {std::ofstream rf("plan_result.txt");rf<<std::fixed<<std::setprecision(6);
     rf<<"START_X "<<g_pp_robot.x<<"\nSTART_Y "<<g_pp_robot.y<<"\nSTART_YAW "<<g_pp_robot.yaw<<"\n";
     rf<<"GOAL_X "<<g_pp_goal.x<<"\nGOAL_Y "<<g_pp_goal.y<<"\n";
     rf<<"TOTAL_DIST "<<totalDist<<"\nPATH_POINTS "<<saved.size()<<"\n";}
    std::cout<<"[PLAN] Saved "<<saved.size()<<" pts, dist="<<totalDist<<"m\n";
}
