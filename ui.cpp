// =============================================================================
// ui.cpp  ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â  Raylib visualization, debug panels, maps, trajectories, overlays
// =============================================================================
#include "common.hpp"
#include <shellapi.h>

// forward declarations from planning.cpp
void pp_load_smoothed();
void pp_trigger_plan();
bool pp_run_pipeline(const std::string& raw_file);
void pp_save_plan();
int  pp_record_path_snapshot(const std::string& fn);
std::string pp_active_map_recorded_path();
bool pp_reset_zone_path(int zone);
const std::vector<PPPoint>& pp_danger_zone_pts();
int  pp_zone_for_point(float x, float y, float max_dist_m);   // 0=zone1, 1=zone2, -1=neither
bool pp_full_drive_slice(float ax, float ay, float bx, float by, std::vector<PPPoint>& out);

// forward declarations from main.cpp (June 22, 2026)
void save_segment_polygon(const char* path);
void save_destinations_for_map();
void save_destinations2_for_map();

// forward declarations from control.cpp
void ctrl_doLoad();
void ctrl_doStart();
void ctrl_doStop();
void ctrl_doReset();
void ped_set_enabled(bool on);   // main.cpp — launch/stop the Python pedestrian detector
void ctrl_load_fixed_test_path();
void ctrl_load_straight_path(float length_m);
bool ctrl_danger_path_range(int& lo, int& hi);
void ctrl_resplice_danger_zone_now();
extern std::atomic<bool> g_ctrl_thread_run;

// forward declarations from localization.cpp
void particle_filter_reloc_thread(std::vector<Point3D>, const DistanceMap*, bool, float, bool, float, float, float);
bool load_ply(const std::string&, std::vector<MapPoint>&);
DistanceMap build_distance_map(const std::vector<MapPoint>&, bool);
extern bool save_trajectory_csv(const std::string&);

// forward declarations from steer_test.cpp
void steer_test_run_A();
void steer_test_run_A_repeat(int num_runs);
void steer_test_run_B();
void steer_test_run_sweep();

// Shared per-destination color palette (index = dest id, 0..NUM_DEST-1).
// Used by both the Step 2 tap-zone editor and the passenger destination panel
// so the same place always reads as the same color everywhere.
static const Color kDestColors[NUM_DEST] = {
    {0,200,240,220}, {230,185,30,220}, {60,210,90,220}, {200,80,220,220}, {235,90,70,220}
};
// Zone 2's destination palette — distinct hues from kDestColors since both can
// appear together in the combined passenger picker strip. Last two entries
// (Basketball Court, Food Court — 2026-07-07) are the danger-zone-crossing
// destinations, not the original 4 zone-2 corner destinations.
static const Color kDestColors2[NUM_DEST2] = {
    {255,140,0,220}, {120,80,200,220}, {40,180,180,220}, {150,210,40,220},
    {235,60,140,220}, {255,215,0,220}
};

// Combined index 0..NUM_DEST-1 = zone 1, NUM_DEST..NUM_DEST_TOTAL-1 = zone 2 — name/
// color lookups shared by the Step 2 tap-zone editor and the passenger destination panel.
static const char* combinedDestName(int ci){ return ci<NUM_DEST ? g_dest_names[ci] : g_dest_names2[ci-NUM_DEST]; }
static Color combinedDestColor(int ci){ return ci<NUM_DEST ? kDestColors[ci] : kDestColors2[ci-NUM_DEST]; }

// =============================================================================
//  INTERNAL HELPERS
// =============================================================================
static bool DrawBtn(Rectangle r, const char* label, Color col, bool enabled=true) {
    if(!enabled){
        DrawRectangleRounded(r,0.25f,6,{30,36,50,180});
        DrawRectangleRoundedLines(r,0.25f,6,1,C_BORDER);
        int tw=MeasureText(label,14);
        DrawText(label,(int)(r.x+r.width/2-tw/2),(int)(r.y+r.height/2-7),14,C_DIM);
        return false;
    }
    Vector2 m=GetMousePosition(); bool hov=CheckCollisionPointRec(m,r);
    float br=hov?(IsMouseButtonDown(MOUSE_LEFT_BUTTON)?-0.15f:0.18f):0;
    DrawRectangleRounded(r,0.25f,6,ColorBrightness(col,br));
    DrawRectangleRoundedLines(r,0.25f,6,1.5f,hov?WHITE:ColorBrightness(col,-0.2f));
    int tw=MeasureText(label,14);
    DrawText(label,(int)(r.x+r.width/2-tw/2),(int)(r.y+r.height/2-7),14,WHITE);
    return hov&&IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
}

struct View2D { float cx, cy, scale; int sw, sh; };
static Vector2 w2s(float wx, float wy, const View2D& v) {
    return {v.sw*0.5f+(wx-v.cx)*v.scale, v.sh*0.5f-(wy-v.cy)*v.scale};
}

static void DrawRangeRings(Vector3 center, bool large) {
    float r1=large?10:5, r2=large?25:10, r3=large?50:15, cr=large?55:20;
    DrawCircle3D(center,r1,{1,0,0},90,Fade(DARKGRAY,0.5f));
    DrawCircle3D(center,r2,{1,0,0},90,Fade(DARKGRAY,0.5f));
    DrawCircle3D(center,r3,{1,0,0},90,Fade(DARKGRAY,0.4f));
    DrawLine3D({center.x-cr,center.y,center.z},{center.x+cr,center.y,center.z},Fade(DARKGRAY,0.5f));
    DrawLine3D({center.x,center.y,center.z-cr},{center.x,center.y,center.z+cr},Fade(DARKGRAY,0.5f));
}

// =============================================================================
//  RIGHT PANEL ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â Step 1: Localize
// =============================================================================
void DrawPanelLocalize(int px, int py, int pw, int ph,
                       AppState& cur_state, float calib_timer,
                       int lidar_hz, int imu_hz)
{
    DrawRectangle(px,py,pw,ph,C_PANEL_DARK);
    DrawRectangle(px,py,1,ph,C_BORDER_HI);
    int x=px+16, y=py+16;
    DrawText("01  LOCALIZE",x,y,18,C_HEADER); y+=28;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=12;

    Color lc=lidar_hz>5?C_GOOD:C_BAD, ic=imu_hz>100?C_GOOD:C_BAD;
    DrawText(TextFormat("LiDAR: %d Hz",lidar_hz),x,y,13,lc);
    DrawText(TextFormat("IMU: %d Hz",imu_hz),x+120,y,13,ic); y+=22;

    const char* sLabel=""; Color sCol=C_DIM;
    switch(cur_state){
        case STATE_CALIBRATING:     sLabel="CALIBRATING IMU...";              sCol=C_WARN; break;
        case STATE_ALIGNMENT_CHECK: sLabel="WAITING FOR ALIGNMENT -- SHIFT+CLICK to seed";   sCol=C_WARN; break;
        case STATE_GLOBAL_RELOC:    sLabel="LOCALISING...";                   sCol={255,165,0,255}; break;
        case STATE_FUSION_RUNNING:
            if(current_inlier_ratio.load()<(active_map_ptr?active_map_ptr->track_accept:0.35f))
                 {sLabel="LOST / WRONG MAP  [R] Reset"; sCol=C_BAD;}
            else if(g_is_turning.load())
                 {sLabel="TRACKING  [TURNING]"; sCol=C_WARN;}
            else {sLabel="TRACKING"; sCol=C_GOOD;}
            break;
    }
    DrawRectangleRounded({(float)x,(float)y,(float)(pw-32),26},0.3f,6,Fade(sCol,0.15f));
    DrawRectangleRoundedLines({(float)x,(float)y,(float)(pw-32),26},0.3f,6,1,sCol);
    int sw2=MeasureText(sLabel,13); DrawText(sLabel,x+(pw-32)/2-sw2/2,y+6,13,sCol); y+=34;

    if(cur_state==STATE_GLOBAL_RELOC){
        float prog=0; {std::lock_guard<std::mutex>lk(g_reloc_mutex);prog=g_reloc_progress;}
        DrawRectangle(x,y,pw-32,12,{30,30,30,200});
        DrawRectangle(x,y,(int)((pw-32)*prog),12,C_GOOD);
        DrawText(TextFormat("%.0f%%",prog*100),x+(pw-32)/2-12,y-1,11,WHITE); y+=20;
    }
    if(cur_state==STATE_CALIBRATING){
        DrawRectangle(x,y,(int)((pw-32)*clampf(calib_timer/3.f,0,1)),8,C_WARN); y+=16;
    }

    if(!g_reloc_error_msg.empty()){ DrawText(g_reloc_error_msg.c_str(),x,y,11,C_BAD); y+=24; }

    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    double ex=0,ey=0,eyaw=0;
    {std::lock_guard<std::mutex>lk(ekf_mutex);ex=ekf.x[0];ey=ekf.x[1];eyaw=ekf.x[2];}
    float rmse=current_rmse.load(), ir=current_inlier_ratio.load();
    DrawText(TextFormat("Pos:  %.3f, %.3f m",(float)ex,(float)ey),x,y,13,C_WHITE); y+=17;
    DrawText(TextFormat("Yaw:  %.1f deg",(float)(eyaw*180/kPI)),x,y,13,C_WHITE); y+=17;
    DrawText(TextFormat("RMSE: %.3f m  Overlap: %d%%",rmse,(int)(ir*100)),x,y,13,
             rmse>RMSE_REJECT?C_BAD:(rmse>RMSE_FULL_TRUST?C_WARN:C_GOOD)); y+=17;
    DrawText(TextFormat("Map: %s",active_map_ptr?active_map_ptr->name.c_str():"none"),x,y,12,C_HEADER); y+=14;

    if(cur_state==STATE_FUSION_RUNNING){
        float imu_yaw_raw; {std::lock_guard<std::mutex>lk(imu_mutex);imu_yaw_raw=imu_angle.z;}
        float fusion_yaw_deg=(float)(eyaw*180.0/kPI);
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=6;
        DrawText("YAW SOURCES",x,y,11,C_DIM); y+=13;
        float imu_disp_deg;
        bool off_locked=(g_yaw_imu_lidar_offset_rad!=0.f);
        if(off_locked){
            float r=wrapA(imu_yaw_raw*(kPI/180.f)-g_yaw_imu_lidar_offset_rad);
            imu_disp_deg=r*(180.f/kPI);
        } else { imu_disp_deg=imu_yaw_raw; }
        float gap=imu_disp_deg-fusion_yaw_deg;
        while(gap>180)gap-=360; while(gap<-180)gap+=360;
        Color gC=fabsf(gap)<1.5f?C_GOOD:fabsf(gap)<4.f?C_WARN:C_BAD;
        DrawText(TextFormat("  IMU   : %+7.1f deg",imu_disp_deg),x,y,12,C_HEADER); y+=15;
        DrawText(TextFormat("  LiDAR : %+7.1f deg",fusion_yaw_deg),x,y,12,{100,220,255,255}); y+=15;
        DrawText(TextFormat("  Fusion: %+7.1f deg",fusion_yaw_deg),x,y,12,C_GOOD); y+=15;
        DrawText(TextFormat("  Gap   : %+7.1f deg%s",gap,off_locked?" [locked]":" [lock@Confirm]"),x,y,12,gC); y+=15;
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=6;
    } else { DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10; }

    DrawText("[SHIFT+CLICK]  Seed from map",x,y,12,C_DIM); y+=15;
    DrawText("[ENTER]        Global search",x,y,12,C_DIM); y+=15;
    DrawText("[M]  Cycle maps   [R]  Reset",x,y,12,C_DIM); y+=15;
    DrawText("[F]  Camera follow   [G]  Range rings",x,y,12,C_DIM); y+=15;
    DrawText("[L-drag] Orbit  [R-drag] Pan  [Home] Top-down",x,y,12,C_DIM); y+=20;

    // Jumps into the passenger Localize panel's calibration mode for whichever map is
    // currently active — the only place that tool lives, but that panel is otherwise
    // only reachable via the passenger flow, which never offers the danger-zone test
    // maps. g_localize_cal_from_technical marks the entry so the app returns straight
    // back here (not the passenger Intro screen) once calibration is done.
    if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},
               TextFormat("Calibrate This Map (%s)", active_map_ptr?active_map_ptr->name.c_str():"none"),
               {90,60,140,255}, active_map_ptr!=nullptr)){
        g_show_localize=true; g_localize_cal_mode=true; g_localize_cal_from_technical=true;
    }
    y+=30;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;

    // ── Record path snapshot (forward / reverse map) ────────────────────────
    // Target file follows whichever zone is currently armed (same zone-aware choice
    // pp_active_map_recorded_path() makes) — the zone was already selected by
    // drawing/arming its polygon above, so recording just targets that same zone.
    {
        static std::string rec_status; static Color rec_col = C_DIM;
        const char* zoneTag = g_zone2_armed ? " (Zone 2)" : "";
        DrawText("Record route (drive it first, then snapshot):", x, y, 11, C_HEADER); y+=15;
        int bw = (pw - 40) / 2;
        if(DrawBtn({(float)x,(float)y,(float)bw,26}, TextFormat("Record Forward Path%s", zoneTag), {60,90,140,255}, true)){
            int n = pp_record_path_snapshot(g_zone2_armed ? PATH_FORWARD2_FILE : PATH_FORWARD_FILE);
            if(n>0){ rec_status = TextFormat("Saved forward path%s: %d pts", zoneTag, n); rec_col = C_GOOD; }
            else   { rec_status = "No trajectory yet — localize & drive first"; rec_col = C_BAD; }
        }
        if(DrawBtn({(float)(x+bw+8),(float)y,(float)bw,26}, TextFormat("Record Reverse Path%s", zoneTag), {60,110,120,255}, true)){
            int n = pp_record_path_snapshot(g_zone2_armed ? PATH_REVERSE2_FILE : PATH_REVERSE_FILE);
            if(n>0){ rec_status = TextFormat("Saved reverse path%s: %d pts", zoneTag, n); rec_col = C_GOOD; }
            else   { rec_status = "No trajectory yet — localize & drive first"; rec_col = C_BAD; }
        }
        y+=30;
        if(!rec_status.empty()){ DrawText(rec_status.c_str(), x, y, 11, rec_col); y+=15; }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;
    }

    // ── Zone 1 localization focus ──
    {
        if(g_zone_valid){
            Color zc = g_zone_armed ? C_GOOD : C_DIM;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},
                       g_zone_armed ? "Zone 1 ARMED — Enter seeds in zone" : "Arm Zone 1 (focus localization)",
                       g_zone_armed ? Color{40,110,70,255} : Color{55,70,95,255}, true)){
                g_zone_armed = !g_zone_armed;
                if(g_zone_armed) g_zone2_armed = false;   // zones are mutually exclusive
                if(g_zone_armed){
                    g_zone_focus_request = true;  // center the view on the zone
                    // Before localizing, place the planning start at the estimated
                    // start (if set) — else the zone centroid — so previews begin from
                    // zone 1 with a sensible heading, not the map origin.
                    if(!g_confirmed_start.valid){
                        if(g_est_start.valid) g_pp_robot = g_est_start;
                        else {
                            float cx,cy,r;
                            if(zone_centroid_radius(g_zone_poly,cx,cy,r)) g_pp_robot = {cx, cy, 0.f, true};
                        }
                    }
                }
                // Which zone is armed changes which corner file pp_active_map_recorded_path()
                // returns — reload g_smooth and replan so the map isn't left routing through
                // a stale corner from before the toggle.
                pp_run_pipeline(TRAJ_CSV_FILE);
                if(g_pp_goal.valid) pp_trigger_plan();
            }
            y+=30;
            if(g_zone_armed){ DrawText("Waiting to localize in Zone 1 — press Enter / Shift+Click", x, y, 10, C_WARN); y+=14; }
        } else {
            DrawText("Zone 1: not set  (draw it in Step 2)", x, y, 11, C_DIM); y+=15;
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;
    }

    // ── Zone 2 localization focus ──
    {
        if(g_zone2_valid){
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},
                       g_zone2_armed ? "Zone 2 ARMED — Enter seeds in zone" : "Arm Zone 2 (focus localization)",
                       g_zone2_armed ? Color{40,110,70,255} : Color{95,65,40,255}, true)){
                g_zone2_armed = !g_zone2_armed;
                if(g_zone2_armed) g_zone_armed = false;   // zones are mutually exclusive
                if(g_zone2_armed){
                    g_zone2_focus_request = true;
                    if(!g_confirmed_start.valid){
                        if(g_est_start.valid) g_pp_robot = g_est_start;
                        else {
                            float cx,cy,r;
                            if(zone_centroid_radius(g_zone2_poly,cx,cy,r)) g_pp_robot = {cx, cy, 0.f, true};
                        }
                    }
                }
                pp_run_pipeline(TRAJ_CSV_FILE);
                if(g_pp_goal.valid) pp_trigger_plan();
            }
            y+=30;
            if(g_zone2_armed){ DrawText("Waiting to localize in Zone 2 — press Enter / Shift+Click", x, y, 10, C_WARN); y+=14; }
        } else {
            DrawText("Zone 2: not set  (draw it in Step 2)", x, y, 11, C_DIM); y+=15;
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;
    }

    // ── Reset reference path (recovery from a corrupted/truncated corner file) ──
    {
        bool isTopLidar = active_map_ptr &&
            (active_map_ptr->name=="Campus Top LiDAR" || active_map_ptr->name=="Campus Top (Rev)");
        if(isTopLidar && (g_zone_valid || g_zone2_valid)){
            DrawText("Reset reference path (restore known-good copy):", x, y, 10, C_DIM); y+=14;
            float half = (pw-32-8)*0.5f;
            if(g_zone_valid && DrawBtn({(float)x,(float)y,half,24},"Reset Zone 1 Path",{110,60,30,255},true)){
                if(pp_reset_zone_path(1)){ g_statusMsg="Zone 1 path restored"; g_statusCol=C_GOOD; }
                else { g_statusMsg="Zone 1 path reset failed (no backup found)"; g_statusCol=C_BAD; }
            }
            if(g_zone2_valid && DrawBtn({(float)(x+half+8),(float)y,half,24},"Reset Zone 2 Path",{110,60,30,255},true)){
                if(pp_reset_zone_path(2)){ g_statusMsg="Zone 2 path restored"; g_statusCol=C_GOOD; }
                else { g_statusMsg="Zone 2 path reset failed (no backup found)"; g_statusCol=C_BAD; }
            }
            y+=30;
            DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;
        }
    }

#if 0  // Test buttons hidden Ã¢â‚¬â€ not needed for operation
    {
        bool idle = !g_loc_test_running.load();
        if (DrawBtn({(float)x,(float)(py+ph-318),(float)(pw-32),28},
                    "Loc Test (60s)  [stationary]", {60,130,80,255}, idle))
            std::thread([]{ loc_test_run_stationary(60); }).detach();
    }

    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ Steer validation subsection ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
    // Sim/Real UDP toggle ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â steer tests require Real UDP; change mode here
    {
        bool canToggle = !g_steer_test_running.load();
        Color mc = g_simMode ? C_WARN : C_GOOD;
        const char* ml = g_simMode ? "Sim mode ON  ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â  click to enable Real UDP"
                                   : "Real UDP mode  ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â  click for Simulation";
        if (DrawBtn({(float)x,(float)(py+ph-284),(float)(pw-32),24}, ml, mc, canToggle))
            g_simMode = !g_simMode;
    }
    // Steer Test A  /  Steer Test A x5 ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â discrete accuracy (only in Real UDP mode)
    {
        bool idle = !g_steer_test_running.load() && !g_simMode;
        int  bw   = (pw - 40) / 2;
        if (DrawBtn({(float)x,        (float)(py+ph-254),(float)bw,28},
                    "Steer Test A",    {45,80,150,255}, idle))
            std::thread(steer_test_run_A).detach();
        if (DrawBtn({(float)(x+bw+8), (float)(py+ph-254),(float)bw,28},
                    "Steer Test A x5", {45,100,130,255}, idle))
            std::thread([]{ steer_test_run_A_repeat(5); }).detach();
    }
    // Steer Test B ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â continuous tracking (corner simulation)
    {
        bool idle = !g_steer_test_running.load() && !g_simMode;
        if (DrawBtn({(float)x,(float)(py+ph-220),(float)(pw-32),28},
                    "Steer Test B  (tracking)", {45,120,100,255}, idle))
            std::thread(steer_test_run_B).detach();
    }
    // Steer Sweep ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â vary command interval {80..300 ms}, staircase 0ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â¢10.5ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â°ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â¢0
    {
        bool idle = !g_steer_test_running.load() && !g_simMode;
        if (DrawBtn({(float)x,(float)(py+ph-186),(float)(pw-32),28},
                    "Steer Sweep  (interval scan)", {80,60,150,255}, idle))
            std::thread(steer_test_run_sweep).detach();
    }
    // Last steer test result ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â visible in Step 1 without going to Drive panel
    {
        DrawRectangle(x, py+ph-150, pw-32, 1, C_BORDER);
        std::string steer_log;
        { std::lock_guard<std::mutex> lk(g_cmdLog_mutex);
          for (const auto& e : g_cmdLog) {
              if (e.find("[SteerTest")   != std::string::npos ||
                  e.find("[SteerRepeat") != std::string::npos ||
                  e.find("[SteerSweep")  != std::string::npos)
              { steer_log = e; break; }
          }
        }
        const char* sl =
            g_steer_test_running.load() ? "Running steer test..." :
            !steer_log.empty()          ? steer_log.c_str()       :
            g_simMode                   ? "Switch to Real UDP to enable tests" : "Ready ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â no test run yet";
        Color sc =
            g_steer_test_running.load()                                              ? C_WARN :
            (!steer_log.empty() && steer_log.find("REFUSED") != std::string::npos)  ? C_BAD  :
            !steer_log.empty()                                                       ? C_GOOD : C_DIM;
        DrawText(sl, x, py+ph-142, 10, sc);
#endif // Test buttons hidden

    bool tracking=(cur_state==STATE_FUSION_RUNNING&&ir>=(active_map_ptr?active_map_ptr->track_accept:0.35f));
    float gyro_z_now=0.f;
    {std::lock_guard<std::mutex> lk_i(imu_mutex); gyro_z_now=imu_gyro.z;}
    bool stable_for_lock=(fabsf(gyro_z_now)<0.02f && rmse<0.25f && ir>0.50f);
    bool canConfirm=tracking && stable_for_lock;

    const char* confirmLabel = canConfirm
        ? "Confirm Pose  \u2192  Go to Planning"
        : (tracking ? "Hold still to lock pose..." : "Waiting for stable tracking...");

    if(DrawBtn({(float)x,(float)(py+ph-60),(float)(pw-32),38}, confirmLabel, C_GOOD, canConfirm)){
        float cx2, cy2, cyaw_lidar, cyaw_corrected;
        {std::lock_guard<std::mutex> lk(ekf_mutex);
         cx2=(float)ekf.x[0]; cy2=(float)ekf.x[1]; cyaw_lidar=(float)ekf.x[2];}
        {
            std::lock_guard<std::mutex> lk_i(imu_mutex);
            float imu_rad=imu_angle.z*(kPI/180.f);
            g_imu_yaw_at_localize=imu_angle.z;
            g_yaw_imu_lidar_offset_rad=wrapA(imu_rad-cyaw_lidar);
        }
        cyaw_corrected=cyaw_lidar;  // use scan-matcher heading directly; g_yaw_imu_lidar_offset_rad stored for UI display only
        g_confirmed_start={cx2, cy2, cyaw_corrected, true};
        g_pp_robot       ={cx2, cy2, cyaw_corrected, true};
        {std::lock_guard<std::mutex> lk_ekf(ekf_mutex); ekf.x[2]=(double)cyaw_corrected;}
        // Auto-arm whichever zone the real localized pose lands inside — corner-mode
        // routing and destination confirm previews need it armed, and requiring a
        // separate manual "Arm Zone N" click after a successful in-zone localize
        // was just an extra step for the same outcome. Zones are mutually exclusive.
        if(g_zone_valid && !g_zone_poly.empty() && point_in_polygon(cx2, cy2, g_zone_poly)){
            g_zone_armed = true; g_zone2_armed = false;
        } else if(g_zone2_valid && !g_zone2_poly.empty() && point_in_polygon(cx2, cy2, g_zone2_poly)){
            g_zone2_armed = true; g_zone_armed = false;
        }
        g_ui_step.store(STEP_PLAN);
        pp_run_pipeline(TRAJ_CSV_FILE);
        if(g_selected_dest>=0&&g_selected_dest<NUM_DEST&&g_destinations[g_selected_dest].valid){
            auto& d=g_destinations[g_selected_dest]; g_pp_goal={d.x,d.y,0,true}; }
        pp_trigger_plan();
    }
    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ SKIP BUTTON ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â always available ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
    Color skip_col={60,60,110,255};
    int skip_y=py+ph-60;
    if(canConfirm){
        // Confirm takes primary slot, skip goes smaller above
        skip_y=py+ph-106;
    }
    if(DrawBtn({(float)x,(float)skip_y,(float)(pw-32),30},
               "Skip Localization  >>  Go to Planning", skip_col, true)){
        // Use estimated start if available, else (0,0) as last resort.
        // Never overwrite g_pp_robot with (0,0) when an estimated start exists —
        // that was causing the planning path to depart from the map origin.
        if(!g_confirmed_start.valid){
            if(g_est_start.valid){
                g_confirmed_start={g_est_start.x,g_est_start.y,g_est_start.yaw,true};
                // g_pp_robot already set from load_est_start_for_map — don't overwrite
            } else {
                g_confirmed_start={0.f,0.f,0.f,true};
                g_pp_robot={0.f,0.f,0.f,true};
            }
        }
        g_ui_step.store(STEP_PLAN);
        pp_run_pipeline(TRAJ_CSV_FILE);
        if(g_selected_dest>=0&&g_selected_dest<NUM_DEST&&g_destinations[g_selected_dest].valid){
            auto& d=g_destinations[g_selected_dest]; g_pp_goal={d.x,d.y,0,true}; }
        pp_trigger_plan();
    }

    if(tracking && !stable_for_lock){
        const char* block_reason =
            (fabsf(gyro_z_now)>=0.02f) ? "rotating -- hold still" :
            (rmse>=0.25f)               ? "RMSE too high" :
            (ir<=0.50f)                 ? "overlap too low" : "waiting...";
        DrawText(TextFormat("Gyro %.3f rad/s  RMSE %.3f  -> %s",gyro_z_now,rmse,block_reason),
                 x, py+ph-16, 10, C_WARN);
    }
}

// =============================================================================
//  RIGHT PANEL ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â Step 2: Plan
// =============================================================================
void DrawPanelPlan(int px, int py, int pw, int ph) {
    DrawRectangle(px,py,pw,ph,C_PANEL_DARK);
    DrawRectangle(px,py,1,ph,C_BORDER_HI);
    int x=px+16, y=py+16;
    DrawText("02  PLAN ROUTE",x,y,18,C_WARN); y+=28;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=12;

    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ navigation buttons ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
    int nav_bw=(pw-48)/2;
    if(DrawBtn({(float)x,(float)y,(float)nav_bw,26},"<< Back to Localize",{50,50,90,255},true))
        g_ui_step.store(STEP_LOCALIZE);
    if(DrawBtn({(float)(x+nav_bw+16),(float)y,(float)nav_bw,26},"Skip to Drive >>",{50,90,50,255},true)){
        g_drive_via_passenger=false; g_ui_step.store(STEP_DRIVE);
    }
    y+=32;
    if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Return to Passenger View",{70,50,110,255},true))
        g_show_dest = true;
    y+=34;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    // Confirmed-start-pose + path-stats readout hidden per request (kept the
    // hasPath/hasGoal/hasPlan flags below since Confirm/Save logic uses them).
    bool hasPath=!g_smooth.empty(), hasGoal=g_pp_goal.valid;
    bool hasPlan=!g_approach_path.empty()||!g_corridor_path.empty();

    // ── Edit Path — select a stretch and nudge it sideways (off a wall) ──
    // Placed high in the panel so it stays visible without scrolling.
    {
        static std::string edit_status; static Color edit_col=C_DIM;
        if(!g_path_edit_mode){
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Edit Path (nudge a stretch)",{70,90,120,255},!g_smooth.empty())){
                g_path_edit_mode=true; g_path_edit_i0=-1; g_path_edit_i1=-1; g_path_edit_offset=0.f;
                edit_status.clear();
            }
            y+=30;
        } else {
            DrawText("EDIT PATH — click two points on the path", x, y, 12, C_WARN); y+=15;
            bool bothSel = (g_path_edit_i0>=0 && g_path_edit_i1>=0);
            if(bothSel){
                int lo=std::min(g_path_edit_i0,g_path_edit_i1), hi=std::max(g_path_edit_i0,g_path_edit_i1);
                DrawText(TextFormat("  selected %d pts   offset %+.2f m", hi-lo+1, g_path_edit_offset),
                         x, y, 11, C_WHITE); y+=16;
                DrawText(TextFormat("  [%d] %.3f, %.3f", lo, g_smooth[lo].x, g_smooth[lo].y),
                         x, y, 11, C_DIM); y+=14;
                DrawText(TextFormat("  [%d] %.3f, %.3f", hi, g_smooth[hi].x, g_smooth[hi].y),
                         x, y, 11, C_DIM); y+=16;
                int bw=(pw-48)/2;
                if(DrawBtn({(float)x,(float)y,(float)bw,26},"<< Nudge Left",{60,90,120,255},true)){
                    pp_shift_path_segment(lo,hi,+0.10f); g_path_edit_offset+=0.10f;
                    if(g_pp_goal.valid) pp_trigger_plan();
                }
                if(DrawBtn({(float)(x+bw+16),(float)y,(float)bw,26},"Nudge Right >>",{60,90,120,255},true)){
                    pp_shift_path_segment(lo,hi,-0.10f); g_path_edit_offset-=0.10f;
                    if(g_pp_goal.valid) pp_trigger_plan();
                }
                y+=30;
                if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Straighten Selection",{130,90,40,255},true)){
                    pp_straighten_path_segment(lo,hi); g_path_edit_offset=0.f;
                    if(g_pp_goal.valid) pp_trigger_plan();
                }
                y+=28;
            } else {
                DrawText(g_path_edit_i0>=0 ? "  click the 2nd point..." : "  click the 1st point...",
                         x, y, 11, C_DIM); y+=16;
            }
            int bw2=(pw-48)/2;
            if(DrawBtn({(float)x,(float)y,(float)bw2,26},"Save Edits",C_GOOD,true)){
                bool ok = pp_save_edited_path();
                edit_status = ok ? "Path saved" : "Save failed";
                edit_col    = ok ? C_GOOD : C_BAD;
            }
            if(DrawBtn({(float)(x+bw2+16),(float)y,(float)bw2,26},"Reset",{120,70,40,255},true)){
                pp_run_pipeline(TRAJ_CSV_FILE); if(g_pp_goal.valid) pp_trigger_plan();
                g_path_edit_i0=-1; g_path_edit_i1=-1; g_path_edit_offset=0.f; edit_status="Reloaded from file";
                edit_col=C_WARN;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Done Editing",{55,55,80,255},true)){
                g_path_edit_mode=false; g_path_edit_i0=-1; g_path_edit_i1=-1;
            }
            y+=28;
            if(!edit_status.empty()){ DrawText(edit_status.c_str(), x, y, 11, edit_col); y+=15; }
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Set Danger Zone — mark the localization-unreliable stretch; can span zone 1 and 2 ──
    {
        static std::string dz_status; static Color dz_col=C_DIM;
        bool isTopLidarMap = active_map_ptr &&
            (active_map_ptr->name=="Campus Top LiDAR" || active_map_ptr->name=="Campus Top (Rev)");
        if(!g_dz_edit_mode){
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Set Danger Zone",{140,60,60,255},isTopLidarMap)){
                g_dz_edit_mode=true; g_dz_edit_zone0=-1; g_dz_edit_idx0=-1; g_dz_edit_zone1=-1; g_dz_edit_idx1=-1;
                dz_status.clear();
            }
            y+=30;
        } else {
            DrawText("DANGER ZONE — click the start, then the end", x, y, 12, RED); y+=15;
            DrawText("  zone 1 = blue, zone 2 = orange (no arming needed)", x, y, 10, C_DIM); y+=14;
            bool bothSel = (g_dz_edit_idx0>=0 && g_dz_edit_idx1>=0);
            if(bothSel){
                DrawText(TextFormat("  selected: zone %d pt %d -> zone %d pt %d",
                         g_dz_edit_zone0,g_dz_edit_idx0,g_dz_edit_zone1,g_dz_edit_idx1), x, y, 11, C_WHITE); y+=16;
            } else {
                DrawText(g_dz_edit_idx0>=0 ? "  click the end point..." : "  click the start point...",
                         x, y, 11, C_DIM); y+=16;
            }
            int bw3=(pw-48)/2;
            if(DrawBtn({(float)x,(float)y,(float)bw3,26},"Save Danger Zone",C_GOOD,bothSel)){
                bool ok = pp_save_danger_zone(g_dz_edit_zone0,g_dz_edit_idx0,g_dz_edit_zone1,g_dz_edit_idx1);
                if(ok) ctrl_resplice_danger_zone_now();   // push into Step 3's plan immediately
                dz_status = ok ? "Danger zone saved" : "Save failed";
                dz_col    = ok ? C_GOOD : C_BAD;
            }
            if(DrawBtn({(float)(x+bw3+16),(float)y,(float)bw3,26},"Cancel",{120,70,40,255},true)){
                g_dz_edit_zone0=-1; g_dz_edit_idx0=-1; g_dz_edit_zone1=-1; g_dz_edit_idx1=-1;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Done",{55,55,80,255},true)){
                g_dz_edit_mode=false; g_dz_edit_zone0=-1; g_dz_edit_idx0=-1; g_dz_edit_zone1=-1; g_dz_edit_idx1=-1;
            }
            y+=28;
            if(!dz_status.empty()){ DrawText(dz_status.c_str(), x, y, 11, dz_col); y+=15; }
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Set Sub-Zone (Danger Zone Fwd test map only) — mark the stretch where the
    // cart should ignore the curve and drive straight, zero steer, until it exits ──
    {
        static std::string dzs_status; static Color dzs_col=C_DIM;
        bool isDzTestMap = active_map_ptr &&
            (active_map_ptr->name=="Danger Zone Fwd" || active_map_ptr->name=="Danger Zone Rev");
        if(!g_dzsub_edit_mode){
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Set Sub-Zone",{140,60,60,255},isDzTestMap)){
                g_dzsub_edit_mode=true; g_dzsub_edit_have0=false; g_dzsub_edit_have1=false;
                dzs_status.clear();
            }
            y+=30;
        } else {
            DrawText("SUB-ZONE — click the start, then the end", x, y, 12, RED); y+=15;
            if(g_dzsub_edit_have0 && g_dzsub_edit_have1){
                DrawText(TextFormat("  selected: (%.1f,%.1f) -> (%.1f,%.1f)",
                         g_dzsub_edit_x0,g_dzsub_edit_y0,g_dzsub_edit_x1,g_dzsub_edit_y1), x, y, 11, C_WHITE); y+=16;
            } else {
                DrawText(g_dzsub_edit_have0 ? "  click the end point..." : "  click the start point...",
                         x, y, 11, C_DIM); y+=16;
            }
            int bw4=(pw-48)/2;
            bool bothSel4 = g_dzsub_edit_have0 && g_dzsub_edit_have1;
            if(DrawBtn({(float)x,(float)y,(float)bw4,26},"Save Sub-Zone",C_GOOD,bothSel4)){
                g_dzsub_entry_mk_x=g_dzsub_edit_x0; g_dzsub_entry_mk_y=g_dzsub_edit_y0;
                g_dzsub_exit_mk_x =g_dzsub_edit_x1; g_dzsub_exit_mk_y =g_dzsub_edit_y1;
                save_dzsub_fwd();
                ctrl_relocate_dzsub_now();   // push into Step 3's plan immediately
                dzs_status = "Sub-zone saved"; dzs_col = C_GOOD;
            }
            if(DrawBtn({(float)(x+bw4+16),(float)y,(float)bw4,26},"Cancel",{120,70,40,255},true)){
                g_dzsub_edit_have0=false; g_dzsub_edit_have1=false;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Done",{55,55,80,255},true)){
                g_dzsub_edit_mode=false; g_dzsub_edit_have0=false; g_dzsub_edit_have1=false;
            }
            y+=28;
            if(!dzs_status.empty()){ DrawText(dzs_status.c_str(), x, y, 11, dzs_col); y+=15; }
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Set Zone 2 Slow Segment (top-LiDAR maps) — mark a stretch of zone 2's
    // corridor where the ZONE2_CRUISE_VOLTAGE boost breaks scan-matching (found on the
    // reverse map); control.cpp suppresses the boost there, using CRUISE_VOLTAGE instead. ──
    {
        static std::string z2s_status; static Color z2s_col=C_DIM;
        bool isTopLidarMap2 = active_map_ptr &&
            (active_map_ptr->name=="Campus Top LiDAR" || active_map_ptr->name=="Campus Top (Rev)");
        if(!g_z2slow_edit_mode){
            const char* lbl2 = g_z2slow_valid ? "Re-set Zone 2 Slow Segment" : "Set Zone 2 Slow Segment";
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},lbl2,{150,110,40,255},isTopLidarMap2)){
                g_z2slow_edit_mode=true; g_z2slow_edit_have0=false; g_z2slow_edit_have1=false;
                z2s_status.clear();
            }
            y+=16;
            DrawText(g_z2slow_valid ? "  segment set - zone 2 boost suppressed there"
                                     : "  none set - full zone 2 uses boosted speed",
                      x, y, 10, g_z2slow_valid?C_GOOD:C_DIM); y+=14;
            if(g_z2slow_valid){
                if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},"Clear Slow Segment",{100,55,40,255},isTopLidarMap2)){
                    clear_z2slow_for_map();
                    z2s_status="Cleared"; z2s_col=C_GOOD;
                }
                y+=26;
            }
            if(!z2s_status.empty()){ DrawText(z2s_status.c_str(), x, y, 11, z2s_col); y+=15; }
        } else {
            DrawText("SLOW SEGMENT - click the start, then the end", x, y, 12, C_WARN); y+=15;
            DrawText("  points snap to zone 2's corridor", x, y, 10, C_DIM); y+=14;
            if(g_z2slow_edit_have0 && g_z2slow_edit_have1){
                DrawText(TextFormat("  selected: (%.1f,%.1f) -> (%.1f,%.1f)",
                         g_z2slow_edit_x0,g_z2slow_edit_y0,g_z2slow_edit_x1,g_z2slow_edit_y1), x, y, 11, C_WHITE); y+=16;
            } else {
                DrawText(g_z2slow_edit_have0 ? "  click the end point..." : "  click the start point...",
                         x, y, 11, C_DIM); y+=16;
            }
            int bw5=(pw-48)/2;
            bool bothSel5 = g_z2slow_edit_have0 && g_z2slow_edit_have1;
            if(DrawBtn({(float)x,(float)y,(float)bw5,26},"Save Segment",C_GOOD,bothSel5)){
                g_z2slow_entry_x=g_z2slow_edit_x0; g_z2slow_entry_y=g_z2slow_edit_y0;
                g_z2slow_exit_x =g_z2slow_edit_x1; g_z2slow_exit_y =g_z2slow_edit_y1;
                save_z2slow_for_map();
                z2s_status = "Slow segment saved"; z2s_col = C_GOOD;
            }
            if(DrawBtn({(float)(x+bw5+16),(float)y,(float)bw5,26},"Cancel",{120,70,40,255},true)){
                g_z2slow_edit_have0=false; g_z2slow_edit_have1=false;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Done",{55,55,80,255},true)){
                g_z2slow_edit_mode=false; g_z2slow_edit_have0=false; g_z2slow_edit_have1=false;
            }
            y+=28;
            if(!z2s_status.empty()){ DrawText(z2s_status.c_str(), x, y, 11, z2s_col); y+=15; }
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Obstacle Test Mode (2026-07-10, test feature) — works on ANY map with a
    // loaded reference path (g_smooth), unlike the tools above which are gated to
    // specific maps. While active, clicking the map cycles the nearest in-band grid
    // cell: none -> pedestrian (orange, ties into the existing camera-pedestrian
    // stop/resume flow) -> static obstacle (red, marked/visualized only for now — no
    // avoidance planning yet, that's the next phase) -> none.
    {
        bool haveRefPath = !g_smooth.empty();
        Color otCol = g_obstacle_test_mode ? Color{150,60,140,255} : Color{70,55,90,255};
        const char* otLbl = g_obstacle_test_mode ? "Obstacle Test Mode: ON  (click map to place)"
                                                  : "Obstacle Test Mode: OFF";
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),28}, otLbl, otCol, haveRefPath)){
            g_obstacle_test_mode = !g_obstacle_test_mode;
            if(g_obstacle_test_mode) g_border_edit_mode = false;   // mutually exclusive click modes
        }
        y+=32;
        if(g_obstacle_test_mode){
            DrawText("  click a cell: 1st=pedestrian(stop) 2nd=static(forbidden) 3rd=clear",
                      x, y, 10, C_DIM); y+=15;
            int n_ped=0, n_static=0;
            for(auto& kv : g_obstacle_grid){
                if(kv.second==OBST_PEDESTRIAN) n_ped++; else if(kv.second==OBST_STATIC) n_static++;
            }
            DrawText(TextFormat("  pedestrian: %d   static: %d", n_ped, n_static), x, y, 10, C_WHITE); y+=15;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},"Clear All Obstacles",{100,40,40,255},
                       !g_obstacle_grid.empty())){
                grid_clear_all();
            }
            y+=26;
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Border/Wall Cells (2026-07-10, extra option) — auto-generated forbidden
    // cells running along both sides of the path at CORRIDOR_HALF_M (the
    // watchdog-trip edge — past this the cart is hitting the wall). Persisted
    // per-map, unlike the pedestrian/static obstacle grid above (that one is
    // runtime-only test data; this one is meant to be generated once, hand-tuned,
    // and saved). Same enforcement scope as the obstacle grid: marked and
    // visualized only for now — the path planner doesn't avoid these yet.
    {
        bool haveRefPath = !g_smooth.empty();
        DrawText("Border cells (forbidden, near corridor edge):", x, y, 12, C_HEADER); y+=16;
        int bwB=(pw-32-16)/2;
        if(DrawBtn({(float)x,(float)y,(float)bwB,26},"Auto-Generate Borders",{90,60,30,255},haveRefPath)){
            border_auto_generate();
        }
        Color beCol = g_border_edit_mode ? Color{150,60,140,255} : Color{70,55,90,255};
        if(DrawBtn({(float)(x+bwB+16),(float)y,(float)bwB,26},
                   g_border_edit_mode?"Edit: ON":"Edit: OFF", beCol, haveRefPath)){
            g_border_edit_mode = !g_border_edit_mode;
            if(g_border_edit_mode) g_obstacle_test_mode = false;   // mutually exclusive click modes
        }
        y+=30;
        DrawText(TextFormat("  %d border cells", (int)g_border_grid.size()), x, y, 10, C_WHITE); y+=15;
        if(g_border_edit_mode){
            DrawText("  click map: adds if empty, erases if present", x, y, 10, C_DIM); y+=15;
        }
        // Visibility toggle (2026-07-27, user request) — border cells previously had
        // no way to hide once generated/loaded (drawing was unconditional on
        // g_border_grid being non-empty); this decouples "the data exists" from
        // "it's currently drawn on the map".
        Color bvCol = g_show_border_cells ? Color{60,90,150,255} : Color{70,70,70,255};
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                   g_show_border_cells?"Show Borders: ON (click to hide)":"Show Borders: OFF (click to show)",
                   bvCol, !g_border_grid.empty())){
            g_show_border_cells = !g_show_border_cells;
        }
        y+=28;
        int bwB2=(pw-32-16)/2;
        if(DrawBtn({(float)x,(float)y,(float)bwB2,24},"Save Borders",C_GOOD,!g_border_grid.empty())){
            save_border_grid();
        }
        if(DrawBtn({(float)(x+bwB2+16),(float)y,(float)bwB2,24},"Clear All Borders",{100,40,40,255},
                   !g_border_grid.empty())){
            border_clear_all();
        }
        y+=28;
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // ── Place Danger Path (Danger Zone Fwd test map only) — drag/rotate the OLD
    // campus danger-zone curve into position by eye, then splice it into g_smooth ──
    {
        static std::string dzp_status; static Color dzp_col=C_DIM;
        bool isDzTestMap = active_map_ptr &&
            (active_map_ptr->name=="Danger Zone Fwd" || active_map_ptr->name=="Danger Zone Rev");
        if(!g_dzplace_mode){
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Place Danger Path",{140,60,140,255},isDzTestMap)){
                if(load_dzplace_raw()){ g_dzplace_mode=true; dzp_status.clear(); }
                else { dzp_status="danger_zone_forward.csv not found/empty"; dzp_col=C_BAD; }
            }
            y+=30;
        } else {
            DrawText("PLACE PATH — drag = move, right-drag = rotate", x, y, 12, MAGENTA); y+=15;
            DrawText(TextFormat("  offset (%.2f,%.2f) m   rot %.1f deg",
                     g_dzplace_offset_x,g_dzplace_offset_y,toDeg(g_dzplace_rotation)), x, y, 11, C_WHITE); y+=16;
            int bw5=(pw-48)/2;
            if(DrawBtn({(float)x,(float)y,(float)bw5,26},"Save Into Path",C_GOOD,true)){
                bool ok = dzplace_save_into_smooth();
                if(ok) ctrl_relocate_dzsub_now();
                dzp_status = ok ? "Spliced and saved" : "Save failed";
                dzp_col    = ok ? C_GOOD : C_BAD;
            }
            if(DrawBtn({(float)(x+bw5+16),(float)y,(float)bw5,26},"Reset",{120,70,40,255},true)){
                g_dzplace_offset_x=0.f; g_dzplace_offset_y=0.f; g_dzplace_rotation=0.f;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Done",{55,55,80,255},true)){
                g_dzplace_mode=false;
            }
            y+=28;
            if(!dzp_status.empty()){ DrawText(dzp_status.c_str(), x, y, 11, dzp_col); y+=15; }
        }
        DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
    }

    // --- Set Destinations section (zone-1 + zone-2) — collapsible (2026-07-10) ---
    // The two "Pick a destination" Go-button lists took up most of the panel height
    // when idle. Collapsed by default behind one toggle; auto-forced open while a
    // destination-placement flow (Set Destinations on Map, either zone) is active so
    // the operator is never stuck unable to see what they're doing mid-setup.
    static bool s_destPanelCollapsed = true;
    bool destSetupActive = g_dest_setup_mode || g_dest_setup_mode2;
    bool showDestPanel = !s_destPanelCollapsed || destSetupActive;
    {
        const char* lbl = s_destPanelCollapsed ? "Destinations >>  (click to expand)"
                                                : "Destinations vv  (click to collapse)";
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),26}, lbl, {55,55,80,255}, true)){
            s_destPanelCollapsed = !s_destPanelCollapsed;
        }
        y += 30;
    }
    if(showDestPanel){
    // --- Set Destinations section (zone-1 destinations; more zones planned later) ---
    // Destination identity (name + index) is shared/global — see g_dest_names.
    // Only the world (x,y) per index is per-map, from dest_<plyname>.cfg.
    // Setup is pick → preview path → Confirm/Retry/Cancel; nothing is saved to
    // g_destinations (or disk) until Confirm.
    {
        if(!g_dest_setup_mode){
            // Pick a destination → set goal + generate the path immediately.
            DrawText("Pick a destination (generates path):",x,y,12,C_HEADER); y+=16;
            for(int i=0;i<NUM_DEST;i++){
                bool sel = (g_selected_dest==i);
                if(g_destinations[i].valid){
                    Color bc = sel ? Color{40,110,70,255} : Color{50,70,95,255};
                    if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                               TextFormat("%s Go: %s  (%.1f, %.1f)", sel?">":" ", g_dest_names[i],
                                          g_destinations[i].x, g_destinations[i].y), bc, true)){
                        g_selected_dest = i;
                        if(!g_zone_armed){ g_zone_armed=true; g_zone2_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
                        g_pp_goal = {g_destinations[i].x, g_destinations[i].y, 0, true};
                        g_journey_dest_name = g_dest_names[i];   // friendly-panel pin label
                        pp_trigger_plan();   // build path now (corner-mode if zone armed)
                        // Also save+load into g_ctrl_path immediately (same as passenger
                        // "Begin Journey") — without this, Step 3 never gets a plan at all
                        // from this button (only the live Step-2 preview updates), so any
                        // danger-zone splice/edit has nothing loaded to apply to.
                        if(g_pp_robot.valid && (!g_approach_path.empty() || !g_corridor_path.empty())){
                            pp_save_plan(); ctrl_doLoad();
                        }
                    }
                    y+=27;
                } else {
                    DrawText(TextFormat("  %s: not set", g_dest_names[i]), x, y, 11, C_BAD); y+=15;
                }
            }
            y+=4;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Set Destinations on Map",{55,55,100,255},true)){
                g_dest_setup_mode=true; g_dest_setup_idx=-1; g_dest_setup_previewing=false;
            }
            y+=32;
        } else if(g_dest_setup_idx<0) {
            // ── Picker list: choose which named destination to (re)place ──
            DrawText("SET DESTINATIONS — pick one to place:",x,y,13,C_WARN); y+=18;
            for(int i=0;i<NUM_DEST;i++){
                Color bc = g_destinations[i].valid ? Color{40,90,60,255} : Color{55,55,80,255};
                if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                           TextFormat("%s: %s", g_dest_names[i],
                               g_destinations[i].valid ?
                               TextFormat("%.1f,%.1f",g_destinations[i].x,g_destinations[i].y) : "not set"),
                           bc, true)){
                    g_dest_setup_idx = i;
                    g_dest_setup_previewing = false;
                    g_dest_setup_prev_goal = g_pp_goal;   // snapshot to restore on Cancel
                }
                y+=27;
            }
            y+=4;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Done",{60,90,60,255},true)){
                g_dest_setup_mode=false;
            }
            y+=32;
        } else if(!g_dest_setup_previewing) {
            // ── Awaiting a map click for the chosen index ──
            DrawText(TextFormat("Click the map to place: %s", g_dest_names[g_dest_setup_idx]),
                      x, y, 12, C_WARN); y+=18;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Cancel",{80,40,40,255},true)){
                g_pp_goal = g_dest_setup_prev_goal;
                if(g_dest_setup_prev_goal.valid) pp_trigger_plan();
                else { g_approach_path.clear(); g_corridor_path.clear(); }
                g_dest_setup_idx = -1;
            }
            y+=30;
        } else {
            // ── Previewing a just-clicked candidate — confirm or retry ──
            DrawText(TextFormat("Preview: %s  (%.1f, %.1f)", g_dest_names[g_dest_setup_idx],
                      g_pp_goal.x, g_pp_goal.y), x, y, 12, C_WARN); y+=16;
            DrawText(TextFormat("Approach: %d pts   Corridor: %d pts",
                      (int)g_approach_path.size(), (int)g_corridor_path.size()), x, y, 11, C_WHITE); y+=18;
            int bw3=(pw-32-16)/2;
            if(DrawBtn({(float)x,(float)y,(float)bw3,26},"Confirm",C_GOOD,true)){
                g_destinations[g_dest_setup_idx] = {g_dest_names[g_dest_setup_idx], g_pp_goal.x, g_pp_goal.y, true};
                save_destinations_for_map();
                g_dest_setup_idx = -1;
                g_dest_setup_previewing = false;
            }
            if(DrawBtn({(float)(x+bw3+16),(float)y,(float)bw3,26},"Retry",{80,80,40,255},true)){
                g_dest_setup_previewing = false;   // click the map again for a new candidate
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},"Cancel",{80,40,40,255},true)){
                g_pp_goal = g_dest_setup_prev_goal;
                if(g_dest_setup_prev_goal.valid) pp_trigger_plan();
                else { g_approach_path.clear(); g_corridor_path.clear(); }
                g_dest_setup_idx = -1;
                g_dest_setup_previewing = false;
            }
            y+=26;
        }
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    // --- Set Destinations section (zone-2 destinations) ---
    // Mirrors the zone-1 section above exactly, operating on g_destinations2/
    // g_dest_names2/g_dest_setup_*2 and arming zone 2 (not zone 1) before each preview.
    {
        if(!g_dest_setup_mode2){
            DrawText("Pick a zone-2 destination (generates path):",x,y,12,C_HEADER); y+=16;
            for(int i=0;i<NUM_DEST2;i++){
                bool sel = (g_selected_dest2==i);
                if(g_destinations2[i].valid){
                    Color bc = sel ? Color{110,70,40,255} : Color{95,70,50,255};
                    if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                               TextFormat("%s Go: %s  (%.1f, %.1f)", sel?">":" ", g_dest_names2[i],
                                          g_destinations2[i].x, g_destinations2[i].y), bc, true)){
                        g_selected_dest2 = i;
                        if(!g_zone2_armed){ g_zone2_armed=true; g_zone_armed=false; pp_run_pipeline(TRAJ_CSV_FILE); }
                        g_pp_goal = {g_destinations2[i].x, g_destinations2[i].y, 0, true};
                        g_journey_dest_name = g_dest_names2[i];   // friendly-panel pin label
                        pp_trigger_plan();
                        // Also save+load into g_ctrl_path immediately (same as passenger
                        // "Begin Journey") — see matching comment on the zone-1 Go button.
                        if(g_pp_robot.valid && (!g_approach_path.empty() || !g_corridor_path.empty())){
                            pp_save_plan(); ctrl_doLoad();
                        }
                    }
                    y+=27;
                } else {
                    DrawText(TextFormat("  %s: not set", g_dest_names2[i]), x, y, 11, C_BAD); y+=15;
                }
            }
            y+=4;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Set Zone 2 Destinations on Map",{100,70,55,255},true)){
                g_dest_setup_mode2=true; g_dest_setup_idx2=-1; g_dest_setup_previewing2=false;
            }
            y+=32;
        } else if(g_dest_setup_idx2<0) {
            DrawText("SET ZONE 2 DESTINATIONS — pick one to place:",x,y,13,C_WARN); y+=18;
            for(int i=0;i<NUM_DEST2;i++){
                Color bc = g_destinations2[i].valid ? Color{90,70,40,255} : Color{55,55,80,255};
                if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                           TextFormat("%s: %s", g_dest_names2[i],
                               g_destinations2[i].valid ?
                               TextFormat("%.1f,%.1f",g_destinations2[i].x,g_destinations2[i].y) : "not set"),
                           bc, true)){
                    g_dest_setup_idx2 = i;
                    g_dest_setup_previewing2 = false;
                    g_dest_setup_prev_goal2 = g_pp_goal;
                }
                y+=27;
            }
            y+=4;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Done",{60,90,60,255},true)){
                g_dest_setup_mode2=false;
            }
            y+=32;
        } else if(!g_dest_setup_previewing2) {
            DrawText(TextFormat("Click the map to place: %s", g_dest_names2[g_dest_setup_idx2]),
                      x, y, 12, C_WARN); y+=18;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Cancel",{80,40,40,255},true)){
                g_pp_goal = g_dest_setup_prev_goal2;
                if(g_dest_setup_prev_goal2.valid) pp_trigger_plan();
                else { g_approach_path.clear(); g_corridor_path.clear(); }
                g_dest_setup_idx2 = -1;
            }
            y+=30;
        } else {
            DrawText(TextFormat("Preview: %s  (%.1f, %.1f)", g_dest_names2[g_dest_setup_idx2],
                      g_pp_goal.x, g_pp_goal.y), x, y, 12, C_WARN); y+=16;
            DrawText(TextFormat("Approach: %d pts   Corridor: %d pts",
                      (int)g_approach_path.size(), (int)g_corridor_path.size()), x, y, 11, C_WHITE); y+=18;
            int bw4=(pw-32-16)/2;
            if(DrawBtn({(float)x,(float)y,(float)bw4,26},"Confirm",C_GOOD,true)){
                g_destinations2[g_dest_setup_idx2] = {g_dest_names2[g_dest_setup_idx2], g_pp_goal.x, g_pp_goal.y, true};
                save_destinations2_for_map();
                g_dest_setup_idx2 = -1;
                g_dest_setup_previewing2 = false;
            }
            if(DrawBtn({(float)(x+bw4+16),(float)y,(float)bw4,26},"Retry",{80,80,40,255},true)){
                g_dest_setup_previewing2 = false;
            }
            y+=30;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},"Cancel",{80,40,40,255},true)){
                g_pp_goal = g_dest_setup_prev_goal2;
                if(g_dest_setup_prev_goal2.valid) pp_trigger_plan();
                else { g_approach_path.clear(); g_corridor_path.clear(); }
                g_dest_setup_idx2 = -1;
                g_dest_setup_previewing2 = false;
            }
            y+=26;
        }
    }
    }   // end if(showDestPanel)
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    // ── Destination grid zones (tap-to-navigate) — HIDDEN ──
    // Superseded by the passenger pax-grid tap system; hidden to keep the Step 2
    // panel short enough that the Edit Path / Build / Confirm controls stay visible.
    // Re-enable by changing the 0 below to 1 if the Step-2 tap grid is needed again.
#if 0
    // Combined index: 0..NUM_DEST-1 = zone 1, NUM_DEST..NUM_DEST_TOTAL-1 = zone 2
    // (see resolve_combined_dest in common.hpp/main.cpp).
    {
        auto combinedName  = combinedDestName;
        auto combinedColor = combinedDestColor;
        if(!g_dest_grid_edit){
            DrawText("Tap Zones:",x,y,12,C_HEADER); y+=14;
            for(int i=0;i<NUM_DEST_TOTAL;i++){
                int cnt=0; for(auto&kv:g_dest_grid) if(kv.second==i) cnt++;
                Color tc=cnt?combinedColor(i):C_DIM;
                DrawText(TextFormat("  %s: %d cell%s",combinedName(i),cnt,cnt==1?"":"s"),x,y,11,tc); y+=13;
            }
            y+=3;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Paint Tap Zones",{55,55,100,255},true)){
                g_dest_grid_edit=true; g_dest_grid_paint_dest=0;
            }
            y+=28;
        } else {
            DrawText("PAINT MODE — click cells on map",x,y,11,C_WARN); y+=16;
            for(int i=0;i<NUM_DEST_TOTAL;i++){
                bool sel=(g_dest_grid_paint_dest==i);
                int cnt=0; for(auto&kv:g_dest_grid) if(kv.second==i) cnt++;
                Color bc=sel?combinedColor(i):Color{50,50,65,255};
                if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},
                           TextFormat("%s %s  (%d cells)",sel?"*":" ",combinedName(i),cnt),bc,true))
                    g_dest_grid_paint_dest=i;
                y+=25;
            }
            bool erase=(g_dest_grid_paint_dest<0);
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),22},
                       TextFormat("%s Eraser",erase?"*":" "),erase?C_BAD:Color{50,50,65,255},true))
                g_dest_grid_paint_dest=-1;
            y+=25;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},"Done — Save Zones",C_GOOD,true)){
                g_dest_grid_edit=false;
                save_dest_grid_for_map();
            }
            y+=30;
        }
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;
#endif

    // ── Zone 1 drawing (per map) ──
    {
        if(!g_zone_edit_mode){
            DrawText(TextFormat("Zone 1: %s", g_zone_valid ? "defined" : "not set"),
                     x, y, 12, g_zone_valid ? C_GOOD : C_DIM); y+=16;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},
                       g_zone_valid ? "Redraw Zone 1" : "Draw Zone 1", {55,80,110,255}, true)){
                g_zone_edit_mode = true; g_zone_edit_verts.clear();
            }
            y+=32;
        } else {
            DrawText("DRAW ZONE 1 — click map to add corners", x, y, 12, C_WARN); y+=16;
            DrawText(TextFormat("  %d vertices", (int)g_zone_edit_verts.size()), x, y, 11, C_WHITE); y+=15;
            int bw=(pw-48)/2;
            bool canSaveZone=(int)g_zone_edit_verts.size()>=3;
            if(DrawBtn({(float)x,(float)y,(float)bw,28},"Save Zone 1",{30,120,60,255},canSaveZone)){
                save_zone(g_zone_edit_verts);
                g_zone_edit_mode=false; g_zone_edit_verts.clear();
            }
            if(DrawBtn({(float)(x+bw+16),(float)y,(float)bw,28},"Cancel",{80,40,40,255},true)){
                g_zone_edit_mode=false; g_zone_edit_verts.clear();
            }
            y+=34;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Undo Last Vertex",{60,50,80,255},
                       !g_zone_edit_verts.empty()))
                g_zone_edit_verts.pop_back();
            y+=30;
        }
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    // ── Zone 2 drawing (per map) ──
    {
        if(!g_zone2_edit_mode){
            DrawText(TextFormat("Zone 2: %s", g_zone2_valid ? "defined" : "not set"),
                     x, y, 12, g_zone2_valid ? C_GOOD : C_DIM); y+=16;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),26},
                       g_zone2_valid ? "Redraw Zone 2" : "Draw Zone 2", {110,80,55,255}, true)){
                g_zone2_edit_mode = true; g_zone2_edit_verts.clear();
            }
            y+=32;
        } else {
            DrawText("DRAW ZONE 2 — click map to add corners", x, y, 12, C_WARN); y+=16;
            DrawText(TextFormat("  %d vertices", (int)g_zone2_edit_verts.size()), x, y, 11, C_WHITE); y+=15;
            int bw2=(pw-48)/2;
            bool canSaveZone2=(int)g_zone2_edit_verts.size()>=3;
            if(DrawBtn({(float)x,(float)y,(float)bw2,28},"Save Zone 2",{30,120,60,255},canSaveZone2)){
                save_zone2(g_zone2_edit_verts);
                g_zone2_edit_mode=false; g_zone2_edit_verts.clear();
            }
            if(DrawBtn({(float)(x+bw2+16),(float)y,(float)bw2,28},"Cancel",{80,40,40,255},true)){
                g_zone2_edit_mode=false; g_zone2_edit_verts.clear();
            }
            y+=34;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Undo Last Vertex",{60,50,80,255},
                       !g_zone2_edit_verts.empty()))
                g_zone2_edit_verts.pop_back();
            y+=30;
        }
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    // ── Estimated start point (per map) ──
    {
        if(!g_est_start_mode){
            DrawText(g_est_start.valid
                     ? TextFormat("Est. start: %.1f, %.1f  (%.0f deg)", g_est_start.x, g_est_start.y, g_est_start.yaw*180.f/kPI)
                     : "Est. start: not set", x, y, 11, g_est_start.valid ? C_GOOD : C_DIM); y+=15;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},
                       g_est_start.valid ? "Re-set Start Point" : "Set Start Point", {55,90,110,255}, true)){
                g_est_start_mode = true;
            }
            y+=28;
        } else {
            DrawText("Click map where the cart starts", x, y, 12, C_WARN); y+=16;
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Cancel",{80,40,40,255},true))
                g_est_start_mode=false;
            y+=28;
        }
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

#if 0  // Corridor polygon panel — hidden from UI
    if(!g_polygon_edit_mode){
        // Mode indicator
        if(g_segment_polygon_valid && g_pp_robot.valid){
            bool in_corr = point_in_polygon(g_pp_robot.x, g_pp_robot.y, g_segment_polygon);
            DrawText(in_corr ? "Mode: CORRIDOR (ref path)" : "Mode: FREE (direct path)",
                     x, y, 12, in_corr ? C_GOOD : C_HEADER);
        } else if(g_segment_polygon_valid){
            DrawText(TextFormat("Polygon: %d verts defined",(int)g_segment_polygon.size()),
                     x, y, 12, C_DIM);
        } else {
            DrawText("Polygon: none ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â always corridor",x,y,12,C_DIM);
        }
        y+=16;
        int bpoly_w=(pw-48)/2;
        if(DrawBtn({(float)x,(float)y,(float)bpoly_w,26},"Edit Polygon",{55,55,100,255},true)){
            g_polygon_edit_mode = true;
            g_polygon_edit_verts = g_segment_polygon; // start from existing vertices
        }
        if(g_segment_polygon_valid){
            if(DrawBtn({(float)(x+bpoly_w+16),(float)y,(float)bpoly_w,26},"Clear Polygon",{80,30,30,255},true)){
                g_segment_polygon.clear(); g_segment_polygon_valid=false;
                save_segment_polygon("segment_polygon.cfg");
            }
        }
        y+=34;
    } else {
        DrawText("POLYGON EDIT MODE",x,y,13,C_WARN); y+=16;
        DrawText(TextFormat("  %d vertices  (click map to add)",(int)g_polygon_edit_verts.size()),
                 x,y,12,C_WHITE); y+=15;
        int bpoly_w=(pw-48)/2;
        bool canClose = (int)g_polygon_edit_verts.size() >= 3;
        if(DrawBtn({(float)x,(float)y,(float)bpoly_w,28},"Close & Save",{30,120,60,255},canClose)){
            g_segment_polygon      = g_polygon_edit_verts;
            g_segment_polygon_valid = true;
            save_segment_polygon("segment_polygon.cfg");
            g_polygon_edit_mode = false;
            g_polygon_edit_verts.clear();
            if(g_pp_goal.valid) pp_trigger_plan();
        }
        if(DrawBtn({(float)(x+bpoly_w+16),(float)y,(float)bpoly_w,28},"Cancel",{80,40,40,255},true)){
            g_polygon_edit_mode = false;
            g_polygon_edit_verts.clear();
        }
        y+=34;
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),24},"Undo Last Vertex",{60,50,80,255},
                   !g_polygon_edit_verts.empty()))
            g_polygon_edit_verts.pop_back();
        y+=32;
    }
#endif // Corridor polygon panel
    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ End corridor polygon section (June 22, 2026) ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬

    // Smoothing toggle — ON = Laplacian+Catmull, OFF = raw recorded points.
    {
        bool sm = g_path_smoothing_enabled.load();
        const char* sl = sm ? "Smoothing: ON  (click to disable)"
                            : "Smoothing: OFF  (click to enable)";
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),26}, sl, sm?C_GOOD:C_WARN, true)){
            g_path_smoothing_enabled.store(!sm);
            // Re-run pipeline so the change takes effect immediately if a path exists.
            if(hasPath){ pp_run_pipeline(TRAJ_CSV_FILE); if(g_pp_goal.valid) pp_trigger_plan(); }
        }
        y+=32;
    }

    // Path overlay toggle — hide the reference-path corridor curve (may be stale across maps).
    {
        bool ov = g_show_path_overlay.load();
        const char* ol = ov ? "Path overlay: ON  (click to hide)"
                            : "Path overlay: OFF  (click to show)";
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),26}, ol, ov?C_GOOD:C_WARN, true))
            g_show_path_overlay.store(!ov);
        y+=32;
    }

    // "Build Path from Trajectory" button HIDDEN 2026-07-10 — redundant: pp_run_pipeline()
    // already checks the per-map recorded path first and is already auto-called on zone
    // arming, every "Go" destination button, map switch, and entering Step 2. This button
    // only ever mattered for the legacy raw trajectory_data.csv fallback (old Lab/Full
    // Campus maps with no per-map path file), which those auto-calls already cover too.
    // Re-enable by uncommenting if a manual re-trigger is ever needed.
    // Color btnCol1={180,100,20,255};
    // if(DrawBtn({(float)x,(float)y,(float)(pw-32),34},"Build Path from Trajectory",btnCol1,true)){
    //     pp_run_pipeline(TRAJ_CSV_FILE); pp_trigger_plan(); }
    // y+=42;

    bool canSave=(hasPlan&&hasGoal&&g_pp_robot.valid);
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    if(DrawBtn({(float)x,(float)(py+ph-60),(float)(pw-32),42},
               canSave?"Confirm Path  >>  Start Drive":"Build a valid path first...",
               canSave?C_GOOD:Color{60,60,60,200}, canSave)){
        pp_save_plan(); ctrl_doLoad();
        g_drive_via_passenger=false; g_ui_step.store(STEP_DRIVE);
    }
}

// =============================================================================
//  RIGHT PANEL ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â Step 3: Drive
// =============================================================================
void DrawPanelDrive(int px, int py, int pw, int ph) {
    DrawRectangle(px,py,pw,ph,C_PANEL_DARK);
    DrawRectangle(px,py,1,ph,C_BORDER_HI);
    int x=px+16, y=py+16;
    DrawText("03  DRIVE",x,y,18,C_GOOD); y+=28;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=12;

    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ navigation buttons ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
    {
        int nav_bw=(pw-48)/2;
        if(DrawBtn({(float)(px+16),(float)(py+56),(float)nav_bw,26},"<< Localize",{50,50,90,255},true))
            g_ui_step.store(STEP_LOCALIZE);
        if(DrawBtn({(float)(px+16+nav_bw+16),(float)(py+56),(float)nav_bw,26},"<< Plan",{50,70,90,255},true))
            g_ui_step.store(STEP_PLAN);
    }
    y+=50;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    const char* cs=""; Color cc=C_DIM;
    switch(g_ctrl_state){
        case CS_IDLE:    cs="IDLE";          cc=C_DIM;    break;
        case CS_LOADED:  cs="LOADED";        cc=C_HEADER; break;
        case CS_RUNNING: cs="RUNNING";       cc=C_GOOD;   break;
        case CS_PAUSED:  cs="PAUSED";        cc=C_WARN;   break;
        case CS_DONE:    cs="GOAL REACHED";   cc=C_GOOD;   break;
        case CS_WATCHDOG:cs="WATCHDOG TRIP";  cc=C_BAD;    break;
    }
    DrawRectangleRounded({(float)x,(float)y,(float)(pw-32),26},0.3f,6,Fade(cc,0.15f));
    DrawRectangleRoundedLines({(float)x,(float)y,(float)(pw-32),26},0.3f,6,1,cc);
    int sw2=MeasureText(cs,13); DrawText(cs,x+(pw-32)/2-sw2/2,y+6,13,cc); y+=34;

    DrawText(g_statusMsg.c_str(),x,y,11,g_statusCol); y+=18;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;

    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ Pre-drive readiness indicator ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
    // Thresholds: LiDAR VLP-16 = 10 Hz ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â¢ require >5. RMSE_FULL_TRUST=0.20 ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â¢ use 0.30.
    // Hysteresis: require 45 consecutive frames (~0.75 s) before flipping state.
    {
        static int  s_lidar_hz=0, s_imu_hz=0;
        static int  s_last_sweep=0, s_last_imu=0;
        static float s_hz_timer=0.f;
        static bool s_ready_shown=false;
        static int  s_ready_frames=0, s_notready_frames=0;

        s_hz_timer += GetFrameTime();
        if(s_hz_timer >= 1.f){
            int cs=lidar_sweeps_total.load(), ci=imu_frames_total.load();
            s_lidar_hz=cs-s_last_sweep; s_imu_hz=ci-s_last_imu;
            s_last_sweep=cs; s_last_imu=ci; s_hz_timer=0.f;
        }
        float rmse=current_rmse.load();
        bool c_lidar  = s_lidar_hz >  5;    // VLP-16 = 10 Hz; was >15 which can never pass
        bool c_imu    = s_imu_hz   > 50;    // 100 Hz target; >50 = healthy
        bool c_rmse   = rmse       < 0.30f; // RMSE_FULL_TRUST=0.20; was 0.15 which flickered
        bool c_loaded = (g_ctrl_state==CS_LOADED||g_ctrl_state==CS_PAUSED);
        bool c_real   = !g_simMode;
        bool c_goal   = (g_ctrl.distToGoal > 1.0f);
        // Stale-plan guard: the loaded plan's saved start pose must still match where
        // the cart actually is. Re-localizing without re-confirming the destination
        // leaves g_ctrl_path/g_ctrl_start built from the OLD position — the plan looks
        // like it "skips" the road because it's a route from a place you're not at
        // anymore. Block READY (not just visually warn) so this can't be missed.
        static const float STALE_PLAN_M = 3.0f;
        float pose_mismatch_m = 0.f;
        bool c_pose_match = true;
        { std::lock_guard<std::mutex> lk_e(ekf_mutex);
          if(ekf.seeded && g_ctrl_start.valid){
              float pdx=(float)ekf.x[0]-g_ctrl_start.x, pdy=(float)ekf.x[1]-g_ctrl_start.y;
              pose_mismatch_m = sqrtf(pdx*pdx+pdy*pdy);
              c_pose_match = pose_mismatch_m <= STALE_PLAN_M;
          }
        }
        bool raw_ready = c_lidar && c_imu && c_rmse && c_loaded && c_real && c_goal && c_pose_match;

        // Hysteresis: flip to READY only after 45 stable frames, revert after 45 not-ready frames
        if(raw_ready){ s_ready_frames++; s_notready_frames=0; }
        else         { s_notready_frames++; s_ready_frames=0; }
        if(s_ready_frames    >= 45) s_ready_shown=true;
        if(s_notready_frames >= 45) s_ready_shown=false;

        if(s_ready_shown){
            DrawRectangleRounded({(float)x,(float)y,(float)(pw-32),20},0.3f,6,Fade(C_GOOD,0.15f));
            int tw=MeasureText("READY TO DRIVE",13);
            DrawText("READY TO DRIVE",x+(pw-32)/2-tw/2,y+3,13,C_GOOD);
        } else {
            DrawRectangleRounded({(float)x,(float)y,(float)(pw-32),20},0.3f,6,Fade(C_BAD,0.12f));
            int tw=MeasureText("NOT READY:",13);
            DrawText("NOT READY:",x,y+3,13,C_BAD);
            int fx=x+tw+6, fy=y+3;
            if(!c_lidar)  { DrawText(TextFormat(" LiDAR %dHz",s_lidar_hz),fx,fy,11,C_BAD); fx+=MeasureText(TextFormat(" LiDAR %dHz",s_lidar_hz),11)+4; }
            if(!c_imu)    { DrawText(TextFormat(" IMU %dHz",  s_imu_hz),  fx,fy,11,C_BAD); fx+=MeasureText(TextFormat(" IMU %dHz",  s_imu_hz),  11)+4; }
            if(!c_rmse)   { DrawText(TextFormat(" RMSE %.2f", rmse),      fx,fy,11,C_BAD); fx+=MeasureText(TextFormat(" RMSE %.2f",rmse),        11)+4; }
            if(!c_loaded) { DrawText(" No plan",                          fx,fy,11,C_BAD); fx+=MeasureText(" No plan",                           11)+4; }
            if(!c_real)   { DrawText(" SimMode",                          fx,fy,11,C_BAD); fx+=MeasureText(" SimMode",                           11)+4; }
            if(!c_goal)   { DrawText(" At goal",                          fx,fy,11,C_BAD); fx+=MeasureText(" At goal",                            11)+4; }
            if(!c_pose_match){
                const char* msg=TextFormat(" PLAN STALE (%.1fm) - re-confirm destination", pose_mismatch_m);
                DrawText(msg, fx, fy, 11, C_BAD);
            }
        }
        y+=26;
    }
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;

    CtrlPose& robot=g_simMode?g_ctrl_sim:g_ctrl_robot_filtered;
    if(robot.valid){
        DrawText(TextFormat("Pos:   %.2f, %.2f m",robot.x,robot.y),x,y,13,C_WHITE); y+=16;
        DrawText(TextFormat("Yaw:   %.1f deg",toDeg(robot.yaw)),x,y,13,C_WHITE); y+=16;
    }
    if(g_confirmed_start.valid){
        DrawText(TextFormat("Start: %.2f, %.2f  %.1f deg",
                 g_confirmed_start.x,g_confirmed_start.y,
                 g_confirmed_start.yaw*180.f/kPI),x,y,12,C_HEADER); y+=15;
    }
    DrawText(TextFormat("Steer: %.2f deg",g_ctrl.cmdSteer),x,y,13,C_WARN); y+=16;
    DrawText(TextFormat("Steer real:%.2f deg%s",g_ctrl.thetaActual,
             g_ctrl.thetaValid?" [FB]":" [--]"),x,y,13,
             g_ctrl.thetaValid?C_GOOD:C_DIM); y+=16;
    DrawText(TextFormat("Speed: %.2f V",g_ctrl.cmdSpeed),x,y,13,C_WARN); y+=16;
    DrawText(TextFormat("CTE:   %.3f m",g_ctrl.crossTrackErr),x,y,13,
             fabsf(g_ctrl.crossTrackErr)>CORRIDOR_HALF_M*0.7f?C_BAD:C_WHITE); y+=16;
    DrawText(TextFormat("Dist:  %.2f m to goal",g_ctrl.distToGoal),x,y,13,C_WHITE); y+=16;
    DrawText(TextFormat("LA:    %.2f m",g_lookaheadM),x,y,13,C_HEADER); y+=16;
    if(g_dr_active){
        DrawText(TextFormat("DANGER ZONE - dead reckoning (%.1fm in)",g_dr_s),x,y,13,C_WARN); y+=16;
    }
    if(g_dzsub_active){
        DrawText(TextFormat("SUB-ZONE - dead reckoning, curve replay (%.1fm in)",g_dzsub_s),x,y,13,C_WARN); y+=16;
    }

    {
        static const int STARTUP_IMU_TICKS=50;
        float _yaw_gap_deg=0.f;
        if(g_imu_drive_ref_valid){
            float imu_deg_d;
            {std::lock_guard<std::mutex> lkd(imu_mutex); imu_deg_d=imu_angle.z;}
            float delta_d=wrapA((imu_deg_d-g_imu_yaw_drive_start_deg)*(kPI/180.f));
            float imu_w=wrapA(g_ekf_yaw_drive_start_rad+delta_d);
            _yaw_gap_deg=toDeg(fabsf(wrapA(imu_w-g_ctrl_robot_filtered.yaw)));
        }
        Color _gc=_yaw_gap_deg<1.0f?C_GOOD:_yaw_gap_deg<3.0f?C_WARN:C_BAD;
        DrawText(TextFormat("Yaw gap:  %.1f deg%s",_yaw_gap_deg,
                            _yaw_gap_deg<1.0f?" [OK]":" [waiting]"),x,y,13,_gc); y+=14;
        if(g_ctrl.startupTick<=STARTUP_IMU_TICKS&&g_ctrl_state==CS_RUNNING){
            int ticks_left=STARTUP_IMU_TICKS-g_ctrl.startupTick;
            DrawText(TextFormat("Startup: IMU snap  (%d ticks left)",ticks_left),x,y,12,C_WARN);
        } else if(g_ctrl_state==CS_RUNNING){
            DrawText("Startup: done  [cruise blend]",x,y,12,C_GOOD);
        }
        y+=16;
    }
    {
        double _tl=g_lidar_sweep_time.load();
        float _age=(_tl>0.0)?clampf((float)(nowSecEpoch()-_tl),0.f,0.99f):0.f;
        Color _ac=_age<0.08f?C_GOOD:_age<0.20f?C_WARN:C_BAD;
        DrawText(TextFormat("Pose age: %.0f ms",_age*1000.f),x,y,13,_ac); y+=16;
        Color _pc=g_dbg_total_pred_ms<40.f?C_GOOD:g_dbg_total_pred_ms<80.f?C_WARN:C_BAD;
        DrawText(TextFormat("Pred:  %.0f ms  (LiDAR %.0f + Act %.0f)",
                            g_dbg_total_pred_ms,g_dbg_dt_lidar_ms,g_dbg_dt_actuator_ms),
                 x,y,12,_pc); y+=20;
    }

    Color modeCol=g_simMode?C_HEADER:C_GOOD;
    DrawText(g_simMode?"MODE: SIMULATION":"MODE: REAL UDP",x,y,13,modeCol); y+=22;
    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=10;

    int bw=(pw-48)/2;
    bool isRunning=(g_ctrl_state==CS_RUNNING);
    bool canStart=(g_ctrl_state==CS_LOADED||g_ctrl_state==CS_PAUSED);
    Color c_start={30,140,60,255}, c_stop={160,40,40,255};
    if(isRunning){ if(DrawBtn({(float)x,(float)y,(float)(pw-32),36},"STOP",c_stop,true)) ctrl_doStop(); }
    else {
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),36},g_simMode?"Start Simulation":"Start Autonomous",c_start,canStart)){
            // Re-plan approach from actual current pose so position/heading variation
            // between runs is handled automatically by the Bezier approach generator.
            if(!g_simMode && g_pp_goal.valid){
                {std::lock_guard<std::mutex> lk(ekf_mutex);
                 g_pp_robot={(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],true};}
                g_confirmed_start={g_pp_robot.x,g_pp_robot.y,g_pp_robot.yaw,true};
                pp_trigger_plan();
                pp_save_plan();
                ctrl_doLoad();
            }
            ctrl_doStart();
        }
    }
    y+=44;

    Color c_reset={100,60,20,255}, c_load={30,60,120,255};
    if(DrawBtn({(float)x,(float)y,(float)bw,32},"Reset",c_reset,true)) ctrl_doReset();
    if(DrawBtn({(float)(x+bw+16),(float)y,(float)bw,32},"Load Plan",c_load,true)) ctrl_doLoad();
    y+=40;

    // ── Camera pedestrian detection (2026-07-08, test feature) ──
    {
        bool pedOn = g_ped_enabled.load();
        Color pedCol = pedOn ? Color{30,120,90,255} : Color{70,70,90,255};
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),30},
                   pedOn ? "Pedestrian Detection: ON  (click to disable)"
                         : "Pedestrian Detection: OFF  (click to enable)",
                   pedCol, true)){
            ped_set_enabled(!pedOn);
        }
        y+=34;
        if(pedOn){
            // Live status line
            if(g_ped_stale.load()){
                DrawText("  detector not reporting - is the camera window running?", x, y, 11, C_WARN);
            } else if(g_ped_detected.load()){
                DrawText("  PERSON IN ZONE - cart will stop", x, y, 12, C_BAD);
            } else {
                DrawText("  clear - no person in stop zone", x, y, 11, C_GOOD);
            }
            y+=16;
        }
    }
    // Resume button: only when paused BY a pedestrian (camera OR obstacle-grid cell)
    // AND whichever triggered it has cleared. Deliberately OUTSIDE the "pedOn" block
    // above — a grid-triggered pause must still show Resume even if the camera
    // detector was never turned on (they're independent triggers, same pause/resume
    // flow — see control.cpp).
    if(g_ped_paused.load()){
        bool stillBlocked = (g_ped_enabled.load() && g_ped_detected.load()) || g_grid_ped_stop.load();
        if(stillBlocked){
            DrawText(g_grid_ped_stop.load() ? "  waiting - still near an obstacle-grid pedestrian cell..."
                                             : "  waiting for person to clear...", x, y, 12, C_WARN);
            y+=18;
        } else {
            if(DrawBtn({(float)x,(float)y,(float)(pw-32),34},
                       "Person cleared - RESUME", {30,150,70,255}, true)){
                ctrl_doStart();   // resumes from CS_PAUSED (preserves danger-zone state)
            }
            y+=38;
        }
    }

    // New Journey button — shown when goal reached
    if(g_ctrl_state==CS_DONE){
        if(DrawBtn({(float)x,(float)y,(float)(pw-32),38},
                   "New Journey  >>  Select Destination",
                   {50,140,200,255}, true)){
            ctrl_doReset();
            g_selected_dest  = -1;
            g_show_dest      = false;
            g_show_intro     = true;
            g_ui_step.store(STEP_LOCALIZE);
        }
        y+=46;
    }



    DrawRectangle(x,y,pw-32,1,C_BORDER); y+=8;
    DrawText("COMMAND LOG",x,y,12,C_DIM); y+=14;
    { std::vector<std::string> log_snap;
      { std::lock_guard<std::mutex> lk(g_cmdLog_mutex);
        log_snap.assign(g_cmdLog.begin(), g_cmdLog.end()); }
    for(int i=0;i<(int)log_snap.size()&&y<py+ph-16;i++){
        const std::string& cmd=log_snap[i];
        Color c=C_DIM;
        if(cmd.find("STEER")!=std::string::npos)      c=C_WARN;
        else if(cmd.find("SPEED")!=std::string::npos) c=C_HEADER;
        else if(cmd.find("ESTOP")!=std::string::npos||cmd.find("BRAKE")!=std::string::npos) c=C_BAD;
        else if(cmd.find("AUTO")!=std::string::npos)  c=C_GOOD;
        if(i==0) c=C_WHITE;
        DrawText(cmd.c_str(),x,y,11,c); y+=13;
    } }

    DrawRectangle(px,py+ph-42,pw,42,{60,10,10,200});
    DrawRectangle(px,py+ph-42,pw,1,C_BAD);
    DrawText("! CRITICAL: Keep LOCALIZE running in Step 1",px+8,py+ph-38,11,C_BAD);
    DrawText("  It provides live pose updates for control.",px+8,py+ph-24,11,C_BAD);
}

// =============================================================================
//  STEP PROGRESS BAR  ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â  fully clickable nodes + >> skip arrows
//  Works at any time even if hardware is not connected.
// =============================================================================
void DrawStepBar(int y, int sw) {
    DrawRectangle(0,y,sw,50,C_PANEL_DARK);
    DrawRectangle(0,y,sw,1,C_BORDER_HI);

    const char* labels[]={"1  LOCALIZE","2  PLAN ROUTE","3  DRIVE"};
    int spacing=sw/4;
    int nodeX[3]={spacing,spacing*2,spacing*3};
    int ny=y+24;
    int cur=g_ui_step.load();
    Vector2 mouse=GetMousePosition();
    bool clicked=IsMouseButtonReleased(MOUSE_LEFT_BUTTON);

    // connector lines
    for(int i=0;i<2;i++){
        Color lc=(cur>i+1)?C_GOOD:C_BORDER;
        DrawRectangle(nodeX[i]+18,ny-1,nodeX[i+1]-nodeX[i]-36,3,lc);
    }

    // >> skip arrows between nodes
    for(int i=0;i<2;i++){
        int ax=(nodeX[i]+nodeX[i+1])/2;
        Rectangle arrowR={(float)(ax-18),(float)(ny-10),36,20};
        bool hov=CheckCollisionPointRec(mouse,arrowR);
        DrawRectangleRounded(arrowR,0.4f,4,hov?Color{40,80,130,210}:Color{20,25,38,180});
        DrawRectangleRoundedLines(arrowR,0.4f,4,1.f,hov?C_BORDER_HI:C_BORDER);
        DrawText(">>",ax-10,ny-7,13,hov?WHITE:C_DIM);
        if(hov&&clicked) g_ui_step.store(i+2);
    }

    // step nodes ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â click any to jump directly
    for(int i=0;i<3;i++){
        int step=i+1;
        bool isCur=(step==cur), isDone=(step<cur);
        bool hov=CheckCollisionPointRec(mouse,{(float)(nodeX[i]-16),(float)(ny-16),32,32});
        Color border=isDone?C_GOOD:(isCur?C_HEADER:(hov?C_WARN:C_BORDER));

        DrawCircle(nodeX[i],ny,14,hov?Color{40,60,100,200}:C_BG);
        DrawCircleLines(nodeX[i],ny,14,border);
        DrawCircleLines(nodeX[i],ny,13,border);

        const char* num=(step==1?"1":step==2?"2":"3");
        if(isDone)     DrawText("v",  nodeX[i]-4,ny-8,14,C_GOOD);
        else if(isCur) DrawText(num,  nodeX[i]-4,ny-8,14,C_HEADER);
        else           DrawText(num,  nodeX[i]-4,ny-8,14,hov?C_WARN:C_BORDER);

        Color lc=isDone?C_GOOD:(isCur?C_HEADER:(hov?C_WARN:C_DIM));
        int tw=MeasureText(labels[i],12);
        DrawText(labels[i],nodeX[i]-tw/2,ny+16,12,lc);

        // jump on click
        if(hov&&clicked&&!isCur) g_ui_step.store(step);

        // hover tooltip
        if(hov&&!isCur){
            const char* tip=TextFormat("Jump to Step %d",step);
            int tipW=MeasureText(tip,11)+12;
            int tx=nodeX[i]-tipW/2, tyy=y-22;
            DrawRectangle(tx,tyy,tipW,18,{20,25,40,230});
            DrawRectangleLines(tx,tyy,tipW,18,C_BORDER_HI);
            DrawText(tip,tx+6,tyy+3,11,C_WARN);
        }
    }

    // hint text
    const char* hint="Click any step or >> to skip";
    DrawText(hint,sw-MeasureText(hint,11)-12,y+4,11,C_DIM);
}

// =============================================================================
//  METRIC GRID OVERLAY ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â 3D ground plane (Step 1)
// =============================================================================
static void DrawGrid3D(const Camera3D& camera, bool large)
{
    float spacing = large ? 10.f : 5.f;
    float extent  = large ? 80.f : 40.f;
    const Color gridCol = {255,255,255,18};
    const Color axisCol = {255,255,255,60};

    // Snap grid origin to grid spacing so lines don't swim as camera moves
    float cx = floorf(camera.target.x / spacing) * spacing;
    float cz = floorf(camera.target.z / spacing) * spacing;

    // Vertical lines (constant Raylib-X = world-X)
    for(float x = cx-extent; x <= cx+extent+0.01f; x += spacing){
        bool axis = (fabsf(x) < spacing*0.01f);
        DrawLine3D({x,0,cz-extent},{x,0,cz+extent}, axis?axisCol:gridCol);
    }
    // Horizontal lines (constant Raylib-Z = -world-Y)
    for(float z = cz-extent; z <= cz+extent+0.01f; z += spacing){
        bool axis = (fabsf(z) < spacing*0.01f);
        DrawLine3D({cx-extent,0,z},{cx+extent,0,z}, axis?axisCol:gridCol);
    }
}

// Labels for the 3D grid ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â called AFTER EndMode3D using GetWorldToScreen
static void DrawGrid3DLabels(const Camera3D& camera, bool large, int mapW, int mapH, int TITLE_H)
{
    float spacing = large ? 10.f : 5.f;
    float extent  = large ? 80.f : 40.f;
    const Color labelCol = {150,150,150,180};
    const int   fs       = 10;

    float cx = floorf(camera.target.x / spacing) * spacing;
    float cz = floorf(camera.target.z / spacing) * spacing;

    // Label X-axis: project a point on each vertical line near the camera target Z
    for(float x = cx-extent; x <= cx+extent+0.01f; x += spacing){
        Vector2 sp = GetWorldToScreen({x, 0, camera.target.z}, camera);
        if(sp.x < 4 || sp.x > mapW-4 || sp.y < TITLE_H+4 || sp.y > mapH-4) continue;
        const char* lbl = TextFormat("%.0f", x);
        int tw = MeasureText(lbl, fs);
        DrawText(lbl, (int)(sp.x - tw*0.5f), (int)(sp.y + 3), fs, labelCol);
    }
    // Label Y-axis: world-Y = -Raylib-Z, project near camera target X
    for(float z = cz-extent; z <= cz+extent+0.01f; z += spacing){
        Vector2 sp = GetWorldToScreen({camera.target.x, 0, z}, camera);
        if(sp.x < 4 || sp.x > mapW-4 || sp.y < TITLE_H+4 || sp.y > mapH-4) continue;
        const char* lbl = TextFormat("%.0f", -z);   // world-Y = -raylib-Z
        DrawText(lbl, (int)(sp.x + 4), (int)(sp.y - fs*0.5f), fs, labelCol);
    }
}

// =============================================================================
//  METRIC GRID OVERLAY ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â drawn behind map cloud in 2D steps
// =============================================================================
static void DrawGrid2D(const View2D& v, int mapW, int mapH, int TITLE_H)
{
    // Adaptive spacing: keep roughly 6ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œ12 lines visible
    float world_span = mapW / v.scale;          // meters across viewport
    float raw = world_span / 8.f;               // target ~8 divisions
    // Round to a clean metric step: 1, 2, 5, 10, 20, 50 ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¦
    float steps[] = {1,2,5,10,20,50,100,200,500};
    float spacing = steps[8];
    for(float s : steps){ if(s >= raw*0.7f){ spacing = s; break; } }

    const Color gridCol  = {255,255,255,18};    // very faint white grid
    const Color axisCol  = {255,255,255,55};    // slightly brighter axis lines
    const Color labelCol = {160,160,160,200};
    const int   fs       = 10;

    // Visible world bounds
    float wx0 = v.cx - (mapW*0.5f)/v.scale;
    float wx1 = v.cx + (mapW*0.5f)/v.scale;
    float wy0 = v.cy - (mapH*0.5f)/v.scale;
    float wy1 = v.cy + (mapH*0.5f)/v.scale;

    float gx = floorf(wx0/spacing)*spacing;
    float gy = floorf(wy0/spacing)*spacing;

    // Vertical lines (constant world-X)
    for(float wx = gx; wx <= wx1+spacing; wx += spacing){
        Vector2 top = w2s(wx, wy1, v);
        Vector2 bot = w2s(wx, wy0, v);
        top.y = fmaxf(top.y, (float)TITLE_H);
        bot.y = fminf(bot.y, (float)mapH-1);
        if(top.x < 0 || top.x > mapW) continue;
        bool isAxis = (fabsf(wx) < spacing*0.01f);
        DrawLineV(top, bot, isAxis ? axisCol : gridCol);
        // Label at bottom edge
        const char* lbl = TextFormat("%.0f", wx);
        int tw = MeasureText(lbl, fs);
        float lx = top.x - tw*0.5f;
        if(lx > 2 && lx+tw < mapW-2)
            DrawText(lbl, (int)lx, mapH-16, fs, labelCol);
    }
    // Horizontal lines (constant world-Y)
    for(float wy = gy; wy <= wy1+spacing; wy += spacing){
        Vector2 left  = w2s(wx0, wy, v);
        Vector2 right = w2s(wx1, wy, v);
        left.x  = fmaxf(left.x,  0.f);
        right.x = fminf(right.x, (float)mapW-1);
        if(left.y < TITLE_H || left.y > mapH) continue;
        bool isAxis = (fabsf(wy) < spacing*0.01f);
        DrawLineV(left, right, isAxis ? axisCol : gridCol);
        // Label at left edge
        const char* lbl = TextFormat("%.0f", wy);
        float ly = left.y - fs*0.5f;
        if(ly > TITLE_H+2 && ly+fs < mapH-2)
            DrawText(lbl, 4, (int)ly, fs, labelCol);
    }
    // Scale bar (bottom-right corner, 100px wide)
    {
        float bar_m = 100.f / v.scale;                 // real-world metres per 100px
        // round to nearest clean value
        float bsteps[] = {0.5f,1,2,5,10,20,50,100,200,500};
        float blen = bsteps[9];
        for(float s : bsteps){ if(s >= bar_m*0.4f){ blen=s; break; } }
        float bpx = blen * v.scale;
        int bx = mapW - (int)bpx - 10;
        int by = mapH - 28;
        DrawRectangle(bx, by+8, (int)bpx, 2, labelCol);
        DrawRectangle(bx, by+4, 2, 10, labelCol);
        DrawRectangle(bx+(int)bpx-2, by+4, 2, 10, labelCol);
        const char* slbl = blen >= 1 ? TextFormat("%.0f m", blen) : TextFormat("%.1f m", blen);
        int stw = MeasureText(slbl, fs);
        DrawText(slbl, bx+(int)(bpx*0.5f)-stw/2, by-2, fs, labelCol);
    }
}

// =============================================================================
//  MAP VIEWPORT RENDERING (2D/3D scene)
// =============================================================================
void DrawMapViewport(int mapW, int mapH, int TITLE_H, int PANEL_W, int BAR_H,
                     int step, Camera3D& camera, View2D& view2d, bool camera_follow,
                     AppState fus_state)
{
    if(step==STEP_LOCALIZE){
        BeginMode3D(camera);
        bool large_now=active_map_ptr&&active_map_ptr->is_large;
        DrawGrid3D(camera, large_now);
        if(g_show_range_rings.load()){
            Vector3 ring_center=camera_follow?camera.target:Vector3{0,0,0};
            DrawRangeRings(ring_center,large_now);
        }

        if(active_map_ptr&&active_map_ptr->loaded)
            for(size_t i=0;i<active_map_ptr->cloud.size();i+=1){
                float mz=active_map_ptr->cloud[i].z;
                float nh=Clamp((mz+0.3f)/2.5f,0.f,1.f); // 0=floor(blue) 1=ceiling(red)
                DrawPoint3D({active_map_ptr->cloud[i].x,mz,-active_map_ptr->cloud[i].y},
                            ColorFromHSV((1.f-nh)*200.f,0.85f,1.f));
            }

        {std::lock_guard<std::mutex>lk(trajectory_mutex);
         if(robot_trajectory.size()>1)
             for(size_t i=1;i<robot_trajectory.size();i++)
                 DrawLine3D(robot_trajectory[i-1],robot_trajectory[i],MAGENTA);}

        double ex3=0,ey3=0,eyaw3=0;
        if(fus_state==STATE_FUSION_RUNNING){std::lock_guard<std::mutex>lk(ekf_mutex);ex3=ekf.x[0];ey3=ekf.x[1];eyaw3=ekf.x[2];}
        else if(fus_state==STATE_GLOBAL_RELOC){std::lock_guard<std::mutex>lk(g_reloc_mutex);if(!g_reloc_particles.empty()){ex3=g_reloc_particles[0].x;ey3=g_reloc_particles[0].y;eyaw3=g_reloc_particles[0].yaw;}}
        float cy3=cosf((float)eyaw3),sy3=sinf((float)eyaw3);

        {std::lock_guard<std::mutex>lk(lidar_mutex);
         for(const auto& p:dense_cloud){
             float wx=cy3*p.x-sy3*p.y+(float)ex3, wy=sy3*p.x+cy3*p.y+(float)ey3;
             float nh=Clamp((p.z+2)/5,0,1);
             DrawPoint3D({wx,p.z,-wy},ColorFromHSV((1-nh)*240,1,1));
         }}

        if(fus_state==STATE_GLOBAL_RELOC){
            // Snapshot under the lock (fast), then draw WITHOUT holding it — holding
            // g_reloc_mutex through thousands of GPU draws stalls the reloc thread,
            // which needs the same lock every iteration to publish its particles.
            static std::vector<Particle> snap;
            { std::lock_guard<std::mutex>lk(g_reloc_mutex); snap = g_reloc_particles; }
            for(size_t i=0;i<snap.size();i+=4)   // thin: 1/4 of particles reads the same
                DrawPoint3D({snap[i].x,0.2f,-snap[i].y},{0,255,0,100});
        }

        if(fus_state==STATE_FUSION_RUNNING){
            Vector3 rp={(float)ex3,0.0f,-(float)ey3};
            float alen=2.0f, ahead=0.5f;
            Vector3 shaft_end={rp.x-sy3*alen,0.0f,rp.z-cy3*alen};
            Vector3 arrow_tip={rp.x-sy3*(alen+ahead),0.3f,rp.z-cy3*(alen+ahead)};
            DrawCylinderEx(rp,shaft_end,0.12f,0.12f,8,YELLOW);
            DrawCylinderEx(shaft_end,arrow_tip,0.30f,0.0f,8,YELLOW);
            DrawSphere(rp,0.18f,YELLOW);
        }
        EndMode3D();
        DrawGrid3DLabels(camera, large_now, mapW, mapH, TITLE_H);
    }

    if(step==STEP_PLAN||step==STEP_DRIVE){
        DrawGrid2D(view2d, mapW, mapH, TITLE_H);

        // ── Zone 1 polygon (saved + in-progress while drawing) ──
        {
            Color zcol = g_zone_armed ? C_GOOD : Color{90,140,200,255};
            if(g_zone_valid && g_zone_poly.size()>=3){
                int n=(int)g_zone_poly.size();
                for(int i=0;i<n;i++){
                    Vector2 a=w2s(g_zone_poly[i].x,g_zone_poly[i].y,view2d);
                    Vector2 b=w2s(g_zone_poly[(i+1)%n].x,g_zone_poly[(i+1)%n].y,view2d);
                    DrawLineEx(a,b,1.5f,Fade(zcol,0.8f));
                }
            }
            if(g_zone_edit_mode){
                int n=(int)g_zone_edit_verts.size();
                for(int i=0;i<n;i++){
                    Vector2 a=w2s(g_zone_edit_verts[i].x,g_zone_edit_verts[i].y,view2d);
                    DrawCircleV(a,4.f,C_WARN);
                    if(i>0) DrawLineEx(w2s(g_zone_edit_verts[i-1].x,g_zone_edit_verts[i-1].y,view2d),a,1.5f,C_WARN);
                }
                if(n>=3) DrawLineEx(w2s(g_zone_edit_verts[n-1].x,g_zone_edit_verts[n-1].y,view2d),
                                    w2s(g_zone_edit_verts[0].x,g_zone_edit_verts[0].y,view2d),1.f,Fade(C_WARN,0.4f));
            }
            // estimated start marker (a small circle + heading tick)
            if(g_est_start.valid){
                Vector2 sp=w2s(g_est_start.x,g_est_start.y,view2d);
                DrawCircleLines((int)sp.x,(int)sp.y,7,Color{120,200,120,255});
                DrawCircleV(sp,3,Color{120,200,120,255});
                DrawLineEx(sp,{sp.x-sinf(g_est_start.yaw)*16.f, sp.y-cosf(g_est_start.yaw)*16.f},2.f,Color{120,200,120,255});
            }
        }

        // ── Zone 2 polygon (saved + in-progress while drawing) ──
        {
            Color zcol2 = g_zone2_armed ? C_GOOD : Color{200,140,70,255};
            if(g_zone2_valid && g_zone2_poly.size()>=3){
                int n=(int)g_zone2_poly.size();
                for(int i=0;i<n;i++){
                    Vector2 a=w2s(g_zone2_poly[i].x,g_zone2_poly[i].y,view2d);
                    Vector2 b=w2s(g_zone2_poly[(i+1)%n].x,g_zone2_poly[(i+1)%n].y,view2d);
                    DrawLineEx(a,b,1.5f,Fade(zcol2,0.8f));
                }
            }
            if(g_zone2_edit_mode){
                int n=(int)g_zone2_edit_verts.size();
                for(int i=0;i<n;i++){
                    Vector2 a=w2s(g_zone2_edit_verts[i].x,g_zone2_edit_verts[i].y,view2d);
                    DrawCircleV(a,4.f,C_WARN);
                    if(i>0) DrawLineEx(w2s(g_zone2_edit_verts[i-1].x,g_zone2_edit_verts[i-1].y,view2d),a,1.5f,C_WARN);
                }
                if(n>=3) DrawLineEx(w2s(g_zone2_edit_verts[n-1].x,g_zone2_edit_verts[n-1].y,view2d),
                                    w2s(g_zone2_edit_verts[0].x,g_zone2_edit_verts[0].y,view2d),1.f,Fade(C_WARN,0.4f));
            }
        }

        // ── DANGER ZONE overlay (July 2026) — scan-matching known-bad stretch ──
        // In Step 3 (driving), draw the danger stretch AS PART OF the loaded plan
        // (g_ctrl_path[lo..hi]) so red is always a segment of the green driven line and
        // can never appear to diverge from it. Elsewhere (Step 2 planning), fall back
        // to the saved region file (pp_danger_zone_pts) which marks where the region is.
        {
            int dlo, dhi;
            if(step==STEP_DRIVE && ctrl_danger_path_range(dlo, dhi) &&
               dhi < (int)g_ctrl_path.size()){
                for(int i=dlo;i<dhi;i++){
                    Vector2 a=w2s(g_ctrl_path[i].x,g_ctrl_path[i].y,view2d);
                    Vector2 b=w2s(g_ctrl_path[i+1].x,g_ctrl_path[i+1].y,view2d);
                    DrawLineEx(a,b,4.0f,Fade(RED,0.85f));
                }
                Vector2 sp=w2s(g_ctrl_path[dlo].x,g_ctrl_path[dlo].y,view2d);
                DrawCircleLines((int)sp.x,(int)sp.y,10,RED);
                DrawText("DANGER ZONE",(int)sp.x+12,(int)sp.y-8,14,RED);
            } else {
                const std::vector<PPPoint>& dz = pp_danger_zone_pts();
                if(dz.size()>=2){
                    for(size_t i=0;i+1<dz.size();i++){
                        Vector2 a=w2s(dz[i].x,dz[i].y,view2d);
                        Vector2 b=w2s(dz[i+1].x,dz[i+1].y,view2d);
                        DrawLineEx(a,b,4.0f,Fade(RED,0.85f));
                    }
                    Vector2 sp=w2s(dz[0].x,dz[0].y,view2d);
                    DrawCircleLines((int)sp.x,(int)sp.y,10,RED);
                    DrawText("DANGER ZONE",(int)sp.x+12,(int)sp.y-8,14,RED);
                }
            }
        }

        // ── SUB-ZONE markers (Danger Zone Fwd test map only) — always visible in both
        // Step 2 and Step 3 so it's obvious where the set points actually are, not just
        // the numeric readout in the side panel. Orange (distinct from the red danger-
        // zone curve above); solid fill while g_dzsub_active (currently driving through
        // it) so it's unmistakable on screen during a run.
        // Traces the ACTUAL g_smooth points between the two markers (not a straight
        // connector between them) — a straight line here was misleading: it looked like
        // the sub-zone itself was being treated as a straight line, when the real
        // driving logic (control.cpp, g_ctrl_path between g_dzsub_lo/hi) already follows
        // the true recorded curve. This overlay now matches that.
        if(active_map_ptr && (active_map_ptr->name=="Danger Zone Fwd" || active_map_ptr->name=="Danger Zone Rev") && !g_smooth.empty()){
            int bi_e=0, bi_x=0; float bd_e=1e18f, bd_x=1e18f;
            for(size_t i=0;i<g_smooth.size();i++){
                float dxe=g_smooth[i].x-g_dzsub_entry_mk_x, dye=g_smooth[i].y-g_dzsub_entry_mk_y;
                float de=dxe*dxe+dye*dye; if(de<bd_e){bd_e=de;bi_e=(int)i;}
                float dxx=g_smooth[i].x-g_dzsub_exit_mk_x, dyx=g_smooth[i].y-g_dzsub_exit_mk_y;
                float dx2=dxx*dxx+dyx*dyx; if(dx2<bd_x){bd_x=dx2;bi_x=(int)i;}
            }
            int slo=std::min(bi_e,bi_x), shi=std::max(bi_e,bi_x);
            Color mc = g_dzsub_active ? ORANGE : Fade(ORANGE,0.75f);
            for(int i=slo;i<shi;i++){
                Vector2 a=w2s(g_smooth[i].x,g_smooth[i].y,view2d);
                Vector2 b=w2s(g_smooth[i+1].x,g_smooth[i+1].y,view2d);
                DrawLineEx(a,b,3.0f,mc);
            }
            Vector2 ep=w2s(g_dzsub_entry_mk_x,g_dzsub_entry_mk_y,view2d);
            Vector2 xp=w2s(g_dzsub_exit_mk_x, g_dzsub_exit_mk_y, view2d);
            DrawCircleV(ep,8.f,mc); DrawCircleLines((int)ep.x,(int)ep.y,8,BLACK);
            DrawCircleV(xp,8.f,mc); DrawCircleLines((int)xp.x,(int)xp.y,8,BLACK);
            DrawText("SUB-ZONE IN", (int)ep.x+12,(int)ep.y-8,13,ORANGE);
            DrawText("SUB-ZONE OUT",(int)xp.x+12,(int)xp.y-8,13,ORANGE);
        }

        // ── ZONE-2 SLOW-SEGMENT markers (top-LiDAR maps) — where the zone-2 speed
        // boost is suppressed (scan-matching failure at high speed, found on the
        // reverse map). Traces the actual zone-2 corridor between the markers, same
        // rationale as the sub-zone overlay above — a straight connector would
        // misleadingly suggest the system treats it as a straight line.
        if(active_map_ptr && (active_map_ptr->name=="Campus Top LiDAR" || active_map_ptr->name=="Campus Top (Rev)") && g_z2slow_valid){
            const std::vector<PPPoint>& z2c = pp_zone_corridor(2);
            if(!z2c.empty()){
                int bi_e2=0, bi_x2=0; float bd_e2=1e18f, bd_x2=1e18f;
                for(size_t i=0;i<z2c.size();i++){
                    float dxe=z2c[i].x-g_z2slow_entry_x, dye=z2c[i].y-g_z2slow_entry_y;
                    float de=dxe*dxe+dye*dye; if(de<bd_e2){bd_e2=de;bi_e2=(int)i;}
                    float dxx=z2c[i].x-g_z2slow_exit_x, dyx=z2c[i].y-g_z2slow_exit_y;
                    float dx2=dxx*dxx+dyx*dyx; if(dx2<bd_x2){bd_x2=dx2;bi_x2=(int)i;}
                }
                int slo2=std::min(bi_e2,bi_x2), shi2=std::max(bi_e2,bi_x2);
                for(int i=slo2;i<shi2;i++){
                    Vector2 a=w2s(z2c[i].x,z2c[i].y,view2d);
                    Vector2 b=w2s(z2c[i+1].x,z2c[i+1].y,view2d);
                    DrawLineEx(a,b,3.5f,YELLOW);
                }
                Vector2 ep2=w2s(g_z2slow_entry_x,g_z2slow_entry_y,view2d);
                Vector2 xp2=w2s(g_z2slow_exit_x, g_z2slow_exit_y, view2d);
                DrawCircleV(ep2,8.f,YELLOW); DrawCircleLines((int)ep2.x,(int)ep2.y,8,BLACK);
                DrawCircleV(xp2,8.f,YELLOW); DrawCircleLines((int)xp2.x,(int)xp2.y,8,BLACK);
                DrawText("SLOW ZONE START",(int)ep2.x+12,(int)ep2.y-8,13,YELLOW);
                DrawText("SLOW ZONE END",  (int)xp2.x+12,(int)xp2.y-8,13,YELLOW);
            }
        }

        // ── OBSTACLE GRID (2026-07-10) — blocked cells, any map with a loaded path.
        // Sparse (only clicked cells), so this stays cheap even on long paths — the
        // full empty-cell grid pattern isn't drawn (would mean an in-band check per
        // candidate cell against every g_smooth point every frame; skipped for now,
        // not needed for the cells to function). Always visible, not just in test
        // mode, so a forbidden zone stays visible while actually driving.
        {
            float cellPx = GRID_CELL_M * view2d.scale;
            for(auto& kv : g_obstacle_grid){
                float cx,cy; grid_cell_center(kv.first.first, kv.first.second, cx, cy);
                Vector2 sp = w2s(cx, cy, view2d);
                if(sp.x<-cellPx || sp.x>view2d.sw+cellPx || sp.y<-cellPx || sp.y>view2d.sh+cellPx) continue;
                Color fill = (kv.second==OBST_STATIC) ? Fade(RED,0.55f) : Fade(ORANGE,0.55f);
                Color line = (kv.second==OBST_STATIC) ? RED : ORANGE;
                Rectangle cr = {sp.x-cellPx*0.5f, sp.y-cellPx*0.5f, cellPx, cellPx};
                DrawRectangleRec(cr, fill);
                DrawRectangleLinesEx(cr, 2.f, line);
            }
        }

        // ── BORDER/WALL CELLS (2026-07-10) — auto-generated forbidden cells near the
        // corridor edge. Drawn in a distinct color (purple) from the pedestrian/static
        // obstacle grid above so the two concepts (transient test obstacles vs. the
        // persisted wall boundary) are visually unmistakable from each other.
        // Gated on g_show_border_cells (2026-07-27) — previously unconditional on
        // g_border_grid being non-empty, so a loaded/generated border set had no way
        // to be hidden again without clearing the actual data.
        if(g_show_border_cells){
            float cellPxB = GRID_CELL_M * view2d.scale;
            for(auto& kv : g_border_grid){
                if(!kv.second) continue;
                float cx,cy; grid_cell_center(kv.first.first, kv.first.second, cx, cy);
                Vector2 sp = w2s(cx, cy, view2d);
                if(sp.x<-cellPxB || sp.x>view2d.sw+cellPxB || sp.y<-cellPxB || sp.y>view2d.sh+cellPxB) continue;
                Rectangle cr = {sp.x-cellPxB*0.5f, sp.y-cellPxB*0.5f, cellPxB, cellPxB};
                DrawRectangleRec(cr, Fade(PURPLE,0.45f));
                DrawRectangleLinesEx(cr, 1.5f, PURPLE);
            }
        }

        // ── PLACE DANGER PATH preview (while the tool is active) ── the old campus
        // danger-zone curve, transformed by the current drag offset/rotation, drawn in
        // bright magenta so it's unmistakable against the orange sub-zone markers and
        // the cyan/blue map cloud. Endpoints marked so it's clear which way to drag.
        if(g_dzplace_mode && !g_dzplace_raw.empty()){
            std::vector<PPPoint> xf; dzplace_transformed_points(xf);
            for(size_t i=0;i+1<xf.size();i++){
                Vector2 a=w2s(xf[i].x,xf[i].y,view2d);
                Vector2 b=w2s(xf[i+1].x,xf[i+1].y,view2d);
                DrawLineEx(a,b,4.0f,MAGENTA);
            }
            Vector2 p0=w2s(xf.front().x,xf.front().y,view2d);
            Vector2 p1=w2s(xf.back().x, xf.back().y, view2d);
            DrawCircleV(p0,7.f,MAGENTA); DrawCircleLines((int)p0.x,(int)p0.y,7,BLACK);
            DrawCircleV(p1,7.f,MAGENTA); DrawCircleLines((int)p1.x,(int)p1.y,7,BLACK);
            DrawText("PATH START",(int)p0.x+10,(int)p0.y-8,13,MAGENTA);
            DrawText("PATH END",  (int)p1.x+10,(int)p1.y-8,13,MAGENTA);
        }

        if(active_map_ptr&&active_map_ptr->loaded){
            // World-space cull first ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â avoids w2s transform for off-screen points.
            float margin = 5.f / view2d.scale;
            float wx0 = view2d.cx - (mapW*0.5f)/view2d.scale - margin;
            float wx1 = view2d.cx + (mapW*0.5f)/view2d.scale + margin;
            float wy0 = view2d.cy - (mapH*0.5f)/view2d.scale - margin;
            float wy1 = view2d.cy + (mapH*0.5f)/view2d.scale + margin;
            for(size_t i=0;i<active_map_ptr->cloud.size();i+=5){
                float px=active_map_ptr->cloud[i].x, py=active_map_ptr->cloud[i].y;
                if(px<wx0||px>wx1||py<wy0||py>wy1) continue;
                Vector2 sp=w2s(px,py,view2d);
                DrawPixel((int)sp.x,(int)sp.y,{200,60,60,200});
            }
        }
        {std::lock_guard<std::mutex>lk(trajectory_mutex);
         if(robot_trajectory.size()>1)
             for(size_t i=1;i<robot_trajectory.size();i++){
                 Vector2 a=w2s(robot_trajectory[i-1].x,-robot_trajectory[i-1].z,view2d);
                 Vector2 b=w2s(robot_trajectory[i].x,-robot_trajectory[i].z,view2d);
                 DrawLineV(a,b,MAGENTA);
             }}
        // Reference-path curves appear only inside the active zone (zone 1 armed).
        // Maps with no zone defined fall back to the manual overlay toggle.
        // Show unless a zone exists on this map and none of them are armed — with two
        // zones, "no zone at all" becomes "neither zone 1 nor zone 2 is drawn".
        bool ref_curves_visible = g_show_path_overlay.load() &&
            ((!g_zone_valid && !g_zone2_valid) || g_zone_armed || g_zone2_armed);
        if(ref_curves_visible)
        for(size_t i=1;i<g_corridor_L.size();i++){
            DrawLineV(w2s(g_corridor_L[i-1].x,g_corridor_L[i-1].y,view2d),
                      w2s(g_corridor_L[i].x,g_corridor_L[i].y,view2d),Fade(SKYBLUE,0.25f));
            DrawLineV(w2s(g_corridor_R[i-1].x,g_corridor_R[i-1].y,view2d),
                      w2s(g_corridor_R[i].x,g_corridor_R[i].y,view2d),Fade(SKYBLUE,0.25f));
        }

        // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ Corridor polygon overlay ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â developer mode only (June 22, 2026) ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬
        if(false){
            const std::vector<Vector2>& poly = g_polygon_edit_mode
                                               ? g_polygon_edit_verts
                                               : g_segment_polygon;
            Color pc = g_polygon_edit_mode ? C_WARN : Fade(C_GOOD, 0.7f);
            int np = (int)poly.size();
            for(int i = 0; i < np; i++){
                Vector2 sp = w2s(poly[i].x, poly[i].y, view2d);
                DrawCircleV(sp, 4.f, pc);
                if(i > 0)
                    DrawLineEx(w2s(poly[i-1].x,poly[i-1].y,view2d), sp, 1.5f, pc);
            }
            if(!g_polygon_edit_mode && np >= 3)
                DrawLineEx(w2s(poly[np-1].x,poly[np-1].y,view2d),
                           w2s(poly[0].x,poly[0].y,view2d), 1.5f, pc);
            if(g_polygon_edit_mode && np >= 3)
                DrawLineEx(w2s(poly[np-1].x,poly[np-1].y,view2d),
                           w2s(poly[0].x,poly[0].y,view2d), 1.f, Fade(C_WARN, 0.35f));
        }
        // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ End corridor polygon overlay ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬

        // ── Destination grid cell overlays ──────────────────────────────────
        // Draw colored 5m×5m cells painted as destination zones. Visible in
        // Steps 2 & 3. In edit mode also shows a white highlight on the hovered cell.
        if(!g_dest_grid.empty()){
            static const Color gc_fill[3]  ={{0,180,220,70},{230,185,30,70},{60,200,80,70}};
            static const Color gc_bdr[3]   ={{0,220,255,180},{255,210,40,180},{80,230,100,180}};
            for(auto& kv : g_dest_grid){
                int col=kv.first.first, row=kv.first.second, di=kv.second;
                if(di<0||di>2) continue;
                float wx0=col*DEST_GRID_CELL_M, wy1=(row+1)*DEST_GRID_CELL_M;
                float wx1=wx0+DEST_GRID_CELL_M, wy0=row*DEST_GRID_CELL_M;
                Vector2 tl=w2s(wx0,wy1,view2d), br=w2s(wx1,wy0,view2d);
                int rx=(int)tl.x, ry=(int)tl.y;
                int rw=(int)(br.x-tl.x), rh=(int)(br.y-tl.y);
                if(rw<1||rh<1||rx+rw<0||rx>mapW||ry+rh<TITLE_H||ry>mapH) continue;
                DrawRectangle(rx,ry,rw,rh,gc_fill[di]);
                DrawRectangleLines(rx,ry,rw,rh,gc_bdr[di]);
            }
        }
        if(step==STEP_PLAN && g_dest_grid_edit){
            Vector2 mp=GetMousePosition();
            if(mp.x>=0&&mp.x<(float)mapW&&mp.y>=(float)TITLE_H&&mp.y<(float)mapH){
                float hx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
                float hy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
                int hc=(int)floorf(hx/DEST_GRID_CELL_M), hr=(int)floorf(hy/DEST_GRID_CELL_M);
                Vector2 tl=w2s(hc*DEST_GRID_CELL_M,(hr+1)*DEST_GRID_CELL_M,view2d);
                Vector2 br=w2s((hc+1)*DEST_GRID_CELL_M,hr*DEST_GRID_CELL_M,view2d);
                int rx=(int)tl.x, ry=(int)tl.y;
                int rw=(int)(br.x-tl.x), rh=(int)(br.y-tl.y);
                if(rw>0&&rh>0) DrawRectangleLines(rx,ry,rw,rh,{255,255,255,200});
            }
        }
        // ── End destination grid ─────────────────────────────────────────────

        if(step==STEP_PLAN){
            if(ref_curves_visible)
            for(size_t i=1;i<g_smooth.size();i++)
                DrawLineEx(w2s(g_smooth[i-1].x,g_smooth[i-1].y,view2d),
                           w2s(g_smooth[i].x,g_smooth[i].y,view2d),0.5f,Fade(WHITE,0.3f));

            // Path-edit selection highlight: the [lo,hi] stretch that will be nudged.
            if(g_path_edit_mode && !g_smooth.empty()){
                int n=(int)g_smooth.size();
                auto ring=[&](int idx,Color c){ if(idx>=0&&idx<n){ Vector2 p=w2s(g_smooth[idx].x,g_smooth[idx].y,view2d); DrawCircleLines((int)p.x,(int)p.y,8,c); DrawCircleV(p,3,c);} };
                if(g_path_edit_i0>=0 && g_path_edit_i1>=0){
                    int lo=std::min(g_path_edit_i0,g_path_edit_i1), hi=std::max(g_path_edit_i0,g_path_edit_i1);
                    lo=std::max(0,lo); hi=std::min(n-1,hi);
                    for(int i=lo+1;i<=hi;i++)
                        DrawLineEx(w2s(g_smooth[i-1].x,g_smooth[i-1].y,view2d),
                                   w2s(g_smooth[i].x,g_smooth[i].y,view2d),3.f,ORANGE);
                }
                ring(g_path_edit_i0, ORANGE);
                ring(g_path_edit_i1, ORANGE);
            }

            // While using Set Danger Zone, draw BOTH zones' corridors regardless of
            // which one is armed — normally only the armed zone's curve is visible
            // (as g_smooth), so the other zone is invisible and impossible to click
            // accurately without arming it first.
            if(g_dz_edit_mode){
                const std::vector<PPPoint>& z1 = pp_zone_corridor(1);
                const std::vector<PPPoint>& z2 = pp_zone_corridor(2);
                for(size_t i=1;i<z1.size();i++)
                    DrawLineEx(w2s(z1[i-1].x,z1[i-1].y,view2d),w2s(z1[i].x,z1[i].y,view2d),1.5f,Fade(SKYBLUE,0.6f));
                for(size_t i=1;i<z2.size();i++)
                    DrawLineEx(w2s(z2[i-1].x,z2[i-1].y,view2d),w2s(z2[i].x,z2[i].y,view2d),1.5f,Fade(C_WARN,0.6f));
            }

            // Danger-zone selection markers — points may be in different corridors
            // (zone 1 vs zone 2), so each is drawn independently via pp_zone_corridor_point
            // rather than assuming both sit in the currently-loaded g_smooth.
            if(g_dz_edit_mode){
                auto ring2=[&](int zone,int idx,Color c){
                    float px,py;
                    if(idx>=0 && pp_zone_corridor_point(zone,idx,px,py)){
                        Vector2 p=w2s(px,py,view2d);
                        DrawCircleLines((int)p.x,(int)p.y,9,c); DrawCircleV(p,4,c);
                    }
                };
                ring2(g_dz_edit_zone0, g_dz_edit_idx0, RED);
                ring2(g_dz_edit_zone1, g_dz_edit_idx1, RED);
            }
            for(size_t i=1;i<g_approach_path.size();i++)
                DrawLineEx(w2s(g_approach_path[i-1].x,g_approach_path[i-1].y,view2d),
                           w2s(g_approach_path[i].x,g_approach_path[i].y,view2d),2,SKYBLUE);
            for(size_t i=1;i<g_corridor_path.size();i++)
                DrawLineEx(w2s(g_corridor_path[i-1].x,g_corridor_path[i-1].y,view2d),
                           w2s(g_corridor_path[i].x,g_corridor_path[i].y,view2d),2.5f,LIME);
            for(size_t i=1;i<g_goal_path.size();i++)
                DrawLineEx(w2s(g_goal_path[i-1].x,g_goal_path[i-1].y,view2d),
                           w2s(g_goal_path[i].x,g_goal_path[i].y,view2d),2,SKYBLUE);
            if(g_pp_robot.valid){
                Vector2 rp2=w2s(g_pp_robot.x,g_pp_robot.y,view2d);
                DrawCircleV(rp2,8,SKYBLUE);
                DrawLineEx(rp2,{rp2.x-sinf(g_pp_robot.yaw)*20,rp2.y-cosf(g_pp_robot.yaw)*20},2,WHITE);
            }
            if(g_pp_goal.valid){
                Vector2 gp=w2s(g_pp_goal.x,g_pp_goal.y,view2d);
                DrawCircleV(gp,8,C_WARN); DrawCircleLines((int)gp.x,(int)gp.y,8,WHITE);
            }
            double ex4=0,ey4=0,eyaw4=0;
            {std::lock_guard<std::mutex>lk(ekf_mutex);ex4=ekf.x[0];ey4=ekf.x[1];eyaw4=ekf.x[2];}
            Vector2 rp2=w2s((float)ex4,(float)ey4,view2d);
            DrawCircleV(rp2,10,YELLOW);
            DrawLineEx(rp2,{rp2.x-sinf((float)eyaw4)*24,rp2.y-cosf((float)eyaw4)*24},2.5f,YELLOW);

            // Hover snap indicator: shows nearest path point while mouse is in the map area
            // so you see exactly where Ctrl+Click will place the goal before clicking.
            if(!g_smooth.empty()){
                Vector2 mp=GetMousePosition();
                if(mp.x>=0&&mp.x<(float)mapW&&mp.y>=(float)TITLE_H&&mp.y<(float)mapH){
                    float hx=view2d.cx+(mp.x-view2d.sw*0.5f)/view2d.scale;
                    float hy=view2d.cy-(mp.y-view2d.sh*0.5f)/view2d.scale;
                    int best=0; float bd=1e9f;
                    for(int i=0;i<(int)g_smooth.size();i++){
                        float dx=g_smooth[i].x-hx, dy=g_smooth[i].y-hy;
                        float d=dx*dx+dy*dy;
                        if(d<bd){bd=d;best=i;}
                    }
                    Vector2 sp=w2s(g_smooth[best].x,g_smooth[best].y,view2d);
                    DrawCircleLines((int)sp.x,(int)sp.y,10,C_WARN);
                    DrawLineEx(sp,{sp.x+sinf(g_smooth[best].yaw)*16.f,
                                   sp.y-cosf(g_smooth[best].yaw)*16.f},1.5f,C_WARN);
                }
            }
        }

        if(step==STEP_DRIVE){
            {
                double lx=0,ly=0,lyaw=0;
                {std::lock_guard<std::mutex>lk(ekf_mutex);lx=ekf.x[0];ly=ekf.x[1];lyaw=ekf.x[2];}
                float cy_l=cosf((float)lyaw), sy_l=sinf((float)lyaw);
                std::lock_guard<std::mutex>lk(lidar_mutex);
                for(size_t i=0;i<dense_cloud.size();i+=2){
                    const auto& p=dense_cloud[i];
                    float wx=cy_l*p.x-sy_l*p.y+(float)lx;
                    float wy=sy_l*p.x+cy_l*p.y+(float)ly;
                    Vector2 sp=w2s(wx,wy,view2d);
                    if(sp.x<0||sp.x>=mapW||sp.y<TITLE_H||sp.y>=mapH) continue;
                    float nh=Clamp((p.z+0.5f)/2.5f,0,1);
                    Color sc=ColorFromHSV((1-nh)*200.f,0.9f,1.f); sc.a=180;
                    DrawPixel((int)sp.x,(int)sp.y,sc);
                }
            }
            {
                int N_path=(int)g_ctrl_path.size();
                for(int i=1;i<N_path;i++){
                    Vector2 a=w2s(g_ctrl_path[i-1].x,g_ctrl_path[i-1].y,view2d);
                    Vector2 b=w2s(g_ctrl_path[i].x,g_ctrl_path[i].y,view2d);
                    DrawLineEx(a,b,1.5f,{100,180,255,90});
                }
                int ci_draw=g_ctrl.lastCI;
                int hi=(g_ctrl_state==CS_RUNNING && g_ctrl.lookaheadIdx>ci_draw)
                        ? std::min(N_path-1, g_ctrl.lookaheadIdx)
                        : std::min(N_path-1, ci_draw+50);
                for(int i=std::max(1,ci_draw);i<=hi;i++){
                    Vector2 a=w2s(g_ctrl_path[i-1].x,g_ctrl_path[i-1].y,view2d);
                    Vector2 b=w2s(g_ctrl_path[i].x,g_ctrl_path[i].y,view2d);
                    DrawLineEx(a,b,2.f,{57,217,138,220});
                }
            }
            {
                const auto& tr=g_ctrl_trail;
                int N=(int)tr.size();
                if(N>1){
                    auto smx=[&](int i)->float{float s=0;int cnt=0;for(int k=i-2;k<=i+2;k++){if(k>=0&&k<N){s+=tr[k].x;cnt++;}}return s/cnt;};
                    auto smy=[&](int i)->float{float s=0;int cnt=0;for(int k=i-2;k<=i+2;k++){if(k>=0&&k<N){s+=tr[k].y;cnt++;}}return s/cnt;};
                    for(int i=1;i<N;i++){
                        float t=(float)i/N;
                        DrawLineV(w2s(smx(i-1),smy(i-1),view2d),w2s(smx(i),smy(i),view2d),
                                  {(unsigned char)(255*t),(unsigned char)(180*(1-t)),50,200});
                    }
                }
            }
            if(g_ctrl_state==CS_DONE){
                DrawRectangle(10,TITLE_H+10,220,46,{0,0,0,160});
                DrawRectangle(18,TITLE_H+20,30,3,{100,180,255,180});
                DrawText("Planned path",54,TITLE_H+15,12,{100,180,255,230});
                DrawRectangle(18,TITLE_H+38,30,3,{255,140,50,200});
                DrawText("Actual path",54,TITLE_H+33,12,{255,140,50,230});
            }
            if(g_ctrl_start.valid){
                Vector2 sp2=w2s(g_ctrl_start.x,g_ctrl_start.y,view2d);
                DrawCircleV(sp2,10,C_HEADER);
            }
            if(g_ctrl_goal.valid){
                Vector2 gp2=w2s(g_ctrl_goal.x,g_ctrl_goal.y,view2d);
                DrawCircleV(gp2,10,C_BAD);
                DrawCircleLines((int)gp2.x,(int)gp2.y,(int)(GOAL_RADIUS_M*view2d.scale),C_BAD);
            }
            CtrlPose& robot=g_simMode?g_ctrl_sim:g_ctrl_robot_filtered;
            if(robot.valid){
                // Use last trajectory point so the circle sits exactly at the tip of the magenta line.
                // Falls back to filtered EKF pose in sim mode or before the first scan match.
                Vector2 rp3;
                if(!g_simMode){
                    std::lock_guard<std::mutex> lk(trajectory_mutex);
                    if(!robot_trajectory.empty()){
                        const auto& last=robot_trajectory.back();
                        rp3=w2s(last.x,-last.z,view2d);
                    } else {
                        rp3=w2s(robot.x,robot.y,view2d);
                    }
                } else {
                    rp3=w2s(robot.x,robot.y,view2d);
                }
                Color rc=(g_ctrl_state==CS_RUNNING)?SKYBLUE:(g_ctrl_state==CS_DONE)?LIME:(g_ctrl_state==CS_WATCHDOG)?C_BAD:GRAY;
                DrawCircleV(rp3,11,rc); DrawCircleLines((int)rp3.x,(int)rp3.y,11,WHITE);
            }
        }

        // "No path available" fade popup (2026-07-19) — same visual pattern as the
        // passenger destination-unavailable message (DrawDestinationPanel): centred,
        // fading, red-bordered box. Set by pp_apply_obstacle_avoidance() (planning.cpp)
        // when a static obstacle has no valid detour within the safe corridor on
        // either side — drawn last so it sits on top of the map/path/obstacle cells.
        if(g_obst_blocked_msg_timer > 0.f){
            g_obst_blocked_msg_timer -= GetFrameTime();
            float a = fminf(1.f, g_obst_blocked_msg_timer);
            const char* m1 = "No path available";
            const char* m2 = "Obstacle blocks the corridor - no valid route around it";
            int fs1=22, fs2=16;
            int w1=MeasureText(m1,fs1), w2=MeasureText(m2,fs2);
            int bw=std::max(w1,w2)+56, bh=92;
            int bx=mapW/2-bw/2, by=TITLE_H+(mapH-TITLE_H)/2-bh/2;
            DrawRectangleRounded({(float)bx,(float)by,(float)bw,(float)bh},0.22f,8,
                                 {50,0,0,(unsigned char)(215*a)});
            DrawRectangleRoundedLines({(float)bx,(float)by,(float)bw,(float)bh},0.22f,8,2.f,
                                      {255,70,70,(unsigned char)(230*a)});
            DrawText(m1, mapW/2-w1/2, by+18, fs1, {255,120,120,(unsigned char)(255*a)});
            DrawText(m2, mapW/2-w2/2, by+52, fs2, {225,160,160,(unsigned char)(225*a)});
        }
    }
}



// ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ End operator UI (June 22, 2026) ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬


// =============================================================================
//  WELCOME PANEL — GIU branded splash shown at startup, before the intro
// =============================================================================
void DrawWelcomePanel(int sw, int sh, Texture2D img, Font font, bool& show_welcome, bool& show_intro) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };
    Vector2 mouse = GetMousePosition();

    // Cover-fit the welcome image to the whole window.
    if(img.id>0){
        float iw=(float)img.width, ih=(float)img.height;
        float s=fmaxf((float)sw/iw,(float)sh/ih);
        DrawTextureEx(img,{(sw-iw*s)*0.5f,(sh-ih*s)*0.5f},0.f,s,WHITE);
    } else {
        DrawRectangle(0,0,sw,sh,{14,19,33,255});
    }

    // Bottom gradient so the title + button read cleanly over the photo.
    int gradH = (int)(sh*0.42f);
    DrawRectangleGradientV(0, sh-gradH, sw, gradH, {0,0,0,0}, {0,0,0,225});
    // Thin top gradient to seat any status text if added later.
    DrawRectangleGradientV(0, 0, sw, (int)(sh*0.18f), {0,0,0,150}, {0,0,0,0});

    // Title block near the bottom.
    float cx=(float)sw*0.5f;
    float titleY = sh - gradH*0.62f;
    TC("German International University", cx, titleY,        44.f, WHITE);
    TC("Autonomous Golf Cart System",    cx, titleY+54.f,   24.f, {180,195,215,230});

    // Get Started button (GIU warm accent).
    float bw=300.f, bh=64.f;
    Rectangle go={cx-bw*0.5f, sh-bh-46.f, bw, bh};
    bool hov=CheckCollisionPointRec(mouse,go);
    Color fill = hov?Color{225,150,35,255}:Color{205,130,25,255};
    DrawRectangleRounded(go,0.32f,10,fill);
    DrawRectangleRoundedLines(go,0.32f,10,2.f,{255,200,120,220});
    const char* gl="Get Started";
    TW(gl, go.x+(go.width-MW(gl,26.f))*0.5f, go.y+(go.height-26.f)*0.5f, 26.f, WHITE);

    // Whole screen is clickable too (tap anywhere), plus the button.
    if(IsMouseButtonReleased(MOUSE_LEFT_BUTTON)){
        show_welcome=false; show_intro=true;
    }
}

// =============================================================================
//  INTRO PANEL
// =============================================================================
void DrawIntroPanel(int sw, int sh, Texture2D bg, Texture2D logo, Texture2D cart_icon, Font font, bool& show_intro, bool& remote_shown, bool& show_dest) {
    Vector2 mouse = GetMousePosition();
    const float SP = 0.6f; // letter spacing

    // Text helpers
    auto TW = [&](const char* t, float x, float y, float sz, Color c){
        DrawTextEx(font, t, {x,y}, sz, SP, c);
    };
    auto MW = [&](const char* t, float sz) -> float {
        return MeasureTextEx(font, t, sz, SP).x;
    };
    auto TC = [&](const char* t, float cx, float y, float sz, Color c){
        TW(t, cx - MW(t,sz)*0.5f, y, sz, c);
    };

    // Background
    if(bg.id > 0) {
        float sc = fmaxf((float)sw/bg.width,(float)sh/bg.height);
        DrawTextureEx(bg,{(sw-bg.width*sc)*0.5f,(sh-bg.height*sc)*0.5f},0.f,sc,WHITE);
    }
    DrawRectangle(0,0,sw,sh,{0,0,0,(unsigned char)(bg.id>0?210:255)});
    DrawRectangle(0,0,sw,4,C_HEADER);
    DrawRectangle(0,sh-4,sw,4,C_HEADER);

    float cx=(float)sw*0.5f;
    float ty=(float)sh/16;

    // Logo
    if(logo.id > 0) {
        int lh=68, lw=(int)((float)logo.width/logo.height*lh);
        DrawRectangle((int)(cx-lw/2-10),(int)ty-5,lw+20,lh+10,WHITE);
        DrawTexturePro(logo,{0,0,(float)logo.width,(float)logo.height},
                       {cx-lw/2,ty,(float)lw,(float)lh},{0,0},0.f,WHITE);
        ty += lh+18;
    }

    // Title & author
    TC("AUTONOMOUS GOLF CART SYSTEM", cx, ty, 52, C_HEADER); ty += 62;
    DrawRectangle((int)(cx-250),(int)ty,500,2,C_BORDER_HI); ty += 12;
    TC("Mohammed Alaa Elkouka  |  Master's Thesis  |  GIU  2026", cx, ty, 16, {170,195,220,185});

    // ── Icon helpers (drawn with primitives) ──────────────────────────────────
    // Remote-control icon: phone + 3 WiFi arcs above it
    auto drawRemoteIcon = [](int icx, int icy, float sz, Color col) {
        // Phone body (slightly below center so arcs fit above)
        float pw=sz*0.32f, ph=sz*0.52f;
        float px=icx-pw/2, py=(float)icy-ph*0.30f;
        DrawRectangleRounded({px,py,pw,ph},0.22f,8,{15,35,80,240});
        DrawRectangleRoundedLines({px,py,pw,ph},0.22f,8,2.5f,col);
        // Screen
        DrawRectangle((int)(px+3),(int)(py+7),(int)(pw-6),(int)(ph-18),{80,140,195,180});
        // Camera dot
        DrawCircle((int)icx,(int)(py+4),3,{55,95,155,200});
        // Home bar
        DrawRectangleRounded({px+pw/2-10,py+ph-6.f,20,3},0.5f,4,col);

        // WiFi arcs above phone, pointing upward (Raylib: 270=up, 215–325 = top arc)
        float ac_x=(float)icx, ac_y=py-6.f;
        DrawCircle((int)ac_x,(int)ac_y,4,col);
        for(int i=0;i<3;i++){
            float r=sz*(0.072f+i*0.085f);
            DrawRing({ac_x,ac_y},r-2.5f,r+2.5f,215.f,325.f,28,col);
        }
    };

    // Autonomous icon: simple side-view car + cyan sensor arcs on right (forward)
    auto drawCartSide = [](int icx, int icy, float sz, Color col) {
        Color dark  = {6, 22, 10, 255};
        Color arc_c = {0, 215, 240, 255};
        Color hlight= {220,240,140,210};

        // Car shifted left so arcs fit on right
        float ox=icx-sz*0.09f, oy=(float)icy;

        // ── Lower body / chassis ──
        float bw=sz*0.68f, bh=sz*0.17f;
        float bx=ox-bw*0.50f, by=oy+sz*0.02f;
        DrawRectangleRounded({bx,by,bw,bh},0.35f,8,col);

        // ── Cabin (offset toward rear; hood extends right = front) ──
        float cw=sz*0.38f, ch=sz*0.20f;
        float cx=bx+bw*0.08f, cy=by-ch;
        DrawRectangleRounded({cx,cy,cw,ch},0.40f,8,col);
        // Glass (dark inset)
        DrawRectangleRounded({cx+sz*0.030f, cy+sz*0.022f,
                               cw-sz*0.060f, ch-sz*0.030f},0.30f,6,dark);

        // ── Wheels with hub rings ──
        float wr=sz*0.102f, wy=by+bh+wr*0.18f;
        float wxf=bx+bw*0.80f, wxr=bx+bw*0.18f;
        DrawCircle((int)wxf,(int)wy,(int)wr,col);
        DrawCircle((int)wxr,(int)wy,(int)wr,col);
        DrawCircle((int)wxf,(int)wy,(int)(wr*0.44f),dark);
        DrawCircle((int)wxr,(int)wy,(int)(wr*0.44f),dark);
        DrawCircle((int)wxf,(int)wy,(int)(wr*0.17f),col);
        DrawCircle((int)wxr,(int)wy,(int)(wr*0.17f),col);

        // ── Headlight (front-right of body) ──
        DrawRectangleRounded({bx+bw-sz*0.036f, by+sz*0.025f,
                               sz*0.036f, sz*0.060f},0.5f,4,hlight);

        // ── Sensor arcs pointing right (forward) ──
        float ax=bx+bw+2.f, ay=by+bh*0.48f;
        DrawCircle((int)ax,(int)ay,4,arc_c);
        for(int i=0;i<3;i++){
            float r=sz*(0.062f+i*0.074f);
            Color ac={arc_c.r,arc_c.g,arc_c.b,(unsigned char)(255-i*55)};
            DrawRing({ax,ay},r-4.0f,r+4.0f,-45.f,45.f,24,ac);
        }
    };

    // ── Cards layout ─────────────────────────────────────────────────────────
    int cardW=600, cardH=545, gap=56;
    int cardY=(int)ty+30;
    if(cardY+cardH>sh-24) cardY=sh-24-cardH;
    int cardX1=(int)cx-gap/2-cardW;
    int cardX2=(int)cx+gap/2;

    const int ICON_OFF=80, ICON_R=48;

    // Card background helper
    auto DrawCard=[](float x,float y,float w,float h,Color fill,Color accent,bool active){
        DrawRectangleRounded({x+3,y+3,w,h},0.08f,8,{0,0,0,70});   // shadow
        DrawRectangleRounded({x,y,w,h},0.08f,8,fill);
        DrawRectangleRoundedLines({x+1,y+1,w-2,h-2},0.08f,8,1.f,{255,255,255,15}); // inner shine
        float bw=active?2.5f:1.5f;
        unsigned char ba=active?200:90;
        DrawRectangleRoundedLines({x,y,w,h},0.08f,8,bw,{accent.r,accent.g,accent.b,ba});
        DrawRectangleRounded({x,y,w,6},0.0f,4,accent);             // top stripe
    };

    // Button helper
    auto DrawBtn=[&](Rectangle btn,Color fill,Color border,const char* label,Color tc){
        DrawRectangleRounded(btn,0.25f,8,fill);
        DrawRectangleRoundedLines(btn,0.25f,8,1.5f,border);
        float fsz=27.f;
        TW(label, btn.x+(btn.width-MW(label,fsz))*0.5f,
                  btn.y+(btn.height-fsz)*0.5f, fsz, tc);
    };

    // ─── Card 1: Remote Control ───────────────────────────────────────────────
    {
        Rectangle rc={(float)cardX1,(float)cardY,(float)cardW,(float)cardH};
        bool hov=CheckCollisionPointRec(mouse,rc);
        bool act=remote_shown||hov;
        Color fill=remote_shown?Color{16,54,112,248}:(hov?Color{18,48,92,238}:Color{10,24,52,228});
        DrawCard(rc.x,rc.y,rc.width,rc.height,fill,C_HEADER,act);

        float icx=(float)(cardX1+cardW/2), icy=(float)(cardY+ICON_OFF);
        DrawCircle((int)icx,(int)icy,ICON_R+20,{30,80,180,30});
        DrawCircle((int)icx,(int)icy,ICON_R+9, {40,100,210,70});
        DrawCircle((int)icx,(int)icy,ICON_R,   {18,52,140,235});
        drawRemoteIcon((int)icx,(int)icy,110.f,C_HEADER);

        float tcy=cardY+ICON_OFF*2+16;
        TC("Remote Control", icx, tcy, 44, C_HEADER);          tcy+=58;
        DrawRectangle((int)(icx-60),(int)tcy,120,3,C_HEADER);  tcy+=20;
        TC("Drive the cart manually from your phone", icx, tcy, 23, {165,205,240,210}); tcy+=42;
        TC("http://10.63.129.200", icx, tcy, 29, {255,228,55,255});                    tcy+=42;
        TC("Connect to campus WiFi, then open the link", icx, tcy, 19, C_DIM);

        Rectangle btn={(float)(cardX1+32),(float)(cardY+cardH-70),(float)(cardW-64),52};
        bool hb=CheckCollisionPointRec(mouse,btn);
        Color bf=remote_shown?C_HEADER:(hb?ColorBrightness(C_HEADER,0.25f):Color{20,52,115,255});
        DrawBtn(btn,bf,C_HEADER,remote_shown?"Connected":"Open on My Device",
                remote_shown?BLACK:WHITE);
        if(hb&&IsMouseButtonReleased(MOUSE_LEFT_BUTTON)){
            bool was=remote_shown; remote_shown=!remote_shown;
            if(!was&&remote_shown) ShellExecuteA(NULL,"open","http://10.63.129.200",NULL,NULL,1);
        }
    }

    // ─── Card 2: Autonomous Driving ──────────────────────────────────────────
    {
        Rectangle rc={(float)cardX2,(float)cardY,(float)cardW,(float)cardH};
        bool hov=CheckCollisionPointRec(mouse,rc);
        Color fill=hov?Color{8,48,20,238}:Color{6,26,12,228};
        DrawCard(rc.x,rc.y,rc.width,rc.height,fill,C_GOOD,hov);

        float icx=(float)(cardX2+cardW/2), icy=(float)(cardY+ICON_OFF);
        DrawCircle((int)icx,(int)icy,ICON_R+20,{20,90,45,30});
        DrawCircle((int)icx,(int)icy,ICON_R+9, {30,130,60,70});
        DrawCircle((int)icx,(int)icy,ICON_R,   {14,68,26,235});
        if(cart_icon.id>0){
            int isz=ICON_R*2;
            DrawTexturePro(cart_icon,
                {0,0,(float)cart_icon.width,(float)cart_icon.height},
                {icx-isz/2.f,icy-isz/2.f,(float)isz,(float)isz},
                {0,0},0.f,WHITE);
        } else {
            drawCartSide((int)icx,(int)icy,110.f,C_GOOD);
        }

        float tcy=cardY+ICON_OFF*2+16;
        TC("Autonomous Driving", icx, tcy, 44, C_GOOD);        tcy+=58;
        DrawRectangle((int)(icx-60),(int)tcy,120,3,C_GOOD);    tcy+=20;
        TC("The cart drives itself to your destination", icx, tcy, 23, {150,210,165,210}); tcy+=46;

        static const char* steps[]={"Find cart location on the map",
                                    "Choose where you want to go",
                                    "Sit back - the cart does the rest"};
        float lx=(float)(cardX2+40);
        for(int i=0;i<3;i++){
            DrawCircle((int)(lx+18),(int)(tcy+18),18,{0,145,55,215});
            DrawCircleLines((int)(lx+18),(int)(tcy+18),18,{80,255,130,140});
            char num[2]={char('1'+i),0};
            TW(num, lx+18-MW(num,22)*0.5f, tcy+7, 22, WHITE);
            TW(steps[i], lx+48, tcy+8, 23, {180,238,192,230});
            tcy+=48;
        }

        Rectangle btn={(float)(cardX2+32),(float)(cardY+cardH-70),(float)(cardW-64),52};
        bool hb=CheckCollisionPointRec(mouse,btn);
        Color bf=hb?ColorBrightness(C_GOOD,0.2f):Color{0,140,58,255};
        DrawBtn(btn,bf,C_GOOD,"Start Journey",WHITE);
        if(hb&&IsMouseButtonReleased(MOUSE_LEFT_BUTTON)){ show_intro=false; g_show_map_select=true; }
    }

    // Footer
    TC("GolfCartControl v1.0  |  GIU Robotics Lab", cx, (float)(sh-26), 15, C_DIM);
}



// =============================================================================
// MAP SELECT PANEL  — "which direction are you heading?" before localizing
// East -> forward map (Campus Top LiDAR, Labs side); West -> reverse map
// (Campus Top (Rev), Food Court side).
// =============================================================================
void DrawMapSelectPanel(int sw, int sh, Texture2D bg, Font font, bool& show_map_select) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };

    Vector2 mouse=GetMousePosition();
    auto Button=[&](Rectangle r,const char* label,Color fill,Color border,Color tc,float fsz)->bool{
        bool hov=CheckCollisionPointRec(mouse,r);
        DrawRectangleRounded(r,0.28f,8,hov?ColorBrightness(fill,0.18f):fill);
        DrawRectangleRoundedLines(r,0.28f,8,1.5f,border);
        TW(label,r.x+(r.width-MW(label,fsz))*0.5f,r.y+(r.height-fsz)*0.5f,fsz,tc);
        return hov && IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
    };

    // Compass-arrow icon drawn from primitives (shaft + triangle head), not a text
    // glyph — the panel font's loaded atlas is ASCII-only and renders '->'/'<-'
    // Unicode arrows as "?" (see DEVELOPMENT_NOTES.md note on this exact bug).
    auto drawArrowIcon=[&](float acx,float acy,float len,Color col,bool pointRight){
        float dir=pointRight?1.f:-1.f;
        float headLen=len*0.42f, headHalfH=len*0.34f, shaftTh=len*0.16f;
        float tipX=acx+dir*len*0.5f, tailX=acx-dir*len*0.5f, neckX=tipX-dir*headLen;
        DrawLineEx({tailX,acy},{neckX,acy},shaftTh,col);
        Vector2 tip={tipX,acy}, top={neckX,acy-headHalfH}, bot={neckX,acy+headHalfH};
        // East/West are mirror images, so tip/neck swap sides between them — the same
        // vertex order therefore has opposite winding for the two, and whichever
        // winding is the "back face" silently doesn't render under whatever culling
        // state is active (confirmed empirically: West's original order rendered,
        // East's didn't; swapping BOTH — the first attempt at this fix — made both
        // disappear, so only the pointRight order actually needed flipping).
        if(pointRight) DrawTriangle(tip,top,bot,col);
        else           DrawTriangle(tip,bot,top,col);
    };

    // Background
    float iw=bg.id>0?(float)bg.width:1920.f, ih=bg.id>0?(float)bg.height:1080.f;
    float bscale=fmaxf((float)sw/iw,(float)sh/ih);
    float bgX=(sw-iw*bscale)*0.5f, bgY=(sh-ih*bscale)*0.5f;
    if(bg.id>0) DrawTextureEx(bg,{bgX,bgY},0.f,bscale,WHITE);
    else DrawRectangle(0,0,sw,sh,{18,24,38,255});
    DrawRectangle(0,0,sw,sh,{0,0,0,160}); // dark overlay

    // Centre card — sized up ~44% 2026-07-27 (user request, matches the readability
    // pass applied to the other passenger-facing cards).
    const float CW=835.f, CH=518.f;
    float cx=(float)sw*0.5f, cy=(float)sh*0.5f;
    float cardX=cx-CW*0.5f, cardY=cy-CH*0.5f;
    // Shadow
    DrawRectangleRounded({cardX+9,cardY+9,CW,CH},0.09f,8,{0,0,0,110});
    // Card body
    DrawRectangleRounded({cardX,cardY,CW,CH},0.09f,8,{13,18,31,248});
    DrawRectangleRoundedLines({cardX+1,cardY+1,CW-2,CH-2},0.09f,8,1.f,{255,255,255,15}); // inner shine
    // Accent top stripe
    DrawRectangleRounded({cardX,cardY,CW,12},0.f,4,{60,120,220,220});

    // Question text
    TC("Which direction will you be heading?", cx, cardY+43.f, 36.f, WHITE);
    TC("This tells us which route to load for your journey.", cx, cardY+98.f, 24.f, {170,185,205,210});

    // Direction tiles
    const float TSZ_W=360.f, TSZ_H=252.f, GAP=40.f, TY=cardY+167.f;
    float tx1=cx-GAP*0.5f-TSZ_W, tx2=cx+GAP*0.5f;

    auto DirectionTile=[&](Rectangle r,const char* dirLabel,const char* landmark,
                            Color fill,Color border,bool pointRight)->bool{
        bool hov=CheckCollisionPointRec(mouse,r);
        DrawRectangleRounded(r,0.12f,10,hov?ColorBrightness(fill,0.18f):fill);
        DrawRectangleRoundedLines(r,0.12f,10,1.5f,border);
        drawArrowIcon(r.x+r.width*0.5f, r.y+60.f, 69.f, WHITE, pointRight);
        TC(dirLabel, r.x+r.width*0.5f, r.y+118.f, 39.f, WHITE);
        TC(landmark, r.x+r.width*0.5f, r.y+173.f, 21.f, {225,235,245,195});
        return hov && IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
    };

    // West on the left, East on the right (compass layout) — arrows then point
    // outward away from the card centre instead of at each other.
    // West -> reverse map (Food Court side)
    if(DirectionTile({tx1,TY,TSZ_W,TSZ_H},"West","Toward the Labs",
                      {22,118,78,255},{70,190,140,220},false)){
        switch_active_map_by_name("Campus Top (Rev)");
        show_map_select=false; g_show_localize=true;
    }

    // East -> forward map (Labs side)
    if(DirectionTile({tx2,TY,TSZ_W,TSZ_H},"East","Toward the Food Court",
                      {18,104,186,255},{70,170,235,220},true)){
        switch_active_map_by_name("Campus Top LiDAR");
        show_map_select=false; g_show_localize=true;
    }

    // Back link (bottom of card)
    if(Button({cx-86.f,cardY+CH-81.f,173.f,55.f},"< Back",{28,32,48,200},{70,80,110,150},{170,180,200,230},22.f)){
        show_map_select=false; g_show_intro=true;
    }

    // Small decorative compass rose, top-right of the whole window (not the card) —
    // a geographic anchor for the East/West choice. Static reference rose (no live
    // heading needed here), drawn last so it sits on top of everything else.
    {
        float rcx=(float)sw-100.f, rcy=90.f, rad=40.f;
        DrawCircle((int)rcx,(int)rcy,(int)(rad+8.f),{0,0,0,80});
        DrawCircle((int)rcx,(int)rcy,(int)rad,{20,26,42,235});
        DrawCircleLines((int)rcx,(int)rcy,(int)rad,{120,140,170,220});
        DrawCircleLines((int)rcx,(int)rcy,(int)(rad-9.f),{70,85,105,150});
        Color tick={150,165,190,220};
        DrawLineEx({rcx,rcy-rad+5.f},{rcx,rcy-rad+14.f},3.f,C_HEADER);
        DrawLineEx({rcx,rcy+rad-5.f},{rcx,rcy+rad-14.f},3.f,tick);
        DrawLineEx({rcx-rad+5.f,rcy},{rcx-rad+14.f,rcy},3.f,tick);
        DrawLineEx({rcx+rad-5.f,rcy},{rcx+rad-14.f,rcy},3.f,tick);
        // North needle
        DrawTriangle({rcx,rcy-rad+17.f},{rcx-8.f,rcy-1.f},{rcx+8.f,rcy-1.f},C_HEADER);
        DrawCircle((int)rcx,(int)rcy,3.5f,{210,220,235,255});
        // Cardinal letters
        TC("N", rcx, rcy-rad-25.f, 19.f, {215,228,248,235});
        TC("S", rcx, rcy+rad+7.f, 17.f, {150,165,190,200});
        TW("E", rcx+rad+8.f, rcy-9.f, 17.f, {150,165,190,200});
        TW("W", rcx-rad-25.f, rcy-9.f, 17.f, {150,165,190,200});
    }
}

// =============================================================================
// LOCALIZE PANEL  — passenger-friendly "find the cart" step
// =============================================================================
void DrawLocalizePanel(int sw, int sh, Texture2D bg, Font font, AppState fus_state, bool& show_localize) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };

    const int TITLE_H=60, BAR_H=96;
    const int MAP_Y0=TITLE_H, MAP_Y1=sh-BAR_H, MAP_H=MAP_Y1-MAP_Y0;
    Vector2 mouse=GetMousePosition();
    float t=(float)GetTime();

    auto Button=[&](Rectangle r,const char* label,Color fill,Color border,Color tc,float fsz)->bool{
        bool hov=CheckCollisionPointRec(mouse,r);
        DrawRectangleRounded(r,0.28f,8,hov?ColorBrightness(fill,0.18f):fill);
        DrawRectangleRoundedLines(r,0.28f,8,1.5f,border);
        TW(label,r.x+(r.width-MW(label,fsz))*0.5f,r.y+(r.height-fsz)*0.5f,fsz,tc);
        return hov && IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
    };

    // ── Localization status snapshot ──
    bool running; float prog; std::string err; float score;
    { std::lock_guard<std::mutex> lk(g_reloc_mutex);
      running=g_reloc_running; prog=g_reloc_progress; err=g_reloc_error_msg; score=g_reloc.score; }
    bool calibrating=(fus_state==STATE_CALIBRATING);
    bool searching  =(running || fus_state==STATE_GLOBAL_RELOC);
    bool located    =(fus_state==STATE_FUSION_RUNNING);
    bool failed     =(!err.empty() && !searching && !located);
    Color sc = located?C_GOOD : failed?C_BAD : C_HEADER;

    // ── Background cover-fit transform (image pixel ↔ screen pixel) ──
    float iw=bg.id>0?(float)bg.width:1920.f;
    float ih=bg.id>0?(float)bg.height:1080.f;
    float bscale=bg.id>0?fmaxf((float)sw/iw,(float)sh/ih):1.f;
    float bgX=(sw-iw*bscale)*0.5f;
    float bgY=(sh-ih*bscale)*0.5f;

    // ====================================================================
    // CALIBRATION MODE — developer only
    // Overlay PLY cloud on campus photo; drag/scroll/[ ] to align.
    // Active map: active_map_ptr->cloud. Saves to localize_bg_<map>.cfg.
    // ====================================================================
    if(g_localize_cal_mode){
        // Init transform to image-centre defaults if not yet configured.
        // Scale is derived from the cloud's own world-space extent fitted to the image,
        // not a flat guess — a hardcoded px/m guess was wildly wrong for large campus
        // photos (search radius rendered far smaller than the real 12m it represents).
        if(!g_localize_bg_tf.valid){
            g_localize_bg_tf.offset_x = iw*0.5f;
            g_localize_bg_tf.offset_y = ih*0.5f;
            g_localize_bg_tf.scale    = 8.f;
            if(active_map_ptr && active_map_ptr->loaded && !active_map_ptr->cloud.empty()){
                float minx=1e9f,maxx=-1e9f,miny=1e9f,maxy=-1e9f;
                for(const auto& p:active_map_ptr->cloud){
                    if(p.x<minx)minx=p.x; if(p.x>maxx)maxx=p.x;
                    if(p.y<miny)miny=p.y; if(p.y>maxy)maxy=p.y;
                }
                float worldW=maxx-minx, worldH=maxy-miny;
                if(worldW>0.5f && worldH>0.5f){
                    float fitScale=fminf(iw/worldW, ih/worldH)*0.85f; // 85% fill, leaves margin
                    g_localize_bg_tf.scale=clampf(fitScale,0.05f,600.f);
                }
            }
            g_localize_bg_tf.setRotation(0.f);
            g_localize_bg_tf.valid = true;
        }

        bool in_map=(mouse.y>(float)MAP_Y0&&mouse.y<(float)MAP_Y1);
        // Guard: don't start drag over bottom-bar button zone
        bool over_bar=(mouse.y>(float)(MAP_Y1-4));

        // Helper: world point at screen centre — all rotate/scale ops anchor to this
        float scr_cx=(float)(sw/2), scr_cy=(float)((MAP_Y0+MAP_Y1)/2);
        auto anchorWorld=[&]()->Vector2{
            float ix=(scr_cx-bgX)/bscale, iy=(scr_cy-bgY)/bscale;
            return g_localize_bg_tf.imgToWorld(ix,iy);
        };
        auto reanchor=[&](Vector2 wc){
            Vector2 nip=g_localize_bg_tf.worldToImg(wc.x,wc.y);
            float tix=(scr_cx-bgX)/bscale, tiy=(scr_cy-bgY)/bscale;
            g_localize_bg_tf.offset_x+=(tix-nip.x);
            g_localize_bg_tf.offset_y+=(tiy-nip.y);
        };

        // ── Left-drag = pan ──
        static bool  cal_pan=false;
        static float cal_px0=0,cal_py0=0,cal_ox0=0,cal_oy0=0;
        if(in_map&&!over_bar&&IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
            cal_pan=true; cal_px0=mouse.x; cal_py0=mouse.y;
            cal_ox0=g_localize_bg_tf.offset_x; cal_oy0=g_localize_bg_tf.offset_y;
        }
        if(!IsMouseButtonDown(MOUSE_LEFT_BUTTON)) cal_pan=false;
        if(cal_pan){
            g_localize_bg_tf.offset_x=cal_ox0+(mouse.x-cal_px0)/bscale;
            g_localize_bg_tf.offset_y=cal_oy0+(mouse.y-cal_py0)/bscale;
        }

        // ── Right-drag = rotate (drag left/right; anchored to screen centre) ──
        static bool  cal_rot_drag=false;
        static float cal_rot_mx0=0, cal_rot0=0;
        if(in_map&&!over_bar&&IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)){
            cal_rot_drag=true; cal_rot_mx0=mouse.x; cal_rot0=g_localize_bg_tf.rotation;
        }
        if(!IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) cal_rot_drag=false;
        if(cal_rot_drag){
            float new_rot=cal_rot0+toRad((mouse.x-cal_rot_mx0)*0.25f); // 0.25 deg per pixel
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.setRotation(new_rot);
            reanchor(wc);
        }

        // ── Scroll = scale (anchored to screen centre) ──
        float wh=GetMouseWheelMove();
        if(wh!=0.f&&in_map){
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.scale=clampf(g_localize_bg_tf.scale*(wh>0?1.08f:0.926f),0.05f,600.f);
            reanchor(wc);
        }

        // ── Keyboard: hold [ ] for rotation, hold + - for fine scale ──
        // Using IsKeyDown so the rotation is CONTINUOUS while the key is held.
        float dt=GetFrameTime();
        float rot_rate=IsKeyDown(KEY_LEFT_SHIFT)?toRad(60.f):toRad(12.f); // deg/s
        if(IsKeyDown(KEY_LEFT_BRACKET)){
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.setRotation(g_localize_bg_tf.rotation+rot_rate*dt);
            reanchor(wc);
        }
        if(IsKeyDown(KEY_RIGHT_BRACKET)){
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.setRotation(g_localize_bg_tf.rotation-rot_rate*dt);
            reanchor(wc);
        }
        float sc_rate=IsKeyDown(KEY_LEFT_SHIFT)?1.5f:1.08f; // scale factor per second
        if(IsKeyDown(KEY_EQUAL)){
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.scale=clampf(g_localize_bg_tf.scale*powf(sc_rate,dt),0.05f,600.f);
            reanchor(wc);
        }
        if(IsKeyDown(KEY_MINUS)){
            Vector2 wc=anchorWorld();
            g_localize_bg_tf.scale=clampf(g_localize_bg_tf.scale/powf(sc_rate,dt),0.05f,600.f);
            reanchor(wc);
        }

        // ── Background ──
        BeginScissorMode(0,MAP_Y0,sw,MAP_H);
        if(bg.id>0) DrawTextureEx(bg,{bgX,bgY},0.f,bscale,WHITE);
        else DrawRectangle(0,MAP_Y0,sw,MAP_H,{18,24,38,255});
        DrawRectangle(0,MAP_Y0,sw,MAP_H,{0,0,0,80}); // light tint

        // ── Project PLY cloud (subsampled) ──
        if(active_map_ptr && active_map_ptr->loaded){
            const auto& cloud=active_map_ptr->cloud;
            const int STRIDE=5;
            for(size_t i=0;i<cloud.size();i+=STRIDE){
                Vector2 ip=g_localize_bg_tf.worldToImg(cloud[i].x,cloud[i].y);
                int sx=(int)(bgX+ip.x*bscale);
                int sy=(int)(bgY+ip.y*bscale);
                if(sx<1||sx>=sw-1||sy<MAP_Y0+1||sy>=MAP_Y1-1) continue;
                DrawPixel(sx,sy,{0,220,245,180});
            }
        }

        // World-origin cross (map 0,0)
        Vector2 orig_ip=g_localize_bg_tf.worldToImg(0.f,0.f);
        int ox=(int)(bgX+orig_ip.x*bscale), oy=(int)(bgY+orig_ip.y*bscale);
        DrawLineEx({(float)(ox-14),(float)oy},{(float)(ox+14),(float)oy},2.f,{255,220,0,220});
        DrawLineEx({(float)ox,(float)(oy-14)},{(float)ox,(float)(oy+14)},2.f,{255,220,0,220});
        DrawCircleLines(ox,oy,7,{255,220,0,220});

        // Controls hint overlay (top-left of map area)
        DrawRectangleRounded({12.f,(float)(MAP_Y0+8),316.f,162.f},0.08f,6,{6,10,18,210});
        TW("MAP ALIGNMENT CALIBRATION",    16.f,(float)(MAP_Y0+14),15.f,{255,220,80,220});
        TW("Left drag     -  pan",         16.f,(float)(MAP_Y0+36),13.f,{180,210,230,200});
        TW("Right drag    -  rotate",      16.f,(float)(MAP_Y0+54),13.f,{180,210,230,200});
        TW("Scroll        -  scale",       16.f,(float)(MAP_Y0+72),13.f,{180,210,230,200});
        TW("Hold [  ]     -  rotate slow", 16.f,(float)(MAP_Y0+90),13.f,{180,210,230,200});
        TW("Shift+[  ]    -  rotate fast", 16.f,(float)(MAP_Y0+108),13.f,{180,210,230,200});
        TW("Hold +  -     -  fine scale",  16.f,(float)(MAP_Y0+126),13.f,{180,210,230,200});
        TW("Yellow cross  -  world (0,0)", 16.f,(float)(MAP_Y0+144),13.f,{255,220,80,170});
        TW(TextFormat("Map: %s",active_map_ptr?active_map_ptr->name.c_str():"none"),
           16.f,(float)(MAP_Y0+162),12.f,{140,160,185,180});

        EndScissorMode();

        // ── Title bar ──
        DrawRectangle(0,0,sw,TITLE_H,{14,19,33,255});
        DrawRectangle(0,TITLE_H-3,sw,3,{220,160,30,200});
        TC("Map Calibration  -  Align the point cloud to the campus photo",
           (float)(sw/2),(TITLE_H-22)*0.5f,20,{255,220,80,240});

        // ── Bottom bar ──
        DrawRectangle(0,MAP_Y1,sw,BAR_H,{12,16,28,255});
        DrawRectangle(0,MAP_Y1,sw,3,{220,160,30,160});
        float barCY=MAP_Y1+BAR_H*0.5f;

        // Transform readout (top of bar)
        float rx_deg=g_localize_bg_tf.rotation*180.f/kPI;
        TC(TextFormat("offset (%.0f, %.0f) px     scale %.2f px/m     rot %.1f deg",
                      (double)g_localize_bg_tf.offset_x,(double)g_localize_bg_tf.offset_y,
                      (double)g_localize_bg_tf.scale,(double)rx_deg),
           (float)(sw/2), barCY-30.f, 13.f, {190,205,220,220});

        // Done (left)
        if(Button({24.f,barCY-22.f,110.f,44.f},"Done",{38,42,56,220},{100,100,120,160},{210,215,225,230},17.f))
            g_localize_cal_mode=false;

        // Rotation buttons (centre-left) — each click = 5 deg
        float rb=barCY-20.f, rh=40.f, rw=90.f;
        float mid=(float)(sw/2);
        if(Button({mid-rw-4.f,rb,rw,rh},"< CCW 5",{30,50,100,220},{80,110,160,160},{170,200,235,230},14.f)){
            Vector2 wc=anchorWorld(); g_localize_bg_tf.setRotation(g_localize_bg_tf.rotation+toRad(5.f)); reanchor(wc);
        }
        if(Button({mid+4.f,rb,rw,rh},"CW 5 >",{30,50,100,220},{80,110,160,160},{170,200,235,230},14.f)){
            Vector2 wc=anchorWorld(); g_localize_bg_tf.setRotation(g_localize_bg_tf.rotation-toRad(5.f)); reanchor(wc);
        }

        // Save (right)
        if(Button({(float)(sw-190-24),barCY-25.f,190.f,50.f},"Save Calibration",{30,140,80,255},WHITE,WHITE,18.f)){
            save_localize_bg_transform();
            g_localize_cal_mode=false;
        }

        return; // skip the passenger UI entirely while in cal mode
    }
    // ====================================================================
    // END CALIBRATION MODE
    // ====================================================================

    // ── Draggable search circle (static state — reset when panel opens via g_localize_panel_needs_init) ──
    static float circ_sx=0.f, circ_sy=0.f;
    static bool  circ_init=false;
    static bool  circ_drag=false;

    bool tf=g_localize_bg_tf.valid;
    if(g_localize_panel_needs_init){ circ_init=false; g_localize_panel_needs_init=false; }

    // Helper: convert a world (x,y) to screen pixel using either the precise calibrated
    // bg transform (tf) or a rough cloud-bounds proportional fallback when not yet calibrated.
    // The fallback maps the map's world extent onto the screen viewport proportionally with
    // Y-flipped (north-up), giving the circle a meaningful starting position even pre-calibration.
    auto worldToScreen=[&](float wx, float wy)->Vector2{
        if(tf){
            Vector2 ip=g_localize_bg_tf.worldToImg(wx,wy);
            return {bgX+ip.x*bscale, bgY+ip.y*bscale};
        }
        if(active_map_ptr && active_map_ptr->loaded && !active_map_ptr->cloud.empty()){
            float minx=1e9f,maxx=-1e9f,miny=1e9f,maxy=-1e9f;
            for(const auto& p:active_map_ptr->cloud){
                if(p.x<minx)minx=p.x; if(p.x>maxx)maxx=p.x;
                if(p.y<miny)miny=p.y; if(p.y>maxy)maxy=p.y;
            }
            float worldW=maxx-minx, worldH=maxy-miny;
            if(worldW>0.5f && worldH>0.5f){
                float tx=(wx-minx)/worldW;
                float ty=1.f-(wy-miny)/worldH;  // Y-flip: north-up
                return {(float)sw*clampf(tx,0.05f,0.95f),
                        (float)MAP_Y0+(float)(MAP_Y1-MAP_Y0)*clampf(ty,0.05f,0.95f)};
            }
        }
        return {(float)sw*0.5f, (float)(MAP_Y0+MAP_Y1)*0.5f};
    };

    if(!circ_init){
        circ_init=true;
        if(g_est_start.valid){
            // Precise configured start (Step 2 → Set Start Point)
            Vector2 sp=worldToScreen(g_est_start.x, g_est_start.y);
            circ_sx=sp.x; circ_sy=sp.y;
        } else if(tf){
            // Calibration saved but no eststart: default to world origin (0,0).
            // For top-LiDAR campus maps the recording starts at the lab parking area,
            // so (0,0) is exactly where the cart parks — matches the yellow cross in cal mode.
            Vector2 sp=worldToScreen(0.f, 0.f);
            circ_sx=sp.x; circ_sy=sp.y;
        } else if(g_zone_valid && !g_zone_poly.empty()){
            // Zone centroid rough fallback (no calibration, no eststart)
            float cx2,cy2,r2;
            zone_centroid_radius(g_zone_poly,cx2,cy2,r2);
            Vector2 sp=worldToScreen(cx2,cy2);
            circ_sx=sp.x; circ_sy=sp.y;
        } else if(g_zone2_valid && !g_zone2_poly.empty()){
            float cx2,cy2,r2;
            zone_centroid_radius(g_zone2_poly,cx2,cy2,r2);
            Vector2 sp=worldToScreen(cx2,cy2);
            circ_sx=sp.x; circ_sy=sp.y;
        } else {
            circ_sx=(float)sw*0.5f; circ_sy=(float)(MAP_Y0+MAP_Y1)*0.5f;
        }
    }
    // Clamp to visible map area
    circ_sx=clampf(circ_sx,80.f,(float)(sw-80));
    circ_sy=clampf(circ_sy,(float)MAP_Y0+50.f,(float)MAP_Y1-50.f);

    // Drag: only allowed when not searching / not yet located
    bool can_drag=!searching && !located && !calibrating;
    const float CIRC_R=32.f;
    bool hov_circ=can_drag && CheckCollisionPointCircle(mouse,{circ_sx,circ_sy},CIRC_R+16.f);
    if(can_drag && hov_circ && IsMouseButtonPressed(MOUSE_LEFT_BUTTON))  circ_drag=true;
    if(!IsMouseButtonDown(MOUSE_LEFT_BUTTON))                             circ_drag=false;
    if(circ_drag){
        circ_sx=mouse.x;
        circ_sy=clampf(mouse.y,(float)MAP_Y0+30.f,(float)MAP_Y1-30.f);
    }

    // When located: smoothly move circle to actual EKF position (if transform available)
    if(located && tf){
        float ex_w, ey_w;
        { std::lock_guard<std::mutex>lk2(ekf_mutex); ex_w=(float)ekf.x[0]; ey_w=(float)ekf.x[1]; }
        Vector2 ip=g_localize_bg_tf.worldToImg(ex_w,ey_w);
        float ex=bgX+ip.x*bscale, ey=bgY+ip.y*bscale;
        circ_sx+=( ex-circ_sx)*0.06f;
        circ_sy+=(ey-circ_sy)*0.06f;
    }

    // Search radius in screen pixels — same physical radius as the large-map (outdoor)
    // particle-filter seed circle (SEED_RADIUS_OUTDOOR, drawn in Step 1's 3D view).
    // Clamp is just a usability floor/ceiling, not a scale override.
    float seed_r_m=active_map_ptr?(active_map_ptr->is_large?SEED_RADIUS_OUTDOOR:SEED_RADIUS_INDOOR):12.f;
    float ring_r=tf?(seed_r_m*g_localize_bg_tf.scale*bscale):72.f;
    ring_r=clampf(ring_r,24.f,520.f);

    // ── MAP BACKGROUND ──
    BeginScissorMode(0,MAP_Y0,sw,MAP_H);
    if(bg.id>0)
        DrawTextureEx(bg,{bgX,bgY},0.f,bscale,WHITE);
    else
        DrawRectangle(0,MAP_Y0,sw,MAP_H,{18,24,38,255});
    // Darker overlay so circle pops
    DrawRectangle(0,MAP_Y0,sw,MAP_H,{0,0,0,130});

    // ── SEARCH CIRCLE ──
    if(!located){
        // Filled search zone (semi-transparent)
        DrawCircle((int)circ_sx,(int)circ_sy,(int)ring_r,{sc.r,sc.g,sc.b,28});
        // Outer ring
        DrawCircleLines((int)circ_sx,(int)circ_sy,(int)ring_r,{sc.r,sc.g,sc.b,130});

        if(searching){
            // Expanding pulse rings radiating outward from circle center
            for(int i=0;i<3;i++){
                float phase=fmodf(t*0.8f + i*0.33f, 1.0f);
                float pr=ring_r*phase;
                unsigned char pa=(unsigned char)(160*(1.f-phase));
                DrawCircleLines((int)circ_sx,(int)circ_sy,pr,{sc.r,sc.g,sc.b,pa});
            }
            // Rotating sweep line from center
            float ang=t*2.6f;
            DrawLineEx({circ_sx,circ_sy},
                       {circ_sx+cosf(ang)*ring_r, circ_sy+sinf(ang)*ring_r},
                       2.5f,{sc.r,sc.g,sc.b,170});
        }

        // Crosshair inside circle
        float ch=CIRC_R*0.9f;
        DrawLineEx({circ_sx-ch,circ_sy},{circ_sx+ch,circ_sy},1.5f,{sc.r,sc.g,sc.b,150});
        DrawLineEx({circ_sx,circ_sy-ch},{circ_sx,circ_sy+ch},1.5f,{sc.r,sc.g,sc.b,150});
        DrawCircle((int)circ_sx,(int)circ_sy,7,sc);

        // Drag affordance hint
        if(can_drag && (hov_circ||circ_drag))
            TC("Drag to move search area", circ_sx, circ_sy+ring_r+9.f, 15.f, {200,215,230,200});
        else if(can_drag && !searching)
            TC("Drag the circle to where the cart is parked",
               circ_sx, circ_sy+ring_r+9.f, 14.f, {170,185,205,150});

        // World-coordinate overlay (developer reference — subtle)
        if(tf){
            float ix=(circ_sx-bgX)/bscale, iy=(circ_sy-bgY)/bscale;
            Vector2 wp=g_localize_bg_tf.imgToWorld(ix,iy);
            DrawText(TextFormat("(%.0f, %.0f) m",(double)wp.x,(double)wp.y),
                (int)circ_sx+10,(int)circ_sy-20,11,{120,155,185,90});
        }
    } else {
        // Located: green pin + checkmark at confirmed position
        DrawCircle((int)circ_sx,(int)circ_sy,20,{0,0,0,120});
        DrawCircle((int)circ_sx,(int)circ_sy,16,C_GOOD);
        DrawLineEx({circ_sx-8,circ_sy+1},{circ_sx-1,circ_sy+8},3.5f,WHITE);
        DrawLineEx({circ_sx-1,circ_sy+8},{circ_sx+10,circ_sy-7},3.5f,WHITE);
    }
    EndScissorMode();

    // ── LIVE SCAN INSET (top-right corner) ──
    // Appears once user presses "Find My Cart" (searching) and stays until they press "Continue".
    if(searching || located){
        const float INS_W=340.f, INS_H=260.f, INS_PAD=16.f;
        const float INS_X=(float)sw-INS_PAD-INS_W, INS_Y=(float)MAP_Y0+INS_PAD;

        // Best world-centre estimate: EKF when located, top particle when searching, seed/est-start otherwise
        float cx=0.f, cy=0.f, yaw_est=0.f;
        bool have_pose=false;
        if(located){
            std::lock_guard<std::mutex>lk_e(ekf_mutex);
            cx=(float)ekf.x[0]; cy=(float)ekf.x[1]; yaw_est=(float)ekf.x[2]; have_pose=true;
        } else {
            std::lock_guard<std::mutex>lk_r(g_reloc_mutex);
            if(!g_reloc_particles.empty()){
                cx=g_reloc_particles[0].x; cy=g_reloc_particles[0].y;
                yaw_est=g_reloc_particles[0].yaw; have_pose=true;
            }
        }
        if(!have_pose){
            if(g_est_start.valid){ cx=g_est_start.x; cy=g_est_start.y; yaw_est=g_est_start.yaw; }
            else if(g_localize_use_custom_seed){ cx=g_localize_seed_x; cy=g_localize_seed_y; }
        }

        // Viewport spans 2× the outdoor seed radius so the full search circle fits on screen
        float span_m=(active_map_ptr&&active_map_ptr->is_large)?
                      2.f*SEED_RADIUS_OUTDOOR : 2.f*SEED_RADIUS_INDOOR;
        float ls=fminf(INS_W,INS_H)/span_m;  // px/m inside inset

        auto w2sI=[&](float wx, float wy)->Vector2{
            return {INS_X+INS_W*0.5f+(wx-cx)*ls,
                    INS_Y+INS_H*0.5f-(wy-cy)*ls};
        };

        BeginScissorMode((int)INS_X,(int)INS_Y,(int)INS_W,(int)INS_H);
        DrawRectangle((int)INS_X,(int)INS_Y,(int)INS_W,(int)INS_H,{6,10,22,235});

        // Map cloud — dim grey dots, stride 6, world-space culled
        float vis_r=span_m*0.7f;
        if(active_map_ptr&&active_map_ptr->loaded){
            const auto& mc=active_map_ptr->cloud;
            for(size_t i=0;i<mc.size();i+=6){
                float dx=mc[i].x-cx, dy=mc[i].y-cy;
                if(dx*dx+dy*dy>vis_r*vis_r) continue;
                Vector2 sp=w2sI(mc[i].x,mc[i].y);
                DrawPixel((int)sp.x,(int)sp.y,{80,100,130,170});
            }
        }

        // Live LiDAR scan transformed to world frame (same pattern as Step-1 3D draw)
        {   float cosy=cosf(yaw_est), siny=sinf(yaw_est);
            std::lock_guard<std::mutex>lk_l(lidar_mutex);
            for(size_t i=0;i<dense_cloud.size();i+=2){
                const auto& p=dense_cloud[i];
                float wx=cosy*p.x-siny*p.y+cx, wy=siny*p.x+cosy*p.y+cy;
                float dx=wx-cx, dy=wy-cy;
                if(dx*dx+dy*dy>vis_r*vis_r) continue;
                Vector2 sp=w2sI(wx,wy);
                DrawPixel((int)sp.x,(int)sp.y,{0,215,245,200});
            }
        }

        // Particle-filter candidates — green dots during global relocalization.
        // Snapshot under the lock, draw without it (see the Step-1 note): holding
        // g_reloc_mutex through thousands of draws stalls the reloc thread.
        if(searching){
            static std::vector<Particle> snap2;
            { std::lock_guard<std::mutex>lk_r2(g_reloc_mutex); snap2 = g_reloc_particles; }
            for(size_t i=0;i<snap2.size();i+=4){
                float dx=snap2[i].x-cx, dy=snap2[i].y-cy;
                if(dx*dx+dy*dy>vis_r*vis_r) continue;
                Vector2 sp=w2sI(snap2[i].x,snap2[i].y);
                DrawCircle((int)sp.x,(int)sp.y,2,{0,255,100,150});
            }
        }

        // Centre marker (current robot/seed position)
        Vector2 cpt=w2sI(cx,cy);
        DrawLineEx({cpt.x-7,cpt.y},{cpt.x+7,cpt.y},1.5f,{255,220,40,180});
        DrawLineEx({cpt.x,cpt.y-7},{cpt.x,cpt.y+7},1.5f,{255,220,40,180});
        DrawCircle((int)cpt.x,(int)cpt.y,4,{255,220,40,230});

        EndScissorMode();

        // Card border + header label (outside scissor so not clipped)
        DrawRectangleRoundedLines({INS_X,INS_Y,INS_W,INS_H},0.06f,6,1.5f,{sc.r,sc.g,sc.b,130});
        TW("LIVE SCAN", INS_X+8.f, INS_Y+5.f, 12.f, {sc.r,sc.g,sc.b,200});
        DrawCircle((int)(INS_X+INS_W-10.f),(int)(INS_Y+9.f),4,sc);
        TW(located?"Located":searching?"Searching...":"Idle",
           INS_X+8.f, INS_Y+INS_H-18.f, 11.f, {sc.r,sc.g,sc.b,170});
    }

    // ── STATUS CARD (bottom-right corner of map area) — sized up ~44% 2026-07-27
    // (user request, matches the readability pass applied to the other cards) ──
    float sW=518.f, sH=located?153.f:searching?199.f:144.f;
    float sX=(float)(sw-20)-sW, sY=(float)MAP_Y1-sH-16.f;
    DrawRectangleRounded({sX,sY,sW,sH},0.12f,8,{8,12,22,225});
    DrawRectangleRoundedLines({sX,sY,sW,sH},0.12f,8,1.5f,{sc.r,sc.g,sc.b,100});

    const char* head; const char* sub;
    if(located)         { head="Cart located!";            sub=TextFormat("Position locked - %d%% map match",(int)(score*100)); }
    else if(searching)  { head="Searching for the cart...";sub="Matching LiDAR against the campus map"; }
    else if(failed)     { head="Could not find the cart";  sub="Make sure the cart is in a mapped area"; }
    else if(calibrating){ head="Warming up sensors...";    sub="Please wait a moment"; }
    else                { head="Ready to search";          sub=tf?"Drag circle to the cart, then Find My Cart":"Press Find My Cart below"; }
    TW(head, sX+20.f, sY+19.f, 29.f, sc);
    TW(sub,  sX+20.f, sY+58.f, 20.f, {180,195,210,200});
    if(searching){
        float bw=sW-40.f, bx=sX+20.f, by=sY+101.f;
        DrawRectangleRounded({bx,by,bw,16},0.5f,4,{22,32,48,255});
        DrawRectangleRounded({bx,by,bw*fmaxf(0.02f,prog),16},0.5f,4,sc);
        TW(TextFormat("%d%%",(int)(prog*100)), bx, by+22.f, 17.f, {160,185,205,170});
    }

    // ── TITLE BAR ──
    DrawRectangle(0,0,sw,TITLE_H,{14,19,33,255});
    DrawRectangle(0,TITLE_H-3,sw,3,{60,120,220,190});
    TC("Where is the cart?", (float)(sw/2), (TITLE_H-28)*0.5f, 28, WHITE);

    // ── BOTTOM BAR ──
    DrawRectangle(0,MAP_Y1,sw,BAR_H,{12,16,28,255});
    DrawRectangle(0,MAP_Y1,sw,3,{60,120,220,160});
    float barCY=MAP_Y1+BAR_H*0.5f;

    // Back (left) -> intro
    Rectangle backBtn={24.f,barCY-24.f,120.f,48.f};
    if(Button(backBtn,"Back",{38,38,42,220},{120,120,120,170},{210,210,210,230},17.f)){
        show_localize=false; g_show_intro=true;
    }

    // TEMP: Skip localization (testing only - remove before deployment)
    Rectangle skipBtn={154.f,barCY-24.f,140.f,48.f};
    if(Button(skipBtn,"Skip (test)",{50,40,16,220},{200,160,60,160},{220,190,120,230},16.f)){
        show_localize=false; g_show_dest=true;
    }

    // Developer: enter calibration mode (aligns PLY cloud to background image)
    Rectangle calBtn={304.f,barCY-22.f,150.f,44.f};
    if(Button(calBtn,"Calibrate Map",{28,32,52,220},{80,100,140,150},{160,180,210,220},14.f)){
        g_localize_cal_mode=true;
    }

    // Centre hint text
    if(located)
        TC("Cart found - press Continue to choose your destination",
           (float)(sw/2), barCY-9.f, 17, {150,255,170,220});
    else if(searching)
        TC("Searching the campus map...", (float)(sw/2), barCY-9.f, 17, {160,185,210,200});
    else if(!calibrating)
        TC("Drag the ring to the cart location, then press Find My Cart",
           (float)(sw/2), barCY-9.f, 16, {185,195,210,200});

    // Primary action (right)
    int bw=260,bh=56;
    Rectangle goBtn={(float)(sw-bw-24),barCY-bh*0.5f,(float)bw,(float)bh};
    if(located){
        if(Button(goBtn,"Continue",{20,160,70,255},WHITE,WHITE,21.f)){
            show_localize=false; g_show_dest=true;
        }
    } else if(searching||calibrating){
        DrawRectangleRounded(goBtn,0.28f,8,{40,44,54,200});
        DrawRectangleRoundedLines(goBtn,0.28f,8,1.5f,{70,70,80,180});
        TC(searching?"Searching...":"Please wait...",
           goBtn.x+goBtn.width*0.5f, goBtn.y+(goBtn.height-20)*0.5f, 20, {120,140,160,200});
    } else {
        const char* lbl=failed?"Try Again":"Find My Cart";
        Color gf=failed?Color{175,65,45,255}:Color{20,150,200,255};
        if(Button(goBtn,lbl,gf,WHITE,WHITE,21.f)){
            if(tf && circ_init){
                // Calibrated: convert dragged circle screen position → world seed
                float ix=(circ_sx-bgX)/bscale, iy=(circ_sy-bgY)/bscale;
                Vector2 wp=g_localize_bg_tf.imgToWorld(ix,iy);
                g_localize_seed_x=wp.x; g_localize_seed_y=wp.y;
                g_localize_seed_r=seed_r_m;
                g_localize_use_custom_seed=true;
            } else if(g_est_start.valid){
                // No bg calibration: seed directly at configured est_start world position
                g_localize_seed_x=g_est_start.x; g_localize_seed_y=g_est_start.y;
                g_localize_seed_r=seed_r_m;
                g_localize_use_custom_seed=true;
            } else if(tf){
                // Calibration saved but no eststart: seed at world (0,0) — recording start
                // = lab parking area for forward map (same as the yellow cross in cal mode)
                g_localize_seed_x=0.f; g_localize_seed_y=0.f;
                g_localize_seed_r=seed_r_m;
                g_localize_use_custom_seed=true;
            }
            // If nothing: falls back to zone centroid / EKF in main.cpp
            g_localize_request=true;
        }
    }
}



// =============================================================================
// DESTINATION PANEL
// =============================================================================
void DrawDestinationPanel(int sw, int sh, Texture2D bg, Font font, bool& show_dest, AppState fus_state) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };

    // Layout constants
    const int TITLE_H = 60;   // title bar height
    const int PICK_H  = 44;   // destination-picker strip (admin only)
    const int BAR_H   = 96;   // bottom button bar height
    const int MAP_Y0  = TITLE_H + (g_pax_grid_edit ? PICK_H : 0);
    const int MAP_Y1  = sh - BAR_H;
    const int MAP_H   = MAP_Y1 - MAP_Y0;

    Vector2 mouse = GetMousePosition();

    // Background cover-fit transform — the grid is anchored to the campus IMAGE
    // (a fixed PAX_COLS×PAX_ROWS over normalized 0..1), NOT to screen pixels, so a
    // cell always covers the same campus spot regardless of window size.
    float iw = bg.id>0 ? (float)bg.width  : (float)sw;
    float ih = bg.id>0 ? (float)bg.height : (float)sh;
    float bscale = bg.id>0 ? fmaxf((float)sw/iw,(float)sh/ih) : 1.f;
    float bgX = (sw - iw*bscale)*0.5f;
    float bgY = (sh - ih*bscale)*0.5f;
    float cellW = (iw*bscale)/(float)PAX_COLS;   // cell width  in screen px
    float cellH = (ih*bscale)/(float)PAX_ROWS;   // cell height in screen px
    auto cellRect=[&](int c,int r)->Rectangle{ return { bgX+c*cellW, bgY+r*cellH, cellW, cellH }; };

    int mc = (int)floorf((mouse.x - bgX)/cellW);
    int mr = (int)floorf((mouse.y - bgY)/cellH);
    bool inMap = (mouse.y >= MAP_Y0 && mouse.y < MAP_Y1);

    // Reusable rounded button → returns true on click
    auto Button=[&](Rectangle r,const char* label,Color fill,Color border,Color tc,float fsz)->bool{
        bool hov=CheckCollisionPointRec(mouse,r);
        DrawRectangleRounded(r,0.28f,8,hov?ColorBrightness(fill,0.18f):fill);
        DrawRectangleRoundedLines(r,0.28f,8,1.5f,border);
        TW(label,r.x+(r.width-MW(label,fsz))*0.5f,r.y+(r.height-fsz)*0.5f,fsz,tc);
        return hov && IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
    };

    // Persistent state: passenger selection + per-tap feedback
    static int  sel_col=-1, sel_row=-1;  // confirmed green selection
    static int  sel_dest=-1;             // resolved destination index for sel_col/row
    static int  fb_col=-1,  fb_row=-1;   // last tap-feedback cell
    static int  fb_type=-1;              // 0=green flash, 1=red flash
    static float fb_timer=0.f;
    static float msg_timer=0.f;
    static bool  msg_unreachable=false;  // which "not available" message to show

    // ── MAP AREA — satellite image ────────────────────────────────────────────
    BeginScissorMode(0, MAP_Y0, sw, MAP_H);
    if(bg.id > 0) {
        float s = fmaxf((float)sw/bg.width, (float)sh/bg.height);
        DrawTextureEx(bg, {(sw-bg.width*s)*0.5f,(sh-bg.height*s)*0.5f}, 0.f, s, WHITE);
    } else {
        DrawRectangle(0,MAP_Y0,sw,MAP_H,{18,24,38,255});
    }
    DrawRectangle(0,MAP_Y0,sw,MAP_H,{0,0,0,70}); // slight dark overlay

    // Grid lines — only in admin edit mode (scissor clips them to the map area)
    if(g_pax_grid_edit) {
        Color gcMinor = {255,255,255,40};
        for(int c=0;c<=PAX_COLS;c++){ int x=(int)(bgX+c*cellW); DrawLine(x,MAP_Y0,x,MAP_Y1,gcMinor); }
        for(int r=0;r<=PAX_ROWS;r++){ int y=(int)(bgY+r*cellH); DrawLine(0,y,sw,y,gcMinor); }
    }

    if(g_pax_grid_edit) {
        // ── ADMIN: show all painted cells, colored by assigned destination ──
        // (or a distinct dark-red hatch for cells explicitly marked unreachable)
        for(auto& kv : g_pax_grid) {
            int col=kv.first.first, row=kv.first.second, di=kv.second;
            if(di!=PAX_UNREACHABLE && (di<0||di>=NUM_DEST_TOTAL)) continue;
            Rectangle cr=cellRect(col,row);
            if(cr.x>=sw||cr.y>=MAP_Y1||cr.x+cr.width<=0||cr.y+cr.height<=MAP_Y0) continue;
            bool isHov = inMap && mc==col && mr==row;
            if(di==PAX_UNREACHABLE){
                Color uc={160,60,20,255};
                DrawRectangleRec(cr,{uc.r,uc.g,uc.b,(unsigned char)(isHov?150:85)});
                DrawRectangleLinesEx(cr,1.f,{uc.r,uc.g,uc.b,(unsigned char)(isHov?235:170)});
                DrawLineEx({cr.x,cr.y},{cr.x+cr.width,cr.y+cr.height},1.f,{255,180,120,160});
                DrawLineEx({cr.x+cr.width,cr.y},{cr.x,cr.y+cr.height},1.f,{255,180,120,160});
                continue;
            }
            Color base = combinedDestColor(di);
            DrawRectangleRec(cr,{base.r,base.g,base.b,(unsigned char)(isHov?150:85)});
            DrawRectangleLinesEx(cr,1.f,{base.r,base.g,base.b,(unsigned char)(isHov?235:170)});
        }
        if(inMap) DrawRectangleLinesEx(cellRect(mc,mr),1.5f,{255,255,255,90});
    } else {
        // ── PASSENGER: plain map — only show results of taps ─────────────
        if(inMap)
            DrawRectangleRec(cellRect(mc,mr),{255,255,255,18});

        if(fb_timer>0.f) {
            fb_timer -= GetFrameTime();
            float a = fminf(1.f, fb_timer);
            Rectangle cr=cellRect(fb_col,fb_row);
            if(fb_type==0) {
                DrawRectangleRec(cr,{20,200,70,(unsigned char)(200*a)});
                DrawRectangleLinesEx(cr,3.f,{140,255,160,(unsigned char)(255*a)});
            } else {
                DrawRectangleRec(cr,{220,30,30,(unsigned char)(190*a)});
                DrawRectangleLinesEx(cr,3.f,{255,60,60,(unsigned char)(255*a)});
                float mx=cr.width*0.25f, my=cr.height*0.25f;
                DrawLineEx({cr.x+mx,cr.y+my},{cr.x+cr.width-mx,cr.y+cr.height-my},3.f,{255,80,80,(unsigned char)(220*a)});
                DrawLineEx({cr.x+cr.width-mx,cr.y+my},{cr.x+mx,cr.y+cr.height-my},3.f,{255,80,80,(unsigned char)(220*a)});
            }
        }

        if(sel_col>=0) {
            Rectangle cr=cellRect(sel_col,sel_row);
            DrawRectangleRec(cr,{20,200,70,200});
            DrawRectangleLinesEx(cr,3.f,{140,255,160,255});
            float cx=cr.x+cr.width*0.5f, cy=cr.y+cr.height*0.5f;
            DrawLineEx({cx-14,cy+2},{cx-4,cy+13},4.f,WHITE);
            DrawLineEx({cx-4,cy+13},{cx+16,cy-10},4.f,WHITE);
        }
    }

    // "Not available" fade message (centred in map area) — two variants: an explicitly
    // marked-unreachable cell (PAX_UNREACHABLE) gets a message stating the cart simply
    // can't go there, distinct from the generic "nothing set here yet" message for an
    // ordinary un-painted tap.
    if(msg_timer>0.f) {
        msg_timer -= GetFrameTime();
        float a = fminf(1.f, msg_timer);
        const char* m1 = msg_unreachable ? "The cart cannot reach this area."
                                          : "This area is not available right now.";
        const char* m2 = msg_unreachable ? "Please choose a reachable area."
                                          : "Please choose another location.";
        // Sized up 2026-07-27 (user request — easier to read at a glance): font
        // 22/18 -> 27/21 -> 32/25 (second +20% pass), padding/height scaled to match.
        float w1=MW(m1,32), w2=MW(m2,25);
        int bw=(int)fmaxf(w1,w2)+79, bh=130;
        int bx=sw/2-bw/2, by=MAP_Y0+MAP_H/2-bh/2;
        // Color distinguishes WHY the tap failed (2026-07-27, user request): red =
        // genuinely unreachable (msg_unreachable, painted -2 by admin), blue = just
        // an ordinary un-painted/not-currently-offered cell — different meanings,
        // shouldn't look identical.
        Color bgCol   = msg_unreachable ? Color{50,0,0,(unsigned char)(215*a)}   : Color{0,15,50,(unsigned char)(215*a)};
        Color lineCol = msg_unreachable ? Color{255,70,70,(unsigned char)(230*a)} : Color{70,150,255,(unsigned char)(230*a)};
        Color m1Col   = msg_unreachable ? Color{255,120,120,(unsigned char)(255*a)} : Color{140,190,255,(unsigned char)(255*a)};
        Color m2Col   = msg_unreachable ? Color{225,160,160,(unsigned char)(225*a)} : Color{170,205,240,(unsigned char)(225*a)};
        DrawRectangleRounded({(float)bx,(float)by,(float)bw,(float)bh},0.22f,8,bgCol);
        DrawRectangleRoundedLines({(float)bx,(float)by,(float)bw,(float)bh},0.22f,8,2.f,lineCol);
        TC(m1,(float)(sw/2),(float)(by+26),32,m1Col);
        TC(m2,(float)(sw/2),(float)(by+74),25,m2Col);
    }

    EndScissorMode();

    // ── CLICK HANDLING (map area only) ───────────────────────────────────────
    if(IsMouseButtonReleased(MOUSE_LEFT_BUTTON) && inMap) {
        auto key=std::make_pair(mc,mr);
        if(g_pax_grid_edit) {
            if(g_pax_paint_dest==-1) {
                g_pax_grid.erase(key);
            } else {
                auto it=g_pax_grid.find(key);
                if(it!=g_pax_grid.end() && it->second==g_pax_paint_dest)
                    g_pax_grid.erase(key);           // tap same brush again = remove
                else
                    g_pax_grid[key]=g_pax_paint_dest;
            }
        } else {
            auto it=g_pax_grid.find(key);
            int di = (it!=g_pax_grid.end()) ? it->second : -1;
            const char* dname=nullptr; SavedDest resolved;
            bool allowed = (di>=0 && resolve_combined_dest(di, dname, resolved));
            fb_col=mc; fb_row=mr;
            if(allowed) {
                fb_type=0; fb_timer=0.8f;
                sel_col=mc; sel_row=mr; sel_dest=di;
                msg_timer=0.f;
            } else {
                fb_type=1; fb_timer=1.2f;
                msg_timer=3.5f;
                msg_unreachable=(di==PAX_UNREACHABLE);
                sel_col=-1; sel_row=-1; sel_dest=-1;
                // Diagnostic (2026-07-08): a painted-but-invalid zone-2 cell has been seen
                // to reject EVERY zone-2 destination after a danger-zone crossing, only
                // fixed by restarting the app — root cause not yet confirmed. Logs the live
                // state to the Step 3 command log so the next occurrence is diagnosable
                // instead of guessed at. di>=NUM_DEST && di<PAX cases only (a real painted
                // destination cell that still failed .valid — excludes plain unpainted taps).
                if(di>=NUM_DEST && di<NUM_DEST_TOTAL){
                    int validCount=0; for(auto& d:g_destinations2) if(d.valid) validCount++;
                    cmdlog_push(TextFormat(
                        "[DIAG] zone2 dest #%d rejected | map=%s zone1_armed=%d zone2_armed=%d z2valid=%d/%d",
                        di-NUM_DEST, active_map_ptr?active_map_ptr->name.c_str():"NULL",
                        (int)g_zone_armed,(int)g_zone2_armed,validCount,NUM_DEST2));
                }
            }
        }
    }

    bool hasSel = (!g_pax_grid_edit && sel_col>=0 && sel_dest>=0);

    // ── TITLE BAR ────────────────────────────────────────────────────────────
    DrawRectangle(0,0,sw,TITLE_H,{14,19,33,255});
    DrawRectangle(0,TITLE_H-3,sw,3, g_pax_grid_edit?Color{210,170,40,200}:Color{60,120,220,190});
    {
        Color accent = g_pax_grid_edit?Color{255,205,60,255}:WHITE;
        const char* title = g_pax_grid_edit ? "ADMIN  -  Set Allowed Areas"
                                             : "Where would you like to go?";
        TC(title, (float)(sw/2), (TITLE_H-28)*0.5f, 28, accent);
    }
    // Background view toggle — top-left (Satellite ↔ 3D)
    {
        Rectangle viewBtn={16.f,(float)((TITLE_H-34)/2),150.f,34.f};
        const char* vl = g_bg_show_3d ? "View:  3D" : "View:  Satellite";
        if(Button(viewBtn,vl,{24,28,48,220},{90,100,150,180},{150,165,210,230},15.f))
            g_bg_show_3d = !g_bg_show_3d;
    }
    // Admin toggle — top-right (passenger mode only)
    if(!g_pax_grid_edit) {
        Rectangle adminBtn={(float)(sw-128),(float)((TITLE_H-34)/2),112.f,34.f};
        if(Button(adminBtn,"Admin",{24,28,48,220},{90,100,150,180},{150,165,210,230},15.f))
            g_pax_grid_edit=true;
        // Jumps to the technical Step 2 "Set Destinations on Map" screen (pick/preview/
        // confirm flow) — the only place destination world coords actually get set.
        // "Return to Passenger View" there comes back here.
        Rectangle destBtn={(float)(sw-128-16-140),(float)((TITLE_H-34)/2),140.f,34.f};
        if(Button(destBtn,"Set Destinations",{24,28,48,220},{90,100,150,180},{150,165,210,230},15.f)){
            show_dest = false;
            g_ui_step.store(STEP_PLAN);
        }
    }

    // ── DESTINATION PICKER STRIP (admin only) — choose the paint brush ───────
    // Combined index: 0..NUM_DEST-1 = zone 1, NUM_DEST..NUM_DEST_TOTAL-1 = zone 2.
    // Two special brushes below NUM_DEST_TOTAL: PAX_UNREACHABLE (-2) marks cells the
    // cart genuinely cannot reach (shows a specific message to the passenger, distinct
    // from an ordinary un-painted cell); -1 is the eraser (removes any paint, back to
    // the default generic "not available" message).
    if(g_pax_grid_edit) {
        int stripY = TITLE_H;
        DrawRectangle(0, stripY, sw, PICK_H, {18,23,38,255});
        DrawRectangle(0, stripY+PICK_H-2, sw, 2, {70,70,90,180});
        int nBtn = NUM_DEST_TOTAL+2;   // + Unreachable + Erase
        int gap = 6, bw = (sw - gap*(nBtn+1)) / nBtn;
        int bx = gap;
        for(int i=0;i<NUM_DEST_TOTAL;i++){
            bool sel = (g_pax_paint_dest==i);
            Color base = combinedDestColor(i);
            Color fill = sel ? base : Color{(unsigned char)(base.r/3),(unsigned char)(base.g/3),(unsigned char)(base.b/3),220};
            Rectangle r={(float)bx,(float)(stripY+6),(float)bw,(float)(PICK_H-12)};
            if(Button(r, combinedDestName(i), fill, sel?WHITE:Color{90,90,100,180}, WHITE, 13.f))
                g_pax_paint_dest=i;
            bx += bw+gap;
        }
        {
            bool sel=(g_pax_paint_dest==PAX_UNREACHABLE);
            Rectangle r={(float)bx,(float)(stripY+6),(float)bw,(float)(PICK_H-12)};
            if(Button(r,"Unreachable",sel?Color{160,60,20,255}:Color{60,35,20,220},
                      sel?WHITE:Color{90,90,100,180},WHITE,13.f))
                g_pax_paint_dest=PAX_UNREACHABLE;
            bx += bw+gap;
        }
        {
            bool sel=(g_pax_paint_dest==-1);
            Rectangle r={(float)bx,(float)(stripY+6),(float)bw,(float)(PICK_H-12)};
            if(Button(r,"Erase",sel?C_BAD:Color{60,40,40,220},sel?WHITE:Color{90,90,100,180},WHITE,15.f))
                g_pax_paint_dest=-1;
        }
    }

    // ── BOTTOM BAR ───────────────────────────────────────────────────────────
    DrawRectangle(0,MAP_Y1,sw,BAR_H,{12,16,28,255});
    DrawRectangle(0,MAP_Y1,sw,3,{60,120,220,160});
    float barCY = MAP_Y1 + BAR_H*0.5f;   // vertical centre of the bar

    if(g_pax_grid_edit) {
        // Cancel (left)  |  hint (centre)  |  Save & Exit (right)
        Rectangle cancelBtn={24.f,barCY-24.f,130.f,48.f};
        if(Button(cancelBtn,"Cancel",{38,38,42,220},{120,120,120,170},{210,210,210,230},17.f)){
            g_pax_grid_edit=false; load_pax_grid();
        }

        TC("Pick a place above, then tap cells to paint them. Tap a painted cell again (or use Erase) to remove it.",
           (float)(sw/2), barCY-8.f, 15, {180,190,210,220});

        Rectangle saveBtn={(float)(sw-204),barCY-26.f,180.f,52.f};
        if(Button(saveBtn,"Save & Exit",{120,90,0,235},YELLOW,YELLOW,19.f)){
            g_pax_grid_edit=false; save_pax_grid();
        }
    } else {
        // Back (left)  |  status (centre)  |  Begin Journey (right)
        Rectangle backBtn={24.f,barCY-24.f,130.f,48.f};
        if(Button(backBtn,"Back",{38,38,42,220},{120,120,120,170},{210,210,210,230},17.f)){
            sel_col=-1; sel_row=-1; sel_dest=-1; show_dest=false; g_show_localize=true;
        }

        // Status text — colour coded, centred
        if(hasSel)
            TC(TextFormat("Selected: %s  -  ready to go!", combinedDestName(sel_dest)),
               (float)(sw/2), barCY-11.f, 21, {150,255,170,235});
        else
            TC("Tap a place on the map to choose where you want to go",
               (float)(sw/2), barCY-10.f, 19, {185,195,210,210});

        // Begin Journey (right)
        int bw=250, bh=56;
        Rectangle goBtn={(float)(sw-bw-24),barCY-bh*0.5f,(float)bw,(float)bh};
        Color gf = hasSel?Color{20,160,70,255}:Color{40,44,54,200};
        Color gb = hasSel?WHITE:Color{70,70,80,180};
        Color gt = hasSel?WHITE:Color{95,95,105,170};
        if(Button(goBtn, hasSel?"Begin Journey":"Select a place first", gf,gb,gt,20.f) && hasSel){
            const char* dname=nullptr; SavedDest resolved;
            if(resolve_combined_dest(sel_dest, dname, resolved)){
                // Localize + plan are already done in the passenger flow, so build the
                // route and drop straight into Step 3 (Drive). Seed the planning pose
                // from the live localized EKF pose (if located), else the estimated
                // start, so the route departs from where the cart actually is.
                if(ekf.seeded){
                    std::lock_guard<std::mutex> lk(ekf_mutex);
                    g_pp_robot        = {(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],true};
                    g_confirmed_start = {(float)ekf.x[0],(float)ekf.x[1],(float)ekf.x[2],true};
                } else {
                    // No live localization (e.g. bench test with LiDAR/IMU at 0 Hz):
                    // pick a start pose so the route still builds — estimated start if
                    // set, else the current planning pose, else world origin (the lab
                    // start). Mirrors the operator "Skip Localization" fallback.
                    if(g_est_start.valid)      g_pp_robot = g_est_start;
                    else if(!g_pp_robot.valid) g_pp_robot = {0.f,0.f,0.f,true};
                    g_confirmed_start = {g_pp_robot.x, g_pp_robot.y, g_pp_robot.yaw, true};
                }
                // Danger-zone CROSSING destinations (Basketball Court, Food Court — the
                // last two zone-2 slots) lie beyond the danger-zone stretch: route to the
                // "Building A" checkpoint first, stash the real destination, and let the
                // main-loop orchestration (g_dzx_state) take over the map-switch once
                // that checkpoint is reached. resolve_combined_dest() already armed zone
                // 2 for this destination's own coords — re-arm zone 1 instead, since
                // Building A belongs to zone 1's corridor.
                bool needsCrossing = (sel_dest==NUM_DEST+4 || sel_dest==NUM_DEST+5);
                // ...BUT only if the cart is actually still on the ZONE-1 side of the
                // danger zone. The zone-2 corridor begins right after the danger stretch,
                // so every zone-2 destination is directly reachable once the cart is
                // already in zone 2 — sending it back to the Building A checkpoint then is
                // a pointless detour (this was the reported bug: Food Court routed to
                // Building A while the near-identical Curve 2 destination went direct,
                // because Curve 2 isn't flagged crossing). pp_zone_for_point: 0=zone1,
                // 1=zone2. If the cart is nearest zone 2's corridor, skip the crossing and
                // route direct like any normal zone-2 destination.
                // GUARD (2026-07-08): only skip when we have a genuinely LIVE localized
                // pose (fus_state==FUSION_RUNNING). g_pp_robot otherwise comes from the
                // STICKY ekf.seeded pose, which can hold a stale zone-2 position left over
                // from an earlier danger-zone test / bench run — that made this skip
                // wrongly fire and silently cancel the ENTIRE crossing sequence ("it was
                // working all day, now it's gone"). When not live-localized we keep the
                // crossing (the safe default the whole flow was built around).
                if(needsCrossing && fus_state==STATE_FUSION_RUNNING){
                    int rz = pp_zone_for_point(g_pp_robot.x, g_pp_robot.y, 1e9f);
                    if(rz==1) needsCrossing=false;   // genuinely already past the danger zone
                }
                g_journey_dest_name = dname;   // friendly-panel pin label
                if(needsCrossing && g_destinations[3].valid){
                    g_dzx_final_x=resolved.x; g_dzx_final_y=resolved.y; g_dzx_final_name=dname;
                    g_dzx_state=DZX_TO_CHECKPOINT;
                    g_zone_armed=true; g_zone2_armed=false;
                    pp_run_pipeline(TRAJ_CSV_FILE);
                    g_pp_goal = {g_destinations[3].x, g_destinations[3].y, 0, true};
                    // Matches the friendly panel's leg banner (DrawDriveMapPanel) — without
                    // this the technical view's plan just looks like it stopped short at
                    // Building A instead of a chosen destination.
                    g_statusMsg = TextFormat("Leg 1/3: To checkpoint (Building A) - then crossing to %s", dname);
                    g_statusCol = C_WARN;
                } else {
                    g_dzx_state=DZX_NONE;
                    g_pp_goal = {resolved.x, resolved.y, 0, true};
                }
                pp_trigger_plan();
                // Best-effort build+load; Step 3's own readiness panel + Start
                // Autonomous auto-replan cover any remaining gaps.
                if(!g_approach_path.empty() || !g_corridor_path.empty()){
                    pp_save_plan(); ctrl_doLoad();
                }
                // Drop straight into Step 3 — the standalone "Route Preview" gate screen
                // was replaced by a permanent split view WITHIN Step 3 itself (map cloud
                // over the campus photo, live car marker, beside the normal drive view),
                // so there's no separate review step to stop at first.
                show_dest=false;
                g_drive_via_passenger=true;
                g_ui_step.store(STEP_DRIVE);
            }
            sel_col=-1; sel_row=-1; sel_dest=-1;
        }
    }
}

// =============================================================================
// DRIVE MAP PANEL — the friendly campus-photo view shown beside the technical Step 3
// view during driving (left half of the screen; Step 3 itself renders into the right
// half — see the split-screen block in main.cpp). Same campus photo + calibrated
// transform technique as the Localize panel's calibration mode
// ([localize_bg_<map>.cfg]), read-only, showing just the planned route (g_ctrl_path)
// and a live car marker (EKF pose) — NOT the raw point cloud, which is already shown
// on the technical side (right half) and just clutters the friendly view here.
// Confined to the (x0,y0,w,h) region passed in — NOT full-screen, unlike the old
// standalone gate this replaced.
// =============================================================================
void DrawDriveMapPanel(int x0, int y0, int w, int h, Texture2D bg, Font font, AppState fus_state) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };

    // Crossing-journey leg banner (2026-07-08): Basketball Court / Food Court route
    // through the danger-zone checkpoint in 3 legs (robot -> Building A -> map-switch
    // + cross -> map-switch back -> real destination). Without this, "Begin Journey"
    // for those two destinations shows a plan that stops at Building A and looks like
    // the wrong/broken destination was picked — this makes the multi-leg nature (and
    // the actual final destination) explicit instead of looking like a mistake.
    const char* legLabel = nullptr;
    switch(g_dzx_state){
        case DZX_TO_CHECKPOINT:
            legLabel = TextFormat("Leg 1/3: To checkpoint (Building A) - then crossing to %s",
                                   g_dzx_final_name.c_str());
            break;
        case DZX_WAIT_ENTER_RELOC:
            legLabel = "Relocalizing in danger-zone map...";
            break;
        case DZX_IN_DZMAP:
            legLabel = TextFormat("Leg 2/3: Crossing danger zone - then continuing to %s",
                                   g_dzx_final_name.c_str());
            break;
        case DZX_WAIT_EXIT_RELOC:
            legLabel = "Relocalizing on main map...";
            break;
        default: break;
    }
    const int TITLE_H = legLabel ? 58 : 40;
    const int MAP_Y0=y0+TITLE_H, MAP_Y1=y0+h, MAP_H=MAP_Y1-MAP_Y0;

    float iw=bg.id>0?(float)bg.width:1920.f;
    float ih=bg.id>0?(float)bg.height:1080.f;
    bool haveTf = g_localize_bg_tf.valid;

    // ── Follow+zoom camera (2026-07-08): while the cart is localized, centre on its
    // live position and zoom in to VIEW_RADIUS_M metres instead of showing the whole
    // campus photo at a fixed cover-fit scale — at that zoomed-out scale the route and
    // car marker were too small to read. Smoothed (Lerp) across frames so it pans,
    // not snaps, as the EKF position updates. Falls back to the old static full-photo
    // cover-fit when there's no live pose to follow (not yet localized).
    static float s_camX=0.f, s_camY=0.f;
    static bool  s_camInit=false;
    const float VIEW_RADIUS_M = 60.f;   // metres shown across the panel's short side (was 40 — widened to cover more area)

    float bscale, bgX, bgY;
    // fus_state==STATE_FUSION_RUNNING, not ekf.seeded — seeded is sticky (stays true
    // from ANY past localization, even a stale one from a previous test/map) and
    // doesn't mean the current position is live/trustworthy. Confirmed on a bench
    // test: LiDAR at 0Hz, not actually localized this session, but ekf.seeded was
    // still true from earlier — the panel confidently followed a stale, wrong
    // position instead of showing "not yet localized".
    bool following = haveTf && fus_state==STATE_FUSION_RUNNING;
    if(following){
        double ex,ey;
        {std::lock_guard<std::mutex> lk(ekf_mutex); ex=ekf.x[0]; ey=ekf.x[1];}
        if(!s_camInit){ s_camX=(float)ex; s_camY=(float)ey; s_camInit=true; }
        else { s_camX=Lerp(s_camX,(float)ex,0.12f); s_camY=Lerp(s_camY,(float)ey,0.12f); }
        bscale = (fminf((float)w,(float)MAP_H)*0.5f) / (VIEW_RADIUS_M * g_localize_bg_tf.scale);
        Vector2 camImg = g_localize_bg_tf.worldToImg(s_camX,s_camY);
        bgX = x0+w*0.5f - camImg.x*bscale;
        bgY = MAP_Y0+MAP_H*0.5f - camImg.y*bscale;
    } else {
        s_camInit=false;   // re-snap (no lerp-in) once localized again
        // ── Background cover-fit transform, fitted to THIS region (w x h), not the
        // whole screen — same cover-fit math as the Localize panel.
        bscale=bg.id>0?fmaxf((float)w/iw,(float)h/ih):1.f;
        bgX=x0+(w-iw*bscale)*0.5f;
        bgY=y0+(h-ih*bscale)*0.5f;
    }

    // ── Map area: campus photo + planned route + live car marker ──
    BeginScissorMode(x0,MAP_Y0,w,MAP_H);
    if(bg.id>0) DrawTextureEx(bg,{bgX,bgY},0.f,bscale,WHITE);
    else DrawRectangle(x0,MAP_Y0,w,MAP_H,{18,24,38,255});
    DrawRectangle(x0,MAP_Y0,w,MAP_H,{0,0,0,70}); // light tint so the route/marker pop

    // Is the active map one of the danger-zone TEST maps? Their world frame is
    // unrelated to the campus frame, so campus-frame overlays (the crossing journey's
    // final destination + remaining-legs preview) must be skipped while one is active
    // (legs 2's map-switch) — the banner still names the final destination there.
    bool onDzTestMap = active_map_ptr &&
        (active_map_ptr->name=="Danger Zone Fwd" || active_map_ptr->name=="Danger Zone Rev");

    if(active_map_ptr && active_map_ptr->loaded && haveTf){
        // Planned route (2026-07-08: drawn as a filled 1.5m-each-side corridor band,
        // not the thin center path line — reads as "the road" at a glance instead of
        // a wire. World-space perpendicular offset per point (same formula as the
        // technical view's g_corridor_L/R in planning.cpp), transformed through the
        // same world->photo transform as the car marker below.
        if(g_ctrl_path.size()>=2){
            const float ROUTE_HALF_W_M = 6.0f;   // doubled again (2026-07-08: 1.5 -> 3.0 -> 6.0m each side)
            auto toScreen=[&](float wx,float wy)->Vector2{
                Vector2 ip=g_localize_bg_tf.worldToImg(wx,wy);
                return {bgX+ip.x*bscale, bgY+ip.y*bscale};
            };
            Color bandCol = Fade(C_GOOD,0.40f);
            // 2026-07-10 FIX: offset perpendicular to the actual SEGMENT direction
            // (b-a), NOT each point's stored .yaw. The recorded top-LiDAR paths carry
            // yaw=0 (and elsewhere path yaw uses a different sign convention than robot
            // yaw), so the old .yaw-based normal sprayed the band into a fan wherever
            // consecutive points had inconsistent/zero yaw (confirmed on the reverse
            // map screenshot). Segment geometry is convention-free and always
            // perpendicular to actual travel direction, so this can't fan out.
            for(size_t i=1;i<g_ctrl_path.size();i++){
                const auto& a=g_ctrl_path[i-1]; const auto& b=g_ctrl_path[i];
                float dx=b.x-a.x, dy=b.y-a.y, len=sqrtf(dx*dx+dy*dy);
                if(len<1e-4f) continue;               // skip degenerate/duplicate points
                float nx=-dy/len*ROUTE_HALF_W_M, ny=dx/len*ROUTE_HALF_W_M;
                Vector2 saL=toScreen(a.x+nx,a.y+ny), saR=toScreen(a.x-nx,a.y-ny);
                Vector2 sbL=toScreen(b.x+nx,b.y+ny), sbR=toScreen(b.x-nx,b.y-ny);
                bool aIn = saL.x>=x0&&saL.x<x0+w&&saL.y>=MAP_Y0&&saL.y<MAP_Y1;
                bool bIn = sbL.x>=x0&&sbL.x<x0+w&&sbL.y>=MAP_Y0&&sbL.y<MAP_Y1;
                if(!aIn && !bIn) continue;
                DrawTriangle(saL,saR,sbR,bandCol); DrawTriangle(saL,sbR,saR,bandCol);
                DrawTriangle(saL,sbR,sbL,bandCol); DrawTriangle(saL,sbL,sbR,bandCol);
            }
        }
        // Remaining legs of a crossing journey (2026-07-08) — the loaded plan above
        // only covers the CURRENT leg (e.g. robot → Building A checkpoint), so the
        // passenger saw a route that just stopped there. Preview the rest (checkpoint
        // → across the danger stretch → final destination) as a DASHED line along the
        // full recorded drive, visually distinct from the solid active leg. Display
        // only; the controller never sees this. Cached — the slice endpoints only
        // change when a new journey is picked.
        if(g_dzx_state!=DZX_NONE && !onDzTestMap){
            static std::vector<PPPoint> s_up;
            static float s_k0x=1e18f,s_k0y=0,s_k1x=0,s_k1y=0;
            float fromX = !g_ctrl_path.empty() ? g_ctrl_path.back().x
                        : (g_destinations[3].valid ? g_destinations[3].x : g_dzx_final_x);
            float fromY = !g_ctrl_path.empty() ? g_ctrl_path.back().y
                        : (g_destinations[3].valid ? g_destinations[3].y : g_dzx_final_y);
            if(fabsf(fromX-s_k0x)>0.05f||fabsf(fromY-s_k0y)>0.05f||
               fabsf(g_dzx_final_x-s_k1x)>0.05f||fabsf(g_dzx_final_y-s_k1y)>0.05f){
                s_k0x=fromX; s_k0y=fromY; s_k1x=g_dzx_final_x; s_k1y=g_dzx_final_y;
                pp_full_drive_slice(fromX,fromY,g_dzx_final_x,g_dzx_final_y,s_up);
            }
            for(size_t i=1;i<s_up.size();i++){
                if((i/4)%2) continue;   // dashed: 4 segments on, 4 off
                Vector2 pa=g_localize_bg_tf.worldToImg(s_up[i-1].x,s_up[i-1].y);
                Vector2 pb=g_localize_bg_tf.worldToImg(s_up[i].x,  s_up[i].y);
                float ax2=bgX+pa.x*bscale, ay2=bgY+pa.y*bscale;
                float bx2=bgX+pb.x*bscale, by2=bgY+pb.y*bscale;
                bool aIn = ax2>=x0&&ax2<x0+w&&ay2>=MAP_Y0&&ay2<MAP_Y1;
                bool bIn = bx2>=x0&&bx2<x0+w&&by2>=MAP_Y0&&by2<MAP_Y1;
                if(aIn||bIn) DrawLineEx({ax2,ay2},{bx2,by2},3.f,Fade(C_GOOD,0.45f));
            }
        }
        // Final-destination pin — the place the passenger actually picked, with its
        // name. During a crossing journey this is the stashed real destination (NOT
        // the checkpoint the current plan ends at); otherwise the plan goal itself.
        // Off-screen (zoomed follow view) → an edge arrow pointing toward it, so the
        // passenger always sees where they're headed.
        {
            float pinX=0.f, pinY=0.f; const char* pinName=nullptr; bool havePin=false;
            if(g_dzx_state!=DZX_NONE){
                if(!onDzTestMap){ pinX=g_dzx_final_x; pinY=g_dzx_final_y;
                                  pinName=g_dzx_final_name.c_str(); havePin=true; }
            } else if(g_pp_goal.valid){
                pinX=g_pp_goal.x; pinY=g_pp_goal.y;
                pinName=g_journey_dest_name.empty()?"Destination":g_journey_dest_name.c_str();
                havePin=true;
            }
            if(havePin){
                const Color pinCol={235,75,95,255};   // red-pink: distinct from route + car
                Vector2 ip=g_localize_bg_tf.worldToImg(pinX,pinY);
                float sx=bgX+ip.x*bscale, sy=bgY+ip.y*bscale;
                bool in = sx>=x0+20&&sx<x0+w-20&&sy>=MAP_Y0+20&&sy<MAP_Y1-20;
                if(in){
                    DrawCircleV({sx,sy},15.f,Fade(pinCol,0.30f));
                    DrawCircleV({sx,sy},9.f,pinCol);
                    DrawCircleLines((int)sx,(int)sy,9,WHITE);
                    if(pinName&&pinName[0]){
                        float tw=MW(pinName,16);
                        DrawRectangleRounded({sx-tw*0.5f-8.f,sy+13.f,tw+16.f,24.f},0.4f,6,{10,14,24,215});
                        TC(pinName,sx,sy+17.f,16,WHITE);
                    }
                } else {
                    // Edge indicator: clamp to the panel border, arrow toward the pin.
                    float ex2=clampf(sx,(float)x0+30.f,(float)(x0+w)-30.f);
                    float ey2=clampf(sy,(float)MAP_Y0+30.f,(float)MAP_Y1-30.f);
                    float ddx=sx-ex2, ddy=sy-ey2, dl=sqrtf(ddx*ddx+ddy*ddy);
                    if(dl>1.f){ ddx/=dl; ddy/=dl; } else { ddx=0.f; ddy=-1.f; }
                    Vector2 tip={ex2+ddx*14.f, ey2+ddy*14.f};
                    Vector2 va={ex2-ddy*9.f, ey2+ddx*9.f};
                    Vector2 vb={ex2+ddy*9.f, ey2-ddx*9.f};
                    DrawCircleV({ex2,ey2},17.f,Fade(pinCol,0.25f));
                    DrawTriangle(tip,va,vb,pinCol);
                    DrawTriangle(tip,vb,va,pinCol);   // both windings (raylib culls CW)
                    if(pinName&&pinName[0]){
                        float tw=MW(pinName,15);
                        float lx2=clampf(ex2-tw*0.5f,(float)x0+6.f,(float)(x0+w)-tw-6.f);
                        float ly2=clampf(ey2+22.f,(float)MAP_Y0+6.f,(float)MAP_Y1-26.f);
                        DrawRectangleRounded({lx2-6.f,ly2-3.f,tw+12.f,23.f},0.4f,6,{10,14,24,215});
                        TW(pinName,lx2,ly2,15,WHITE);
                    }
                }
            }
        }
        // Live car marker — same world->photo transform as the route above, so it
        // lines up with the route exactly the way the cart lines up with the real
        // road in the photo once calibration is good. Same fus_state check as
        // `following` above — don't draw a confident marker from a stale pose.
        if(following){
            double ex,ey;
            {std::lock_guard<std::mutex> lk(ekf_mutex); ex=ekf.x[0]; ey=ekf.x[1];}
            Vector2 cp=g_localize_bg_tf.worldToImg((float)ex,(float)ey);
            float csx=bgX+cp.x*bscale, csy=bgY+cp.y*bscale;
            if(csx>=x0&&csx<x0+w&&csy>=MAP_Y0&&csy<MAP_Y1){
                // Plain circle marker — no heading tick — doubled again (2026-07-08:
                // 18/12 -> 36/24) and a colour distinct from the green route band so
                // it doesn't blend in.
                const Color carMarkerCol = GOLD;
                DrawCircleV({csx,csy},36.f,Fade(carMarkerCol,0.35f));
                DrawCircleV({csx,csy},24.f,carMarkerCol);
                DrawCircleLines((int)csx,(int)csy,24,BLACK);
            }
        }
    } else {
        TC(active_map_ptr && active_map_ptr->loaded ?
               "No calibration saved for this map yet - showing photo only" :
               "Map not loaded",
           (float)(x0+w/2), (float)(MAP_Y0+MAP_H/2), 16.f, {220,190,80,220});
    }
    EndScissorMode();

    // ── Title strip ──
    DrawRectangle(x0,y0,w,TITLE_H,{14,19,33,255});
    DrawRectangle(x0,y0+TITLE_H-3,w,3,C_HEADER);
    if(legLabel){
        TC(TextFormat("Live Map - %s", active_map_ptr?active_map_ptr->name.c_str():"map"),
           (float)(x0+w/2),(float)(y0+6),16,C_HEADER);
        TC(legLabel,(float)(x0+w/2),(float)(y0+30),15,{255,205,90,255});
    } else {
        TC(TextFormat("Live Map - %s", active_map_ptr?active_map_ptr->name.c_str():"map"),
           (float)(x0+w/2),(float)(y0+(TITLE_H-18)*0.5f),18,C_HEADER);
    }
}

// =============================================================================
// JOURNEY END PANEL — shown once the cart reaches its final destination via the
// passenger flow (2026-07-08). Asks whether to start another journey; "Yes" resets
// straight back to the Welcome splash (a fresh passenger flow from the very start),
// "No" just dismisses the card and leaves the cart parked at Step 3. Same centred-
// card style as the Map Select panel.
// =============================================================================
void DrawJourneyEndPanel(int sw, int sh, Texture2D bg, Font font, bool& show_journey_end,
                          bool& show_welcome, bool& show_intro, bool& show_map_select,
                          bool& show_localize, bool& show_dest) {
    const float SP = 0.6f;
    auto TW=[&](const char* t,float x,float y,float sz,Color c){ DrawTextEx(font,t,{x,y},sz,SP,c); };
    auto MW=[&](const char* t,float sz)->float{ return MeasureTextEx(font,t,sz,SP).x; };
    auto TC=[&](const char* t,float cx,float y,float sz,Color c){ TW(t,cx-MW(t,sz)*0.5f,y,sz,c); };

    Vector2 mouse=GetMousePosition();
    auto Button=[&](Rectangle r,const char* label,Color fill,Color border,Color tc,float fsz)->bool{
        bool hov=CheckCollisionPointRec(mouse,r);
        DrawRectangleRounded(r,0.28f,8,hov?ColorBrightness(fill,0.18f):fill);
        DrawRectangleRoundedLines(r,0.28f,8,1.5f,border);
        TW(label,r.x+(r.width-MW(label,fsz))*0.5f,r.y+(r.height-fsz)*0.5f,fsz,tc);
        return hov && IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
    };

    // Background
    float iw=bg.id>0?(float)bg.width:1920.f, ih=bg.id>0?(float)bg.height:1080.f;
    float bscale=fmaxf((float)sw/iw,(float)sh/ih);
    float bgX=(sw-iw*bscale)*0.5f, bgY=(sh-ih*bscale)*0.5f;
    if(bg.id>0) DrawTextureEx(bg,{bgX,bgY},0.f,bscale,WHITE);
    else DrawRectangle(0,0,sw,sh,{18,24,38,255});
    DrawRectangle(0,0,sw,sh,{0,0,0,170}); // dark overlay

    // Centre card — sized up ~44% 2026-07-27 (user request, matches the readability
    // pass applied to the unreachable/not-available destination card).
    const float CW=806.f, CH=432.f;
    float cx=(float)sw*0.5f, cy=(float)sh*0.5f;
    float cardX=cx-CW*0.5f, cardY=cy-CH*0.5f;
    DrawRectangleRounded({cardX+9,cardY+9,CW,CH},0.10f,8,{0,0,0,110});
    DrawRectangleRounded({cardX,cardY,CW,CH},0.10f,8,{13,18,31,248});
    DrawRectangleRoundedLines({cardX+1,cardY+1,CW-2,CH-2},0.10f,8,1.f,{255,255,255,15});
    DrawRectangleRounded({cardX,cardY,CW,12},0.f,4,{0,140,58,220});

    TC("You've Arrived!", cx, cardY+49.f, 43.f, WHITE);
    TC("Thank you for riding with us today - we hope", cx, cardY+118.f, 24.f, {180,200,190,215});
    TC("you enjoyed the journey across campus.", cx, cardY+150.f, 24.f, {180,200,190,215});
    TC("Would you like to take another journey?", cx, cardY+210.f, 27.f, {225,235,230,235});

    const float BW=317.f, BH=84.f, BY=cardY+274.f;
    if(Button({cx-BW-17.f,BY,BW,BH},"Yes, Let's Go Again",{20,150,70,255},{90,220,150,220},WHITE,24.f)){
        show_journey_end=false;
        // Fresh passenger flow from the very start — same as first launch.
        show_intro=false; show_map_select=false; show_localize=false; show_dest=false;
        show_welcome=true;
    }
    if(Button({cx+17.f,BY,BW,BH},"No, I'm Done",{45,48,58,255},{110,115,130,200},{215,220,228,235},24.f)){
        show_journey_end=false;
    }
}
