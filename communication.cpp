// =============================================================================
// communication.cpp  —  UDP send/receive, ESP32 commands, telemetry parsing
// =============================================================================
#include "common.hpp"

// =============================================================================
//  GLOBALS (owned here)
// =============================================================================
SOCKET g_txSock = INVALID_SOCKET;
SOCKET g_rxSock = INVALID_SOCKET;
struct sockaddr_in g_esp32Addr = {};

std::vector<LogRow>  g_log_rows;
double               g_log_run_start     = 0.0;
int                  g_log_send_seq      = 0;
int                  g_log_last_ack_seq  = -1;
double               g_log_last_send_time = 0.0;
double               g_log_last_ack_time  = 0.0;
float                g_log_telem_steer   = 0.f;
float                g_log_telem_speed   = 0.f;
int                  g_log_telem_seq     = -1;
std::string          g_log_filename      = "";

float    g_wheel_speed_mps    = 1.0f;
bool     g_wheel_speed_valid  = false;
uint64_t g_wheel_speed_lastMs = 0;

// =============================================================================
//  NETWORK INIT
// =============================================================================
bool initNetwork() {
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2,2), &wsa) != 0) return false;
    // Single socket bound to LOCAL_TELEM_PORT for both sending commands and receiving telemetry.
    // Binding to a fixed local port means PC sends commands FROM port 4211, so Windows Firewall
    // stateful inspection allows the ESP32's reply back to that port — no manual firewall rule needed.
    g_txSock = socket(AF_INET, SOCK_DGRAM, 0);
    if(g_txSock == INVALID_SOCKET) return false;
    // Allow reuse so a previous crashed session holding port 4211 doesn't block us
    int reuse = 1;
    setsockopt(g_txSock, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));
    u_long nb = 1; ioctlsocket(g_txSock, FIONBIO, &nb);
    struct sockaddr_in local = {};
    local.sin_family      = AF_INET;
    local.sin_port        = htons(LOCAL_TELEM_PORT);  // bind to 4211
    local.sin_addr.s_addr = INADDR_ANY;
    bind(g_txSock, (struct sockaddr*)&local, sizeof(local));
    g_rxSock = g_txSock;  // same socket receives telemetry replies on port 4211
    g_esp32Addr.sin_family = AF_INET;
    g_esp32Addr.sin_port   = htons(ESP32_CMD_PORT);
    inet_pton(AF_INET, ESP32_IP, &g_esp32Addr.sin_addr);
    return true;
}

// =============================================================================
//  COMMAND SEND
// =============================================================================
void ctrl_sendCmd(const std::string& cmd) {
    if(g_txSock != INVALID_SOCKET) {
        sendto(g_txSock, cmd.c_str(), (int)cmd.size(), 0,
               (struct sockaddr*)&g_esp32Addr, sizeof(g_esp32Addr));
        if(!g_simMode && g_log_run_start > 0.0) {
            g_log_send_seq++;
            g_log_last_send_time = nowSecEpoch();
        }
    }
    cmdlog_push((g_simMode ? "[SIM] " : "[UDP] ") + cmd);
}

// =============================================================================
//  TELEMETRY POLL  (non-blocking, called from control thread)
// =============================================================================
void pollTelemetry() {
    if(g_rxSock == INVALID_SOCKET) return;
    char buf[512] = {};
    struct sockaddr_in from = {};
    int fl = sizeof(from);
    int n = recvfrom(g_rxSock, buf, sizeof(buf)-1, 0, (struct sockaddr*)&from, &fl);
    if(n > 0) {
        buf[n] = '\0';
        g_telemStr = std::string(buf);

        if(strstr(buf, "STEER_DONE") != nullptr) {
            const char* sd = strstr(buf, "STEER_DONE,");
            if(sd) {
                float actual_deg = (float)atof(sd + strlen("STEER_DONE,"));
                if(actual_deg >= -25.f && actual_deg <= 25.f) {
                    g_ctrl.thetaActual = -actual_deg;  // negate: ESP32 negates the cmd angle on input
                    g_ctrl.thetaValid  = true;
                }
            }
        }

        if(g_log_run_start > 0.0) {
            g_log_last_ack_time = nowSecEpoch();
            const char* p = buf;
            auto parseField  = [&](const char* key, float& out) {
                const char* f = strstr(p, key);
                if(f){ f += strlen(key); out = (float)atof(f); }
            };
            auto parseFieldI = [&](const char* key, int& out) {
                const char* f = strstr(p, key);
                if(f){ f += strlen(key); out = atoi(f); }
            };
            parseField ("STEER_DEG,", g_log_telem_steer);
            parseField ("SPEED_V,",   g_log_telem_speed);
            parseFieldI("SEQ:",       g_log_telem_seq);

            float wheel_mps_raw = -1.f;
            parseField("WHEEL_MPS,", wheel_mps_raw);
            if(wheel_mps_raw >= 0.f && wheel_mps_raw < 10.f) {
                g_wheel_speed_mps   = wheel_mps_raw;
                g_wheel_speed_valid = true;
                g_wheel_speed_lastMs = (uint64_t)(nowSecEpoch() * 1000.0);
            }
        }
    }
}

// =============================================================================
//  DATA LOGGER
// =============================================================================
void logger_start_run() {
    g_log_rows.clear();
    g_log_rows.reserve(6000);  // 50 Hz × 120 s — eliminates vector realloc stalls mid-drive
    g_log_run_start    = nowSecEpoch();
    g_log_send_seq     = 0;
    g_log_last_ack_seq = -1;
    std::time_t t  = (std::time_t)g_log_run_start;
    std::tm*    tm = std::localtime(&t);
    char buf[64];  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", tm);
    g_log_filename = std::string("ctrl_log_") + buf + ".csv";
}

void logger_write_csv() {
    if(g_log_filename.empty() || g_log_rows.empty()) return;
    std::ofstream f(g_log_filename);
    if(!f.is_open()) return;
    f << "time_s,pc_send_time_s,esp_ack_time_s,udp_rtt_ms,udp_dropped,"
         "send_seq,ack_seq,steer_cmd_deg,steer_actual_deg,"
         "steer_raw_deg,steer_filtered_deg,steer_delta_deg,steer_sent,tick_index,"
         "speed_cmd_v,speed_actual_v,wheel_mps,cte_m,lookahead_m,"
         "dist_to_goal_m,robot_x,robot_y,robot_yaw_deg,path_index,"
         "lidar_timestamp_s,ctrl_timestamp_s,pipeline_delay_ms,"
         "imu_raw_deg,ekf_yaw_deg,"
         "path_x,path_y,path_yaw_deg,lookahead_x,lookahead_y,"
         "scan_rmse,scan_inlier_ratio,"
         "ped_enabled,ped_stop,ped_caution,grid_ped_stop,dzx_state,map_name\n";
    for(const auto& r : g_log_rows) {
        f << std::fixed << std::setprecision(4)
          << r.time_s            << ","
          << r.pc_send_time_s    << ","
          << r.esp_ack_time_s    << ","
          << r.udp_rtt_ms        << ","
          << (r.udp_dropped?1:0) << ","
          << r.send_seq          << ","
          << r.ack_seq           << ","
          << r.steer_cmd_deg      << ","
          << r.steer_actual_deg   << ","
          << r.steer_raw_deg      << ","
          << r.steer_filtered_deg << ","
          << r.steer_delta_deg    << ","
          << r.steer_sent         << ","
          << r.tick_index         << ","
          << r.speed_cmd_v        << ","
          << r.speed_actual_v    << ","
          << r.wheel_mps         << ","
          << r.cte_m             << ","
          << r.lookahead_m       << ","
          << r.dist_to_goal_m    << ","
          << r.robot_x           << ","
          << r.robot_y           << ","
          << r.robot_yaw_deg     << ","
          << r.path_index        << ","
          << r.lidar_timestamp_s << ","
          << r.ctrl_timestamp_s  << ","
          << r.pipeline_delay_ms << ","
          << r.imu_raw_deg       << ","
          << r.ekf_yaw_deg       << ","
          << r.path_x            << ","
          << r.path_y            << ","
          << r.path_yaw_deg      << ","
          << r.lookahead_x       << ","
          << r.lookahead_y       << ","
          << r.scan_rmse         << ","
          << r.scan_inlier_ratio << ","
          << r.ped_enabled       << ","
          << r.ped_stop          << ","
          << r.ped_caution       << ","
          << r.grid_ped_stop     << ","
          << r.dzx_state         << ","
          << r.map_name          << "\n";
    }
    int total = (int)g_log_rows.size(), dropped = 0, rtt_cnt = 0;
    float rtt_sum = 0.f, delay_sum = 0.f; int delay_cnt = 0;
    for(auto& r : g_log_rows) {
        if(r.udp_dropped) dropped++;
        if(r.udp_rtt_ms > 0.f){ rtt_sum += r.udp_rtt_ms; rtt_cnt++; }
        if(r.pipeline_delay_ms > 0.f){ delay_sum += r.pipeline_delay_ms; delay_cnt++; }
    }
    f << "\n# SUMMARY\n"
      << "# Total ticks: "   << total   << "\n"
      << "# UDP drops: "     << dropped << " ("
         << std::fixed << std::setprecision(1)
         << (total > 0 ? 100.f*dropped/total : 0.f) << "%)\n"
      << "# Avg RTT ms: "    << (rtt_cnt>0 ? rtt_sum/rtt_cnt : 0.f) << "\n"
      << "# Avg pipeline delay ms: " << (delay_cnt>0 ? delay_sum/delay_cnt : 0.f)
         << " (tune CTRL_PREDICTION_DELAY_S to match)\n"
      << "# Run duration s: " << (g_log_rows.empty() ? 0.0 : g_log_rows.back().time_s) << "\n";

    // Slim steer trace — easy to plot, 7 columns only
    std::string trace_name = g_log_filename;
    {
        size_t p = trace_name.find("ctrl_log_");
        if(p != std::string::npos) trace_name.replace(p, 9, "steer_trace_");
    }
    std::ofstream ft(trace_name);
    if(ft.is_open()){
        ft << "time_s,tick,steer_raw_deg,steer_filtered_deg,steer_sent_deg,gate_fired,cte_m,wheel_mps,lookahead_m\n";
        for(const auto& r : g_log_rows)
            ft << std::fixed << std::setprecision(4)
               << r.time_s             << ","
               << r.tick_index         << ","
               << r.steer_raw_deg      << ","
               << r.steer_filtered_deg << ","
               << r.steer_cmd_deg      << ","
               << r.steer_sent         << ","
               << r.cte_m              << ","
               << r.wheel_mps          << ","
               << r.lookahead_m        << "\n";
    }
}

void logger_tick(const CtrlPose& robot) {
    if(g_log_run_start == 0.0) return;
    // Snapshot lidar sweep time once — LiDAR thread updates it at 20 Hz and a
    // race between the two reads below could produce a negative pipeline_delay_ms.
    const double lidar_snap = g_lidar_sweep_time.load();
    LogRow r{};
    r.time_s           = nowSecEpoch() - g_log_run_start;
    r.pc_send_time_s   = g_log_last_send_time - g_log_run_start;
    r.esp_ack_time_s   = (g_log_last_ack_time > 0.0)
                         ? (g_log_last_ack_time - g_log_run_start) : 0.0;
    double rtt         = (g_log_last_ack_time > g_log_last_send_time)
                         ? (g_log_last_ack_time - g_log_last_send_time)*1000.0 : 0.0;
    r.udp_rtt_ms       = (float)rtt;
    r.udp_dropped      = (g_log_telem_seq >= 0 && g_log_send_seq - g_log_telem_seq > 1);
    r.send_seq         = g_log_send_seq;
    r.ack_seq          = g_log_telem_seq;
    r.steer_cmd_deg      = g_ctrl.cmdSteer;
    r.steer_actual_deg   = g_log_telem_steer;
    r.steer_raw_deg      = g_ctrl.cmdSteerRaw;
    r.steer_filtered_deg = g_ctrl.steerFiltered;
    r.steer_delta_deg    = g_ctrl.steerFiltered - g_ctrl.lastSentSteer;
    r.steer_sent         = g_ctrl.steerSentThisTick ? 1 : 0;
    r.tick_index         = (int)g_log_rows.size();
    r.speed_cmd_v        = g_ctrl.cmdSpeed;
    r.speed_actual_v     = g_log_telem_speed;
    r.wheel_mps          = g_wheel_speed_valid ? g_wheel_speed_mps : 0.f;
    r.cte_m            = g_ctrl.crossTrackErr;
    r.lookahead_m      = g_lookaheadM;
    r.dist_to_goal_m   = g_ctrl.distToGoal;
    r.robot_x          = robot.x;
    r.robot_y          = robot.y;
    r.robot_yaw_deg    = toDeg(robot.yaw);
    r.path_index       = g_ctrl.lastCI;
    r.lidar_timestamp_s = lidar_snap - g_log_run_start;
    r.ctrl_timestamp_s  = r.time_s;
    r.pipeline_delay_ms = (float)((r.ctrl_timestamp_s - r.lidar_timestamp_s) * 1000.0);
    r.imu_raw_deg       = g_ctrl.imuRawDeg;
    r.ekf_yaw_deg       = g_ctrl.ekfYawDeg;
    if(!g_ctrl_path.empty()) {
        int ci = std::min(g_ctrl.lastCI,       (int)g_ctrl_path.size()-1);
        int li = std::min(g_ctrl.lookaheadIdx, (int)g_ctrl_path.size()-1);
        r.path_x        = g_ctrl_path[ci].x;
        r.path_y        = g_ctrl_path[ci].y;
        r.path_yaw_deg  = toDeg(g_ctrl_path[ci].yaw);
        r.lookahead_x   = g_ctrl_path[li].x;
        r.lookahead_y   = g_ctrl_path[li].y;
    }
    r.scan_rmse         = current_rmse.load();
    r.scan_inlier_ratio = current_inlier_ratio.load();
    r.ped_enabled       = g_ped_enabled.load() ? 1 : 0;
    r.ped_stop          = g_ped_detected.load() ? 1 : 0;
    r.ped_caution       = g_ped_caution.load() ? 1 : 0;
    r.grid_ped_stop     = g_grid_ped_stop.load() ? 1 : 0;
    r.dzx_state         = (int)g_dzx_state;
    r.map_name          = active_map_ptr ? active_map_ptr->name : "";
    g_log_rows.push_back(r);
}
