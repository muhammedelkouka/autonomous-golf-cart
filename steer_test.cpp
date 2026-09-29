// =============================================================================
// steer_test.cpp  —  Isolated steering subsystem test harness
// =============================================================================
#include "common.hpp"
#include <limits>

// reused from communication.cpp — no socket code duplicated here
void ctrl_sendCmd(const std::string& cmd);
void pollTelemetry();

std::atomic<bool> g_steer_test_running{false};

// =============================================================================
//  TEST A  —  discrete accuracy (stationary)
//  Runs in its own detached thread; never blocks the 60 FPS render loop.
// =============================================================================
void steer_test_run_A() {
    if (g_simMode) {
        cmdlog_push("[SteerTestA] REFUSED: disable sim mode first");
        return;
    }

    // RAII: set flag on entry, clear on any exit path
    struct Guard {
        std::atomic<bool>& f;
        Guard(std::atomic<bool>& f_) : f(f_) { f = true; }
        ~Guard() { f = false; }
    } guard(g_steer_test_running);

    struct StepResult {
        int   step_index;
        float cmd_deg;
        float actual_deg;   // NaN if dropped
        float error_deg;    // NaN if dropped
        float latency_ms;
        bool  dropped;
    };

    const float targets[] = {0.f, 5.f, 0.f, -5.f, 0.f, 10.f, -10.f, 0.f};
    const int   N         = (int)(sizeof(targets) / sizeof(targets[0]));

    ctrl_sendCmd("AUTO");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<StepResult> results;
    results.reserve(N);

    float last_cmd    = std::numeric_limits<float>::quiet_NaN();
    float last_actual = std::numeric_limits<float>::quiet_NaN();

    for (int i = 0; i < N; i++) {
        float target = targets[i];

        // FIX 3: if this step is within 0.25° of the last commanded angle,
        // skip the wait — firmware won't move and STEER_DONE would timeout.
        if (!std::isnan(last_cmd) && !std::isnan(last_actual) &&
            std::fabs(target - last_cmd) < 0.25f) {
            StepResult r;
            r.step_index = i;
            r.cmd_deg    = target;
            r.dropped    = false;
            r.actual_deg = last_actual;
            r.error_deg  = 0.f;
            r.latency_ms = 0.f;
            results.push_back(r);
            last_cmd = target;
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            continue;
        }

        // (a) reset feedback flag
        g_ctrl.thetaValid = false;

        // (b) record send time
        double sendTime = nowSecEpoch();

        // (c) send STEER command
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << "STEER," << target;
            ctrl_sendCmd(ss.str());
        }

        // (d) poll until thetaValid or dynamic timeout (5 deg/s + 1500 ms margin)
        float moveDeg   = std::isnan(last_cmd) ? 23.0f : fabsf(target - last_cmd);
        int   timeoutMs = (int)(moveDeg / 5.0f * 1000.0f) + 1500;
        bool  got       = false;
        float actualDeg = std::numeric_limits<float>::quiet_NaN();
        float latencyMs = 0.f;
        double deadline = sendTime + timeoutMs / 1000.0;

        while (nowSecEpoch() < deadline) {
            pollTelemetry();
            if (g_ctrl.thetaValid) {
                // (e) record actual + latency
                actualDeg = g_ctrl.thetaActual;
                latencyMs = (float)((nowSecEpoch() - sendTime) * 1000.0);
                got       = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // (f/e) store result
        StepResult r;
        r.step_index = i;
        r.cmd_deg    = target;
        r.dropped    = !got;
        r.actual_deg = got ? actualDeg : std::numeric_limits<float>::quiet_NaN();
        r.error_deg  = got ? (actualDeg - target) : std::numeric_limits<float>::quiet_NaN();
        r.latency_ms = got ? latencyMs : (float)timeoutMs;
        results.push_back(r);

        // update for next step's deadband check
        last_cmd    = target;
        if (got) last_actual = actualDeg;

        // (g) settle before next target
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }

    ctrl_sendCmd("MANUAL");

    // --- Write timestamped CSV ---
    std::time_t now_t = std::time(nullptr);
    std::tm*    tm_p  = std::localtime(&now_t);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", tm_p);
    std::string csv_name = std::string("steer_test_A_") + ts_buf + ".csv";

    {
        std::ofstream csv(csv_name);
        csv << "step_index,cmd_deg,actual_deg,error_deg,latency_ms,dropped\n";
        for (const auto& r : results) {
            csv << r.step_index << ","
                << std::fixed << std::setprecision(2) << r.cmd_deg << ",";
            if (!r.dropped)
                csv << std::fixed << std::setprecision(2) << r.actual_deg;
            csv << ",";
            if (!r.dropped)
                csv << std::fixed << std::setprecision(2) << r.error_deg;
            csv << ","
                << std::fixed << std::setprecision(1) << r.latency_ms << ","
                << (r.dropped ? 1 : 0) << "\n";
        }
    }

    // --- One-line summary to cmdLog ---
    int   drop_count  = 0;
    float sum_abs     = 0.f;
    float max_abs     = 0.f;
    int   valid_count = 0;
    for (const auto& r : results) {
        if (r.dropped) { drop_count++; continue; }
        float ae = std::fabs(r.error_deg);
        sum_abs += ae;
        if (ae > max_abs) max_abs = ae;
        valid_count++;
    }
    float mean_abs = valid_count > 0 ? sum_abs / (float)valid_count : 0.f;

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(2)
            << "[SteerTestA] mean|err|=" << mean_abs
            << "deg max|err|=" << max_abs
            << "deg drops=" << drop_count
            << " " << csv_name;
    cmdlog_push(summary.str());
}

// =============================================================================
//  TEST B  —  continuous tracking test (corner simulation)
//
//  Sends a staircase of small commands at 100 ms intervals, matching the
//  real driving 50 Hz loop (every other tick).  Measures how far the physical
//  wheel lags behind each command — the key metric for corner tracking quality.
//
//  Does NOT wait for STEER_DONE.  Instead it polls GET_STATE immediately after
//  each STEER command to read the actual wheel position under continuous motion.
//
//  CSV columns: cmd_index, cmd_deg, actual_deg, lag_deg, timestamp_ms
//  Last row (cmd_index == N) is the post-stream settle measurement.
// =============================================================================
void steer_test_run_B() {
    if (g_simMode) {
        cmdlog_push("[SteerTestB] REFUSED: disable sim mode first");
        return;
    }

    struct Guard {
        std::atomic<bool>& f;
        Guard(std::atomic<bool>& f_) : f(f_) { f = true; }
        ~Guard() { f = false; }
    } guard(g_steer_test_running);

    // ── Setup ─────────────────────────────────────────────────────────────
    ctrl_sendCmd("AUTO");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    g_ctrl.thetaValid = false;
    ctrl_sendCmd("CENTER");
    double center_dl = nowSecEpoch() + 3.0;
    while (nowSecEpoch() < center_dl) {
        pollTelemetry();
        if (g_ctrl.thetaValid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // ── Build staircase sequence (~20 commands) ────────────────────────────
    // Ramp  0 → 10.5° in 1.5° steps (8 cmds)
    // Hold at 10.5° for 5 steps         (5 cmds)
    // Ramp 9.0 → 0°  in 1.5° steps     (7 cmds)
    std::vector<float> cmds;
    for (float a = 0.f;  a <= 10.5f + 0.01f; a += 1.5f) cmds.push_back(a);
    for (int   h = 0;    h < 5;               h++)        cmds.push_back(10.5f);
    for (float a = 9.0f; a >= -0.01f;         a -= 1.5f) cmds.push_back(a);

    struct Row {
        int   cmd_index;
        float cmd_deg;
        float actual_deg;
        float lag_deg;
        float timestamp_ms;
    };

    std::vector<Row> rows;
    rows.reserve(cmds.size() + 1);

    // Parse STEER_DEG from a MATLAB_STATE telemetry string
    auto parseSteerDeg = [](const std::string& s) -> float {
        const char* key = "STEER_DEG,";
        size_t p = s.find(key);
        if (p == std::string::npos) return std::numeric_limits<float>::quiet_NaN();
        return (float)atof(s.c_str() + p + strlen(key));
    };

    const float kNaN   = std::numeric_limits<float>::quiet_NaN();
    double      t_start = nowSecEpoch();

    // ── Stream ────────────────────────────────────────────────────────────
    for (int i = 0; i < (int)cmds.size(); i++) {
        float cmd   = cmds[i];
        float ts_ms = (float)((nowSecEpoch() - t_start) * 1000.0);

        // Send STEER (do not wait for STEER_DONE)
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << "STEER," << cmd;
            ctrl_sendCmd(ss.str());
        }

        // Immediately query actual position via GET_STATE
        g_telemStr.clear();
        ctrl_sendCmd("GET_STATE");

        float  actual   = kNaN;
        double poll_dl  = nowSecEpoch() + 0.200;
        while (nowSecEpoch() < poll_dl) {
            pollTelemetry();
            if (g_telemStr.find("MATLAB_STATE") != std::string::npos) {
                actual = parseSteerDeg(g_telemStr);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        Row r;
        r.cmd_index    = i;
        r.cmd_deg      = cmd;
        r.actual_deg   = actual;
        r.lag_deg      = std::isnan(actual) ? kNaN : (actual - cmd);
        r.timestamp_ms = ts_ms;
        rows.push_back(r);

        // 100 ms between commands
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ── Settle measurement (3 s after last command) ────────────────────────
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    float  settle_actual = kNaN;
    g_telemStr.clear();
    ctrl_sendCmd("GET_STATE");
    double final_dl = nowSecEpoch() + 0.200;
    while (nowSecEpoch() < final_dl) {
        pollTelemetry();
        if (g_telemStr.find("MATLAB_STATE") != std::string::npos) {
            settle_actual = parseSteerDeg(g_telemStr);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Add settle row (cmd_index == N, large timestamp gap — Python uses this to split)
    {
        float last_cmd = cmds.empty() ? 0.f : cmds.back();
        Row r;
        r.cmd_index    = (int)cmds.size();
        r.cmd_deg      = last_cmd;
        r.actual_deg   = settle_actual;
        r.lag_deg      = std::isnan(settle_actual) ? kNaN : (settle_actual - last_cmd);
        r.timestamp_ms = (float)((nowSecEpoch() - t_start) * 1000.0);
        rows.push_back(r);
    }

    ctrl_sendCmd("MANUAL");

    // ── Write CSV ──────────────────────────────────────────────────────────
    std::time_t now_t = std::time(nullptr);
    std::tm*    tm_p  = std::localtime(&now_t);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", tm_p);
    std::string csv_name = std::string("steer_test_B_") + ts_buf + ".csv";

    {
        std::ofstream csv(csv_name);
        csv << "cmd_index,cmd_deg,actual_deg,lag_deg,timestamp_ms\n";
        for (const auto& r : rows) {
            csv << r.cmd_index << ","
                << std::fixed << std::setprecision(2) << r.cmd_deg << ",";
            if (!std::isnan(r.actual_deg))
                csv << std::fixed << std::setprecision(2) << r.actual_deg;
            csv << ",";
            if (!std::isnan(r.lag_deg))
                csv << std::fixed << std::setprecision(2) << r.lag_deg;
            csv << "," << std::fixed << std::setprecision(1) << r.timestamp_ms << "\n";
        }
    }

    // ── Summary (stream rows only, not settle) ─────────────────────────────
    float max_lag = 0.f, sum_lag = 0.f;
    int   lag_cnt = 0;
    for (int i = 0; i < (int)rows.size() - 1; i++) {
        if (std::isnan(rows[i].lag_deg)) continue;
        float al = std::fabs(rows[i].lag_deg);
        if (al > max_lag) max_lag = al;
        sum_lag += al;
        lag_cnt++;
    }
    float mean_lag = lag_cnt > 0 ? sum_lag / (float)lag_cnt : 0.f;

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(2)
            << "[SteerTestB] max_lag=" << max_lag
            << "deg mean_lag=" << mean_lag
            << "deg settled=" << (std::isnan(settle_actual) ? 0.f : settle_actual)
            << "deg " << csv_name;
    cmdlog_push(summary.str());
}

// =============================================================================
//  SWEEP TEST  —  vary command interval across {80,100,120,150,200,250,300} ms
//
//  For each interval: CENTER, then staircase 0→10.5°→0 in 1.5° steps.
//  After every STEER command polls GET_STATE immediately (streaming, no wait
//  for STEER_DONE) to capture lag under continuous motion — same method as B.
//
//  actual_deg is stored in raw ESP32 convention (negated from PC); Python
//  sign-corrects it to compute real_lag = (−actual_deg) − cmd_deg.
//
//  CSV: steer_sweep_YYYYMMDD_HHMMSS.csv
//  Columns: interval_ms, cmd_index, cmd_deg, actual_deg, lag_deg, timestamp_ms
// =============================================================================
void steer_test_run_sweep() {
    if (g_simMode) {
        cmdlog_push("[SteerSweep] REFUSED: disable sim mode first");
        return;
    }

    struct Guard {
        std::atomic<bool>& f;
        Guard(std::atomic<bool>& f_) : f(f_) { f = true; }
        ~Guard() { f = false; }
    } guard(g_steer_test_running);

    const int kIntervals[] = {80, 100, 120, 150, 200, 250, 300};
    const int N_INT = (int)(sizeof(kIntervals) / sizeof(kIntervals[0]));

    // Staircase: 0→10.5° in 1.5° steps, then 9→0°
    std::vector<float> staircase;
    for (float a = 0.f; a <= 10.5f + 0.01f; a += 1.5f) staircase.push_back(a);
    for (float a = 9.0f; a >= -0.01f; a -= 1.5f)        staircase.push_back(a);

    struct Row {
        int   interval_ms;
        int   cmd_index;
        float cmd_deg;
        float actual_deg;   // raw ESP32 (negated from PC convention)
        float lag_deg;      // actual_deg - cmd_deg (raw; Python negates actual for real_lag)
        float timestamp_ms;
    };

    const float kNaN = std::numeric_limits<float>::quiet_NaN();

    auto parseSteerDeg = [](const std::string& s) -> float {
        const char* key = "STEER_DEG,";
        size_t p = s.find(key);
        if (p == std::string::npos) return std::numeric_limits<float>::quiet_NaN();
        return (float)atof(s.c_str() + p + strlen(key));
    };

    // Timestamp for CSV name — fixed at start so all intervals share one file
    std::time_t now_t = std::time(nullptr);
    std::tm*    tm_p  = std::localtime(&now_t);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", tm_p);
    std::string csv_name = std::string("steer_sweep_") + ts_buf + ".csv";

    struct IntervalSummary { int interval_ms; float max_lag; float mean_lag; };
    std::vector<IntervalSummary> summaries;
    summaries.reserve(N_INT);

    std::vector<Row> all_rows;
    all_rows.reserve(N_INT * (int)staircase.size());

    ctrl_sendCmd("AUTO");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for (int ii = 0; ii < N_INT; ii++) {
        int interval_ms = kIntervals[ii];

        // Center wheel before each interval, wait for STEER_DONE (thetaValid)
        g_ctrl.thetaValid = false;
        ctrl_sendCmd("CENTER");
        double center_dl = nowSecEpoch() + 6.0;
        while (nowSecEpoch() < center_dl) {
            pollTelemetry();
            if (g_ctrl.thetaValid) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // settle

        double t_start = nowSecEpoch();

        for (int ci = 0; ci < (int)staircase.size(); ci++) {
            float cmd   = staircase[ci];
            float ts_ms = (float)((nowSecEpoch() - t_start) * 1000.0);

            {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2) << "STEER," << cmd;
                ctrl_sendCmd(ss.str());
            }

            // Immediately poll actual position via GET_STATE (no STEER_DONE wait)
            g_telemStr.clear();
            ctrl_sendCmd("GET_STATE");

            float  actual  = kNaN;
            double poll_dl = nowSecEpoch() + 0.200;
            while (nowSecEpoch() < poll_dl) {
                pollTelemetry();
                if (g_telemStr.find("MATLAB_STATE") != std::string::npos) {
                    actual = parseSteerDeg(g_telemStr);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            Row r;
            r.interval_ms  = interval_ms;
            r.cmd_index    = ci;
            r.cmd_deg      = cmd;
            r.actual_deg   = actual;
            r.lag_deg      = std::isnan(actual) ? kNaN : (actual - cmd);
            r.timestamp_ms = ts_ms;
            all_rows.push_back(r);

            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }

        // Per-interval summary using sign-corrected lag: real_lag = −actual − cmd
        float max_lag = 0.f, sum_lag = 0.f;
        int   valid_cnt = 0;
        for (const auto& r : all_rows) {
            if (r.interval_ms != interval_ms || std::isnan(r.actual_deg)) continue;
            float real_lag = std::fabs(-r.actual_deg - r.cmd_deg);
            if (real_lag > max_lag) max_lag = real_lag;
            sum_lag += real_lag;
            valid_cnt++;
        }
        float mean_lag = valid_cnt > 0 ? sum_lag / (float)valid_cnt : 0.f;
        summaries.push_back({interval_ms, max_lag, mean_lag});

        {
            std::ostringstream msg;
            msg << std::fixed << std::setprecision(2)
                << "[SteerSweep] interval=" << interval_ms
                << "ms max_lag=" << max_lag
                << "deg mean_lag=" << mean_lag << "deg";
            cmdlog_push(msg.str());
        }
    }

    ctrl_sendCmd("MANUAL");

    // Write single CSV covering all intervals
    {
        std::ofstream csv(csv_name);
        csv << "interval_ms,cmd_index,cmd_deg,actual_deg,lag_deg,timestamp_ms\n";
        for (const auto& r : all_rows) {
            csv << r.interval_ms << "," << r.cmd_index << ","
                << std::fixed << std::setprecision(2) << r.cmd_deg << ",";
            if (!std::isnan(r.actual_deg))
                csv << std::fixed << std::setprecision(2) << r.actual_deg;
            csv << ",";
            if (!std::isnan(r.lag_deg))
                csv << std::fixed << std::setprecision(2) << r.lag_deg;
            csv << "," << std::fixed << std::setprecision(1) << r.timestamp_ms << "\n";
        }
    }

    // Final multi-interval summary to cmdLog
    {
        std::ostringstream summary;
        summary << std::fixed << std::setprecision(2)
                << "[SteerSweep] DONE " << csv_name;
        for (const auto& s : summaries)
            summary << " " << s.interval_ms << "ms=max" << s.max_lag
                    << "/mean" << s.mean_lag << "deg";
        cmdlog_push(summary.str());
    }
}

// =============================================================================
//  TEST A REPEAT  —  run the standard sequence N times to measure scatter
//  Runs in its own detached thread; never blocks the 60 FPS render loop.
//
//  CSV columns:
//    run_index, step_index, cmd_deg, actual_deg, error_deg,
//    latency_ms, dropped, center_offset_deg
//  step_index == -1 rows are the CENTER (zero-return) step before each run.
//  center_offset_deg is filled only on those rows; empty on normal step rows.
// =============================================================================
void steer_test_run_A_repeat(int num_runs) {
    if (g_simMode) {
        cmdlog_push("[SteerRepeat] REFUSED: disable sim mode first");
        return;
    }

    // RAII: set flag on entry, clear on any exit path
    struct Guard {
        std::atomic<bool>& f;
        Guard(std::atomic<bool>& f_) : f(f_) { f = true; }
        ~Guard() { f = false; }
    } guard(g_steer_test_running);

    struct StepResult {
        int   run_index;
        int   step_index;   // -1 = center step, 0..N-1 = normal step
        float cmd_deg;
        float actual_deg;   // NaN if dropped
        float error_deg;    // NaN if dropped; for step -1: offset from 0
        float latency_ms;
        bool  dropped;
        // Only meaningful when step_index == -1
        float center_offset_deg;
    };

    const float targets[] = {0.f, 5.f, 0.f, -5.f, 0.f, 10.f, -10.f, 0.f};
    const int   N         = (int)(sizeof(targets) / sizeof(targets[0]));
    const float kNaN      = std::numeric_limits<float>::quiet_NaN();

    std::vector<StepResult> all_results;
    all_results.reserve(num_runs * (N + 1));

    ctrl_sendCmd("AUTO");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<float> run_mean_abs(num_runs, kNaN);

    float last_cmd    = kNaN;
    float last_actual = kNaN;

    for (int run = 1; run <= num_runs; run++) {
        // ── (a) CENTER step — return wheel to 0° ──────────────────────────
        bool  center_got    = false;
        float center_actual = kNaN;
        float center_lat    = 0.f;

        // FIX 3: skip CENTER wait if already within 0.25° of 0°
        if (!std::isnan(last_cmd) && !std::isnan(last_actual) &&
            std::fabs(0.f - last_cmd) < 0.25f) {
            center_got    = true;
            center_actual = last_actual;
            center_lat    = 0.f;
        } else {
            g_ctrl.thetaValid  = false;
            double center_send = nowSecEpoch();
            ctrl_sendCmd("CENTER");

            float moveDeg_c   = std::isnan(last_cmd) ? 23.0f : fabsf(0.f - last_cmd);
            int   timeoutMs_c = (int)(moveDeg_c / 5.0f * 1000.0f) + 1500;
            double center_dl  = center_send + timeoutMs_c / 1000.0;
            while (nowSecEpoch() < center_dl) {
                pollTelemetry();
                if (g_ctrl.thetaValid) {
                    center_actual = g_ctrl.thetaActual;
                    center_lat    = (float)((nowSecEpoch() - center_send) * 1000.0);
                    center_got    = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (!center_got) center_lat = (float)timeoutMs_c;
        }
        last_cmd    = 0.f;
        if (center_got) last_actual = center_actual;

        StepResult cr;
        cr.run_index          = run;
        cr.step_index         = -1;
        cr.cmd_deg            = 0.f;
        cr.dropped            = !center_got;
        cr.actual_deg         = center_got ? center_actual : kNaN;
        cr.error_deg          = center_got ? center_actual : kNaN;   // deviation from 0
        cr.latency_ms         = center_lat;
        cr.center_offset_deg  = center_got ? center_actual : kNaN;
        all_results.push_back(cr);

        // Log center offset so user sees progress in the panel
        {
            std::ostringstream cl;
            cl << std::fixed << std::setprecision(2)
               << "[SteerRepeat] run " << run << "/" << num_runs
               << " center_offset="
               << (center_got ? center_actual : kNaN) << "deg";
            cmdlog_push(cl.str());
        }

        // ── (b) Target sequence ────────────────────────────────────────────
        float sum_abs    = 0.f;
        int   valid_cnt  = 0;

        for (int i = 0; i < N; i++) {
            float target = targets[i];

            // FIX 3: skip wait if within 0.25° of last commanded angle
            if (!std::isnan(last_cmd) && !std::isnan(last_actual) &&
                std::fabs(target - last_cmd) < 0.25f) {
                StepResult r;
                r.run_index         = run;
                r.step_index        = i;
                r.cmd_deg           = target;
                r.dropped           = false;
                r.actual_deg        = last_actual;
                r.error_deg         = 0.f;
                r.latency_ms        = 0.f;
                r.center_offset_deg = kNaN;
                all_results.push_back(r);
                last_cmd = target;
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
                continue;
            }

            g_ctrl.thetaValid = false;
            double send_time  = nowSecEpoch();
            {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2) << "STEER," << target;
                ctrl_sendCmd(ss.str());
            }

            float moveDeg   = std::isnan(last_cmd) ? 23.0f : fabsf(target - last_cmd);
            int   timeoutMs = (int)(moveDeg / 5.0f * 1000.0f) + 1500;
            bool  got    = false;
            float actual = kNaN;
            float lat    = 0.f;
            double dl    = send_time + timeoutMs / 1000.0;
            while (nowSecEpoch() < dl) {
                pollTelemetry();
                if (g_ctrl.thetaValid) {
                    actual = g_ctrl.thetaActual;
                    lat    = (float)((nowSecEpoch() - send_time) * 1000.0);
                    got    = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            StepResult r;
            r.run_index         = run;
            r.step_index        = i;
            r.cmd_deg           = target;
            r.dropped           = !got;
            r.actual_deg        = got ? actual : kNaN;
            r.error_deg         = got ? (actual - target) : kNaN;
            r.latency_ms        = got ? lat : (float)timeoutMs;
            r.center_offset_deg = kNaN;
            all_results.push_back(r);

            if (got) { sum_abs += std::fabs(r.error_deg); valid_cnt++; }

            last_cmd = target;
            if (got) last_actual = actual;

            std::this_thread::sleep_for(std::chrono::milliseconds(800));
        }

        run_mean_abs[run - 1] = valid_cnt > 0 ? sum_abs / (float)valid_cnt : kNaN;

        // ── (c) Inter-run pause ────────────────────────────────────────────
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    ctrl_sendCmd("MANUAL");

    // --- Write combined CSV ---
    std::time_t now_t = std::time(nullptr);
    std::tm*    tm_p  = std::localtime(&now_t);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", tm_p);
    std::string csv_name = std::string("steer_test_repeat_") + ts_buf + ".csv";

    {
        std::ofstream csv(csv_name);
        csv << "run_index,step_index,cmd_deg,actual_deg,error_deg,"
               "latency_ms,dropped,center_offset_deg\n";
        for (const auto& r : all_results) {
            csv << r.run_index << "," << r.step_index << ","
                << std::fixed << std::setprecision(2) << r.cmd_deg << ",";
            if (!r.dropped && !std::isnan(r.actual_deg))
                csv << std::fixed << std::setprecision(2) << r.actual_deg;
            csv << ",";
            if (!r.dropped && !std::isnan(r.error_deg))
                csv << std::fixed << std::setprecision(2) << r.error_deg;
            csv << ","
                << std::fixed << std::setprecision(1) << r.latency_ms << ","
                << (r.dropped ? 1 : 0) << ",";
            if (r.step_index == -1 && !std::isnan(r.center_offset_deg))
                csv << std::fixed << std::setprecision(2) << r.center_offset_deg;
            csv << "\n";
        }
    }

    // --- Summary: per-run mean abs error + worst run ---
    int   worst_run = 1;
    float worst_val = 0.f;
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(2)
            << "[SteerRepeat] DONE ";
    for (int r = 0; r < num_runs; r++) {
        float v = std::isnan(run_mean_abs[r]) ? 0.f : run_mean_abs[r];
        summary << "run" << (r + 1) << "=" << v << "deg ";
        if (v > worst_val) { worst_val = v; worst_run = r + 1; }
    }
    summary << "worst=run" << worst_run << " " << csv_name;
    cmdlog_push(summary.str());
}
