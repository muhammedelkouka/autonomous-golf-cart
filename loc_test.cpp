// =============================================================================
// loc_test.cpp  —  Localization diagnostic logger (stationary capture)
//
// Reads localization globals read-only. Never writes to localization.cpp state.
// Runs in its own detached thread; never blocks the 60 FPS render loop.
//
// CSV: loc_test_stationary_YYYYMMDD_HHMMSS.csv
// Line 1: # comment with calibration values
// Line 2: column header
// Lines 3+: data rows at ~50 Hz
// =============================================================================
#include "common.hpp"
#include <cmath>

std::atomic<bool> g_loc_test_running{false};

void loc_test_run_stationary(int duration_sec) {
    // RAII: set flag on entry, clear on any exit path
    struct Guard {
        std::atomic<bool>& f;
        Guard(std::atomic<bool>& f_) : f(f_) { f = true; }
        ~Guard() { f = false; }
    } guard(g_loc_test_running);

    // (a) Wait until calibration is complete
    double wait_start = nowSecEpoch();
    while (calib_state.is_calibrating.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (nowSecEpoch() - wait_start > 10.0) {
            cmdlog_push("[LocTest] WARNING: calibration took >10s — proceeding anyway");
            break;
        }
    }

    // (b) Capture calibration result once (sum_acc is stable after is_calibrating=false)
    Vector3 gbias = {};
    {
        std::lock_guard<std::mutex> lk(imu_mutex);
        gbias = gyro_bias_init;
    }
    int    cal_samples = calib_state.samples.load();
    double acc_mean[3] = {};
    if (cal_samples > 0) {
        acc_mean[0] = calib_state.sum_acc[0] / cal_samples;
        acc_mean[1] = calib_state.sum_acc[1] / cal_samples;
        acc_mean[2] = calib_state.sum_acc[2] / cal_samples;
    }

    // Timestamp for filename (fixed at test start)
    std::time_t now_t = std::time(nullptr);
    std::tm*    tm_p  = std::localtime(&now_t);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", tm_p);
    std::string csv_name = std::string("loc_test_stationary_") + ts_buf + ".csv";

    {
        std::ostringstream msg;
        msg << "[LocTest] Started " << duration_sec << "s stationary capture -> " << csv_name;
        cmdlog_push(msg.str());
    }

    // (c) Collect rows at ~50 Hz
    struct Row {
        float time_s;
        float imu_acc_x,  imu_acc_y,  imu_acc_z;
        float imu_gyro_x, imu_gyro_y, imu_gyro_z;
        float imu_yaw_deg;
        float ekf_x, ekf_y, ekf_yaw_deg, ekf_vx, ekf_vy;
        float scan_rmse, inlier_ratio;
        int   is_stationary, is_turning;
        int   lidar_sweeps, imu_frames;
    };

    std::vector<Row> rows;
    rows.reserve(duration_sec * 52);  // slight headroom over 50 Hz

    double t_start = nowSecEpoch();
    double t_end   = t_start + (double)duration_sec;

    while (nowSecEpoch() < t_end) {
        Row r{};
        r.time_s = (float)(nowSecEpoch() - t_start);

        {
            std::lock_guard<std::mutex> lk(imu_mutex);
            r.imu_acc_x   = imu_acc.x;
            r.imu_acc_y   = imu_acc.y;
            r.imu_acc_z   = imu_acc.z;
            r.imu_gyro_x  = imu_gyro.x;
            r.imu_gyro_y  = imu_gyro.y;
            r.imu_gyro_z  = imu_gyro.z;
            r.imu_yaw_deg = imu_angle.z;
        }

        {
            std::lock_guard<std::mutex> lk(ekf_mutex);
            r.ekf_x       = (float)ekf.x[0];
            r.ekf_y       = (float)ekf.x[1];
            r.ekf_yaw_deg = toDeg((float)ekf.x[2]);
            r.ekf_vx      = (float)ekf.x[3];
            r.ekf_vy      = (float)ekf.x[4];
        }

        r.scan_rmse     = current_rmse.load();
        r.inlier_ratio  = current_inlier_ratio.load();
        r.is_stationary = g_is_stationary.load() ? 1 : 0;
        r.is_turning    = g_is_turning.load()    ? 1 : 0;
        r.lidar_sweeps  = lidar_sweeps_total.load();
        r.imu_frames    = imu_frames_total.load();

        rows.push_back(r);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // (d) Write CSV
    {
        std::ofstream csv(csv_name);
        csv << std::fixed << std::setprecision(6);
        csv << "# gyro_bias_x=" << gbias.x
            << " gyro_bias_y=" << gbias.y
            << " gyro_bias_z=" << gbias.z
            << " accel_mean_x=" << acc_mean[0]
            << " accel_mean_y=" << acc_mean[1]
            << " accel_mean_z=" << acc_mean[2] << "\n";
        csv << "time_s,imu_acc_x,imu_acc_y,imu_acc_z,"
               "imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_yaw_deg,"
               "ekf_x,ekf_y,ekf_yaw_deg,ekf_vx,ekf_vy,"
               "scan_rmse,inlier_ratio,is_stationary,is_turning,"
               "lidar_sweeps,imu_frames\n";
        for (const auto& r : rows) {
            csv << r.time_s       << ","
                << r.imu_acc_x   << "," << r.imu_acc_y   << "," << r.imu_acc_z   << ","
                << r.imu_gyro_x  << "," << r.imu_gyro_y  << "," << r.imu_gyro_z  << ","
                << r.imu_yaw_deg << ","
                << r.ekf_x       << "," << r.ekf_y        << ","
                << r.ekf_yaw_deg << ","
                << r.ekf_vx      << "," << r.ekf_vy       << ","
                << r.scan_rmse   << "," << r.inlier_ratio  << ","
                << r.is_stationary << "," << r.is_turning  << ","
                << r.lidar_sweeps  << "," << r.imu_frames  << "\n";
        }
    }

    // (e) Summary to g_cmdLog
    if (!rows.empty()) {
        int   n        = (int)rows.size();
        float yaw_sum  = 0.f, yaw_sum2 = 0.f;
        float rmse_sum = 0.f;
        int   stat_cnt = 0;
        for (const auto& r : rows) {
            yaw_sum  += r.ekf_yaw_deg;
            yaw_sum2 += r.ekf_yaw_deg * r.ekf_yaw_deg;
            rmse_sum += r.scan_rmse;
            if (r.is_stationary) stat_cnt++;
        }
        float yaw_mean = yaw_sum / (float)n;
        float yaw_var  = yaw_sum2 / (float)n - yaw_mean * yaw_mean;
        float yaw_std  = (yaw_var > 0.f) ? std::sqrt(yaw_var) : 0.f;
        float rmse_mean = rmse_sum / (float)n;
        int   stat_pct  = 100 * stat_cnt / n;

        std::ostringstream summary;
        summary << std::fixed << std::setprecision(3)
                << "[LocTest] DONE  ekf_yaw=" << yaw_mean << "+/-" << yaw_std
                << "deg  rmse=" << rmse_mean
                << "m  stationary=" << stat_pct << "%  "
                << csv_name;
        cmdlog_push(summary.str());
    }
}
