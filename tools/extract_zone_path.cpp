// extract_zone_path.cpp — one-off offline tool, no Raylib dependency.
//
// Cuts a contiguous sub-segment out of a full recorded VTK "Trajectory path.ply"
// (thousands of points spanning the whole build drive) using a zone polygon
// (same "x y" per-line format as zone1_<map>.cfg / zone2_<map>.cfg) as the crop
// boundary, and writes it as a trajectory CSV in the exact format
// pp_record_path_snapshot() / pp_run_pipeline() expect (x,y,z,yaw,rmse,
// inlier_ratio,timestamp,is_turning) — matching path_forward.csv / path_reverse.csv,
// which were cut from these same PLY files the same way for zone 1.
//
// Usage: extract_zone_path.exe <trajectory.ply> <zone_polygon.cfg> <out.csv>
//
// The PLY and the zone polygon must already be in the same coordinate frame —
// true here because Campus_TopLidar_Forward/Reverse.ply were built by merging
// this same Trajectory path.ply with the other VTK exports from one recording
// session, so the zone polygons (drawn against the merged map) line up directly.
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

struct Pt { float x, y, z; };

static bool load_ply(const std::string& path, std::vector<Pt>& out) {
    out.clear();
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return false;
    std::string line;
    int vcnt = 0, xi = -1, yi = -1, zi = -1, pc = 0;
    bool bin_le = false, hd = false, in_vertex = false;
    std::vector<int> psz;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "end_header") { hd = true; break; }
        if (line.rfind("format", 0) == 0 && line.find("binary_little_endian") != std::string::npos) bin_le = true;
        else if (line.rfind("element vertex", 0) == 0) {
            std::istringstream ss(line); std::string a, b; ss >> a >> b >> vcnt; in_vertex = true;
        } else if (line.rfind("element", 0) == 0) {
            in_vertex = false;
        } else if (in_vertex && line.rfind("property", 0) == 0) {
            std::istringstream ss(line); std::string kw, tp, nm; ss >> kw >> tp >> nm;
            int sz = (tp == "double" || tp == "float64") ? 8 : 4;
            if (nm == "x") xi = pc; if (nm == "y") yi = pc; if (nm == "z") zi = pc;
            psz.push_back(sz); pc++;
        }
    }
    if (!hd || vcnt == 0 || xi < 0) return false;
    out.reserve(vcnt);
    int stride = 0; std::vector<int> off(pc);
    for (int i = 0; i < pc; i++) { off[i] = stride; stride += psz[i]; }
    auto rf = [&](const char* row, int idx) -> float {
        const char* ptr = row + off[idx];
        if (psz[idx] == 8) { double d; memcpy(&d, ptr, 8); return (float)d; }
        float v; memcpy(&v, ptr, 4); return v;
    };
    if (bin_le) {
        std::vector<char> row(stride);
        for (int i = 0; i < vcnt; i++) {
            f.read(row.data(), stride);
            if (!f) break;
            out.push_back({ rf(row.data(), xi), rf(row.data(), yi), rf(row.data(), zi) });
        }
    } else {
        for (int i = 0; i < vcnt; i++) {
            if (!std::getline(f, line)) break;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            std::istringstream ss(line);
            std::vector<double> vals(pc);
            for (int k = 0; k < pc; k++) ss >> vals[k];
            out.push_back({ (float)vals[xi], (float)vals[yi], (float)vals[zi] });
        }
    }
    return !out.empty();
}

static bool load_polygon(const std::string& path, std::vector<Pt>& poly) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line; float vx, vy;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (sscanf(line.c_str(), "%f %f", &vx, &vy) == 2) poly.push_back({ vx, vy, 0 });
    }
    return poly.size() >= 3;
}

static bool point_in_polygon(float px, float py, const std::vector<Pt>& poly) {
    int n = (int)poly.size(); if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i].y > py) != (poly[j].y > py)) &&
            (px < (poly[j].x - poly[i].x) * (py - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
            inside = !inside;
    }
    return inside;
}

static void write_csv(const std::vector<Pt>& traj, int first, int last, const char* outPath) {
    std::ofstream out(outPath);
    if (!out.is_open()) { fprintf(stderr, "Failed to open output: %s\n", outPath); return; }
    out << "x,y,z,yaw,rmse,inlier_ratio,timestamp,is_turning\n";
    char buf[256];
    int ts = 0;
    for (int i = first; i <= last; i++) {
        snprintf(buf, sizeof(buf), "%.6f,%.6f,0.200000,0.000000,0.000000,1.000000,%d.000000,0\n",
                 traj[i].x, traj[i].y, ts++);
        out << buf;
    }
    fprintf(stderr, "Wrote %d points (index %d..%d) to %s\n", last - first + 1, first, last, outPath);
}

int main(int argc, char** argv) {
    // Mode 1: --list <trajectory.ply> [step=25] — scan the full trajectory to find
    // the index range for a corner by eye (index, x, y, cumulative distance from
    // point 0). Use this to find <start>/<end> for --range.
    if (argc >= 3 && std::string(argv[1]) == "--list") {
        std::vector<Pt> traj;
        if (!load_ply(argv[2], traj)) { fprintf(stderr, "Failed to load PLY: %s\n", argv[2]); return 1; }
        int step = (argc >= 4) ? atoi(argv[3]) : 25;
        if (step < 1) step = 1;
        double cum = 0.0;
        printf("index,x,y,cum_dist_m\n");
        for (int i = 0; i < (int)traj.size(); i++) {
            if (i > 0) {
                float dx = traj[i].x - traj[i-1].x, dy = traj[i].y - traj[i-1].y;
                cum += sqrt(dx*dx + dy*dy);
            }
            if (i % step == 0 || i == (int)traj.size()-1)
                printf("%d,%.3f,%.3f,%.2f\n", i, traj[i].x, traj[i].y, cum);
        }
        return 0;
    }
    // Mode 2: --range <trajectory.ply> <start_idx> <end_idx> <out.csv> — extract an
    // exact, manually-chosen index range (found via --list).
    if (argc == 6 && std::string(argv[1]) == "--range") {
        std::vector<Pt> traj;
        if (!load_ply(argv[2], traj)) { fprintf(stderr, "Failed to load PLY: %s\n", argv[2]); return 1; }
        int start = atoi(argv[3]), end = atoi(argv[4]);
        if (start < 0) start = 0;
        if (end >= (int)traj.size()) end = (int)traj.size()-1;
        if (start > end) { fprintf(stderr, "start (%d) > end (%d)\n", start, end); return 1; }
        write_csv(traj, start, end, argv[5]);
        return 0;
    }
    // Mode 3 (default): <trajectory.ply> <zone_polygon.cfg> <out.csv> — crop to the
    // contiguous range where the trajectory is inside the zone polygon.
    if (argc != 4) {
        fprintf(stderr,
            "Usage:\n"
            "  %s --list <trajectory.ply> [step=25]\n"
            "  %s --range <trajectory.ply> <start_idx> <end_idx> <out.csv>\n"
            "  %s <trajectory.ply> <zone_polygon.cfg> <out.csv>\n", argv[0], argv[0], argv[0]);
        return 1;
    }
    std::vector<Pt> traj, poly;
    if (!load_ply(argv[1], traj)) { fprintf(stderr, "Failed to load PLY: %s\n", argv[1]); return 1; }
    if (!load_polygon(argv[2], poly)) { fprintf(stderr, "Failed to load zone polygon: %s\n", argv[2]); return 1; }
    fprintf(stderr, "Loaded %zu trajectory points, %zu-vertex zone polygon\n", traj.size(), poly.size());

    // Find the contiguous [first, last] index range where the trajectory enters
    // and leaves the zone polygon — contiguous (not just "every point that tests
    // inside") so a few boundary-noise points that flicker outside don't punch
    // holes in the extracted curve.
    int first = -1, last = -1;
    for (int i = 0; i < (int)traj.size(); i++) {
        if (point_in_polygon(traj[i].x, traj[i].y, poly)) {
            if (first < 0) first = i;
            last = i;
        }
    }
    if (first < 0) { fprintf(stderr, "No trajectory points fall inside the zone polygon.\n"); return 1; }
    fprintf(stderr, "Zone-crossing range: index %d..%d (%d points)\n", first, last, last - first + 1);
    write_csv(traj, first, last, argv[3]);
    return 0;
}
