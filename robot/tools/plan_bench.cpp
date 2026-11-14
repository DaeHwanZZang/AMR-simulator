// 경로계획 벤치. 맵 하나에서 plan_path (+ 곡선화) 시간을 잰다. MQTT/물리 없이 돈다.
//   build/bin/plan_bench MAP X0 Y0 X1 Y1 [--repeat N] [--dump FILE] [--no-center] [--no-smooth]
//                        [--width W] [--length L] [--obstacle X,Y,W,H ...]
// --dump 는 경유점 좌표를 파일로 남긴다 (planner 를 고친 전후 결과가 같은지 diff 로 본다).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "map_model.hpp"
#include "path_refine.hpp"
#include "planner.hpp"

using namespace fms;

namespace {

double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: plan_bench MAP X0 Y0 X1 Y1 [--repeat N] [--dump FILE] [--no-center] [--no-smooth]\n"
                     "                  [--width W] [--length L] [--obstacle X,Y,W,H ...]\n";
        return 2;
    }
    const std::string map_path = argv[1];
    const std::pair<double, double> start{std::stod(argv[2]), std::stod(argv[3])};
    const std::pair<double, double> goal{std::stod(argv[4]), std::stod(argv[5])};
    int repeat = 5;
    std::string dump;
    bool center = true, smooth = true;
    double width = 0.5, length = 0.7;
    std::vector<std::tuple<double, double, double, double>> obstacles;
    for (int i = 6; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { std::cerr << k << " needs a value\n"; std::exit(2); }
            return argv[++i];
        };
        if (k == "--repeat") repeat = std::max(1, std::stoi(next()));
        else if (k == "--dump") dump = next();
        else if (k == "--no-center") center = false;
        else if (k == "--no-smooth") smooth = false;
        else if (k == "--width") width = std::stod(next());
        else if (k == "--length") length = std::stod(next());
        else if (k == "--obstacle") {
            double x, y, w, h;
            if (std::sscanf(next().c_str(), "%lf,%lf,%lf,%lf", &x, &y, &w, &h) != 4) { std::cerr << "--obstacle X,Y,W,H\n"; return 2; }
            obstacles.emplace_back(x, y, w, h);
        } else { std::cerr << "unknown option: " << k << "\n"; return 2; }
    }

    try {
        auto t0 = std::chrono::steady_clock::now();
        MapModel map = MapModel::load(map_path);
        map.classify_grid();
        std::printf("map %dx%d res %.5f  load %.1f ms\n", map.metadata.width_px, map.metadata.height_px,
                    map.metadata.resolution, ms_since(t0));

        CenterlineParams centerline;
        centerline.enabled = center;
        std::unique_ptr<ClearanceField> clearance;
        if (center) {
            t0 = std::chrono::steady_clock::now();
            clearance = std::make_unique<ClearanceField>(map);
            std::printf("clearance field %.1f ms\n", ms_since(t0));
        }
        const auto* obs = obstacles.empty() ? nullptr : &obstacles;

        std::vector<std::pair<double, double>> path, curved;
        std::vector<double> plan_ms, smooth_ms;
        for (int r = 0; r < repeat; ++r) {
            t0 = std::chrono::steady_clock::now();
            path = plan_path(map, start, goal, width, length, obs, nullptr, clearance.get(), &centerline);
            plan_ms.push_back(ms_since(t0));
            if (smooth) {
                t0 = std::chrono::steady_clock::now();
                curved = smooth_path(map, path, width, length, obs, nullptr, SmoothParams());
                smooth_ms.push_back(ms_since(t0));
            }
        }
        auto report = [](const char* name, std::vector<double> v) {
            if (v.empty()) return;
            const double first = v.front();
            std::sort(v.begin(), v.end());
            std::printf("%s: first %.1f ms, min %.1f, median %.1f, max %.1f (n=%zu)\n", name, first, v.front(),
                        v[v.size() / 2], v.back(), v.size());
        };
        report("plan_path", plan_ms);
        report("smooth_path", smooth_ms);
        std::printf("waypoints %zu, curved samples %zu\n", path.size(), curved.size());

        if (!dump.empty()) {
            std::FILE* f = std::fopen(dump.c_str(), "w");
            if (!f) { std::cerr << "cannot write " << dump << "\n"; return 1; }
            std::fprintf(f, "# plan_path\n");
            for (auto& p : path) std::fprintf(f, "%.9f %.9f\n", p.first, p.second);
            std::fprintf(f, "# smooth_path\n");
            for (auto& p : curved) std::fprintf(f, "%.9f %.9f\n", p.first, p.second);
            std::fclose(f);
        }
    } catch (const std::exception& e) {
        std::cerr << "plan_bench: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
