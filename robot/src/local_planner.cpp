#include "local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "log.hpp"

namespace fms {

namespace {

struct Sampled {
    std::vector<LpPoint> pts;
    std::vector<double> arc;  // pts[i] 까지 누적 호 길이 (시작점 기준)
    size_t seg = 0;           // 시작점이 속한 nominal 구간 번호
    std::vector<double> vertex_arc;  // nominal[seg+1+k] 까지의 호 길이
};

LpPoint project_on_segment(LpPoint a, LpPoint b, LpPoint p) {
    double vx = b.first - a.first, vy = b.second - a.second;
    double len2 = vx * vx + vy * vy;
    double t = len2 > 0 ? std::clamp(((p.first - a.first) * vx + (p.second - a.second) * vy) / len2, 0.0, 1.0) : 0.0;
    return {a.first + t * vx, a.second + t * vy};
}

Sampled resample_from(const std::vector<LpPoint>& nominal, size_t seg, LpPoint pos, double step) {
    Sampled s;
    s.seg = std::min(seg, nominal.size() - 2);
    LpPoint q = project_on_segment(nominal[s.seg], nominal[s.seg + 1], pos);
    s.pts.push_back(q);
    s.arc.push_back(0.0);
    LpPoint cur = q;
    double acc = 0.0;
    for (size_t i = s.seg + 1; i < nominal.size(); ++i) {
        double seg_len = std::hypot(nominal[i].first - cur.first, nominal[i].second - cur.second);
        double dx = seg_len > 1e-9 ? (nominal[i].first - cur.first) / seg_len : 0.0;
        double dy = seg_len > 1e-9 ? (nominal[i].second - cur.second) / seg_len : 0.0;
        double next_mark = std::ceil((acc + 1e-9) / step) * step;
        for (double m = next_mark; m < acc + seg_len - 1e-9; m += step) {
            double u = m - acc;
            s.pts.push_back({cur.first + dx * u, cur.second + dy * u});
            s.arc.push_back(m);
        }
        acc += seg_len;
        cur = nominal[i];
        s.pts.push_back(cur);
        s.arc.push_back(acc);
        s.vertex_arc.push_back(acc);
    }
    return s;
}

bool inside_any(LpPoint p, const std::vector<LpRect>& obstacles, double margin) {
    for (auto& [ox, oy, ow, oh] : obstacles)
        if (std::abs(p.first - ox) <= ow / 2 + margin && std::abs(p.second - oy) <= oh / 2 + margin) return true;
    return false;
}

double polyline_length(const std::vector<LpPoint>& p) {
    double len = 0.0;
    for (size_t i = 0; i + 1 < p.size(); ++i) len += std::hypot(p[i + 1].first - p[i].first, p[i + 1].second - p[i].second);
    return len;
}

// nominal 에서 cand 호 길이 이후의 꼭짓점들을 tail 로 붙인다
void append_tail(std::vector<LpPoint>& out, const std::vector<LpPoint>& nominal, const Sampled& s, double after_arc) {
    for (size_t k = 0; k < s.vertex_arc.size(); ++k)
        if (s.vertex_arc[k] > after_arc + 1e-6) out.push_back(nominal[s.seg + 1 + k]);
}

}  // namespace

size_t lp_advance_progress(const std::vector<LpPoint>& nominal, size_t progress, LpPoint pos) {
    if (nominal.size() < 2) return 0;
    size_t last = nominal.size() - 2;
    size_t best = std::min(progress, last), hi = std::min(last, best + 15);
    double best_d = 1e18;
    for (size_t i = best; i <= hi; ++i) {
        LpPoint q = project_on_segment(nominal[i], nominal[i + 1], pos);
        double d = std::hypot(q.first - pos.first, q.second - pos.second);
        if (d < best_d - 1e-9) { best_d = d; best = i; }
    }
    return best;
}

double lp_blocked_distance(const std::vector<LpPoint>& pts, const std::vector<LpRect>& obstacles, double robot_width) {
    if (pts.size() < 2 || obstacles.empty()) return -1.0;
    // 우회 경로는 팽창 경계(반폭)에 붙어서 만들어지므로, 막힘 판정은 5 cm 덜 엄격하게 해서 자기가 만든 경로를 막힘으로 보지 않게 한다
    const double margin = std::max(0.0, robot_width / 2 - 0.05);
    double travelled = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        double len = std::hypot(pts[i + 1].first - pts[i].first, pts[i + 1].second - pts[i].second);
        int n = std::max(1, (int)std::ceil(len / 0.05));
        for (int k = 0; k <= n; ++k) {
            double t = (double)k / n;
            LpPoint p{pts[i].first + t * (pts[i + 1].first - pts[i].first), pts[i].second + t * (pts[i + 1].second - pts[i].second)};
            if (inside_any(p, obstacles, margin)) return travelled + t * len;
        }
        travelled += len;
    }
    return -1.0;
}

std::optional<std::vector<LpPoint>> LocalPlanner::detour(
    const MapModel& map, LpPoint pos, const std::vector<LpPoint>& nominal, size_t progress,
    const std::vector<LpRect>& obstacles, double robot_width, double robot_length,
    const std::vector<std::string>* allowed, const ClearanceField* clearance, const CenterlineParams* centering,
    const SmoothParams* smooth) {
    if (nominal.size() < 2 || obstacles.empty()) return std::nullopt;
    // 계획은 장애물을 plan_margin 만큼 더 키워서 한다: 추종 오차/조향 지연이 있어도 막힘 판정 구역(반폭 - 5 cm)까지 들어가지 않게
    std::vector<LpRect> inflated_obs;
    for (auto& [ox, oy, ow, oh] : obstacles) inflated_obs.emplace_back(ox, oy, ow + 2 * config.plan_margin, oh + 2 * config.plan_margin);
    const double margin = robot_width / 2;
    size_t seg = lp_advance_progress(nominal, progress, pos);
    Sampled s = resample_from(nominal, seg, pos, config.sample_step);
    const size_t N = s.pts.size();

    size_t fb = N;
    for (size_t i = 0; i < N; ++i) if (inside_any(s.pts[i], inflated_obs, margin)) { fb = i; break; }
    if (fb == N) return std::nullopt;  // nominal 이 막히지 않았다 -> 우회할 필요 없음
    size_t ie = fb;
    while (ie < N && inside_any(s.pts[ie], inflated_obs, margin)) ++ie;
    if (ie >= N) return std::nullopt;  // 목표가 장애물 안
    size_t cand = std::min(N - 1, ie + (size_t)std::ceil(config.rejoin_margin / config.sample_step));
    while (cand < N - 1 && inside_any(s.pts[cand], inflated_obs, margin)) ++cand;
    if (inside_any(s.pts[cand], inflated_obs, margin)) return std::nullopt;

    auto dist_pos = [&](size_t i) { return std::hypot(s.pts[i].first - pos.first, s.pts[i].second - pos.second); };
    bool short_rejoin = false;  // 창이 좁아 rejoin_margin 을 못 채웠다 (합류점이 장애물에 바짝 붙는다)
    if (dist_pos(cand) > config.radius * 0.9) {  // 창 밖: 창 안에서 막힘 구간 뒤쪽으로 가장 먼 자유 지점
        size_t pick = N;
        for (size_t j = ie; j < N; ++j)
            if (dist_pos(j) <= config.radius * 0.9 && !inside_any(s.pts[j], inflated_obs, margin)) pick = j;
        if (pick == N) return std::nullopt;
        short_rejoin = pick < cand;
        cand = pick;
    }
    LpPoint goal = (cand == N - 1) ? nominal.back() : s.pts[cand];

    std::vector<LpPoint> path;
    try {
        path = plan_path_window(map, pos, goal, pos, config.radius, robot_width, robot_length, inflated_obs, allowed, clearance, centering);
    } catch (const PlanningError&) {
        return std::nullopt;
    }
    // 합류점은 장애물 바로 뒤 nominal 위에 잡는다. 그런데 우회로가 반대편으로 돌아 들어가면 nominal 뒤쪽(심하면 목적지)을
    // 지나쳐 합류점까지 갔다가 되돌아오게 된다. 창 안에서 nominal 의 가장 뒤쪽 자유 지점으로 바로 가는 경로가
    // "합류점 경유 + nominal 을 따라 그 지점까지" 보다 확실히 짧으면 그 경로를 쓴다 (구간은 노드 하나라 nominal 중간을 건너뛰어도 된다).
    size_t far = cand;
    for (size_t j = cand + 1; j < N; ++j)
        if (dist_pos(j) <= config.radius * 0.9 && !inside_any(s.pts[j], inflated_obs, margin)) far = j;
    if (far > cand) {
        LpPoint far_goal = (far == N - 1) ? nominal.back() : s.pts[far];
        try {
            auto direct = plan_path_window(map, pos, far_goal, pos, config.radius, robot_width, robot_length, inflated_obs, allowed, clearance, centering);
            if (polyline_length(direct) + config.shortcut_gain < polyline_length(path) + (s.arc[far] - s.arc[cand])) {
                path = std::move(direct);
                cand = far;
            }
        } catch (const PlanningError&) {
        }
    }
    // 합류점이 장애물에 바짝 붙으면 거기서 nominal 로 급하게 꺾는 코너(+ 수 cm 토막 구간)가 생겨 로봇이 그 자리에서 헌팅한다.
    // 이때는 합류점(과 그 앞 경유점)을 건너뛰고 nominal 의 다음 꼭짓점으로 곧장 간다 (직선 시야가 열려 있는 만큼만).
    if (short_rejoin) {
        const LpPoint* next = nullptr;
        for (size_t k = 0; k < s.vertex_arc.size() && !next; ++k)
            if (s.vertex_arc[k] > s.arc[cand] + 1e-6) next = &nominal[s.seg + 1 + k];
        if (next) {
            const auto& grid = static_planning_grid(map, robot_width + (smooth ? 2 * smooth->extra_clearance : 0.0));
            const int w = map.metadata.width_px, h = map.metadata.height_px;
            auto sees = [&](LpPoint a, LpPoint b) {
                double len = std::hypot(b.first - a.first, b.second - a.second);
                int n = std::max(1, (int)std::ceil(len / (map.metadata.resolution * 0.5)));
                for (int k = 0; k <= n; ++k) {
                    double t = (double)k / n, x = a.first + t * (b.first - a.first), y = a.second + t * (b.second - a.second);
                    auto [px, py] = map.world_to_pixel(x, y);
                    if (px < 0 || px >= w || py < 0 || py >= h || grid[(size_t)py * w + px] != 2 || inside_any({x, y}, inflated_obs, margin)) return false;
                }
                return true;
            };
            while (path.size() > 1 && sees(path[path.size() - 2], *next)) path.pop_back();
        }
    }
    append_tail(path, nominal, s, s.arc[cand]);

    if (smooth) {
        // 곡선은 smooth_path 처럼 A* 팽창보다 extra_clearance 만큼 더 넉넉한 벽 여유를 요구한다 (좁은 복도 모퉁이에서 추종 오차로 벽을 스치지 않게)
        const auto& grid = static_planning_grid(map, robot_width + 2 * smooth->extra_clearance);
        const int w = map.metadata.width_px, h = map.metadata.height_px;
        auto free_at = [&](double x, double y) {
            auto [px, py] = map.world_to_pixel(x, y);
            return px >= 0 && px < w && py >= 0 && py < h && grid[(size_t)py * w + px] == 2 && !inside_any({x, y}, obstacles, margin + 0.5 * config.plan_margin);  // 곡선은 plan_margin 의 절반까지만 코너를 자를 수 있다
        };
        path = smooth_polyline(path, free_at, map.metadata.resolution, *smooth);
    }
    return path;
}

std::optional<std::vector<LpPoint>> LocalPlanner::rejoin(
    const MapModel& map, LpPoint pos, const std::vector<LpPoint>& nominal, size_t progress,
    double robot_width, const SmoothParams* smooth) {
    if (nominal.size() < 2) return std::nullopt;
    size_t seg = lp_advance_progress(nominal, progress, pos);
    Sampled s = resample_from(nominal, seg, pos, config.sample_step);
    size_t idx = std::min(s.pts.size() - 1, (size_t)std::ceil(config.rejoin_ahead / config.sample_step));
    LpPoint target = s.pts[idx];

    const auto& grid = static_planning_grid(map, robot_width);
    const int w = map.metadata.width_px, h = map.metadata.height_px;
    auto free_at = [&](double x, double y) {
        auto [px, py] = map.world_to_pixel(x, y);
        return px >= 0 && px < w && py >= 0 && py < h && grid[(size_t)py * w + px] == 2;
    };
    // 지금 위치에서 합류점까지 직선 시야 확인
    double len = std::hypot(target.first - pos.first, target.second - pos.second);
    int n = std::max(1, (int)std::ceil(len / map.metadata.resolution));
    for (int k = 1; k <= n; ++k) {
        double t = (double)k / n;
        if (!free_at(pos.first + t * (target.first - pos.first), pos.second + t * (target.second - pos.second))) return std::nullopt;
    }
    std::vector<LpPoint> path = {pos, target};
    append_tail(path, nominal, s, s.arc[idx]);
    if (smooth) {
        const auto& wide = static_planning_grid(map, robot_width + 2 * smooth->extra_clearance);  // 곡선은 벽 여유를 더 둔다 (detour 와 같다)
        auto free_wide = [&](double x, double y) {
            auto [px, py] = map.world_to_pixel(x, y);
            return px >= 0 && px < w && py >= 0 && py < h && wide[(size_t)py * w + px] == 2;
        };
        path = smooth_polyline(path, free_wide, map.metadata.resolution, *smooth);
    }
    return path;
}

double LocalPlanner::p95_ms() const {
    if (recent_.empty()) return 0.0;
    std::vector<double> v(recent_.begin(), recent_.end());
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, (size_t)std::ceil(v.size() * 0.95) - 1)];
}

void LocalPlanner::record(const std::string& robot_id, double ms) {
    recent_.push_back(ms);
    if (recent_.size() > 200) recent_.pop_front();
    if (++total_ == 1 || total_ % 20 == 0)
        log::info("[%s] local plan: n=%zu p95=%.1f ms (last %.1f ms)", robot_id.c_str(), total_, p95_ms(), ms);
}

}  // namespace fms
