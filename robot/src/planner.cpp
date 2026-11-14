#include "planner.hpp"

#include "path_refine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <map>
#include <new>
#include <queue>

namespace fms {

namespace {

using Px = std::pair<int, int>;

const std::vector<std::tuple<int, int, double>> kNeighbors = {
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, 1.41421356}, {1, -1, 1.41421356}, {-1, 1, 1.41421356}, {-1, -1, 1.41421356},
};

// 정적 팽창 grid 캐시. 맵은 런타임에 안 바뀐다고 가정한다 (planner.py 의 _inflation_cache 와 동일 목적)
std::map<std::pair<const MapModel*, int>, std::vector<uint8_t>> g_inflation_cache;

}  // namespace

// 비-FREE 칸을 반지름 radius_px 원판으로 팽창시킨다: 어떤 칸에서 (dx, dy), dx^2 + dy^2 <= radius_px^2 안에
// 비-FREE 칸이 있으면 그 칸은 0 이 된다 (맵 밖은 막힘으로 치지 않는다). tools/clean_map.py inflate_free 와 같은 규칙.
// 구현: 행마다 "같은 행에서 가장 가까운 비-FREE 칸까지의 가로 거리"를 구해 두고, 세로 오프셋 dy 마다 그 행의 가로 거리가
// 원판의 가로 반폭 floor(sqrt(r^2 - dy^2)) 이하인지 본다. 원판의 모든 오프셋으로 grid 를 밀어 OR 하던 예전 구현과 결과가
// 같고 (tests/unit 이 무차별 대입과 비교한다), gachon(3872x2272) 에서 수 초 -> 수십 ms 다.
std::vector<uint8_t> inflate_grid(const std::vector<uint8_t>& grid, int w, int h, int radius_px) {
    if (radius_px <= 0) return grid;
    const int cap = radius_px + 1;
    std::vector<uint16_t> hdist(grid.size());
    for (int y = 0; y < h; ++y) {
        const uint8_t* src = grid.data() + (size_t)y * w;
        uint16_t* row = hdist.data() + (size_t)y * w;
        int d = cap;
        for (int x = 0; x < w; ++x) {
            d = src[x] != 2 ? 0 : std::min(cap, d + 1);
            row[x] = (uint16_t)d;
        }
        d = cap;
        for (int x = w - 1; x >= 0; --x) {
            d = src[x] != 2 ? 0 : std::min(cap, d + 1);
            if (d < row[x]) row[x] = (uint16_t)d;
        }
    }
    std::vector<uint16_t> half(2 * radius_px + 1);  // dy 별 원판 가로 반폭
    for (int dy = -radius_px; dy <= radius_px; ++dy) {
        int k = 0;
        while ((k + 1) * (k + 1) + dy * dy <= radius_px * radius_px) ++k;
        half[dy + radius_px] = (uint16_t)k;
    }
    std::vector<uint8_t> out((size_t)w * h);
    std::vector<uint8_t> hit(w);
    for (int y = 0; y < h; ++y) {
        std::fill(hit.begin(), hit.end(), 0);
        for (int dy = -radius_px; dy <= radius_px; ++dy) {
            const int yy = y + dy;
            if (yy < 0 || yy >= h) continue;
            const uint16_t* row = hdist.data() + (size_t)yy * w;
            const uint16_t k = half[dy + radius_px];
            for (int x = 0; x < w; ++x) hit[x] |= (uint8_t)(row[x] <= k);
        }
        const uint8_t* src = grid.data() + (size_t)y * w;
        uint8_t* dst = out.data() + (size_t)y * w;
        for (int x = 0; x < w; ++x) dst[x] = hit[x] ? 0 : src[x];  // 비-FREE 칸은 hdist 0 이라 항상 hit
    }
    return out;
}

namespace {

const std::vector<uint8_t>& get_inflated_grid(const MapModel& map_model, int radius_px) {
    auto key = std::make_pair(&map_model, radius_px);
    auto it = g_inflation_cache.find(key);
    if (it != g_inflation_cache.end()) return it->second;
    const auto& grid = map_model.classify_grid();
    auto inflated = inflate_grid(grid, map_model.metadata.width_px, map_model.metadata.height_px, radius_px);
    return g_inflation_cache.emplace(key, std::move(inflated)).first->second;
}

std::vector<uint8_t> apply_dynamic_obstacles(
    const MapModel& map_model, std::vector<uint8_t> grid, int w, int h,
    const std::vector<std::tuple<double, double, double, double>>& obstacles, double robot_width) {
    double margin = robot_width / 2;
    for (auto& [ox, oy, ow, oh] : obstacles) {
        auto [x0, y0] = map_model.world_to_pixel(ox - ow / 2 - margin, oy - oh / 2 - margin);
        auto [x1, y1] = map_model.world_to_pixel(ox + ow / 2 + margin, oy + oh / 2 + margin);
        x0 = std::max(0, std::min(x0, x1)); x1 = std::min(w - 1, std::max(x0, x1));
        y0 = std::max(0, std::min(y0, y1)); y1 = std::min(h - 1, std::max(y0, y1));
        if (x0 <= x1 && y0 <= y1) {
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) grid[(size_t)y * w + x] = 0;
        }
    }
    return grid;
}

bool line_of_sight(Px a, Px b, const std::vector<uint8_t>& grid, int w, int h) {
    auto drivable = [&](int px, int py) {
        return px >= 0 && px < w && py >= 0 && py < h && grid[(size_t)py * w + px] == 2;
    };
    int x0 = a.first, y0 = a.second, x1 = b.first, y1 = b.second;
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    while (x != x1 || y != y1) {
        if (!drivable(x, y)) return false;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
    return drivable(x1, y1);
}

std::vector<Px> simplify(const std::vector<Px>& path_px, const std::vector<uint8_t>& grid, int w, int h) {
    if (path_px.size() <= 2) return path_px;
    std::vector<Px> result = {path_px[0]};
    size_t anchor = 0, i = 1;
    while (i < path_px.size() - 1) {
        if (!line_of_sight(path_px[anchor], path_px[i + 1], grid, w, h)) {
            result.push_back(path_px[i]);
            anchor = i;
        }
        ++i;
    }
    result.push_back(path_px.back());
    return result;
}

// 시야가 트이고, 사이의 A* 점이 모두 선분에서 tol_px 안에 있을 때만 건너뛴다.
// 복도 중앙을 따라 온 A* 경로를 벽 쪽으로 질러가지 않게 한다 (LOS 만 보면 팽창 경계를 스치는 지름길이 된다).
double point_segment_dist(Px p, Px a, Px b) {
    double vx = b.first - a.first, vy = b.second - a.second;
    double wx = p.first - a.first, wy = p.second - a.second;
    double len2 = vx * vx + vy * vy;
    double t = len2 > 0 ? std::clamp((wx * vx + wy * vy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(wx - t * vx, wy - t * vy);
}

std::vector<Px> simplify_within(const std::vector<Px>& path_px, const std::vector<uint8_t>& grid, int w, int h,
                                double tol_px) {
    if (path_px.size() <= 2) return path_px;
    std::vector<Px> result = {path_px[0]};
    size_t anchor = 0;
    while (anchor < path_px.size() - 1) {
        size_t best = anchor + 1;
        for (size_t j = anchor + 2; j < path_px.size(); ++j) {
            if (!line_of_sight(path_px[anchor], path_px[j], grid, w, h)) break;
            bool close = true;
            for (size_t k = anchor + 1; k < j && close; ++k)
                close = point_segment_dist(path_px[k], path_px[anchor], path_px[j]) <= tol_px;
            if (!close) break;
            best = j;
        }
        result.push_back(path_px[best]);
        anchor = best;
    }
    return result;
}

// A* 작업 버퍼. 칸 번호(y * w + x)로 바로 찾는 평면 배열이고 스레드마다 하나를 계속 다시 쓴다.
// 큰 맵(수백만 칸)에서 호출마다 초기화하지 않도록 세대 스탬프를 쓴다: mark[i] == gen * 2 면 이번 탐색에서 g 가 있고,
// gen * 2 + 1 이면 닫혔다. calloc 으로 받아서 탐색이 지나가지 않은 영역은 실제 메모리를 쓰지 않는다.
struct AstarScratch {
    double* g = nullptr;
    int32_t* parent = nullptr;
    uint32_t* mark = nullptr;
    size_t capacity = 0;
    uint32_t gen = 0;

    ~AstarScratch() { release(); }
    void release() {
        std::free(g); std::free(parent); std::free(mark);
        g = nullptr; parent = nullptr; mark = nullptr; capacity = 0;
    }
    void begin(size_t cells) {
        if (cells > capacity || gen >= 0x7ffffff0u) {
            const size_t want = std::max(cells, capacity);
            release();
            g = (double*)std::calloc(want, sizeof(double));
            parent = (int32_t*)std::calloc(want, sizeof(int32_t));
            mark = (uint32_t*)std::calloc(want, sizeof(uint32_t));
            if (!g || !parent || !mark) { release(); throw std::bad_alloc(); }
            capacity = want;
            gen = 0;
        }
        ++gen;
    }
};

// cell_factor(px, py) >= 1: 그 칸에 들어가는 이동 비용 배수. nullptr 이면 1 (최단 경로).
// 배수가 1 이상이므로 유클리드 휴리스틱은 그대로 허용(admissible)이다.
// 힙에 넣는 순서와 비용 계산은 예전 해시맵 구현과 같아서 같은 입력이면 같은 경로가 나온다.
template <class Factor>
std::optional<std::vector<Px>> astar(Px start, Px goal, const std::vector<uint8_t>& grid, int w, int h,
                                     const Factor* cell_factor) {
    auto drivable = [&](int px, int py) {
        return px >= 0 && px < w && py >= 0 && py < h && grid[(size_t)py * w + px] == 2;
    };
    auto heuristic = [&](int x, int y) { return std::hypot(x - goal.first, y - goal.second); };
    auto index = [&](int x, int y) { return (int32_t)(y * w + x); };

    thread_local AstarScratch scratch;
    scratch.begin((size_t)w * h);
    double* const g = scratch.g;
    int32_t* const parent = scratch.parent;
    uint32_t* const mark = scratch.mark;
    const uint32_t seen = scratch.gen * 2, closed = seen + 1;

    using Entry = std::pair<double, int32_t>;
    auto cmp = [](const Entry& a, const Entry& b) { return a.first > b.first; };
    std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> open(cmp);
    const int32_t start_i = index(start.first, start.second), goal_i = index(goal.first, goal.second);
    open.push({0.0, start_i});
    g[start_i] = 0.0;
    parent[start_i] = -1;
    mark[start_i] = seen;

    while (!open.empty()) {
        const int32_t current = open.top().second;
        open.pop();
        if (mark[current] == closed) continue;
        mark[current] = closed;

        if (current == goal_i) {
            std::vector<Px> path;
            for (int32_t i = current; i >= 0; i = parent[i]) path.push_back({i % w, i / w});
            std::reverse(path.begin(), path.end());
            return path;
        }
        const int cx = current % w, cy = current / w;
        const double g_current = g[current];
        for (auto& [dx, dy, cost] : kNeighbors) {
            const int nx = cx + dx, ny = cy + dy;
            if (!drivable(nx, ny)) continue;
            const int32_t neighbor = index(nx, ny);
            if (mark[neighbor] == closed) continue;
            if (dx != 0 && dy != 0 && !(drivable(cx + dx, cy) && drivable(cx, cy + dy))) continue;
            const double tentative = g_current + cost * (cell_factor ? (*cell_factor)(nx, ny) : 1.0);
            if (mark[neighbor] != seen || tentative < g[neighbor]) {
                parent[neighbor] = current;
                g[neighbor] = tentative;
                mark[neighbor] = seen;
                open.push({tentative + heuristic(nx, ny), neighbor});
            }
        }
    }
    return std::nullopt;
}

// 정적 팽창 + 동적 장애물 + 허가 없는 교통 구역을 반영한 계획용 grid (2 = 주행 가능)
std::vector<uint8_t> planning_grid(const MapModel& map_model, double robot_width,
                                   const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles,
                                   const std::vector<std::string>& allowed, double traffic_margin) {
    int radius_px = (int)std::ceil((robot_width / 2) / map_model.metadata.resolution);
    std::vector<uint8_t> inflated = get_inflated_grid(map_model, radius_px);
    int w = map_model.metadata.width_px, h = map_model.metadata.height_px;
    if (dynamic_obstacles && !dynamic_obstacles->empty())
        inflated = apply_dynamic_obstacles(map_model, inflated, w, h, *dynamic_obstacles, robot_width);
    if (!map_model.occupancy_zones.empty()) {
        for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
            const auto p = map_model.pixel_to_world(x, y);
            if (map_model.traffic_blocked(p.first, p.second, traffic_margin, allowed))
                inflated[(size_t)y*w+x] = 0;
        }
    }
    return inflated;
}

}  // namespace

std::vector<std::pair<double, double>> plan_path(
    const MapModel& map_model, std::pair<double, double> start, std::pair<double, double> goal,
    double robot_width, double robot_length,
    const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles,
    const std::vector<std::string>* allowed_zone_ids,
    const ClearanceField* clearance, const CenterlineParams* centering) {
    const std::vector<std::string> empty;
    const auto& allowed = allowed_zone_ids ? *allowed_zone_ids : empty;
    const double traffic_margin = std::hypot(robot_width, robot_length) / 2 + 0.1;
    if (map_model.traffic_blocked(start.first, start.second, traffic_margin, allowed)
            || map_model.traffic_blocked(goal.first, goal.second, traffic_margin, allowed))
        throw PlanningError("endpoint intersects an unauthorized occupancy zone");

    if (!map_model.is_free(start.first, start.second))
        throw PlanningError("start point is not drivable");
    if (!map_model.is_free(goal.first, goal.second))
        throw PlanningError("goal point is not drivable");

    auto start_px = map_model.world_to_pixel(start.first, start.second);
    auto goal_px = map_model.world_to_pixel(goal.first, goal.second);
    if (start_px == goal_px) return {goal};

    std::vector<uint8_t> inflated = planning_grid(map_model, robot_width, dynamic_obstacles, allowed, traffic_margin);
    const int w = map_model.metadata.width_px, h = map_model.metadata.height_px;
    auto drivable = [&](int px, int py) {
        return px >= 0 && px < w && py >= 0 && py < h && inflated[(size_t)py * w + px] == 2;
    };
    if (!drivable(start_px.first, start_px.second) || !drivable(goal_px.first, goal_px.second)) {
        inflated[(size_t)start_px.second * w + start_px.first] = 2;
        inflated[(size_t)goal_px.second * w + goal_px.first] = 2;
    }

    const double res = map_model.metadata.resolution;
    const bool centered = clearance && centering && centering->enabled && centering->weight > 0
                          && centering->reach > 0;
    // 벽에서 먼 칸일수록 싸다. 복도에서는 가운데가 가장 싸고, reach 밖(넓은 홀)은 모두 같은 비용이다.
    const double reach_px = centered ? centering->reach / res : 0.0;
    auto factor = [&](int px, int py) {
        double d = std::min(clearance->dist_at(px, py), reach_px);
        double gap = 1.0 - d / reach_px;
        return 1.0 + centering->weight * gap * gap;
    };
    auto path_px = astar(start_px, goal_px, inflated, w, h, centered ? &factor : nullptr);
    if (!path_px)
        throw PlanningError("no path found (blocked, or every corridor is narrower than the robot)");

    auto waypoints_px = centered ? simplify_within(*path_px, inflated, w, h, centering->tolerance / res)
                                 : simplify(*path_px, inflated, w, h);
    std::vector<std::pair<double, double>> waypoints;
    for (auto& p : waypoints_px) waypoints.push_back(map_model.pixel_to_world(p.first, p.second));
    if (!waypoints.empty()) waypoints.back() = goal;
    else waypoints = {goal};
    return waypoints;
}

const std::vector<uint8_t>& static_planning_grid(const MapModel& map_model, double robot_width) {
    int radius_px = (int)std::ceil((robot_width / 2) / map_model.metadata.resolution);
    return get_inflated_grid(map_model, radius_px);
}

std::vector<std::pair<double, double>> plan_path_window(
    const MapModel& map_model, std::pair<double, double> start, std::pair<double, double> goal,
    std::pair<double, double> center, double radius_m, double robot_width, double robot_length,
    const std::vector<std::tuple<double, double, double, double>>& obstacles,
    const std::vector<std::string>* allowed_zone_ids, const ClearanceField* clearance, const CenterlineParams* centering) {
    const std::vector<std::string> empty;
    const auto& allowed = allowed_zone_ids ? *allowed_zone_ids : empty;
    const double traffic_margin = std::hypot(robot_width, robot_length) / 2 + 0.1;
    const double res = map_model.metadata.resolution;
    const int W = map_model.metadata.width_px, H = map_model.metadata.height_px;
    const auto& full = static_planning_grid(map_model, robot_width);

    auto c_px = map_model.world_to_pixel(center.first, center.second);
    const int r_px = (int)std::ceil(radius_m / res);
    const int x0 = std::max(0, c_px.first - r_px), x1 = std::min(W - 1, c_px.first + r_px);
    const int y0 = std::max(0, c_px.second - r_px), y1 = std::min(H - 1, c_px.second + r_px);
    if (x0 > x1 || y0 > y1) throw PlanningError("local window is outside the map");
    const int ww = x1 - x0 + 1, hh = y1 - y0 + 1;
    std::vector<uint8_t> grid((size_t)ww * hh);
    for (int y = 0; y < hh; ++y)
        std::copy(full.begin() + (size_t)(y0 + y) * W + x0, full.begin() + (size_t)(y0 + y) * W + x0 + ww, grid.begin() + (size_t)y * ww);
    // 감지된 장애물 (+ 로봇 반폭) 을 막힘으로
    const double margin = robot_width / 2;
    for (auto& [ox, oy, ow, oh] : obstacles) {
        auto a = map_model.world_to_pixel(ox - ow / 2 - margin, oy - oh / 2 - margin);
        auto b = map_model.world_to_pixel(ox + ow / 2 + margin, oy + oh / 2 + margin);
        int ax = std::max(x0, std::min(a.first, b.first)), bx = std::min(x1, std::max(a.first, b.first));
        int ay = std::max(y0, std::min(a.second, b.second)), by = std::min(y1, std::max(a.second, b.second));
        for (int y = ay; y <= by; ++y) for (int x = ax; x <= bx; ++x) grid[(size_t)(y - y0) * ww + (x - x0)] = 0;
    }
    if (!map_model.occupancy_zones.empty()) {
        for (int y = 0; y < hh; ++y) for (int x = 0; x < ww; ++x) {
            const auto p = map_model.pixel_to_world(x + x0, y + y0);
            if (map_model.traffic_blocked(p.first, p.second, traffic_margin, allowed)) grid[(size_t)y * ww + x] = 0;
        }
    }
    auto s_px = map_model.world_to_pixel(start.first, start.second);
    auto g_px = map_model.world_to_pixel(goal.first, goal.second);
    Px s{s_px.first - x0, s_px.second - y0}, g{g_px.first - x0, g_px.second - y0};
    if (s.first < 0 || s.first >= ww || s.second < 0 || s.second >= hh || g.first < 0 || g.first >= ww || g.second < 0 || g.second >= hh)
        throw PlanningError("start or goal is outside the local window");
    if (s == g) return {goal};
    {   // 이미 장애물 여유 안쪽에 들어와 있어도 빠져나갈 수 있게, 시작점 주변 0.3 m 는 정적으로 비어 있으면 연다
        const int rr = (int)std::ceil(0.3 / res);
        for (int dy = -rr; dy <= rr; ++dy) for (int dx = -rr; dx <= rr; ++dx) {
            int x = s.first + dx, y = s.second + dy;
            if (dx * dx + dy * dy > rr * rr || x < 0 || x >= ww || y < 0 || y >= hh) continue;
            if (full[(size_t)(y + y0) * W + (x + x0)] == 2) grid[(size_t)y * ww + x] = 2;
        }
    }
    grid[(size_t)s.second * ww + s.first] = 2;  // 시작/끝 칸은 항상 통과 가능 (plan_path 와 같은 규칙)
    grid[(size_t)g.second * ww + g.first] = 2;

    const bool centered = clearance && centering && centering->enabled && centering->weight > 0 && centering->reach > 0;
    const double reach_px = centered ? centering->reach / res : 0.0;
    auto factor = [&](int px, int py) {
        double d = std::min(clearance->dist_at(px + x0, py + y0), reach_px);
        double gap = 1.0 - d / reach_px;
        return 1.0 + centering->weight * gap * gap;
    };
    auto path_px = astar(s, g, grid, ww, hh, centered ? &factor : nullptr);
    if (!path_px) throw PlanningError("no detour inside the local window");
    auto wp_px = centered ? simplify_within(*path_px, grid, ww, hh, centering->tolerance / res) : simplify(*path_px, grid, ww, hh);
    std::vector<std::pair<double, double>> out;
    for (auto& p : wp_px) out.push_back(map_model.pixel_to_world(p.first + x0, p.second + y0));
    if (out.empty()) out = {goal}; else out.back() = goal;
    out.front() = start;
    return out;
}

std::vector<std::pair<double, double>> smooth_path(
    const MapModel& map_model, const std::vector<std::pair<double, double>>& path, double robot_width, double robot_length,
    const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles,
    const std::vector<std::string>* allowed_zone_ids, const SmoothParams& prm) {
    if (path.size() < 3) return path;
    const std::vector<std::string> empty;
    const auto& allowed = allowed_zone_ids ? *allowed_zone_ids : empty;
    const double traffic_margin = std::hypot(robot_width, robot_length) / 2 + 0.1;
    // 곡선은 A* 팽창(반폭)보다 5 cm 더 넉넉한 여유를 요구한다 (추종 오차와 몸체 반지름 차이를 흡수)
    const std::vector<uint8_t> grid = planning_grid(map_model, robot_width + 2 * prm.extra_clearance, dynamic_obstacles, allowed, traffic_margin);
    const int w = map_model.metadata.width_px, h = map_model.metadata.height_px;
    auto free_at = [&](double x, double y) {
        auto [px, py] = map_model.world_to_pixel(x, y);
        return px >= 0 && px < w && py >= 0 && py < h && grid[(size_t)py * w + px] == 2;
    };
    return smooth_polyline(path, free_at, map_model.metadata.resolution, prm);
}

std::vector<std::pair<double, double>> smooth_polyline(
    const std::vector<std::pair<double, double>>& path, const std::function<bool(double, double)>& free_at,
    double resolution, const SmoothParams& prm) {
    if (path.size() < 3) return path;
    using P = std::pair<double, double>;
    std::vector<P> out = {path.front()};
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const P prev = path[i - 1], cur = path[i], next = path[i + 1];
        double ix = cur.first - prev.first, iy = cur.second - prev.second;
        double ox = next.first - cur.first, oy = next.second - cur.second;
        double len_in = std::hypot(ix, iy), len_out = std::hypot(ox, oy);
        if (len_in < 1e-6 || len_out < 1e-6) { out.push_back(cur); continue; }
        ix /= len_in; iy /= len_in; ox /= len_out; oy /= len_out;
        double phi = std::acos(std::clamp(ix * ox + iy * oy, -1.0, 1.0));  // 꺾임각
        if (phi < prm.min_turn || phi > prm.max_turn) { out.push_back(cur); continue; }
        // 접선 길이 d = R tan(phi/2). 이웃 코너의 곡선과 겹치지 않게 각 구간의 45% 까지만 쓴다
        double d = std::min({prm.radius * std::tan(phi / 2), 0.45 * len_in, 0.45 * len_out});
        bool done = false;
        for (int attempt = 0; attempt < 4 && d >= 0.05 && !done; ++attempt, d *= 0.5) {
            const P a{cur.first - ix * d, cur.second - iy * d}, b{cur.first + ox * d, cur.second + oy * d};
            const double r_eff = d / std::tan(phi / 2);
            const double hdl = 4.0 / 3.0 * std::tan(phi / 4) * r_eff;  // 원호에 가까운 3차 Bézier 제어점 길이
            const P c1{a.first + ix * hdl, a.second + iy * hdl}, c2{b.first - ox * hdl, b.second - oy * hdl};
            const double arc = r_eff * phi;
            const int n = std::max(4, (int)std::ceil(arc / prm.step));
            const int check = std::max(n, (int)std::ceil(arc / resolution));
            bool ok = true;
            for (int k = 0; k <= check && ok; ++k) {
                double s = (double)k / check, u = 1 - s;
                ok = free_at(u*u*u*a.first + 3*u*u*s*c1.first + 3*u*s*s*c2.first + s*s*s*b.first,
                             u*u*u*a.second + 3*u*u*s*c1.second + 3*u*s*s*c2.second + s*s*s*b.second);
            }
            if (!ok) continue;  // 곡선이 팽창 장애물에 걸린다 -> 크기를 줄여 다시
            for (int k = 0; k <= n; ++k) {
                double s = (double)k / n, u = 1 - s;
                out.push_back({u*u*u*a.first + 3*u*u*s*c1.first + 3*u*s*s*c2.first + s*s*s*b.first,
                               u*u*u*a.second + 3*u*u*s*c1.second + 3*u*s*s*c2.second + s*s*s*b.second});
            }
            done = true;
        }
        if (!done) out.push_back(cur);  // 곡선화 실패: 원래 코너 (정지·회전)
    }
    out.push_back(path.back());
    return out;
}

}  // namespace fms
