// 맵 모델, 팽창, A*, 곡선화, 로컬 창 계획, 장애물 우회 (map_model / planner / local_planner).
// 맵: maps/gen_loop (레이아웃 칸 (r,c) 중심 = world (c+1, r+1)), maps/empty_hall1 (회귀 테스트 맵).
#include <cmath>
#include <random>
#include <vector>

#include "check.hpp"
#include "local_planner.hpp"
#include "map_model.hpp"
#include "path_refine.hpp"
#include "planner.hpp"

using namespace fms;

namespace {

using Pt = std::pair<double, double>;

const MapModel& loop_map() {
    static MapModel m = MapModel::load("maps/gen_loop/fms_map.json");
    return m;
}
const MapModel& hall_map() {
    static MapModel m = MapModel::load("maps/empty_hall1/fms_map.json");
    return m;
}

// 예전 구현과 같은 규칙의 무차별 대입 팽창 (inflate_grid 의 기준)
std::vector<uint8_t> inflate_brute(const std::vector<uint8_t>& grid, int w, int h, int r) {
    std::vector<uint8_t> out = grid;
    if (r <= 0) return out;  // 반지름 0 이면 그대로 (UNKNOWN 칸 값도 유지)
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            bool hit = false;
            for (int dy = -r; dy <= r && !hit; ++dy)
                for (int dx = -r; dx <= r && !hit; ++dx) {
                    if (dx * dx + dy * dy > r * r) continue;
                    const int sx = x + dx, sy = y + dy;
                    if (sx < 0 || sx >= w || sy < 0 || sy >= h) continue;  // 맵 밖은 막힘이 아니다
                    hit = grid[(size_t)sy * w + sx] != 2;
                }
            if (hit) out[(size_t)y * w + x] = 0;
        }
    return out;
}

// 폴리라인(첫 점 포함)을 step 간격으로 훑어 전부 pred 를 만족하는지
template <class Pred>
bool all_along(const std::vector<Pt>& pts, double step, Pred pred) {
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double dx = pts[i + 1].first - pts[i].first, dy = pts[i + 1].second - pts[i].second;
        const int n = std::max(1, (int)std::ceil(std::hypot(dx, dy) / step));
        for (int k = 0; k <= n; ++k)
            if (!pred(pts[i].first + dx * k / n, pts[i].second + dy * k / n)) return false;
    }
    return true;
}

bool in_rect(double x, double y, double cx, double cy, double w, double h) {
    return std::abs(x - cx) < w / 2 && std::abs(y - cy) < h / 2;
}

std::vector<Pt> with_start(Pt start, const std::vector<Pt>& path) {
    std::vector<Pt> out = {start};
    out.insert(out.end(), path.begin(), path.end());
    return out;
}

}  // namespace

TEST(map_coordinates_and_cells) {
    const MapModel& m = loop_map();
    CHECK(m.metadata.width_px > 0 && m.metadata.height_px > 0);
    // origin = 왼쪽 위 픽셀의 world 좌표, y 는 아래로 증가
    auto p0 = m.world_to_pixel(m.metadata.origin_x, m.metadata.origin_y);
    CHECK(p0.first == 0 && p0.second == 0);
    auto p1 = m.world_to_pixel(m.metadata.origin_x + 2.5 * m.metadata.resolution, m.metadata.origin_y + 7.5 * m.metadata.resolution);
    CHECK(p1.first == 2 && p1.second == 7);
    // 픽셀 -> world -> 픽셀 왕복
    for (auto px : {std::pair<int, int>{0, 0}, {17, 5}, {m.metadata.width_px - 1, m.metadata.height_px - 1}}) {
        auto w = m.pixel_to_world(px.first, px.second);
        CHECK(m.world_to_pixel(w.first, w.second) == px);
    }
    // 통로 칸 중심은 FREE, world (0,0) 은 여백(주행 불가), 맵 밖은 OCCUPIED
    CHECK(m.is_free(1.0, 1.0));
    CHECK(m.cell_at(1.0, 1.0) == CellType::FREE);
    CHECK(!m.is_free(0.0, 0.0));
    CHECK(!m.in_bounds(-500.0, -500.0));
    CHECK(m.cell_at(-500.0, -500.0) == CellType::OCCUPIED);
    CHECK(!m.is_free(-500.0, -500.0));
    // 감속 구역이 없는 곳은 제한 없음
    CHECK(std::isinf(m.speed_limit_at(1.0, 1.0)));
}

TEST(inflate_matches_brute_force) {
    std::mt19937 rng(7);
    for (int trial = 0; trial < 6; ++trial) {
        const int w = 37 + trial * 5, h = 23 + trial * 3;
        std::vector<uint8_t> grid((size_t)w * h, 2);
        std::uniform_int_distribution<int> px(0, w - 1), py(0, h - 1), kind(0, 1);
        for (int i = 0; i < w * h / 25; ++i) grid[(size_t)py(rng) * w + px(rng)] = (uint8_t)kind(rng);  // 0=OCCUPIED, 1=UNKNOWN
        for (int r : {0, 1, 2, 3, 5, 8}) CHECK(inflate_grid(grid, w, h, r) == inflate_brute(grid, w, h, r));
    }
    // 실제 맵 조각에서도 같다 (로봇 반폭 8 px)
    const MapModel& m = loop_map();
    const auto& full = m.classify_grid();
    const int W = m.metadata.width_px, w = 200, h = 160, x0 = 300, y0 = 300;
    std::vector<uint8_t> crop((size_t)w * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) crop[(size_t)y * w + x] = full[(size_t)(y + y0) * W + (x + x0)];
    CHECK(inflate_grid(crop, w, h, 8) == inflate_brute(crop, w, h, 8));
}

TEST(plan_path_reaches_goal_on_free_cells) {
    const MapModel& m = loop_map();
    const Pt start{1.0, 1.0}, goal{19.0, 12.0};
    ClearanceField field(m);
    for (bool centered : {true, false}) {
        CenterlineParams c;
        c.enabled = centered;
        auto path = plan_path(m, start, goal, 0.5, 0.7, nullptr, nullptr, &field, &c);
        CHECK(!path.empty());
        CHECK_NEAR(path.back().first, goal.first, 1e-9);
        CHECK_NEAR(path.back().second, goal.second, 1e-9);
        CHECK(all_along(with_start(start, path), 0.02, [&](double x, double y) { return m.is_free(x, y); }));
        // 곡선화: 끝점은 그대로, 샘플도 전부 FREE
        auto curved = smooth_path(m, path, 0.5, 0.7, nullptr, nullptr, SmoothParams());
        CHECK(curved.size() >= path.size());
        CHECK_NEAR(curved.back().first, goal.first, 1e-9);
        CHECK_NEAR(curved.back().second, goal.second, 1e-9);
        CHECK(all_along(with_start(start, curved), 0.02, [&](double x, double y) { return m.is_free(x, y); }));
    }
    // 중앙 주행 경로는 최단 경로보다 벽에서 멀다 (가장 좁은 곳 기준)
    CenterlineParams on, off;
    off.enabled = false;
    auto centered = with_start(start, plan_path(m, start, goal, 0.5, 0.7, nullptr, nullptr, &field, &on));
    auto shortest = with_start(start, plan_path(m, start, goal, 0.5, 0.7, nullptr, nullptr, &field, &off));
    auto min_clearance = [&](const std::vector<Pt>& pts) {
        double best = 1e9;
        all_along(pts, 0.05, [&](double x, double y) { best = std::min(best, field.dist_px(x, y)); return true; });
        return best;
    };
    CHECK(min_clearance(centered) >= min_clearance(shortest));
}

TEST(plan_path_errors) {
    const MapModel& m = loop_map();
    CHECK_THROWS(plan_path(m, {0.0, 0.0}, {1.0, 1.0}), PlanningError);   // 시작점이 여백(UNKNOWN)
    CHECK_THROWS(plan_path(m, {1.0, 1.0}, {0.0, 0.0}), PlanningError);   // 목표점이 여백
    CHECK_THROWS(plan_path(m, {1.0, 1.0}, {3.5, 5.0}), PlanningError);   // 목표점이 벽 안 (레이아웃 '#')
    // 로봇이 통로(1 m)보다 넓으면 길이 없다
    CHECK_THROWS(plan_path(m, {5.0, 5.0}, {5.0, 8.0}, 1.6, 1.6), PlanningError);
    // 같은 픽셀이면 목표만 돌려준다
    auto same = plan_path(m, {1.0, 1.0}, {1.0, 1.0});
    CHECK(same.size() == 1);
}

TEST(plan_path_avoids_dynamic_obstacle) {
    const MapModel& m = hall_map();
    const Pt start{3.0, 3.0}, goal{9.0, 3.0};
    const std::vector<std::tuple<double, double, double, double>> obs = {{6.0, 3.0, 0.6, 0.6}};
    auto direct = plan_path(m, start, goal);
    auto around = plan_path(m, start, goal, 0.5, 0.7, &obs);
    CHECK(!all_along(with_start(start, direct), 0.02, [&](double x, double y) { return !in_rect(x, y, 6.0, 3.0, 0.6, 0.6); }));
    // 장애물 + 로봇 반폭(0.25 m, 픽셀 반올림 여유 한 칸) 밖으로만 지난다
    const double grown = 0.6 + 0.5 - 2 * m.metadata.resolution;
    CHECK(all_along(with_start(start, around), 0.02, [&](double x, double y) { return !in_rect(x, y, 6.0, 3.0, grown, grown); }));
    CHECK_NEAR(around.back().first, goal.first, 1e-9);
}

TEST(plan_path_window_stays_local) {
    const MapModel& m = hall_map();
    const Pt start{3.0, 3.0}, goal{6.0, 3.0};
    const std::vector<std::tuple<double, double, double, double>> obs = {{4.5, 3.0, 0.5, 0.5}};
    auto path = plan_path_window(m, start, goal, start, 5.0, 0.5, 0.7, obs);
    CHECK(!path.empty());
    CHECK_NEAR(path.back().first, goal.first, 1e-9);
    CHECK_NEAR(path.back().second, goal.second, 1e-9);
    const double grown = 0.5 + 0.5 - 2 * m.metadata.resolution;
    CHECK(all_along(path, 0.02, [&](double x, double y) { return !in_rect(x, y, 4.5, 3.0, grown, grown); }));
    CHECK(all_along(path, 0.05, [&](double x, double y) { return std::hypot(x - 3.0, y - 3.0) <= 5.0 * std::sqrt(2.0) + 0.1; }));
    // 목표가 창 밖이면 실패
    CHECK_THROWS(plan_path_window(m, start, {20.0, 3.0}, start, 5.0, 0.5, 0.7, obs), PlanningError);
}

TEST(local_planner_detours_and_rejoins) {
    const MapModel& m = hall_map();
    const std::vector<LpPoint> nominal = {{3.0, 3.0}, {9.0, 3.0}};
    const std::vector<LpRect> rects = {{6.0, 3.0, 0.5, 0.5}};
    const LpPoint pos{3.0, 3.0};
    // 정상 경로는 막혀 있다: 장애물 앞면(5.75) - 반폭 근처에서 처음 겹친다
    const double blocked = lp_blocked_distance(nominal, rects, 0.5);
    CHECK(blocked > 2.0 && blocked < 3.0);
    CHECK(lp_blocked_distance(nominal, {}, 0.5) < 0);

    LocalPlanner lp;
    SmoothParams smooth;
    for (const SmoothParams* sp : {(const SmoothParams*)nullptr, (const SmoothParams*)&smooth}) {
        auto path = lp.detour(m, pos, nominal, 0, rects, 0.5, 0.7, nullptr, nullptr, nullptr, sp);
        CHECK(path.has_value());
        if (!path) continue;
        CHECK_NEAR(path->back().first, 9.0, 1e-6);
        CHECK_NEAR(path->back().second, 3.0, 1e-6);
        CHECK(lp_blocked_distance(with_start(pos, *path), rects, 0.5) < 0);  // 우회 경로는 안 막힌다
        CHECK(all_along(with_start(pos, *path), 0.02, [&](double x, double y) { return m.is_free(x, y); }));
    }
    // 창(반경 1 m) 안에 합류할 곳이 없으면 우회 실패
    LocalPlanner tiny;
    tiny.config.radius = 0.6;
    const std::vector<LpRect> wall = {{3.6, 3.0, 0.4, 3.0}};
    CHECK(!tiny.detour(m, pos, nominal, 0, wall, 0.5, 0.7, nullptr, nullptr, nullptr, nullptr).has_value());

    // 진행 번호는 줄어들지 않는다
    const std::vector<LpPoint> bent = {{3.0, 3.0}, {6.0, 3.0}, {6.0, 6.0}, {9.0, 6.0}};
    size_t prog = lp_advance_progress(bent, 0, {4.0, 3.0});
    CHECK(prog == 0);
    prog = lp_advance_progress(bent, prog, {6.0, 4.5});
    CHECK(prog == 1);
    CHECK(lp_advance_progress(bent, prog, {4.0, 3.0}) >= 1);

    // 정상 경로에서 벗어난 자리에서 복귀 경로: 정상 경로 끝으로 이어진다
    auto back = lp.rejoin(m, {4.0, 3.8}, nominal, 0, 0.5, nullptr);
    CHECK(back.has_value());
    if (back) {
        CHECK_NEAR(back->back().first, 9.0, 1e-6);
        CHECK_NEAR(back->back().second, 3.0, 1e-6);
    }
}
