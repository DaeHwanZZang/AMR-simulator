// 동적 장애물 필드, 몸체 겹침 판정, 장애물 기억 (dynamic_obstacles / obstacle_memory).
#include <algorithm>
#include <string>
#include <vector>

#include "check.hpp"
#include "dynamic_obstacles.hpp"
#include "obstacle_memory.hpp"

using namespace fms;

namespace {

DynamicObstacle box(const std::string& id, double x, double y, double w = 0.4, double h = 0.4,
                    ObstacleSource src = ObstacleSource::MANUAL) {
    DynamicObstacle o;
    o.obstacle_id = id;
    o.x = x; o.y = y; o.width = w; o.height = h;
    o.source = src;
    return o;
}

bool has_id(const std::vector<DynamicObstacle>& v, const std::string& id) {
    return std::any_of(v.begin(), v.end(), [&](const DynamicObstacle& o) { return o.obstacle_id == id; });
}

}  // namespace

TEST(obstacle_distance_and_overlap) {
    DynamicObstacle o = box("a", 2.0, 2.0, 1.0, 0.4);
    CHECK_NEAR(o.distance_to(2.0, 2.0), 0.0, 1e-12);   // 안쪽은 0
    CHECK_NEAR(o.distance_to(3.0, 2.0), 0.5, 1e-12);   // 오른쪽 변에서 0.5
    CHECK_NEAR(o.distance_to(2.0, 3.2), 1.0, 1e-12);   // 아랫변에서 1.0
    CHECK_NEAR(o.distance_to(2.8, 2.6), 0.5, 1e-12);   // 모서리 (0.3, 0.4) -> 0.5
    // 사각형 몸체 vs 로봇 원판(반지름 0.21)
    CHECK(o.overlaps_disc(2.6, 2.0, 0.21));    // 변까지 0.1
    CHECK(!o.overlaps_disc(2.75, 2.0, 0.21));  // 변까지 0.25
    CHECK(!o.overlaps_disc(2.5 + 0.2, 2.2 + 0.2, 0.21));  // 모서리까지 0.283
    // 원판 몸체 (다른 로봇): 사각형 치수는 감지용이고 충돌은 반지름으로 본다
    DynamicObstacle peer = box("robot:R2", 5.0, 5.0, 0.9, 0.9, ObstacleSource::ROBOT);
    peer.body_radius = 0.21;
    CHECK(peer.overlaps_disc(5.4, 5.0, 0.21));    // 중심 거리 0.40 < 0.42
    CHECK(!peer.overlaps_disc(5.43, 5.0, 0.21));  // 0.43
    CHECK(!peer.overlaps_disc(5.35, 5.35, 0.21)); // 사각형(±0.45) 안이지만 원판끼리는 0.495 떨어져 있다
}

TEST(obstacle_field_detects_in_range) {
    DynamicObstacleField f(1);
    f.add_manual("near", 2.0, 0.0, 0.4, 0.4);
    f.add_manual("far", 10.0, 0.0, 0.4, 0.4);
    auto seen = f.detected(0.0, 0.0, 3.0);
    CHECK(seen.size() == 1 && seen[0].obstacle_id == "near");
    CHECK(f.detected(0.0, 0.0, 9.8).size() == 2);  // 가장자리까지 거리 9.8
    f.clear(std::string("near"));
    CHECK(f.detected(0.0, 0.0, 3.0).empty());
    f.clear();
    CHECK(f.obstacles.empty());
}

TEST(obstacle_field_sync_external) {
    DynamicObstacleField f(1);
    f.add_manual("mine", 1.0, 1.0, 0.4, 0.4);
    f.sync_external("world:", {box("world:a", 2, 2), box("world:b", 3, 3)});
    f.sync_external("robot:", {box("robot:R2", 4, 4, 0.9, 0.9, ObstacleSource::ROBOT)});
    CHECK(f.obstacles.size() == 4);
    // 갱신 + 없어진 것 삭제. 다른 접두와 수동 장애물은 건드리지 않는다
    f.sync_external("world:", {box("world:a", 2.5, 2)});
    CHECK(f.obstacles.size() == 3);
    CHECK(f.obstacles.count("world:b") == 0);
    CHECK_NEAR(f.obstacles.at("world:a").x, 2.5, 1e-12);
    CHECK(f.obstacles.count("mine") == 1 && f.obstacles.count("robot:R2") == 1);
    CHECK(f.obstacles.at("robot:R2").source == ObstacleSource::ROBOT);
    f.sync_external("robot:", {});
    CHECK(f.obstacles.count("robot:R2") == 0);
    // 외부 장애물은 tick 으로 움직이지 않는다
    f.tick(1.0);
    CHECK_NEAR(f.obstacles.at("world:a").x, 2.5, 1e-12);
}

TEST(obstacle_zone_spawns_crossing_obstacles) {
    DynamicObstacleField f(3);
    f.configure_zone(Zone{0, 0, 10, 10}, std::make_pair(ZoneSide::LEFT, ZoneSide::RIGHT), 5.0, 2, 0.4, 0.4, 1.0);
    f.tick(0.05);  // 설정 직후 첫 tick 에 하나 생긴다
    CHECK(f.obstacles.size() == 1);
    const std::string id = f.obstacles.begin()->first;
    CHECK(f.obstacles.at(id).source == ObstacleSource::SIM);
    CHECK_NEAR(f.obstacles.at(id).x, 0.0, 1e-9);  // LEFT 변에서 출발
    f.tick(1.0);
    CHECK(f.obstacles.at(id).x > 0.5);            // RIGHT 로 간다
    for (int i = 0; i < 400; ++i) f.tick(0.05);   // 20 s: 첫 장애물은 도착해서 사라졌다
    CHECK(f.obstacles.count(id) == 0);
    int sim = 0;
    for (auto& [oid, o] : f.obstacles) sim += o.source == ObstacleSource::SIM;
    CHECK(sim <= 2);                               // 동시에 최대 max_sim_count
    f.stop_zone();
    for (int i = 0; i < 400; ++i) f.tick(0.05);
    CHECK(f.obstacles.empty());
}

TEST(memory_remembers_static_obstacle_out_of_range) {
    ObstacleMemory mem;
    mem.config.base_ttl = 10.0;
    const double range = 3.0;
    auto o = box("a", 5.0, 0.0);
    mem.update(0.1, 3.0, 0.0, range, {o});          // 보인다 (가장자리까지 1.8 m)
    CHECK(mem.remembered_count() == 0);
    CHECK(mem.known({o}).size() == 1);              // 보이는 것은 중복 없이 한 번
    // 로봇이 멀어져 범위 밖: 기억으로 남는다
    mem.update(0.1, -5.0, 0.0, range, {});
    CHECK(mem.remembered_count() == 1);
    CHECK(has_id(mem.known({}), "a"));
    for (int i = 0; i < 95; ++i) mem.update(0.1, -5.0, 0.0, range, {});   // 9.6 s
    CHECK(has_id(mem.known({}), "a"));
    for (int i = 0; i < 10; ++i) mem.update(0.1, -5.0, 0.0, range, {});   // 10.6 s > ttl
    CHECK(!has_id(mem.known({}), "a"));
    CHECK(mem.remembered_count() == 0);
}

TEST(memory_forgets_when_checked_in_range_or_moving) {
    ObstacleMemory mem;
    mem.config.base_ttl = 30.0;
    const double range = 3.0;
    // 범위 안인데 안 보인다 = 치워졌다 -> 바로 잊는다
    mem.update(0.1, 3.0, 0.0, range, {box("a", 5.0, 0.0)});
    mem.update(0.1, 3.0, 0.0, range, {});
    CHECK(mem.known({}).empty());
    // SIM(가로지르는) 장애물은 기억하지 않는다
    mem.update(0.1, 3.0, 0.0, range, {box("s", 5.0, 0.0, 0.4, 0.4, ObstacleSource::SIM)});
    mem.update(0.1, -5.0, 0.0, range, {});
    CHECK(mem.known({}).empty());
    // 보는 동안 5 cm 넘게 움직인 것도 기억하지 않는다
    mem.update(0.1, 3.0, 0.0, range, {box("m", 5.0, 0.0)});
    mem.update(0.1, 3.0, 0.0, range, {box("m", 5.2, 0.0)});
    mem.update(0.1, -5.0, 0.0, range, {});
    CHECK(mem.known({}).empty());
    // 기억 끔
    ObstacleMemory off;
    off.config.base_ttl = 0.0;
    off.update(0.1, 3.0, 0.0, range, {box("a", 5.0, 0.0)});
    off.update(0.1, -5.0, 0.0, range, {});
    CHECK(off.known({}).empty());
}

TEST(memory_ttl_doubles_when_seen_again_in_place) {
    ObstacleMemory mem;
    mem.config.base_ttl = 10.0;
    mem.config.max_ttl = 15.0;
    const double range = 3.0;
    auto o = box("a", 5.0, 0.0);
    auto away = [&](double seconds) {
        for (int i = 0; i < (int)(seconds * 10 + 0.5); ++i) mem.update(0.1, -5.0, 0.0, range, {});
    };
    mem.update(0.1, 3.0, 0.0, range, {o});
    away(10.5);                                   // 만료
    CHECK(!has_id(mem.known({}), "a"));
    mem.update(0.1, 3.0, 0.0, range, {o});        // 같은 자리에서 다시 보임 -> ttl 2배 (상한 15)
    away(12.0);
    CHECK(has_id(mem.known({}), "a"));            // 10 s 넘었지만 아직 기억
    away(4.0);                                    // 16 s > 15
    CHECK(!has_id(mem.known({}), "a"));
    mem.clear();
    CHECK(mem.known({}).empty());
}
