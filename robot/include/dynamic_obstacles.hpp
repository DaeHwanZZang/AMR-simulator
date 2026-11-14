// 동적 장애물 시뮬레이션 (robot_client/sim/dynamic_obstacles.py 포팅)
// 로봇마다 완전히 독립. FMS/뷰어에서 절대 쓰지 않는다.
#pragma once

#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace fms {

// MANUAL = 사람이 놓은 것 (INJECT_OBSTACLE, 월드 장애물), SIM = 구역에서 자동 생성되어 가로지르는 것, ROBOT = 다른 로봇
enum class ObstacleSource { MANUAL, SIM, ROBOT };
const char* to_string(ObstacleSource s);

// 구역의 네 면. world 가 y-down 이라 TOP=y_min 쪽, BOTTOM=y_max 쪽.
enum class ZoneSide { TOP = 1, RIGHT = 2, BOTTOM = 3, LEFT = 4 };
ZoneSide zone_side_from_char(char c);  // '1'~'4', 아니면 throw

struct DynamicObstacle {
    std::string obstacle_id;
    double x, y, width, height;
    ObstacleSource source;
    std::optional<std::pair<double, double>> target;
    double speed = 0.5;
    // 몸체 모양. 0 이면 width x height 사각형, 0 보다 크면 그 반지름의 원판이다 (다른 로봇: 물리 충돌 형상과 같다).
    // 감지/계획은 항상 사각형(x, y, width, height)을 쓰고, 이 값은 충돌 판정(overlaps_disc)에만 쓴다.
    double body_radius = 0.0;

    double distance_to(double px, double py) const;
    // 중심 (cx, cy), 반지름 r 인 원판(로봇 몸체)이 이 장애물 몸체와 겹치는가
    bool overlaps_disc(double cx, double cy, double r) const;
    bool step(double dt);  // target 도착 시 true (SIM 전용)
};

struct Zone { double x_min, y_min, x_max, y_max; };

class DynamicObstacleField {
public:
    std::map<std::string, DynamicObstacle> obstacles;

    explicit DynamicObstacleField(unsigned seed = std::random_device{}()) : rng_(seed) {}

    void tick(double dt);

    void configure_zone(Zone zone, std::optional<std::pair<ZoneSide, ZoneSide>> sides,
                         double spawn_interval = 5.0, int max_sim_count = 3,
                         double obstacle_width = 0.4, double obstacle_height = 0.4,
                         double obstacle_speed = 0.5);
    void stop_zone() { has_zone_ = false; }

    void add_manual(const std::string& id, double x, double y, double width, double height);
    void clear(const std::optional<std::string>& id = std::nullopt);
    // 밖에서 관리하는 장애물(월드 장애물, 다른 로봇)을 맞춘다: id 가 prefix 로 시작하는 것 중 items 에 없는 것은 지우고,
    // items 는 넣거나 갱신한다. items 의 obstacle_id 는 prefix 로 시작해야 한다.
    void sync_external(const std::string& prefix, const std::vector<DynamicObstacle>& items);

    std::vector<DynamicObstacle> detected(double x, double y, double sensor_range) const;

private:
    std::mt19937 rng_;
    bool has_zone_ = false;
    Zone zone_{};
    std::optional<std::pair<ZoneSide, ZoneSide>> sides_;
    double spawn_interval_ = 5.0;
    int max_sim_count_ = 3;
    double obstacle_width_ = 0.4, obstacle_height_ = 0.4, obstacle_speed_ = 0.5;
    double since_spawn_ = 0.0;
    long long seq_ = 0;

    std::pair<double, double> edge_point(ZoneSide side, double along) const;
    void spawn_crossing();
};

}  // namespace fms
