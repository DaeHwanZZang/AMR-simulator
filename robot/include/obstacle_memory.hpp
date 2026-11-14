// 장애물 기억. 센서 반경 밖으로 나간 정지 장애물을 일정 시간 기억해서, 갈림길 우회 때 방금 피한 길로 되돌아가는
// "경로 결정장애"(왕복)를 막는다. 엔진/맵을 모르는 순수 로직이다 (위치는 호출자가 넘긴다).
//  - 센서 반경 안에서 안 보이면 "치워졌다" 로 보고 바로 잊는다.
//  - 움직이는 장애물(SIM/target 있음/처음 본 자리에서 이동)은 기억하지 않는다.
//  - 기억 시간은 적응형: 만료된 뒤 같은 자리에서 다시 보이면 2배 (최대 max_ttl).
#pragma once

#include <map>
#include <string>
#include <vector>

#include "dynamic_obstacles.hpp"

namespace fms {

struct MemoryConfig {
    double base_ttl = 30.0;    // s, --obstacle-memory. 0 이하면 기억 끔 (known() == detected)
    double max_ttl = 300.0;    // s, --obstacle-memory-max
    double same_place = 0.3;   // m, 이 안에서 다시 보이면 "같은 자리"
    double moving_eps = 0.05;  // m, 처음 본 자리에서 이만큼 움직였으면 움직이는 장애물
};

class ObstacleMemory {
public:
    MemoryConfig config;

    // detected = 지금 센서에 보이는 것, (x, y) = 로봇 추정 위치. 로그 문구는 log_id 접두와 함께 info 로 남긴다.
    void update(double dt, double x, double y, double sensor_range, const std::vector<DynamicObstacle>& detected,
                const std::string& log_id = "");
    std::vector<DynamicObstacle> known(const std::vector<DynamicObstacle>& detected) const;  // 보이는 것 + 유효한 기억
    size_t remembered_count() const;  // 범위 밖에서 기억 중인 개수
    void clear() { items_.clear(); }

private:
    struct Item {
        DynamicObstacle obs;
        double first_x, first_y;
        double unseen = 0.0;
        double ttl;
        bool visible = true, moving = false, expired = false;
    };
    std::map<std::string, Item> items_;
};

}  // namespace fms
