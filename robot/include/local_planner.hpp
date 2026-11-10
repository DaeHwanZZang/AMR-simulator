// 로컬 계획 (장애물 우회). 구간의 정상 경로(nominal, 장애물 없이 계획한 경로)를 기준으로
//  - detour(): 앞쪽 막힘 구간 뒤의 합류점까지 로봇 중심 반경 창 안에서 우회 경로를 만들고 nominal 꼬리를 이어 붙인다.
//  - rejoin(): 장애물이 사라진 뒤 nominal 위 조금 앞 지점으로 돌아가는 경로를 만든다.
// 장애물은 현재 감지된 위치만 쓴다 (이동 예측 없음).
#pragma once

#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "map_model.hpp"
#include "planner.hpp"

namespace fms {

using LpPoint = std::pair<double, double>;
using LpRect = std::tuple<double, double, double, double>;  // x, y (중심), width, height

struct LocalPlanConfig {
    double radius = 5.0;         // m, 로컬 계획 반경
    double rejoin_margin = 1.0;  // m, 막힘 구간이 끝난 뒤 이만큼 더 간 곳에서 합류한다 (완만하게 돌아오려고)
    double rejoin_ahead = 0.5;   // m, 장애물이 사라진 뒤 nominal 위 이만큼 앞 지점으로 복귀한다
    double plan_margin = 0.15;   // m, 우회 계획 때 장애물을 이만큼 더 키운다 (막힘 판정보다 넉넉히 비켜 가려고)
    double sample_step = 0.1;    // m
};

// nominal(폴리라인)에서 로봇에 가장 가까운 구간 번호 (progress 이후, 앞쪽 일부만 본다). 단조 증가.
size_t lp_advance_progress(const std::vector<LpPoint>& nominal, size_t progress, LpPoint pos);

// pts(첫 점 = 로봇 위치) 가 장애물(+반폭) 사각형과 처음 겹치는 곳까지의 호 길이. 안 겹치면 음수.
double lp_blocked_distance(const std::vector<LpPoint>& pts, const std::vector<LpRect>& obstacles, double robot_width);

class LocalPlanner {
public:
    LocalPlanConfig config;

    std::optional<std::vector<LpPoint>> detour(
        const MapModel& map, LpPoint pos, const std::vector<LpPoint>& nominal, size_t progress,
        const std::vector<LpRect>& obstacles, double robot_width, double robot_length,
        const std::vector<std::string>* allowed, const ClearanceField* clearance, const CenterlineParams* centering,
        const SmoothParams* smooth);

    std::optional<std::vector<LpPoint>> rejoin(
        const MapModel& map, LpPoint pos, const std::vector<LpPoint>& nominal, size_t progress,
        double robot_width, const SmoothParams* smooth);

    // 계획 시간 통계 (ms). 20 회마다 로그로 남긴다.
    double p95_ms() const;
    size_t count() const { return total_; }
    void record(const std::string& robot_id, double ms);

private:
    std::deque<double> recent_;  // 최근 200 개
    size_t total_ = 0;
};

}  // namespace fms
