// 위치추정 보정 (라이다 위치추정 시뮬레이션).
// 로봇이 아는 자세 = 보정 오프셋 T 를 얹은 오도메트리:  추정 = T ∘ 오도메트리.
// 주기마다 "라이다 측정값" z = 참 자세 + 가우시안 노이즈 를 받아 추정을 z 쪽으로 gain 만큼 당긴다
// (AMCL 이 map->odom 변환을 고치는 구조와 같다). 한 번에 당기는 양은 max_step 으로 묶어서 추정이 튀지 않게 한다.
// 측정은 가끔(dropout) 빠진다. rate_hz 가 0 이면 보정이 없고 추정 = 순수 오도메트리다.
// 이 클래스는 물리 엔진을 모른다. 참 자세는 호출하는 쪽(PhysicsWorld)이 넘긴다.
#pragma once

#include <random>

namespace fms {

struct Pose2 {
    double x = 0, y = 0, theta = 0;
};

struct LocalizerParams {
    double rate_hz = 5.0;         // 보정 주기. 0 이면 보정 없음
    double sigma_xy = 0.02;       // 측정 노이즈 표준편차 (m)
    double sigma_theta = 0.01;    // 측정 노이즈 표준편차 (rad)
    double gain = 0.15;           // 한 번에 측정 쪽으로 당기는 비율 (0~1)
    double dropout = 0.05;        // 측정이 빠질 확률
    double max_step_xy = 0.15;    // 한 번에 고치는 최대 위치 보정 (m)
    double max_step_theta = 0.1;  // 한 번에 고치는 최대 방위 보정 (rad)
    unsigned seed = 1;
};

struct LocalizerStats {
    long corrections = 0;   // 적용한 보정 횟수
    long dropouts = 0;      // 빠진 측정 횟수
    double last_step_xy = 0;      // 마지막 보정이 추정을 옮긴 거리 (m)
    double last_step_theta = 0;   // 마지막 보정이 추정을 돌린 각 (rad)
};

class Localizer {
public:
    Localizer() : rng_(1) {}
    explicit Localizer(const LocalizerParams& p) : p_(p), rng_(p.seed) {}

    bool enabled() const { return p_.rate_hz > 0.0; }
    const LocalizerParams& params() const { return p_; }
    const LocalizerStats& stats() const { return stats_; }

    // 위치 재지정(teleport) 때: 오도메트리가 그 자세로 다시 시작하므로 오프셋을 지운다
    void reset() { tx_ = ty_ = tth_ = 0.0; acc_ = 0.0; }

    Pose2 estimate(const Pose2& odom) const;

    // dt 가 지났다. 주기가 되면 측정(참 자세 + 노이즈)으로 추정을 보정한다. 보정했으면 true.
    bool update(double dt, const Pose2& odom, const Pose2& truth);

private:
    LocalizerParams p_;
    std::mt19937 rng_;
    LocalizerStats stats_;
    double tx_ = 0, ty_ = 0, tth_ = 0;  // 보정 오프셋 T (오도메트리 -> 추정)
    double acc_ = 0.0;
};

}  // namespace fms
