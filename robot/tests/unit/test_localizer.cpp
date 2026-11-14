// 위치추정 보정 (localizer.cpp). 참 자세는 시험이 직접 넘긴다 (엔진 없음).
#include <cmath>

#include "check.hpp"
#include "localizer.hpp"

using namespace fms;

namespace {

LocalizerParams quiet_params() {
    LocalizerParams p;
    p.rate_hz = 5.0;
    p.sigma_xy = 0.0;
    p.sigma_theta = 0.0;
    p.dropout = 0.0;
    return p;
}

}  // namespace

TEST(localizer_estimate_is_offset_odometry) {
    Localizer loc(quiet_params());
    const Pose2 odom{1.0, 2.0, 0.3};
    Pose2 e = loc.estimate(odom);
    CHECK_NEAR(e.x, 1.0, 1e-12);
    CHECK_NEAR(e.theta, 0.3, 1e-12);
    // set_estimate: 지금 오도메트리에서 추정이 그 자세가 된다. 이후 오도메트리 변화는 그대로 따라간다
    loc.set_estimate(odom, Pose2{5.0, -1.0, 1.0});
    e = loc.estimate(odom);
    CHECK_NEAR(e.x, 5.0, 1e-9);
    CHECK_NEAR(e.y, -1.0, 1e-9);
    CHECK_NEAR(e.theta, 1.0, 1e-9);
    loc.reset();
    e = loc.estimate(odom);
    CHECK_NEAR(e.x, 1.0, 1e-12);
    CHECK_NEAR(e.y, 2.0, 1e-12);
}

TEST(localizer_converges_to_truth) {
    Localizer loc(quiet_params());
    loc.set_tracking(true);
    const Pose2 odom{0.0, 0.0, 0.0}, truth{0.2, -0.1, 0.05};
    for (int i = 0; i < 400; ++i) loc.update(0.05, odom, truth);  // 20 s, 100 회 보정
    Pose2 e = loc.estimate(odom);
    CHECK_NEAR(e.x, truth.x, 1e-3);
    CHECK_NEAR(e.y, truth.y, 1e-3);
    CHECK_NEAR(e.theta, truth.theta, 1e-3);
    CHECK(loc.stats().corrections > 90);
    CHECK(loc.stats().dropouts == 0);
}

TEST(localizer_rate_and_tracking_gates) {
    const Pose2 odom{0.0, 0.0, 0.0}, truth{0.2, 0.0, 0.0};
    // 주기 0.2 s: 그 전에는 보정하지 않는다
    Localizer loc(quiet_params());
    loc.set_tracking(true);
    CHECK(!loc.update(0.05, odom, truth));
    CHECK(!loc.update(0.05, odom, truth));
    CHECK(!loc.update(0.05, odom, truth));
    CHECK(loc.update(0.05, odom, truth));
    // 추적이 꺼져 있으면 (UNLOCALIZED) 보정 없음
    Localizer idle(quiet_params());
    for (int i = 0; i < 100; ++i) CHECK(!idle.update(0.05, odom, truth));
    CHECK_NEAR(idle.estimate(odom).x, 0.0, 1e-12);
    // rate 0 = 순수 오도메트리
    LocalizerParams off = quiet_params();
    off.rate_hz = 0.0;
    Localizer pure(off);
    pure.set_tracking(true);
    CHECK(!pure.enabled());
    for (int i = 0; i < 100; ++i) pure.update(0.05, odom, truth);
    CHECK_NEAR(pure.estimate(odom).x, 0.0, 1e-12);
}

TEST(localizer_step_cap_and_rejection) {
    const Pose2 odom{0.0, 0.0, 0.0};
    // gain 1 이어도 한 번에 max_step 까지만 당긴다
    LocalizerParams p = quiet_params();
    p.gain = 1.0;
    Localizer loc(p);
    loc.set_tracking(true);
    CHECK(loc.update(0.2, odom, Pose2{0.4, 0.0, 0.3}));
    Pose2 e = loc.estimate(odom);
    CHECK_NEAR(e.x, p.max_step_xy, 1e-9);
    CHECK_NEAR(e.theta, p.max_step_theta, 1e-9);
    CHECK_NEAR(loc.stats().last_step_xy, p.max_step_xy, 1e-9);
    // 측정이 추정에서 max_residual 보다 멀면 버린다 (틀린 위치는 추적이 몰래 고치지 못한다)
    Localizer far(quiet_params());
    far.set_tracking(true);
    for (int i = 0; i < 20; ++i) CHECK(!far.update(0.2, odom, Pose2{2.0, 0.0, 0.0}));
    CHECK(far.stats().rejected == 20);
    CHECK_NEAR(far.estimate(odom).x, 0.0, 1e-12);
    // 측정 누락 확률 1 이면 전부 dropout
    LocalizerParams d = quiet_params();
    d.dropout = 1.0;
    Localizer drop(d);
    drop.set_tracking(true);
    for (int i = 0; i < 10; ++i) drop.update(0.2, odom, Pose2{0.1, 0.0, 0.0});
    CHECK(drop.stats().dropouts == 10);
    CHECK(drop.stats().corrections == 0);
}
