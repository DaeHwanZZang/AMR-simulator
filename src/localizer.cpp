#include "localizer.hpp"

#include <algorithm>
#include <cmath>

namespace fms {

namespace {
double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

Pose2 Localizer::estimate(const Pose2& o) const {
    double c = std::cos(tth_), s = std::sin(tth_);
    return Pose2{c * o.x - s * o.y + tx_, s * o.x + c * o.y + ty_, normalize(o.theta + tth_)};
}

bool Localizer::update(double dt, const Pose2& odom, const Pose2& truth) {
    if (!enabled()) return false;
    acc_ += dt;
    const double period = 1.0 / p_.rate_hz;
    if (acc_ < period) return false;
    acc_ = std::fmod(acc_, period);

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    if (uni(rng_) < p_.dropout) {
        ++stats_.dropouts;
        return false;
    }
    std::normal_distribution<double> nxy(0.0, p_.sigma_xy), nth(0.0, p_.sigma_theta);
    Pose2 z{truth.x + nxy(rng_), truth.y + nxy(rng_), truth.theta + nth(rng_)};

    Pose2 est = estimate(odom);
    double rx = z.x - est.x, ry = z.y - est.y, rth = normalize(z.theta - est.theta);
    double dx = p_.gain * rx, dy = p_.gain * ry, dth = p_.gain * rth;
    double mag = std::hypot(dx, dy);
    if (mag > p_.max_step_xy) {
        dx *= p_.max_step_xy / mag;
        dy *= p_.max_step_xy / mag;
        mag = p_.max_step_xy;
    }
    dth = std::max(-p_.max_step_theta, std::min(p_.max_step_theta, dth));

    // 새 추정 est' 를 만드는 오프셋 T' 를 다시 푼다:  est' = T' ∘ odom
    Pose2 ne{est.x + dx, est.y + dy, normalize(est.theta + dth)};
    tth_ = normalize(ne.theta - odom.theta);
    double c = std::cos(tth_), s = std::sin(tth_);
    tx_ = ne.x - (c * odom.x - s * odom.y);
    ty_ = ne.y - (s * odom.x + c * odom.y);

    ++stats_.corrections;
    stats_.last_step_xy = mag;
    stats_.last_step_theta = std::abs(dth);
    return true;
}

}  // namespace fms
