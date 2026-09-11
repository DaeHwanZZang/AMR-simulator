#include "kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace fms {

double TrapezoidalKinematics::remaining_length() const {
    if (remaining_.empty()) return 0.0;
    double total = std::hypot(remaining_[0].first - x, remaining_[0].second - y);
    for (size_t i = 0; i + 1 < remaining_.size(); ++i) {
        total += std::hypot(remaining_[i + 1].first - remaining_[i].first,
                             remaining_[i + 1].second - remaining_[i].second);
    }
    return total;
}

void TrapezoidalKinematics::advance(double dist) {
    while (dist > 1e-9 && !remaining_.empty()) {
        double tx = remaining_[0].first, ty = remaining_[0].second;
        double dx = tx - x, dy = ty - y;
        double seg_len = std::hypot(dx, dy);
        if (seg_len < 1e-9) { remaining_.erase(remaining_.begin()); continue; }
        theta = std::atan2(dy, dx);
        if (dist >= seg_len) {
            x = tx; y = ty;
            dist -= seg_len;
            remaining_.erase(remaining_.begin());
        } else {
            double frac = dist / seg_len;
            x += dx * frac;
            y += dy * frac;
            dist = 0.0;
        }
    }
}

RobotKinematicState TrapezoidalKinematics::hold(double dt) {
    velocity_ = std::max(0.0, velocity_ - max_deceleration * dt);
    acceleration_ = velocity_ > 1e-9 ? -max_deceleration : 0.0;
    return get_state();
}

RobotKinematicState TrapezoidalKinematics::step(double dt) {
    double remaining_dist = remaining_length();
    double accel;
    if (remaining_dist < 1e-9) {
        accel = velocity_ > 1e-9 ? -max_deceleration : 0.0;
    } else {
        double stop_dist = max_deceleration > 0 ? (velocity_ * velocity_) / (2 * max_deceleration) : 0.0;
        if (remaining_dist <= stop_dist) accel = -max_deceleration;
        else if (velocity_ < max_velocity) accel = max_acceleration;
        else accel = 0.0;
    }
    double new_velocity = std::max(0.0, std::min(max_velocity, velocity_ + accel * dt));
    double move_dist = std::min((velocity_ + new_velocity) / 2 * dt, remaining_dist);
    advance(move_dist);

    velocity_ = remaining_length() > 1e-9 ? new_velocity : 0.0;
    acceleration_ = accel;
    return get_state();
}

RobotKinematicState TrapezoidalKinematics::get_state() const {
    return RobotKinematicState{x, y, theta, velocity_, acceleration_, remaining_, is_path_complete()};
}

}  // namespace fms
