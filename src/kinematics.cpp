#include "kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace fms {

namespace {

constexpr double kPi = 3.14159265358979323846;
// 이 각도 오차 이내면 "정렬 완료"로 본다. 사다리꼴 각가속도 프로파일이 정확히
// 0으로 수렴하지 못하는 부동소수 오차를 흡수하기 위한 여유.
constexpr double kHeadingTolerance = 0.02;  // rad (~1.1도)

double normalize_angle(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

}  // namespace

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
    // theta 는 더이상 여기서 건드리지 않는다 -- rotate_step 이 사다리꼴 각속도로 관리한다.
    while (dist > 1e-9 && !remaining_.empty()) {
        double tx = remaining_[0].first, ty = remaining_[0].second;
        double dx = tx - x, dy = ty - y;
        double seg_len = std::hypot(dx, dy);
        if (seg_len < 1e-9) { remaining_.erase(remaining_.begin()); continue; }
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

double TrapezoidalKinematics::rotate_step(double target_theta, double dt) {
    double error = normalize_angle(target_theta - theta);
    double abs_error = std::abs(error);
    if (abs_error < 1e-9) { angular_velocity_ = 0.0; return 0.0; }
    double sign = error > 0 ? 1.0 : -1.0;
    double speed = std::abs(angular_velocity_);
    double stop_angle = max_angular_acceleration > 0 ? (speed * speed) / (2 * max_angular_acceleration) : 0.0; //지금 속도에서 최대로 감속했을 때 멈출 때까지 도는 각도
    double accel_mag;
    if (abs_error <= stop_angle) accel_mag = -max_angular_acceleration;
    else if (speed < max_angular_velocity) accel_mag = max_angular_acceleration;
    else accel_mag = 0.0;
    double new_speed = std::max(0.0, std::min(max_angular_velocity, speed + accel_mag * dt));

    double turn = std::min((speed + new_speed) / 2 * dt, abs_error); //이번 틱 회전량

    theta = normalize_angle(theta + sign * turn);
    angular_velocity_ = sign * new_speed;
    return normalize_angle(target_theta - theta);
}

RobotKinematicState TrapezoidalKinematics::hold(double dt) {
    velocity_ = std::max(0.0, velocity_ - max_deceleration * dt);
    acceleration_ = velocity_ > 1e-9 ? -max_deceleration : 0.0;
    double speed = std::max(0.0, std::abs(angular_velocity_) - max_angular_acceleration * dt);
    angular_velocity_ = angular_velocity_ >= 0 ? speed : -speed;
    return get_state();
}

RobotKinematicState TrapezoidalKinematics::step_straight(double dt) {
    // 회전 없이 목표점까지 사다리꼴 속도로 직선 이동. velocity_ 는 크기만 갖는다 (후진도 양수).
    angular_velocity_ = 0.0;
    double dx = straight_->tx - x, dy = straight_->ty - y;
    double dist = std::hypot(dx, dy);
    double vmax = std::min(max_velocity, straight_->max_speed);
    double stop_dist = max_deceleration > 0 ? (velocity_ * velocity_) / (2 * max_deceleration) : 0.0;
    double accel;
    if (dist <= stop_dist) accel = -max_deceleration;
    else if (velocity_ < vmax) accel = max_acceleration;
    else accel = 0.0;
    double new_velocity = std::max(0.0, std::min(vmax, velocity_ + accel * dt));
    // 감속 끝에 속도가 0 이 되어 목표 직전에서 멈추는 일을 막는다 (최소 속도로 마저 들어간다)
    double move = std::max((velocity_ + new_velocity) / 2 * dt, std::min(dist, 0.01 * dt));
    if (move >= dist - 1e-9) {
        x = straight_->tx;
        y = straight_->ty;
        straight_.reset();
        velocity_ = 0.0;
        acceleration_ = 0.0;
        return get_state();
    }
    x += dx / dist * move;
    y += dy / dist * move;
    velocity_ = new_velocity;
    acceleration_ = accel;
    return get_state();
}

RobotKinematicState TrapezoidalKinematics::step(double dt) {
    if (straight_) return step_straight(dt);
    double remaining_dist = remaining_length();
    bool has_target = false;
    double heading_error = 0.0;


    //경로가 남았을 때 목적지까지의 방향으로 회전
    if (remaining_dist > 1e-9) {
        double dx = remaining_[0].first - x, dy = remaining_[0].second - y;
        heading_error = rotate_step(std::atan2(dy, dx), dt);
        has_target = true;
    //경로가 남지 않았을 때 최종각도까지 회전
    } else if (final_theta_) {
        heading_error = rotate_step(*final_theta_, dt);
        has_target = true;
    } else {
        angular_velocity_ = 0.0;
    }

    // 목표 방향으로 정렬되기 전까지는 직진하지 않는다 (제자리 회전 후 전진).
    bool facing = !has_target || std::abs(heading_error) < kHeadingTolerance;

    double accel;
    if (!facing || remaining_dist < 1e-9) {
        accel = velocity_ > 1e-9 ? -max_deceleration : 0.0;
    } else {
        double stop_dist = max_deceleration > 0 ? (velocity_ * velocity_) / (2 * max_deceleration) : 0.0;
        if (remaining_dist <= stop_dist) accel = -max_deceleration;
        else if (velocity_ < max_velocity) accel = max_acceleration;
        else accel = 0.0;
    }
    double new_velocity = std::max(0.0, std::min(max_velocity, velocity_ + accel * dt));
    double move_dist = facing ? std::min((velocity_ + new_velocity) / 2 * dt, remaining_dist) : 0.0;
    advance(move_dist);

    velocity_ = remaining_length() > 1e-9 ? new_velocity : 0.0;
    acceleration_ = accel;
    return get_state();
}

bool TrapezoidalKinematics::is_path_complete() const {
    if (straight_) return false;
    if (remaining_length() >= 1e-9) return false;
    if (final_theta_ && std::abs(normalize_angle(*final_theta_ - theta)) >= kHeadingTolerance) return false;
    return true;
}

RobotKinematicState TrapezoidalKinematics::get_state() const {
    // 직선 이동 중에는 목표점 하나를 local_path 로 보고한다 (뷰어 표시, 동적 장애물 막힘 판정용)
    std::vector<std::pair<double, double>> path = remaining_;
    if (straight_) path = {{straight_->tx, straight_->ty}};
    return RobotKinematicState{x, y, theta, velocity_, acceleration_, angular_velocity_, std::move(path),
                                is_path_complete()};
}

}  // namespace fms
