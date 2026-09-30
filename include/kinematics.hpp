// 로봇 물리 시뮬레이션 (robot_client/sim/kinematics.py 포팅)
// 등가속/등감속 사다리꼴 속도 프로파일.
#pragma once

#include <optional>
#include <utility>
#include <vector>

namespace fms {

struct RobotKinematicState {
    double x, y, theta;
    double velocity, acceleration;
    double angular_velocity;
    std::vector<std::pair<double, double>> local_path;
    bool path_complete;
};

class TrapezoidalKinematics {
public:
    double x, y, theta;
    double max_velocity, max_acceleration, max_deceleration;
    double max_angular_velocity, max_angular_acceleration;

    TrapezoidalKinematics(double x_, double y_, double theta_ = 0.0, double max_v = 1.0,
                           double max_a = 0.5, double max_d = 0.5, double max_angular_v = 1.57,
                           double max_angular_a = 3.14)
        : x(x_), y(y_), theta(theta_), max_velocity(max_v), max_acceleration(max_a),
          max_deceleration(max_d), max_angular_velocity(max_angular_v), max_angular_acceleration(max_angular_a) {}

    // 경로가 바뀌면 최종 정렬 목표(final_theta_)는 해제한다. 필요하면 set_final_theta 로 다시 지정.
    void set_path(std::vector<std::pair<double, double>> waypoints) {
        remaining_ = std::move(waypoints);
        final_theta_.reset();
    }
    // 경로 마지막 지점 도착 후 이 각도로 정렬해야 is_path_complete() 가 true 가 된다.
    void set_final_theta(double target_theta) { final_theta_ = target_theta; }
    void set_pose(double nx, double ny, const double* ntheta = nullptr) {
        x = nx; y = ny;
        if (ntheta) theta = *ntheta;
    }
    void emergency_stop() {
        velocity_ = 0.0; acceleration_ = 0.0; angular_velocity_ = 0.0;
        remaining_.clear(); final_theta_.reset();
    }
    bool is_path_complete() const;

    RobotKinematicState hold(double dt);
    RobotKinematicState step(double dt);
    RobotKinematicState get_state() const;

private:
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
    double angular_velocity_ = 0.0;
    std::vector<std::pair<double, double>> remaining_;
    std::optional<double> final_theta_;

    double remaining_length() const;
    void advance(double dist);
    // target_theta 로 사다리꼴 각가속도 프로파일을 따라 회전. 회전 후 남은 각도 오차 반환.
    double rotate_step(double target_theta, double dt);
};

}  // namespace fms
