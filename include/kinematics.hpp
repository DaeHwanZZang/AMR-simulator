// 로봇 물리 시뮬레이션 (robot_client/sim/kinematics.py 포팅)
// 등가속/등감속 사다리꼴 속도 프로파일.
#pragma once

#include <utility>
#include <vector>

namespace fms {

struct RobotKinematicState {
    double x, y, theta;
    double velocity, acceleration;
    std::vector<std::pair<double, double>> local_path;
    bool path_complete;
};

class TrapezoidalKinematics {
public:
    double x, y, theta;
    double max_velocity, max_acceleration, max_deceleration;

    TrapezoidalKinematics(double x_, double y_, double theta_ = 0.0, double max_v = 1.0,
                           double max_a = 0.5, double max_d = 0.5)
        : x(x_), y(y_), theta(theta_), max_velocity(max_v), max_acceleration(max_a),
          max_deceleration(max_d) {}

    void set_path(std::vector<std::pair<double, double>> waypoints) { remaining_ = std::move(waypoints); }
    void set_pose(double nx, double ny, const double* ntheta = nullptr) {
        x = nx; y = ny;
        if (ntheta) theta = *ntheta;
    }
    void emergency_stop() { velocity_ = 0.0; acceleration_ = 0.0; remaining_.clear(); }
    bool is_path_complete() const { return remaining_length() < 1e-9; }

    RobotKinematicState hold(double dt);
    RobotKinematicState step(double dt);
    RobotKinematicState get_state() const;

private:
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
    std::vector<std::pair<double, double>> remaining_;

    double remaining_length() const;
    void advance(double dist);
};

}  // namespace fms
