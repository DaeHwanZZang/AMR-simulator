// 오더 실행기 (robot_client/executor.py 포팅). 상세 로직/주석은 원본 참고 —
// 여기서는 구조만 요약한다. 상태머신 규칙은 CLAUDE.md/robot_client.md 문서 그대로다.
#pragma once

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "battery.hpp"
#include "dynamic_obstacles.hpp"
#include "kinematics.hpp"
#include "map_model.hpp"
#include "schemas.hpp"

namespace fms {

class MapSync;  // map_sync.hpp -- SET_MAP 수신 시 다운로드를 맡긴다

class OrderExecutor {
public:
    std::string robot_id;
    double robot_width, robot_length;
    DynamicObstacleField obstacle_field;
    double sensor_range;
    TrapezoidalKinematics kinematics;
    BatterySim battery;
    const MapModel* map = nullptr;  // 없으면 경로계획/충돌감지 꺼짐 (소유하지 않음)
    MapSync* map_sync = nullptr;    // 없으면 SET_MAP 은 로깅만 (소유하지 않음)

    OrderExecutor(std::string robot_id_, double start_x, double start_y, double start_theta,
                  double max_velocity, double max_acceleration, BatterySim battery_,
                  const MapModel* map_model, double robot_width_, double robot_length_,
                  DynamicObstacleField obstacle_field_, double sensor_range_);

    // -- 상태 조회 --
    RobotState state() const { return estop_latched_ ? RobotState::EMERGENCY : base_state_; }
    bool motion_inhibited() const;
    bool can_accept_job() const;

    // -- 물리 패널 --
    std::string press_start();
    std::string press_stop();
    void set_estop(bool latched);
    bool toggle_estop();
    void set_mode(OperatingMode mode);
    bool pause(PauseSource source);
    bool resume(PauseSource by);
    void reset_error();

    // -- 오더/명령 --
    bool accept_order(const Order& order);
    void handle_instant(const InstantAction& action);
    void raise_fatal(ErrorInfo error);
    bool force_relocalize(double x, double y, double theta = 0.0);

    // 맵 교체 (map_sync 워커가 받아온 새 번들). 현재 위치 기준으로 경로 재계산.
    // uid/version 은 State 로 FMS 에 보고된다 (FMS 의 job 게이팅 기준).
    void set_map(const MapModel* new_map, std::string map_uid, std::string map_version);

    // -- 매 tick --
    void tick(double dt);

    State build_state(long long header_id, double timestamp) const;

    // GUI/콘솔 설정 변경 (IDLE 일 때만 허용, main 이 호출)
    OperatingMode mode() const { return mode_; }
    bool paused() const { return paused_; }
    std::optional<std::string> order_id() const { return order_id_.empty() ? std::nullopt : std::optional(order_id_); }
    bool estop_latched() const { return estop_latched_; }
    std::optional<PauseSource> pause_source() const { return pause_source_; }

private:
    RobotState base_state_ = RobotState::IDLE;
    std::optional<BusySubState> sub_state_;
    bool estop_latched_ = false;
    bool paused_ = false;
    std::optional<PauseSource> pause_source_;
    OperatingMode mode_ = OperatingMode::AUTO;
    std::vector<ErrorInfo> errors_;

    std::string current_map_uid_;      // 비어 있으면 State 에 map_id 안 실음
    std::string current_map_version_;

    std::string order_id_;
    int order_update_id_ = 0;
    std::vector<OrderNode> nodes_;
    int current_index_ = 0;
    std::string last_node_id_;
    int last_node_sequence_id_ = 0;
    std::optional<std::pair<std::string, int>> last_completed_;

    std::vector<int> segment_;
    // 이번 세그먼트 경로계획 결과. segment_wp_bounds_[k] = segment_[k] 노드에
    // 도달하는 시점의 누적 경유점 개수. 세그먼트 중간에서 멈췄다가(EMERGENCY/PAUSE/
    // MANUAL) 재개할 때 이미 지나온 노드로 되돌아가지 않도록 진행도를 추적한다.
    std::vector<size_t> segment_wp_bounds_;
    size_t segment_wp_total_ = 0;
    std::optional<Action> acting_;
    double acting_elapsed_ = 0.0;
    std::vector<std::pair<std::string, ActionStatus>> action_states_;  // 삽입 순서 유지

    double since_dynamic_replan_ = 0.0;
    std::set<std::string> detected_obstacle_ids_;
    std::vector<DynamicObstacle> last_detected_;

    void set_base(RobotState s) { base_state_ = s; sub_state_ = std::nullopt; }
    void set_busy(BusySubState sub) { base_state_ = RobotState::BUSY; sub_state_ = sub; }
    void resume_motion();
    bool apply_order_update(const Order& order);
    std::vector<DynamicObstacle> update_detections();
    bool path_blocked_by(const std::vector<DynamicObstacle>& detected) const;
    void try_dynamic_replan(const std::vector<DynamicObstacle>& detected);
    bool check_collision(double x, double y, double prev_x, double prev_y);
    void tick_action(double dt);
    void replan();
    std::vector<std::pair<double, double>> plan_segment_path(
        const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles = nullptr);
    void consume_reached_nodes();
    void on_segment_arrived();
    void clear_order();
    void set_action_status(const std::string& action_id, ActionStatus st);
    ActionStatus get_action_status(const std::string& action_id, ActionStatus def) const;
};

}  // namespace fms
