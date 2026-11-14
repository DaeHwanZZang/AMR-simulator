// 오더 실행기 (robot_client/executor.py 포팅). 상세 로직/주석은 원본 참고 —
// 여기서는 구조만 요약한다. 상태머신 규칙은 CLAUDE.md/robot_client.md 문서 그대로다.
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "battery.hpp"
#include "chargers.hpp"
#include "dynamic_obstacles.hpp"
#include "kinematics.hpp"
#include "map_model.hpp"
#include "path_refine.hpp"
#include "planner.hpp"
#include "obstacle_memory.hpp"
#include "local_planner.hpp"
#include "physics_world.hpp"
#include "recorder.hpp"
#include "scan_matcher.hpp"
#include "schemas.hpp"

namespace fms {

class MapSync;  // map_sync.hpp -- SET_MAP 수신 시 다운로드를 맡긴다

class OrderExecutor {
public:
    std::string robot_id;
    double robot_width, robot_length;
    DynamicObstacleField obstacle_field;
    double sensor_range;
    std::unique_ptr<PhysicsWorld> physics;  // 몸체/벽 물리 (kinematics 가 가리킨다). kinematics 보다 먼저 선언해야 한다
    TrapezoidalKinematics kinematics;
    BatterySim battery;
    const MapModel* map = nullptr;  // 없으면 경로계획/충돌감지 꺼짐 (소유하지 않음)
    MapSync* map_sync = nullptr;    // 없으면 SET_MAP 은 로깅만 (소유하지 않음)
    PathRefineParams path_refine;   // enabled=false 가 기본 (A* 결과 그대로 사용)
    bool smooth_enabled = true;     // 곡선 주행 (코너 Bézier + Pure Pursuit). --no-smooth-path 로 끔
    SmoothParams smooth;
    ObstacleMemory obstacle_memory; // 범위 밖 정지 장애물 기억 (--obstacle-memory)
    LocalPlanner local_planner;     // 장애물 우회 (config.radius = --local-plan-radius)
    double local_plan_rate = 5.0;   // Hz, --local-plan-rate
    CenterlineParams centerline;    // 복도 중앙 주행. enabled=true 가 기본 (--no-center-path 로 끔)
    std::unique_ptr<Recorder> recorder;  // 있으면 tick 마다 한 줄 기록 (--record)
    // 로컬라이징 (가상 라이다 + 스캔 매칭). 힌트를 받고 localize_time 초 뒤에 매칭 결과로 끝난다
    LidarParams lidar;
    MatchParams match_params;
    double localize_time = 1.0;
    // false(기본): 시작/맵 교체/순간이동 뒤에도 위치를 안다고 본다. true: UNLOCALIZED 가 되어 힌트가 있어야 주행
    bool require_localization = false;
    // 배터리 임계값 (%). low 아래면 WARNING LOW_BATTERY, critical 아래로 내려가는 순간 FATAL BATTERY_CRITICAL
    // (오더를 버린다), 0 이면 FATAL BATTERY_EMPTY + 구동 불가. critical 아래에서는 DOCK 오더만 받는다
    double battery_low = 20.0;
    double battery_critical = 5.0;
    std::mt19937 lidar_rng{1};  // main 이 --seed/로봇 id 로 다시 시드한다

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
    // 오더(또는 오더 업데이트)를 받아 order_id 를 바꾼 직후, 경로계획 전에 부른다.
    // 큰 맵에서 A* 가 수 초~수십 초 걸려도 FMS 가 "오더 수신"(State.order_id)을 바로 확인하게 State 를 먼저 보낸다.
    std::function<void()> on_order_received;
    bool accept_order(const Order& order);
    void handle_instant(const InstantAction& action);
    void raise_fatal(ErrorInfo error);
    // 강제 로컬라이징: 로봇(참 자세)을 그 자세로 옮기고 바로 LOCALIZED (디버그/시뮬 전용, 오더를 버린다)
    bool force_relocalize(double x, double y, double theta = 0.0);
    // 시뮬 전용 순간이동 (엘리베이터가 옮겨 준 것): 참 자세만 옮기고 UNLOCALIZED 로 만든다. 힌트가 있어야 다시 주행한다
    bool sim_teleport(double x, double y, double theta, const std::string& map_uid = "");
    // 위치 힌트: 추정을 힌트로 두고 LOCALIZING 을 시작한다 (참 자세는 그대로). 맵이 없거나 맵 밖이면 거절
    bool localize_hint(double x, double y, double theta);
    LocalizationState localization() const { return localization_; }
    double localization_score() const { return loc_score_; }

    // 맵 교체 (map_sync 워커가 받아온 새 번들). 현재 위치 기준으로 경로 재계산.
    // uid/version 은 State 로 FMS 에 보고된다 (FMS 의 job 게이팅 기준).
    void set_map(const MapModel* new_map, std::string map_uid, std::string map_version);

    // GUI 에서 로봇 크기를 바꿨을 때 물리 상자도 맞춘다. friction/mass 가 0 보다 크면 그 값으로도 바꾼다
    void apply_robot_geometry(double friction = -1.0, double mass = -1.0);

    // 공유 세계 반영 (시뮬 전용): 월드 장애물과 다른 로봇을 obstacle_field 에 맞춘다 (없어진 것은 지운다).
    // main 루프가 tick 직전에 부른다. 이후는 다른 장애물과 같은 경로를 탄다 (감지 -> 기억 -> 우회/대기, 충돌 판정).
    // 다른 맵(map_id 가 둘 다 있고 다름)의 것은 무시한다.
    void sync_world(const std::vector<WorldObstacle>& world, const std::vector<PeerRobot>& peers);

    // -- 매 tick --
    void tick(double dt);

    State build_state(long long header_id, double timestamp) const;
    // 뷰어/GUI 디버그용 물리 관측값 (fms/v1/{id}/physics). 프로토콜이 아니다
    PhysicsDebug build_physics(long long header_id, double timestamp) const;
    // Global 경로(현재 구간) + Local 계획 상태 (sim/v1/{id}/navigation). 바뀌었을 때(+하트비트)만 값이 있다.
    std::optional<NavigationDebug> take_navigation(long long header_id, double timestamp);

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

    // 복도 중앙 주행·경로 다듬기용 장애물 거리 장. 맵당 lazy 생성, set_map 에서 무효화.
    std::unique_ptr<ClearanceField> clearance_;

    // 로컬라이징 상태. UNLOCALIZED/LOCALIZING 이면 주행 억제 + 오더 거절 (motion_inhibited / can_accept_job)
    LocalizationState localization_ = LocalizationState::UNLOCALIZED;
    double loc_score_ = 0.0;
    std::optional<Pose2> loc_hint_;
    double loc_timer_ = 0.0;
    std::unique_ptr<ScanMatcher> matcher_;  // 맵당 lazy 생성 (likelihood field), set_map 에서 버린다
    void set_unlocalized(const char* why);
    void tick_localization(double dt);

    std::string current_map_uid_;      // 비어 있으면 State 에 map_id 안 실음
    std::string current_map_version_;

    std::string order_id_;
    int order_update_id_ = 0;
    std::vector<OrderNode> nodes_;
    int current_index_ = 0;
    std::string last_node_id_;
    int last_node_sequence_id_ = 0;
    std::optional<std::pair<std::string, int>> last_completed_;

    std::vector<int> segment_;  // 지금 계획/주행 중인 목적지 노드 (항상 최대 1개: 도착한 뒤에 다음 노드를 계획한다)
    // 구간의 정상 경로(장애물 없이 계획한 최종 경로). 장애물 우회/복귀의 기준이 된다.
    std::vector<std::pair<double, double>> leg_nominal_;
    size_t leg_progress_ = 0;      // leg_nominal_ 에서 로봇이 지나는 구간 번호 (단조 증가)
    bool detour_active_ = false;   // 우회 경로를 따라가는 중
    double since_local_plan_ = 1.0;
    double since_global_replan_ = 1.0;  // 로컬 우회가 안 될 때 구간 전체 재계획(장애물 반영)을 시도하는 간격
    std::optional<Action> acting_;
    std::vector<LoadInfo> loads_;              // 싣고 있는 적재물 (PICK/DROP 완료 기준)
    void update_loads(const Action& action);
    double acting_elapsed_ = 0.0;

    // Global 경로 = 지금 향하는 노드 하나까지의 정상 경로(leg_nominal_) + 액션 직선 꼬리. 다음 구간은 도착한 뒤 replan() 이 만든다.
    // 표시 전용이다 (sim/v1/{id}/navigation). 구간이 바뀌거나 Order/맵이 바뀌면 version 을 올린다.
    int nav_version_ = 0;
    std::string nav_sig_ = "?";
    double nav_sent_at_ = -1e9;
    void nav_changed() { ++nav_version_; }
    std::vector<std::pair<double, double>> node_tail(const OrderNode& n) const;  // 구간 목표 뒤에 이어지는 직선 (도킹 진입, STRAIGHT)

    // 경로계획 없는 직선 이동 (STRAIGHT 액션, DOCK/PARK 진입, 언도킹). 있으면 tick 이 이것만 돌린다.
    // kinematics 쪽 직선 목표는 PAUSE/E-STOP 에서 지워지므로 여기 목표를 들고 있다가 재개 때 다시 건다.
    enum class StraightPurpose { ACTION, DOCK_IN, UNDOCK };
    struct StraightMotion { double tx, ty, speed; StraightPurpose purpose; };
    std::optional<StraightMotion> straight_;

    // 도킹/주차 상태. DOCK/PARK 진입을 시작할 때 생기고 언도킹이 끝나야 사라진다.
    // 진입이나 언도킹 도중에 오더가 취소돼도 남아 있어서, 다음 이동 전에 exit 지점까지 먼저 빠져나온다
    // (도크 안쪽은 planner 팽창 영역이라 A* 로는 못 나온다).
    struct Docked {
        bool charger;            // DOCK = true (충전), PARK = false
        std::optional<Charger> station;  // DOCK 대상 충전기 (chargers.json)
        std::string entry;       // 진입 방향 (FORWARD|BACKWARD) — 충전기 접점 방향과 맞아야 한다
        double exit_x, exit_y;   // 진입 직전 자세 = 언도킹 목표
        double min_charge;       // 이 충전량(%) 이상이어야 새 job 을 받는다 (DOCK 만)
        double speed;
        bool contact = false;    // 진입 완료 (충전 접점 연결)
    };
    std::optional<Docked> docked_;
    std::vector<std::pair<std::string, ActionStatus>> action_states_;  // 삽입 순서 유지

    std::vector<Charger> chargers_;   // 지금 맵의 충전기 (맵 폴더 chargers.json, set_map 때 다시 읽는다)
    double prev_charge_ = -1.0;       // 배터리 임계값 하향 통과 감지용 (음수 = 아직 모름)
    void check_battery();
    bool has_error(const char* type) const;
    void fail_action(const char* error_type, const std::string& desc);

    double sim_time_ = 0.0;  // 시작 후 누적 tick 시간 (기록용)
    std::set<std::string> detected_obstacle_ids_;
    std::vector<DynamicObstacle> last_detected_;

    void set_base(RobotState s) { base_state_ = s; sub_state_ = std::nullopt; }
    void set_busy(BusySubState sub) { base_state_ = RobotState::BUSY; sub_state_ = sub; }
    void resume_motion();
    bool apply_order_update(const Order& order);
    std::vector<DynamicObstacle> update_detections();
    bool path_blocked_by(const std::vector<DynamicObstacle>& detected) const;
    // 장애물 처리: 달리면서 우회/복귀. true 면 이번 tick 은 정지(대기)로 처리했다.
    bool handle_obstacles(double dt, const std::vector<DynamicObstacle>& detected);
    void restore_nominal();
    bool check_collision(double x, double y, double prev_x, double prev_y);
    // 동적 장애물/다른 로봇과 몸체가 겹쳤는가 (참 자세 원판 vs 장애물 몸체, 감지 여부와 무관).
    // 움직이는 중에 새로 겹치면 FATAL COLLISION 을 올리고 true. 서 있을 때 와서 닿은 것, 이미 겹쳐 있던 것은 넘어간다.
    bool check_obstacle_contact(bool moving);
    std::set<std::string> colliding_ids_;  // 지금 몸체가 겹쳐 있는 장애물 (같은 접촉으로 오류를 반복하지 않게)
    void tick_logic(double dt);
    void record_sample(double dt);
    void tick_action(double dt);
    void tick_straight(double dt, const std::vector<DynamicObstacle>& detected);
    void finish_straight();
    void start_undock();
    bool docked_charging() const { return docked_ && docked_->charger && docked_->contact; }
    // 충전기에 붙어 있고 min_charge 미달 -> CHARGING 으로 보고하고 job 을 받지 않는다
    bool charge_hold() const { return docked_charging() && battery.charge < docked_->min_charge; }
    RobotState idle_state() const { return charge_hold() ? RobotState::CHARGING : RobotState::IDLE; }
    std::pair<double, double> approach_point(const OrderNode& node) const;
    void replan();
    std::vector<std::pair<double, double>> plan_segment_path(
        const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles = nullptr);
    void on_segment_arrived();
    void clear_order();
    void set_action_status(const std::string& action_id, ActionStatus st);
    ActionStatus get_action_status(const std::string& action_id, ActionStatus def) const;
};

}  // namespace fms
