// FMS <-> Robot Client 통신 메시지 스키마 (C++ 포팅)
// common/schemas.py 의 1:1 대응. Pydantic 의 필드/검증 규칙을 그대로 옮긴다.
// JSON 표현은 Python 쪽과 완전히 호환되어야 한다 (같은 필드명, 같은 enum 문자열).
#pragma once

#include <cjson/cJSON.h>

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace fms {

constexpr const char* PROTOCOL_VERSION = "1.0";

// 파싱/검증 실패. RobotComm 이 잡아서 메시지를 버리고 로그만 남긴다
// (comm.py 의 ValidationError 처리와 동일한 정책 — 이상한 메시지를 실행하느니
// 무시하는 쪽이 안전하다).
struct ValidationError : std::runtime_error {
    explicit ValidationError(const std::string& msg) : std::runtime_error(msg) {}
};

// =============================================================================
// Enum
// =============================================================================

enum class RobotState { IDLE, BUSY, CHARGING, ERROR, EMERGENCY };
enum class BusySubState { MOVING, WAITING, ACTING };
enum class OperatingMode { AUTO, MANUAL };
enum class PauseSource { LOCAL, FMS };
// STRAIGHT/DOCK/PARK 는 Action.params 가 필요하다 (Action::from_json 이 검증).
enum class ActionType { PICK, DROP, CHARGE, WAIT, STRAIGHT, DOCK, PARK };
enum class ActionStatus { WAITING, RUNNING, FINISHED, FAILED };
enum class ErrorLevel { WARNING, FATAL };
enum class ConnectionState { ONLINE, OFFLINE, CONNECTION_BROKEN };
enum class InstantActionType {
    CANCEL_ORDER, PAUSE, RESUME, RESET_ERROR, SET_MAP, INJECT_FAULT,
    INJECT_OBSTACLE, CLEAR_OBSTACLE, START_OBSTACLE_ZONE, STOP_OBSTACLE_ZONE,
};

std::string to_string(RobotState v);
std::string to_string(BusySubState v);
std::string to_string(OperatingMode v);
std::string to_string(PauseSource v);
std::string to_string(ActionType v);
std::string to_string(ActionStatus v);
std::string to_string(ErrorLevel v);
std::string to_string(ConnectionState v);
std::string to_string(InstantActionType v);

RobotState robot_state_from(const std::string& s);
BusySubState busy_sub_state_from(const std::string& s);
OperatingMode operating_mode_from(const std::string& s);
PauseSource pause_source_from(const std::string& s);
ActionType action_type_from(const std::string& s);
ActionStatus action_status_from(const std::string& s);
ErrorLevel error_level_from(const std::string& s);
ConnectionState connection_state_from(const std::string& s);
InstantActionType instant_action_type_from(const std::string& s);

// =============================================================================
// 값 객체
// =============================================================================

struct Pose {
    double x = 0.0, y = 0.0;
    double theta = 0.0;  // -pi ~ pi
    cJSON* to_json() const;
};

// Action/InstantAction 의 params 값 (JSON float|str|bool)
using ParamValue = std::variant<double, std::string, bool>;
using Params = std::map<std::string, ParamValue>;

// params 헬퍼 (문자열/숫자 어느 쪽으로 와도 관대하게 읽는다 — tools/*.py 가 JSON
// number 로 보내는 값도, 문자열로 보내는 값도 있다)
double param_double(const Params& p, const std::string& key, double def);
std::string param_string(const Params& p, const std::string& key, const std::string& def);
inline bool param_has(const Params& p, const std::string& key) { return p.find(key) != p.end(); }

// 노드 도착 후 동작. params 는 액션 종류별:
//   STRAIGHT : distance (m, 필수, 0 아님. + 전진 / - 후진), speed (m/s, 선택)
//              노드 theta 방향으로 경로계획 없이 직선 이동한다.
//   DOCK     : entry (FORWARD|BACKWARD, 필수), undock_distance (m > 0, 필수),
//              min_charge (% 0~100, 필수), speed (선택)
//   PARK     : entry, undock_distance (필수), speed (선택)
//   DOCK/PARK 의 노드 position 은 도킹(주차) 완료 자세다. 로봇은 그 앞 undock_distance 지점까지
//   A* 로 간 뒤 직선으로 들어간다. 빠져나올 때도 같은 직선을 되짚는다.
struct Action {
    std::string action_id;
    ActionType action_type = ActionType::WAIT;
    double duration = 0.0;
    std::optional<std::string> payload_id;
    Params params;
    cJSON* to_json() const;
    static Action from_json(cJSON* obj);  // throws ValidationError
};

struct TimeWindow {
    double enter = 0.0, exit = 0.0;
    cJSON* to_json() const;
    static TimeWindow from_json(cJSON* obj);
};

struct Point {
    double x = 0.0, y = 0.0;
    cJSON* to_json() const;
};

struct ObstacleInfo {
    double x = 0.0, y = 0.0, width = 0.0, height = 0.0;
    cJSON* to_json() const;
};

struct NodeProgress {
    std::string node_id;
    int sequence_id = 0;
    bool released = false;
    cJSON* to_json() const;
};

struct ActionProgress {
    std::string action_id;
    ActionStatus status = ActionStatus::WAITING;
    cJSON* to_json() const;
};

struct ErrorInfo {
    std::string error_type;
    ErrorLevel level = ErrorLevel::WARNING;
    std::string description;
    cJSON* to_json() const;
};

struct BatteryInfo {
    double charge = 100.0;
    bool charging = false;
    std::optional<double> estimated_runtime;
    cJSON* to_json() const;
};

// =============================================================================
// Order : FMS -> Robot
// =============================================================================

struct OrderNode {
    std::string node_id;
    int sequence_id = 0;
    bool released = false;
    Pose position;
    std::optional<Action> action;
    std::optional<TimeWindow> reserved_window;
    static OrderNode from_json(cJSON* obj);
};

struct Order {
    std::string version = PROTOCOL_VERSION;
    long long header_id = 0;
    double timestamp = 0.0;
    std::string robot_id;

    std::string order_id;
    int order_update_id = 0;
    std::optional<std::string> task_id;
    std::vector<OrderNode> nodes;

    // schemas.py Order._check_sequence 검증까지 포함
    static Order from_json_string(const std::string& payload);
};

// =============================================================================
// State : Robot -> FMS
// =============================================================================

struct State {
    std::string version = PROTOCOL_VERSION;
    long long header_id = 0;
    double timestamp = 0.0;
    std::string robot_id;

    RobotState state = RobotState::IDLE;
    std::optional<BusySubState> sub_state;
    OperatingMode mode = OperatingMode::AUTO;
    bool paused = false;
    std::optional<PauseSource> pause_source;

    Pose pose;
    double velocity = 0.0;
    double width = 0.5;
    double length = 0.7;

    std::vector<Point> local_path;

    // 현재 주행에 쓰는 맵. FMS 는 이 값이 활성 맵과 같은 로봇에게만 job 을 준다.
    // 둘 다 값이 있거나 둘 다 비어 있어야 한다.
    std::optional<std::string> map_id;
    std::optional<std::string> map_version;

    std::string last_node_id;
    int last_node_sequence_id = 0;

    std::optional<std::string> order_id;
    int order_update_id = 0;
    std::vector<NodeProgress> node_states;
    std::vector<ActionProgress> action_states;

    BatteryInfo battery;
    std::vector<ErrorInfo> errors;
    std::vector<ObstacleInfo> detected_obstacles;

    std::string to_json_string() const;
};

// =============================================================================
// Connection : Robot -> FMS
// =============================================================================

struct Connection {
    std::string version = PROTOCOL_VERSION;
    long long header_id = 0;
    double timestamp = 0.0;
    std::string robot_id;
    ConnectionState connection_state = ConnectionState::ONLINE;

    std::string to_json_string() const;
};

// =============================================================================
// InstantAction : FMS -> Robot
// =============================================================================

struct InstantAction {
    std::string version = PROTOCOL_VERSION;
    long long header_id = 0;
    double timestamp = 0.0;
    std::string robot_id;

    std::string action_id;
    InstantActionType action_type = InstantActionType::PAUSE;
    Params params;

    static InstantAction from_json_string(const std::string& payload);
    std::string to_json_string() const;  // 로봇 클라이언트가 만들 일은 없지만 CLI 툴에서 쓴다

    // params 헬퍼 (param_double 등의 얇은 래퍼)
    double get_double(const std::string& key, double def) const;
    std::string get_string(const std::string& key, const std::string& def) const;
    int get_int(const std::string& key, int def) const;
    bool has(const std::string& key) const;
};

// =============================================================================
// 토픽 헬퍼
// =============================================================================

constexpr const char* TOPIC_PREFIX = "fms/v1";

std::string topic_order(const std::string& robot_id);
std::string topic_state(const std::string& robot_id);
std::string topic_connection(const std::string& robot_id);
std::string topic_instant(const std::string& robot_id);

}  // namespace fms
