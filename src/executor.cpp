#include "executor.hpp"

#include <algorithm>
#include <cmath>

#include "log.hpp"
#include "map_sync.hpp"
#include "planner.hpp"

namespace fms {

namespace {

double _DYNAMIC_REPLAN_RETRY = 1.0;
double _SENSOR_HYSTERESIS_MARGIN = 0.5;

// Liang-Barsky: 선분 a->b 가 사각형 [rx0,rx1]x[ry0,ry1] 과 겹치는가.
// planner.plan_path 와 같은 기준(사각형+robot_width/2 팽창)으로 막힘을 판정해야
// executor.py 에서 겪었던 "회피 경로 생성됐지만 계속 막힘 오판" 버그를 피한다.
bool segment_intersects_rect(double ax, double ay, double bx, double by, double rx0, double ry0,
                              double rx1, double ry1) {
    double dx = bx - ax, dy = by - ay;
    double t0 = 0.0, t1 = 1.0;
    double ps[4] = {-dx, dx, -dy, dy};
    double qs[4] = {ax - rx0, rx1 - ax, ay - ry0, ry1 - ay};
    for (int i = 0; i < 4; ++i) {
        double p = ps[i], q = qs[i];
        if (p == 0) {
            if (q < 0) return false;
            continue;
        }
        double t = q / p;
        if (p < 0) {
            if (t > t1) return false;
            t0 = std::max(t0, t);
        } else {
            if (t < t0) return false;
            t1 = std::min(t1, t);
        }
    }
    return t0 <= t1;
}

}  // namespace

OrderExecutor::OrderExecutor(std::string robot_id_, double start_x, double start_y, double start_theta,
                              double max_velocity, double max_acceleration, BatterySim battery_,
                              const MapModel* map_model, double robot_width_, double robot_length_,
                              DynamicObstacleField obstacle_field_, double sensor_range_)
    : robot_id(std::move(robot_id_)), robot_width(robot_width_), robot_length(robot_length_),
      obstacle_field(std::move(obstacle_field_)), sensor_range(sensor_range_),
      kinematics(start_x, start_y, start_theta, max_velocity, max_acceleration, max_acceleration),
      battery(std::move(battery_)), map(map_model) {}

bool OrderExecutor::motion_inhibited() const {
    return estop_latched_ || paused_ || mode_ == OperatingMode::MANUAL || base_state_ == RobotState::ERROR;
}

bool OrderExecutor::can_accept_job() const {
    if (state() != RobotState::IDLE || mode_ != OperatingMode::AUTO || paused_ || !order_id_.empty())
        return false;
    for (auto& e : errors_)
        if (e.level == ErrorLevel::FATAL) return false;
    return true;
}

std::string OrderExecutor::press_start() {
    if (estop_latched_) {
        log::warning("[%s] start ignored: release the e-stop latch first", robot_id.c_str());
        return "release the e-stop latch first";
    }
    if (base_state_ == RobotState::ERROR) {
        reset_error();
        return "error cleared";
    }
    if (paused_) {
        resume(PauseSource::LOCAL);
        return "pause released";
    }
    pause(PauseSource::LOCAL);
    return "paused";
}

std::string OrderExecutor::press_stop() {
    if (order_id_.empty()) {
        log::debug("[%s] stop button: no active order", robot_id.c_str());
        return "no active order";
    }
    log::info("[%s] stop button: cancelling order %s", robot_id.c_str(), order_id_.c_str());
    clear_order();
    return "order cancelled";
}

void OrderExecutor::set_estop(bool latched) {
    if (latched == estop_latched_) return;
    estop_latched_ = latched;
    if (latched) {
        kinematics.emergency_stop();
        log::warning("[%s] e-stop latched -- stopping immediately (job %s kept)", robot_id.c_str(), order_id_.c_str());
    } else {
        log::info("[%s] e-stop latch released -- returning to %s", robot_id.c_str(), to_string(base_state_).c_str());
        resume_motion();
    }
}

bool OrderExecutor::toggle_estop() {
    set_estop(!estop_latched_);
    return estop_latched_;
}

void OrderExecutor::set_mode(OperatingMode m) {
    if (m == mode_) return;
    mode_ = m;
    if (m == OperatingMode::MANUAL) {
        kinematics.emergency_stop();
        log::info("[%s] switched to MANUAL -- motion stopped (job %s kept)", robot_id.c_str(), order_id_.c_str());
    } else {
        log::info("[%s] switched to AUTO -- replanning from current pose", robot_id.c_str());
        resume_motion();
    }
}

bool OrderExecutor::pause(PauseSource source) {
    if (paused_) return false;
    paused_ = true;
    pause_source_ = source;
    kinematics.emergency_stop();
    log::info("[%s] paused (%s)", robot_id.c_str(), to_string(source).c_str());
    return true;
}

bool OrderExecutor::resume(PauseSource by) {
    if (!paused_) return false;
    if (by == PauseSource::FMS && pause_source_ == PauseSource::LOCAL) {
        log::warning("[%s] rejecting FMS resume: pause was set locally", robot_id.c_str());
        return false;
    }
    paused_ = false;
    pause_source_ = std::nullopt;
    log::info("[%s] pause released (%s)", robot_id.c_str(), to_string(by).c_str());
    resume_motion();
    return true;
}

void OrderExecutor::reset_error() {
    errors_.clear();
    if (base_state_ == RobotState::ERROR) {
        set_base(RobotState::IDLE);
        resume_motion();
    }
}

void OrderExecutor::resume_motion() {
    if (motion_inhibited()) return;
    if (acting_) return;
    replan();
}

void OrderExecutor::set_map(const MapModel* new_map, std::string map_uid, std::string map_version) {
    map = new_map;
    current_map_uid_ = std::move(map_uid);
    current_map_version_ = std::move(map_version);
    log::info("[%s] map switched: uid=%s version=%s", robot_id.c_str(), current_map_uid_.c_str(),
              current_map_version_.c_str());
    // 정지 지점이 노드 사이일 수 있고 새 맵에 금지구역이 추가됐을 수 있으므로
    // 현재 위치 기준으로 A* 를 다시 돌린다 (EMERGENCY/PAUSE/MANUAL 이면 재개 시 자동).
    resume_motion();
}

void OrderExecutor::set_action_status(const std::string& id, ActionStatus st) {
    for (auto& [aid, s] : action_states_)
        if (aid == id) { s = st; return; }
    action_states_.push_back({id, st});
}

ActionStatus OrderExecutor::get_action_status(const std::string& id, ActionStatus def) const {
    for (auto& [aid, s] : action_states_)
        if (aid == id) return s;
    return def;
}

bool OrderExecutor::accept_order(const Order& order) {
    if (last_completed_ && order.order_id == last_completed_->first &&
        order.order_update_id <= last_completed_->second) {
        log::info("[%s] ignoring already-completed order: %s update %d", robot_id.c_str(), order.order_id.c_str(),
                  order.order_update_id);
        return false;
    }

    if (order_id_ == order.order_id && !order_id_.empty()) {
        if (order.order_update_id <= order_update_id_) {
            log::info("[%s] ignoring stale order: %s update %d <= %d", robot_id.c_str(),
                      order.order_id.c_str(), order.order_update_id, order_update_id_);
            return false;
        }
        return apply_order_update(order);
    }

    if (!can_accept_job()) {
        log::warning("[%s] rejecting order %s: state=%s mode=%s paused=%d active=%s", robot_id.c_str(),
                     order.order_id.c_str(), to_string(state()).c_str(), to_string(mode_).c_str(),
                     paused_, order_id_.c_str());
        return false;
    }

    order_id_ = order.order_id;
    order_update_id_ = order.order_update_id;
    nodes_ = order.nodes;
    current_index_ = 0;
    action_states_.clear();
    for (auto& n : nodes_)
        if (n.action) set_action_status(n.action->action_id, ActionStatus::WAITING);
    acting_ = std::nullopt;

    int released_count = 0;
    for (auto& n : nodes_) if (n.released) ++released_count;
    log::info("[%s] order received %s update=%d, %zu nodes (%d released)", robot_id.c_str(),
              order.order_id.c_str(), order.order_update_id, nodes_.size(), released_count);
    replan();
    return true;
}

bool OrderExecutor::apply_order_update(const Order& order) {
    size_t old_len = nodes_.size();
    order_update_id_ = order.order_update_id;
    nodes_ = order.nodes;
    for (auto& n : nodes_) {
        if (!n.action) continue;
        bool known = std::any_of(action_states_.begin(), action_states_.end(),
                                  [&](auto& p) { return p.first == n.action->action_id; });
        if (!known) set_action_status(n.action->action_id, ActionStatus::WAITING);
    }

    if (!acting_) replan();

    int released_count = 0;
    for (auto& n : nodes_) if (n.released) ++released_count;
    log::info("[%s] order updated %s update=%d, nodes %zu -> %zu (%d released)", robot_id.c_str(),
              order.order_id.c_str(), order.order_update_id, old_len, nodes_.size(), released_count);
    return true;
}

void OrderExecutor::handle_instant(const InstantAction& action) {
    switch (action.action_type) {
        case InstantActionType::CANCEL_ORDER:
            log::info("[%s] order cancelled: %s", robot_id.c_str(), order_id_.c_str());
            clear_order();
            break;
        case InstantActionType::PAUSE:
            pause(PauseSource::FMS);
            break;
        case InstantActionType::RESUME:
            resume(PauseSource::FMS);
            break;
        case InstantActionType::RESET_ERROR:
            log::info("[%s] error reset", robot_id.c_str());
            reset_error();
            break;
        case InstantActionType::SET_MAP: {
            std::string uid = action.get_string("map_id", "");
            std::string version = action.get_string("map_version", "");
            std::string url = action.get_string("url", "");
            if (uid.empty() || version.empty() || url.empty()) {
                log::warning("[%s] SET_MAP ignored: map_id/map_version/url required", robot_id.c_str());
                break;
            }
            log::info("[%s] SET_MAP received: map_id=%s version=%s url=%s", robot_id.c_str(),
                      uid.c_str(), version.c_str(), url.c_str());
            if (map_sync)
                map_sync->request(uid, version, url);  // 백그라운드 다운로드, tick 루프가 회수
            else
                log::warning("[%s] SET_MAP: map sync disabled (no cache configured)", robot_id.c_str());
            break;
        }
        case InstantActionType::INJECT_FAULT: {
            std::string fault = action.get_string("fault", "UNKNOWN_FAULT");
            log::warning("[%s] fault injected: %s (order %s aborted)", robot_id.c_str(), fault.c_str(), order_id_.c_str());
            ErrorInfo e; e.error_type = fault; e.level = ErrorLevel::FATAL; e.description = "injected fault";
            raise_fatal(e);
            break;
        }
        case InstantActionType::INJECT_OBSTACLE: {
            std::string oid = action.has("obstacle_id") ? action.get_string("obstacle_id", "") : action.action_id;
            double x = action.get_double("x", 0.0), y = action.get_double("y", 0.0);
            double width = action.get_double("width", 0.4), height = action.get_double("height", 0.4);
            obstacle_field.add_manual(oid, x, y, width, height);
            log::info("[%s] manual obstacle placed: %s (%.2f, %.2f) %.2fx%.2f", robot_id.c_str(), oid.c_str(), x, y,
                      width, height);
            break;
        }
        case InstantActionType::CLEAR_OBSTACLE: {
            if (action.has("obstacle_id")) {
                std::string oid = action.get_string("obstacle_id", "");
                obstacle_field.clear(oid);
                log::info("[%s] manual obstacle removed: %s", robot_id.c_str(), oid.c_str());
            } else {
                obstacle_field.clear();
                log::info("[%s] manual obstacles removed: all", robot_id.c_str());
            }
            break;
        }
        case InstantActionType::START_OBSTACLE_ZONE: {
            Zone zone{action.get_double("x_min", 0), action.get_double("y_min", 0),
                      action.get_double("x_max", 0), action.get_double("y_max", 0)};
            std::optional<std::pair<ZoneSide, ZoneSide>> sides;
            std::string sides_str = action.get_string("sides", "");
            if (!sides_str.empty()) {
                auto dash = sides_str.find('-');
                sides = std::make_pair(zone_side_from_char(sides_str[0]),
                                        zone_side_from_char(sides_str[dash + 1]));
            }
            obstacle_field.configure_zone(zone, sides, action.get_double("spawn_interval", 5.0),
                                           action.get_int("max_count", 3), action.get_double("width", 0.4),
                                           action.get_double("height", 0.4), action.get_double("speed", 0.5));
            log::info("[%s] obstacle zone started: zone=(%.2f,%.2f,%.2f,%.2f)", robot_id.c_str(), zone.x_min,
                      zone.y_min, zone.x_max, zone.y_max);
            break;
        }
        case InstantActionType::STOP_OBSTACLE_ZONE:
            obstacle_field.stop_zone();
            log::info("[%s] obstacle zone spawning stopped (existing obstacles kept)", robot_id.c_str());
            break;
    }
}

void OrderExecutor::tick(double dt) {
    obstacle_field.tick(dt);
    auto detected = update_detections();
    last_detected_ = detected;

    if (motion_inhibited()) {
        battery.step(dt, 0.0);
        return;
    }
    if (acting_) {
        tick_action(dt);
        battery.step(dt, 0.0);
        return;
    }
    if (order_id_.empty()) {
        set_base(RobotState::IDLE);
        battery.step(dt, 0.0);
        return;
    }
    if (current_index_ >= (int)nodes_.size()) {
        log::info("[%s] order complete: %s", robot_id.c_str(), order_id_.c_str());
        last_completed_ = std::make_pair(order_id_, order_update_id_);
        clear_order();
        battery.step(dt, 0.0);
        return;
    }
    if (!nodes_[current_index_].released) {
        set_busy(BusySubState::WAITING);
        battery.step(dt, 0.0);
        return;
    }
    if (segment_.empty()) {
        replan();
        if (segment_.empty()) {
            if (base_state_ != RobotState::ERROR) set_busy(BusySubState::WAITING);
            battery.step(dt, 0.0);
            return;
        }
    }
    if (path_blocked_by(detected)) {
        auto kin_state = kinematics.hold(dt);
        battery.step(dt, kin_state.velocity);
        set_busy(BusySubState::WAITING);
        since_dynamic_replan_ += dt;
        if (since_dynamic_replan_ >= _DYNAMIC_REPLAN_RETRY) {
            since_dynamic_replan_ = 0.0;
            try_dynamic_replan(detected);
        }
        return;
    }
    since_dynamic_replan_ = 0.0;

    double prev_x = kinematics.x, prev_y = kinematics.y;
    auto kin_state = kinematics.step(dt);
    consume_reached_nodes();
    battery.step(dt, kin_state.velocity);

    if (check_collision(kin_state.x, kin_state.y, prev_x, prev_y)) return;

    if (kinematics.is_path_complete()) on_segment_arrived();
    else set_busy(BusySubState::MOVING);
}

bool OrderExecutor::check_collision(double x, double y, double prev_x, double prev_y) {
    if (map == nullptr || map->is_free(x, y)) return false;
    CellType cell = map->cell_at(x, y);
    log::error("[%s] collision: (%.2f, %.2f), stopping at (%.2f, %.2f), order %s aborted", robot_id.c_str(), x, y, prev_x,
               prev_y, order_id_.c_str());
    kinematics.set_pose(prev_x, prev_y);
    ErrorInfo e; e.error_type = "COLLISION"; e.level = ErrorLevel::FATAL;
    e.description = "(" + std::to_string(x) + ", " + std::to_string(y) + ") is an " +
                     std::string(cell == CellType::OCCUPIED ? "OCCUPIED" : "UNKNOWN") + " cell";
    raise_fatal(e);
    return true;
}

std::vector<DynamicObstacle> OrderExecutor::update_detections() {
    double x = kinematics.x, y = kinematics.y;
    std::vector<DynamicObstacle> detected;
    for (auto& [id, obs] : obstacle_field.obstacles) {
        double threshold = sensor_range;
        if (detected_obstacle_ids_.count(id)) threshold += _SENSOR_HYSTERESIS_MARGIN;
        if (obs.distance_to(x, y) <= threshold) detected.push_back(obs);
    }
    std::set<std::string> now_ids;
    for (auto& o : detected) now_ids.insert(o.obstacle_id);
    for (auto& o : detected) {
        if (!detected_obstacle_ids_.count(o.obstacle_id))
            log::info("[%s] obstacle detected: %s (%.2f, %.2f) %.2fx%.2f source=%s", robot_id.c_str(),
                      o.obstacle_id.c_str(), o.x, o.y, o.width, o.height,
                      o.source == ObstacleSource::MANUAL ? "MANUAL" : "SIM");
    }
    for (auto& id : detected_obstacle_ids_)
        if (!now_ids.count(id)) log::info("[%s] obstacle no longer detected: %s", robot_id.c_str(), id.c_str());
    detected_obstacle_ids_ = now_ids;
    return detected;
}

bool OrderExecutor::path_blocked_by(const std::vector<DynamicObstacle>& detected) const {
    if (detected.empty()) return false;
    std::vector<std::pair<double, double>> points = {{kinematics.x, kinematics.y}};
    for (auto& p : kinematics.get_state().local_path) points.push_back(p);
    if (points.size() < 2) return false;
    double margin = robot_width / 2;
    for (auto& obs : detected) {
        double hw = obs.width / 2 + margin, hh = obs.height / 2 + margin;
        double rx0 = obs.x - hw, rx1 = obs.x + hw, ry0 = obs.y - hh, ry1 = obs.y + hh;
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            if (segment_intersects_rect(points[i].first, points[i].second, points[i + 1].first,
                                         points[i + 1].second, rx0, ry0, rx1, ry1))
                return true;
        }
    }
    return false;
}

void OrderExecutor::try_dynamic_replan(const std::vector<DynamicObstacle>& detected) {
    if (map == nullptr || segment_.empty()) return;
    std::vector<std::tuple<double, double, double, double>> obstacles;
    for (auto& o : detected) obstacles.emplace_back(o.x, o.y, o.width, o.height);
    try {
        auto path = plan_segment_path(&obstacles);
        kinematics.set_path(path);
        log::info("[%s] replanned around dynamic obstacle (%zu detected)", robot_id.c_str(), detected.size());
    } catch (const PlanningError& e) {
        log::debug("[%s] no detour around dynamic obstacle, keeping wait: %s", robot_id.c_str(), e.what());
    }
}

void OrderExecutor::raise_fatal(ErrorInfo error) {
    errors_.push_back(std::move(error));
    kinematics.emergency_stop();
    clear_order();
    set_base(RobotState::ERROR);
}

void OrderExecutor::tick_action(double dt) {
    if (acting_->action_type == ActionType::CHARGE) set_base(RobotState::CHARGING);
    else set_busy(BusySubState::ACTING);

    acting_elapsed_ += dt;
    if (acting_elapsed_ < acting_->duration) return;
    set_action_status(acting_->action_id, ActionStatus::FINISHED);
    log::info("[%s] action complete: %s (%s)", robot_id.c_str(), acting_->action_id.c_str(),
              to_string(acting_->action_type).c_str());
    acting_ = std::nullopt;
    acting_elapsed_ = 0.0;
    battery.charging = false;
    replan();
}

void OrderExecutor::replan() {
    segment_.clear();
    segment_wp_bounds_.clear();
    segment_wp_total_ = 0;
    if (motion_inhibited() || order_id_.empty()) {
        kinematics.set_path({});
        return;
    }
    for (int i = current_index_; i < (int)nodes_.size(); ++i) {
        auto& node = nodes_[i];
        if (!node.released) break;
        segment_.push_back(i);
        if (node.action) break;
    }
    if (segment_.empty()) {
        log::debug("[%s] skipping replan: no released nodes in this segment", robot_id.c_str());
        kinematics.set_path({});
        return;
    }
    log::info("[%s] planning path from (%.2f, %.2f)", robot_id.c_str(), kinematics.x, kinematics.y);
    try {
        auto path = plan_segment_path();
        log::info("[%s] path planned: %zu waypoints", robot_id.c_str(), path.size());
        kinematics.set_path(path);
    } catch (const PlanningError& e) {
        log::error("[%s] path planning failed: %s", robot_id.c_str(), e.what());
        segment_.clear();
        ErrorInfo err; err.error_type = "NO_PATH"; err.level = ErrorLevel::FATAL; err.description = e.what();
        raise_fatal(err);
    }
}

std::vector<std::pair<double, double>> OrderExecutor::plan_segment_path(
    const std::vector<std::tuple<double, double, double, double>>* dynamic_obstacles) {
    std::vector<std::pair<double, double>> targets;
    for (int i : segment_) targets.emplace_back(nodes_[i].position.x, nodes_[i].position.y);

    segment_wp_bounds_.clear();
    if (map == nullptr) {
        for (size_t k = 0; k < targets.size(); ++k) segment_wp_bounds_.push_back(k + 1);
        segment_wp_total_ = targets.size();
        return targets;
    }

    std::vector<std::pair<double, double>> full_path;
    std::pair<double, double> start = {kinematics.x, kinematics.y};
    for (auto& target : targets) {
        auto seg = plan_path(*map, start, target, robot_width, robot_length, dynamic_obstacles);
        full_path.insert(full_path.end(), seg.begin(), seg.end());
        segment_wp_bounds_.push_back(full_path.size());
        start = target;
    }
    segment_wp_total_ = full_path.size();
    return full_path;
}

void OrderExecutor::consume_reached_nodes() {
    // 세그먼트 중간 노드를 물리적으로 지나갈 때마다 current_index_ 를 전진시킨다.
    // 이래야 EMERGENCY/PAUSE/MANUAL 로 멈췄다 재개할 때 replan 이 "다음 미도달
    // 노드" 부터 계획하고, 이미 지나온 경유지로 되돌아가지 않는다.
    // 세그먼트의 마지막 노드는 on_segment_arrived 가 처리한다 (action 실행 포함).
    if (segment_.empty() || segment_wp_bounds_.size() != segment_.size()) return;
    int remaining = (int)kinematics.get_state().local_path.size();
    int consumed = (int)segment_wp_total_ - remaining;
    if (consumed <= 0) return;

    int passed = 0;
    for (size_t k = 0; k + 1 < segment_.size(); ++k)
        if (consumed >= (int)segment_wp_bounds_[k]) passed = (int)k + 1;
    if (passed <= 0) return;

    int last_reached = segment_[passed - 1];
    if (last_reached + 1 <= current_index_) return;
    auto& node = nodes_[last_reached];
    last_node_id_ = node.node_id;
    last_node_sequence_id_ = node.sequence_id;
    current_index_ = last_reached + 1;
    log::info("[%s] node passed: %s (seq %d)", robot_id.c_str(), node.node_id.c_str(), node.sequence_id);
}

void OrderExecutor::on_segment_arrived() {
    if (segment_.empty()) { replan(); return; }
    int node_index = segment_.back();
    auto node = nodes_[node_index];
    segment_.clear();

    last_node_id_ = node.node_id;
    last_node_sequence_id_ = node.sequence_id;
    current_index_ = std::max(current_index_, node_index + 1);

    if (node.action && get_action_status(node.action->action_id, ActionStatus::WAITING) != ActionStatus::FINISHED) {
        acting_ = node.action;
        acting_elapsed_ = 0.0;
        set_action_status(node.action->action_id, ActionStatus::RUNNING);
        if (node.action->action_type == ActionType::CHARGE) {
            set_base(RobotState::CHARGING);
            battery.charging = true;
        } else {
            set_busy(BusySubState::ACTING);
        }
        log::info("[%s] action started: %s (%s, %.1fs)", robot_id.c_str(), node.action->action_id.c_str(),
                  to_string(node.action->action_type).c_str(), node.action->duration);
        return;
    }

    replan();
    if (segment_.empty() && base_state_ != RobotState::ERROR) {
        if (current_index_ < (int)nodes_.size()) set_busy(BusySubState::WAITING);
        else set_base(RobotState::IDLE);
    }
}

void OrderExecutor::clear_order() {
    order_id_.clear();
    order_update_id_ = 0;
    nodes_.clear();
    current_index_ = 0;
    segment_.clear();
    segment_wp_bounds_.clear();
    segment_wp_total_ = 0;
    acting_ = std::nullopt;
    acting_elapsed_ = 0.0;
    action_states_.clear();
    kinematics.set_path({});
    battery.charging = false;
    since_dynamic_replan_ = 0.0;
    set_base(RobotState::IDLE);
}

bool OrderExecutor::force_relocalize(double x, double y, double theta) {
    clear_order();
    kinematics.emergency_stop();
    kinematics.set_pose(x, y, &theta);
    log::warning("[%s] force relocalized: (%.2f, %.2f, %.2f)", robot_id.c_str(), x, y, theta);
    return true;
}

State OrderExecutor::build_state(long long header_id, double timestamp) const {
    auto k = kinematics.get_state();
    State s;
    s.header_id = header_id;
    s.timestamp = timestamp;
    s.robot_id = robot_id;
    s.state = state();
    s.sub_state = (s.state == RobotState::BUSY) ? sub_state_ : std::nullopt;
    s.mode = mode_;
    s.paused = paused_;
    s.pause_source = pause_source_;
    s.pose = Pose{std::round(k.x * 10000) / 10000, std::round(k.y * 10000) / 10000,
                  std::round(k.theta * 10000) / 10000};
    s.velocity = std::round(k.velocity * 10000) / 10000;
    s.width = robot_width;
    s.length = robot_length;
    for (auto& [x, y] : k.local_path)
        s.local_path.push_back(Point{std::round(x * 10000) / 10000, std::round(y * 10000) / 10000});
    if (!current_map_uid_.empty()) {
        s.map_id = current_map_uid_;
        s.map_version = current_map_version_;
    }
    s.last_node_id = last_node_id_;
    s.last_node_sequence_id = last_node_sequence_id_;
    s.order_id = order_id_.empty() ? std::nullopt : std::optional(order_id_);
    s.order_update_id = order_update_id_;
    for (int i = current_index_; i < (int)nodes_.size(); ++i) {
        auto& n = nodes_[i];
        s.node_states.push_back(NodeProgress{n.node_id, n.sequence_id, n.released});
    }
    for (auto& [aid, st] : action_states_) s.action_states.push_back(ActionProgress{aid, st});
    s.battery = battery.info();
    s.errors = errors_;
    for (auto& o : last_detected_)
        s.detected_obstacles.push_back(ObstacleInfo{std::round(o.x * 10000) / 10000,
                                                      std::round(o.y * 10000) / 10000, o.width, o.height});
    return s;
}

}  // namespace fms
