#include "comm.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include "log.hpp"

namespace fms {

RobotComm::RobotComm(std::string robot_id, std::string host, int port, int keepalive)
    : robot_id_(std::move(robot_id)), host_(std::move(host)), port_(port), keepalive_(keepalive) {
    static bool lib_inited = false;
    if (!lib_inited) { mosquitto_lib_init(); lib_inited = true; }
    build_client();
}

RobotComm::~RobotComm() {
    if (mosq_) mosquitto_destroy(mosq_);
}

std::string RobotComm::connection_payload(ConnectionState state) {
    Connection c;
    c.header_id = next_header_id();
    c.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    c.robot_id = robot_id_;
    c.connection_state = state;
    return c.to_json_string();
}

void RobotComm::build_client() {
    std::string client_id = "robot-" + robot_id_;
    mosq_ = mosquitto_new(client_id.c_str(), true, this);
    mosquitto_connect_callback_set(mosq_, &RobotComm::on_connect);
    mosquitto_disconnect_callback_set(mosq_, &RobotComm::on_disconnect);
    mosquitto_message_callback_set(mosq_, &RobotComm::on_message);

    std::string will = connection_payload(ConnectionState::CONNECTION_BROKEN);
    mosquitto_will_set(mosq_, topic_connection(robot_id_).c_str(), (int)will.size(), will.c_str(), 1, true);
}

bool RobotComm::connect() {
    int rc = mosquitto_connect(mosq_, host_.c_str(), port_, keepalive_);
    if (rc != MOSQ_ERR_SUCCESS) return false;
    mosquitto_loop_start(mosq_);
    return true;
}

bool RobotComm::wait_until_connected(double timeout_sec) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        if (connected_) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

void RobotComm::disconnect() {
    std::string payload = connection_payload(ConnectionState::OFFLINE);
    mosquitto_publish(mosq_, nullptr, topic_connection(robot_id_).c_str(), (int)payload.size(),
                       payload.c_str(), 1, true);
    // 다른 로봇에게 "이 몸체는 사라졌다" 를 알린다 (빈 physics). 비정상 종료면 2 초 뒤 소식이 끊긴 것으로 지워진다.
    mosquitto_publish(mosq_, nullptr, topic_physics(robot_id_).c_str(), 0, nullptr, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // wait_for_publish 근사
    mosquitto_loop_stop(mosq_, false);
    mosquitto_disconnect(mosq_);
}

void RobotComm::rebind_id(const std::string& new_robot_id) {
    if (new_robot_id == robot_id_) return;
    std::string old_id = robot_id_;
    std::string payload = connection_payload(ConnectionState::OFFLINE);
    mosquitto_publish(mosq_, nullptr, topic_connection(old_id).c_str(), (int)payload.size(), payload.c_str(),
                       1, true);
    // 이전 ID 의 retained factsheet 를 지운다 (새 ID 로는 접속할 때 다시 낸다).
    mosquitto_publish(mosq_, nullptr, topic_factsheet(old_id).c_str(), 0, nullptr, 1, true);
    mosquitto_publish(mosq_, nullptr, topic_physics(old_id).c_str(), 0, nullptr, 1, false);  // 이전 ID 의 몸체를 다른 로봇에서 지운다
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mosquitto_loop_stop(mosq_, false);
    mosquitto_disconnect(mosq_);
    mosquitto_destroy(mosq_);

    robot_id_ = new_robot_id;
    build_client();
    connect();
    log::info("[%s] ID change complete (was: %s)", new_robot_id.c_str(), old_id.c_str());
}

void RobotComm::set_factsheet(const Factsheet& factsheet) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        factsheet_ = factsheet;
    }
    if (connected_) publish_factsheet();
}

void RobotComm::publish_factsheet() {
    Factsheet sheet;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!factsheet_) return;
        sheet = *factsheet_;
    }
    sheet.robot_id = robot_id_;
    sheet.header_id = next_header_id();
    sheet.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::string payload = sheet.to_json_string();
    mosquitto_publish(mosq_, nullptr, topic_factsheet(robot_id_).c_str(), (int)payload.size(), payload.c_str(), 1,
                       true);
}

void RobotComm::publish_state(const State& state) {
    std::string payload = state.to_json_string();
    mosquitto_publish(mosq_, nullptr, topic_state(robot_id_).c_str(), (int)payload.size(), payload.c_str(), 0,
                       false);
}

void RobotComm::publish_physics(const PhysicsDebug& debug) {
    std::string payload = debug.to_json_string();
    mosquitto_publish(mosq_, nullptr, topic_physics(robot_id_).c_str(), (int)payload.size(), payload.c_str(), 0,
                       false);
}

void RobotComm::publish_navigation(const NavigationDebug& nav) {
    std::string payload = nav.to_json_string();
    mosquitto_publish(mosq_, nullptr, topic_sim_navigation(robot_id_).c_str(), (int)payload.size(), payload.c_str(), 1,
                       true);
}

void RobotComm::publish_online() {
    std::string payload = connection_payload(ConnectionState::ONLINE);
    mosquitto_publish(mosq_, nullptr, topic_connection(robot_id_).c_str(), (int)payload.size(),
                       payload.c_str(), 1, true);
}

void RobotComm::on_world_obstacle(const std::string& obstacle_id, const std::string& payload) {
    if (obstacle_id.empty()) return;
    if (payload.empty()) {  // retained 삭제
        std::lock_guard<std::mutex> lock(mutex_);
        world_.erase(obstacle_id);
        return;
    }
    cJSON* j = cJSON_Parse(payload.c_str());
    if (!j) {
        log::error("[%s] ignoring invalid world obstacle %s", robot_id_.c_str(), obstacle_id.c_str());
        return;
    }
    cJSON* x = cJSON_GetObjectItemCaseSensitive(j, "x");
    cJSON* y = cJSON_GetObjectItemCaseSensitive(j, "y");
    cJSON* w = cJSON_GetObjectItemCaseSensitive(j, "width");
    cJSON* h = cJSON_GetObjectItemCaseSensitive(j, "height");
    cJSON* mid = cJSON_GetObjectItemCaseSensitive(j, "map_id");
    if (cJSON_IsNumber(x) && cJSON_IsNumber(y)) {
        WorldObstacle o;
        o.id = obstacle_id;
        o.x = x->valuedouble;
        o.y = y->valuedouble;
        if (cJSON_IsNumber(w) && w->valuedouble > 0) o.width = w->valuedouble;
        if (cJSON_IsNumber(h) && h->valuedouble > 0) o.height = h->valuedouble;
        if (cJSON_IsString(mid)) o.map_id = mid->valuestring;
        std::lock_guard<std::mutex> lock(mutex_);
        world_[obstacle_id] = std::move(o);
    } else {
        log::error("[%s] ignoring world obstacle %s: x/y required", robot_id_.c_str(), obstacle_id.c_str());
    }
    cJSON_Delete(j);
}

void RobotComm::on_peer_physics(const std::string& peer_id, const std::string& payload) {
    if (peer_id == robot_id_) return;  // 자기 자신의 physics 도 같은 구독으로 들어온다
    if (payload.empty()) {             // 그 로봇이 정상 종료했다 (disconnect 가 빈 physics 를 낸다): 몸체를 바로 지운다
        std::lock_guard<std::mutex> lock(mutex_);
        peers_.erase(peer_id);
        return;
    }
    cJSON* j = cJSON_Parse(payload.c_str());
    if (!j) return;
    cJSON* x = cJSON_GetObjectItemCaseSensitive(j, "x");
    cJSON* y = cJSON_GetObjectItemCaseSensitive(j, "y");
    cJSON* th = cJSON_GetObjectItemCaseSensitive(j, "theta");
    cJSON* w = cJSON_GetObjectItemCaseSensitive(j, "width");
    cJSON* l = cJSON_GetObjectItemCaseSensitive(j, "length");
    cJSON* mid = cJSON_GetObjectItemCaseSensitive(j, "map_id");
    if (cJSON_IsNumber(x) && cJSON_IsNumber(y)) {
        PeerEntry e;
        e.robot.id = peer_id;
        e.robot.x = x->valuedouble;
        e.robot.y = y->valuedouble;
        if (cJSON_IsNumber(th)) e.robot.theta = th->valuedouble;
        if (cJSON_IsNumber(w) && w->valuedouble > 0) e.robot.width = w->valuedouble;
        if (cJSON_IsNumber(l) && l->valuedouble > 0) e.robot.length = l->valuedouble;
        if (cJSON_IsString(mid)) e.robot.map_id = mid->valuestring;
        e.seen = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        peers_[peer_id] = std::move(e);
    }
    cJSON_Delete(j);
}

std::vector<WorldObstacle> RobotComm::snapshot_world() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<WorldObstacle> out;
    out.reserve(world_.size());
    for (auto& [id, o] : world_) out.push_back(o);
    return out;
}

std::vector<PeerRobot> RobotComm::snapshot_peers(double max_age_sec) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PeerRobot> out;
    for (auto it = peers_.begin(); it != peers_.end();) {
        // 한동안 소식이 없는 로봇은 꺼진 것으로 보고 지운다 (몸체가 세계에서 사라진다)
        if (std::chrono::duration<double>(now - it->second.seen).count() > max_age_sec) { it = peers_.erase(it); continue; }
        if (it->first != robot_id_) out.push_back(it->second.robot);
        ++it;
    }
    return out;
}

std::vector<Incoming> RobotComm::drain() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Incoming> items(inbox_.begin(), inbox_.end());
    inbox_.clear();
    return items;
}

void RobotComm::on_connect(mosquitto*, void* userdata, int rc) {
    auto* self = static_cast<RobotComm*>(userdata);
    if (rc != 0) {
        log::error("[%s] connection failed: %d", self->robot_id_.c_str(), rc);
        return;
    }
    self->connected_ = true;
    std::string ot = topic_order(self->robot_id_), it = topic_instant(self->robot_id_);
    mosquitto_subscribe(self->mosq_, nullptr, ot.c_str(), 1);
    mosquitto_subscribe(self->mosq_, nullptr, it.c_str(), 1);
    std::string tt = topic_sim_teleport(self->robot_id_);
    mosquitto_subscribe(self->mosq_, nullptr, tt.c_str(), 1);
    {
        // 끊겨 있던 동안 지워진 월드 장애물을 놓치지 않게 표를 비운다 (retained 가 다시 온다)
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->world_.clear();
    }
    if (self->sub_world_) mosquitto_subscribe(self->mosq_, nullptr, TOPIC_SIM_WORLD_OBSTACLES_ALL, 1);
    if (self->sub_peers_) mosquitto_subscribe(self->mosq_, nullptr, TOPIC_PHYSICS_ALL, 0);
    self->publish_online();
    self->publish_factsheet();
    log::info("[%s] connected to broker %s:%d", self->robot_id_.c_str(), self->host_.c_str(), self->port_);
}

void RobotComm::on_disconnect(mosquitto*, void* userdata, int rc) {
    auto* self = static_cast<RobotComm*>(userdata);
    self->connected_ = false;
    if (rc != 0) log::warning("[%s] disconnected: %d (retrying)", self->robot_id_.c_str(), rc);
}

void RobotComm::on_message(mosquitto*, void* userdata, const mosquitto_message* msg) {
    auto* self = static_cast<RobotComm*>(userdata);
    const std::string topic = msg->topic;
    const std::string payload = msg->payload && msg->payloadlen > 0
                                    ? std::string((const char*)msg->payload, (size_t)msg->payloadlen) : std::string();
    // 공유 세계 토픽. 월드 장애물은 빈 페이로드가 "삭제" 라서 아래의 빈 메시지 무시보다 먼저 본다.
    static const std::string world_prefix = TOPIC_SIM_WORLD_OBSTACLES_PREFIX;
    static const std::string fms_prefix = std::string(TOPIC_PREFIX) + "/", physics_suffix = "/physics";
    if (topic.compare(0, world_prefix.size(), world_prefix) == 0) {
        self->on_world_obstacle(topic.substr(world_prefix.size()), payload);
        return;
    }
    if (topic.size() > fms_prefix.size() + physics_suffix.size() && topic.compare(0, fms_prefix.size(), fms_prefix) == 0
            && topic.compare(topic.size() - physics_suffix.size(), physics_suffix.size(), physics_suffix) == 0) {
        self->on_peer_physics(topic.substr(fms_prefix.size(), topic.size() - fms_prefix.size() - physics_suffix.size()), payload);
        return;
    }
    if (payload.empty()) return;
    try {
        if (topic == topic_order(self->robot_id_)) {
            Order o = Order::from_json_string(payload);
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->inbox_.push_back(std::move(o));
        } else if (topic == topic_instant(self->robot_id_)) {
            InstantAction a = InstantAction::from_json_string(payload);
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->inbox_.push_back(std::move(a));
        } else if (topic == topic_sim_teleport(self->robot_id_)) {
            cJSON* j = cJSON_Parse(payload.c_str());
            if (!j) throw ValidationError("teleport: invalid JSON");
            cJSON* x = cJSON_GetObjectItemCaseSensitive(j, "x");
            cJSON* y = cJSON_GetObjectItemCaseSensitive(j, "y");
            cJSON* th = cJSON_GetObjectItemCaseSensitive(j, "theta");
            cJSON* mid = cJSON_GetObjectItemCaseSensitive(j, "map_id");
            if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) {
                cJSON_Delete(j);
                throw ValidationError("teleport: x/y required");
            }
            SimTeleport t;
            t.x = x->valuedouble;
            t.y = y->valuedouble;
            t.theta = cJSON_IsNumber(th) ? th->valuedouble : 0.0;
            if (cJSON_IsString(mid)) t.map_id = mid->valuestring;
            cJSON_Delete(j);
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->inbox_.push_back(std::move(t));
        } else {
            log::debug("[%s] unsubscribed topic: %s", self->robot_id_.c_str(), topic.c_str());
        }
    } catch (const ValidationError& e) {
        log::error("[%s] ignoring invalid message (%s): %s", self->robot_id_.c_str(), topic.c_str(), e.what());
    }
}

}  // namespace fms
