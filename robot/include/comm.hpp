// 로봇 MQTT 통신 (robot_client/comm.py 포팅). libmosquitto 사용.
#pragma once

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "schemas.hpp"

namespace fms {

// 시뮬 전용 순간이동 (sim/v1/{id}/teleport). 프로토콜 밖: 실제 로봇은 엘리베이터가 옮겨 준다.
// 참 자세만 옮기고 로봇은 UNLOCALIZED 가 된다 (이동한 줄 모른다).
struct SimTeleport {
    double x = 0, y = 0, theta = 0;
    std::string map_id;  // 선택. 지금 맵 uid 와 다르면 무시
};

using Incoming = std::variant<Order, InstantAction, SimTeleport>;

class RobotComm {
public:
    RobotComm(std::string robot_id, std::string host = "localhost", int port = 1883, int keepalive = 15);
    ~RobotComm();

    bool connect();  // false 면 접속 실패 (OSError 대응)
    void disconnect();
    void rebind_id(const std::string& new_robot_id);
    bool wait_until_connected(double timeout_sec = 5.0);
    bool is_connected() const { return connected_; }

    void publish_state(const State& state);
    void publish_physics(const PhysicsDebug& debug);  // 디버그 토픽 (뷰어 전용, 프로토콜 밖)
    void publish_navigation(const NavigationDebug& nav);  // sim/v1/{id}/navigation (retained, 뷰어 전용)
    void publish_online();
    // 로봇 사양. 접속(재접속)할 때마다 retained 로 다시 낸다. robot_id 는 발행할 때 채운다.
    void set_factsheet(const Factsheet& factsheet);
    long long next_header_id() { return ++header_id_; }

    std::vector<Incoming> drain();

    // 공유 세계 (시뮬 전용). 접속 전에 정한다: 월드 장애물(sim/v1/world/obstacles/+) / 다른 로봇(fms/v1/+/physics) 구독 여부.
    void enable_world(bool world_obstacles, bool peers) { sub_world_ = world_obstacles; sub_peers_ = peers; }
    // 지금 있는 월드 장애물 전부 / 최근 max_age_sec 안에 physics 를 보낸 다른 로봇 (자기 자신 제외).
    // 수신 스레드가 채우는 표의 사본이다. 메시지마다 큐에 쌓지 않는다 (physics 는 로봇마다 20 Hz).
    std::vector<WorldObstacle> snapshot_world();
    std::vector<PeerRobot> snapshot_peers(double max_age_sec = 2.0);

    std::string robot_id() const { return robot_id_; }

private:
    std::string robot_id_, host_;
    int port_, keepalive_;
    long long header_id_ = 0;
    std::atomic<bool> connected_{false};
    mosquitto* mosq_ = nullptr;

    std::mutex mutex_;
    std::deque<Incoming> inbox_;
    std::optional<Factsheet> factsheet_;
    void publish_factsheet();

    bool sub_world_ = true, sub_peers_ = true;
    struct PeerEntry {
        PeerRobot robot;
        std::chrono::steady_clock::time_point seen;
    };
    std::map<std::string, WorldObstacle> world_;  // mutex_ 로 보호
    std::map<std::string, PeerEntry> peers_;      // mutex_ 로 보호
    void on_world_obstacle(const std::string& obstacle_id, const std::string& payload);
    void on_peer_physics(const std::string& peer_id, const std::string& payload);

    void build_client();
    std::string connection_payload(ConnectionState state);

    static void on_connect(mosquitto*, void* userdata, int rc);
    static void on_disconnect(mosquitto*, void* userdata, int rc);
    static void on_message(mosquitto*, void* userdata, const mosquitto_message* msg);
};

}  // namespace fms
