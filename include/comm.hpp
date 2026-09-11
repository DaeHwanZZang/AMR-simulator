// 로봇 MQTT 통신 (robot_client/comm.py 포팅). libmosquitto 사용.
#pragma once

#include <mosquitto.h>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

#include "schemas.hpp"

namespace fms {

using Incoming = std::variant<Order, InstantAction>;

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
    void publish_online();
    long long next_header_id() { return ++header_id_; }

    std::vector<Incoming> drain();

    std::string robot_id() const { return robot_id_; }

private:
    std::string robot_id_, host_;
    int port_, keepalive_;
    long long header_id_ = 0;
    std::atomic<bool> connected_{false};
    mosquitto* mosq_ = nullptr;

    std::mutex mutex_;
    std::deque<Incoming> inbox_;

    void build_client();
    std::string connection_payload(ConnectionState state);

    static void on_connect(mosquitto*, void* userdata, int rc);
    static void on_disconnect(mosquitto*, void* userdata, int rc);
    static void on_message(mosquitto*, void* userdata, const mosquitto_message* msg);
};

}  // namespace fms
