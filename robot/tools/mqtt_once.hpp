// CLI 툴 공용: 한 번 접속해서 메시지 하나 발행하고 끊는다 (tools/*.py 자매품 공통 패턴).
#pragma once

#include <mosquitto.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace fms::tools {

class OneShotPublisher {
public:
    OneShotPublisher(const std::string& host, int port) {
        static bool inited = false;
        if (!inited) { mosquitto_lib_init(); inited = true; }
        std::string cid = "cli-" + std::to_string((long long)std::chrono::steady_clock::now().time_since_epoch().count());
        mosq_ = mosquitto_new(cid.c_str(), true, nullptr);
        ok_ = mosquitto_connect(mosq_, host.c_str(), port, 10) == MOSQ_ERR_SUCCESS;
        if (ok_) mosquitto_loop_start(mosq_);
    }
    ~OneShotPublisher() {
        if (mosq_) {
            mosquitto_loop_stop(mosq_, false);
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
    }
    bool ok() const { return ok_; }
    // retain = true 면 브로커가 마지막 값을 보관한다 (빈 payload + retain = 보관된 값 삭제)
    void publish(const std::string& topic, const std::string& payload, int qos = 1, bool retain = false) {
        mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)payload.size(), payload.empty() ? nullptr : payload.c_str(), qos,
                          retain);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));  // wait_for_publish 근사
    }

private:
    mosquitto* mosq_ = nullptr;
    bool ok_ = false;
};

inline double now_epoch() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace fms::tools
