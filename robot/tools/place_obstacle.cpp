// tools/place_obstacle.py 포팅. 동적 장애물 수동 배치/제거 (INJECT_OBSTACLE/CLEAR_OBSTACLE).
#include <cstdio>
#include <iostream>

#include "mqtt_once.hpp"
#include "schemas.hpp"

using namespace fms;
using namespace fms::tools;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: place_obstacle ROBOT_ID --x X --y Y [--width W] [--height H] "
                     "[--obstacle-id ID] | --clear [--obstacle-id ID]\n"
                     "       place_obstacle --world --x X --y Y [--width W] [--height H] [--obstacle-id ID] [--map-id UID]\n"
                     "       place_obstacle --world --clear --obstacle-id ID\n"
                     "  ROBOT_ID: 그 로봇에게만 보이는 장애물 (INJECT_OBSTACLE/CLEAR_OBSTACLE).\n"
                     "  --world : 모든 로봇과 뷰어가 같이 보는 월드 장애물 (retained sim/v1/world/obstacles/ID). 나중에 뜬 로봇도 받는다.\n";
        return 1;
    }
    const bool world = std::string(argv[1]) == "--world";
    std::string robot_id = world ? "" : argv[1];
    std::string map_id;
    std::string host = "localhost";
    int port = 1883;
    double x = 0, y = 0, width = 0.4, height = 0.4;
    std::string obstacle_id;
    bool do_clear = false;

    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() { return std::string(argv[++i]); };
        if (k == "--host") host = next();
        else if (k == "--port") port = std::stoi(next());
        else if (k == "--x") x = std::stod(next());
        else if (k == "--y") y = std::stod(next());
        else if (k == "--width") width = std::stod(next());
        else if (k == "--height") height = std::stod(next());
        else if (k == "--obstacle-id") obstacle_id = next();
        else if (k == "--clear") do_clear = true;
        else if (k == "--map-id") map_id = next();
    }

    OneShotPublisher pub(host, port);
    if (!pub.ok()) { std::cerr << "broker connection failed\n"; return 1; }

    if (world) {
        // 장애물 하나 = retained 토픽 하나. 빈 페이로드로 지운다 (전체 삭제는 뷰어의 Shift+U: ID 목록은 구독해야 알 수 있다)
        if (do_clear) {
            if (obstacle_id.empty()) { std::cerr << "--world --clear needs --obstacle-id ID\n"; return 1; }
            pub.publish(topic_sim_world_obstacle(obstacle_id), "", 1, true);
            std::cout << "published -> " << topic_sim_world_obstacle(obstacle_id) << "  (cleared)\n";
            return 0;
        }
        std::string oid = obstacle_id.empty() ? ("W-" + std::to_string((long long)(now_epoch() * 1000))) : obstacle_id;
        char payload[256];
        if (map_id.empty())
            std::snprintf(payload, sizeof payload, "{\"x\": %.4f, \"y\": %.4f, \"width\": %.4f, \"height\": %.4f}", x, y, width, height);
        else
            std::snprintf(payload, sizeof payload, "{\"x\": %.4f, \"y\": %.4f, \"width\": %.4f, \"height\": %.4f, \"map_id\": \"%s\"}",
                          x, y, width, height, map_id.c_str());
        pub.publish(topic_sim_world_obstacle(oid), payload, 1, true);
        std::cout << "published -> " << topic_sim_world_obstacle(oid) << "  (" << x << ", " << y << ") " << width << "x"
                  << height << "\n";
        return 0;
    }

    InstantAction a;
    a.header_id = 1;
    a.timestamp = now_epoch();
    a.robot_id = robot_id;
    a.action_id = "IA-" + std::to_string((long long)(now_epoch() * 1000));

    if (do_clear) {
        a.action_type = InstantActionType::CLEAR_OBSTACLE;
        if (!obstacle_id.empty()) a.params["obstacle_id"] = obstacle_id;
        pub.publish(topic_instant(robot_id), a.to_json_string());
        std::cout << "published -> " << topic_instant(robot_id) << "  CLEAR_OBSTACLE "
                  << (obstacle_id.empty() ? "(all)" : obstacle_id) << "\n";
        return 0;
    }

    a.action_type = InstantActionType::INJECT_OBSTACLE;
    std::string oid = obstacle_id.empty() ? ("MANUAL-" + std::to_string((long long)(now_epoch() * 1000))) : obstacle_id;
    a.params["obstacle_id"] = oid;
    a.params["x"] = x;
    a.params["y"] = y;
    a.params["width"] = width;
    a.params["height"] = height;
    pub.publish(topic_instant(robot_id), a.to_json_string());
    std::cout << "published -> " << topic_instant(robot_id) << "  INJECT_OBSTACLE (" << oid << ") ("
              << x << ", " << y << ") " << width << "x" << height << "\n";
    return 0;
}
