// 프로토콜 메시지 파싱/검증/직렬화 (schemas.cpp).
#include <string>

#include "check.hpp"
#include "schemas.hpp"

using namespace fms;

namespace {

// nodes 자리에 넣을 노드 JSON 을 받아 Order 문자열을 만든다
std::string order_json(const std::string& nodes, const std::string& extra = "") {
    return R"({"header_id":1,"timestamp":10.5,"robot_id":"R1","order_id":"o1","order_update_id":0,)" + extra +
           R"("nodes":[)" + nodes + "]}";
}

std::string node_json(int seq, double x, double y, const std::string& action = "", bool released = true) {
    std::string s = R"({"node_id":"n)" + std::to_string(seq) + R"(","sequence_id":)" + std::to_string(seq) +
                    R"(,"released":)" + (released ? "true" : "false") + R"(,"position":{"x":)" + std::to_string(x) +
                    R"(,"y":)" + std::to_string(y) + R"(,"theta":0})";
    if (!action.empty()) s += R"(,"action":)" + action;
    return s + "}";
}

bool contains(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }

}  // namespace

TEST(schemas_order_parses) {
    const std::string straight = R"({"action_id":"a1","action_type":"STRAIGHT","params":{"distance":0.5,"speed":0.3}})";
    Order o = Order::from_json_string(order_json(node_json(0, 1, 2) + "," + node_json(1, 3, 4, straight)));
    CHECK(o.robot_id == "R1");
    CHECK(o.order_id == "o1");
    CHECK(o.nodes.size() == 2);
    CHECK_NEAR(o.nodes[1].position.x, 3.0, 1e-9);
    CHECK(o.nodes[0].released);
    CHECK(!o.nodes[0].action.has_value());
    CHECK(o.nodes[1].action.has_value());
    CHECK(o.nodes[1].action->action_type == ActionType::STRAIGHT);
    CHECK_NEAR(param_double(o.nodes[1].action->params, "distance", 0.0), 0.5, 1e-9);
}

TEST(schemas_order_rejects_bad_input) {
    CHECK_THROWS(Order::from_json_string("not json"), ValidationError);
    CHECK_THROWS(Order::from_json_string(order_json("")), ValidationError);  // 노드 없음
    CHECK_THROWS(Order::from_json_string(order_json(node_json(0, 1, 2), R"("surprise":1,)")), ValidationError);
    // sequence_id 는 오름차순 + 중복 없음
    CHECK_THROWS(Order::from_json_string(order_json(node_json(1, 1, 2) + "," + node_json(0, 3, 4))), ValidationError);
    CHECK_THROWS(Order::from_json_string(order_json(node_json(0, 1, 2) + "," + node_json(0, 3, 4))), ValidationError);
    // released 는 앞에서부터 연속이어야 한다
    CHECK_THROWS(Order::from_json_string(order_json(node_json(0, 1, 2, "", false) + "," + node_json(1, 3, 4))),
                 ValidationError);
    // theta 범위
    const std::string bad_theta =
        R"({"node_id":"n0","sequence_id":0,"released":true,"position":{"x":0,"y":0,"theta":7}})";
    CHECK_THROWS(Order::from_json_string(order_json(bad_theta)), ValidationError);
}

TEST(schemas_action_params_are_validated) {
    auto order_with = [](const std::string& action) { return order_json(node_json(0, 1, 2, action)); };
    // STRAIGHT: distance 필수, 0 불가
    CHECK_THROWS(Order::from_json_string(order_with(R"({"action_id":"a","action_type":"STRAIGHT","params":{}})")),
                 ValidationError);
    CHECK_THROWS(
        Order::from_json_string(order_with(R"({"action_id":"a","action_type":"STRAIGHT","params":{"distance":0}})")),
        ValidationError);
    // DOCK: entry, undock_distance, min_charge
    CHECK_THROWS(Order::from_json_string(order_with(
                     R"({"action_id":"a","action_type":"DOCK","params":{"entry":"SIDEWAYS","undock_distance":0.6,"min_charge":80}})")),
                 ValidationError);
    CHECK_THROWS(Order::from_json_string(order_with(
                     R"({"action_id":"a","action_type":"DOCK","params":{"entry":"BACKWARD","undock_distance":0.6,"min_charge":180}})")),
                 ValidationError);
    Order ok = Order::from_json_string(order_with(
        R"({"action_id":"a","action_type":"DOCK","params":{"entry":"BACKWARD","undock_distance":0.6,"min_charge":80}})"));
    CHECK(ok.nodes[0].action->action_type == ActionType::DOCK);
    // PARK 는 min_charge 가 없어도 된다
    Order park = Order::from_json_string(order_with(
        R"({"action_id":"a","action_type":"PARK","params":{"entry":"FORWARD","undock_distance":0.5}})"));
    CHECK(park.nodes[0].action->action_type == ActionType::PARK);
    // 모르는 액션
    CHECK_THROWS(Order::from_json_string(order_with(R"({"action_id":"a","action_type":"JUMP"})")), ValidationError);
}

TEST(schemas_instant_action_round_trip) {
    InstantAction a;
    a.header_id = 3;
    a.timestamp = 1.25;
    a.robot_id = "R1";
    a.action_id = "IA-1";
    a.action_type = InstantActionType::INJECT_OBSTACLE;
    a.params["obstacle_id"] = std::string("box");
    a.params["x"] = 4.5;
    InstantAction b = InstantAction::from_json_string(a.to_json_string());
    CHECK(b.action_type == InstantActionType::INJECT_OBSTACLE);
    CHECK(b.robot_id == "R1");
    CHECK(b.get_string("obstacle_id", "") == "box");
    CHECK_NEAR(b.get_double("x", 0.0), 4.5, 1e-9);
    CHECK(!b.has("y"));
    CHECK_NEAR(b.get_double("y", 7.0), 7.0, 1e-9);
    CHECK_THROWS(InstantAction::from_json_string(
                     R"({"header_id":1,"timestamp":1,"robot_id":"R1","action_id":"x","action_type":"NOPE"})"),
                 ValidationError);
}

TEST(schemas_state_and_debug_topics_serialize) {
    State s;
    s.robot_id = "R1";
    s.loads.push_back(LoadInfo{"T1", "box-7"});
    s.detected_obstacles.push_back(ObstacleInfo{1.0, 2.0, 0.4, 0.4});
    const std::string js = s.to_json_string();
    for (const char* key : {"\"robot_id\"", "\"pose\"", "\"local_path\"", "\"localization\"", "\"loads\"",
                            "\"detected_obstacles\"", "\"node_states\"", "\"errors\"", "\"battery\""})
        CHECK(contains(js, key));
    CHECK(contains(js, "box-7"));

    PhysicsDebug p;
    p.robot_id = "R1";
    p.x = 1.5;
    p.width = 0.5;
    p.length = 0.7;
    p.map_id = "m1";
    const std::string pj = p.to_json_string();
    for (const char* key : {"\"x\"", "\"y\"", "\"theta\"", "\"est_x\"", "\"odom_x\"", "\"width\"", "\"length\"", "\"map_id\""})
        CHECK(contains(pj, key));

    CHECK(topic_state("R1") == "fms/v1/R1/state");
    CHECK(topic_physics("R1") == "fms/v1/R1/physics");
    CHECK(topic_sim_world_obstacle("box") == "sim/v1/world/obstacles/box");
}
