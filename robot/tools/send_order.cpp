// tools/send_order.py 포팅. 임시 FMS 대역 -- 오더/즉시명령 발행.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

#include "mqtt_once.hpp"
#include "schemas.hpp"

using namespace fms;
using namespace fms::tools;

namespace {

std::vector<Pose> parse_path(const std::string& text) {
    // "x1,y1[,theta1] x2,y2[,theta2] ..." 형식. 콤마=좌표 구분, 공백=노드 구분. theta 는 라디안 (기본 0)
    std::vector<Pose> pts;
    std::stringstream ss(text);
    std::string tok;
    while (ss >> tok) {
        std::vector<double> v;
        std::stringstream ts(tok);
        std::string part;
        while (std::getline(ts, part, ',')) v.push_back(std::stod(part));
        if (v.size() != 2 && v.size() != 3) throw std::runtime_error("--path format is X,Y[,THETA]");
        pts.push_back(Pose{v[0], v[1], v.size() == 3 ? v[2] : 0.0});
    }
    return pts;
}

// --param KEY=VALUE. 숫자로 읽히면 number, true/false 면 bool, 나머지는 문자열
void add_param(Params& params, const std::string& kv) {
    auto eq = kv.find('=');
    if (eq == std::string::npos || eq == 0) throw std::runtime_error("--param format is KEY=VALUE");
    std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
    if (v == "true" || v == "false") { params[k] = (v == "true"); return; }
    try {
        size_t n;
        double d = std::stod(v, &n);
        if (n == v.size()) { params[k] = d; return; }
    } catch (...) {}
    params[k] = v;
}


// =============================================================================
// --script: 텍스트 스크립트 -> OrderNode 목록 (한 Order 로 묶인다)
// =============================================================================
// 한 줄에 명령 하나. "//" 와 "#" 뒤는 주석. 줄 앞 번호("0 START") 는 있어도 무시한다.
//   START / END                         선택. END 뒤에 명령이 오면 오류
//   GOTO 줄번호 횟수                    그 번호의 줄로 횟수만큼 점프 (노드로 펼쳐서 한 Order 로 보낸다)
//   MOVE x y [theta]                    노드 하나. theta 는 rad, "90deg" 도 가능
//   PICK|DROP|CHARGE|WAIT [초] [key=value ...]
//   STRAIGHT|DOCK|PARK key=value ...    직전 노드에 붙는 액션. 이미 액션이 있으면 같은 좌표 노드를 복제한다

[[noreturn]] void script_error(int line, const std::string& msg) {
    throw std::runtime_error("script line " + std::to_string(line) + ": " + msg);
}

double parse_number(const std::string& s, int line, const char* what) {
    try {
        size_t n;
        double d = std::stod(s, &n);
        if (n == s.size()) return d;
    } catch (...) {}
    script_error(line, std::string(what) + " 가 숫자가 아니다: '" + s + "'");
}

// "2.3" 또는 "90deg" -> rad
double parse_angle(const std::string& s, int line) {
    if (s.size() > 3 && s.compare(s.size() - 3, 3, "deg") == 0)
        return parse_number(s.substr(0, s.size() - 3), line, "theta") * M_PI / 180.0;
    return parse_number(s, line, "theta");
}

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}

bool all_digits(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

struct Stmt {
    int line_no;
    int label = -1;                 // 줄 앞 번호 (GOTO 대상). 없으면 -1
    std::vector<std::string> tok;   // tok[0] = 명령
};

constexpr size_t MAX_NODES = 5000;       // GOTO 로 펼친 노드 수 상한
constexpr size_t MAX_STEPS = 200000;     // 실행한 문장 수 상한 (안전장치)

std::vector<OrderNode> parse_script(const std::string& file, const std::string& order_id) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("script 파일을 열 수 없다: " + file);

    // --- 1) 줄 -> 문장. 문법 검사 가능한 것(START/END 위치, 라벨 중복)은 여기서 한다
    std::vector<Stmt> prog;
    std::string text;
    int line_no = 0;
    bool ended = false;
    while (std::getline(in, text)) {
        ++line_no;
        for (const char* c : {"//", "#"}) {
            auto pos = text.find(c);
            if (pos != std::string::npos) text.erase(pos);
        }
        std::stringstream ss(text);
        Stmt st;
        st.line_no = line_no;
        for (std::string w; ss >> w;) st.tok.push_back(w);
        if (!st.tok.empty() && all_digits(st.tok[0])) {  // 줄 번호 = GOTO 라벨
            st.label = std::stoi(st.tok[0]);
            st.tok.erase(st.tok.begin());
            for (auto& o : prog)
                if (o.label == st.label) script_error(line_no, "줄 번호 " + std::to_string(st.label) + " 가 " + std::to_string(o.line_no) + " 번째 줄과 중복된다");
        }
        if (st.tok.empty()) continue;
        std::string cmd = upper(st.tok[0]);
        if (ended) script_error(line_no, "END 뒤에 명령이 있다: " + st.tok[0]);
        if (cmd == "END") ended = true;
        if (cmd == "START" && !prog.empty()) script_error(line_no, "START 는 첫 명령이어야 한다");
        prog.push_back(st);
    }

    // --- 2) 실행 (GOTO 는 여기서 펼친다). GOTO n N = n 번 줄로 N 번 점프 (총 N+1 번 실행)
    std::vector<OrderNode> nodes;
    std::map<size_t, int> remaining;  // GOTO 문장 인덱스 -> 남은 점프 수 (소진되면 지워서 바깥 루프가 다시 돌 때 새로 시작)
    size_t steps = 0;
    for (size_t pc = 0; pc < prog.size();) {
        if (++steps > MAX_STEPS) script_error(prog[pc].line_no, "실행이 너무 길다 (GOTO 반복 확인)");
        const Stmt& st = prog[pc];
        const auto& tok = st.tok;
        std::string cmd = upper(tok[0]);
        int ln = st.line_no;
        ++pc;

        if (cmd == "START") {
        } else if (cmd == "END") {
            break;
        } else if (cmd == "GOTO") {
            if (tok.size() != 3) script_error(ln, "GOTO 줄번호 횟수  (예: GOTO 1 50 = 1번 줄로 50번 점프. 횟수 없는 무한 반복은 지원하지 않는다)");
            if (!all_digits(tok[1])) script_error(ln, "GOTO 대상이 줄 번호가 아니다: '" + tok[1] + "'");
            if (!all_digits(tok[2])) script_error(ln, "GOTO 횟수는 0 이상의 정수여야 한다: '" + tok[2] + "'");
            int target = std::stoi(tok[1]);
            auto dest = std::find_if(prog.begin(), prog.end(), [&](const Stmt& s) { return s.label == target; });
            if (dest == prog.end()) script_error(ln, "줄 번호 " + tok[1] + " 이(가) 없다");
            size_t me = pc - 1;
            auto it = remaining.find(me);
            if (it == remaining.end()) it = remaining.emplace(me, std::stoi(tok[2])).first;
            if (it->second > 0) {
                --it->second;
                pc = (size_t)(dest - prog.begin());
            } else {
                remaining.erase(it);  // 소진: 다음으로 진행
            }
        } else if (cmd == "MOVE") {
            if (tok.size() != 3 && tok.size() != 4) script_error(ln, "MOVE x y [theta]");
            double th = tok.size() == 4 ? parse_angle(tok[3], ln) : 0.0;
            OrderNode n;
            n.position = Pose{parse_number(tok[1], ln, "x"), parse_number(tok[2], ln, "y"), std::atan2(std::sin(th), std::cos(th))};
            nodes.push_back(n);
        } else {
            ActionType type;
            try { type = action_type_from(cmd); }
            catch (...) { script_error(ln, "알 수 없는 명령: " + tok[0] + " (MOVE|PICK|DROP|CHARGE|WAIT|STRAIGHT|DOCK|PARK|GOTO|START|END)"); }
            if (nodes.empty()) script_error(ln, cmd + " 앞에 MOVE 가 있어야 한다 (액션은 직전 MOVE 위치에서 실행된다)");
            if (nodes.back().action) {  // 노드당 액션 1개: 같은 좌표 노드를 복제한다
                OrderNode dup;
                dup.position = nodes.back().position;
                nodes.push_back(dup);
            }
            Action a;
            a.action_type = type;
            a.action_id = order_id + "-A" + std::to_string(nodes.size() - 1);
            bool positional_ok = type == ActionType::PICK || type == ActionType::DROP || type == ActionType::CHARGE || type == ActionType::WAIT;
            for (size_t i = 1; i < tok.size(); ++i) {
                if (tok[i].find('=') == std::string::npos) {
                    if (!positional_ok || i != 1) script_error(ln, "'" + tok[i] + "': key=value 형식이어야 한다");
                    a.duration = parse_number(tok[i], ln, "duration");
                    continue;
                }
                Params one;
                try { add_param(one, tok[i]); } catch (const std::exception& e) { script_error(ln, e.what()); }
                for (auto& [k, v] : one) {
                    if (k == "duration") a.duration = parse_number(tok[i].substr(tok[i].find('=') + 1), ln, "duration");
                    else if (k == "payload_id") a.payload_id = tok[i].substr(tok[i].find('=') + 1);
                    else a.params[k] = v;
                }
            }
            // 로봇과 같은 검증을 미리 돌린다 (잘못된 params 면 로봇이 오더를 통째로 버리므로)
            cJSON* aj = a.to_json();
            try { Action::from_json(aj); } catch (const ValidationError& e) {
                cJSON_Delete(aj);
                script_error(ln, std::string("잘못된 액션: ") + e.what());
            }
            cJSON_Delete(aj);
            nodes.back().action = a;
        }
        if (nodes.size() > MAX_NODES) script_error(ln, "노드가 " + std::to_string(MAX_NODES) + "개를 넘는다 (GOTO 횟수 확인)");
    }
    if (nodes.empty()) throw std::runtime_error("script 에 MOVE 가 하나도 없다: " + file);
    return nodes;
}

void usage() {
    std::cout <<
        "Usage: send_order ROBOT_ID --path X,Y[,THETA] [X2,Y2[,THETA2] ...] --order-id ID [--update N] "
        "[--host H] [--port P]\n"
        "          [--action PICK|DROP|CHARGE|WAIT|STRAIGHT|DOCK|PARK [--duration S] [--param KEY=VALUE ...]]\n"
        "        (--action 은 마지막 노드에 붙는다. 예: --action DOCK --param entry=BACKWARD\n"
        "         --param undock_distance=0.6 --param min_charge=80 / --action STRAIGHT --param distance=-1.0)\n"
        "        send_order ROBOT_ID --script FILE [--order-id ID] [--dry-run]   (텍스트 스크립트를 Order 하나로 묶어 발행.\n"
        "         MOVE x y [theta] / PICK|DROP|CHARGE|WAIT [초] / STRAIGHT|DOCK|PARK key=value / GOTO 줄번호 횟수 / START / END, 주석 // #)\n"
        "        send_order ROBOT_ID --instant CANCEL_ORDER|PAUSE|RESUME|RESET_ERROR|INJECT_FAULT [--fault NAME]\n"
        "        send_order ROBOT_ID --instant INIT_POSITION --param x=X --param y=Y [--param theta=RAD]  (위치 힌트)\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 1; }
    std::string robot_id = argv[1];
    std::string host = "localhost";
    int port = 1883;
    std::string path_text, order_id = "O1", instant_text, fault = "UNKNOWN_FAULT";
    int update_id = 0;
    std::string action_text, script_file;
    bool order_id_set = false, dry_run = false;
    double duration = 0.0;
    Params params;

    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() { return std::string(argv[++i]); };
        if (k == "--host") host = next();
        else if (k == "--port") port = std::stoi(next());
        else if (k == "--path") {
            path_text.clear();
            while (i + 1 < argc && argv[i + 1][0] != '-') path_text += std::string(" ") + next();
        } else if (k == "--order-id") { order_id = next(); order_id_set = true; }
        else if (k == "--script") script_file = next();
        else if (k == "--dry-run") dry_run = true;
        else if (k == "--update") update_id = std::stoi(next());
        else if (k == "--instant") instant_text = next();
        else if (k == "--fault") fault = next();
        else if (k == "--action") action_text = next();
        else if (k == "--duration") duration = std::stod(next());
        else if (k == "--param") add_param(params, next());
        else { usage(); return 1; }
    }

    std::unique_ptr<OneShotPublisher> pub_ptr;
    auto connect = [&]() -> OneShotPublisher& {
        pub_ptr = std::make_unique<OneShotPublisher>(host, port);
        if (!pub_ptr->ok()) { std::cerr << "broker connection failed (" << host << ":" << port << ")\n"; std::exit(1); }
        return *pub_ptr;
    };

    if (!instant_text.empty()) {
        OneShotPublisher& pub = connect();
        InstantAction a;
        a.header_id = 1;
        a.timestamp = now_epoch();
        a.robot_id = robot_id;
        a.action_id = "IA-" + std::to_string((long long)(now_epoch() * 1000));
        a.action_type = instant_action_type_from(instant_text);
        if (a.action_type == InstantActionType::INJECT_FAULT) a.params["fault"] = fault;
        for (auto& [k, v] : params) a.params[k] = v;  // --param KEY=VALUE (예: INIT_POSITION 의 x, y, theta)
        pub.publish(topic_instant(robot_id), a.to_json_string());
        std::cout << "published -> " << topic_instant(robot_id) << "  " << instant_text << "\n";
        return 0;
    }

    if (!script_file.empty() && !path_text.empty()) { std::cerr << "--script 와 --path 는 같이 쓸 수 없다\n"; return 1; }
    if (path_text.empty() && script_file.empty()) { usage(); return 1; }
    Order order;
    order.header_id = 1;
    order.timestamp = now_epoch();
    order.robot_id = robot_id;
    order.order_update_id = update_id;
    std::string summary;
    if (!script_file.empty()) {
        if (!order_id_set) order_id = std::filesystem::path(script_file).stem().string();
        order.order_id = order_id;
        try { order.nodes = parse_script(script_file, order_id); }
        catch (const std::exception& e) { std::cerr << e.what() << "\n"; return 1; }
        for (size_t i = 0; i < order.nodes.size(); ++i) {
            order.nodes[i].node_id = "N" + std::to_string(i);
            order.nodes[i].sequence_id = (int)i;
            order.nodes[i].released = true;
        }
        summary = "script " + script_file;
    } else {
        auto pts = parse_path(path_text);
        order.order_id = order_id;
        for (size_t i = 0; i < pts.size(); ++i) {
            OrderNode n;
            n.node_id = "N" + std::to_string(i);
            n.sequence_id = (int)i;
            n.released = true;
            n.position = pts[i];
            order.nodes.push_back(n);
        }
        if (!action_text.empty()) {
            Action a;
            a.action_id = order_id + "-A" + std::to_string(pts.size() - 1);
            a.action_type = action_type_from(action_text);
            a.duration = duration;
            a.params = params;
            // 로봇과 같은 검증을 미리 돌린다 (잘못된 params 면 로봇이 오더를 통째로 버리므로)
            cJSON* aj = a.to_json();
            try { Action::from_json(aj); } catch (const ValidationError& e) {
                cJSON_Delete(aj);
                std::cerr << "invalid action: " << e.what() << "\n";
                return 1;
            }
            cJSON_Delete(aj);
            order.nodes.back().action = a;
        }
        summary = action_text.empty() ? "" : "last node action " + action_text;
    }

    // Order 에는 to_json_string 이 없다 (로봇은 만들 일이 없어서) -- 여기서만 직접 만든다
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "version", PROTOCOL_VERSION);
    cJSON_AddNumberToObject(root, "header_id", (double)order.header_id);
    cJSON_AddNumberToObject(root, "timestamp", order.timestamp);
    cJSON_AddStringToObject(root, "robot_id", order.robot_id.c_str());
    cJSON_AddStringToObject(root, "order_id", order.order_id.c_str());
    cJSON_AddNumberToObject(root, "order_update_id", order.order_update_id);
    cJSON_AddNullToObject(root, "task_id");
    cJSON* nodes = cJSON_CreateArray();
    for (auto& n : order.nodes) {
        cJSON* nd = cJSON_CreateObject();
        cJSON_AddStringToObject(nd, "node_id", n.node_id.c_str());
        cJSON_AddNumberToObject(nd, "sequence_id", n.sequence_id);
        cJSON_AddBoolToObject(nd, "released", n.released);
        cJSON_AddItemToObject(nd, "position", n.position.to_json());
        if (n.action) cJSON_AddItemToObject(nd, "action", n.action->to_json());
        else cJSON_AddNullToObject(nd, "action");
        cJSON_AddNullToObject(nd, "reserved_window");
        cJSON_AddItemToArray(nodes, nd);
    }
    cJSON_AddItemToObject(root, "nodes", nodes);
    char* txt = cJSON_PrintUnformatted(root);
    std::string payload(txt);
    free(txt);
    cJSON_Delete(root);

    if (dry_run) {
        std::cout << "dry-run (발행 안 함) order_id=" << order.order_id << " update_id=" << update_id << "\n";
        for (auto& n : order.nodes) {
            std::cout << "  " << n.node_id << "  (" << n.position.x << ", " << n.position.y << ", " << n.position.theta << ")";
            if (n.action) {
                std::cout << "  " << to_string(n.action->action_type);
                if (n.action->duration > 0) std::cout << " " << n.action->duration << "s";
                std::cout << "  [" << n.action->action_id << "]";
            }
            std::cout << "\n";
        }
        return 0;
    }
    connect().publish(topic_order(robot_id), payload);
    std::cout << "published -> " << topic_order(robot_id) << "\n  order_id=" << order.order_id
              << " update_id=" << update_id << "\n  " << order.nodes.size() << " node(s)"
              << (summary.empty() ? "" : ", " + summary) << "\n";
    return 0;
}
