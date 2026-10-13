// 로봇 클라이언트 진입점 (robot_client/main.py 포팅)
// 프로세스 1개 = 로봇 1대. --gui 는 Qt Widgets 로컬 제어판을 띄운다 (robot_client/gui.py
// 의 1:1 포팅, gui.cpp). Tk/Qt 둘 다 메인스레드에서 돌아야 하므로, --gui 일 때는
// tick/MQTT 루프를 백그라운드 스레드로 보내고 메인 스레드가 Qt 이벤트루프를 돈다
// (Python 쪽과 동일한 스레드 모델).
#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <thread>

#include "clock.hpp"
#include "comm.hpp"
#include "command_queue.hpp"
#include "executor.hpp"
#include "gui.hpp"
#include "log.hpp"
#include "map_model.hpp"
#include "map_sync.hpp"

using namespace fms;

namespace {

std::atomic<bool>* g_stop_flag = nullptr;
void handle_signal(int) { if (g_stop_flag) g_stop_flag->store(true); }

struct Args {
    std::string id;
    std::string host = "localhost";
    int port = 1883;
    double x = 0.0, y = 0.0, theta = 0.0;
    std::optional<std::string> map;
    std::string map_cache = "map_cache";
    double robot_width = 0.5, robot_length = 0.7;
    double max_velocity = 1.2, max_acceleration = 0.6;
    double battery = 100.0;
    std::optional<long long> seed;
    double speed_noise = 0.05;
    std::string log_level = "INFO";
    bool gui = false;
    bool path_refine = false;

    double sensor_range = 3.0;
    std::optional<Zone> obstacle_zone;
    std::optional<std::pair<ZoneSide, ZoneSide>> obstacle_sides;
    double obstacle_spawn_interval = 5.0;
    int obstacle_max_count = 3;
    double obstacle_width = 0.4, obstacle_height = 0.4, obstacle_speed = 0.5;
    std::optional<long long> obstacle_seed;
};

Zone parse_zone(const std::string& text) {
    std::vector<double> parts;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) parts.push_back(std::stod(item));
    if (parts.size() != 4) throw std::runtime_error("--obstacle-zone must be XMIN,YMIN,XMAX,YMAX");
    return Zone{parts[0], parts[1], parts[2], parts[3]};
}

std::pair<ZoneSide, ZoneSide> parse_sides(const std::string& text) {
    auto dash = text.find('-');
    if (dash == std::string::npos || dash == 0 || dash + 1 >= text.size())
        throw std::runtime_error("--obstacle-sides must be ENTRY-EXIT (e.g. 3-1)");
    return {zone_side_from_char(text[0]), zone_side_from_char(text[dash + 1])};
}

bool parse_args(int argc, char** argv, Args& a) {
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for: ") + argv[i]);
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if (k == "--id") a.id = next(i);
        else if (k == "--host") a.host = next(i);
        else if (k == "--port") a.port = std::stoi(next(i));
        else if (k == "--x") a.x = std::stod(next(i));
        else if (k == "--y") a.y = std::stod(next(i));
        else if (k == "--theta") a.theta = std::stod(next(i));
        else if (k == "--map") a.map = next(i);
        else if (k == "--map-cache") a.map_cache = next(i);
        else if (k == "--robot-width") a.robot_width = std::stod(next(i));
        else if (k == "--robot-length") a.robot_length = std::stod(next(i));
        else if (k == "--max-velocity") a.max_velocity = std::stod(next(i));
        else if (k == "--max-acceleration") a.max_acceleration = std::stod(next(i));
        else if (k == "--battery") a.battery = std::stod(next(i));
        else if (k == "--seed") a.seed = std::stoll(next(i));
        else if (k == "--speed-noise") a.speed_noise = std::stod(next(i));
        else if (k == "--log-level") a.log_level = next(i);
        else if (k == "--gui") a.gui = true;
        else if (k == "--path-refine") a.path_refine = true;
        else if (k == "--sensor-range") a.sensor_range = std::stod(next(i));
        else if (k == "--obstacle-zone") a.obstacle_zone = parse_zone(next(i));
        else if (k == "--obstacle-sides") a.obstacle_sides = parse_sides(next(i));
        else if (k == "--obstacle-spawn-interval") a.obstacle_spawn_interval = std::stod(next(i));
        else if (k == "--obstacle-max-count") a.obstacle_max_count = std::stoi(next(i));
        else if (k == "--obstacle-width") a.obstacle_width = std::stod(next(i));
        else if (k == "--obstacle-height") a.obstacle_height = std::stod(next(i));
        else if (k == "--obstacle-speed") a.obstacle_speed = std::stod(next(i));
        else if (k == "--obstacle-seed") a.obstacle_seed = std::stoll(next(i));
        else if (k == "-h" || k == "--help") {
            std::cout <<
                "AMR robot client (C++)\n"
                "  --id ID (required) --host H --port P --x --y --theta --map PATH\n"
                "  --map-cache DIR (SET_MAP 번들 저장 위치, 기본 map_cache)\n"
                "  --robot-width --robot-length --max-velocity --max-acceleration --battery\n"
                "  --seed N --speed-noise R --log-level LV --gui\n"
                "  --path-refine (A* 경로를 PSO 로 다듬기, 기본 꺼짐)\n"
                "  --sensor-range --obstacle-zone XMIN,YMIN,XMAX,YMAX --obstacle-sides E-X\n"
                "  --obstacle-spawn-interval --obstacle-max-count --obstacle-width --obstacle-height\n"
                "  --obstacle-speed --obstacle-seed\n";
            return false;
        } else {
            throw std::runtime_error("unknown option: " + k);
        }
    }
    if (a.id.empty()) throw std::runtime_error("--id is required");
    return true;
}

// python random.Random(f"{seed}-{id}") 와 동일한 시퀀스는 안 나온다 (PRNG 알고리즘이
// 다름). 대신 "같은 문자열 시드 -> 같은 결과" 재현성만 C++ 쪽에서 자체적으로 보장한다.
unsigned seed_from_string(const std::string& s) {
    return (unsigned)(std::hash<std::string>{}(s) & 0xffffffffu);
}

// GUI 가 큐에 넣은 명령을 tick 루프 스레드에서 실제로 실행한다 (main.py 의
// _dispatch_gui_command 포팅). exec_mutex 는 호출자가 이미 잠근 상태로 들어온다.
void dispatch_gui_command(OrderExecutor& exec, RobotComm& comm, MapSync& map_sync, const GuiCommand& cmd) {
    std::visit(
        [&](auto&& c) {
            using T = std::decay_t<decltype(c)>;
            if constexpr (std::is_same_v<T, CmdMove>) {
                static std::atomic<long long> seq{0};
                Order order;
                order.header_id = 1;
                order.robot_id = exec.robot_id;
                order.order_id = "GUI-" + std::to_string(++seq);
                order.order_update_id = 0;
                OrderNode n;
                n.node_id = "N0"; n.sequence_id = 0; n.released = true;
                n.position = Pose{c.x, c.y, 0.0};
                order.nodes.push_back(n);
                if (!exec.accept_order(order))
                    log::warning("[%s] GUI move command rejected (another order active or state mismatch)",
                                 exec.robot_id.c_str());
            } else if constexpr (std::is_same_v<T, CmdResyncMap>) {
                map_sync.retry_last();
            } else if constexpr (std::is_same_v<T, CmdRelocalize>) {
                exec.force_relocalize(c.x, c.y, c.theta);
            } else if constexpr (std::is_same_v<T, CmdPressStart>) {
                exec.press_start();
            } else if constexpr (std::is_same_v<T, CmdPressStop>) {
                exec.press_stop();
            } else if constexpr (std::is_same_v<T, CmdEstop>) {
                exec.set_estop(c.latched);
            } else if constexpr (std::is_same_v<T, CmdMode>) {
                exec.set_mode(c.mode);
            } else if constexpr (std::is_same_v<T, CmdConfigure>) {
                if (exec.state() != RobotState::IDLE) {
                    log::warning("[%s] settings change rejected: not IDLE (%s)", exec.robot_id.c_str(),
                                 to_string(exec.state()).c_str());
                    return;
                }
                exec.robot_width = c.robot_width;
                exec.robot_length = c.robot_length;
                exec.kinematics.max_velocity = c.max_velocity;
                exec.kinematics.max_acceleration = c.max_acceleration;
                exec.kinematics.max_deceleration = c.max_acceleration;
                exec.battery.max_velocity = c.max_velocity;
                exec.sensor_range = c.sensor_range;
                if (c.robot_id != exec.robot_id) {
                    comm.rebind_id(c.robot_id);
                    exec.robot_id = c.robot_id;
                }
                log::info("[%s] GUI settings applied: size=%.2fx%.2f max_velocity=%.3f max_acceleration=%.3f "
                          "sensor_range=%.2f",
                          exec.robot_id.c_str(), exec.robot_width, exec.robot_length, c.max_velocity,
                          c.max_acceleration, c.sensor_range);
            } else if constexpr (std::is_same_v<T, CmdSetBattery>) {
                exec.battery.charge = std::max(0.0, std::min(100.0, c.charge));
                log::info("[%s] GUI debug: battery set to %.0f%%", exec.robot_id.c_str(), exec.battery.charge);
            }
        },
        cmd);
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        if (!parse_args(argc, argv, args)) return 0;
    } catch (const std::exception& e) {
        std::cerr << "argument error: " << e.what() << "\n";
        return 1;
    }
    log::set_level(args.log_level);

    unsigned speed_seed = seed_from_string(args.seed ? (std::to_string(*args.seed) + "-" + args.id)
                                                       : (args.id + "-nondeterministic"));
    std::mt19937 speed_rng(speed_seed);
    std::uniform_real_distribution<double> noise_dist(-args.speed_noise, args.speed_noise);
    double max_velocity = args.max_velocity * (1.0 + noise_dist(speed_rng));

    std::optional<MapModel> map_model;
    if (args.map) {
        try {
            map_model = MapModel::load(*args.map);
        } catch (const std::exception& e) {
            log::error("[%s] failed to load map: %s", args.id.c_str(), e.what());
            return 1;
        }
        // 시작 위치가 벽/미탐색 위여도 실행은 한다. 오더를 받으면 경로계획이 NO_PATH 로 실패하고,
        // 강제 위치지정(force_relocalize)으로 주행 가능한 곳에 옮기면 그 오류가 풀린다.
        CellType cell = map_model->cell_at(args.x, args.y);
        if (!drivable(cell)) {
            log::warning("[%s] start position (%.2f, %.2f) is not drivable (%s) -- starting anyway; "
                         "orders will fail with NO_PATH until the robot is relocalized onto a free cell",
                         args.id.c_str(), args.x, args.y, cell == CellType::OCCUPIED ? "OCCUPIED" : "UNKNOWN");
        } else {
            log::info("[%s] map check passed (path planning + collision detection enabled)", args.id.c_str());
        }
    }

    unsigned obstacle_seed = seed_from_string(
        args.obstacle_seed ? (std::to_string(*args.obstacle_seed) + "-" + args.id) : (args.id + "-obstacle"));
    DynamicObstacleField obstacle_field(obstacle_seed);
    if (args.obstacle_zone) {
        obstacle_field.configure_zone(*args.obstacle_zone, args.obstacle_sides, args.obstacle_spawn_interval,
                                       args.obstacle_max_count, args.obstacle_width, args.obstacle_height,
                                       args.obstacle_speed);
    }

    BatterySim battery(args.battery, 0.01, 0.05, 1.0, max_velocity);
    OrderExecutor executor(args.id, args.x, args.y, args.theta, max_velocity, args.max_acceleration,
                            battery, map_model ? &*map_model : nullptr, args.robot_width, args.robot_length,
                            std::move(obstacle_field), args.sensor_range);

    executor.path_refine.enabled = args.path_refine;
    executor.path_refine.seed = seed_from_string(args.seed ? (std::to_string(*args.seed) + "-" + args.id + "-refine")
                                                           : (args.id + "-refine"));

    // SET_MAP 다운로더. 워커 스레드가 번들을 받아오면 robot_loop 가 회수해 executor.map 을 교체한다.
    MapSync map_sync(args.map_cache, args.id);
    executor.map_sync = &map_sync;

    RobotComm comm(args.id, args.host, args.port);
    if (!comm.connect()) {
        log::error("[%s] broker connection failed (%s:%d) -- is mosquitto running?", args.id.c_str(),
                   args.host.c_str(), args.port);
        return 1;
    }
    if (!comm.wait_until_connected(5.0)) {
        log::error("[%s] broker not responding", args.id.c_str());
        comm.disconnect();
        return 1;
    }

    RealtimeClock clock;
    constexpr double TICK_PERIOD = 0.05;
    constexpr double STATE_PERIOD = 0.2;
    Ticker ticker(clock, TICK_PERIOD);
    ticker.reset();

    std::atomic<bool> stop_flag{false};
    g_stop_flag = &stop_flag;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    log::info("[%s] started. initial pose=(%.2f, %.2f, %.2f) max_velocity=%.3f m/s", args.id.c_str(), args.x, args.y,
              args.theta, max_velocity);

    std::mutex exec_mutex;      // GUI 스레드와 tick 루프 스레드가 executor 를 공유할 때만 의미 있음
    CommandQueue command_queue;  // GUI -> tick 루프. --gui 아니면 아무도 안 채운다

    std::unique_ptr<MapModel> synced_map;  // map_sync 가 받아온 맵 (executor.map 이 여기를 가리킴)

    auto robot_loop = [&]() {
        double since_state = 0.0;
        while (!stop_flag) {
            double dt = ticker.next();
            if (stop_flag) break;
            std::lock_guard<std::mutex> lock(exec_mutex);  // gui 모드 아니면 경합 없어 사실상 공짜
            for (auto& msg : comm.drain()) {
                if (std::holds_alternative<Order>(msg)) executor.accept_order(std::get<Order>(msg));
                else executor.handle_instant(std::get<InstantAction>(msg));
            }
            if (auto ready = map_sync.take_ready()) {
                auto previous = std::move(synced_map);  // 교체 후 해제
                synced_map = std::move(ready->model);
                executor.set_map(synced_map.get(), ready->map_uid, ready->map_version);
            }
            while (auto cmd = command_queue.pop()) dispatch_gui_command(executor, comm, map_sync, *cmd);
            executor.tick(dt);
            since_state += dt;
            if (since_state >= STATE_PERIOD) {
                since_state = 0.0;
                comm.publish_state(executor.build_state(comm.next_header_id(), clock.now()));
            }
        }
    };

    if (args.gui) {
        // Qt 는 메인스레드에서 돌아야 하므로 tick/MQTT 루프를 백그라운드로 보낸다
        // (robot_client/gui.py 와 동일한 스레드 모델).
        std::thread loop_thread(robot_loop);
        launch_gui(argc, argv, executor, exec_mutex, command_queue, map_sync, max_velocity,
                   args.max_acceleration, args.robot_width, args.robot_length, [&]() { stop_flag = true; });
        stop_flag = true;  // 창이 닫혔는데 on_close 가 아직 못 돌았을 경우 대비
        if (loop_thread.joinable()) loop_thread.join();
    } else {
        robot_loop();
    }

    log::info("[%s] shutting down", args.id.c_str());
    comm.disconnect();
    return 0;
}
