# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

이 리포에서 일하는 AI 코딩 에이전트(Claude Code 등)용 안내. 코드와 대조해 2026-11-12 (`v1.4` 이후, MuJoCo 물리·곡선 주행·장애물 우회·Global 경로(M1~M3)·장애물 기억·로봇 간 인식·월드 장애물·시간 배속·단위 테스트 미태그) 기준으로 맞춰 두었다.

## 개요

AMR(자율주행 로봇) 시뮬레이터 모노레포. 세 부분이 MQTT+JSON 으로만 통신한다.

| 경로 | 내용 |
|---|---|
| `robot/` (`src/ include/ tools/ tests/ Makefile`) | C++20 로봇 클라이언트. 프로세스 1개 = 로봇 1대. Python 원본(`robot_client/*.py`, 이 리포에 없음) 포팅이라 주석에 원본 참조가 많다. 이동은 MuJoCo 물리로 계산한다 |
| `tools/*.py` | Python 도구: 맵 생성/정리/편집기, 통합 런처. 루트 `./launcher` 가 진입점 |
| `scripts/` | `launch.sh`/`stop.sh` (터미널 대화형 N대 실행/종료) |
| `build/` | 빌드 산출물 (gitignore): `bin/` 실행파일, `obj/` 오브젝트 |
| `viewer/` | 라이브 플릿 뷰어 (Python/PySide6). 관측자. 유일한 발행은 `b` 모드 월드 장애물 배치/삭제 (`sim/v1/world/obstacles/{id}`, retained) |
| `broker/` | 개발용 mosquitto 설정 (`mosquitto.conf`: `127.0.0.1:1883`, 익명 허용, persistence 끔) |

FMS(`fms_server`)는 형제 폴더 `../AMR-fms` 별도 리포(private)에 있고 같은 프로토콜로 붙는다. FMS 가 없으면 `build/bin/send_order` 가 그 역할을 대신한다. `maps/` 는 클라이언트와 뷰어가 같이 쓰며, `viewer/maps` 사본은 없다.

## 작업 규칙

- 사용자는 한국어로 소통한다. 코드 주석과 README 도 한국어로 쓰고 기존 스타일을 따른다.
- push, 태그, 공개 리포 대상 작업은 사용자 확인 후에 한다. 이 리포는 public 이다.
- 버전은 annotated 태그(`v1.0`~`v1.4`)로 매긴다. 사용자가 "버전 올려" 라고 하면 마지막 태그의 minor 를 +1 하고 태그 메시지에 변경 요약을 쓴다.
- 기능을 바꾸면 README 와 이 파일의 해당 절도 같이 고친다.
- 린트는 없다. 테스트는 단위 테스트(`make -C robot test`, 프레임워크 없이 `robot/tests/unit/check.hpp`)와 통합 테스트(`robot/tests/physics_test.py`), 시나리오 러너(`tools/run_scenario.py`)가 있다. 아래 "검증" 절.
- `CLAUDE.md` 와 `AGENTS.md` 는 머리말만 다른 사본이다. 한쪽을 고치면 다른 쪽도 같이 고친다.

## Git 운영

- `origin` = `DaeHwanZZang/AMR-simulator` (public, 메인).
- 기존 org 리포 `GC-WAYPOINT/AMR_client` 는 원래 `upstream`(fetch 전용, push URL `DISABLED`)으로 썼다. 현재 로컬 클론에는 이 remote 가 없다. 필요하면 다시 추가한다:
  `git remote add upstream <URL> && git remote set-url --push upstream DISABLED`
- org 에 반영할 버전은 수동으로 가져간다 (cherry-pick/patch). C++ 클라이언트는 `robot/` 아래에 있어 org 루트 레이아웃과 경로가 다르다. 뽑을 때 `git format-patch --relative=robot <범위>` 로 접두를 떼고 org 에서 `git am` 한다.
- `viewer/`, `broker/` 는 `git subtree` 로 원본 이력을 보존해 합쳤다. 기준 태그 `v1.1` = PSO 작업 직전.

## 명령 (리포 루트에서 실행)

```bash
# C++ 빌드 (macOS: brew install mosquitto cjson qt)
robot/tools/setup_mujoco.sh # 최초 1회. MuJoCo 를 robot/third_party/mujoco/ 에 설치 (gitignore). 없으면 make 가 이 안내를 하고 멈춘다
make -C robot          # build/bin/{robot_client,send_order,place_obstacle,obstacle_zone,plan_bench}
make -C robot test     # 단위 테스트 (build/bin/unit_tests, 브로커 불필요, 1 초 안쪽). 이름 일부로 고르기: build/bin/unit_tests planner memory
make -C robot clean   # build/ 삭제
./launcher            # 통합 런처 GUI (.venv 없으면 자동 생성)

# Python 도구 + 뷰어 공용 venv (최초 1회, Python 3.11+)
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# 실행
mosquitto -c broker/mosquitto.conf -v             # 브로커
build/bin/robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y -4.0
build/bin/robot_client --id AMR-002 --x 0 --y 0 --gui     # Qt 제어판
build/bin/robot_client --id AMR-003 --host <FMS IP>       # --map 없음: FMS 가 SET_MAP 으로 맵 전달
build/bin/robot_client -h                                 # 전체 옵션 (속도/크기/배터리/센서/동적 장애물/--seed/--path-refine/--friction/--mass/--record/--time-scale/--no-robot-detect/--no-world-obstacles 등)
build/bin/robot_client --id AMR-001 --map maps/empty_hall1/fms_map.json --x 3 --y 3 --friction 0.2 --record records   # 저마찰 + 학습 데이터 기록 (라이다 위치보정은 기본 켜짐, 끄려면 --loc-rate 0)
scripts/launch.sh           # 터미널 대화형으로 N대 실행 (PID 는 .run/pids, 로그는 logs/). jq 필요
scripts/stop.sh             # scripts/launch.sh 가 띄운 로봇 종료
.venv/bin/python tools/launcher.py   # (= ./launcher) 브로커/로봇/뷰어/편집기/생성기/make/테스트 실행·정지

# FMS 대역 (오더/즉시명령 1회 발행)
build/bin/send_order AMR-001 --path 2,1 5,1 --order-id o1
build/bin/send_order AMR-001 --path 14.75,5,0 --order-id d1 --action DOCK --param entry=BACKWARD --param undock_distance=0.6 --param min_charge=80   # 마지막 노드에 액션 (X,Y[,THETA])
build/bin/send_order AMR-001 --script scripts/examples/demo_job.txt   # 텍스트 스크립트 -> Order 1개 (--dry-run 이면 발행 없이 노드/액션만 출력)
build/bin/send_order AMR-001 --instant PAUSE          # CANCEL_ORDER|PAUSE|RESUME|RESET_ERROR|INJECT_FAULT

# 장애물 (--world = 모든 로봇/뷰어가 같이 보는 월드 장애물, 로봇 ID = 그 로봇에게만 보이는 것)
build/bin/place_obstacle --world --x 10 --y 6 --width 0.6 --height 0.6 --obstacle-id box   # 지우기: --world --clear --obstacle-id box
build/bin/place_obstacle AMR-001 --x 10 --y 6                                              # 지우기: AMR-001 --clear

# 다중 로봇 시나리오 (자체 브로커 + 로봇 N대 + 오더/장애물 + 기대 검사, 종료 코드 0/1)
.venv/bin/python tools/run_scenario.py scripts/scenarios/head_on.json --viewer   # --time-scale N, --keep
build/bin/plan_bench maps/gachon/fms_map.json 45.5 30 84 30                      # 경로계획 시간 측정 (--dump 로 경유점 저장)

# 맵 도구
.venv/bin/python tools/gen_map.py tools/map_layouts/loop.txt --out maps/gen_loop     # 실제 맵 조각을 이어 붙여 생성
.venv/bin/python tools/synth_map.py tools/map_layouts/loop.txt --out maps/synth_loop # 벽+질감 노이즈로 한 번에 생성
.venv/bin/python tools/clean_map.py maps/gen_loop/fms_map.json --out DIR --level normal   # 노이즈 정리 (gen_map.py --clean 으로도)
.venv/bin/python tools/map_gen_gui.py   # 생성 GUI. synth_map/gen_map 을 QProcess 로 실행
.venv/bin/python tools/map_editor.py maps/gen_loop/fms_map.json   # 편집기
```

뷰어는 `viewer/` 에서 실행한다 (루트 `.venv` 로도 돌고, `viewer/requirements.txt` 로 따로 venv 를 만들어도 된다):

```bash
../.venv/bin/python tools/fleet_monitor_qt.py ../maps/warehouse/warehouse.json --host <브로커 IP>
```

- 빌드: `robot/Makefile` 은 `CXX := clang++` 고정이고 cmake 는 없다. `-MMD -MP` 로 헤더 의존성을 추적한다. 이게 없으면 `schemas.hpp` 를 바꿨을 때 stale `.o` 가 링크되어 런타임 세그폴트가 난다.
- Ubuntu 의존성(clang, libmosquitto-dev, libcjson-dev, qt6-base-dev, pkg-config, jq, curl, unzip)은 README 참고.
- 런타임에 `curl`/`unzip`/`jq` 가 필요하다 (`map_sync.cpp` 는 fork+exec, `scripts/launch.sh` 는 jq). 없으면 조용히 실패한다.
- 런처로 띄운 프로세스는 별도 세션이라 그냥 `ps` 로는 안 보인다 (현재 tty 만 나옴). `ps -ax | grep robot_client` 또는 `pgrep -fl robot_client` 를 쓴다.

## 검증

1. 브로커를 띄운다.
2. 로봇을 띄운다. 시작 좌표는 FREE 칸이어야 한다 (아래 "좌표계" 참고).
3. `build/bin/send_order` 로 오더를 보낸다.
4. `mosquitto_sub -t 'fms/v1/#' -v` 또는 뷰어로 State 를 확인한다.

단위 테스트: `make -C robot test`. `robot/tests/unit/test_*.cpp` 가 schemas / map_model / planner(팽창을 무차별 대입과 비교, A*, 곡선화, 창 계획) / local_planner / dynamic_obstacles / obstacle_memory / localizer 를 MQTT·물리 없이 시험한다. 새 파일은 `robot/Makefile` `UNIT_SRC` 에 넣고 `TEST(이름) { CHECK(...); }` 로 쓴다. planner 를 고치면 `build/bin/plan_bench ... --dump` 로 전후 경유점이 같은지도 본다.

자동 회귀 테스트: `.venv/bin/python robot/tests/physics_test.py [시나리오...]`. 자체 mosquitto(포트 18851, 누가 쓰고 있으면 빈 포트 — 남의 브로커를 재사용하지 않는다: retained 월드 장애물이 남아 있으면 시험이 오염된다)와 `empty_hall1` 으로 이동, STRAIGHT,
DOCK/PARK, 충전기·배터리, 충돌, 일시정지/취소, 벽 안 시작, 저마찰 슬립, 110 m 주행, 동적 장애물, GUI(offscreen) 기동과 스냅샷, physics 토픽, 위치추정 보정(켬/끔), 로컬라이징(UNLOCALIZED/힌트/기본 LOCALIZED)을 시험한다. 시나리오 헬퍼 `Robot` 은 기본으로 `--require-localization` 을 붙이고 힌트로 위치를 잡는다 (플래그 없는 기본 동작은 `default` 시나리오). 교통 구역 시나리오는 FMS 가 필요해 이 리포에서 뺐다.
`make -C robot` 으로 바이너리를 먼저 만들어야 한다. 이동/도착/충돌/도킹 코드를 건드렸으면 돌린다. 전체는 실시간으로 약 20 분이다.
헬퍼(`Robot`, 브로커 기동, `check`, `world_place`, `min_separation`)는 `robot/tests/simharness.py` 에 있고 `tools/run_scenario.py` 와 같이 쓴다.
`AMR_TEST_TIME_SCALE=4` 를 주면 로봇을 4 배속(`--time-scale`)으로 돌린다: `move curve dock detour script` 는 1 과 4 에서 결과가 같았다(3 분 41 초 -> 1 분 20 초). 벽시계로 기다리는 시나리오(pause, battery 등)는 1 에서만 검증했다.

시나리오 하나만 돌리려면 이름을 인자로 준다 (여러 개 가능, 인자가 없으면 전부). 실패가 하나라도 있으면 종료 코드 1 이다:

```bash
.venv/bin/python robot/tests/physics_test.py detour memory
```

이름(`physics_test.py` 맨 아래 `ALL`): `move straight dock collision default script curve lazy pause ghost lowfric slip loop longrun obstacle collide_obstacle robots world reroute memory shortcut navigation detour detour_clear zone dockint gui topic estimate localization unlocalized charger battery`. 새 시나리오는 `test_*` 함수를 만들고 `ALL` 에 등록한다.

다중 로봇은 시나리오 러너로 본다: `.venv/bin/python tools/run_scenario.py scripts/scenarios/<이름>.json [--viewer] [--time-scale N] [--keep]`. 시나리오 JSON 은 `map`, `robots[{id,x,y,theta,at,args}]`, `obstacles[{id,x,y,width,height,at,clear_at}]`(월드 장애물), `orders[{robot,at,path|script,action,params}]`, `expect[{robot,pose,tol} | {no_fatal} | {min_separation} | {robot,error}]`, `timeout` 이고 시각은 시뮬 초다. `min_separation` 은 `physics` 토픽 참 자세로 잰 로봇 쌍의 최소 중심 거리다. 예시: `head_on`(정면 교차), `parked`(서 있는 로봇 우회), `late_join`(장애물을 먼저 놓고 로봇을 나중에 띄움), `crossing_staggered`(시차 직각 교차)는 통과하고, `crossing`(동시 직각 교차)은 **알려진 한계라 실패한다** (아래 "공유 세계").

CI(`.github/workflows/ci.yml`): macOS 는 빌드 + 단위 테스트 + 통합 일부(`move straight dock collision curve collide_obstacle world robots`, 2 배속), Ubuntu 는 빌드 + 단위 테스트. Ubuntu 쪽(`setup_mujoco.sh` 의 Linux 경로 포함)은 로컬에서 돌려 보지 못했고 첫 실행 결과로 확인해야 한다.

맵 도구는 출력 png/json 을 직접 열어 확인한다. GUI 는 `QT_QPA_PLATFORM=offscreen` 으로 띄우고 `grab()` 스크린샷을 찍어 볼 수 있다. 실제 클릭과 다이얼로그 흐름은 사용자가 확인해야 한다.

## 맵과 좌표계

- 메타 `fms_map.json` 의 키는 `image`, `resolution`, `origin_x`/`origin_y`, `width_px`/`height_px`, 임계값 `free_thresh`/`occupied_thresh` 이다. 런처와 `scripts/launch.sh` 는 `image`, `resolution`, `width_px` 를 모두 가진 json 을 메타로 본다.
- `origin` 은 **왼쪽 위 픽셀의 world 좌표**이고 y 는 아래로 증가한다. 변환식은 `px = floor((x - origin_x) / res)` (`map_model.cpp`).
- 칸 분류: 회색값 `>= free_thresh` 이면 FREE, `<= occupied_thresh` 이면 OCCUPIED, 그 사이는 UNKNOWN, 맵 밖은 OCCUPIED. **주행 가능 = FREE 만** (`drivable()`).
- planner 는 비-FREE 칸 전부를 로봇 반폭(`ceil(width/2/res)` px)만큼 팽창시킨 뒤 A* 를 돌린다. 시작점이나 목표점이 비-FREE 면 `PlanningError` 를 던진다.
- 생성 맵(`gen_map`/`synth_map`)은 res 0.03125 (32 px/m)를 쓴다. 사방에 회색 여백 10 타일을 두고 origin 을 -10 으로 잡아서, 레이아웃 칸 (r,c) 의 중심이 world `(c+1, r+1)` m 가 된다. 그래서 world (0,0) 은 여백(UNKNOWN) 위라 주행할 수 없다. 기본 `--x 0 --y 0` 으로 띄우면 오더가 NO_PATH 로 실패한다.
- 같은 폴더에 맵을 다시 생성하면 origin 이 -여백 으로 초기화된다. 편집기에서 바꾼 원점은 사라진다.
- `maps/` 구성:
  - `641931de9eae7cecb34d5765`, `warehouse`: 실제 맵이다. `AMR_master/maps/` 에서 수동으로 복사했으므로 원본이 바뀌면 직접 다시 복사한다.
  - `gen_loop`, `synth_loop`, `gui_map`, `test1~3`, `empty_hall1`, `gachon`, `gachon_1f`: 생성 맵이다.
  - `empty_hall1`: 회귀 테스트(`physics_test.py`)가 쓰는 맵이고 `chargers.json` 이 있다. 테스트 좌표가 이 맵 기준이라 함부로 다시 생성하지 않는다.
  - `gui_map`: 사용자가 편집기로 원점을 (-11.65625, -11.625)로 바꾸고 `edits.json` 을 붙였다. 함부로 다시 생성하지 않는다.
  - `test1`: `grid.png`/`edited_navi_gridmap.png` 가 없어서 편집기로 열 수 없다.

## 아키텍처 (C++ 클라이언트)

- **진입점 `robot/src/main.cpp`**:
  - 순서: 인자 파싱 → `MapModel` 로드(옵션) → 시작 위치 검사 → `OrderExecutor` + `RobotComm` + `MapSync` 조립 → `robot_loop`.
  - 루프: `Ticker` 로 50 ms tick (`TICK_PERIOD`), 200 ms 마다 `State` 발행 (`STATE_PERIOD`).
  - 한 tick 안의 순서: MQTT 수신(`comm.drain()`) → `map_sync.take_ready()` 로 새 맵 교체 → GUI 명령 처리 → `executor.sync_world()`(월드 장애물 + 다른 로봇) → `executor.tick(dt)` → 상태 발행.
  - 시간 배속 `--time-scale N` (`clock.hpp` `Ticker`): 벽시계로 `50 ms / N` 마다 돌고 `dt` 는 시뮬 시간(벽시계 경과 × N)이다. 타임스탬프는 벽시계 그대로다. `tick()` 의 dt 상한(0.1 s)은 그대로라 CPU 가 못 따라가면 배속만 낮아진다. 같이 도는 로봇은 같은 값이어야 하고 FMS 연동 때는 1 로 둔다.
- **스레드 모델**: `--gui` 면 `robot_loop` 를 백그라운드 스레드로, Qt 이벤트루프를 메인 스레드로 돌린다. 공유 상태는 `exec_mutex` 로 보호한다. GUI→루프 명령은 `CommandQueue` (`robot/include/command_queue.hpp`: Move/Relocalize/LocalizeHint/Start/Stop/Estop/Mode/Configure/ResyncMap/SetBattery) 로 보낸다. 맵 다운로드는 별도 워커 스레드(`MapSync`)라 tick 을 막지 않는다. `executor` 를 건드리는 코드는 항상 이 락 규칙을 따라야 한다.
- **`OrderExecutor` (`executor.cpp`, 가장 큼)**:
  - 처리 범위: 오더 상태머신 (released/order_update_id/거절 규칙), e-stop/PAUSE/MANUAL, 노드 단위 경로계획(현재 목적지 노드 하나까지만 계획하고 도착·정렬 뒤 다음 노드를 계획한다. 중간 노드 theta 도 지켜진다), 액션(PICK/DROP/CHARGE/WAIT/STRAIGHT/DOCK/PARK), 충돌 감지, 동적 장애물 우회(`handle_obstacles`).
  - 소유 객체: `PhysicsWorld`(MuJoCo, `physics`), `TrapezoidalKinematics`(사다리꼴 궤적 = 명령 생성기, 위의 world 를 가리킨다), `BatterySim`, `DynamicObstacleField`, `Recorder`(`--record` 일 때만). 멤버 선언 순서가 중요하다: `physics` 가 `kinematics` 보다 먼저여야 한다.
  - `tick(dt)` 는 `dt` 를 0.1 초로 자른 뒤 `tick_logic` 을 돌고 `Recorder` 에 한 줄 쓴다. 움직이지 않는 분기(idle/액션/대기)는 `kinematics.brake(dt)` 를 불러 굴러가던 로봇을 감속시킨다.
  - `map` 포인터는 소유하지 않는다. `main` 의 `map_model`/`synced_map` 이 소유한다. 맵 교체는 `set_map()` 으로만 한다.
- **위치 오류 규칙** (v1.3):
  - 시작 위치가 비-FREE 여도 프로세스는 경고 로그만 남기고 뜬다 (`main.cpp` 시작 검사).
  - 그 상태에서 오더를 받으면 `replan()` 이 `PlanningError` 를 잡아 FATAL `NO_PATH` 를 올리고 ERROR 상태가 된다 (설명에 로봇 좌표 포함).
  - `force_relocalize` 로 FREE 칸에 옮기면(또는 맵이 없으면) `NO_PATH`/`COLLISION` 오류를 지운다. 비-FREE 칸이면 경고만 남긴다.
  - `force_relocalize` 는 GUI 제어판(`CmdRelocalize`)에서 부르고, 플래그 없는 기본 시작 때 `main` 이 한 번 부른다. MQTT instant action 에는 없다 (대신 `INIT_POSITION` 힌트).
  - `COLLISION` 은 이동 중에만 검사한다 (물리: 몸체-벽 접촉, 위 "물리").
- **물리 (MuJoCo, v1.4 이후)**:
  - 구조: `TrapezoidalKinematics` 는 이상 궤적을 내는 명령 생성기이고, 위치는 `PhysicsWorld` 가 정한다. `step()` 은 `sync_from_world()` 로 측정 자세를 읽고 → 이상 궤적을 한 걸음 내고 → 그 변위를 (v, w) 로 바꿔 `world.drive()` → `world.step(dt)` → 다시 측정한다. `kinematics.x/y/theta` 는 항상 마지막 **추정** 자세다 (오도메트리 + 위치추정 보정, 참 자세 아님). `executor` 는 이 값만 보면 된다.
  - 모드 스위치는 없다. `world_ == nullptr` 이면 순수 운동학(이전 동작)으로 도는 분기가 `kinematics.cpp` 에 남아 있지만 executor 는 항상 world 를 붙인다.
  - `physics_world.cpp` 만 `mujoco.h` 를 include 한다 (pimpl). 월드: 바닥 평면 + 차동구동 몸체(바퀴 구 2, 무마찰 캐스터 2, 충돌 원판) + 벽 상자. 맵 → 벽은 비-FREE 칸을 직사각형으로 병합하고(`merge_blocked_cells`), 로봇 주변 8 m 창만 MJCF 에 넣는다. 3 m 벗어나면 `rebuild()` 로 재컴파일하고 qpos/qvel/ctrl 을 옮긴다. 월드는 `set_map()` 때마다 맵 그리드에서 다시 만든다.
  - **충돌 형상은 원판(반지름 `width/2 - 0.04`)이다.** planner 가 반지름 `width/2` 원으로 팽창시키기 때문에 맞춘 것이고, 직사각형으로 하면 좁은 통로 모퉁이에서 회전 중 COLLISION 이 난다 (`gen_loop` 에서 확인). `length` 는 표시와 관성에만 쓴다. 이 결정을 바꾸면 planner 팽창도 같이 봐야 한다.
  - 충돌: 몸체-벽 접촉력 20 N 이상이 0.25 초 지속되면 `kinematics.collided()` 가 true 가 되고 `check_collision` 이 FATAL `COLLISION` 을 올린다. 기존 "중심점이 비-FREE" 검사도 남아 있다. 시작/강제 위치가 벽 안이면 `ghost` 로 몸체-벽 충돌을 끈다 (v1.3 의 "벽 위에서도 뜬다" 보존, 엔진이 튕겨내지 못하게).
  - 도착: 경로는 실제로 멈추고(`at_rest`) 마지막 지점 3 cm 이내여야 `is_path_complete()`. 이상 궤적이 끝났는데 못 닿으면 `goal_` 을 다시 건다. 직선 이동은 이상 궤적 목표를 매 tick 지금 heading 진행선 위로 투영하고(옆 방향 오차는 직선으로 못 줄인다), 목표 방위로 조향하며, 진행 방향 남은 거리 1 cm 이내에서 서면 끝난다. 끝난 뒤 정착 대기(`settle_`)도 `straight_active()` 에 포함되어 executor 의 `finish_straight` 가 실제로 선 뒤에 불린다.
  - 이상 궤적 속도는 실제 속도 ±0.3 m/s (각속도 ±0.5 rad/s)로 묶는다. 방위 허용치는 서 있다 출발할 때 0.05 rad, 달리는 중 0.3 rad 이고(이상 궤적은 0.02), 큰 꺾임(0.3 rad 초과) 경유점에서 서도록 `stop_distance_ahead()` 로 감속한다. 이걸 빼면 슬립/지연 때문에 서고-돌고를 반복하거나 모퉁이에서 순간 정지(수 m/s² 감속)한다.
  - 바퀴 서보 게인 `kServoGain = 6`: 60 이면 접촉 compliance 와 얽혀 정지 후에도 떨린다. 일단 6 이어도 슬립이 쓸 만하게 나온다.
  - `emergency_stop()`/`set_pose()` 는 world 를 즉시 정지/순간이동시킨다. `hold()` 와 `brake()` 는 `max_deceleration` 으로 감속시킨다. world 는 `kinematics.step/hold/brake` 안에서만 진행하므로 호출이 없으면 시간이 멈춘다.
  - 뷰어 디버그 토픽: `fms/v1/{id}/physics` (`PhysicsDebug`, `schemas.hpp`). `State` 는 뷰어 스키마가 `extra=forbid` 라 필드를 못 더해서 프로토콜 밖 토픽으로 뺐다. `main` 루프가 tick 마다 `executor.build_physics()` 를 발행한다. 뷰어는 pydantic 없이 dict 로 읽고(`fleet_monitor.py`), `qt_viewer.py` 가 외삽/궤적/GT/패널(`p`,`l`,`o`,`k`)로 그린다. 뷰어 몸체 = 보고 위치(`est_*` = State.pose), 청록 GT = 참 위치(`x y theta`), 노란 점선 = 보정 전 오도메트리(`odom_*`). 필드를 바꾸면 `schemas.cpp` `PhysicsDebug::to_json_string` 과 `qt_viewer.py` `_physics_text`/`display_pose`/`truth_pose` 를 같이 고친다.
  - GUI 제어판(`gui.cpp`)에는 같은 값을 보이는 Physics 절과 마찰/질량 설정칸이 있다 (`CmdConfigure.friction/mass` -> `apply_robot_geometry`). `AMR_GUI_SNAPSHOT=파일.png` 이면 1.5 초 뒤 창을 PNG 로 저장한다 (offscreen 레이아웃 확인용).
  - **추정 자세 기반 제어 + 위치추정 보정 (기본 켜짐)**: `sync_from_world()` 가 `world.est_*` 를 읽는다. 추정 = `Localizer`(`localizer.hpp/cpp`)가 오도메트리(`odom_*`, 바퀴 엔코더 적분)에 얹는 보정 오프셋 `T` (`추정 = T ∘ 오도메트리`). `Localizer::update()` 는 `PhysicsWorld::step()` 끝에서 주기(`--loc-rate`, 기본 5 Hz)마다 `z = 참 자세 + 노이즈` 로 추정을 `gain` 만큼 당기고 `T` 를 다시 푼다 (한 번에 0.15 m/0.1 rad 상한, dropout 확률로 측정 누락). `--loc-rate 0` 이면 보정 없음(추정 = 순수 오도메트리). `Localizer` 는 엔진을 모른다 (참 자세는 호출자가 넘긴다) — 나중에 EKF 등으로 바꿀 때 이 클래스만 갈아낀다. 제어/경로계획/도착/`State.pose` 가 전부 추정으로 돈다. 참 자세(`world.x()` 등)는 충돌 판정(`check_collision`), 기록, `physics` 토픽(뷰어)에만 쓴다. `teleport`/`set_pose` 는 오도메트리를 그 자세로 맞추고 `T` 를 지운다. 경로선은 뷰어에서 추정 좌표로 그린다.
  - 추정에 노이즈가 있어서 `kinematics.cpp` `step_ideal` 이 5 cm 안의 경유점(마지막 제외)을 건너뛰고 2 cm 안이면 방위를 유지한다 (아니면 구간 경계의 겹친 경유점 방위가 노이즈로 흔들려 제자리 헌팅한다). 도착 허용(3 cm)보다 `kAimMinDist` 를 작게 해야 마지막 보정 접근(옆 3 cm 를 돌아서 가기)이 된다.
  - 기록: `--record DIR` → `recorder.cpp` CSV(+meta.json). 열 정의는 README "물리 시뮬레이션" 절. `State` 스키마는 바뀌지 않았다 (오도메트리 등은 State 가 아니라 기록 파일에만).
  - 물리가 아닌 것: 동적 장애물과 다른 로봇은 MuJoCo 객체가 아니다 (밀거나 밀리지 않는다). 대신 몸체 겹침을 기하로 판정해 `COLLISION` 을 올린다 (아래 "공유 세계"). IMU 없음. 라이다는 로컬라이징용 가상 스캔(`scan_matcher`)뿐이고 장애물 감지에는 안 쓴다 (감지는 거리만 보고 벽에 가려지는 것은 따지지 않는다).
  - 디버깅: `AMR_PHYSICS_DUMP=파일` 로 마지막 MJCF 를 덤프해 파이썬 `mujoco` 로 재현할 수 있다. 서보/접촉 파라미터는 그렇게 스윕해서 정했다.
- **로컬라이징 / 충전기 / 교통 구역** (makerton 에서 역이식):
  - 기본은 시작 자세를 이미 아는 상태(LOCALIZED, 위치추정 보정 켜짐)로 뜬다. `--require-localization` 이면 UNLOCALIZED 로 떠서 `INIT_POSITION` 힌트(또는 GUI Pose Hint/Force Localize) 전에는 오더를 거절하고 움직이지 않는다 (`motion_inhibited`/`can_accept_job`). 플래그가 꺼져 있으면 `SET_MAP` 으로 다른 맵 uid 가 와도, 순간이동(`sim/v1/{id}/teleport`)을 받아도 LOCALIZED 를 유지한다 (`OrderExecutor::require_localization`).
  - 힌트를 받으면 LOCALIZING -> `localize_time`(1 s) 뒤 가상 라이다 스캔 매칭(`scan_matcher.cpp`, 360빔/20 m/노이즈 0.02 m)으로 LOCALIZED 또는 WARNING `LOCALIZATION_FAILED`(UNLOCALIZED 로 남음). 힌트는 ±1.5 m, ±30° 안이어야 한다. 점수는 `State.localization_score`. `Localizer` 의 추적 보정(`set_tracking`)은 LOCALIZED 일 때만 켜진다. 시작 직후 기본 경로는 `force_relocalize` 한 번으로 이 상태를 만든다.
  - `State.localization` 필드와 `INIT_POSITION` 액션(params x, y, theta, 선택 map_id)이 프로토콜에 추가됐다. `viewer/common/schemas.py` 도 같이 고쳤다 (FMS 원본 `AMR-fms/common` 과 동일).
  - 충전기: 맵 폴더의 `chargers.json`(`chargers.cpp`). `chargers.json` 이 **있는 맵에서만** DOCK 이 충전기 근처(0.3 m/0.35 rad)·진입 방향·자세(0.06 m/0.1 rad)를 검사해 `DOCK_FAILED` 를 낸다. 없는 맵은 예전처럼 충전기 없이 DOCK 을 받는다. `CHARGE` 가 충전기 밖이면 `CHARGE_FAILED`.
  - 배터리: `--battery-low`(기본 20, WARNING `LOW_BATTERY`), `--battery-critical`(기본 5, FATAL `BATTERY_CRITICAL` 이면 DOCK 오더만 받음), 0% 는 `BATTERY_EMPTY`.
  - 교통 구역: 맵 폴더 `annotations.json` 의 `occupancy` 구역과 노드 `allowed_zone_ids`, `Order.traffic_control`. 허가 없는 구역은 재계획에서 막고 직선 이동은 `TRAFFIC_BLOCKED`(FATAL). 구역 정의는 FMS 가 만든 번들에서 온다 (이 리포에는 편집기가 없다).
  - 감속 구역: `annotations.json` 의 `type=slow` 구역(다각형 + `speed_limit` m/s). `map_model.cpp` 가 `slow_zones` 로 읽고, `tick_logic` 이 매 tick `MapModel::speed_limit_at(추정 x, y)` 를 `kinematics.speed_cap` 에 건다 (겹치면 가장 낮은 값). 주행을 막지 않고 경로계획에도 안 쓴다. 값이 잘못된 구역은 그 구역만 건너뛰고 맵은 거부하지 않는다. 다각형 판정은 `common/map_annotations.py` `Zone.contains` 와 같은 ray casting 이다.
  - 복도 중앙 주행: planner A* 에 벽 근접 비용을 더한다. `--no-center-path`, `--center-weight`(6), `--center-reach`(1.5 m).
  - 오더를 받으면 경로계획 전에 State 를 바로 한 번 발행한다 (`on_order_received`).
- **공유 세계: 장애물 충돌 / 월드 장애물 / 로봇 간 인식** (`executor.cpp` `check_obstacle_contact`/`sync_world`, `comm.cpp`):
  - 로봇 프로세스는 서로 독립이라, 같이 보는 장애물과 서로의 몸체는 MQTT 로 맞춘다. 전부 프로토콜 밖이고 FMS 와 맞출 필요가 없다.
  - **장애물 충돌**: 매 tick `obstacle_field.obstacles` 전체(감지 여부 무관)와 참 자세 몸체 원판(`body_disc_radius(width)` = `width/2 - kBodySkin`, `physics_world.hpp`, 벽 충돌 원판과 같은 값)의 겹침을 본다 (`DynamicObstacle::overlaps_disc`). **달리는 중(|v| > 0.05 m/s)에 새로 겹치면** FATAL `COLLISION` ("hit obstacle <id>" / "hit robot <id>"). 서 있는 로봇에 장애물이 와서 닿은 것, 이미 겹쳐 있던 것(`colliding_ids_`)은 넘어간다 — `RESET_ERROR` 뒤 같은 접촉으로 오류가 반복되지 않는다. tick 시작(장애물이 움직인 직후)과 `check_collision`(로봇이 움직인 직후) 두 곳에서 부른다.
  - **월드 장애물**: 장애물 하나 = retained 토픽 하나 `sim/v1/world/obstacles/{id}` (JSON `x y width height`, 선택 `map_id`, 빈 페이로드 = 삭제). 전체 목록을 한 토픽에 두지 않은 이유는 뷰어·CLI 가 동시에 쓸 때 덮어쓰기 경합이 생겨서다. `comm.cpp` 가 수신 스레드에서 표(`world_`)만 갱신하고 `snapshot_world()` 로 꺼낸다. 재접속 때 표를 비운다 (끊긴 동안의 삭제를 retained 재전송으로 맞춘다). 놓는 쪽: 뷰어 `b`, `place_obstacle --world`, 시험의 `world_place`. 로봇별 `INJECT_OBSTACLE`/`CLEAR_OBSTACLE`(프로토콜)은 그대로 있다.
  - **로봇 간 인식**: 각 로봇이 `fms/v1/+/physics` 를 구독해 다른 로봇의 **참 자세**와 `width`/`length`/`map_id`(PhysicsDebug 에 추가한 필드)를 표(`peers_`)에 둔다. 메시지마다 큐에 쌓지 않는다 (로봇마다 20 Hz). 2 초 동안 소식이 없으면 지우고, 정상 종료하는 로봇은 빈 physics 를 한 번 내서 바로 지워지게 한다. 참 자세를 쓰는 이유: 이건 "센서가 본 것" 의 대역이고 다른 장애물도 참 좌표다 (`State.pose` 를 쓰면 추정 오차만큼 헛것을 피한다).
  - `OrderExecutor::sync_world(world, peers)` 가 tick 직전에 `obstacle_field` 를 맞춘다 (`DynamicObstacleField::sync_external`: 접두 `world:` = MANUAL, `robot:` = `ObstacleSource::ROBOT`, 없어진 것은 삭제). 다른 맵(`map_id` 가 둘 다 있고 다름)은 무시한다. 다른 로봇의 감지/계획용 사각형은 회전한 `length×width` 의 축 정렬 경계 상자(보수적)이고, 충돌 판정은 `body_radius` 원판이다. 이후는 다른 장애물과 같은 경로다: 감지 → `ObstacleMemory`(5 cm 넘게 움직이면 기억 안 함, 서 있는 로봇은 정지 장애물처럼 기억) → `handle_obstacles` 우회/재경로/대기 → 충돌 판정. `State.detected_obstacles` 에도 실린다 (AMR-fms 는 이 필드를 스키마에만 두고 로직에 쓰지 않는다, 2026-11-12 확인).
  - 끄기: `--no-robot-detect`, `--no-world-obstacles` (`comm.enable_world`).
  - **한계 (이동 예측 없음)**: 경로 막힘 판정이 상대의 지금 위치만 본다. 서 있는 로봇 우회, 정면 교차, 시차 교차는 된다. 두 로봇이 같은 시각에 직각으로 교차하면 충돌 1 초쯤 전에야 우회를 시작해 둘 다 `COLLISION` 이 나거나 맞닿은 채 교착한다 (`scripts/scenarios/crossing.json` 이 재현한다). 통과 폭이 없는 복도에서 마주치면 둘 다 WAITING 으로 교착한다. 둘 다 FMS 교통 관제가 막아야 하는 상황이고, 고치려면 상대 진행 방향으로 사각형을 `v × 예측 시간` 만큼 늘리는 예측이 필요하다 (미구현).
- **곡선 주행 (기본 켜짐, `--no-smooth-path` 로 끔)**:
  - 계획: `plan_segment_path()` 가 구간(노드 1개)의 A* 경유점을 (PSO 뒤에) `smooth_path()`(`planner.cpp`)로 다듬는다. 중간 코너마다 접선이 이어지는 3차 Bézier(목표 반경 0.6 m, 접선 길이는 이웃 구간의 45% 이하)를 샘플(0.1 m)로 넣는다. 샘플이 계획용 팽창 grid 의 비-주행 칸에 걸리면 크기를 절반씩 줄이고, 끝내 안 되면 그 코너는 원래대로 둔다(정지·회전). 꺾임각 0.05 rad 미만(직선)/2.6 rad 초과(U턴)는 곡선화하지 않는다. `SmoothParams`.
  - 추종: `TrapezoidalKinematics::set_path(path, pursuit=true)` 면 `step_ideal` 이 경유점 대신 속도 비례 전방 주시점(0.35 + 0.5 v, 최대 1 m)을 겨냥하고(Pure Pursuit), 곡률 속도 제한(`curvature_speed_cap`: v ≤ 0.8 w_max / κ, 그 지점까지 감속 가능한 범위)을 건다. 경로를 따라 끌고 가지 않고 heading 방향으로 나아가며(조향은 주시점이 한다), 지나친 점은 걷어 낸다. 마지막 점 접근·정지·최종 정렬·3 cm 도착·`goal_` 재걸기는 기존 규칙 그대로다. 곡선을 달리는 중 방위 허용치는 0.6 rad, 서 있다 출발할 때는 0.15 rad(주시점 방위가 추정 노이즈에 흔들려서). 전방 주시 거리는 앞쪽 곡률이 크면 줄인다(현 오차 L²/8R ≤ 4 cm, 최소 0.3 m): 좁은 복도 코너에서 안쪽 벽을 스치던 문제(COLLISION)를 잡았다. `smooth_path` 는 A* 팽창보다 5 cm 더 넉넉한 여유(`SmoothParams::extra_clearance`)를 요구한다.
  - FMS 노드는 곡선으로 통과하지 않는다: 노드마다 정지·정렬한다(위 "경로계획"). 곡선은 한 노드 구간 안의 A* 코너에만 적용된다. STRAIGHT/DOCK 진입·탈출은 별도 직선 제어라 해당 없다.
  - `State.local_path` 는 이제 곡선 샘플이 포함된 추종 경로다(직선 구간은 끝점만). 장애물 우회(M2)는 아래 절. Global 경로/`navigation` 토픽/뷰어 표시(M3)는 아래 "Global 경로" 항. 아직 없는 것: 계획 워커 스레드(M4, 필요할 때만).
- **장애물 우회 (로컬 계획, 달리면서)** (`local_planner.cpp`, `executor.cpp` `handle_obstacles`/`restore_nominal`):
  - 감지: 센서 반경(`--sensor-range`, 기본 3 m) 안의 동적 장애물(사각형 좌표)만 본다. 계획에는 `ObstacleMemory`(`obstacle_memory.hpp/cpp`, 엔진 비의존)가 돌려주는 `known` = 보이는 것 + 범위 밖 정지 장애물 기억을 쓴다 (`tick_logic` 에서 `update` 후 `handle_obstacles` 로 넘김). 규칙: 범위 안인데 안 보이면 치워진 것으로 보고 즉시 삭제, 움직이는 장애물(SIM/target/0.05 m 이상 이동)은 기억 안 함, ttl 기본 30 s(`--obstacle-memory`, 0=끔)이고 만료 뒤 같은 자리(0.3 m)에서 다시 보이면 2배(최대 `--obstacle-memory-max` 300 s). `CLEAR_OBSTACLE`/오더 종료로는 안 지우고 `set_map`/`force_relocalize`/텔레포트에서 지운다. `State.detected_obstacles`/`tick_straight` 는 기억이 아니라 지금 보이는 것만 쓴다. 시험 `physics_test.py memory`. 감지는 현재 위치만 쓰고 이동 예측은 없다(가로지르는 장애물에는 반응이 늦을 수 있다). 유효 지평선 = `min(--local-plan-radius, sensor_range)`.
  - `replan()` 이 구간의 정상 경로(`leg_nominal_`, 장애물 없이 계획한 곡선 경로)를 저장한다. 장애물이 있어도 정상 경로는 장애물 없이 만든다.
  - 현재 경로(`local_path`)가 장애물(+반폭 5 cm 덜)과 겹치지 않으면 그대로 간다(경로 유지). 겹치면 `--local-plan-rate`(5 Hz) 간격으로 `LocalPlanner::detour()`: 정상 경로 위 막힘 구간을 찾고 그 뒤 `rejoin_margin`(1 m) 지점(창 밖이면 창 안에서 가장 먼 자유 지점)을 합류점으로 삼아, 로봇 중심 반경 R 창(`plan_path_window`: 정적 팽창 grid 를 잘라 장애물 사각형 + 반폭 + `plan_margin` 0.15 m 를 막고 A*)에서 우회 경로를 구하고 정상 경로 꼬리를 이어 `smooth_polyline` 으로 다듬는다. 곡선은 `plan_margin` 의 절반까지만 코너를 자를 수 있다. 달리고 있어도 서지 않고 새 경로로 갈아탄다.
  - 합류점 보정 (`LocalPlanner::detour`): (1) 창 안 nominal 의 가장 뒤쪽 자유 지점으로 바로 가는 A* 가 "합류점 경유 + nominal 따라가기" 보다 `shortcut_gain`(1 m) 넘게 짧으면 그 경로를 쓴다. 우회로가 반대편으로 돌아 들어갈 때 목적지를 지나쳐 장애물 뒤 합류점까지 갔다 되돌아오던 문제를 막는다 (구간은 노드 하나라 nominal 중간을 건너뛰어도 된다, `physics_test.py shortcut`). (2) 합류점이 창 가장자리에 걸려 `rejoin_margin` 을 못 채우면(`short_rejoin`) 장애물에 바짝 붙은 급코너 + 수 cm 토막 구간이 생겨 로봇이 제자리에서 헌팅했다 (`detour` 시험 간헐 실패). 이때는 직선 시야가 열린 만큼 합류점을 건너뛰고 nominal 다음 꼭짓점으로 간다. (3) 우회/복귀 곡선화는 `smooth_path` 처럼 `extra_clearance`(5 cm) 넓힌 팽창 grid 를 쓴다 (좁은 복도 모퉁이 COLLISION 방지). 이 grid 는 `plan_segment_path` 가 미리 만들어 캐시한다 (큰 맵에서 첫 우회 tick 이 1 초 넘게 멈추지 않게).
  - 로컬 창 안에 우회로가 없으면(복도 전체 차단, 창 밖으로 돌아가야 함) **구간 전체를 장애물을 반영해 다시 계획**한다(`plan_segment_path(&obs)`, 1 초 간격 재시도, 정상 경로 `leg_nominal_` 은 그대로). 이 단계가 먼 길로 도는 Global 재경로다 (gui_map 0,0 -> 10.26,10.23 에 8.3,0 1.5x3 상자 시험으로 필요함이 확인됨). 재경로 중 장애물이 사라지면 `restore_nominal()` 이 직선 시야가 막혀 있으므로 구간을 다시 계획한다.
  - 재경로도 실패하면(길이 없음) 막힌 지점까지 거리가 제동거리(`v^2/2a`) + 0.3 m 이하일 때만 `hold()`(WAITING) 한다. 그 전에는 접근하면서 재시도한다.
  - 장애물이 사라지면 `restore_nominal()`: 이미 정상 경로 15 cm 안이면 그대로, 아니면 정상 경로 위 0.5 m 앞으로 `rejoin()`(직선 시야가 막히면 구간 재계획).
  - 속도 제한: `kinematics.speed_cap = sqrt(2 * 0.8 * a_dec * (sensor_range - 0.5))` (기본 설정 약 1.7 m/s 라 영향 없음).
  - 계획 시간은 `LocalPlanner::record()` 가 `local plan: n= p95= ms` 로 로그에 남긴다 (첫 호출과 20 회마다). 워커 스레드 없이 tick 안에서 동기 계산한다(p95 수십 ms 이하).
  - 직선 액션(STRAIGHT/DOCK 진입·탈출)은 우회하지 않고 장애물이 비킬 때까지 기다린다.
  - 한계: 로봇끼리 상호작용 없음, 이동 예측 없음. 재경로는 장애물이 보이는 동안 1 초마다 구간 전체 A* 를 돌려 큰 맵에서는 느릴 수 있다(워커 스레드 미구현).
- **Global 경로 + `navigation` 토픽 (M3)** (`executor.cpp` `take_navigation`, `schemas.hpp` `NavigationDebug`):
  - 표시 전용이다. 주행 판단에는 쓰지 않는다 (실제 추종은 `State.local_path`).
  - Global = **지금 향하는 노드 하나까지의 정상 경로**(`leg_nominal_`, `replan()` 이 만든 것과 같다) + DOCK/PARK 진입·STRAIGHT 직선 꼬리(`node_tail`). a->b->c 이면 b 로 가는 동안 a->b 만 있고, b 도착 뒤 `replan()` 이 b->c 를 만들면 그게 Global 이 된다. 액션 중/노드 대기처럼 움직이는 구간이 없으면 비어 있다(`status=IDLE`). 미래 구간은 계산하지 않는다.
  - Local(`State.local_path`)은 Global 과 같은 경로를 따라가고, 장애물 우회/재경로 때만 갈라진다. 우회 중에도 Global 은 정상 경로 그대로다. 구간 실패는 Global 이 아니라 기존 FATAL `NO_PATH` 로 보고된다.
  - 토픽 `sim/v1/{id}/navigation` (retained, QoS 1, 프로토콜 밖, `main` 이 tick 마다 `take_navigation` 을 불러 값이 있을 때만 발행). 서명(order/현재 노드/version/status/우회 여부/합류점)이 바뀔 때 최대 5 Hz, 안 바뀌면 5 s 하트비트. `version` 은 `replan()` 성공, Order/맵 변경, 종료 때 오른다. Order 종료/취소 땐 `status=IDLE` + 빈 경로. 필드: `order_id version status current_index global_path detour_active local_goal plan_radius plan_rate`. `local_goal` = 우회 경로가 정상 경로에 다시 붙는 점(정상 경로에서 10 cm 넘게 벗어난 마지막 점의 다음 점), `plan_radius` = min(--local-plan-radius, --sensor-range).
  - 뷰어: `fleet_monitor.py` 가 구독해 `update_navigation` 으로 넘긴다 (`TOPIC_NAVIGATION_ALL` 은 공유 `common/schemas.py` 를 안 건드리려고 fleet_monitor 안에 둔다). `qt_viewer.py` 가 Global 얇은 선(로봇 가까운 점부터), 계획 반경 점선 원(우회 중 진하게), 합류점 마름모를 그린다. 키 `n` Global, `m` 반경/합류점. Local(추종) 경로는 기존 굵은 선.
  - 뷰어 장애물 배치: 키 `b` 로 배치 모드를 켜고 맵을 클릭하면 3x3 맵 픽셀 사각형을 **월드 장애물**(`sim/v1/world/obstacles/{id}`, retained)로 놓는다. `u`/`Cmd+Z` 는 이 뷰어가 직전에 놓은 것, `Shift+U` 는 지금 있는 월드 장애물 전부(다른 뷰어/도구가 놓은 것 포함)를 지운다. 발행은 `fleet_monitor.py` `publish_world_obstacle` 하나뿐이다 (뷰어의 유일한 발행 경로). 뷰어는 같은 토픽을 구독해서 그린다 (`qt_viewer.py` `update_world_obstacle`/`_draw_world_obstacle`): 놓은 것도 브로커를 한 바퀴 돌아와서 그려지고, 뷰어를 다시 띄워도, 로봇이 나중에 떠도 남아 있다. 맵 uid 가 있으면 `map_id` 를 붙여 다른 층 로봇은 무시한다. 키 `h` 는 단축키 도움말 패널.
  - 필드를 바꾸면 `schemas.cpp` `NavigationDebug::to_json_string` 과 `qt_viewer.py` `update_navigation_items` 를 같이 고친다.
  - 시험: `physics_test.py navigation` (바로 앞 노드까지만, 도착 뒤 다음 구간, 우회 중 `detour_active`, 종료 후 비움).
  - 알려진 한계: 우회(`LocalPlanner::detour`)의 중앙 주행 비용은 정적 벽 기준이라 동적 장애물 가장자리는 반영되지 않고, 합류 복귀(`rejoin`)는 중앙 비용 없는 직선이다.
- **직선 이동 / 도킹 / 주차** (v1.4):
  - `straight_`(StraightMotion: ACTION/DOCK_IN/UNDOCK)가 있으면 `tick` 은 `tick_straight` 만 돈다. 경로계획 없이 `kinematics.set_straight()` 로 heading 을 유지한 채 목표점까지 간다. PAUSE/e-stop 은 kinematics 쪽 목표를 지우므로 executor 가 목표를 들고 있다가 재개 때 다시 건다. `replan()` 은 `straight_` 가 있으면 아무것도 안 한다.
  - STRAIGHT: 노드 도착·정렬 후 노드 theta 방향 `distance` 만큼.
  - DOCK/PARK: `plan_segment_path` 가 해당 노드 목표를 `approach_point()`(진입 완료 자세에서 `undock_distance` 만큼 앞/뒤)로 바꾼다. 도착하면 직선으로 진입하고, 진입을 시작하는 순간 `docked_`(exit 지점 = 진입 시작 자세)를 만든다.
  - 언도킹은 액션이 아니다. `replan()` 이 세그먼트를 만들 때 `docked_` 가 있으면 먼저 `start_undock()` 으로 exit 지점까지 직선 이동한다. `docked_` 는 언도킹 완료 때만 지운다 (취소돼도 남음). 강제 위치지정은 지운다.
  - DOCK 은 `min_charge` 미만이면 `idle_state()` 가 CHARGING 을 돌려줘서 `can_accept_job()` 이 거절한다. `clear_order()` 는 도킹 중이면 `battery.charging` 을 유지한다.
  - 마커 보정은 일부러 넣지 않았다 (사용자 결정). v1.4 에는 도착 오차도 없었지만 물리 도입 뒤에는 물리가 만든다 (보통 수 mm, 위 "물리").
- **경로계획**: `planner.cpp` A* (로봇 크기 반영, 동적 장애물 사각형 입력). 장애물은 센서 범위(`--sensor-range`) 안에서만 인식한다. README 의 known issue 참고.
  - 성능 (gachon 3872×2272 기준, `plan_bench`): 팽창 `inflate_grid` 는 행별 가로 거리 + 세로 오프셋별 원판 반폭 비교로 수 초 → 수십 ms, A* 는 해시맵 대신 칸 번호 평면 배열(`AstarScratch`: `calloc` + 세대 스탬프, 스레드마다 재사용)로 313 → 52 ms / 1214 → 191 ms. 첫 계획(팽창 grid 2 개 생성 포함)은 약 6.9 s → 0.12 s. 두 변경 모두 결과가 예전과 같아야 한다: 팽창은 단위 테스트가 무차별 대입과 비교하고, A* 는 힙에 넣는 순서와 비용 계산을 그대로 뒀다 (`plan_bench --dump` 로 9 개 경우 경유점 동일 확인). 계획은 여전히 tick 안에서 동기 실행이다 (워커 스레드 없음).
- **PSO 경로 다듬기** (`path_refine.cpp`, `--path-refine`, 기본 꺼짐):
  - 위치: `plan_segment_path()` 에서 구간별 A* 결과의 중간 경유점을 PSO 로 미세 조정한다.
  - 경유점 **개수와 목표점은 바꾸지 않는다** (도착 판정과 곡선화 입력이 그대로이도록). 세그먼트는 항상 노드 1개다.
  - 동적 장애물 회피 경로에는 적용하지 않는다 (거리 장이 정적 맵 기준이라서).
  - 충돌 샘플이 A* 보다 늘면 원본을 쓴다.
  - 거리 장(`ClearanceField`)은 executor 가 lazy 생성하고 `set_map()` 에서 버린다. seed 는 `--seed` 와 로봇 id 에서 파생한다.
- **프로토콜**: `schemas.hpp/cpp` 가 메시지와 JSON 직렬화(cJSON)를 정의한다.
  - 메시지: Order / InstantAction / State / Factsheet.
  - 토픽: `fms/v1/{id}/{order,instant,state,connection,factsheet}`.
  - `Factsheet`: 바뀌지 않는 로봇 사양(`model width length max_speed trays actions traffic_control reports_loads`). `main` 이 `comm.set_factsheet()` 로 넘기면 `comm.cpp` 가 접속(재접속)할 때마다 retained QoS 1 로 낸다. 로봇 ID 를 바꾸면 이전 ID 의 retained 를 지운다. FMS 가 지원 액션으로 후보를 거르는 데 쓴다. 새 액션을 추가하면 `main.cpp` 의 `sheet.actions` 에도 넣는다.
  - `State.loads` (`LoadInfo{tray, payload_id}`): `payload_id` 가 있는 PICK 이 끝나면 비어 있는 가장 낮은 칸(`T1`, `T2`, ...)에 싣고, DROP 이 끝나면 내린다 (`executor.cpp` `update_loads`). FMS 가 장부와 대조한다.
  - 프로토콜 밖 토픽 (FMS 와 맞출 필요 없음): `fms/v1/{id}/physics`(위 "물리". 다른 로봇도 구독한다, 빈 페이로드 = 그 로봇 종료), `sim/v1/{id}/navigation`(위 "Global 경로"), `sim/v1/{id}/teleport`(수신. JSON `x`, `y` 필수, `theta`/`map_id` 선택 -> `executor.sim_teleport`), `sim/v1/world/obstacles/{id}`(수신, retained, 위 "공유 세계").
  - `Action.params`(v1.4 추가, float|str|bool map): STRAIGHT/DOCK/PARK 필수 인자. 검증은 `schemas.cpp` `check_action_params()` 와 `viewer/common/schemas.py` `Action._check_params` 가 같은 규칙 (FMS 원본 `AMR-fms/common/schemas.py` 도 동일).
  - InstantAction 종류: `CANCEL_ORDER, PAUSE, RESUME, RESET_ERROR, SET_MAP, INIT_POSITION, INJECT_FAULT, INJECT_OBSTACLE, CLEAR_OBSTACLE, START_OBSTACLE_ZONE, STOP_OBSTACLE_ZONE`.
  - `comm.cpp` 는 libmosquitto 래퍼다 (LWT, 재접속, `connection` retained 토픽).
  - **스키마 변경은 FMS 쪽과 맞춰야 한다.** 원본은 `AMR-fms/common/{schemas,map_model,map_annotations}.py` 이고 `viewer/common/` 은 바이트 동일 사본이다. 원본을 먼저 고치고 `cp` 로 사본을 갱신한 뒤 C++ 를 포팅한다 (AMR-fms `fms_server/tests/test_common_sync.py` 가 사본 일치를 검사). 커밋은 리포마다 따로 한다.
- **맵 로드/동기화**: `map_model.cpp` 가 `fms_map.json` 과 그 `image` png(`stb_image`)를 읽는다. `SET_MAP` 을 받으면 `map_sync.cpp` 가 zip 을 받아 `map_cache/<map_uid>/` 에 풀고 `State.map_id/map_version` 으로 보고한다. FMS 는 이 값이 일치하는 로봇에만 job 을 배정한다.
- **`send_order --script`** (`robot/tools/send_order.cpp` `parse_script`): 텍스트 파일 한 줄 = 명령 하나를 Order 하나로 컴파일해 발행한다 (프로토콜/로봇 코드는 그대로, FMS 가 같은 Order 를 보내도 동일).
  - 문법: `MOVE x y [theta]`(노드 하나, rad 또는 `90deg`), `PICK|DROP|CHARGE|WAIT [초] [key=value..]`, `STRAIGHT|DOCK|PARK key=value..`(직전 MOVE 노드에 붙는 액션), `START`/`END`(선택, END 뒤 명령은 오류), 주석 `//` `#`. 줄 앞 번호는 GOTO 라벨(중복 불가)이고 없어도 된다.
  - 반복: `GOTO 줄번호 횟수` (줄 앞 번호가 라벨). 횟수만큼 점프하고 파일을 읽는 단계에서 노드로 펼쳐 한 Order 로 보낸다. 횟수 없는 GOTO(무한 반복)는 Order 가 정적 노드 목록이라 지원하지 않고 오류를 낸다. 노드 5000개 / 실행 20만 문장 상한. 안쪽 GOTO 는 바깥 루프가 다시 돌 때 횟수가 새로 시작한다. 예시 `scripts/examples/loop_job.txt`.
  - 한 노드에는 액션이 하나뿐이라 액션이 연달아 오면 같은 좌표 노드를 자동 복제한다 (길이 0 구간은 로봇이 문제없이 지나간다, `physics_test.py script`). 첫 액션 앞에는 MOVE 가 있어야 한다.
  - 액션 인자는 로봇과 같은 검증(`Action::from_json`)을 미리 돌려 줄 번호와 함께 오류를 낸다. `--order-id` 기본값은 `파일명-HHMMSS` (파일명만 쓰면 같은 스크립트 재전송을 로봇이 "이미 끝난 오더" 로 무시한다). 예시는 `scripts/examples/demo_job.txt`, 런처에는 "스크립트 실행" 항목이 있다.
- **C++ `tools/`**: `send_order`, `place_obstacle`, `obstacle_zone` 은 별도 실행 파일이고 `schemas.o` 만 링크한다. `mqtt_once.hpp` 는 1회 발행 헬퍼다 (`retain` 인자). `plan_bench` 는 경로계획 벤치로 코어 오브젝트 전부를 링크한다 (MQTT 접속 없음).

## 아키텍처 (Python 도구, `tools/`)

같은 폴더끼리 import 한다 (`gen_map` → `clean_map`, `synth_map` → `gen_map`, `map_editor` → `map_edit_model`/`clean_map`, `launcher` → `launcher_core`).

- **`gen_map.py`**:
  - 입력: 레이아웃 텍스트 (`tools/map_layouts/`, `.`=통로 / `#`=벽, 1 글자 = 1 m).
  - 생성: marching squares 로 타일링하고 각 타일에 `grid.png` 실제 조각을 붙인다. 벽은 항상 타일 가운데 선이다.
  - 검사: `planner.cpp` 의 inflate 규칙을 Python 으로 재현했다 (`clean_map.inflate_free`). **planner 규칙이 바뀌면 같이 고친다.** 통로 사분면에 노이즈 픽셀을 허용하지 않는 이유도 이 팽창 때문이다.
  - 출력: `grid.png` + `edited_navi_gridmap.png` + `gen_info.json` + `layout.txt`. `edited_navi_gridmap.png` 는 최초 생성 때만 복사하고 이후에는 보존한다 (편집기/FMS 웹 UI 수정용). 메타의 `image` 는 이 파일이다.
- **`synth_map.py`**: 같은 레이아웃을 픽셀 마스크로 편다 (칸 중심 = 타일 모서리, gen_map 과 같은 배치). 경계 양쪽 `wall-px/2` 에 벽을 긋고 질감 노이즈를 맵 전체에 한 번에 깐다. 조각 경계가 없어서 이음매가 없다. 저장/검사/파싱은 `gen_map` 의 `save_map`/`check`/`load_layout` 을 쓴다.
- **`clean_map.py`**:
  - 처리 순서: 통로 구멍 메움 → FREE opening → 작은 FREE 섬 삭제 → FREE 에서 먼 비-FREE 를 128 로 → 선택(실선 벽, 값 평탄화).
  - 프리셋: light/normal/strong.
  - scipy 없이 morphology 와 run-length 라벨링을 직접 구현했다.
- **`map_gen_gui.py`**: 옵션 폼 + 시드 랜덤 재생성 버튼. 생성은 자식 QProcess 로 돌린다.
- **`map_editor.py` / `map_edit_model.py`**:
  - 데이터 흐름: grid.png + `edits.json`(world 좌표 도형: rect/line/path, 흰·검·회 3값) → `edited_navi_gridmap.png`.
  - 렌더 규칙은 모델(GUI 없음)이 가진다: 픽셀 중심이 도형 안이면 칠하고 AA 는 없다. 편집기는 UI 만 한다.
  - 시작 시 grid.png 와 edited_navi_gridmap.png 둘 다 필요하다. 저장은 수동이다.
  - **원점 재지정 (O 도구)**: 고른 픽셀 모서리가 world (0,0) 이 된다. 도형 world 좌표를 같이 평행이동해서 픽셀 위치는 그대로다. `fms_map.json` 의 origin 은 저장할 때만, 확인을 받고 쓴다. `edits.json.origin` 은 도형을 저장한 당시의 좌표계이고, `load_edits` 가 이 값이 맵 origin 과 다르면 보정한다.
  - 통로 끊김 검사는 `clean_map.inflate_free`/`label` 을 재사용한다 (로봇 반폭 8 px).
- **통합 런처 `launcher.py` / `launcher_core.py` / `launcher_tools.json`**:
  - `launcher_core` 는 Qt 가 없어서 단독으로 테스트할 수 있다.
  - 툴 목록은 `launcher_tools.json` 에 있고 코드는 툴 이름을 모른다. 예외는 브로커로, `listens` 키로 식별한다. **새 툴은 json 항목만 추가하면 붙는다.** 스키마 설명은 json 의 `_설명` 키에 있다.
  - mode: `single` / `per_map` / `multi`(폼으로 여러 개, `id_fmt`/`step` 으로 자동 이름·좌표) / `oneshot`.
  - cmd 템플릿: `{python}`, `{repo}`, `{host}`, `{port}`, `{map}`, `{폼 키}` 를 치환한다. 조건부 인자는 `{"if": 키, "args": [...]}`.
  - 그 밖의 키: `requires`(["broker"]: 없으면 자동 시작 제안), `needs_map`, `needs_build`(없으면 make 제안), `cwd`, `stop_timeout_s`, `env`({이름: 값} 환경변수, 값도 `{키}` 치환).
  - 현재 항목 id: `broker`, `robot`, `viewer`, `map_editor`, `map_gen`, `setup_mujoco`, `physics_test`, `unit_test`, `scenario`, `instant`, `run_script`, `make`. `robot_client` 에 옵션을 추가하면 `robot` 항목의 `cmd` 와 `params` 에도 넣어야 런처 폼에 나온다.
  - 프로세스: `Popen(start_new_session=True)` 로 띄우고 출력은 `logs/<이름>.log` 로 보낸다. 그래서 런처가 죽어도 자식은 살아남는다. 화면 로그는 이 파일을 tail 한 것이다 (최근 2000 줄).
  - 재부착: 다음 실행 때 `.run/launcher.json` 의 PID 와 명령(`ps -p PID -o command=`)을 대조해서 다시 붙는다.
  - 정지: SIGTERM 을 보낸다. `stop_timeout_s` 가 지나면 사용자 확인 후 프로세스 그룹에 SIGKILL 을 보낸다.
  - **파이썬 Qt 툴(`map_editor.py`, `map_gen_gui.py`)은 SIGTERM/SIGINT 를 `window.close()` 로 바꿔** closeEvent(저장 확인 등)를 타게 했다. 새 Qt 툴을 추가하면 같은 처리를 넣는다 (signal 핸들러 + 200 ms no-op `QTimer`, 이 타이머가 없으면 Qt 루프가 시그널을 못 받는다).
  - PATH 에 brew `sbin` 을 더한다 (mosquitto 위치).

## 주의 / 알려진 문제

- robot/Makefile 은 소스를 자동 수집하지 않는다. `.cpp` 를 추가하면 `CORE_SRC` 에 직접 넣는다 (`gui.cpp` 는 `GUI_OBJ` 로 따로 있다). 단위 테스트 파일은 `UNIT_SRC` 에 넣는다.
- 통합 테스트/시나리오는 로봇이 서로를 본다. 한 시험 안에서 로봇을 여러 대 띄우면 서로 장애물이 된다 (의도하지 않으면 `--no-robot-detect`). 비정상 종료한 로봇의 몸체는 2 초 동안 남는다.
- MuJoCo 는 `robot/tools/setup_mujoco.sh` 가 PyPI 휠에서 꺼내 `robot/third_party/mujoco/` 에 둔다 (macOS 에서만 실행해 봤고 Linux 는 미검증). macOS 는 휠의 install name 이 framework 경로라 스크립트가 `install_name_tool`/`codesign` 으로 고친다. 링크는 rpath 를 절대경로로 박으므로 리포를 옮기면 다시 `make` 한다.
- 물리 극단값 한계: `--friction 0.12 --max-acceleration 3` 같은 조합은 제자리 회전이 접지력을 넘어 방위가 헌팅하며 거의 못 간다. 마찰 0.15~0.5 / 가속 0.6~2.0 은 완주한다. 개선하려면 `rotate_step` 대신 측정 각속도를 쓰는 PD 제어가 필요하다.
- 제자리 회전 때 몸체가 오도메트리보다 3~4% 더 돈다 (구 바퀴 접촉 때문). 오도메트리 오차가 12 m 에 2~15 cm 쌓이는 이유 중 하나다.
- 셸이 zsh 라 변수 단어 분리가 안 된다. `$VAR` 로 인자 목록을 전개할 수 없으니 `${=VAR}` 를 쓴다. macOS `sed -i` 는 백업 인자(`sed -i ''`)가 필요하다.
- `send_order --path` 는 `X,Y[,THETA]` 토큰을 받고, `-` 로 시작하는 토큰에서 경로 수집을 멈춘다. 그래서 음수 좌표를 줄 수 없다. 생성 맵이 칸 중심을 양수 좌표로 두는 이유다.
- 주석의 오래된 참조:
  - `scripts/launch.sh`/`Makefile` 주석의 "CLAUDE.md 결정 4", "최종 타깃" 은 org 상위 프로젝트 문서를 가리킨다. 이 리포에는 해당 내용이 없다.
  - `scripts/launch.sh` 주석의 `robot_new` 는 이 리포 루트의 예전 이름이다.
- 맵 이미지 불일치: `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png` 인데, 예전 뷰어 사본은 `grid.png` 를 썼다. 뷰어는 메타의 `image` 를 따르므로 표시가 예전과 다를 수 있다 (미결정).
- README known issue:
  - 양 갈림길이 다 막혀도 기억 시간(적응형) 동안은 왕복하지 않는다. 치워진 장애물도 기억 시간만큼 피한다.
  - 로봇 간 인식은 들어갔다 (위 "공유 세계"). 남은 한계: 이동 예측이 없어 동시 직각 교차에서 충돌/교착한다.
  - PSO 는 이 둘과 무관하다. 갈림길 왕복은 장애물 기억(`ObstacleMemory`)으로 풀었다.
- `rotate_step` (`kinematics.cpp`) 미수정 문제:
  - 목표 방향 부호가 바뀌면 각속도가 순간 반전된다.
  - 목표 도달 시 남은 각도로 잘라서 급정지한다.
  - `hold()` 는 각속도 값만 줄이고 `theta` 는 움직이지 않는다. 그래서 보고되는 각속도와 실제 회전이 어긋난다.
- `.gitignore`: `*.o`, `*.d`, `build/`, `robot/third_party/mujoco/`, `map_cache/`, `records/`, `logs/`, `.run/`, `.venv/`, `__pycache__/`, `.DS_Store`, `.pytest_cache/`, `viewer/.map_cache/`.
