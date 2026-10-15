# CLAUDE.md

이 리포에서 일하는 AI 코딩 에이전트(Claude Code 등)용 안내. 코드와 대조해 2026-10-15 (`v1.4`) 기준으로 맞춰 두었다.

## 개요

AMR(자율주행 로봇) 시뮬레이터 모노레포. 세 부분이 MQTT+JSON 으로만 통신한다.

| 경로 | 내용 |
|---|---|
| 루트 (`src/ include/ tools/ maps/`) | C++20 로봇 클라이언트. 프로세스 1개 = 로봇 1대. Python 원본(`robot_client/*.py`, 이 리포에 없음) 포팅이라 주석에 원본 참조가 많다 |
| `tools/*.py` | Python 도구: 맵 생성/정리/편집기, 통합 런처 |
| `viewer/` | 라이브 플릿 뷰어 (Python/PySide6). 순수 관측자, 아무것도 발행하지 않는다 |
| `broker/` | 개발용 mosquitto 설정 (`mosquitto.conf`: `127.0.0.1:1883`, 익명 허용, persistence 끔) |

외부 `fms_server`(이 리포에 없음)도 같은 프로토콜로 붙는다. FMS 가 없으면 `tools/send_order` 가 그 역할을 대신한다. `maps/` 는 클라이언트와 뷰어가 같이 쓰며, `viewer/maps` 사본은 없다.

## 작업 규칙

- 사용자는 한국어로 소통한다. 코드 주석과 README 도 한국어로 쓰고 기존 스타일을 따른다.
- push, 태그, 공개 리포 대상 작업은 사용자 확인 후에 한다. 이 리포는 public 이다.
- 버전은 annotated 태그(`v1.0`~`v1.4`)로 매긴다. 사용자가 "버전 올려" 라고 하면 마지막 태그의 minor 를 +1 하고 태그 메시지에 변경 요약을 쓴다.
- 기능을 바꾸면 README 와 이 파일의 해당 절도 같이 고친다.
- 테스트/린트 프레임워크는 없다. 검증은 아래 "검증" 절처럼 직접 실행해서 한다.

## Git 운영

- `origin` = `DaeHwanZZang/AMR-simulator` (public, 메인).
- 기존 org 리포 `GC-WAYPOINT/AMR_client` 는 원래 `upstream`(fetch 전용, push URL `DISABLED`)으로 썼다. 현재 로컬 클론에는 이 remote 가 없다. 필요하면 다시 추가한다:
  `git remote add upstream <URL> && git remote set-url --push upstream DISABLED`
- org 에 반영할 버전은 수동으로 가져간다 (cherry-pick/patch). 그래서 C++ 클라이언트 파일은 루트 경로에서 옮기지 않는다. 옮기면 org 쪽과 충돌한다.
- `viewer/`, `broker/` 는 `git subtree` 로 원본 이력을 보존해 합쳤다. 기준 태그 `v1.1` = PSO 작업 직전.

## 명령 (리포 루트에서 실행)

```bash
# C++ 빌드 (macOS: brew install mosquitto cjson qt)
make                  # robot_client + tools/{send_order,place_obstacle,obstacle_zone}
make clean

# Python 도구 + 뷰어 공용 venv (최초 1회, Python 3.11+)
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# 실행
mosquitto -c broker/mosquitto.conf -v             # 브로커
./robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y 6.0
./robot_client --id AMR-002 --x 0 --y 0 --gui     # Qt 제어판
./robot_client --id AMR-003 --host <FMS IP>       # --map 없음: FMS 가 SET_MAP 으로 맵 전달
./robot_client -h                                 # 전체 옵션 (속도/크기/배터리/센서/동적 장애물/--seed/--path-refine 등)
./launch.sh           # 터미널 대화형으로 N대 실행 (PID 는 .run/pids, 로그는 logs/). jq 필요
./stop.sh             # launch.sh 가 띄운 로봇 종료
.venv/bin/python tools/launcher.py   # 통합 런처 GUI: 브로커/로봇/뷰어/편집기/생성기/make 실행·정지

# FMS 대역 (오더/즉시명령 1회 발행)
tools/send_order AMR-001 --path 2,1 5,1 --order-id o1
tools/send_order AMR-001 --path 14.75,5,0 --order-id d1 --action DOCK --param entry=BACKWARD --param undock_distance=0.6 --param min_charge=80   # 마지막 노드에 액션 (X,Y[,THETA])
tools/send_order AMR-001 --instant PAUSE          # CANCEL_ORDER|PAUSE|RESUME|RESET_ERROR|INJECT_FAULT

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

- 빌드: `Makefile` 은 `CXX := clang++` 고정이고 cmake 는 없다. `-MMD -MP` 로 헤더 의존성을 추적한다. 이게 없으면 `schemas.hpp` 를 바꿨을 때 stale `.o` 가 링크되어 런타임 세그폴트가 난다.
- Ubuntu 의존성(clang, libmosquitto-dev, libcjson-dev, qt6-base-dev, pkg-config, jq, curl, unzip)은 README 참고.
- 런타임에 `curl`/`unzip`/`jq` 가 필요하다 (`map_sync.cpp` 는 fork+exec, `launch.sh` 는 jq). 없으면 조용히 실패한다.
- 런처로 띄운 프로세스는 별도 세션이라 그냥 `ps` 로는 안 보인다 (현재 tty 만 나옴). `ps -ax | grep robot_client` 또는 `pgrep -fl robot_client` 를 쓴다.

## 검증

1. 브로커를 띄운다.
2. 로봇을 띄운다. 시작 좌표는 FREE 칸이어야 한다 (아래 "좌표계" 참고).
3. `tools/send_order` 로 오더를 보낸다.
4. `mosquitto_sub -t 'fms/v1/#' -v` 또는 뷰어로 State 를 확인한다.

맵 도구는 출력 png/json 을 직접 열어 확인한다. GUI 는 `QT_QPA_PLATFORM=offscreen` 으로 띄우고 `grab()` 스크린샷을 찍어 볼 수 있다. 실제 클릭과 다이얼로그 흐름은 사용자가 확인해야 한다.

## 맵과 좌표계

- 메타 `fms_map.json` 의 키는 `image`, `resolution`, `origin_x`/`origin_y`, `width_px`/`height_px`, 임계값 `free_thresh`/`occupied_thresh` 이다. 런처와 `launch.sh` 는 `image`, `resolution`, `width_px` 를 모두 가진 json 을 메타로 본다.
- `origin` 은 **왼쪽 위 픽셀의 world 좌표**이고 y 는 아래로 증가한다. 변환식은 `px = floor((x - origin_x) / res)` (`map_model.cpp`).
- 칸 분류: 회색값 `>= free_thresh` 이면 FREE, `<= occupied_thresh` 이면 OCCUPIED, 그 사이는 UNKNOWN, 맵 밖은 OCCUPIED. **주행 가능 = FREE 만** (`drivable()`).
- planner 는 비-FREE 칸 전부를 로봇 반폭(`ceil(width/2/res)` px)만큼 팽창시킨 뒤 A* 를 돌린다. 시작점이나 목표점이 비-FREE 면 `PlanningError` 를 던진다.
- 생성 맵(`gen_map`/`synth_map`)은 res 0.03125 (32 px/m)를 쓴다. 사방에 회색 여백 10 타일을 두고 origin 을 -10 으로 잡아서, 레이아웃 칸 (r,c) 의 중심이 world `(c+1, r+1)` m 가 된다. 그래서 world (0,0) 은 여백(UNKNOWN) 위라 주행할 수 없다. 기본 `--x 0 --y 0` 으로 띄우면 오더가 NO_PATH 로 실패한다.
- 같은 폴더에 맵을 다시 생성하면 origin 이 -여백 으로 초기화된다. 편집기에서 바꾼 원점은 사라진다.
- `maps/` 구성:
  - `641931de9eae7cecb34d5765`, `warehouse`: 실제 맵이다. `AMR_master/maps/` 에서 수동으로 복사했으므로 원본이 바뀌면 직접 다시 복사한다.
  - `gen_loop`, `synth_loop`, `gui_map`, `test1~3`: 생성 맵이다.
  - `gui_map`: 사용자가 편집기로 원점을 (-11.65625, -11.625)로 바꾸고 `edits.json` 을 붙였다. 함부로 다시 생성하지 않는다.
  - `test1`: `grid.png`/`edited_navi_gridmap.png` 가 없어서 편집기로 열 수 없다.

## 아키텍처 (C++ 클라이언트)

- **진입점 `src/main.cpp`**:
  - 순서: 인자 파싱 → `MapModel` 로드(옵션) → 시작 위치 검사 → `OrderExecutor` + `RobotComm` + `MapSync` 조립 → `robot_loop`.
  - 루프: `Ticker` 로 50 ms tick (`TICK_PERIOD`), 200 ms 마다 `State` 발행 (`STATE_PERIOD`).
  - 한 tick 안의 순서: MQTT 수신(`comm.drain()`) → `map_sync.take_ready()` 로 새 맵 교체 → GUI 명령 처리 → `executor.tick(dt)` → 상태 발행.
- **스레드 모델**: `--gui` 면 `robot_loop` 를 백그라운드 스레드로, Qt 이벤트루프를 메인 스레드로 돌린다. 공유 상태는 `exec_mutex` 로 보호한다. GUI→루프 명령은 `CommandQueue` (`include/command_queue.hpp`: Move/Relocalize/Start/Stop/Estop/Mode/Configure/ResyncMap/SetBattery) 로 보낸다. 맵 다운로드는 별도 워커 스레드(`MapSync`)라 tick 을 막지 않는다. `executor` 를 건드리는 코드는 항상 이 락 규칙을 따라야 한다.
- **`OrderExecutor` (`executor.cpp`, 가장 큼)**:
  - 처리 범위: 오더 상태머신 (released/order_update_id/거절 규칙), e-stop/PAUSE/MANUAL, 세그먼트 단위 경로계획, 액션(PICK/DROP/CHARGE/WAIT/STRAIGHT/DOCK/PARK), 충돌 감지, 동적 장애물 재계획.
  - 소유 객체: `TrapezoidalKinematics`(이동/회전 사다리꼴 프로파일), `BatterySim`, `DynamicObstacleField`.
  - `map` 포인터는 소유하지 않는다. `main` 의 `map_model`/`synced_map` 이 소유한다. 맵 교체는 `set_map()` 으로만 한다.
- **위치 오류 규칙** (v1.3):
  - 시작 위치가 비-FREE 여도 프로세스는 경고 로그만 남기고 뜬다 (`main.cpp` 시작 검사).
  - 그 상태에서 오더를 받으면 `replan()` 이 `PlanningError` 를 잡아 FATAL `NO_PATH` 를 올리고 ERROR 상태가 된다 (설명에 로봇 좌표 포함).
  - `force_relocalize` 로 FREE 칸에 옮기면(또는 맵이 없으면) `NO_PATH`/`COLLISION` 오류를 지운다. 비-FREE 칸이면 경고만 남긴다.
  - `force_relocalize` 는 현재 GUI 제어판(`CmdRelocalize`)에서만 부를 수 있다. MQTT instant action 에는 없다.
  - `COLLISION` 은 이동 중에만 검사한다.
- **직선 이동 / 도킹 / 주차** (v1.4):
  - `straight_`(StraightMotion: ACTION/DOCK_IN/UNDOCK)가 있으면 `tick` 은 `tick_straight` 만 돈다. 경로계획 없이 `kinematics.set_straight()` 로 heading 을 유지한 채 목표점까지 간다. PAUSE/e-stop 은 kinematics 쪽 목표를 지우므로 executor 가 목표를 들고 있다가 재개 때 다시 건다. `replan()` 은 `straight_` 가 있으면 아무것도 안 한다.
  - STRAIGHT: 노드 도착·정렬 후 노드 theta 방향 `distance` 만큼.
  - DOCK/PARK: `plan_segment_path` 가 해당 노드 목표를 `approach_point()`(진입 완료 자세에서 `undock_distance` 만큼 앞/뒤)로 바꾼다. 도착하면 직선으로 진입하고, 진입을 시작하는 순간 `docked_`(exit 지점 = 진입 시작 자세)를 만든다.
  - 언도킹은 액션이 아니다. `replan()` 이 세그먼트를 만들 때 `docked_` 가 있으면 먼저 `start_undock()` 으로 exit 지점까지 직선 이동한다. `docked_` 는 언도킹 완료 때만 지운다 (취소돼도 남음). 강제 위치지정은 지운다.
  - DOCK 은 `min_charge` 미만이면 `idle_state()` 가 CHARGING 을 돌려줘서 `can_accept_job()` 이 거절한다. `clear_order()` 는 도킹 중이면 `battery.charging` 을 유지한다.
  - 도착 오차와 마커 보정은 일부러 넣지 않았다 (사용자 결정).
- **경로계획**: `planner.cpp` A* (로봇 크기 반영, 동적 장애물 사각형 입력). 장애물은 센서 범위(`--sensor-range`) 안에서만 인식한다. README 의 known issue 참고.
- **PSO 경로 다듬기** (`path_refine.cpp`, `--path-refine`, 기본 꺼짐):
  - 위치: `plan_segment_path()` 에서 구간별 A* 결과의 중간 경유점을 PSO 로 미세 조정한다.
  - 경유점 **개수와 목표점은 바꾸지 않는다** (`segment_wp_bounds_` 진행도 추적 때문).
  - 동적 장애물 회피 경로에는 적용하지 않는다 (거리 장이 정적 맵 기준이라서).
  - 충돌 샘플이 A* 보다 늘면 원본을 쓴다.
  - 거리 장(`ClearanceField`)은 executor 가 lazy 생성하고 `set_map()` 에서 버린다. seed 는 `--seed` 와 로봇 id 에서 파생한다.
- **프로토콜**: `schemas.hpp/cpp` 가 메시지와 JSON 직렬화(cJSON)를 정의한다.
  - 메시지: Order / InstantAction / State.
  - 토픽: `fms/v1/{id}/{order,instant,state,connection}`.
  - `Action.params`(v1.4 추가, float|str|bool map): STRAIGHT/DOCK/PARK 필수 인자. 검증은 `schemas.cpp` `check_action_params()` 와 `viewer/common/schemas.py` `Action._check_params` 가 같은 규칙. 외부 FMS 원본 `common/schemas.py` 는 아직 안 고쳤다.
  - InstantAction 종류: `CANCEL_ORDER, PAUSE, RESUME, RESET_ERROR, SET_MAP, INJECT_FAULT, INJECT_OBSTACLE, CLEAR_OBSTACLE, START_OBSTACLE_ZONE, STOP_OBSTACLE_ZONE`.
  - `comm.cpp` 는 libmosquitto 래퍼다 (LWT, 재접속, `connection` retained 토픽).
  - **스키마 변경은 FMS 쪽과 맞춰야 한다.** 뷰어의 `viewer/common/schemas.py` 도 같이 고친다.
- **맵 로드/동기화**: `map_model.cpp` 가 `fms_map.json` 과 그 `image` png(`stb_image`)를 읽는다. `SET_MAP` 을 받으면 `map_sync.cpp` 가 zip 을 받아 `map_cache/<map_uid>/` 에 풀고 `State.map_id/map_version` 으로 보고한다. FMS 는 이 값이 일치하는 로봇에만 job 을 배정한다.
- **C++ `tools/`**: `send_order`, `place_obstacle`, `obstacle_zone` 은 별도 실행 파일이고 `schemas.o` 만 링크한다. `mqtt_once.hpp` 는 1회 발행 헬퍼다.

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
  - 그 밖의 키: `requires`(["broker"]: 없으면 자동 시작 제안), `needs_map`, `needs_build`(없으면 make 제안), `cwd`, `stop_timeout_s`.
  - 프로세스: `Popen(start_new_session=True)` 로 띄우고 출력은 `logs/<이름>.log` 로 보낸다. 그래서 런처가 죽어도 자식은 살아남는다. 화면 로그는 이 파일을 tail 한 것이다 (최근 2000 줄).
  - 재부착: 다음 실행 때 `.run/launcher.json` 의 PID 와 명령(`ps -p PID -o command=`)을 대조해서 다시 붙는다.
  - 정지: SIGTERM 을 보낸다. `stop_timeout_s` 가 지나면 사용자 확인 후 프로세스 그룹에 SIGKILL 을 보낸다.
  - **파이썬 Qt 툴(`map_editor.py`, `map_gen_gui.py`)은 SIGTERM/SIGINT 를 `window.close()` 로 바꿔** closeEvent(저장 확인 등)를 타게 했다. 새 Qt 툴을 추가하면 같은 처리를 넣는다 (signal 핸들러 + 200 ms no-op `QTimer`, 이 타이머가 없으면 Qt 루프가 시그널을 못 받는다).
  - PATH 에 brew `sbin` 을 더한다 (mosquitto 위치).

## 주의 / 알려진 문제

- Makefile 은 소스를 자동 수집하지 않는다. `.cpp` 를 추가하면 `CORE_SRC` 에 직접 넣는다 (`gui.cpp` 는 `GUI_OBJ` 로 따로 있다).
- 셸이 zsh 라 변수 단어 분리가 안 된다. `$VAR` 로 인자 목록을 전개할 수 없으니 `${=VAR}` 를 쓴다. macOS `sed -i` 는 백업 인자(`sed -i ''`)가 필요하다.
- `send_order --path` 는 `X,Y[,THETA]` 토큰을 받고, `-` 로 시작하는 토큰에서 경로 수집을 멈춘다. 그래서 음수 좌표를 줄 수 없다. 생성 맵이 칸 중심을 양수 좌표로 두는 이유다.
- 주석의 오래된 참조:
  - `launch.sh`/`Makefile` 주석의 "CLAUDE.md 결정 4", "최종 타깃" 은 org 상위 프로젝트 문서를 가리킨다. 이 리포에는 해당 내용이 없다.
  - `launch.sh` 주석의 `robot_new` 는 이 리포 루트의 예전 이름이다.
- 맵 이미지 불일치: `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png` 인데, 예전 뷰어 사본은 `grid.png` 를 썼다. 뷰어는 메타의 `image` 를 따르므로 표시가 예전과 다를 수 있다 (미결정).
- README known issue:
  - 양 갈림길이 다 막히면 왕복한다.
  - 로봇끼리 서로를 장애물로 인식하지 않는다.
  - PSO 는 이 둘을 해결하지 않는다. 장애물 기억(시간 감쇠)이 필요하다.
- `rotate_step` (`kinematics.cpp`) 미수정 문제:
  - 목표 방향 부호가 바뀌면 각속도가 순간 반전된다.
  - 목표 도달 시 남은 각도로 잘라서 급정지한다.
  - `hold()` 는 각속도 값만 줄이고 `theta` 는 움직이지 않는다. 그래서 보고되는 각속도와 실제 회전이 어긋난다.
- `.gitignore`: `*.o`, `*.d`, `robot_client`, `tools/` 바이너리, `map_cache/`, `logs/`, `.run/`, `.venv/`, `__pycache__/`, `.DS_Store`.
