# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

AMR 시뮬레이터 모노레포. 세 부분이 MQTT+JSON 으로만 통신한다.

| 경로 | 내용 |
|---|---|
| 루트 (`src/ include/ tools/ maps/`) | C++20 로봇 클라이언트. 프로세스 1개 = 로봇 1대. Python 원본(`robot_client/*.py`) 포팅이라 주석에 원본 참조가 많다 |
| `viewer/` | 라이브 플릿 뷰어 (Python/PySide6). 순수 관측자, 아무것도 발행하지 않는다 |
| `broker/` | 개발용 mosquitto 설정 (`mosquitto.conf`) |

외부 `fms_server` 와도 같은 프로토콜로 붙는다 (이 리포에는 없음). `maps/` 는 클라이언트와 뷰어가 공유한다 — `viewer/maps` 사본은 없다.

## Git 운영

- 이 리포(`origin` = `DaeHwanZZang/AMR-simulator`, public)가 메인이다. 기존 org 리포 `GC-WAYPOINT/AMR_client` 는 `upstream` (fetch 전용, push URL `DISABLED`).
- org 에 반영할 버전은 수동으로 가져간다 (cherry-pick/patch). 그래서 C++ 클라이언트는 루트 경로를 옮기지 않는다 — 옮기면 org 쪽과 충돌한다.
- `viewer/`, `broker/` 는 `git subtree` 로 원본 이력을 보존해 합쳤다. 기준 태그 `v1.1` = PSO 작업 직전.
- push, 태그, 공개 리포 대상 작업은 사용자 확인 후에 한다.

## 명령 (리포 루트에서 실행)

```bash
make                  # robot_client + tools/{send_order,place_obstacle,obstacle_zone}
make clean
./robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y 6.0
./robot_client --id AMR-002 --x 0 --y 0 --gui     # Qt 제어판
./robot_client --id AMR-003 --host <FMS IP>       # --map 없음: FMS 가 SET_MAP 으로 맵 전달
./launch.sh           # 인터랙티브 N대 실행 (PID 는 .run/pids, 로그는 logs/)
./stop.sh             # launch.sh 가 띄운 로봇 종료
mosquitto -c broker/mosquitto.conf -v             # 브로커 (127.0.0.1:1883, 익명 허용 = 개발용)
```

뷰어 (`viewer/` 에서 실행, venv 는 직접 만든다):

```bash
python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python tools/fleet_monitor_qt.py ../maps/warehouse/warehouse.json --host <브로커 IP>
```

- 테스트/린트 없음. 검증은 mosquitto 브로커를 띄우고 `robot_client` + `tools/send_order` 로 수동 확인.
- macOS: `brew install mosquitto cjson qt`. Ubuntu 의존성(clang, libmosquitto-dev, libcjson-dev, qt6-base-dev, pkg-config, jq, curl, unzip)은 README 참고.
- `Makefile` 은 `CXX := clang++` 고정, cmake 없음. `-MMD -MP` 로 헤더 의존성을 추적한다 (없으면 `schemas.hpp` 변경 시 stale `.o` 링크로 런타임 세그폴트).
- 런타임에 `curl`/`unzip`/`jq` 가 필요하다 (`map_sync.cpp` 는 fork+exec, `launch.sh` 는 jq). 없으면 조용히 실패한다.

## 아키텍처

- **진입점 `src/main.cpp`**: 인자 파싱 → `MapModel` 로드(옵션) → `OrderExecutor` + `RobotComm` + `MapSync` 조립 → `robot_loop`. 루프는 `Ticker` 로 50ms tick, 200ms 마다 `State` 발행. 한 tick 안에서: MQTT 수신(`comm.drain()`) → `map_sync.take_ready()` 로 새 맵 교체 → GUI 명령 처리 → `executor.tick(dt)` → 상태 발행.
- **스레드 모델**: `--gui` 면 `robot_loop` 를 백그라운드 스레드로, Qt 이벤트루프를 메인 스레드로 돌린다. 공유 상태는 `exec_mutex`, GUI→루프 명령은 `CommandQueue`. 맵 다운로드는 별도 워커 스레드(`MapSync`)이며 tick 을 막지 않는다. `executor` 를 건드리는 코드는 항상 이 락 규칙을 따라야 한다.
- **`OrderExecutor` (`executor.cpp`, 가장 큼)**: 오더 상태머신 (released/order_update_id/거절 규칙), e-stop/PAUSE/MANUAL, 세그먼트 단위 경로계획, 액션 실행, 충돌 감지, 동적 장애물 재계획을 한 곳에서 처리. 소유 객체: `TrapezoidalKinematics`(이동/회전 사다리꼴 프로파일), `BatterySim`, `DynamicObstacleField`. `map` 포인터는 소유하지 않는다 (`main` 의 `map_model`/`synced_map` 이 소유) — 맵 교체는 `set_map()` 으로만.
- **경로계획**: `planner.cpp` A* (로봇 크기 반영, 동적 장애물 사각형 입력). 장애물은 센서 범위(`sensor_range`) 안에서만 인식한다 — README 의 known issue(양 갈림길 막힘 시 왕복, 로봇끼리 장애물 미인식) 참고.
- **프로토콜**: `schemas.hpp/cpp` 가 Order/InstantAction/State 와 토픽(`fms/v1/{id}/...`)·JSON 직렬화를 정의 (cJSON). `comm.cpp` 는 libmosquitto 래퍼 (LWT, 재접속, `connection` retained 토픽). 스키마 변경은 FMS 쪽과 맞춰야 한다.
- **맵**: `map_model.cpp` 가 `fms_map.json`(+grid png, `stb_image`) 로드. `SET_MAP` 수신 시 `map_sync.cpp` 가 zip 을 받아 `map_cache/<map_uid>/` 에 풀고 `State.map_id/map_version` 으로 보고한다. FMS 는 이 값이 일치하는 로봇에만 job 을 배정한다.
- **PSO 경로 다듬기 (`path_refine.cpp`, `--path-refine`, 기본 꺼짐)**: `plan_segment_path()` 에서 구간별 A* 결과의 중간 경유점을 PSO 로 미세 조정한다. 규칙: 경유점 **개수와 목표점은 바꾸지 않는다** (`segment_wp_bounds_` 진행도 추적 때문), 동적 장애물 회피 경로에는 적용하지 않는다 (거리 장이 정적 맵 기준), 충돌 샘플이 A* 보다 늘면 원본을 쓴다. 거리 장(`ClearanceField`)은 executor 가 lazy 생성하고 `set_map()` 에서 버린다. seed 는 `--seed` 와 로봇 id 에서 파생.
- **`tools/`**: 별도 실행 파일(`send_order`, `place_obstacle`, `obstacle_zone`)로 `schemas.o` 만 링크. `mqtt_once.hpp` 는 1회 발행 헬퍼.

## 주의

- `maps/` 는 `AMR_master/maps/` 의 수동 복사본이다. 원본이 바뀌면 직접 재복사.
- 맵 이미지 불일치: `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png` 인데, 예전 뷰어 사본은 `grid.png` 를 썼다. 뷰어 표시가 예전과 다를 수 있다 (미결정).
- 알려진 미해결 문제는 README 의 known issue (갈림길 왕복, 로봇끼리 장애물 미인식). PSO 는 이걸 해결하지 않는다 — 장애물 기억(시간 감쇠)이 필요하다.
- Makefile 은 소스 자동 수집을 안 한다. `.cpp` 추가 시 `CORE_SRC` 에 직접 넣는다.
- 셸이 zsh 라 변수 단어 분리가 안 된다 (`$VAR` 로 인자 목록 전개 불가, `${=VAR}` 사용).
- `launch.sh`/`Makefile` 주석의 "CLAUDE.md 결정 4", "최종 타깃" 은 원래 org 상위 프로젝트 문서 참조다. 이 리포에는 해당 내용이 없다.
- `.gitignore`: `*.o`, `*.d`, `robot_client`, `tools/` 바이너리, `map_cache/`, `logs/`, `.run/`.
- 코드 주석/README 는 한국어. 같은 스타일을 따른다.
