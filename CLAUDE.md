# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

AMR 로봇 클라이언트 (C++20). 프로세스 1개 = 로봇 1대. MQTT+JSON 프로토콜로 FMS(`fms_server`), `MQTT_broker`, `AMR_viewer` 와 통신한다. Python 원본(`robot_client/*.py`)의 포팅이라 주석에 원본 참조가 많다. 이 폴더는 독립 빌드/실행 가능해야 한다 (`maps/` 사본 자체 보유, 상위 폴더 참조 금지).

## 명령 (리포 루트에서 실행)

```bash
make                  # robot_client + tools/{send_order,place_obstacle,obstacle_zone}
make clean
./robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y 6.0
./robot_client --id AMR-002 --x 0 --y 0 --gui     # Qt 제어판
./robot_client --id AMR-003 --host <FMS IP>       # --map 없음: FMS 가 SET_MAP 으로 맵 전달
./launch.sh           # 인터랙티브 N대 실행 (PID 는 .run/pids, 로그는 logs/)
./stop.sh             # launch.sh 가 띄운 로봇 종료
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
- **`tools/`**: 별도 실행 파일(`send_order`, `place_obstacle`, `obstacle_zone`)로 `schemas.o` 만 링크. `mqtt_once.hpp` 는 1회 발행 헬퍼.

## 주의

- `maps/` 는 `AMR_master/maps/` 의 수동 복사본이다. 원본이 바뀌면 직접 재복사.
- `.gitignore`: `*.o`, `*.d`, `robot_client`, `tools/` 바이너리, `map_cache/`, `logs/`, `.run/`.
- 코드 주석/README 는 한국어. 같은 스타일을 따른다.
