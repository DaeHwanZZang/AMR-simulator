# AMR_client — 로봇 클라이언트 (C++)

프로토콜(MQTT+JSON)이 언어 무관이라 `fms_server` 와 그대로 섞어 쓴다.

## 리포 구성

| 경로 | 내용 |
|---|---|
| (루트) `src/ include/ tools/ maps/` | C++ 로봇 클라이언트 (아래 문서) |
| `viewer/` | 라이브 플릿 뷰어 (Python/Qt) -- `viewer/README.md` |
| `broker/` | 개발용 mosquitto 설정 -- `broker/README.md` |

세 폴더는 MQTT 로만 통신한다. `maps/` 는 클라이언트와 뷰어가 공유한다.
빠른 실행: `mosquitto -c broker/mosquitto.conf -v` -> `./robot_client ...` ->
`cd viewer && .venv/bin/python tools/fleet_monitor_qt.py ../maps/warehouse/warehouse.json`

## 빌드

### macOS

```bash
brew install mosquitto cjson qt
make                                 # robot_client + tools/{send_order,place_obstacle,obstacle_zone}
```

### Ubuntu (배포 타깃)

```bash
sudo apt install clang build-essential libmosquitto-dev libcjson-dev qt6-base-dev pkg-config jq curl unzip
make
```

- `clang`/`build-essential`: `Makefile` 이 `CXX := clang++` 로 고정돼 있어 필요
  (g++로 바꾸려면 `Makefile` 의 `CXX` 줄을 직접 수정 — 자동 분기 없음)
- `libmosquitto-dev`/`libcjson-dev`: 브로커 패키지(`mosquitto`)나 런타임
  라이브러리(`libmosquitto1`/`libcjson1`)만 깔려 있으면 헤더가 없어 `mosquitto.h`/
  `cjson/cJSON.h` not found 로 빌드 실패한다
- `qt6-base-dev`: `--gui` 제어판(Qt Widgets)용. 없으면
  `QtWidgets/QApplication` not found
- `pkg-config`: Linux 분기가 `pkg-config --cflags/--libs Qt6Widgets` 로 Qt
  경로를 잡는 데 씀 (macOS 는 프레임워크라 불필요)
- `jq`: 빌드엔 안 쓰지만 `launch.sh` 가 맵 폴더 안 메타데이터 json 자동탐지에
  쓴다. 없으면 원인 설명 없이 "메타데이터 json 을 못 찾음" 만 뜬다
- `curl`/`unzip`: 빌드엔 안 쓰지만 `map_sync.cpp` 가 SET_MAP 번들 다운로드에
  fork+exec 로 호출. `unzip` 없으면 다운로드는 되는데 압축해제 단계서 조용히
  실패 → Map Sync 상태 FAILED, `State.map_version` 계속 `None` 으로 보고돼
  FMS 가 job dispatch 를 422(맵 버전 불일치)로 계속 거절한다

cmake 없이 순수 Makefile (`Makefile` 상단 `BREW`/`UNAME_S` 로 macOS/Linux 분기).

## 실행

```bash
./robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y 6.0
./robot_client --id AMR-002 --x 0 --y 0 --gui    # 제어판 창(Qt)
./robot_client --id AMR-003 --host <FMS IP>      # --map 없이: FMS 가 SET_MAP 으로 맵을 내려준다
./launch.sh                                       # 인터랙티브 실행기 — N대 한번에
```

옵션은 `./robot_client --help`. 브로커 주소는 `--host`/`--port` (기본
localhost:1883). `launch.sh`는 이 폴더 안 `maps/`에서만 맵을 찾는다 —
`AMR_master/maps/` 갱신되면 여기도 수동 재복사 필요.

## 맵 동기화 (SET_MAP 다운로더)

FMS 에 로봇이 등록돼 있으면, 접속 후 FMS 가 `fms/v1/{id}/instant` 로 `SET_MAP`
(params: `map_id`=map_uid, `map_version`, `url`) 을 보낸다. 로봇은:

1. `url` 로 번들 zip 을 HTTP GET (`curl`), `--map-cache DIR` (기본 `map_cache/`)
   밑 `<map_uid>/` 에 압축해제 (`unzip`). manifest 의 version 이 이미 캐시에 있는
   것과 같으면 다운로드 생략
2. `<map_uid>/fms_map.json` 으로 `MapModel` 리로드 → 현재 주행 맵 교체 → 현재
   위치 기준 A\* 재계산
3. `State.map_id`(=map_uid)/`map_version` 을 보고. FMS 는 이 값이 배정 맵의 활성
   버전과 같은 로봇에게만 job 을 준다

다운로드는 백그라운드 스레드에서 돈다 — tick 루프(주행/상태보고)를 막지 않고,
실패해도 로봇은 기존 맵으로 계속 돈다. `--gui` 의 **Map Sync** 패널에서 상태
(NONE/SYNCING/SYNCED/FAILED)·map_uid·version·에러를 보고 **Re-sync** 로 재시도.
`curl`/`unzip` 은 fork+exec (shell 안 거침). `map_cache/` 는 gitignore.

## GUI 스레드 모델

Qt 는 메인스레드 전용이라 `--gui` 주면 tick/MQTT 루프를 백그라운드 스레드로
보내고, `CommandQueue`(GUI→루프)와 `std::mutex`(공유 상태 읽기/쓰기)로 보호한다.

## 구성

| 영역 | 파일 |
|---|---|
| 프로토콜/맵 계약 | `include+src/schemas.*`, `map_model.*` |
| 경로계획/오더 실행/통신/시계 | `include+src/{planner,executor,comm,clock}.*` |
| 물리 시뮬레이션 | `include+src/{kinematics,battery,dynamic_obstacles}.*` |
| 로컬 제어판 | `gui.hpp/cpp` (Qt Widgets) |
| CLI 도구 | `tools/*.cpp` |
| SET_MAP 번들 다운로더 | `map_sync.hpp/cpp` |

기능 범위: MQTT(LWT/재접속), 오더 상태머신(released/order_update_id/거절 규칙),
4축 상태, e-stop/PAUSE, A* 경로계획, 사다리꼴 운동학, 배터리, 동적 장애물 회피,
충돌 감지, 강제 재위치, Qt 로컬 제어판.


## known issue
 - 경로 결정장애 이슈
    A to B에 2가지 갈림길이 존재할 때 로컬 패스플래닝은 동적 장애물을 센서 인식 범위 내에서만 기억하기 때문에 2가지 갈림길에 모두 동적 장애물이 존재할 경우 왔다 갔다 무한 반복하는 현상 발생함
    대응 방안: 동적 장애물을 일정 시간 메모리에 기록함. (맹점: 말그대로 동적 장애물 이기 때문에 메모리 시간을 어떻게 부여할지 고민 필요)

- 로봇 자체를 장애물로 인식 필요
    실제 환경에서는 다수의 로봇이 서로 장애물로 인식해야함. 물론 fms의 관제를 통해 부딪히는 상황이 없어야 하지만 시뮬레이션 자체에서 로봇이 겹쳐지는 상황은 프로그램 자체의 신뢰성을 떨어뜨림
