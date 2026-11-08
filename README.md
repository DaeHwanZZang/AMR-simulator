# AMR_client — 로봇 클라이언트 (C++)

프로토콜(MQTT+JSON)이 언어 무관이라 `fms_server`(별도 리포 AMR-fms)와 그대로 섞어 쓴다.

## 리포 구성

| 경로 | 내용 |
|---|---|
| `robot/` | C++ 로봇 클라이언트 (`src/ include/ tools/ tests/ Makefile`, 아래 문서). 빌드 산출물은 `build/` |
| `tools/` | Python 도구: 통합 런처, 맵 생성/정리/편집기 |
| `maps/` | 맵 (클라이언트와 뷰어가 공유) |
| `scripts/` | `launch.sh`/`stop.sh` (터미널 대화형 N대 실행/종료) |
| `viewer/` | 라이브 플릿 뷰어 (Python/Qt) -- `viewer/README.md` |
| `broker/` | 개발용 mosquitto 설정 -- `broker/README.md` |

세 폴더는 MQTT 로만 통신한다. `maps/` 는 클라이언트와 뷰어가 공유한다.
**가장 쉬운 시작: 리포 루트에서 `./launcher`** (브로커/로봇/뷰어/맵 도구/빌드/테스트를 한 창에서 실행. `.venv` 는 없으면 자동으로 만든다).

수동 실행: `mosquitto -c broker/mosquitto.conf -v` -> `build/bin/robot_client ...` ->
`cd viewer && .venv/bin/python tools/fleet_monitor_qt.py ../maps/warehouse/warehouse.json`

## 빌드

### macOS

```bash
brew install mosquitto cjson qt
robot/tools/setup_mujoco.sh                # 최초 1회: MuJoCo 를 robot/third_party/mujoco/ 에 설치 (brew 패키지가 없다)
make -C robot                       # build/bin/{robot_client,send_order,place_obstacle,obstacle_zone}
```

### Ubuntu (배포 타깃)

```bash
sudo apt install clang build-essential libmosquitto-dev libcjson-dev qt6-base-dev pkg-config jq curl unzip python3-pip
robot/tools/setup_mujoco.sh                # 최초 1회: MuJoCo 를 robot/third_party/mujoco/ 에 설치
make -C robot
```

- MuJoCo(`robot/tools/setup_mujoco.sh`): PyPI 휠에서 C 헤더와 공유 라이브러리만 꺼내 `robot/third_party/mujoco/`
  (gitignore) 에 둔다. `python3 -m pip` 가 필요하다. 다른 곳에 있으면 `make MUJOCO_DIR=/경로`.
  없이 `make -C robot` 하면 `MuJoCo 가 없다 ... robot/tools/setup_mujoco.sh 를 실행하라` 로 멈춘다. 휠은 macOS(arm64/x86_64)와
  Linux(x86_64/aarch64) 용이 있고, 이 스크립트는 macOS 에서만 실행해 봤다 (Linux 는 미검증).
- `clang`/`build-essential`: `robot/Makefile` 이 `CXX := clang++` 로 고정돼 있어 필요
  (g++로 바꾸려면 `robot/Makefile` 의 `CXX` 줄을 직접 수정 — 자동 분기 없음)
- `libmosquitto-dev`/`libcjson-dev`: 브로커 패키지(`mosquitto`)나 런타임
  라이브러리(`libmosquitto1`/`libcjson1`)만 깔려 있으면 헤더가 없어 `mosquitto.h`/
  `cjson/cJSON.h` not found 로 빌드 실패한다
- `qt6-base-dev`: `--gui` 제어판(Qt Widgets)용. 없으면
  `QtWidgets/QApplication` not found
- `pkg-config`: Linux 분기가 `pkg-config --cflags/--libs Qt6Widgets` 로 Qt
  경로를 잡는 데 씀 (macOS 는 프레임워크라 불필요)
- `jq`: 빌드엔 안 쓰지만 `scripts/launch.sh` 가 맵 폴더 안 메타데이터 json 자동탐지에
  쓴다. 없으면 원인 설명 없이 "메타데이터 json 을 못 찾음" 만 뜬다
- `curl`/`unzip`: 빌드엔 안 쓰지만 `map_sync.cpp` 가 SET_MAP 번들 다운로드에
  fork+exec 로 호출. `unzip` 없으면 다운로드는 되는데 압축해제 단계서 조용히
  실패 → Map Sync 상태 FAILED, `State.map_version` 계속 `None` 으로 보고돼
  FMS 가 job dispatch 를 422(맵 버전 불일치)로 계속 거절한다

cmake 없이 순수 Makefile (`robot/Makefile` 상단 `BREW`/`UNAME_S` 로 macOS/Linux 분기).

## 실행

```bash
build/bin/robot_client --id AMR-001 --map maps/warehouse/warehouse.json --x 1.2 --y -4.0
build/bin/robot_client --id AMR-002 --x 0 --y 0 --gui    # 제어판 창(Qt)
build/bin/robot_client --id AMR-003 --host <FMS IP>      # --map 없이: FMS 가 SET_MAP 으로 맵을 내려준다
scripts/launch.sh                                       # 인터랙티브 실행기 — N대 한번에
```

옵션은 `build/bin/robot_client --help`. 브로커 주소는 `--host`/`--port` (기본
localhost:1883). `scripts/launch.sh`는 리포 루트 `maps/`에서 맵을 찾는다.

### 로컬라이징 / 충전기 / 배터리

- 기본: 시작 자세에서 바로 LOCALIZED. `--require-localization` 이면 UNLOCALIZED 로 떠서 위치 힌트 전에는 오더를 거절한다.
  ```bash
  build/bin/robot_client --id AMR-001 --map maps/empty_hall1/fms_map.json --x 3 --y 3 --require-localization
  build/bin/send_order AMR-001 --instant INIT_POSITION --param x=3 --param y=3 --param theta=0   # ±1.5 m, ±30° 안
  ```
  제어판(`--gui`)에는 **Pose Hint**(같은 동작)와 **Force Localize**(디버그: 그 자리로 옮기고 바로 LOCALIZED)가 있다.
  `State.localization` = `UNLOCALIZED | LOCALIZING | LOCALIZED`, `localization_score` 는 가상 라이다 스캔 매칭 점수 (0.8 미만이면 `LOCALIZATION_FAILED` 경고). 뷰어는 위치를 모르는 로봇을 흐린 빨간 점선으로 그린다.
- 충전기: 맵 폴더의 `chargers.json` (예: `maps/empty_hall1/chargers.json`). 이 파일이 있는 맵에서만 DOCK 이 충전기 위치·진입 방향·자세를 검사하고 어긋나면 `DOCK_FAILED`.
- 배터리: `--battery-low`(기본 20, 경고), `--battery-critical`(기본 5, 이후 DOCK 오더만 받음).
- 복도 중앙 주행(기본 켜짐): `--no-center-path`, `--center-weight`, `--center-reach`. 재현용 `--seed`, `--obstacle-seed`, `--map-cache`.
- 맵 `annotations.json` 의 점유 구역(occupancy)과 노드 `allowed_zone_ids` 가 있으면 허가 없는 구역을 피하고, 직선 이동이 막히면 `TRAFFIC_BLOCKED`.

### 직선 이동 / 도킹 / 주차 액션

오더 노드의 `action` 으로 지정한다. 셋 다 `params` 가 필수이고, 값이 틀리면 로봇이 오더를 통째로 버린다
(`send_order` 가 보내기 전에 같은 검증을 한다).

| action_type | params | 동작 |
|---|---|---|
| `STRAIGHT` | `distance` (m, + 전진 / - 후진), `speed` (선택, 기본 0.2 m/s) | 노드에 도착해 노드 theta 로 정렬한 뒤, 경로계획 없이 그 방향으로 직선 이동 |
| `DOCK` | `entry` (`FORWARD`/`BACKWARD`), `undock_distance` (m), `min_charge` (%), `speed` (선택) | 충전기에 진입해 충전하며 대기 |
| `PARK` | `entry`, `undock_distance`, `speed` (선택) | 정밀 주차 후 대기 (충전 없음) |

- DOCK/PARK 의 노드 position 은 **진입을 마친 자세**다. 로봇은 그 앞 `undock_distance` 지점까지
  A* 로 가서 theta 로 정렬하고 직선으로 들어간다. `FORWARD` 는 전진 진입, `BACKWARD` 는 후진 진입이다.
  충전기가 벽에 붙어 있어 A* 로는 못 들어가는 칸이라도 된다 (진입 직선은 팽창 검사를 안 한다). 다만 물리 몸체
  (반지름 `width/2 - 0.04`)가 실제로 들어갈 자리여야 한다. 벽에 `0.21 m` 보다 가깝게 찍으면 벽에 눌려
  COLLISION 이다. 진입 시작 지점은 A* 가 닿는 곳이어야 하니 `undock_distance` 를 충분히 준다.
- 빠져나오기(언도킹)는 별도 액션이 없다. 도킹/주차 상태에서 이동이 필요한 오더를 받으면 먼저 진입했던
  직선을 되짚어 나온다. 전진 진입했으면 후진으로, 후진 진입했으면 전진으로 나온다. 진입이나 언도킹 도중
  취소돼도 다음 오더에서 먼저 빠져나온다.
- DOCK 후 충전량이 `min_charge` 미만이면 state 가 `CHARGING` 이고 새 job 을 거절한다. 넘으면 `IDLE` 로
  바뀌어 job 을 받는다 (충전은 언도킹 전까지 계속, `battery.charging=true`).
- 직선 이동 중에도 충돌 감지(COLLISION), 동적 장애물 대기(우회는 안 함), PAUSE/e-stop 정지·재개가 그대로 된다.
  강제 위치지정을 하면 도킹 상태가 풀린다.
- 마커 보정은 시뮬레이션하지 않는다. 도착 오차는 물리가 만든다: 직선 이동은 진행 방향 1 cm 이내, 옆 방향은
  목표 방위로 조향해서 보통 수 mm 이내다 (아래 물리 절).

```bash
# 리포 루트에서. theta 는 --path 의 세 번째 값 (라디안)
build/bin/send_order AMR-001 --path 3,1.5,0 --order-id s1 --action STRAIGHT --param distance=-1.0
build/bin/send_order AMR-001 --path 14.75,5,0 --order-id d1 --action DOCK \
    --param entry=BACKWARD --param undock_distance=0.6 --param min_charge=80
build/bin/send_order AMR-001 --path 14.75,5,3.14159 --order-id p1 --action PARK \
    --param entry=FORWARD --param undock_distance=0.6
```

### 텍스트 스크립트로 job 만들기 (`send_order --script`)

FMS 없이 동작을 텍스트 파일에 순서대로 적으면 Order 하나(= job 하나)로 묶어 발행한다. 로봇 쪽은 달라지지 않는다.

```
// scripts/examples/demo_job.txt
0 START
1 MOVE 6 3 0          // x y theta(rad). "90deg" 도 가능, 생략하면 0
2 WAIT 2              // 직전 MOVE 위치에서 2초
3 PICK 3              // 같은 자리에서 액션을 연달아 쓰면 같은 좌표 노드가 자동으로 복제된다
4 MOVE 6 8 1.57
5 DROP 3
6 END
```

- 명령: `MOVE x y [theta]`, `PICK|DROP|CHARGE|WAIT [초] [key=value ...]`, `STRAIGHT|DOCK|PARK key=value ...`, `START`, `END`.
  액션은 직전 `MOVE` 노드에 붙는다 (첫 액션 앞에는 MOVE 필요). `//` `#` 뒤는 주석. END 뒤 명령은 오류.
- 반복: 줄 앞 번호가 라벨이다. `50 GOTO 1 3` 은 1번 줄로 3번 점프한다(총 4바퀴). 파일을 읽을 때 노드로 펼쳐 Order 하나로 보내므로
  **횟수는 필수**다 (횟수 없는 무한 반복은 오류, 오래 돌리려면 큰 수를 쓴다). 노드 5000개까지. 예시 `scripts/examples/loop_job.txt`.
- 경로계획은 현재 목적지까지만 한다: `A -> B -> C` 면 A->B 만 계획하고 B 에 도착해 정지·theta 정렬한 뒤 B->C 를 계획한다 (C 가 막혀 있어도 B 까지는 간다).
- 잘못된 줄은 줄 번호와 함께 오류를 내고 발행하지 않는다 (액션 인자는 로봇과 같은 검증).

```bash
build/bin/send_order AMR-001 --script scripts/examples/demo_job.txt --dry-run   # 발행 없이 컴파일 결과만 출력
build/bin/send_order AMR-001 --script scripts/examples/demo_job.txt             # order_id 기본값 = 파일명
```

### 물리 시뮬레이션 (MuJoCo)

로봇 이동은 MuJoCo 물리 엔진이 계산한다 (기본 동작이고 끌 수 없다). 바퀴 슬립, 관성, 벽과의 접촉이 그대로 나온다.

```text
오더 -> A* 경로 -> 사다리꼴 이상 궤적 -> (v, w) -> 바퀴 속도 서보 -> MuJoCo (2 ms x 25 / tick) -> 실제 자세
                      ^---------- 매 tick 실제(측정) 자세에서 다시 출발 ----------------------'
```

- **이상 궤적은 명령 생성기다.** 기존 `TrapezoidalKinematics` 가 측정 자세에서 한 걸음 낸 변위를 몸체 목표 속도 (v, w) 로
  바꿔 바퀴에 준다. 실제가 못 따라가면 다음 tick 에 그 자리에서 다시 계산한다 (폐루프).
- **로봇은 자기 위치를 추정으로 안다 (참 자세를 모른다).** `State.pose`, 경로 추종, 도착 판정은 전부 추정 자세로 돈다.
  추정 = 바퀴 엔코더 적분(오도메트리) + 위치추정 보정이다. 엔코더는 슬립을 못 봐서 오도메트리는 틀어지고(저마찰이면 12 m 에
  1 m 이상), 보정이 그걸 고친다 (아래 "위치추정 보정"). 참 자세(GT)는 물리 엔진과 뷰어(`physics` 토픽)에만 있고, 벽 충돌 판정은
  참 자세 기준이다. `State.velocity` 는 실제 속력(크기)이다. 강제 재위치(GUI)는 오도메트리와 추정을 그 자세로 맞춘다.
- **몸체 모델**: 차동구동 + 캐스터 2개. 질량 60 kg (`--mass`), 바퀴 반지름 0.075 m, 바퀴 간격 `width - 0.15` m,
  바퀴 토크 한계 12 N·m, 바퀴-바닥 마찰 `--friction` (기본 0.8). 질량 분포는 `length x width x 0.2 m` 직육면체 기준이다.
- **충돌 형상은 원판이다 (반지름 `width/2 - 0.04`).** planner 가 로봇을 반지름 `width/2` 원으로 보고 팽창시키므로
  같은 가정이어야 계획된 경로에서 제자리 회전하다 벽에 부딪히지 않는다. `length` 는 State/뷰어 표시와 관성에만 쓰인다.
  직사각형으로 하면 폭 1 m 통로 모퉁이에서 회전 중 충돌이 났다 (실제로 확인).
- **벽**: FREE 가 아닌 칸(OCCUPIED/UNKNOWN/맵 밖)을 직사각형으로 병합해 상자로 만든다 (실제 맵 2347x1687 px 이 22,897 개, 8 ms).
  로봇 주변 8 m 창 안의 상자만 모델에 넣고, 창 중심에서 3 m 벗어나면 모델을 다시 컴파일한다 (상태는 이어받는다). 그래서 맵
  크기와 tick 비용이 상관없다 (실제 맵 주행 중 CPU 약 4%). 맵 그리드가 원본이고 물리 월드는 로드/`SET_MAP` 때마다 거기서 만든다.
- **충돌 판정**: 몸체-벽 접촉력이 20 N 이상으로 0.25 초 이어지면 FATAL `COLLISION` 이다 (로봇은 그 자리에서 선다).
  시작/강제 위치가 벽 안이면 (v1.3 의 "벽 위에서도 일단 뜬다") 몸체-벽 충돌을 꺼서 엔진이 로봇을 튕겨내지 않게 한다. 기록의 `ghost` 열.
- **정지와 도착**: 경로 끝은 실제로 멈추고 마지막 지점에서 3 cm 이내여야 도착이다 (못 닿으면 마지막 지점을 다시 건다).
  직선 이동(STRAIGHT/DOCK/PARK/언도킹)은 목표 방위로 조향하며 가고, 진행 방향 남은 거리 1 cm 이내에서 서면 끝이다.
  경로의 큰 꺾임(0.3 rad 초과)에서는 감속해 서서 돌고, 작은 꺾임은 서지 않고 조향하며 지나간다.
- **PAUSE/e-stop/강제 정지는 즉시 선다.** 오더 취소나 대기 중에 굴러가고 있으면 `max_deceleration` 으로 감속해 선다.
- **tick 의 dt 는 0.1 초로 자른다.** A* 가 루프를 수백 ms 막았다 돌아와도 로봇이 한 번에 튀지 않는다 (막혀 있던 시간은 건너뛴다).
- **서보 게인은 6 으로 낮게 잡았다.** 60 이면 접촉 compliance 와 얽혀 정지 후에도 몇 cm/s 로 계속 떨렸다.

슬립은 마찰이 낮거나 가속 요구가 마찰 한계를 넘을 때 생긴다. 예: `--friction 0.2 --max-acceleration 1.0`. 정상 마찰(0.8)에서는
슬립률 중앙값이 0.001 이고 추정 오차(오도메트리 대 참)는 12 m 에 2~15 cm 정도다 (제자리 회전 때 바퀴 접촉 때문에 몸체가 오도메트리보다 3~4% 더 돈다).
한계: `--friction 0.12 --max-acceleration 3` 처럼 극단이면 제자리 회전이 접지력을 넘어 방위가 ±0.5 rad 로 헌팅하며 거의 못 간다.
마찰 0.15~0.5, 가속 0.6~2.0 은 완주한다.

**위치추정 보정 (라이다 시뮬레이션, 기본 켜짐)**: 주기마다 "라이다 측정" `z = GT + 가우시안 노이즈` 를 받아 추정을 z 쪽으로
`gain` 만큼 당긴다 (AMCL 이 map->odom 변환을 고치는 구조: `추정 = T ∘ 오도메트리`, `T` 를 갱신). 한 번에 고치는 양은
0.15 m / 0.1 rad 로 묶어 추정이 튀지 않게 하고, 측정은 가끔 빠진다. 보정은 GT 를 노이즈 낀 측정값으로만 쓴다 (로봇이 GT 를 직접 알지 못한다).

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--loc-rate HZ` | 5 | 보정 주기. **0 이면 보정 없는 순수 오도메트리** (오차 누적 시험용) |
| `--loc-noise-xy M` | 0.02 | 측정 노이즈 위치 표준편차 |
| `--loc-noise-theta RAD` | 0.01 | 측정 노이즈 방위 표준편차 |
| `--loc-gain K` | 0.15 | 한 번에 측정 쪽으로 당기는 비율 (0~1) |
| `--loc-dropout P` | 0.05 | 측정이 빠질 확률 |

`--friction 0.3` 로 4 개 노드를 도는 시험에서 보정이 켜져 있으면 추정 오차가 최대 3 cm 였고 (보정 전 오도메트리는 1 m 이상 벌어짐),
`--loc-rate 0` 이면 추정이 그대로 벌어진다. 낮은 마찰 + 보정 주기가 느리면 제자리 회전에서 보정 지연 때문에 추정이 흔들릴 수 있다 (5 Hz, gain 0.15 는 μ 0.3 까지 완주).
기록 CSV 에는 `x y theta` (GT), `odom_*` (보정 전), `est_*` (보정된 추정) 가 모두 있다. 노이즈에도 경유점 방위가 흔들리지 않도록
`kinematics.cpp` 가 5 cm 안의 경유점은 건너뛰고 2 cm 안이면 방위를 유지한다.

**뷰어 물리 디버그**: 로봇이 `fms/v1/{id}/physics` 토픽으로 tick 마다(20 Hz) 물리 관측값을 발행한다. `State` 는 `extra=forbid` 라
필드를 더할 수 없어서 따로 뺐고, 프로토콜이 아니라서 FMS 와 맞출 필요가 없다 (뷰어만 읽는다). 내용: 실제 자세/속도(`v_fwd`, `v_lat`, `yaw_rate`),
바퀴 속도/목표/슬립, 오도메트리(보정 전)/추정(보정 후) 자세, 위치추정 설정과 통계, 몸체 목표 속도, 벽 접촉력, ghost/collided/at_rest, 마찰/질량/한계, 남은 경로 길이와 목표.
Qt 뷰어는 이걸로 자세를 외삽해 부드럽게 그리고(30 fps), 왼쪽 패널(`p` 토글)에 값을 띄우며, 지나온 궤적(`l`), **GT(참 위치, 오차 없음)**
(청록 윤곽 + 십자 + "GT" 라벨 + 청록 점선 궤적, `o`)와 보정 전 순수 오도메트리(노란 점선 윤곽, `i`)를 그린다.
몸체는 로봇이 보고한 위치(`State.pose` = 보정된 추정)이고,
GT 와 벌어진 만큼이 누적 오차다 (패널 `error`). 보정 알고리즘의 기준값은 `physics` 토픽의 `x y theta` 또는 기록 CSV 의 `x y theta` 열이다, 속도 벡터(초록)와, 벽에 눌릴 때 빨간 GT 윤곽을 그린다. 계획 경로는 현재 구간 굵게 + 남은 구간 눈금선 + 경유점 점 +
목표 링과 남은 거리로 표시하고, 이미 지난 경유점은 뺀다. `k` 는 궤적과 피크값(최대 슬립/접촉력)을 지운다.
로컬 제어판(`--gui`)에도 같은 값(Physics 절)과 마찰/질량 설정칸이 있다. 제어판 레이아웃은 `AMR_GUI_SNAPSHOT=파일.png` 로
(`QT_QPA_PLATFORM=offscreen` 과 함께) 1.5 초 뒤 PNG 로 저장해 볼 수 있다.

**데이터 기록 (`--record DIR`)**: tick(50 ms) 마다 한 줄씩 `DIR/<id>_<시작시각>.csv` 에 쓰고, 설정은 `.meta.json` 에 둔다
(크기, 질량, 마찰, 최대 속도/가속도, 시작 자세, seed 등). 오더 하나가 에피소드이고 `order_id` 열로 나눈다 (비어 있으면 대기).

| 열 | 내용 |
|---|---|
| `t`, `wall_time` | 로봇 시작 후 시뮬레이션 시간(s), epoch |
| `state`, `sub_state`, `order_id`, `node_index`, `action` | 상태와 오더 진행 |
| `x y theta` | 실제 자세 (엔진 값) |
| `odom_x odom_y odom_theta` | 바퀴 엔코더만 적분한 자세 |
| `v_fwd v_lat yaw_rate` | 몸체 좌표 실제 속도 (전진 +, 왼쪽 +) |
| `cmd_v cmd_w` | 이상 궤적이 준 몸체 목표 속도 |
| `wheel_w_l/r`, `wheel_cmd_l/r` | 바퀴 각속도와 목표 (rad/s) |
| `slip_l slip_r` | 바퀴별 종방향 슬립률 (0 = 구름, ±1 이상 = 헛돎) |
| `wall_force`, `ghost` | 몸체-벽 최대 접촉력(N), 벽 안 시작 여부 |
| `battery`, `charging`, `detected_obstacles`, `error` | 배터리, 감지 장애물 수, 첫 오류 종류 |

물리가 아닌 것: 동적 장애물은 사각형 좌표일 뿐 물리 객체가 아니다 (로봇이 밀지 못하고 센서 반경으로만 인식). 로봇끼리는
프로세스가 따로라 서로 부딪히지 않는다. 라이다/IMU 센서는 없다.

디버깅: `AMR_PHYSICS_DUMP=파일` 환경변수를 주면 마지막으로 컴파일한 MJCF 를 그 파일에 쓴다 (파이썬 `mujoco` 로 열어 볼 수 있다).
통합 테스트: `.venv/bin/python robot/tests/physics_test.py [시나리오...]` (empty_hall1 에서 이동/직선/도킹/충돌/일시정지/
벽 안 시작/저마찰/장거리/동적 장애물, 약 8분).

### 통합 런처 (`tools/launcher.py`)

브로커, 로봇 클라이언트, 뷰어, 맵 편집기, 맵 생성기를 한 창에서 켜고 끈다.

```bash
# 리포 루트에서 (최초 1회: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt)
.venv/bin/python tools/launcher.py
```

- 각 프로그램은 별도 프로세스로 뜨고 자기 창은 따로 뜬다. 출력은 `logs/<이름>.log` 에 남고 화면에는 마지막 2000줄만 보인다.
- 위쪽 브로커 host/port/맵은 공통 값이다. 로봇·뷰어는 브로커가 꺼져 있으면 먼저 켤지 묻는다.
- `추가 ▾` 에서 로봇(대수 지정, 이름은 AMR-NNN 자동, y 는 대수마다 1 m 씩 증가), 맵 편집기(선택한 맵, 맵당 1개), 빌드(make)를 추가한다.
- 정지는 SIGTERM 이다. 맵 편집기는 저장 안 한 수정이 있으면 확인 창이 뜨고, 제한 시간(편집기 30 초)이 지나면 강제 종료할지 묻는다.
- 런처를 닫을 때 "전부 정지 / 그대로 두고 종료 / 취소" 를 고른다. 그대로 두고 종료하거나 비정상 종료하면 다음 실행 때 `.run/launcher.json` 으로 프로세스를 다시 찾아 붙는다.
- 로봇 시작 위치가 벽/미탐색이어도 실행은 된다 (`start position ... is not drivable` 경고). 대신 오더를 받으면 `NO_PATH` 오류가 나고, 제어판(GUI)의 강제 위치지정으로 주행 가능한 칸에 옮기면 `NO_PATH`/`COLLISION` 오류가 풀린다. 레이아웃 맵은 칸 (r,c) 중심이 world ((c+1) m, (r+1) m).
- 새 툴은 `tools/launcher_tools.json` 에 항목만 추가하면 붙는다 (파일 안 `_설명` 참고). `scripts/launch.sh`/`scripts/stop.sh` 는 따로 그대로 쓸 수 있다 (`.run/pids` 는 런처가 쓰지 않는다).

## 테스트 맵 생성 (`tools/gen_map.py`)

손으로 그린 맵(`maps/warehouse`)은 너무 정형화돼 있어서, 실제 SLAM 맵
(`maps/641931.../grid.png`) 에서 1 m 타일 조각을 잘라 레이아웃대로 이어붙인다.
점선 벽, 벽 너머 회색(미탐색), 레이저 줄무늬가 원본 그대로 남는다.

```bash
python3.11 -m venv .venv && .venv/bin/pip install numpy pillow   # 리포 루트. viewer/.venv 를 써도 된다
.venv/bin/python tools/gen_map.py tools/map_layouts/loop.txt --out maps/gen_loop --seed 1
build/bin/robot_client --id AMR-001 --map maps/gen_loop/fms_map.json --x 1.0 --y 1.0
```

- 레이아웃은 텍스트 한 글자 = 1 m: `.` 통로, `#` 막힘, `;` 주석. 바깥 1칸은 자동으로 막힌다
- 출력 파일: `grid.png`(생성 결과, 다시 생성하면 덮어씀), `edited_navi_gridmap.png`(편집기/FMS 웹 UI
  수정용. **최초 생성 때만** `grid.png` 복사본으로 만들고 이후엔 건드리지 않음 -- 새 레이아웃으로 다시
  만들려면 이 파일을 지우고 생성), `fms_map.json`(image 가 `edited_navi_gridmap.png` 를 가리킴)
- 여백: 사방에 `--margin-tiles`(기본 10 = 320 px = 10 m) 만큼 회색(미탐색) 여백. 검사는 여백 전 이미지로 한다
- 좌표: origin = (-여백 m, -여백 m) 이라 여백과 상관없이 레이아웃 (행 r, 열 c) 칸 중심 =
  world (c+1, r+1) m. 칸 중심 좌표는 전부 양수라 `send_order --path` 로 바로 보낼 수 있다
  (`send_order` 는 음수 좌표를 못 받음)
- 생성 후 planner 와 같은 규칙(비-FREE 를 로봇 반폭만큼 팽창)으로 모든 통로 칸이
  주행 가능하고 하나로 이어지는지 검사한다. 실패하면 seed 를 바꿔 `--tries` 번 재시도
- 원본에 모서리 조각이 드물어서 일부 모서리는 일자벽 조각 두 장으로 합성된다.
  타일 경계(1 m 격자)에서 줄무늬가 끊기는 건 남아 있다 — 표시 문제, 주행 영향 없음
- 대각선으로만 맞닿은 통로는 이어지지 않는다 (planner 도 못 지나간다)

### 원본 없이 한 번에 그리기 (`tools/synth_map.py`)

조각을 이어붙이면 조각 경계마다 벽 위치와 노이즈가 어긋난다. `synth_map.py` 는 같은
레이아웃으로 맵 전체에 벽(경계 양쪽 1 px 씩 = 32 px 타일의 16, 17 번째 픽셀)을 긋고
질감 노이즈를 맵 단위로 깔아서 이음매가 없다. 좌표, 검사, 출력 파일은 `gen_map.py` 와 같다.

```bash
.venv/bin/python tools/synth_map.py tools/map_layouts/loop.txt --out maps/synth_loop --seed 1   # 리포 루트에서
```

- 질감: 통로 247~255 저주파, 벽 20~60, 벽에서 `--fringe-px`(기본 2) 안에 회색 테두리
  (`--fringe-prob`). 통로 쪽 테두리는 비-FREE 라 planner 기준 통로가 테두리 폭만큼 좁아진다
- 불규칙성 (기본 켜짐, 0 으로 끄면 실선 직선 벽):
  - `--wobble-px 1.0` 벽 흔들림. 0 이 아니면 벽이 정확히 16, 17 번째 픽셀이 아니다
  - `--dash-gap 0.25 --dash-px 5` 점선 벽. 벽 픽셀의 25% 를 128 로 끊는다 (128 도 비-FREE 라 주행 영향 없음)
  - `--fur-prob 0.06 --fur-len 10` 털. 끊긴 곳에서 막힘 쪽으로 가는 1 px FREE 선. planner 팽창으로 지워진다
  - `--ray-prob 0.012 --ray-len 150` 긴 광선. 끊긴 곳에서 부채꼴로 최대 150 px, 일부는 옅은 회색.
    여백도 같이 그려서 회색 영역으로 길게 뻗는다 (통로/다른 벽을 만나면 멈춤). 검사는 여백을 잘라낸 이미지로 한다

### 생성 GUI (`tools/map_gen_gui.py`)

레이아웃 편집, 방식 선택(합성 / 실제 조각), 옵션 입력, 생성 버튼, 결과 미리보기와 로그가
한 창에 있다. 내부에서는 `synth_map.py` / `gen_map.py` 를 그대로 실행하므로 CLI 와 결과가 같다.
`seed 랜덤 재생성` 버튼은 옵션은 그대로 두고 seed 만 무작위로 바꿔 바로 다시 생성한다 (덮어쓰기 확인 없음).
마지막으로 쓴 옵션과 레이아웃 텍스트는 `QSettings` 로 저장돼 다음 실행 때 복원된다 (생성 시, 창 닫을 때 저장).

```bash
.venv/bin/pip install PySide6                 # 리포 루트. numpy, pillow 와 같이 필요
.venv/bin/python tools/map_gen_gui.py         # 기본 출력 maps/gui_map (이미 있으면 덮어쓸지 묻는다)
```

### 맵 편집기 (`tools/map_editor.py`)

`grid.png` 위에 흰색/검정/회색 도형(사각형·선분·자유형)을 얹어 `edited_navi_gridmap.png` 를 만든다.
이미지에 직접 그리지 않고 도형을 world 좌표(m)로 `edits.json` 에 저장하므로, 다시 열어 도형을 선택·이동·꼭짓점 수정·
색/굵기 변경·삭제할 수 있고 줌해도 모양이 그대로다. 색은 255/0/128 세 값뿐이고 안티앨리어싱이 없다 (중간 회색값은 UNKNOWN 장애물이 되기 때문).

- 맵 폴더에 `grid.png` 와 `edited_navi_gridmap.png` 가 둘 다 없으면 경고를 띄우고 시작하지 않는다.
- 저장은 Ctrl+S 수동뿐이다. `edits.json` + `edited_navi_gridmap.png` 를 쓴다. `fms_map.json` 은 원점을 바꿨을 때만 `origin_x`/`origin_y` 두 값을 고친다 (저장 전 확인).
- **원점 지정 (O)**: 클릭한 지점(가장 가까운 픽셀 모서리)을 world (0, 0) 으로 만든다. 이미지와 도형은 그대로이고 좌표계만 옮겨진다 (도형 world 좌표는 같이 평행이동, 실행 취소 가능). 화면에 보라색 원 + x(빨강)/y(초록) 축으로 현재 원점이 표시된다. 원점이 바뀌면 로봇 시작 위치, 오더/구역 좌표처럼 이 맵의 world 좌표를 쓰는 값도 같이 바뀌어야 한다. 주의: 맵 생성기로 같은 폴더에 다시 생성하면 origin 이 -여백 으로 되돌아간다.
  저장 직후 주행 가능 영역(로봇 반폭 8 px 팽창 기준)이 늘어났으면 통로 끊김 경고를 띄운다.
- 디스크의 `edited_navi_gridmap.png` 가 `grid.png + edits.json` 렌더와 다르면(다른 도구로 수정) 첫 저장 때 덮어쓸지 묻는다.
- `gen_map`/`synth_map` 이 `grid.png` 를 다시 만들어도 `edits.json` 은 남는다. 새 `grid.png` 위에 같은 도형이 다시 얹힌다.
- 조작: V 선택 / R 사각형 / L 선분(Shift 15° 단위) / P 자유형, 휠 줌, 가운데·오른쪽 버튼 또는 Space+드래그 이동,
  F 전체 보기, Delete 삭제, Ctrl+Z / Ctrl+Shift+Z 실행 취소·다시 실행. `결과 미리보기` 는 저장될 실제 픽셀을 보여 준다.
- 렌더 규칙은 GUI 없는 `tools/map_edit_model.py` 에 있다 (FMS 웹 UI 도 같은 규칙을 쓰게 하려는 것). 로봇/뷰어는 시작할 때 이미지를 읽으므로 저장 후 재시작해야 반영된다.

```bash
.venv/bin/python tools/map_editor.py maps/gen_loop/fms_map.json   # 리포 루트에서 (PySide6, numpy, pillow)
```

### 노이즈 정리 (`tools/clean_map.py`)

생성 맵(또는 아무 맵)의 레이저 줄무늬, 벽 바깥 털, 통로 점을 지운다. 생성할 때
`--clean` 으로 붙이거나, 이미 있는 맵에 따로 돌린다 (리포 루트에서):

```bash
.venv/bin/python tools/gen_map.py tools/map_layouts/loop.txt --out maps/gen_loop --seed 1 --clean normal
.venv/bin/python tools/clean_map.py maps/gen_loop/fms_map.json --out maps/gen_loop_clean --level normal
```

| 프리셋 | 지우는 것 | 남는 것 |
|---|---|---|
| `light` | 통로 점, 막힘 영역 줄무늬, 멀리 튄 점 | 벽 털, 점선 벽, 질감 |
| `normal` | + 벽 틈으로 삐져나간 털 | 점선 벽, 벽 앞 회색 테두리, 통로 질감 |
| `strong` | + 원래 벽 (통로 경계에 실선 벽을 새로 긋고 값 평탄화) | 통로 모양만. 손으로 그린 맵에 가깝다 |

- 단계별 값은 `--fill-hole-px`, `--open-px`, `--min-free-px`, `--wall-reach-px`,
  `--[no-]solid-walls`, `--[no-]flatten` 로 덮어쓴다 (`gen_map.py` 에서는 `--clean-` 접두사)
- 정리 전후 planner 기준 주행 가능 면적을 출력한다. 통로 점을 메우면 면적이 늘고,
  털 깎기(opening)는 줄인다. `gen_map.py --clean` 은 정리 뒤에 레이아웃 검사를 돈다
- 원본 SLAM 맵에서 벽 틈으로 넓게 퍼진 부채꼴 FREE 는 통로와 이어져 있고 폭도 넓어서
  안 지워진다 (생성 맵에는 조각 추출 단계에서 걸러져 거의 없다)

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
| 프로토콜/맵 계약 | `robot/{include,src}/schemas.*`, `map_model.*` |
| 경로계획/오더 실행/통신/시계 | `robot/{include,src}/{planner,executor,comm,clock}.*` |
| 물리 시뮬레이션 | `robot/{include,src}/{physics_world,kinematics,battery,dynamic_obstacles}.*` (MuJoCo 는 `physics_world.cpp` 안에만) |
| 데이터 기록 | `robot/{include,src}/recorder.*` |
| 로컬 제어판 | `gui.hpp/cpp` (Qt Widgets) |
| CLI 도구 | `tools/*.cpp`, 테스트 맵 생성/정리 `tools/{gen_map,synth_map,clean_map}.py` |
| SET_MAP 번들 다운로더 | `map_sync.hpp/cpp` |

기능 범위: MQTT(LWT/재접속), 오더 상태머신(released/order_update_id/거절 규칙),
4축 상태, e-stop/PAUSE, A* 경로계획, MuJoCo 물리(사다리꼴 궤적 추종), 배터리, 동적 장애물 회피,
충돌 감지, 강제 재위치, Qt 로컬 제어판.


## known issue
 - 경로 결정장애 이슈
    A to B에 2가지 갈림길이 존재할 때 로컬 패스플래닝은 동적 장애물을 센서 인식 범위 내에서만 기억하기 때문에 2가지 갈림길에 모두 동적 장애물이 존재할 경우 왔다 갔다 무한 반복하는 현상 발생함
    대응 방안: 동적 장애물을 일정 시간 메모리에 기록함. (맹점: 말그대로 동적 장애물 이기 때문에 메모리 시간을 어떻게 부여할지 고민 필요)

- 로봇 자체를 장애물로 인식 필요
    실제 환경에서는 다수의 로봇이 서로 장애물로 인식해야함. 물론 fms의 관제를 통해 부딪히는 상황이 없어야 하지만 시뮬레이션 자체에서 로봇이 겹쳐지는 상황은 프로그램 자체의 신뢰성을 떨어뜨림
