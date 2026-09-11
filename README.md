# AMR_client — 로봇 클라이언트 (C++)

Python `robot_client` 의 1:1 C++ 재구현. 프로토콜(MQTT+JSON)이 언어 무관이라
`MQTT_broker`, `AMR_viewer`, `fms_server` 와 그대로 섞어 쓴다. 이 폴더 하나만
복사하면 독립적으로 빌드/실행된다 — `common/`, `maps/` 사본을 자체 보유.

## 빌드

```bash
brew install mosquitto cjson qt     # macOS. Linux: apt install libmosquitto-dev libcjson-dev qt6-base-dev
make                                 # robot_client + tools/{send_order,place_obstacle,obstacle_zone}
```

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
Python 원본은 GIL 덕에 락이 필요 없었던 부분 — 유일하게 원본과 다르게 설계한 지점.

## 알려진 차이

- 난수 시드가 Python 과 값까지 같이 재현되진 않는다 (PRNG 알고리즘이 다름)
- GUI 색/레이아웃은 원본을 따랐지만 위젯 여백은 Qt 기본값 기준 (픽셀 단위 동일 X)

## 대응 관계 (Python → C++)

| Python | C++ |
|---|---|
| `common/schemas.py`, `map_model.py` | `include+src/schemas.*`, `map_model.*` |
| `robot_client/{planner,executor,comm,clock}.py` | `include+src/{planner,executor,comm,clock}.*` |
| `robot_client/sim/*.py` | `include+src/{kinematics,battery,dynamic_obstacles}.*` |
| `robot_client/gui.py` (Tk) | `gui.hpp/cpp` (Qt Widgets) |
| `tools/*.py` | `tools/*.cpp` |
| (신규, Python 원본 없음) | `map_sync.hpp/cpp` — SET_MAP 번들 다운로더 |

포팅 범위: MQTT(LWT/재접속), 오더 상태머신(released/order_update_id/거절 규칙),
4축 상태, e-stop/PAUSE, A* 경로계획, 사다리꼴 운동학, 배터리, 동적 장애물 회피,
충돌 감지, 강제 재위치, Qt 로컬 제어판 — 전부 1:1.


## known issue
 - 경로 결정장애 이슈
    A to B에 2가지 갈림길이 존재할 때 로컬 패스플래닝은 동적 장애물을 센서 인식 범위 내에서만 기억하기 때문에 2가지 갈림길에 모두 동적 장애물이 존재할 경우 왔다 갔다 무한 반복하는 현상 발생함
    대응 방안: 동적 장애물을 일정 시간 메모리에 기록함. (맹점: 말그대로 동적 장애물 이기 때문에 메모리 시간을 어떻게 부여할지 고민 필요)

- 로봇 자체를 장애물로 인식 필요
    실제 환경에서는 다수의 로봇이 서로 장애물로 인식해야함. 물론 fms의 관제를 통해 부딪히는 상황이 없어야 하지만 시뮬레이션 자체에서 로봇이 겹쳐지는 상황은 프로그램 자체의 신뢰성을 떨어뜨림
