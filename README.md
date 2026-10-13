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
- 새 툴은 `tools/launcher_tools.json` 에 항목만 추가하면 붙는다 (파일 안 `_설명` 참고). `launch.sh`/`stop.sh` 는 따로 그대로 쓸 수 있다 (`.run/pids` 는 런처가 쓰지 않는다).

## 테스트 맵 생성 (`tools/gen_map.py`)

손으로 그린 맵(`maps/warehouse`)은 너무 정형화돼 있어서, 실제 SLAM 맵
(`maps/641931.../grid.png`) 에서 1 m 타일 조각을 잘라 레이아웃대로 이어붙인다.
점선 벽, 벽 너머 회색(미탐색), 레이저 줄무늬가 원본 그대로 남는다.

```bash
python3.11 -m venv .venv && .venv/bin/pip install numpy pillow   # 리포 루트. viewer/.venv 를 써도 된다
.venv/bin/python tools/gen_map.py tools/map_layouts/loop.txt --out maps/gen_loop --seed 1
./robot_client --id AMR-001 --map maps/gen_loop/fms_map.json --x 1.0 --y 1.0
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
| 프로토콜/맵 계약 | `include+src/schemas.*`, `map_model.*` |
| 경로계획/오더 실행/통신/시계 | `include+src/{planner,executor,comm,clock}.*` |
| 물리 시뮬레이션 | `include+src/{kinematics,battery,dynamic_obstacles}.*` |
| 로컬 제어판 | `gui.hpp/cpp` (Qt Widgets) |
| CLI 도구 | `tools/*.cpp`, 테스트 맵 생성/정리 `tools/{gen_map,synth_map,clean_map}.py` |
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
