# AMR_viewer — 라이브 플릿 뷰어 (Python/Qt)

MQTT 로 로봇들을 관측만 하는 뷰어. `common/` 은 자체 사본을 갖고, 맵은 리포
루트 `../maps/` 를 공유한다. **관측자** — 로봇을 시뮬레이션하지 않고, 발행은
장애물 배치/취소(`b`/`u`)뿐이다. 어떤 언어의 로봇 클라이언트든(`AMR_client` 등)
프로토콜만 지키면 그대로 보인다.

## 구조

```
common/            schemas.py + map_model.py (사본)
tools/
  qt_viewer.py         라이브 렌더러 (PySide6/QGraphicsView)
  fleet_monitor.py      MQTT 구독 -> 렌더러
  fleet_monitor_qt.py    진입점
  map_viewer.py          정적 확인용 matplotlib (레거시)
assets/icons/      창 아이콘
```

## 설치/실행

```bash
# 리포 루트가 아니라 viewer/ 에서 실행
cd viewer
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python tools/fleet_monitor_qt.py ../maps/warehouse/warehouse.json --host <브로커 IP>
```

`--host`/`--port` 기본값은 localhost:1883. 로봇은 이 폴더와 무관한 별도
프로세스/PC 에서 돌아도 된다.

## 맵 따라가기

맵 인자를 생략하거나 `--follow ROBOT_ID` 를 주면 로봇이 보고한 `State.map_id`/`map_version` 으로 맵을 고른다
(`tools/map_follow.py`: 리포 루트 `map_cache/<uid>/` -> `--fms` 주소의 번들을 `viewer/.map_cache/` 로 받음).
맵이 바뀌면 화면 맵도 바뀌고, 다른 맵(층)의 로봇은 그리지 않고 패널에만 적는다.
`--map` 으로 띄운 로봇은 map_id 를 보고하지 않으므로 정적 맵 인자나 `--fallback-map JSON` 을 준다 (런처는 고른 맵을 넘긴다).
FMS 가 없으면 `--fms ""` 로 다운로드를 끈다. 위치를 모르는 로봇(`State.localization` UNLOCALIZED/LOCALIZING)은 흐리게 점선으로 그린다.

## 조작 키 / 물리 디버그

| 키 | 동작 |
|---|---|
| `t` | 계획 경로 토글 (현재 구간 굵게, 남은 구간 눈금선, 경유점 점, 목표 링 + 남은 거리) |
| `p` | 물리 패널 토글 (보고 위치/GT/오차, 속도/가속도/바퀴/슬립/접촉력/마찰·질량/경로) |
| `l` | 지나온 궤적 토글 |
| `o` | GT(참 위치, 청록 윤곽 + 점선 궤적) 토글. 몸체는 로봇이 보고한 추정 위치(`State.pose`, 위치보정 포함)라서 둘이 벌어진 만큼이 추정 오차다 |
| `i` | 보정 전 순수 오도메트리(노란 점선 윤곽) 토글. 몸체(보정된 추정)와 벌어진 만큼이 위치추정 보정이 고치는 양 |
| `k` | 궤적과 피크값(최대 슬립/접촉력) 지우기 |
| `f` | 로봇 시점 고정 (로봇1 -> 로봇2 ... -> 해제 순환). 마우스 드래그하면 해제, 스크롤 줌은 유지 |
| `g` `c` `z` `r` | 격자 / 색상 / 구역 / 보기 초기화 |
| `h` | 단축키 도움말 패널 토글 (좌하단, 그룹별 + 토글 켜짐 `●` / 꺼짐 `○` 표시) |
| `b` | 장애물 배치 모드 토글. 켠 뒤 맵을 클릭하면 3x3 맵 픽셀(해상도 0.03125 m 면 약 9.4 cm) 사각형을 월드 장애물로 놓는다 (retained `sim/v1/world/obstacles/{id}`). 드래그는 그대로 화면 이동 (4 px 이내로 뗀 것만 클릭) |
| `u` / `Cmd+Z` | 이 뷰어가 직전에 놓은 장애물 삭제. `Shift+U` 는 지금 있는 월드 장애물 전부 (다른 뷰어나 `place_obstacle --world` 가 놓은 것 포함) |

월드 장애물은 모든 로봇과 뷰어가 같이 본다. 장애물 하나가 retained 토픽 하나라서 나중에 뜬 로봇도 받고, 뷰어를 다시 띄워도 남아 있다.
뷰어도 같은 토픽을 구독해서 그린다 (오렌지 사각형): 놓은 것은 브로커를 한 바퀴 돌아와서 그려지고, 다른 뷰어나 도구가 놓은 것도 보인다.
맵 uid 가 있는 맵(FMS 번들)에서는 `map_id` 를 붙여서 다른 층의 로봇은 무시한다. 브로커를 다시 띄우면(개발용 설정은 persistence 가 꺼져 있다) 전부 사라진다.
로봇은 센서 반경 안에서만 감지하고, 로봇이 감지한 것은 검은 사각형(`State.detected_obstacles`)으로 따로 그려진다. 다른 로봇도 여기에 나온다.

물리 패널과 외삽은 로봇이 `fms/v1/{id}/physics` 로 발행하는 디버그 토픽을 쓴다 (프로토콜 밖, 20 Hz). 이 토픽이 없는 로봇
(다른 구현)은 `State` 값만으로 이전처럼 그려진다. 벽에 눌리는 접촉력이 5 N 을 넘으면 GT 윤곽이 빨개진다.

## 맵 갱신

맵은 리포 루트 `maps/` 하나를 로봇 클라이언트와 공유한다 (뷰어 전용 사본 없음).
참고: 루트 `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png`
이다. 예전 뷰어 사본은 `grid.png` 를 썼다 -- 표시 이미지가 달라 보이면 이 차이다.
