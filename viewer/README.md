# AMR_viewer — 라이브 플릿 뷰어 (Python/Qt)

MQTT 로 로봇들을 관측만 하는 뷰어. `common/` 은 자체 사본을 갖고, 맵은 리포
루트 `../maps/` 를 공유한다. **순수 관측자** — 아무것도 발행하지 않고,
로봇을 시뮬레이션하지 않는다. 어떤 언어의 로봇 클라이언트든(`AMR_client` 등)
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

물리 패널과 외삽은 로봇이 `fms/v1/{id}/physics` 로 발행하는 디버그 토픽을 쓴다 (프로토콜 밖, 20 Hz). 이 토픽이 없는 로봇
(다른 구현)은 `State` 값만으로 이전처럼 그려진다. 벽에 눌리는 접촉력이 5 N 을 넘으면 GT 윤곽이 빨개진다.

## 맵 갱신

맵은 리포 루트 `maps/` 하나를 로봇 클라이언트와 공유한다 (뷰어 전용 사본 없음).
참고: 루트 `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png`
이다. 예전 뷰어 사본은 `grid.png` 를 썼다 -- 표시 이미지가 달라 보이면 이 차이다.
