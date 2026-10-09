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

## 맵 갱신

맵은 리포 루트 `maps/` 하나를 로봇 클라이언트와 공유한다 (뷰어 전용 사본 없음).
참고: 루트 `maps/641931.../fms_map.json` 의 `image` 는 `edited_navi_gridmap.png`
이다. 예전 뷰어 사본은 `grid.png` 를 썼다 -- 표시 이미지가 달라 보이면 이 차이다.
