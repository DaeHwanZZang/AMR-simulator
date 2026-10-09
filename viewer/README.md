# AMR_viewer — 라이브 플릿 뷰어 (Python/Qt)

MQTT 로 로봇들을 관측만 하는 뷰어. 이 폴더 하나만 복사하면 독립 실행된다
(`common/`, `maps/` 자체 사본 보유). **순수 관측자** — 아무것도 발행하지 않고,
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
maps/              맵 사본
```

## 설치/실행

```bash
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python tools/fleet_monitor_qt.py maps/warehouse/warehouse.json --host <브로커 IP>
```

`--host`/`--port` 기본값은 localhost:1883. 로봇은 이 폴더와 무관한 별도
프로세스/PC 에서 돌아도 된다.

## 맵 갱신

`maps/`는 `AMR_master/maps/`의 사본. 원본 갱신되면 여기도 수동 재복사 (자동
동기화 없음).
