# MQTT_broker — FMS 브로커 설정

`mosquitto.conf` 하나. 로봇(`AMR_client`)/뷰어(`AMR_viewer`)/`fms_server` 가
서로 통신하는 브로커 설정. 위치는 어디 PC든 상관없다 — 네트워크로 서로 도달만
가능하면 된다.

## 설치

```bash
# macOS
brew install mosquitto

# Ubuntu / WSL(Ubuntu)
sudo apt update && sudo apt install mosquitto mosquitto-clients

# Windows 네이티브 (WSL 미사용)
# https://mosquitto.org/download/ 인스톨러 사용. 실사용은 WSL 권장 (CLAUDE.md 기준 Ubuntu 타깃)
```

## 실행

```bash
mosquitto -c mosquitto.conf -v
```

## 설정값

| 항목 | 값 | 의미 |
|---|---|---|
| `listener` | `1883 127.0.0.1` | 로컬루프백만 — **개발용**, 아래 참고 |
| `allow_anonymous` | `true` | 인증 없음 — **개발용** |
| `max_queued_messages` / `max_inflight_messages` | `10000` / `100` | 로봇 20대 x 상태보고 대비 |
| `persistence` | `false` | 재시작 시 유지 안 함 |

## 여러 PC로 나눠 쓸 때

`listener 1883 0.0.0.0`으로 바꾸고, `allow_anonymous false` + `password_file`
로 최소 인증 붙일 것 (`mosquitto_passwd -c <file> <user>`). 방화벽에서 `1883`
포트 개방 필요. WSL 은 Windows 방화벽/포트포워딩(`netsh interface portproxy`)
추가로 필요할 수 있음.

## 접속 확인

```bash
mosquitto_sub -h <브로커 IP> -p 1883 -t 'fms/v1/#' -v
```

토픽 구조는 `AMR_master/common/schemas.py` 참고.
