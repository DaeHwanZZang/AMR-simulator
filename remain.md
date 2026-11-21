# 남은 작업 (2026-11-12 기준)

이번 작업 범위: 장애물 충돌 판정(A1), 시간 배속(B3), A* 성능(B1), 로봇 간 인식 + 월드 장애물 + 시나리오 러너(A2/D4/C3), 단위 테스트 + CI(C1/C2).
구현과 문서는 끝났다. 아래는 아직 안 한 것, 확인 못 한 것, 사용자 결정이 필요한 것이다.
명령은 전부 리포 루트(`AMR-simulator/`)에서 실행한다.

## 1. 바로 해야 하는 검증 (구현은 끝, 실행만 남음)

- [x] **전체 회귀 마지막 묶음** (2026-11-12 31/31 통과) — 최종 빌드 뒤 아직 안 돌렸다 (약 5 분).
      `.venv/bin/python robot/tests/physics_test.py gui topic estimate localization unlocalized charger battery`
      나머지 3 묶음은 최종 빌드로 106/106 통과. 이 묶음은 변경 전 기준선에서 통과했던 것들이다.
- [x] **시나리오 재확인** (2026-11-12 4개 모두 통과) — `parked`, `late_join` 은 러너의 오더 동시 발행 수정 전에 돌린 결과다 (로봇 코드는 같음).
      `for s in head_on parked late_join crossing_staggered; do .venv/bin/python tools/run_scenario.py scripts/scenarios/$s.json --time-scale 2; done`
- [ ] **뷰어 수동 확인** (실제 클릭은 offscreen 으로 못 봤다)
      브로커 + 로봇 + 뷰어를 띄우고: `b` 로 장애물 배치 → 로봇이 피하는지 / 로봇을 나중에 띄워도 피하는지 / `u`, `Shift+U` 삭제 / 뷰어를 껐다 켜도 사각형이 남는지.
      쉬운 방법: `.venv/bin/python tools/run_scenario.py scripts/scenarios/head_on.json --viewer --keep`
- [ ] **런처 폼 확인** — `./launcher` 에서 새 항목 "다중 로봇 시나리오", "단위 테스트" 와 로봇 폼의 시간 배속 / 다른 로봇 인식 끄기 / 월드 장애물 무시가 보이고 실행되는지. 명령 조립(`launcher_core.build_argv`)만 확인했다.

## 2. 사용자 결정이 필요한 것

- [ ] **커밋** — 전부 미커밋이다 (수정 32 파일, 신규: `.github/`, `robot/tests/unit/`, `robot/tests/simharness.py`, `robot/tools/plan_bench.cpp`, `scripts/scenarios/`, `tools/run_scenario.py`, 그리고 이전 작업의 `robot/{include,src}/obstacle_memory.*`).
      이번 작업 전의 미커밋분(장애물 기억, 감속 구역, Factsheet/loads, 뷰어 배치)과 섞여 있다. 기능별로 나눠 커밋할지 한 번에 할지 정해야 한다.
- [ ] **push → CI 첫 실행** — `.github/workflows/ci.yml` 은 로컬에서 돌릴 수 없어 YAML 구문만 확인했다. public 리포라 push 는 확인 후에 한다.
      첫 실행에서 볼 것: macOS 통합 테스트가 2 배속에서 안정적인지 / Ubuntu 빌드 (`setup_mujoco.sh` 의 Linux 경로, Qt6 `-fPIC` 여부는 미검증).
- [ ] **`v1.5` 태그** — `v1.4` 이후 MuJoCo 물리, 곡선 주행, 우회, Global 경로, 장애물 기억, 이번 작업까지 쌓였다.
- [ ] **잔여 파일** — `maps/gachon/grid copy.png`, `temp_txt.md`, `scripts/examples/1111.txt` 를 지울지 넣을지. 이 `remain.md` 도 마찬가지.
- [ ] **고아 프로세스** (어제부터 떠 있음, 건드리지 않았다)
      - PID 93330 `mosquitto -p 18851` : 예전 테스트가 남긴 브로커. 테스트는 이제 포트가 차 있으면 빈 포트를 쓰므로 영향은 없다.
      - PID 82089 `mosquitto -c logs/test_broker.conf`, PID 82122 `robot_client --id AMR-T1 ... --port 1884` : 수동 테스트 세션으로 보인다. `robot_client` 는 옛 바이너리로 돌고 있다.
      정리하려면: `kill 93330 82122 82089`

## 3. 알려진 한계 (이번에 드러난 것)

- [ ] **동시 직각 교차에서 충돌/교착** — `scripts/scenarios/crossing.json` 이 실패한다 (3 회 중 2 회 둘 다 `COLLISION`, 1 회 맞닿은 채 교착).
      원인: 서로를 3 m 밖에서 감지하지만 경로 막힘 판정이 상대의 지금 위치만 봐서 충돌 1 초쯤 전에야 우회를 시작한다.
      제안(A4 축소판): `executor.cpp` `sync_world` 에서 다른 로봇의 사각형을 진행 방향으로 `v × 1.5 s` 만큼 늘린다 (충돌 판정용 중심은 따로 유지). `PeerRobot` 에 `v_fwd` 추가. 둘 다 서로 양보해 흔들릴 수 있어 실험이 필요하다. 넣으면 `crossing.json` 으로 검증하고 CI/`physics_test.py robots` 에 추가.
- [ ] **맞닿은 채 교착해도 오류가 없다** — 위 1 회차처럼 둘 다 BUSY/WAITING 으로 무한 대기. 좁은 복도 정면 대치도 같다. FMS 가 알 수 있게 대기 타임아웃 WARNING 이 필요하다 (원래 추천 목록의 D3).
- [ ] **시간 배속 검증 범위** — 4 배속에서 `move curve dock detour script` 만 1 배속과 비교했다. 벽시계로 기다리는 시나리오(`pause`, `battery`, `charger` 등)는 배속에서 안 돌려 봤다. 전체 회귀를 배속으로 돌리려면 시나리오별 확인이 필요하다.
- [ ] **계획 워커 스레드(B2) 판단** — B1 결과: gachon 첫 계획 약 6.9 s → 0.12 s, 이후 313 → 52 ms / 1214 → 191 ms.
      아직 tick(50 ms)보다 길 수 있어 긴 경로 재계획 때 몇 tick 이 밀린다. 지금 맵 크기에서는 급하지 않다는 게 내 판단이고, 더 큰 맵이나 잦은 재경로가 문제가 되면 한다.

## 4. 작은 정리 (선택)

- [ ] 뷰어 검증 스크립트를 리포 테스트로 옮기기 — offscreen 으로 월드 장애물 표시/배치/삭제 6 항목을 확인한 스크립트가 세션 임시 폴더에만 있다 (세션이 지워지면 사라진다). `viewer/tests/` 같은 곳에 두면 회귀로 쓸 수 있다.
- [x] `robot/tests/physics_test.py` 맨 위 import 중 헬퍼 분리 뒤 안 쓰는 것 정리.
- [ ] CI 통합 테스트 범위 확대 — 지금은 macOS 에서 8 개 시나리오만. Ubuntu 는 빌드 + 단위 테스트뿐.
- [ ] `README.md` 제목이 아직 `AMR_client`.

## 5. 이번 범위 밖으로 남긴 추천 항목

| 항목 | 내용 |
|---|---|
| A3 | 장애물 감지에 시야(LOS) 반영 — 지금은 거리만 봐서 벽 너머도 보인다 |
| A4 | 이동 장애물 예측 (위 3 의 교차 문제와 같은 뿌리) |
| B2 | 계획 워커 스레드 |
| D1 | `reserved_window` — 파싱만 하고 안 쓴다. 지킬지 무시할지 FMS 쪽 의도 확인 필요 |
| D2 | 결함 주입 확장 (위치 상실, 통신 두절, 바퀴 정지 등) |
| D3 | 직선 액션/재경로 실패 대기에 타임아웃 WARNING |
| E1 | `rotate_step` 을 측정 각속도 PD 로 (저마찰 헌팅도 같이) |
| E2 | 제자리 회전 3~4% 과회전 |
| E3 | 배터리 소모·적재 질량을 물리와 연결 |
| 기타 | `--record` CSV 그리는 도구, `executor.cpp` 파일 분리 |

## 참고: 이번에 끝난 것

- 시간 배속 `--time-scale` / 테스트 `AMR_TEST_TIME_SCALE`
- 팽창 + A* 고속화 (결과 동일 확인), `build/bin/plan_bench`
- 단위 테스트 `make -C robot test` (23 케이스)
- 장애물/로봇 충돌 판정 (`COLLISION`), 로봇 간 인식, 월드 장애물 토픽(`sim/v1/world/obstacles/{id}`), `place_obstacle --world`
- 뷰어 `b` 배치를 월드 장애물로 전환
- 시나리오 러너 `tools/run_scenario.py` + `scripts/scenarios/` 5 개, 런처 항목 2 개
- `robot/tests/simharness.py` 분리, 새 회귀 시나리오 `collide_obstacle` `robots` `world`, 낡은 `script` 기대값 수정
- `.github/workflows/ci.yml`, README / CLAUDE.md / AGENTS.md / viewer/README.md 갱신
