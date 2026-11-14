"""물리(MuJoCo) 통합 테스트. empty_hall1 에서 robot_client 를 실제로 띄워 MQTT 로 시험한다.

리포 루트에서: .venv/bin/python robot/tests/physics_test.py [시나리오 이름...]
시나리오 이름은 맨 아래 ALL 참고 (생략하면 전부).
mosquitto 를 자체 포트(18851, 쓰는 중이면 빈 포트)로 띄우므로 이미 떠 있는 브로커와 부딪히지 않는다. 로봇 로그와 기록 CSV 는 임시 폴더에 남는다.
AMR_TEST_TIME_SCALE=N 이면 로봇을 N 배속으로 돌린다 (기본 1). 헬퍼(Robot, 브로커, check)는 simharness.py 에 있다.
"""
import json, math, os, re, shutil, signal, subprocess, sys, time

import paho.mqtt.client as mqtt

from simharness import *  # noqa: F401,F403  (Robot, check, dist, REPO, SCR, PORT, MAP, REC, TIME_SCALE, results, procs, ...)

def adiff(a, b):
    d = a - b
    while d > math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def test_move():
    r = Robot("M1", 3, 3)
    r.order("o1", ["12,3", "12,9,0"])
    s = r.idle_done("o1", 40)
    ok = s is not None
    p = r.pose()
    check("move: reaches goal and IDLE", ok and s["state"] == "IDLE" and not s["errors"], f"state={s and s['state']} pose={p}")
    check("move: final position error < 3 cm", dist((p["x"], p["y"]), (12, 9)) < 0.03, f"err={dist((p['x'], p['y']), (12, 9)) * 1000:.1f} mm")
    check("move: final heading error < 0.03 rad", abs(adiff(p["theta"], 0.0)) < 0.03, f"theta={p['theta']}")
    rows = [x for x in r.csv_rows() if x["order_id"] == "o1"]
    vmax = max(abs(float(x["v_fwd"])) for x in rows)
    wmax = max(abs(float(x["yaw_rate"])) for x in rows)
    check("move: speed within limits (v<1.35, w<2.5)", vmax < 1.35 and wmax < 2.5, f"vmax={vmax:.2f} wmax={wmax:.2f}")
    check("move: no wall contact", max(float(x["wall_force"]) for x in rows) < 1.0)
    # 하나 더: 같은 로봇으로 연속 오더
    r.order("o2", ["12,3,0"])
    s = r.idle_done("o2", 30)
    p = r.pose()
    check("move: second order works", s is not None and s["state"] == "IDLE" and dist((p["x"], p["y"]), (12, 3)) < 0.03, f"pose={p}")
    r.stop()


def test_straight():
    r = Robot("S1", 20, 6)
    r.order("s1", ["22,6,0"], "STRAIGHT", ["distance=1.0"])
    s = r.idle_done("s1", 40)
    p = r.pose()
    check("straight +1.0: ends 1.0 m ahead", s is not None and abs(p["x"] - 23.0) < 0.03 and abs(p["y"] - 6.0) < 0.03 and abs(p["theta"]) < 0.03,
          f"pose={p}")
    r.order("s2", ["23,6,0"], "STRAIGHT", ["distance=-1.5"])
    s = r.idle_done("s2", 40)
    p = r.pose()
    check("straight -1.5: reverses, heading kept", s is not None and abs(p["x"] - 21.5) < 0.03 and abs(p["y"] - 6.0) < 0.03 and abs(p["theta"]) < 0.03,
          f"pose={p}")
    r.order("s3", ["21.5,6,1.5708"], "STRAIGHT", ["distance=1.0", "speed=0.5"])
    s = r.idle_done("s3", 40)
    p = r.pose()
    check("straight at 90deg heading", s is not None and abs(p["x"] - 21.5) < 0.04 and abs(p["y"] - 7.0) < 0.04, f"pose={p}")
    r.stop()


def test_dock_park():
    r = Robot("D1", 30, 6, extra=["--battery", "50"])
    # DOCK 후진 진입: 승인점 (31,6) 에서 후진해 (30,6) 로 들어간다
    r.order("d1", ["30,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.0", "min_charge=56"])
    s = r.idle_done("d1", 60)
    p = r.pose()
    # 보고 방위 = 추정 (스캔 매칭 + 추적 노이즈 σθ 0.01 rad 가 섞인다). 충전 접점 허용 0.1 rad 보다 충분히 작으면 된다
    check("dock BACKWARD: docked at (30,6) heading 0", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.03 and abs(p["theta"]) < 0.05,
          f"pose={p} state={s and s['state']}")
    check("dock: charging below min_charge -> CHARGING", s is not None and s["state"] == "CHARGING", f"state={s and s['state']} batt={s and s['battery']}")
    r.order("d2", ["35,6,0"])
    time.sleep(1.0)
    s = r.state
    check("dock: job rejected while below min_charge", s["order_id"] is None, f"order_id={s['order_id']}")
    s = r.wait(lambda s: s["state"] == "IDLE", 30)
    check("dock: becomes IDLE after reaching min_charge", s is not None, f"batt={r.state['battery']}")
    r.order("d3", ["35,6,0"])
    s = r.idle_done("d3", 40)
    p = r.pose()
    check("dock: undocks forward then drives on", s is not None and dist((p["x"], p["y"]), (35, 6)) < 0.03, f"pose={p}")
    # PARK 전진 진입 -> 후진 언도킹
    r.order("p1", ["40,6,0"], "PARK", ["entry=FORWARD", "undock_distance=0.8"])
    s = r.idle_done("p1", 60)
    p = r.pose()
    check("park FORWARD: parked at (40,6)", s is not None and dist((p["x"], p["y"]), (40, 6)) < 0.03 and abs(p["theta"]) < 0.03, f"pose={p}")
    r.order("p2", ["45,6,0"])
    # 언도킹 중간에 위치를 본다: 후진이므로 x 가 먼저 39.2 쪽으로 간다
    mn = 99
    t0 = time.time()
    while time.time() - t0 < 6:
        mn = min(mn, r.state["pose"]["x"])
        time.sleep(0.05)
    check("park: undock reverses first", mn < 39.4, f"min x={mn:.2f}")
    s = r.idle_done("p2", 60)
    p = r.pose()
    check("park: then drives to next goal", s is not None and dist((p["x"], p["y"]), (45, 6)) < 0.03, f"pose={p}")
    r.stop()


def has_err(s, kind):
    return s is not None and any(e["error_type"] == kind for e in s["errors"])


def test_charger():
    """충전기는 맵의 물리 설비 (maps/empty_hall1/chargers.json: CH-1 (30,6,0) 후진 진입)."""
    r = Robot("H1", 20, 6, extra=["--battery", "50"])
    r.order("h1", ["25,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.0", "min_charge=0"])
    s = r.wait(lambda s: has_err(s, "DOCK_FAILED"), 40)
    check("charger: DOCK where no charger -> DOCK_FAILED", s is not None and s["state"] == "ERROR" and not s["battery"]["charging"],
          f"state={s and (s['state'], s['errors'])}")
    r.instant("RESET_ERROR")
    r.order("h2", ["20,6,0"], "CHARGE")
    s = r.wait(lambda s: has_err(s, "CHARGE_FAILED"), 30)
    check("charger: CHARGE off the charger -> CHARGE_FAILED", s is not None, f"errors={s and s['errors']}")
    r.instant("RESET_ERROR")
    r.order("h3", ["30,6,0"], "DOCK", ["entry=FORWARD", "undock_distance=1.0", "min_charge=0"])
    s = r.wait(lambda s: has_err(s, "DOCK_FAILED"), 60)
    check("charger: wrong entry side -> DOCK_FAILED, not charging", s is not None and not s["battery"]["charging"],
          f"errors={s and [e['description'] for e in s['errors']]}")
    r.instant("RESET_ERROR")
    r.order("h4", ["30,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.0", "min_charge=0"])
    s = r.wait(lambda s: s["order_id"] is None and s["battery"]["charging"], 90)
    p = r.pose()
    check("charger: undocks, then docks BACKWARD and charges", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.04,
          f"pose={p} batt={r.state['battery']} errors={r.state['errors']}")
    b0 = r.state["battery"]["charge"]
    time.sleep(2.0)
    check("charger: battery rises while docked", r.state["battery"]["charge"] > b0, f"{b0} -> {r.state['battery']['charge']}")
    r.stop()


def test_battery():
    r = Robot("B1", 3, 6, extra=["--battery", "20.3"])
    r.order("b1", ["40,6"])
    s = r.wait(lambda s: has_err(s, "LOW_BATTERY"), 30)
    check("battery: LOW_BATTERY warning below 20%, order continues", s is not None and s["order_id"] == "b1",
          f"state={s and (s['state'], s['battery'])}")
    r.stop()

    r = Robot("B2", 3, 6, extra=["--battery", "5.3"])
    r.order("b2", ["40,6"])
    s = r.wait(lambda s: has_err(s, "BATTERY_CRITICAL"), 30)
    check("battery: crossing 5% -> FATAL BATTERY_CRITICAL, order dropped", s is not None and s["state"] == "ERROR" and s["order_id"] is None,
          f"state={s and (s['state'], s['battery'])}")
    r.instant("RESET_ERROR")
    r.wait(lambda s: s["state"] == "IDLE", 5)
    r.order("b3", ["10,6"])
    time.sleep(1.0)
    check("battery: below critical, normal order rejected", r.state["order_id"] is None and r.state["state"] == "IDLE",
          f"state={r.state['state']} order={r.state['order_id']}")
    r.order("b4", ["30,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.0", "min_charge=0"])
    s = r.wait(lambda s: s["order_id"] is None and s["battery"]["charging"], 90)
    check("battery: below critical, DOCK order accepted and charges", s is not None, f"state={r.state['state']} batt={r.state['battery']}")
    r.stop()

    r = Robot("B3", 3, 6, extra=["--battery", "0.15"])
    r.order("b5", ["40,6"])
    s = r.wait(lambda s: has_err(s, "BATTERY_EMPTY"), 30)
    check("battery: empty -> FATAL BATTERY_EMPTY", s is not None and s["state"] == "ERROR", f"state={s and (s['state'], s['battery'])}")
    time.sleep(1.0)
    p0 = r.pose()
    r.instant("RESET_ERROR")
    time.sleep(1.0)
    r.order("b6", ["10,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.0", "min_charge=0"])
    time.sleep(2.0)
    p1 = r.pose()
    check("battery: empty robot cannot drive (even after reset / DOCK order)",
          dist((p0["x"], p0["y"]), (p1["x"], p1["y"])) < 0.02 and has_err(r.state, "BATTERY_EMPTY"),
          f"moved={dist((p0['x'], p0['y']), (p1['x'], p1['y'])):.3f} state={r.state['state']}")
    r.stop()


def test_collision():
    r = Robot("C1", 50, 6)
    r.order("c1", ["50,6,0"], "STRAIGHT", ["distance=40", "speed=0.8"])
    s = r.wait(lambda s: any(e["error_type"] == "COLLISION" for e in s["errors"]), 60)
    p = r.pose()
    check("collision: COLLISION FATAL raised on wall", s is not None, f"errors={s and s['errors']}")
    check("collision: robot stays inside, no ejection", 52 < p["x"] < 54.6 and abs(p["y"] - 6) < 0.3, f"pose={p}")
    time.sleep(1.0)
    p2 = r.pose()
    check("collision: robot stays put after error", dist((p["x"], p["y"]), (p2["x"], p2["y"])) < 0.02, f"{p} -> {p2}")
    rows = r.csv_rows()
    check("collision: wall force recorded", max(float(x["wall_force"]) for x in rows) > 20, f"max={max(float(x['wall_force']) for x in rows):.0f} N")
    r.stop()


def test_pause_cancel():
    r = Robot("P1", 3, 3)
    r.order("q1", ["20,3,0"])
    r.wait(lambda s: s["velocity"] > 0.8, 15)
    r.instant("PAUSE")
    r.wait(lambda s: s["paused"], 5)
    time.sleep(0.5)
    p = r.pose()
    v = r.state["velocity"]
    check("pause: stops immediately", v < 0.02, f"v={v}")
    time.sleep(1.0)
    p2 = r.pose()
    check("pause: stays still", dist((p["x"], p["y"]), (p2["x"], p2["y"])) < 0.01)
    r.instant("RESUME")
    s = r.wait(lambda s: s["velocity"] > 0.5, 10)
    check("resume: moves again", s is not None)
    r.instant("CANCEL_ORDER")
    time.sleep(0.2)
    s = r.wait(lambda s: s["velocity"] < 0.05, 5)
    check("cancel: coasts to a stop", s is not None, f"v={r.state['velocity']}")
    p = r.pose()
    time.sleep(1.0)
    p2 = r.pose()
    check("cancel: stays stopped (no stale velocity)", dist((p["x"], p["y"]), (p2["x"], p2["y"])) < 0.01, f"{p} -> {p2}")
    r.order("q2", [f"{p['x'] + 3:.2f},{p['y']:.2f},0"])
    s = r.idle_done("q2", 30)
    p3 = r.pose()
    check("cancel: next order runs fine", s is not None and abs(p3["x"] - (p["x"] + 3)) < 0.04, f"pose={p3}")
    r.stop()


def test_lazy_plan():
    """경로계획은 현재 목적지까지만: C 가 맵 밖이어도 A->B 는 가고, B 에 도착한 뒤에야 C 에서 NO_PATH 가 난다."""
    r = Robot("L1", 3, 3)
    r.order("lz1", ["5,3", "7,3", "500,500"])
    s = r.wait(lambda s: s["state"] == "ERROR", 60)
    p = r.pose()
    errs = [e["error_type"] for e in (s or {}).get("errors", [])]
    check("lazy: reaches B before failing on unreachable C", s is not None and dist((p["x"], p["y"]), (7, 3)) < 0.1 and "NO_PATH" in errs,
          f"pose={p} errors={errs}")
    r.stop()
    r = Robot("L2", 3, 3)
    r.order("lz2", ["5,3,0", "5,5,1.57", "7,5,3.14"])
    s = r.idle_done("lz2", 60)
    p = r.pose()
    check("lazy: 3-node order completes, last theta aligned", s is not None and dist((p["x"], p["y"]), (7, 5)) < 0.05 and abs(adiff(p["theta"], 3.14)) < 0.1, f"pose={p}")
    with open(f"{SCR}/L2.log") as f:
        plans = sum(1 for line in f if "planning path from" in line)
    check("lazy: one planning per node", plans == 3, f"plans={plans}")
    r.stop()


def test_ghost_start():
    r = Robot("G1", 0.0, 0.0, localize=False)  # 맵 여백(UNKNOWN) 위
    time.sleep(2.0)
    p = r.pose()
    check("ghost start: no ejection", abs(p["x"]) < 0.01 and abs(p["y"]) < 0.01, f"pose={p}")
    # 벽 속에서는 라이다가 아무것도 못 본다 -> 힌트를 줘도 로컬라이징 실패, 오더는 거절
    r.hint(0.0, 0.0)
    s = r.wait(lambda s: any(e["error_type"] == "LOCALIZATION_FAILED" for e in s["errors"]), 10)
    check("ghost start: localization fails inside the wall", s is not None and s["localization"] == "UNLOCALIZED",
          f"state={s and (s['localization'], s['errors'])}")
    r.order("g1", ["5,5,0"])
    time.sleep(1.0)
    check("ghost start: order rejected while UNLOCALIZED", r.state["order_id"] is None)
    rows = r.csv_rows()
    check("ghost start: ghost flag recorded", any(x["ghost"] == "1" for x in rows))
    p = r.pose()
    check("ghost start: still at start", abs(p["x"]) < 0.01 and abs(p["y"]) < 0.01, f"pose={p}")
    r.stop()


def test_unlocalized():
    """켜자마자 UNLOCALIZED: 오더 거절 -> 먼 힌트는 실패 -> 가까운 힌트(0.9 m, 11°)는 스캔 매칭으로 참 자세에 수렴."""
    r = Robot("U2", 3, 3, localize=False)
    check("unlocalized: starts UNLOCALIZED", r.state["localization"] == "UNLOCALIZED")
    r.order("u1", ["5,3"])
    time.sleep(1.0)
    check("unlocalized: order rejected", r.state["order_id"] is None and r.state["state"] == "IDLE")
    r.hint(8.0, 3.0, 0.3)  # 5 m 틀림 (창 ±1.5 m 밖)
    s = r.wait(lambda s: any(e["error_type"] == "LOCALIZATION_FAILED" for e in s["errors"]), 10)
    check("unlocalized: far hint fails", s is not None and s["localization"] == "UNLOCALIZED",
          f"score={s and s['localization_score']}")
    r.hint(3.8, 2.5, 0.2)
    s = r.wait(lambda s: s["localization"] == "LOCALIZED", 10)
    p = s and s["pose"]
    check("unlocalized: near hint localizes", s is not None and not s["errors"], f"state={s and (s['localization'], s['errors'])}")
    check("unlocalized: estimate snaps to true pose (< 3 cm, < 0.02 rad)",
          p is not None and dist((p["x"], p["y"]), (3, 3)) < 0.03 and abs(adiff(p["theta"], 0.0)) < 0.02, f"pose={p}")
    r.order("u2", ["5,3"])
    s = r.idle_done("u2", 30)
    check("unlocalized: drives after localization", s is not None and dist((s["pose"]["x"], s["pose"]["y"]), (5, 3)) < 0.03)
    r.stop()


def test_low_friction():
    # 위치보정을 켜면 로봇이 참 위치를 따라 속도를 조절해 슬립이 줄어든다. 슬립/오도메트리 오차 자체를 보려고 끈다
    r = Robot("F1", 3, 3, extra=["--friction", "0.15", "--max-acceleration", "2.0", "--loc-rate", "0"])
    r.order("f1", ["15,3,0"])
    s = r.idle_done("f1", 90)
    p = r.pose()
    rows = [x for x in r.csv_rows() if x["order_id"] == "f1"]
    slip = max(max(abs(float(x["slip_l"])), abs(float(x["slip_r"]))) for x in rows)
    last = rows[-1]
    # 가속 때 앞으로 미끄러진 만큼 오도메트리가 앞서 가고 감속 때 다시 줄어든다. 끝에선 상쇄되니 주행 중 최대 오차를 본다
    odom_err = max(math.hypot(float(x["odom_x"]) - float(x["x"]), float(x["odom_y"]) - float(x["y"])) for x in rows)
    check("low friction: still arrives", s is not None and dist((p["x"], p["y"]), (15, 3)) < 0.05, f"pose={p}")
    check("low friction: wheel slip occurs (>0.3)", slip > 0.3, f"max slip={slip:.2f}")
    check("low friction: odometry drifts from truth during the run (>2 cm)", odom_err > 0.02, f"odom err={odom_err * 100:.1f} cm")
    r.stop()


def test_normal_slip_small():
    r = Robot("N1", 3, 3)
    r.order("n1", ["15,3,0"])
    r.idle_done("n1", 60)
    rows = [x for x in r.csv_rows() if x["order_id"] == "n1" and abs(float(x["v_fwd"])) > 0.3]
    slip = sorted(max(abs(float(x["slip_l"])), abs(float(x["slip_r"]))) for x in rows)
    med = slip[len(slip) // 2]
    last = r.csv_rows()[-1]
    odom_err = math.hypot(float(last["odom_x"]) - float(last["x"]), float(last["odom_y"]) - float(last["y"]))
    check("normal friction: median slip small (<0.08)", med < 0.08, f"median slip={med:.3f}")
    check("normal friction: odometry close to truth (<35 cm / 12 m, ~2% heading drift)", odom_err < 0.35, f"odom err={odom_err * 100:.1f} cm")
    r.stop()


def test_loop_map():
    # 통로가 좁은 생성 맵: 물리 충돌 판정이 planner 와 어긋나는지 본다 (정보용 + 완주 확인)
    m = f"{REPO}/maps/gen_loop/fms_map.json"
    r = Robot("L1", 1, 1, mapfile=m)
    r.order("l1", ["1,9,0", "10,9,0", "10,1,0", "1,1,0"])
    s = r.idle_done("l1", 120)
    errs = r.state["errors"]
    p = r.pose()
    check("loop map: completes loop without COLLISION", s is not None and not errs, f"state={s and s['state']} errors={errs} pose={p}")
    r.stop()


def _corner_run(rid, extra):
    """gen_loop 에서 (1,9) -> (12,12): 한 노드 구간 안에 90도 코너가 있다. (소요 시간, 코너 구간 최소 속도, 에러, 최종 자세)"""
    m = f"{REPO}/maps/gen_loop/fms_map.json"
    r = Robot(rid, 1, 9, mapfile=m, extra=extra)
    t0 = time.time()
    r.order(rid.lower(), ["12,12,0"])
    s = r.idle_done(rid.lower(), 90)
    dur = time.time() - t0
    rows = [x for x in r.csv_rows() if x["order_id"] == rid.lower()]
    # 코너(12,9) 근처 1.2 m 안에서의 실제 속도
    near = [abs(float(x["v_fwd"])) for x in rows if math.hypot(float(x["x"]) - 12, float(x["y"]) - 9) < 1.2]
    errs = r.state["errors"]
    p = r.pose()
    r.stop()
    wall = max((float(x["wall_force"]) for x in rows), default=0.0)
    return s, dur, (min(near) if near else None), errs, p, max((abs(float(x["v_fwd"])) for x in rows), default=0), wall


def test_curve():
    """곡선 주행: 90도 코너에서 서지 않고 지나간다. --no-smooth-path 는 코너에서 선다 (비교)."""
    s, dur, vmin, errs, p, vmax, wall = _corner_run("C1", [])
    check("curve: completes without error", s is not None and not errs and dist((p["x"], p["y"]), (12, 12)) < 0.05, f"state={s and s['state']} errors={errs} pose={p}")
    check("curve: does not stop at the 90deg corner", vmin is not None and vmin >= 0.3, f"min v near corner={vmin} (vmax={vmax:.2f}) dur={dur:.1f}s")
    check("curve: no wall contact along the curve (force < 20 N)", wall < 20.0, f"max wall force={wall:.1f} N")
    s2, dur2, vmin2, errs2, p2, _, _ = _corner_run("C2", ["--no-smooth-path"])
    check("curve: --no-smooth-path still completes (stops at corner)", s2 is not None and not errs2 and dist((p2["x"], p2["y"]), (12, 12)) < 0.05, f"state={s2 and s2['state']} errors={errs2} min v={vmin2} dur={dur2:.1f}s")
    check("curve: smooth is faster than no-smooth", dur < dur2, f"{dur:.1f}s vs {dur2:.1f}s")


def test_longrun():
    # 55 m 가로지르기: 벽 창(window)이 여러 번 다시 잡힌다. 자세가 끊기지 않아야 한다.
    r = Robot("R1", 2, 3)
    r.order("r1", ["52,3,0", "52,12,0", "2,12,0"])
    s = r.idle_done("r1", 200)
    p = r.pose()
    rows = [x for x in r.csv_rows() if x["order_id"] == "r1"]
    jump = 0.0   # 참 자세의 tick 간 이동 속도 (m/s). tick 시간(dt)이 늘어난 것과 위치 점프를 구분한다
    for a, b in zip(rows, rows[1:]):
        dt = max(1e-3, float(b["t"]) - float(a["t"]))
        jump = max(jump, math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) / dt)
    check("longrun: completes 110 m route", s is not None and s["state"] == "IDLE" and not s["errors"] and dist((p["x"], p["y"]), (2, 12)) < 0.03, f"pose={p} errors={s and s['errors']}")
    check("longrun: no position jump at window rebuilds (true speed < 1.5 m/s)", jump < 1.5, f"max step speed={jump:.2f} m/s")
    check("longrun: no wall contact", max(float(x["wall_force"]) for x in rows) < 1.0)
    r.stop()


def _place(rid, oid_x, oid_y, w, h, oid=None):
    a = [f"{REPO}/build/bin/place_obstacle", rid, "--port", str(PORT), "--x", str(oid_x), "--y", str(oid_y), "--width", str(w), "--height", str(h)]
    if oid:
        a += ["--obstacle-id", oid]
    subprocess.run(a, capture_output=True)


def _clear(rid):
    subprocess.run([f"{REPO}/build/bin/place_obstacle", rid, "--port", str(PORT), "--clear"], capture_output=True)


def test_obstacle():
    # 홀 전체를 가로막는 상자(1 x 200): 우회로도 재경로도 없다 -> 서서 기다리다가 치우면 간다
    r = Robot("O1", 3, 6)
    r.order("b1", ["30,6,0"])
    r.wait(lambda s: s["pose"]["x"] > 8 and s["velocity"] > 0.8, 30)
    _place("O1", 20, 6, 1.0, 200.0)
    s = r.wait(lambda s: s["sub_state"] == "WAITING" if s.get("sub_state") else False, 20)
    check("obstacle: robot waits when no route exists", s is not None)
    time.sleep(3)
    p = r.pose()
    check("obstacle: stops short of the obstacle", s is not None and p["x"] < 20 - 0.5 - 0.21 and r.state["state"] != "ERROR", f"pose={p} state={r.state['state']}")
    rows = r.csv_rows()
    amax = 0.0
    for a, b in zip(rows, rows[1:]):
        if abs(float(a["v_fwd"])) > 0.6:
            amax = max(amax, abs(float(b["v_fwd"]) - float(a["v_fwd"])) / max(1e-3, float(b["t"]) - float(a["t"])))
    check("obstacle: no abrupt stops from cruise speed (|a| < 2 m/s^2 above 0.6 m/s)", amax < 2.0, f"max |a|={amax:.2f}")
    _clear("O1")
    s = r.idle_done("b1", 90)
    p = r.pose()
    check("obstacle: resumes and arrives after clearing", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.05, f"pose={p}")
    r.stop()


def test_collide_obstacle():
    """장애물도 물리적으로 있다: 설 수 없는 거리에 갑자기 나타나면 COLLISION. 서 있는 로봇 위에 놓인 것은 로봇 오류가 아니다."""
    r = Robot("K1", 3, 6)
    time.sleep(0.5)
    _place("K1", 3, 6, 0.4, 0.4, oid="on-top")   # 서 있는 로봇 몸체와 겹치게 놓는다
    time.sleep(1.5)
    check("collide_obstacle: an obstacle dropped on a standing robot is not an error", r.state["state"] != "ERROR", f"errors={r.state['errors']}")
    _clear("K1")
    r.order("k1", ["30,6,0"])
    s = r.wait(lambda s: s["pose"]["x"] > 8 and s["velocity"] > 0.8, 30)
    check("collide_obstacle: cruising", s is not None)
    # 로봇 0.6 m 앞에 홀을 가로막는 벽(두께 0.4): 제동거리보다 가까워서 설 수도 돌아갈 수도 없다
    _place("K1", r.pose()["x"] + 0.6, 6, 0.4, 200.0, oid="wall")
    s = r.wait(lambda s: s["state"] == "ERROR", 10)
    errs = [e for e in (s or {}).get("errors", []) if e["error_type"] == "COLLISION"]
    check("collide_obstacle: driving into an obstacle raises COLLISION", bool(errs) and "obstacle wall" in errs[0]["description"],
          f"errors={(s or {}).get('errors')}")
    check("collide_obstacle: order is dropped", s is not None and s["order_id"] is None)
    # 겹친 채로 오류를 지워도 같은 접촉으로 다시 오류가 나지 않고, 치우면 다시 간다
    r.instant("RESET_ERROR")
    time.sleep(1.0)
    check("collide_obstacle: no repeated error while still touching", r.state["state"] != "ERROR", f"errors={r.state['errors']}")
    _clear("K1")
    r.order("k2", ["30,6,0"])
    s = r.idle_done("k2", 90)
    p = r.pose()
    check("collide_obstacle: drives again after the obstacle is removed", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.05, f"pose={p} state={r.state['state']}")
    r.stop()


def test_robots():
    """로봇끼리 서로를 장애물로 본다: 서 있는 로봇을 돌아가고, 마주 오는 로봇과 겹치지 않고 지나간다."""
    # 1) 서 있는 로봇: P1 의 직선 경로 한가운데 P2 가 서 있다
    a = Robot("P1", 5, 6)
    b = Robot("P2", 12, 6)
    a.order("p1", ["20,6,0"])
    s = a.idle_done("p1", 90)
    p = a.pose()
    check("robots: drives around a parked robot and arrives", s is not None and s["state"] == "IDLE" and dist((p["x"], p["y"]), (20, 6)) < 0.05,
          f"pose={p} state={a.state['state']} errors={a.state['errors']}")
    with a.lock:
        saw = any(st.get("detected_obstacles") for st in a.states)
        dev = max(abs(st["pose"]["y"] - 6) for st in a.states)
    check("robots: the parked robot is reported as a detected obstacle", saw)
    check("robots: path deviates around the parked robot (> 0.3 m)", dev > 0.3, f"max |dy|={dev:.2f}")
    sep = min_separation(a, b)
    check("robots: bodies never overlap while passing (center distance > 0.45 m)", sep > 0.45, f"min={sep:.2f}")
    check("robots: the parked robot is not disturbed", b.state["state"] == "IDLE" and not b.state["errors"], f"errors={b.state['errors']}")
    # 2) 정면 교차: 서로 자리를 바꾼다
    with a.lock, b.lock:
        a.truths.clear(); b.truths.clear()
    a.order("p2", ["12,6,3.14"])
    b.order("q2", ["20,6,0"])
    sa = a.idle_done("p2", 120)
    sb = b.idle_done("q2", 120)
    pa, pb = a.pose(), b.pose()
    check("robots: head-on, both arrive", sa is not None and sb is not None and dist((pa["x"], pa["y"]), (12, 6)) < 0.05 and dist((pb["x"], pb["y"]), (20, 6)) < 0.05,
          f"a={pa} {a.state['state']} {a.state['errors']} b={pb} {b.state['state']} {b.state['errors']}")
    sep = min_separation(a, b)
    check("robots: head-on, bodies never overlap (center distance > 0.45 m)", sep > 0.45, f"min={sep:.2f}")
    # 3) 꺼진 로봇은 세계에서 사라진다
    b.stop()
    time.sleep(1.0)
    check("robots: a robot that shut down is no longer detected", not a.state.get("detected_obstacles"), f"detected={a.state.get('detected_obstacles')}")
    a.stop()


def test_world():
    """월드 장애물 (retained sim/v1/world/obstacles): 로봇이 뜨기 전에 놓은 것도 보이고, 지우면 사라진다."""
    world_place("box1", 10, 6, 0.6, 0.6)
    try:
        r = Robot("W1", 5, 6)   # 장애물을 놓은 뒤에 뜬다 (예전 INJECT_OBSTACLE 방식으로는 못 받던 경우)
        r.order("w1", ["12.5,6,0"])   # 도착해도 장애물이 센서 범위(3 m) 안이다
        s = r.idle_done("w1", 90)
        p = r.pose()
        check("world: a robot started after the obstacle was placed avoids it and arrives",
              s is not None and s["state"] == "IDLE" and dist((p["x"], p["y"]), (12.5, 6)) < 0.05, f"pose={p} state={r.state['state']} errors={r.state['errors']}")
        with r.lock:
            dev = max(abs(st["pose"]["y"] - 6) for st in r.states)
            seen = any(st.get("detected_obstacles") for st in r.states)
        check("world: detour around the world obstacle (> 0.3 m off the straight line)", dev > 0.3 and seen, f"max |dy|={dev:.2f} seen={seen}")
        # 센서 범위 안에서 지우면 곧바로 사라지고 돌아올 때는 직선으로 간다
        # (범위 밖에서 지우면 로봇은 알 수 없어서 장애물 기억 시간 동안 계속 피한다: memory 시나리오)
        world_clear("box1")
        time.sleep(1.0)
        with r.lock:
            n0 = len(r.states)
        r.order("w2", ["5,6,3.14"])
        s = r.idle_done("w2", 90)
        with r.lock:
            back = r.states[n0:]
        dev = max(abs(st["pose"]["y"] - 6) for st in back)
        check("world: after clearing, drives straight back", s is not None and dev < 0.15 and not any(st.get("detected_obstacles") for st in back), f"max |dy|={dev:.2f}")
        # --no-world-obstacles 면 무시한다
        r.stop()
        world_place("box2", 10, 6, 0.6, 0.6)
        r2 = Robot("W2", 5, 6, extra=["--no-world-obstacles"])
        time.sleep(1.0)
        check("world: --no-world-obstacles ignores world obstacles", not r2.state.get("detected_obstacles"), f"detected={r2.state.get('detected_obstacles')}")
        r2.stop()
    finally:
        world_clear("box1")
        world_clear("box2")


def test_reroute():
    """로컬 창(5 m) 안에 우회로가 없어도 구간 전체를 다시 계획해 다른 길로 간다 (gui_map 0,0 -> 10.26,10.23, 8.3,0 에 1.5x3 상자)."""
    m = f"{REPO}/maps/gui_map/fms_map.json"
    r = Robot("G1", 0, 0, mapfile=m)
    r.order("rr1", ["10.26,10.23,0"])
    r.wait(lambda s: s["pose"]["x"] > 3, 30)
    _place("G1", 8.3, 0, 1.5, 3.0)
    s = r.idle_done("rr1", 120)
    p = r.pose()
    errs = [e["error_type"] for st in r.states for e in st.get("errors", [])]
    check("reroute: arrives at the goal without error", s is not None and dist((p["x"], p["y"]), (10.26, 10.23)) < 0.05 and not errs, f"pose={p} errors={errs}")
    with open(f"{SCR}/G1.log") as f:
        log = f.read()
    check("reroute: the leg was replanned around the obstacle", "rerouted the whole leg" in log or "detour around" in log)
    r.stop()


def _both_blocked(rid, extra):
    """gui_map 0,0 -> 10.26,10.23: 8.3,0 상자로 남쪽 길(x~3.4)로 돌게 한 뒤 그 길도 막는다. 로봇이 동쪽 원래 길(x>5, y<3)로 되돌아가는지(왕복) 본다."""
    m = f"{REPO}/maps/gui_map/fms_map.json"
    r = Robot(rid, 0, 0, mapfile=m, extra=extra)
    r.order(rid + "o", ["10.26,10.23,0"])
    r.wait(lambda s: s["pose"]["x"] > 3, 30)
    _place(rid, 8.3, 0, 1.5, 3.0)
    r.wait(lambda s: s["pose"]["y"] > 1.5 and s["pose"]["x"] < 4.0, 40)   # 남쪽 길로 접어들었다
    _place(rid, 3.4, 5.0, 3.0, 3.0)
    t0 = time.time()
    xmax = 0.0   # 동쪽 원래 길(x>5, y<3)로 되돌아간 정도
    while time.time() - t0 < 40:
        q = r.pose()
        if q["y"] < 3.0:
            xmax = max(xmax, q["x"])
        time.sleep(0.2)
    with open(f"{SCR}/{rid}.log") as f:
        log = f.read()
    return r, xmax, log


def test_memory():
    """장애물 기억: 양쪽 길이 다 막혀도 방금 피한 길로 되돌아가 왕복하지 않는다. 기억을 끄면(--obstacle-memory 0) 되돌아간다 (비교)."""
    r, xmax, log = _both_blocked("M1", [])
    check("memory: does not flip back to the blocked east route", xmax < 5.0 and r.state["state"] != "ERROR", f"xmax={xmax:.2f}")
    check("memory: out-of-range obstacle is remembered", "obstacle remembered" in log)
    # 앞 장애물을 치우면 (기억 중인 쪽은 그대로) 남쪽 길로 도착한다
    _clear("M1")   # 로봇은 치워졌는지 모른다: 범위 안에서 다시 보고 확인하면 풀린다
    s = r.idle_done("M1o", 120)
    p = r.pose()
    check("memory: arrives after the blocking obstacles are removed", s is not None and dist((p["x"], p["y"]), (10.26, 10.23)) < 0.05, f"pose={p}")
    r.stop()
    r, xmax0, log0 = _both_blocked("M0", ["--obstacle-memory", "0"])
    check("memory: without memory the robot flips back east (control)", xmax0 >= 5.0, f"xmax={xmax0:.2f}")
    r.stop()


def test_shortcut():
    """우회 경로가 목적지를 지나쳐 합류점까지 갔다가 되돌아오지 않는다 (gui_map 14.9,0 -> 14.9,3.4, 동쪽 복도를 막음).
    정상 경로는 블록 동쪽으로 돈다. 막히면 서쪽으로 돌아 아래 복도의 목적지에 바로 서야 한다 (예전엔 장애물 뒤 합류점 x~16 까지 갔다 왔다)."""
    m = f"{REPO}/maps/gui_map/fms_map.json"
    r = Robot("K1", 14.9, 0, mapfile=m)
    r.order("sc1", ["14.9,3.4,0"])
    went_east = r.wait(lambda s: s["pose"]["x"] > 15.6, 20) is not None
    _place("K1", 16.35, 1.92, 0.95, 0.1)
    xmax = 0.0   # 아래 복도(y > 2.8)에서 동쪽으로 간 최대 x
    t0 = time.time()
    while time.time() - t0 < 60:
        s = r.state
        if s["pose"]["y"] > 2.8:
            xmax = max(xmax, s["pose"]["x"])
        if s["state"] in ("IDLE", "ERROR") and s["order_id"] is None and time.time() - t0 > 1:
            break
        time.sleep(0.05)
    p = r.pose()
    took = time.time() - t0
    check("shortcut: nominal route goes east first (test setup)", went_east)
    check("shortcut: arrives at the goal", dist((p["x"], p["y"]), (14.9, 3.4)) < 0.05 and r.state["state"] == "IDLE", f"pose={p} t={took:.1f}s")
    check("shortcut: does not overshoot the goal to the old rejoin point", 0 < xmax < 15.3, f"xmax in lower corridor={xmax:.2f}")
    r.stop()


def _local_plan_p95(rid):
    p95 = None
    with open(f"{SCR}/{rid}.log") as f:
        for line in f:
            m = re.search(r"local plan: n=\d+ p95=([\d.]+) ms", line)
            if m:
                p95 = float(m.group(1))
    return p95


def test_detour():
    """넓은 홀: 경로 위 1x3 상자를 서지 않고 돌아간다 (달리면서 우회). 몸체가 상자와 겹치지 않고, 계획 시간 p95 <= 200 ms."""
    r = Robot("D2", 3, 6)
    r.order("dt1", ["30,6,0"])
    r.wait(lambda s: s["pose"]["x"] > 8 and s["velocity"] > 0.8, 30)
    t_place = time.time()
    _place("D2", 20, 6, 1.0, 3.0)
    s = r.idle_done("dt1", 90)
    p = r.pose()
    with r.lock:
        waited = any(st.get("sub_state") == "WAITING" and st["pose"]["x"] > 10 for st in r.states)
        errs = [e["error_type"] for st in r.states for e in st.get("errors", [])]
    check("detour: arrives at the goal without error", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.05 and not errs, f"pose={p} errors={errs}")
    check("detour: never stops to wait", not waited)
    rows = [x for x in r.csv_rows() if x["order_id"] == "dt1"]
    def gap(x):   # 몸체 중심에서 상자(20,6, 1x3)까지의 거리 (원판 반지름 0.21 보다 커야 겹치지 않는다)
        dx = max(abs(float(x["x"]) - 20) - 0.5, 0.0)
        dy = max(abs(float(x["y"]) - 6) - 1.5, 0.0)
        return math.hypot(dx, dy)
    mingap = min(gap(x) for x in rows)
    check("detour: body never overlaps the obstacle (center gap > 0.21 m)", mingap > 0.21, f"min gap={mingap:.3f} m")
    near = [abs(float(x["v_fwd"])) for x in rows if 16 < float(x["x"]) < 24]
    check("detour: keeps moving while passing (min v > 0.25 m/s)", near and min(near) > 0.25, f"min v={min(near) if near else None}")
    p95 = _local_plan_p95("D2")
    check("detour: local plan time p95 <= 200 ms", p95 is not None and p95 <= 200.0, f"p95={p95} ms")
    r.stop()


def test_detour_clear():
    """우회 중 장애물을 치우면 정상 경로로 복귀해서 도착한다."""
    r = Robot("D3", 3, 6)
    r.order("dc1", ["30,6,0"])
    r.wait(lambda s: s["pose"]["x"] > 8 and s["velocity"] > 0.8, 30)
    _place("D3", 20, 6, 1.0, 3.0)
    s = r.wait(lambda s: abs(s["pose"]["y"] - 6) > 0.8, 40)
    check("detour_clear: robot leaves the straight line to go around", s is not None, f"pose={r.pose()}")
    _clear("D3")
    s = r.idle_done("dc1", 90)
    p = r.pose()
    errs = r.state["errors"]
    check("detour_clear: rejoins and arrives after the obstacle is gone", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.05 and not errs, f"pose={p} errors={errs}")
    r.stop()


def test_zone_obstacles():
    """가로지르는 장애물(구역 스폰, 시드 고정) 속에서 장거리 오더가 충돌 없이 끝난다 (현재 위치만 보므로 늦게 반응할 수 있다)."""
    r = Robot("Z1", 3, 6, extra=["--obstacle-zone", "14,2,24,10", "--obstacle-spawn-interval", "2", "--obstacle-max-count", "3",
                                  "--obstacle-width", "0.6", "--obstacle-height", "0.6", "--obstacle-speed", "0.5", "--obstacle-seed", "7"])
    r.order("z1", ["30,6,0"])
    s = r.idle_done("z1", 150)
    p = r.pose()
    errs = [e["error_type"] for st in r.states for e in st.get("errors", [])]
    check("zone obstacles: completes without COLLISION", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.05 and "COLLISION" not in errs, f"pose={p} errors={set(errs)}")
    r.stop()


def test_dock_interrupt():
    r = Robot("K1", 30, 6, extra=["--battery", "90"])
    r.order("k1", ["30,6,0"], "DOCK", ["entry=BACKWARD", "undock_distance=1.5", "min_charge=0"])
    r.wait(lambda s: s.get("sub_state") == "ACTING", 30)
    time.sleep(2.0)
    r.instant("PAUSE")
    r.wait(lambda s: s["paused"], 5)
    time.sleep(0.3)
    p = r.pose()
    check("dock pause: stops mid-insert", r.state["velocity"] < 0.02 and 30.0 < p["x"] < 31.5, f"pose={p}")
    r.instant("RESUME")
    s = r.idle_done("k1", 60)
    p = r.pose()
    check("dock resume: finishes insert at (30,6)", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.03 and abs(p["theta"]) < 0.03, f"pose={p}")
    # 진입 중 취소 -> 다음 오더가 먼저 언도킹
    r.order("k2", ["40,6,0"])
    time.sleep(0.3)
    r.instant("CANCEL_ORDER")
    time.sleep(0.5)
    r.order("k3", ["40,6,0"])
    s = r.idle_done("k3", 60)
    p = r.pose()
    check("dock: order after cancel undocks first and arrives", s is not None and dist((p["x"], p["y"]), (40, 6)) < 0.03, f"pose={p}")
    r.stop()


def test_physics_topic():
    r = Robot("T1", 20, 6, extra=["--friction", "0.5"])
    got = []
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.on_message = lambda cl, u, m: got.append((time.time(), json.loads(m.payload)))
    c.connect("localhost", PORT)
    c.subscribe("fms/v1/T1/physics")
    c.loop_start()
    r.order("p1", ["23,6,0"], "STRAIGHT", ["distance=-1.5"])
    r.idle_done("p1", 40)
    c.loop_stop()
    check("physics topic: published", len(got) > 20, f"n={len(got)}")
    if got:
        d = got[-1][1]
        need = {"x", "y", "theta", "odom_x", "v_fwd", "v_lat", "yaw_rate", "wheel_w", "wheel_cmd", "slip", "cmd_v", "cmd_w",
                "wall_force", "ghost", "collided", "at_rest", "friction", "mass", "path_points", "path_remaining", "goal", "sim_time"}
        check("physics topic: has all fields", need <= set(d), f"missing={need - set(d)}")
        check("physics topic: friction/mass reported", abs(d["friction"] - 0.5) < 1e-6 and abs(d["mass"] - 60) < 1e-6)
        span = got[-1][0] - got[0][0]
        rate = len(got) / span if span > 0 else 0
        check("physics topic: rate ~20 Hz (every tick)", 15 <= rate <= 25, f"rate={rate:.1f}")
        check("physics topic: reverse straight has negative v_fwd", min(g[1]["v_fwd"] for g in got) < -0.1,
              f"min v_fwd={min(g[1]['v_fwd'] for g in got):.3f}")
    r.stop()


def test_navigation():
    """sim/v1/{id}/navigation: Global 경로는 바로 앞 노드 구간만, 도착 뒤 다음 구간. 우회 중엔 detour_active, 끝나면 비운다."""
    m = f"{REPO}/maps/gui_map/fms_map.json"
    r = Robot("N1", 0, 0, mapfile=m)
    got = []
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.on_message = lambda cl, u, msg: got.append(json.loads(msg.payload))
    c.connect("localhost", PORT)
    c.subscribe("sim/v1/N1/navigation", 1)
    c.loop_start()
    time.sleep(1.0)
    check("navigation: idle message (retained/heartbeat) before any order", got and got[-1]["status"] == "IDLE" and got[-1]["global_path"] == [])
    r.order("n1", ["3,0", "10.26,10.23,0"])
    t0 = time.time()
    while time.time() - t0 < 10 and not (got and got[-1]["order_id"] == "n1" and got[-1]["status"] == "READY"):
        time.sleep(0.1)
    d = got[-1]
    gp = d["global_path"]
    check("navigation: global path published for the order", d["order_id"] == "n1" and d["status"] == "READY" and len(gp) > 1, f"status={d['status']} n={len(gp)}")
    if gp:
        check("navigation: only the next node is planned (a->b, not b->c)", dist(gp[-1], (3, 0)) < 0.05 and min(dist(p, (10.26, 10.23)) for p in gp) > 3,
              f"end={gp[-1]}")
        check("navigation: plan radius reported", d["plan_radius"] > 0, f"radius={d['plan_radius']}")
    # (3,0) 도착 뒤 다음 구간(b->c)이 생긴다
    t0 = time.time()
    while time.time() - t0 < 30 and not (got[-1]["global_path"] and dist(got[-1]["global_path"][-1], (10.26, 10.23)) < 0.05):
        time.sleep(0.1)
    d = got[-1]
    check("navigation: next leg is planned after arriving at the node", d["global_path"] and dist(d["global_path"][-1], (10.26, 10.23)) < 0.05,
          f"end={d['global_path'][-1] if d['global_path'] else None}")
    r.wait(lambda s: s["pose"]["x"] > 4, 30)
    _place("N1", 8.3, 0, 1.5, 3.0)
    t0 = time.time()
    while time.time() - t0 < 40 and not any(g["detour_active"] for g in got):
        time.sleep(0.2)
    check("navigation: detour_active reported while avoiding", any(g["detour_active"] for g in got))
    s = r.idle_done("n1", 120)
    time.sleep(6)   # 하트비트/변경 반영
    d = got[-1]
    check("navigation: cleared after the order ends", s is not None and d["status"] == "IDLE" and d["global_path"] == [], f"status={d['status']} n={len(d['global_path'])}")
    c.loop_stop()
    r.stop()


def test_estimated_pose():
    """보정 없음(--loc-rate 0): 제어와 State.pose 는 순수 오도메트리로 돈다. 저마찰이면 참 자세가 추정에서 벌어진다."""
    r = Robot("E1", 3, 3, extra=["--friction", "0.3", "--loc-rate", "0"])
    r.order("e1", ["12,3", "12,9,0"])
    s = r.idle_done("e1", 60)
    p = r.pose()
    rows = [x for x in r.csv_rows() if x["order_id"] == "e1"]
    last = rows[-1]
    tx, ty = float(last["x"]), float(last["y"])
    ox, oy = float(last["odom_x"]), float(last["odom_y"])
    # 처음 위치는 스캔 매칭이 정한다 (추정 = T∘오도메트리, T 는 매칭 결과). 보정이 없으니 그 뒤로 T 가 그대로여야 한다
    first = rows[0]
    th = float(first["est_theta"]) - float(first["odom_theta"])
    c, sn = math.cos(th), math.sin(th)
    tx0 = float(first["est_x"]) - (c * float(first["odom_x"]) - sn * float(first["odom_y"]))
    ty0 = float(first["est_y"]) - (sn * float(first["odom_x"]) + c * float(first["odom_y"]))
    px, py = c * ox - sn * oy + tx0, sn * ox + c * oy + ty0
    check("no localization: State.pose follows raw odometry (fixed offset)", dist((p["x"], p["y"]), (px, py)) < 0.02,
          f"state=({p['x']:.3f},{p['y']:.3f}) odom∘T=({px:.3f},{py:.3f})")
    check("no localization: arrives at goal by estimate", s is not None and dist((p["x"], p["y"]), (12, 9)) < 0.03)
    off = dist((tx, ty), (p["x"], p["y"]))
    check("no localization: true pose differs from estimate at low friction (> 3 cm)", off > 0.03, f"off={off * 100:.1f} cm")
    r.stop()


def test_localization():
    """기본 켜짐(라이다 위치보정 시뮬레이션): 저마찰에서도 추정이 참 자세를 따라가고, 보정 전 오도메트리는 벌어진다."""
    r = Robot("L1", 3, 3, extra=["--friction", "0.3"])
    r.order("l1", ["12,3", "12,9", "6,9", "6,12,0"])
    s = r.idle_done("l1", 90)
    p = r.pose()
    rows = [x for x in r.csv_rows() if x["order_id"] == "l1"]
    est_err = [dist((float(x["est_x"]), float(x["est_y"])), (float(x["x"]), float(x["y"]))) for x in rows]
    odom_err = [dist((float(x["odom_x"]), float(x["odom_y"])), (float(x["x"]), float(x["y"]))) for x in rows]
    last = rows[-1]
    check("localization: order completes at low friction", s is not None and s["state"] == "IDLE", f"state={s and s['state']}")
    check("localization: estimate error stays small (max < 8 cm)", max(est_err) < 0.08, f"max={max(est_err) * 100:.1f} cm")
    check("localization: raw odometry drifts far more than estimate", max(odom_err) > 5 * max(est_err),
          f"odom max={max(odom_err) * 100:.0f} cm est max={max(est_err) * 100:.1f} cm")
    check("localization: true final position within 5 cm of goal",
          dist((float(last["x"]), float(last["y"])), (6, 12)) < 0.05, f"true=({last['x']},{last['y']}) state=({p['x']:.2f},{p['y']:.2f})")
    # 위치 재지정(없으니 생략): 보정 통계가 토픽에 올라오는지만 본다
    r.stop()


def test_gui_offscreen():
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    p = subprocess.Popen([f"{REPO}/build/bin/robot_client", "--id", "U1", "--port", str(PORT), "--x", "3", "--y", "3", "--map", MAP, "--gui"],
                         env=dict(env, AMR_GUI_SNAPSHOT=f"{SCR}/gui_U1.png"), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=REPO)
    time.sleep(3.0)
    alive = p.poll() is None
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(5)
    except Exception:
        p.kill()
    out = p.stdout.read().decode(errors="ignore")
    check("gui(offscreen): starts and runs", alive and "physics: wall boxes built" in out, out[-200:] if not alive else "")
    check("gui(offscreen): snapshot saved (physics panel renders)", os.path.exists(f"{SCR}/gui_U1.png"))


def test_default_localized():
    """--require-localization 없이 켜면 시작 자세에서 바로 LOCALIZED 이고 힌트 없이 오더가 돈다."""
    r = Robot("D1", 3, 3, localize=False, require=False)
    check("default: starts LOCALIZED", r.state["localization"] == "LOCALIZED", f"loc={r.state['localization']}")
    r.order("d1", ["5,3"])
    s = r.idle_done("d1", 40)
    p = r.pose()
    check("default: drives without a pose hint", s is not None and dist((p["x"], p["y"]), (5, 3)) < 0.05, f"pose={p}")
    r.stop()


def test_script():
    """send_order --script: MOVE 2개 + WAIT + PICK(복제 노드) 이 Order 하나로 묶여 끝까지 돈다."""
    r = Robot("S1", 3, 3)
    path = f"{SCR}/job.txt"
    open(path, "w").write("0 START\n1 MOVE 5 3 0   // x y th\n2 WAIT 1\n3 PICK 1\n4 MOVE 5 6 1.57\n5 END\n")
    # --order-id 를 고정한다 (기본값은 파일명-HHMMSS 라 action_id 를 예측할 수 없다)
    run = subprocess.run([f"{REPO}/build/bin/send_order", "S1", "--port", str(PORT), "--order-id", "job", "--script", path], capture_output=True, text=True)
    check("script: published", run.returncode == 0, run.stderr.strip())
    s = r.idle_done("job", 60)
    p = r.pose()
    check("script: reaches last MOVE", s is not None and dist((p["x"], p["y"]), (5, 6)) < 0.05, f"pose={p}")
    with r.lock:
        seen = {(a["action_id"], a["status"])
                for st in r.states for a in st.get("action_states", [])}
    ids = {i for i, _ in seen}
    check("script: WAIT and PICK ran", {"job-A0", "job-A1"} <= ids and ("job-A1", "FINISHED") in seen, f"seen={sorted(seen)}")
    open(f"{SCR}/loop.txt", "w").write("1 MOVE 5 3\n2 MOVE 5 4\n3 GOTO 1 2\n")
    dry = subprocess.run([f"{REPO}/build/bin/send_order", "S1", "--script", f"{SCR}/loop.txt", "--dry-run"], capture_output=True, text=True)
    check("script: GOTO 1 2 expands to 6 nodes", dry.stdout.count("  N") == 6, dry.stdout.strip().replace("\n", " | ") or dry.stderr)
    run = subprocess.run([f"{REPO}/build/bin/send_order", "S1", "--port", str(PORT), "--script", f"{SCR}/loop.txt", "--order-id", "loop"], capture_output=True, text=True)
    s = r.idle_done("loop", 60)
    p = r.pose()
    check("script: GOTO loop runs to the end", s is not None and dist((p["x"], p["y"]), (5, 4)) < 0.05, f"pose={p}")
    bad = open(f"{SCR}/bad.txt", "w"); bad.write("WAIT 1\n"); bad.close()
    run = subprocess.run([f"{REPO}/build/bin/send_order", "S1", "--port", str(PORT), "--script", f"{SCR}/bad.txt"], capture_output=True, text=True)
    check("script: invalid script is rejected with line number", run.returncode != 0 and "line 1" in run.stderr, run.stderr.strip())
    r.stop()


ALL = {
    "move": test_move, "straight": test_straight, "dock": test_dock_park, "collision": test_collision,
    "default": test_default_localized, "script": test_script, "curve": test_curve, "lazy": test_lazy_plan,
    "pause": test_pause_cancel, "ghost": test_ghost_start, "lowfric": test_low_friction, "slip": test_normal_slip_small,
    "loop": test_loop_map, "longrun": test_longrun, "obstacle": test_obstacle, "collide_obstacle": test_collide_obstacle, "robots": test_robots, "world": test_world, "reroute": test_reroute, "memory": test_memory, "shortcut": test_shortcut, "navigation": test_navigation, "detour": test_detour, "detour_clear": test_detour_clear, "zone": test_zone_obstacles, "dockint": test_dock_interrupt, "gui": test_gui_offscreen, "topic": test_physics_topic, "estimate": test_estimated_pose, "localization": test_localization, "unlocalized": test_unlocalized, "charger": test_charger, "battery": test_battery,
}

if __name__ == "__main__":
    for need in (f"{REPO}/build/bin/robot_client", f"{REPO}/build/bin/send_order", f"{REPO}/build/bin/place_obstacle"):
        if not os.path.exists(need):
            sys.exit(f"{need} 없음: 먼저 make 를 실행하라")
    print(f"로그/기록 폴더: {SCR}")
    names = sys.argv[1:] or list(ALL)
    shutil.rmtree(REC, ignore_errors=True)
    start_broker()
    try:
        for n in names:
            print(f"== {n}", flush=True)
            try:
                ALL[n]()
            except Exception as e:  # noqa
                check(f"{n}: exception", False, repr(e))
    finally:
        stop_all()
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} PASS")
    sys.exit(1 if bad else 0)
