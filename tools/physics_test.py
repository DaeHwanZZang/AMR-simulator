"""물리(MuJoCo) 통합 테스트. empty_hall1 에서 robot_client 를 실제로 띄워 MQTT 로 시험한다.

리포 루트에서: .venv/bin/python tools/physics_test.py [시나리오 이름...]
시나리오: move straight dock collision pause ghost lowfric slip loop longrun obstacle dockint gui topic estimate localization (생략하면 전부, 약 8분)
mosquitto 를 자체 포트(18851)로 띄우므로 이미 떠 있는 브로커와 부딪히지 않는다. 로봇 로그와 기록 CSV 는 임시 폴더에 남는다.
"""
import csv, glob, json, math, os, shutil, signal, subprocess, sys, tempfile, threading, time

import paho.mqtt.client as mqtt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCR = tempfile.mkdtemp(prefix="physics_test_")
os.environ["PATH"] = os.environ.get("PATH", "") + ":/opt/homebrew/sbin:/usr/local/sbin"  # mosquitto 위치 (macOS brew)
PORT = 18851
MAP = f"{REPO}/maps/empty_hall1/fms_map.json"
REC = f"{SCR}/rec_test"

results = []
procs = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)


class Robot:
    def __init__(self, rid, x, y, theta=0.0, extra=(), record=True, mapfile=MAP):
        self.rid = rid
        self.state = None
        self.states = []
        self.lock = threading.Lock()
        cmd = [f"{REPO}/robot_client", "--id", rid, "--port", str(PORT), "--x", str(x), "--y", str(y),
               "--theta", str(theta), "--seed", "1", "--speed-noise", "0"]
        if mapfile:
            cmd += ["--map", mapfile]
        if record:
            cmd += ["--record", REC]
        cmd += list(extra)
        self.log = open(f"{SCR}/{rid}.log", "w")
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT, cwd=REPO)
        procs.append(self.p)
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.c.on_message = self._on
        self.c.connect("localhost", PORT)
        self.c.subscribe(f"fms/v1/{rid}/state")
        self.c.loop_start()
        self.wait(lambda s: True, 10, "first state")

    def _on(self, c, u, msg):
        s = json.loads(msg.payload)
        with self.lock:
            self.state = s
            self.states.append(s)

    def wait(self, cond, timeout, what=""):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                s = self.state
            if s is not None and cond(s):
                return s
            time.sleep(0.05)
        return None

    def order(self, oid, path, action=None, params=(), upd=0):
        a = [f"{REPO}/tools/send_order", self.rid, "--port", str(PORT), "--order-id", oid, "--update", str(upd), "--path"] + path
        if action:
            a += ["--action", action]
            for p in params:
                a += ["--param", p]
        r = subprocess.run(a, capture_output=True, text=True)
        if r.returncode != 0:
            print("send_order failed", r.stdout, r.stderr)
        return r.returncode == 0

    def instant(self, kind):
        subprocess.run([f"{REPO}/tools/send_order", self.rid, "--port", str(PORT), "--instant", kind], capture_output=True)

    def pose(self):
        return self.state["pose"]

    def idle_done(self, oid, timeout):
        """오더 oid 가 끝나(IDLE 이고 order_id 없음) 기다린다"""
        def done(s):
            return s["state"] in ("IDLE", "ERROR", "CHARGING") and s["order_id"] is None
        time.sleep(0.5)
        return self.wait(done, timeout)

    def stop(self):
        self.c.loop_stop()
        self.p.send_signal(signal.SIGTERM)
        try:
            self.p.wait(3)
        except Exception:
            self.p.kill()
        self.log.close()

    def csv_rows(self):
        files = sorted(glob.glob(f"{REC}/{self.rid}_*.csv"))
        if not files:
            return []
        return list(csv.DictReader(open(files[-1])))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


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
    check("dock BACKWARD: docked at (30,6) heading 0", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.03 and abs(p["theta"]) < 0.03,
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


def test_ghost_start():
    r = Robot("G1", 0.0, 0.0)  # 맵 여백(UNKNOWN) 위
    time.sleep(2.0)
    p = r.pose()
    check("ghost start: no ejection", abs(p["x"]) < 0.01 and abs(p["y"]) < 0.01, f"pose={p}")
    r.order("g1", ["5,5,0"])
    s = r.wait(lambda s: any(e["error_type"] == "NO_PATH" for e in s["errors"]), 10)
    check("ghost start: order fails with NO_PATH", s is not None, f"errors={s and s['errors']}")
    rows = r.csv_rows()
    check("ghost start: ghost flag recorded", any(x["ghost"] == "1" for x in rows))
    p = r.pose()
    check("ghost start: still at start", abs(p["x"]) < 0.01 and abs(p["y"]) < 0.01, f"pose={p}")
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


def test_obstacle():
    r = Robot("O1", 3, 6)
    r.order("b1", ["30,6,0"])
    r.wait(lambda s: s["pose"]["x"] > 8 and s["velocity"] > 0.8, 30)
    subprocess.run([f"{REPO}/tools/place_obstacle", "O1", "--port", str(PORT), "--x", "20", "--y", "6", "--width", "1.0", "--height", "3.0"], capture_output=True)
    s = r.wait(lambda s: s["sub_state"] == "WAITING" if s.get("sub_state") else False, 20)
    check("obstacle: robot waits when path blocked", s is not None)
    time.sleep(4)
    rows = r.csv_rows()
    amax = 0.0
    for a, b in zip(rows, rows[1:]):
        if abs(float(a["v_fwd"])) > 0.6:
            amax = max(amax, abs(float(b["v_fwd"]) - float(a["v_fwd"])) / max(1e-3, float(b["t"]) - float(a["t"])))
    check("obstacle: no abrupt stops from cruise speed (|a| < 2 m/s^2 above 0.6 m/s)", amax < 2.0, f"max |a|={amax:.2f}")
    subprocess.run([f"{REPO}/tools/place_obstacle", "O1", "--port", str(PORT), "--clear"], capture_output=True)
    s = r.idle_done("b1", 60)
    p = r.pose()
    check("obstacle: resumes and arrives after clearing", s is not None and dist((p["x"], p["y"]), (30, 6)) < 0.04, f"pose={p}")
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
    check("no localization: State.pose follows raw odometry", dist((p["x"], p["y"]), (ox, oy)) < 0.02,
          f"state=({p['x']:.3f},{p['y']:.3f}) odom=({ox:.3f},{oy:.3f})")
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
    p = subprocess.Popen([f"{REPO}/robot_client", "--id", "U1", "--port", str(PORT), "--x", "3", "--y", "3", "--map", MAP, "--gui"],
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


ALL = {
    "move": test_move, "straight": test_straight, "dock": test_dock_park, "collision": test_collision,
    "pause": test_pause_cancel, "ghost": test_ghost_start, "lowfric": test_low_friction, "slip": test_normal_slip_small,
    "loop": test_loop_map, "longrun": test_longrun, "obstacle": test_obstacle, "dockint": test_dock_interrupt, "gui": test_gui_offscreen, "topic": test_physics_topic, "estimate": test_estimated_pose, "localization": test_localization,
}

if __name__ == "__main__":
    for need in (f"{REPO}/robot_client", f"{REPO}/tools/send_order", f"{REPO}/tools/place_obstacle"):
        if not os.path.exists(need):
            sys.exit(f"{need} 없음: 먼저 make 를 실행하라")
    print(f"로그/기록 폴더: {SCR}")
    names = sys.argv[1:] or list(ALL)
    shutil.rmtree(REC, ignore_errors=True)
    mq = subprocess.Popen(["mosquitto", "-p", str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(mq)
    time.sleep(1.0)
    try:
        for n in names:
            print(f"== {n}", flush=True)
            try:
                ALL[n]()
            except Exception as e:  # noqa
                check(f"{n}: exception", False, repr(e))
    finally:
        for p in procs:
            try:
                p.kill()
            except Exception:
                pass
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} PASS")
    sys.exit(1 if bad else 0)
