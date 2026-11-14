"""시험 공용 헬퍼: 자체 mosquitto, robot_client 프로세스 래퍼(Robot), 월드 장애물, 결과 집계.

physics_test.py (회귀 테스트) 와 tools/run_scenario.py (다중 로봇 시나리오 러너) 가 같이 쓴다.
브로커는 자체 포트로 따로 띄운다 (기본 18851, 이미 쓰는 중이면 빈 포트). 남이 띄운 브로커를 재사용하지 않는 이유:
retained 메시지(월드 장애물, navigation)가 남아 있으면 다음 시험이 오염된다.
"""
import csv, glob, json, math, os, signal, socket, subprocess, tempfile, threading, time

import paho.mqtt.client as mqtt

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# 로봇 로그/기록 CSV 폴더. AMR_TEST_SCRATCH 로 위치를 정할 수 있다 (CI 가 실패 로그를 모을 때)
SCR = os.environ.get("AMR_TEST_SCRATCH") or tempfile.mkdtemp(prefix="physics_test_")
os.makedirs(SCR, exist_ok=True)
os.environ["PATH"] = os.environ.get("PATH", "") + ":/opt/homebrew/sbin:/usr/local/sbin:/usr/sbin"  # mosquitto 위치 (macOS brew, Linux)


def _pick_port(preferred=18851):
    """preferred 가 비어 있으면 그 포트, 누가 쓰고 있으면 OS 가 주는 빈 포트"""
    for port in (preferred, 0):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(("127.0.0.1", port))
            return s.getsockname()[1]
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError("no free port for the test broker")


PORT = _pick_port()
MAP = f"{REPO}/maps/empty_hall1/fms_map.json"
REC = f"{SCR}/rec_test"
# 시간 배속 (robot_client --time-scale). 기본 1 = 실시간. 벽시계로 기다리는 시나리오(pause, battery 등)는 1 에서만 검증됐다
TIME_SCALE = float(os.environ.get("AMR_TEST_TIME_SCALE", "1"))

results = []
procs = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)


class Robot:
    def __init__(self, rid, x, y, theta=0.0, extra=(), record=True, mapfile=MAP, localize=True, require=True):
        self.rid = rid
        self.state = None
        self.states = []
        self.truth = None     # 마지막 참 자세 (x, y, theta)
        self.truths = []      # (벽시계, x, y)
        self.lock = threading.Lock()
        cmd = [f"{REPO}/build/bin/robot_client", "--id", rid, "--port", str(PORT), "--x", str(x), "--y", str(y),
               "--theta", str(theta), "--seed", "1", "--speed-noise", "0"]
        if mapfile:
            cmd += ["--map", mapfile]
        if record:
            cmd += ["--record", REC]
        if TIME_SCALE != 1.0:
            cmd += ["--time-scale", str(TIME_SCALE)]
        if require:  # B 시나리오는 힌트로 위치를 잡는 흐름. 기본(플래그 없음) 동작은 default_localized 가 따로 본다
            cmd += ["--require-localization"]
        cmd += list(extra)
        self.log = open(f"{SCR}/{rid}.log", "w")
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT, cwd=REPO)
        procs.append(self.p)
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.c.on_message = self._on
        self.c.connect("localhost", PORT)
        self.c.subscribe(f"fms/v1/{rid}/state")
        self.c.subscribe(f"fms/v1/{rid}/physics")
        self.c.loop_start()
        self.wait(lambda s: True, 10, "first state")
        if localize and mapfile:
            # 켜자마자는 UNLOCALIZED. 시작 자세를 힌트로 줘서 스캔 매칭으로 위치를 잡는다
            self.hint(x, y, theta)
            if self.wait(lambda s: s["localization"] == "LOCALIZED", 10) is None:
                print(f"  ({rid}: localization failed: {self.state and self.state.get('errors')})")

    def _on(self, c, u, msg):
        s = json.loads(msg.payload)
        with self.lock:
            if msg.topic.endswith("/physics"):  # 참 자세 (20 Hz). 로봇 간 거리 검사에 쓴다
                self.truth = (s["x"], s["y"], s["theta"])
                self.truths.append((time.time(), s["x"], s["y"]))
                return
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
        a = [f"{REPO}/build/bin/send_order", self.rid, "--port", str(PORT), "--order-id", oid, "--update", str(upd), "--path"] + path
        if action:
            a += ["--action", action]
            for p in params:
                a += ["--param", p]
        r = subprocess.run(a, capture_output=True, text=True)
        if r.returncode != 0:
            print("send_order failed", r.stdout, r.stderr)
        return r.returncode == 0

    def instant(self, kind):
        subprocess.run([f"{REPO}/build/bin/send_order", self.rid, "--port", str(PORT), "--instant", kind], capture_output=True)

    def hint(self, x, y, theta=0.0):
        subprocess.run([f"{REPO}/build/bin/send_order", self.rid, "--port", str(PORT), "--instant", "INIT_POSITION",
                        "--param", f"x={x}", "--param", f"y={y}", "--param", f"theta={theta}"], capture_output=True)

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
        # 로봇이 쓰는 중에 읽으면 마지막 줄이 잘려 있다 (빈 칸 = None). 그런 줄은 버린다
        return [r for r in csv.DictReader(open(files[-1])) if None not in r.values()]


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])



def min_separation(a, b):
    """두 로봇(Robot)의 참 자세 기록에서 중심 간 최소 거리. 시각이 가장 가까운 표본끼리 비교한다."""
    with a.lock:
        ta = list(a.truths)
    with b.lock:
        tb = list(b.truths)
    if not ta or not tb:
        return float("inf")
    best, j = float("inf"), 0
    for t, x, y in ta:
        while j + 1 < len(tb) and abs(tb[j + 1][0] - t) <= abs(tb[j][0] - t):
            j += 1
        if abs(tb[j][0] - t) < 0.2:
            best = min(best, math.hypot(x - tb[j][1], y - tb[j][2]))
    return best


_world_client = None


def _world():
    global _world_client
    if _world_client is None:
        _world_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        _world_client.connect("localhost", PORT)
        _world_client.loop_start()
    return _world_client


def world_place(oid, x, y, w=0.4, h=0.4):
    """월드 장애물 (모든 로봇이 본다): retained sim/v1/world/obstacles/{oid}"""
    _world().publish(f"sim/v1/world/obstacles/{oid}", json.dumps({"x": x, "y": y, "width": w, "height": h}), qos=1, retain=True).wait_for_publish(2)


def world_clear(oid):
    _world().publish(f"sim/v1/world/obstacles/{oid}", b"", qos=1, retain=True).wait_for_publish(2)


def start_broker():
    """자체 mosquitto 를 PORT 로 띄운다. 접속될 때까지 기다린다."""
    mq = subprocess.Popen(["mosquitto", "-p", str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(mq)
    for _ in range(50):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect(("127.0.0.1", PORT))
            return mq
        except OSError:
            time.sleep(0.1)
        finally:
            s.close()
    raise RuntimeError(f"mosquitto did not start on port {PORT} (is it installed?)")


def stop_all():
    global _world_client
    if _world_client is not None:
        try:
            _world_client.loop_stop()
            _world_client.disconnect()
        except Exception:
            pass
        _world_client = None
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass
