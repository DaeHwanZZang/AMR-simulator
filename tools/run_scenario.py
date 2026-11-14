#!/usr/bin/env python3
"""다중 로봇 시나리오 러너. 시나리오 파일(JSON) 하나를 무인으로 돌리고 기대 결과를 검사한다.

리포 루트에서:
    .venv/bin/python tools/run_scenario.py scripts/scenarios/head_on.json [--viewer] [--time-scale N] [--keep]

자체 mosquitto(빈 포트)를 띄우고 로봇 N 대를 실행한 뒤, 시각에 맞춰 월드 장애물과 오더를 보내고, 전부 끝나면(또는 timeout)
expect 를 검사한다. 종료 코드: 전부 통과 0, 하나라도 실패 1. 먼저 `make -C robot` 으로 바이너리를 만들어야 한다.

  --viewer      뷰어(viewer/tools/fleet_monitor_qt.py)를 같은 브로커에 붙여 실제 창으로 띄운다
  --time-scale  시간 배속 (시나리오의 time_scale 보다 우선). 시나리오의 시각/timeout 은 시뮬 시간(초)이다
  --keep        끝난 뒤 로봇/브로커/뷰어를 그대로 두고 Enter 를 기다린다 (결과를 뷰어에서 살펴볼 때)

시나리오 형식:
{
  "map": "maps/empty_hall1/fms_map.json",          // 리포 루트 기준
  "time_scale": 1, "timeout": 120,
  "robots":    [{"id": "A", "x": 5, "y": 6, "theta": 0, "at": 0, "args": ["--max-velocity", "0.8"]}],
  "obstacles": [{"id": "box", "x": 10, "y": 6, "width": 0.6, "height": 0.6, "at": 0, "clear_at": 30}],   // 월드 장애물
  "orders":    [{"robot": "A", "at": 1, "path": ["20,6,0"], "action": "DOCK", "params": ["entry=BACKWARD"]},
                {"robot": "A", "at": 1, "script": "scripts/examples/demo_job.txt"}],
  "expect":    [{"robot": "A", "pose": [20, 6], "tol": 0.05},      // 마지막 보고 위치
                {"no_fatal": true},                                  // 어떤 로봇도 FATAL 오류가 없었다
                {"min_separation": 0.45},                            // 모든 로봇 쌍의 참 위치 중심 거리 최솟값 (m)
                {"robot": "A", "error": "COLLISION"}]                // 그 오류가 한 번이라도 났다 (일부러 실패시키는 시나리오)
}
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from itertools import combinations

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args():
    ap = argparse.ArgumentParser(description="다중 로봇 시나리오 러너")
    ap.add_argument("scenario", help="시나리오 JSON 파일")
    ap.add_argument("--viewer", action="store_true", help="뷰어 창을 같이 띄운다")
    ap.add_argument("--time-scale", type=float, default=None, help="시간 배속 (시나리오 값보다 우선)")
    ap.add_argument("--keep", action="store_true", help="끝난 뒤 Enter 를 누를 때까지 그대로 둔다")
    return ap.parse_args()


def load_scenario(path):
    with open(path, encoding="utf-8") as f:
        sc = json.load(f)
    for key in ("map", "robots"):
        if key not in sc:
            sys.exit(f"{path}: '{key}' 가 없다")
    ids = [r["id"] for r in sc["robots"]]
    if len(set(ids)) != len(ids):
        sys.exit(f"{path}: 로봇 id 가 겹친다")
    for o in sc.get("orders", []):
        if o["robot"] not in ids:
            sys.exit(f"{path}: orders 의 로봇 '{o['robot']}' 가 robots 에 없다")
        if ("path" in o) == ("script" in o):
            sys.exit(f"{path}: order 는 path 와 script 중 하나만 가져야 한다")
    return sc


def main():
    args = parse_args()
    sc = load_scenario(args.scenario)
    scale = args.time_scale if args.time_scale else float(sc.get("time_scale", 1))
    os.environ["AMR_TEST_TIME_SCALE"] = str(scale)   # simharness 가 import 때 읽는다

    sys.path.insert(0, os.path.join(REPO, "robot", "tests"))
    import simharness as h

    for need in ("robot_client", "send_order"):
        if not os.path.exists(f"{REPO}/build/bin/{need}"):
            sys.exit(f"build/bin/{need} 없음: 먼저 make -C robot 을 실행하라")
    map_path = os.path.join(REPO, sc["map"])
    if not os.path.exists(map_path):
        sys.exit(f"맵이 없다: {map_path}")

    name = os.path.splitext(os.path.basename(args.scenario))[0]
    print(f"시나리오 {name}: 로봇 {len(sc['robots'])}대, 맵 {sc['map']}, 배속 {scale:g}, 브로커 포트 {h.PORT}, 로그 {h.SCR}", flush=True)

    # 사건 목록 (시뮬 시각 순). 같은 시각이면 로봇 시작 -> 장애물 -> 오더 순서
    events = []
    for r in sc["robots"]:
        events.append((float(r.get("at", 0)), 0, "robot", r))
    for o in sc.get("obstacles", []):
        events.append((float(o.get("at", 0)), 1, "place", o))
        if "clear_at" in o:
            events.append((float(o["clear_at"]), 1, "clear", o))
    for i, o in enumerate(sc.get("orders", [])):
        events.append((float(o.get("at", 0)), 2, "order", dict(o, _n=i)))
    events.sort(key=lambda e: (e[0], e[1]))

    robots = {}
    ordered = set()
    senders = []
    viewer = None
    ok = True

    def check(label, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        h.check(f"{name}: {label}", bool(cond), detail)

    try:
        h.start_broker()
        if args.viewer:
            viewer = subprocess.Popen([sys.executable, "tools/fleet_monitor_qt.py", map_path, "--port", str(h.PORT)],
                                      cwd=os.path.join(REPO, "viewer"))
            h.procs.append(viewer)
            time.sleep(2.0)   # 창이 뜨고 구독할 시간

        t0 = time.time()
        sim_now = lambda: (time.time() - t0) * scale  # noqa: E731
        for at, _, kind, e in events:
            while sim_now() < at:
                time.sleep(0.02)
            if kind == "robot":
                robots[e["id"]] = h.Robot(e["id"], e["x"], e["y"], e.get("theta", 0.0), extra=[str(a) for a in e.get("args", [])],
                                          record=False, mapfile=map_path, localize=False, require=False)
                print(f"  t={sim_now():6.1f}  robot {e['id']} at ({e['x']}, {e['y']})", flush=True)
            elif kind == "place":
                h.world_place(e["id"], e["x"], e["y"], e.get("width", 0.4), e.get("height", 0.4))
                print(f"  t={sim_now():6.1f}  obstacle {e['id']} at ({e['x']}, {e['y']})", flush=True)
            elif kind == "clear":
                h.world_clear(e["id"])
                print(f"  t={sim_now():6.1f}  obstacle {e['id']} cleared", flush=True)
            elif kind == "order":
                r = robots.get(e["robot"])
                if r is None:
                    check(f"order #{e['_n']} sent", False, f"robot {e['robot']} is not running yet")
                    continue
                oid = e.get("order_id", f"{name}-{e['robot']}-{e['_n']}")
                ordered.add(e["robot"])
                print(f"  t={sim_now():6.1f}  order {oid} -> {e['robot']}", flush=True)

                # send_order 는 한 번에 1 초쯤 걸린다. 같은 시각의 오더가 실제로 같이 나가도록 스레드로 보낸다
                def send(e=e, r=r, oid=oid):
                    if "script" in e:
                        run = subprocess.run([f"{REPO}/build/bin/send_order", e["robot"], "--port", str(h.PORT), "--order-id", oid,
                                              "--script", os.path.join(REPO, e["script"])], capture_output=True, text=True)
                        sent = run.returncode == 0
                        if not sent:
                            print(run.stderr.strip())
                    else:
                        sent = r.order(oid, [str(p) for p in e["path"]], e.get("action"), e.get("params", ()))
                    check(f"order #{e['_n']} sent to {e['robot']}", sent)

                t = threading.Thread(target=send, daemon=True)
                t.start()
                senders.append(t)
        for t in senders:
            t.join(15)

        # 오더를 받은 로봇이 전부 끝날 때까지 (오더 없음 + IDLE/ERROR/CHARGING) 기다린다
        timeout = float(sc.get("timeout", 120))
        time.sleep(0.6)   # 방금 보낸 오더가 State 에 반영될 시간

        def all_done():
            for rid in ordered:
                s = robots[rid].state
                if s is None or s["order_id"] is not None or s["state"] not in ("IDLE", "ERROR", "CHARGING"):
                    return False
            return True

        while not all_done() and sim_now() < timeout:
            time.sleep(0.05)
        check(f"finished within {timeout:g} s (sim)", all_done(), f"t={sim_now():.1f}")
        time.sleep(0.5)

        # 기대 결과
        for ex in sc.get("expect", []):
            if "pose" in ex:
                r = robots[ex["robot"]]
                p, tol = r.pose(), float(ex.get("tol", 0.05))
                d = h.dist((p["x"], p["y"]), ex["pose"])
                check(f"{ex['robot']} ends at ({ex['pose'][0]:g}, {ex['pose'][1]:g}) within {tol:g} m", d <= tol,
                      f"pose=({p['x']:.3f}, {p['y']:.3f}) d={d:.3f} state={r.state['state']}")
            elif ex.get("no_fatal"):
                bad = {}
                for rid, r in robots.items():
                    with r.lock:
                        fatal = sorted({e["error_type"] for st in r.states for e in st["errors"] if e["level"] == "FATAL"})
                    if fatal:
                        bad[rid] = fatal
                check("no FATAL error on any robot", not bad, str(bad) if bad else "")
            elif "min_separation" in ex:
                worst = min((h.min_separation(a, b), f"{a.rid}-{b.rid}") for a, b in combinations(robots.values(), 2)) \
                    if len(robots) >= 2 else (float("inf"), "-")
                check(f"robots stay at least {ex['min_separation']:g} m apart (centers)", worst[0] >= float(ex["min_separation"]),
                      f"min={worst[0]:.2f} ({worst[1]})")
            elif "error" in ex:
                r = robots[ex["robot"]]
                with r.lock:
                    seen = any(e["error_type"] == ex["error"] for st in r.states for e in st["errors"])
                check(f"{ex['robot']} raised {ex['error']}", seen)
            else:
                check("known expect entry", False, json.dumps(ex, ensure_ascii=False))

        if args.keep:
            try:
                input("끝났다. Enter 를 누르면 로봇/브로커/뷰어를 정리한다... ")
            except (EOFError, KeyboardInterrupt):
                pass
    except KeyboardInterrupt:
        print("\n중단")
        ok = False
    finally:
        for r in robots.values():
            try:
                r.stop()
            except Exception:  # noqa: BLE001
                pass
        h.stop_all()

    bad = [r for r in h.results if not r[1]]
    print(f"\n{len(h.results) - len(bad)}/{len(h.results)} PASS")
    return 0 if ok and not bad else 1


if __name__ == "__main__":
    sys.exit(main())
