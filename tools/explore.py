#!/usr/bin/env python3
"""Autonomous room mapping for the car.

Stop-and-scan loop: sweep the sonar, localize against the map so far, add
the sweep to the map, pick the nearest frontier, turn toward it, drive a
short hop, repeat until no frontier is left or the time is up. The safety
layer (tools/safety.py) is the only thing that sends motion frames.

  tools/explore.py [--duration 240] [--run-dir build/runs/<stamp>] [--dry-run]
                   [--fault stale-at=12 | estop-at=12] [--step 10]

Run directory: log.jsonl (every event), poses.jsonl (pose per cycle),
map.png (redrawn every cycle), map.npy (log-odds grid), frames/*.jpg
(camera captures), summary.json. A live view is served on
http://127.0.0.1:8766/ while the run is going.

Press q or space in the terminal to e-stop.
"""
import argparse, json, math, os, sys, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car                                   # noqa: E402
from sonar import Sonar, load_cal, MAX_RANGE          # noqa: E402
from safety import Safety, DIR_FWD, DIR_LEFT, DIR_RIGHT, DIR_BACK, MAX_PULSE_MS, MIN_FORWARD_CM  # noqa: E402
import mapping                                        # noqa: E402
from mapping import Grid, Matcher, next_waypoint, render  # noqa: E402
from vision import Vision                           # noqa: E402

VIEWER_HTML = """<!doctype html><meta charset=utf-8><title>Room map</title>
<body style="margin:0;background:#0f1115;color:#e6e8ee;font:14px -apple-system,sans-serif">
<div style="display:flex;gap:16px;padding:16px;flex-wrap:wrap">
<div><img id=m src=map.png style="width:600px;image-rendering:pixelated;border:1px solid #333"></div>
<div style="min-width:280px"><h3 style="margin:0 0 8px">Room map</h3><pre id=s style="white-space:pre-wrap;color:#8a92a6"></pre>
<img id=f src=frame.jpg style="width:320px;border:1px solid #333"></div></div>
<script>setInterval(()=>{const t=Date.now();document.getElementById('m').src='map.png?'+t;document.getElementById('f').src='frame.jpg?'+t;
fetch('status.json?'+t).then(r=>r.json()).then(j=>document.getElementById('s').textContent=JSON.stringify(j,null,1)).catch(()=>{})},2000)</script>"""


class RunLog:
    def __init__(self, run_dir):
        self.dir = run_dir
        os.makedirs(os.path.join(run_dir, "frames"), exist_ok=True)
        self.f = open(os.path.join(run_dir, "log.jsonl"), "a")
        self.poses = open(os.path.join(run_dir, "poses.jsonl"), "a")
        self.t0 = time.monotonic()
        self.status = {}

    def t(self):
        return round(time.monotonic() - self.t0, 3)

    def __call__(self, event, **fields):
        rec = {"t": self.t(), "event": event, **fields}
        self.f.write(json.dumps(rec) + "\n"); self.f.flush()
        print(f"[{rec['t']:7.2f}] {event} {json.dumps(fields) if fields else ''}", flush=True)

    def pose(self, pose, **fields):
        self.poses.write(json.dumps({"t": self.t(), "x": round(pose[0], 1), "y": round(pose[1], 1), "th": round(pose[2], 1), **fields}) + "\n")
        self.poses.flush()


class CameraGrab(threading.Thread):
    """Fetches /capture every couple of seconds and keeps the latest frame."""

    def __init__(self, host, run_dir, period=2.5):
        super().__init__(daemon=True)
        self.url = f"http://{host}/capture"; self.dir = run_dir; self.period = period
        self.latest = None; self.count = 0; self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                data = urllib.request.urlopen(self.url, timeout=6).read()
                if data[:2] == b"\xff\xd8":
                    self.latest = data; self.count += 1
                    with open(os.path.join(self.dir, "frame.jpg"), "wb") as f:
                        f.write(data)
            except Exception:
                pass
            self.stop.wait(self.period)

    def save(self, tag):
        if self.latest:
            with open(os.path.join(self.dir, "frames", f"{tag}.jpg"), "wb") as f:
                f.write(self.latest)


def start_viewer(run_dir, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                body, ctype = VIEWER_HTML.encode(), "text/html"
            else:
                fp = os.path.join(run_dir, path.lstrip("/"))
                if ".." in path or not os.path.isfile(fp):
                    self.send_response(404); self.end_headers(); return
                body = open(fp, "rb").read()
                ctype = {"png": "image/png", "jpg": "image/jpeg", "json": "application/json"}.get(fp.rsplit(".", 1)[-1], "text/plain")
            self.send_response(200); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body))); self.end_headers()
            self.wfile.write(body)
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    except OSError:
        return None
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def keyboard_estop(safety):
    """q or space in the terminal raises the e-stop. Skipped when stdin is not a tty."""
    if not sys.stdin.isatty():
        return
    import termios, tty
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    def run():
        try:
            tty.setcbreak(fd)
            while not safety.estop:
                ch = sys.stdin.read(1)
                if ch in ("q", " ", "\x03"):
                    safety.raise_estop("keyboard")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    threading.Thread(target=run, daemon=True).start()


def _num(reply):
    if not reply:
        return None
    i, j = reply.rfind("_"), reply.rfind("}")
    try:
        return int(reply[i + 1:j])
    except ValueError:
        return None


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class Explorer:
    def __init__(self, a):
        self.a = a
        self.log = RunLog(a.run_dir)
        self.car = Car(a.host)
        self.sonar = Sonar(self.car, on_reading=self._on_reading)
        self.safety = Safety(self.car, self.log, max_speed=max(a.speed, a.turn_speed), dry_run=a.dry_run)
        cal = load_cal()
        self.fwd_cps = cal.get("forward_cm_per_s", 30.0)
        self.turn_dps = cal.get("turn_deg_per_s", 120.0)
        if "forward_cm_per_s" not in cal or "turn_deg_per_s" not in cal:
            self.log("warn", msg="calibration.json incomplete; using defaults", fwd_cps=self.fwd_cps, turn_dps=self.turn_dps)
        self.grid = Grid(size_cm=a.size, cell_cm=a.cell)
        self.matcher = Matcher(self.grid)
        self.pose = (0.0, 0.0, 0.0)
        self.trail = [self.pose]
        self.cycles = 0; self.corrections = []; self.stuck = 0; self.last_target = None
        self.camera = CameraGrab(a.host, a.run_dir); self.camera.start()
        self.viewer = None if a.no_viewer else start_viewer(a.run_dir, a.viewer_port)
        self.fault = self._parse_fault(a.fault)
        self.frozen = False
        self.floor_base = None
        self.done = False
        self.abort = None
        self.no_frontier = 0
        self.vision = None if a.no_vision else Vision(log=self.log)
        self.rooms = []                       # (x, y, label) where the label changed
        self.room = "unknown"
        self.room_votes = []
        self.last_vision_tag = None

    @staticmethod
    def _parse_fault(spec):
        if not spec:
            return None
        kind, _, at = spec.partition("=")
        return (kind, float(at))

    # -- sensor access with fault injection ------------------------------------
    def _check_fault(self):
        if self.fault and not self.frozen and self.log.t() >= self.fault[1]:
            kind = self.fault[0]
            self.frozen = True
            self.log("fault", kind=kind)
            if kind == "estop-at":
                self.safety.raise_estop("fault")
            elif kind != "stale-at":
                raise SystemExit(f"unknown fault {kind}")

    def _feed_frozen(self):
        return self.frozen and self.fault and self.fault[0] == "stale-at"

    def _on_reading(self, angle, cm):
        """Every live sonar reading proves the feed is alive; readings near
        straight ahead also refresh the forward-veto distance."""
        self._check_fault()
        if self._feed_frozen():
            return
        self.safety.mark_alive()
        self.safety.update_distance(cm, ahead=abs(angle - 90) <= 10)

    def read_ahead(self):
        """Distance straight ahead (refreshes the safety layer through _on_reading)."""
        self._check_fault()
        if self.sonar.angle != 90:
            self.sonar.servo(90)
        return self.sonar.range_cm()

    def sweep(self):
        self._check_fault()
        return self.sonar.sweep(step=self.a.step)

    def watchdog_loop(self):
        """Background liveness probe. Every 0.3 s it asks the UNO for its
        off-the-ground flag: a reply proves the link is alive, and a lifted
        car stops at once. If no reply of any kind arrives for STALE_S the car
        is stopped and the run aborts. Under the stale fault, replies are
        ignored, which is how G5 exercises this path."""
        while not self.done:
            time.sleep(0.3)
            self._check_fault()
            if self.safety.estop or self.abort:
                continue
            try:
                line, r = self.car.send({"N": 23}, wait=0.6)
            except OSError:
                r = None
            if r and not self._feed_frozen():
                self.safety.mark_alive()
                if "_true" in r and self.cycles >= 1:
                    self.abort = "lifted"
                    self.safety.stop("lifted")
                    self.log("abort", reason="car lifted off the ground")
                    continue
            if self.cycles >= 1 and self.safety.stale() and self.abort is None:
                self.abort = "stale"
                self.safety.stop("stale")
                self.log("abort", reason="stale sensors")

    # -- motion primitives ------------------------------------------------------
    def _pulses(self, total_ms):
        total_ms = int(total_ms)
        while total_ms >= 60:                 # anything shorter does not move the car
            ms = min(total_ms, MAX_PULSE_MS)
            yield ms
            total_ms -= ms

    def turn(self, deg):
        """Spin in place by deg (positive = left). Returns degrees applied."""
        if abs(deg) < 8:
            return 0.0
        direction = DIR_LEFT if deg > 0 else DIR_RIGHT
        total_ms = abs(deg) / self.turn_dps * 1000.0
        applied = 0.0
        for ms in self._pulses(total_ms):
            self.read_ahead()
            if self.abort or not self.safety.pulse(direction, self.a.turn_speed, ms):
                break
            time.sleep(ms / 1000.0 + 0.35)
            applied += ms / 1000.0 * self.turn_dps
        return math.copysign(applied, deg)

    def floor_ok(self):
        """Cliff and lift guard from the three floor sensors and the firmware's
        off-the-ground flag. A step down shows up as a sensor reading far from
        its running baseline before the wheels reach the edge."""
        self._check_fault()
        line, r = self.car.send({"N": 23}, wait=0.8)
        if r and "_true" in r:                  # firmware prints _true when all three floor sensors see nothing
            self.log("cliff", why="off_ground"); return False
        vals = []
        for i in range(3):
            line, r = self.car.send({"N": 22, "D1": i}, wait=0.8)
            v = _num(r); vals.append(v)
        if all(v is not None for v in vals):
            if self.floor_base is None:
                self.floor_base = vals
            else:
                for i, (v, b) in enumerate(zip(vals, self.floor_base)):
                    if abs(v - b) > self.a.cliff_delta and (v > 900 or v < 25):
                        self.log("cliff", why="sensor_jump", sensor="LMR"[i], value=v, base=b); return False
                self.floor_base = [round(0.8 * b + 0.2 * v) for v, b in zip(vals, self.floor_base)]
        return True

    def drive(self, cm):
        """Drive forward cm in hops, checking the distance ahead and the floor
        before each. Returns cm applied."""
        total_ms = cm / self.fwd_cps * 1000.0
        applied = 0.0
        for ms in self._pulses(total_ms):
            d = self.read_ahead()
            if d is not None and d < MIN_FORWARD_CM + 8:
                self.log("blocked", dist=d); break
            if self.abort:
                break
            if not self.floor_ok():
                self.safety.stop("cliff")
                self.safety.pulse(DIR_BACK, self.a.speed, 350); time.sleep(0.7)
                applied -= 0.35 * self.fwd_cps
                self.mark_ahead_blocked(); break
            if not self.safety.pulse(DIR_FWD, self.a.speed, ms):
                break
            time.sleep(ms / 1000.0 + 0.35)
            applied += ms / 1000.0 * self.fwd_cps
        return applied

    def consume_vision(self):
        """Fold the latest vision answer into the room label and the map."""
        if not (self.vision and self.vision.latest):
            return
        tag, out, ts = self.vision.latest
        if tag == self.last_vision_tag:
            return
        self.last_vision_tag = tag
        label = out.get("room", "unknown")
        if label != "unknown":
            self.room_votes = (self.room_votes + [label])[-3:]
        if len(self.room_votes) == 3 and self.room_votes.count(self.room_votes[-1]) >= 2:
            label = self.room_votes[-1]
            if label != self.room:
                self.room = label
                self.rooms.append((round(self.pose[0]), round(self.pose[1]), label))
                self.log("room", label=label, at=[round(self.pose[0]), round(self.pose[1])])

    def mark_ahead_blocked(self):
        """Paint a wall of occupied cells just ahead so the planner avoids it."""
        x, y, th = self.pose
        h = math.radians(th)
        for lateral in range(-20, 21, 5):
            px = x + 22 * math.cos(h) - lateral * math.sin(h)
            py = y + 22 * math.sin(h) + lateral * math.cos(h)
            r, c = self.grid.to_cell(px, py)
            if self.grid.in_bounds(r, c):
                self.grid.logodds[r, c] = mapping.L_MAX

    def predict(self, dth, dist):
        x, y, th = self.pose
        th = wrap(th + dth)
        return (x + dist * math.cos(math.radians(th)), y + dist * math.sin(math.radians(th)), th)

    # -- one cycle --------------------------------------------------------------
    def cycle(self):
        self.cycles += 1
        scan = self.sweep()
        self.log("scan", n=len(scan), echoes=sum(1 for _, r in scan if r and r < MAX_RANGE))
        if self.cycles > 1:
            est, s, ps = self.matcher.match(self.pose, scan)
            corr = (est[0] - self.pose[0], est[1] - self.pose[1], wrap(est[2] - self.pose[2]))
            self.corrections.append(corr)
            self.log("localize", dx=round(corr[0], 1), dy=round(corr[1], 1), dth=round(corr[2], 1), score=round(s, 2), prior=round(ps, 2))
            self.pose = est
        self.grid.integrate_scan(self.pose, scan)
        self.trail.append(self.pose)
        wp, path, front = next_waypoint(self.grid, self.pose, lookahead_cm=self.a.hop)
        self.camera.save(f"cycle{self.cycles:03d}")
        self.consume_vision()
        if self.vision and self.vision.enabled and self.camera.latest:
            self.vision.describe_async(self.camera.latest, f"cycle{self.cycles:03d}")
        self.log.pose(self.pose, cycle=self.cycles, frontier_cells=int(front.sum()), room=self.room)
        self.save_map(path, front)
        if wp is None:
            self.no_frontier += 1
            if self.no_frontier <= 3:
                self.log("no_frontier", action="turn 90 and look again", attempt=self.no_frontier)
                applied = self.turn(90.0)
                self.pose = self.predict(applied, 0)
                return True
            self.log("complete", reason="no reachable frontier after looking around", cycles=self.cycles)
            return False
        self.no_frontier = 0
        if self.last_target and math.hypot(wp[0] - self.last_target[0], wp[1] - self.last_target[1]) < 10:
            self.stuck += 1
        else:
            self.stuck = 0
        self.last_target = wp
        if self.stuck >= 3:
            self.log("stuck", target=[round(v) for v in wp])
            self.stuck = 0
            self.safety.pulse(DIR_BACK, self.a.speed, 400); time.sleep(0.8)
            self.pose = self.predict(0, -0.4 * self.fwd_cps)
            applied = self.turn(90.0 if self.cycles % 2 else -90.0)
            self.pose = self.predict(applied, 0)
            return True
        hazards = Vision.hazard_ahead(self.vision.latest[1]) if (self.vision and self.vision.latest) else []
        if hazards and self.vision.latest[0] == f"cycle{self.cycles - 1:03d}":
            paint = "stairs_down" in hazards
            self.log("hazard", types=hazards, action="turn away" + (", mark ahead blocked" if paint else ""))
            if paint:
                self.mark_ahead_blocked()
            applied = self.turn(90.0 if self.cycles % 2 else -90.0)
            self.pose = self.predict(applied, 0)
            return True
        want = math.degrees(math.atan2(wp[1] - self.pose[1], wp[0] - self.pose[0]))
        dth = wrap(want - self.pose[2])
        dist = min(math.hypot(wp[0] - self.pose[0], wp[1] - self.pose[1]), self.a.hop)
        self.log("plan", target=[round(v) for v in wp], turn=round(dth), hop=round(dist))
        applied = self.turn(dth)
        self.pose = self.predict(applied, 0)
        moved = self.drive(dist)
        self.pose = self.predict(0, moved)
        self.log("moved", turn=round(applied), cm=round(moved))
        return not self.safety.estop

    def save_map(self, path=None, front=None):
        import numpy as np
        im = render(self.grid, self.trail, path=path, front=front, scale=3)
        if self.rooms:
            from PIL import ImageDraw
            d = ImageDraw.Draw(im)
            for x, y, label in self.rooms:
                r, c = self.grid.to_cell(x, y)
                d.text((c * 3 + 8, (self.grid.n - 1 - r) * 3 - 6), label, fill=(20, 120, 40))
        im.save(os.path.join(self.a.run_dir, "map.png"))
        np.save(os.path.join(self.a.run_dir, "map.npy"), self.grid.logodds)
        st = {"cycles": self.cycles, "pose": [round(v, 1) for v in self.pose], "t": self.log.t(),
              "occupied_cells": int(self.grid.occupied().sum()), "free_cells": int(self.grid.free().sum()),
              "frames": self.camera.count, "estop": self.safety.estop, "room": self.room,
              "rooms": [list(r) for r in self.rooms],
              "vision": {"enabled": bool(self.vision and self.vision.enabled), "calls": self.vision.calls if self.vision else 0,
                         "failures": self.vision.failures if self.vision else 0}}
        with open(os.path.join(self.a.run_dir, "status.json"), "w") as f:
            json.dump(st, f)
        return st

    # -- run ----------------------------------------------------------------------
    def run(self):
        self.log("start", host=self.a.host, dry_run=self.a.dry_run, fault=self.a.fault, step=self.a.step,
                 fwd_cps=self.fwd_cps, turn_dps=self.turn_dps, duration=self.a.duration)
        keyboard_estop(self.safety)
        threading.Thread(target=self.watchdog_loop, daemon=True).start()
        reason = "duration"
        try:
            self.read_ahead()
            while self.log.t() < self.a.duration:
                self._check_fault()
                if self.safety.estop:
                    reason = "estop"; break
                if self.abort:
                    reason = self.abort; break
                if not self.cycle():
                    reason = "estop" if self.safety.estop else ("complete" if not self.abort else self.abort); break
                if self.abort:
                    reason = self.abort; break
        except KeyboardInterrupt:
            reason = "interrupt"
        finally:
            self.done = True
            self.safety.stop("end")
            try:
                self.sonar.center()
            except Exception:
                pass
            self.camera.stop.set()
            st = self.save_map()
            summary = {**st, "reason": reason, "motion_frames": self.safety.motion_frames, "stops": self.safety.stops,
                       "corrections": [[round(c, 1) for c in x] for x in self.corrections],
                       "max_correction_cm": round(max([math.hypot(c[0], c[1]) for c in self.corrections] or [0]), 1),
                       "max_correction_deg": round(max([abs(c[2]) for c in self.corrections] or [0]), 1)}
            with open(os.path.join(self.a.run_dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=1)
            self.log("end", **{k: v for k, v in summary.items() if k != "corrections"})
            self.car.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local")
    ap.add_argument("--duration", type=float, default=240, help="seconds")
    ap.add_argument("--run-dir", default=None)
    ap.add_argument("--dry-run", action="store_true", help="never send a motion frame")
    ap.add_argument("--fault", default=None, help="stale-at=<s> freezes the distance feed; estop-at=<s> raises e-stop")
    ap.add_argument("--step", type=int, default=10, help="sweep step, degrees")
    ap.add_argument("--hop", type=float, default=35.0, help="max drive per cycle, cm")
    ap.add_argument("--speed", type=int, default=120)
    ap.add_argument("--turn-speed", type=int, default=140)
    ap.add_argument("--size", type=int, default=2000, help="map side, cm (2000 covers a house floor)")
    ap.add_argument("--cliff-delta", type=int, default=300, help="floor-sensor jump that counts as a drop")
    ap.add_argument("--cell", type=int, default=5, help="cell size, cm")
    ap.add_argument("--viewer-port", type=int, default=8766)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--no-vision", action="store_true", help="skip the OpenAI scene descriptions")
    a = ap.parse_args()
    if a.run_dir is None:
        a.run_dir = os.path.join("build", "runs", time.strftime("%Y%m%d-%H%M%S"))
    if os.path.isdir(a.run_dir):
        for name in ("log.jsonl", "poses.jsonl", "summary.json"):
            try: os.remove(os.path.join(a.run_dir, name))
            except OSError: pass
    os.makedirs(a.run_dir, exist_ok=True)
    Explorer(a).run()


if __name__ == "__main__":
    main()
