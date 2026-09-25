#!/usr/bin/env python3
"""Person follow checks.

  firmware    FollowDrive.h scenarios, host-compiled; UNO sketch fits (check_uno_build.py)
  tracker     tools/tests/test_follow.py (geometry, choice, lost policy, loop, Apple Vision)
  controller  controller.py end to end against a fake car (TCP) and a fake camera (MJPEG)
"""
import json, os, pathlib, socket, subprocess, sys, tempfile, threading, time, unittest, urllib.error, urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = str(ROOT / ".venv/bin/python") if (ROOT / ".venv/bin/python").exists() else sys.executable

HOST_TEST = r"""
#include <cstdio>
#include <cstdlib>
#include "FollowDrive.h"
static int fails = 0;
#define CHECK(c, what) do { if (!(c)) { std::printf("FAIL %s (line %d)\n", what, __LINE__); fails++; } } while (0)
typedef FollowDrive F;
// yaw10 as the car sees it: a left turn decreases it (kYawLeft = -1)
static void run(F &f, int16_t yaw10, uint32_t now, int &l, int &r, bool ground = true) { f.wheels(ground, yaw10, now, l, r); }
static void pings(F &f, uint16_t a, uint16_t b) { f.sonar(a); f.sonar(b); }
int main() {
  int l, r;
  const uint16_t FAR = F::kGapCm + F::kBand + 30, GAP = F::kGapCm;
  { F f; run(f, 0, 100, l, r); CHECK(l == 0 && r == 0, "no bearing yet: still"); }
  { F f; f.hint(0, 1, 0, FAR, 0, 1000);
    run(f, 0, 1100, l, r); CHECK(l == r && l >= F::kVmin && l <= F::kVmax, "camera says far, dead ahead, sonar silent: straight on");
    f.hint(0, 1, 0, GAP, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == 0 && r == 0, "camera range in the gap band: hold");
    f.hint(0, 1, 0, F::kTooCloseCm - 5, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == -F::kVmin && r == -F::kVmin, "camera says too close, ahead: back off straight");
    f.hint(0, 1, 0, 0, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == 0 && r == 0, "no camera range: no forward");
    f.hint(0, 1, 0, FAR, 0, 1000);
    run(f, 0, 1100 + F::kHintMs, l, r); CHECK(l == 0 && r == 0, "stale bearing: stop");
    run(f, 0, 1100, l, r, false); CHECK(l == 0 && r == 0, "lifted off the floor: stop"); }
  { F f; int far, nearer; f.hint(0, 1, 0, FAR + 20, 0, 1000); run(f, 0, 1100, far, r);
    f.hint(0, 1, 0, F::kGapCm + F::kBand + 10, 0, 1000); run(f, 0, 1100, nearer, r);
    CHECK(far > nearer, "further away drives faster"); }
  // sonar reflex: only for something close, and only when two pings agree
  { F f; f.hint(0, 1, 0, FAR, 0, 1000);
    pings(f, 120, 60); run(f, 0, 1100, l, r); CHECK(l > 0 && r > 0, "sonar sees things further than the brake distance: ignored, the camera drives");
    pings(f, 120, F::kBrakeCm - 4); run(f, 0, 1100, l, r); CHECK(l > 0 && r > 0, "one close ping alone is a stray echo: ignored");
    pings(f, F::kBrakeCm - 4, F::kBrakeCm - 2); run(f, 0, 1100, l, r); CHECK(l == 0 && r == 0, "two close pings ahead: brake");
    pings(f, F::kBackCm - 4, F::kBackCm - 2); run(f, 0, 1100, l, r); CHECK(l == -F::kVmin && r == -F::kVmin, "two very close pings: back off");
    pings(f, 0, F::kBackCm - 2); run(f, 0, 1100, l, r); CHECK(l > 0 && r > 0, "a no-echo ping breaks the agreement"); }
  { F f; f.hint(30, 1, 0, FAR, 0, 1000); pings(f, F::kBrakeCm - 4, F::kBrakeCm - 4); run(f, 0, 1100, l, r);
    CHECK(r == F::kSpin && l == -F::kSpin, "obstacle close, person to the left: no forward, turn to them"); }
  { F f; f.hint(10, 1, 0, FAR, 0, 1000); run(f, 0, 1100, l, r); CHECK(r > l && l > 0, "person 10 deg left: arc left"); }
  { F f; f.hint(-10, 1, 0, FAR, 0, 1000); run(f, 0, 1100, l, r); CHECK(l > r && r > 0, "person 10 deg right: arc right"); }
  { F f; f.hint(20, 1, 0, GAP, 0, 1000); run(f, 0, 1100, l, r); CHECK(r == F::kSpin && l == -F::kSpin, "at the gap, off to the left: turn left in place"); }
  { F f; f.hint(4, 1, 0, GAP, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == 0 && r == 0, "at the gap, nearly ahead: deadband"); }
  { F f; f.hint(30, 1, 0, F::kTooCloseCm - 5, 0, 1000); run(f, 0, 1100, l, r); CHECK(r > 0 && l < 0, "too close but off to the side: no reverse, turn to them"); }
  { F f; f.hint(-30, 2, 0, 0, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == F::kSearch && r == -F::kSearch, "search right"); }
  { F f; f.hint(30, 0, 0, FAR, 0, 1000); run(f, 0, 1100, l, r); CHECK(l == 0 && r == 0, "lost: stop"); }
  { F f; f.hint(90, 1, 0, FAR, 0, 1000); CHECK(f.bearing == 60, "bearing clamps to 60"); }
  { F f; f.hint(60, 1, 0, 400, 0, 1000); run(f, 0, 1100, l, r);
    CHECK(r <= 255 && l >= -255 && r > l, "far and far off-axis: wheel PWM fits a byte (no uint8 wrap)"); }
  // gyro hold: person 20 deg left; after the car has turned 20 deg left the error is gone
  { F f; f.hint(20, 1, 0, GAP, 0, 1000); run(f, -200, 1100, l, r); CHECK(l == 0 && r == 0, "turned onto the aim: stop turning");
    run(f, -300, 1100, l, r); CHECK(l == F::kSpin && r == -F::kSpin, "overshot left by 10: turn back right"); }
  // latency: the frame is 330 ms old and the car has since turned 15 deg left; the person
  // was 20 deg left of where it pointed then, so only 5 deg remain
  { F f;
    for (uint32_t t = 0; t <= 600; t += 60) f.sample(t < 420 ? 0 : -150, t);
    f.hint(20, 1, 330, GAP, -150, 600);
    CHECK(f.aim == -200, "aim uses the heading when the frame was taken");
    run(f, -150, 650, l, r); CHECK(l == 0 && r == 0, "5 deg left is inside the deadband"); }
  { F f; for (uint32_t t = 0; t <= 600; t += 60) f.sample(-10 * int(t / 60), t);
    CHECK(f.yawAgo(0, -77) == -77, "age 0: the heading now");
    CHECK(f.yawAgo(5000, -77) == f.hist[(f.head + 1) & 7], "age past the history: its oldest entry"); }
  { F f; f.hint(0, 1, 0, FAR, 0, 0xFFFFFF00u); run(f, 0, 0x50u, l, r); CHECK(l > 0 && r > 0, "bearing across the millis wrap is fresh"); }
  if (fails) { std::printf("%d follow drive checks failed\n", fails); return 1; }
  std::printf("follow drive host checks passed\n");
  return 0;
}
"""


def firmware():
    with tempfile.TemporaryDirectory() as d:
        src, exe = pathlib.Path(d) / "t.cpp", pathlib.Path(d) / "t"
        src.write_text(HOST_TEST)
        subprocess.run(["c++", "-std=c++11", "-Wall", "-I", str(ROOT / "firmware/uno_v4_mod"), str(src), "-o", str(exe)], check=True)
        host = subprocess.run([str(exe)], capture_output=True, text=True)
    print(host.stdout.strip())
    build = subprocess.run([sys.executable, str(ROOT / "scripts/check_uno_build.py")], capture_output=True, text=True, cwd=ROOT)
    print(build.stdout.strip())
    src = (ROOT / "firmware/uno_v4_mod/ApplicationFunctionSet_xxx0.cpp").read_text()
    wired = all(k in src for k in ("case 29:", "followDrive.hint(", "followDrive.wheels(", "followDrive.sample(", "followDrive.sonar(d)", "followDrive.hinted = false", 'doc["D4"]'))
    print("follow wired into the sketch:", wired)
    return host.returncode == 0 and build.returncode == 0 and "uno build verification passed" in build.stdout and wired


def tracker():
    r = subprocess.run([PY, "-m", "unittest", "-v", "tools.tests.test_follow"], cwd=ROOT, capture_output=True, text=True)
    out = r.stdout + r.stderr
    print("\n".join(out.strip().splitlines()[-4:]))
    vision = "skipped" not in out
    print("Apple Vision tests ran:", vision)
    return r.returncode == 0 and vision


# --- controller end to end ----------------------------------------------------

class FakeCar:
    """Car TCP port: records every JSON frame, answers with {H_ok}."""
    def __init__(self, port):
        self.frames, self.srv = [], socket.create_server(("127.0.0.1", port), reuse_port=True)
        self.slow, self.times = False, []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            c, _ = self.srv.accept()
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, c):
        buf = ""
        while True:
            try:
                data = c.recv(4096)
            except OSError:
                return
            if not data:
                return
            buf += data.decode()
            while "}" in buf:
                frame, buf = buf.split("}", 1)
                try:
                    obj = json.loads(frame + "}")
                except ValueError:
                    continue
                self.frames.append(obj)
                self.times.append(time.monotonic())
                if obj.get("N") == 21:
                    if self.slow:                       # a UNO busy pinging: the poll waits out its 0.6 s
                        continue
                    c.sendall(("{%s_120}" % obj.get("H")).encode())
                elif obj.get("N") != 29:
                    c.sendall(("{%s_ok}" % obj.get("H")).encode())


class FakeCamera:
    """MJPEG on a port, serving the fixture photo about 10 times a second."""
    def __init__(self, port, jpeg):
        self.jpeg, self.srv = jpeg, socket.create_server(("127.0.0.1", port), reuse_port=True)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            c, _ = self.srv.accept()
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, c):
        try:
            c.recv(4096)
            c.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=f\r\n\r\n")
            while True:
                c.sendall(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(self.jpeg) + self.jpeg + b"\r\n")
                time.sleep(0.1)
        except OSError:
            pass


def controller():
    jpeg = (ROOT / "tools/tests/fixtures/people-cc0.jpg").read_bytes()
    try:
        car, cam = FakeCar(18100), FakeCamera(18081, jpeg)
    except OSError as e:
        print(f"cannot bind the fake car's ports on localhost: {e}")
        return False
    port = 18765
    proc = subprocess.Popen([PY, str(ROOT / "tools/controller.py"), "--host", "127.0.0.1", "--port", str(port), "--no-browser",
                             "--car-port", "18100", "--video-port", "18081"],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    base = f"http://127.0.0.1:{port}"

    def get(path):
        with urllib.request.urlopen(base + path, timeout=5) as r:
            return json.loads(r.read())

    def post(path, body):
        req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())

    ok = True
    def check(cond, what):
        nonlocal ok
        print(("ok   " if cond else "FAIL ") + what)
        ok = ok and cond

    try:
        for _ in range(50):
            try:
                get("/follow"); break
            except OSError:
                time.sleep(0.2)
        st = get("/follow")
        check(st.get("unavailable") is None, "controller has Apple Vision")
        with urllib.request.urlopen(base + "/stream", timeout=5) as r:
            head = r.read(200)
            check(r.headers.get_content_type() == "multipart/x-mixed-replace" and b"image/jpeg" in head, "/stream relays the camera")
        # the right-hand near person: upper-body box about x 0.53..0.80, y 0.16..0.52
        post("/follow/select", {"x": 0.66, "y": 0.5, "start": True})
        stop_at = time.time() + 1.6
        while time.time() < stop_at:
            get("/follow"); time.sleep(0.25)          # the page keeps checking in
        st = get("/follow")
        check(st["on"] and st["locked"] and st["seen"], "select locks on and follows")
        modes = [f for f in car.frames if f.get("N") == 101]
        check(bool(modes) and modes[0].get("D1") == 3 and modes[0].get("D2") == 95, "follow mode sent with the saved sonar front (95 raw)")
        hints = [f for f in car.frames if f.get("N") == 29]
        check(len(hints) >= 5, f"bearings stream to the car ({len(hints)} in 1.6 s)")
        # person centre x = 0.665: (0.5 - 0.666) * 62 = -10.3, right of centre
        check(all(h["D2"] == 1 and -13 <= h["D1"] <= -8 for h in hints), f"bearings say seen, about 10 deg right: {sorted(set(h['D1'] for h in hints))}")
        check(all(isinstance(h.get("D3"), int) and 0 <= h["D3"] <= 2000 for h in hints), "bearings carry the frame age in ms")
        # the page polls distance every 350 ms, each poll waiting up to 0.6 s for its reply;
        # bearings must keep flowing (the UNO drops one older than 0.5 s)
        car.slow, polling = True, True
        def poll():
            while polling:
                try:
                    post("/cmd", {"frame": {"N": 21, "D1": 2}, "wait": 0.6})
                except OSError:
                    pass
                time.sleep(0.35)
        threading.Thread(target=poll, daemon=True).start()
        n = len(car.frames)
        stop_at = time.time() + 3.0
        while time.time() < stop_at:
            get("/follow"); time.sleep(0.25)
        polling, car.slow = False, False
        ts = [t for f, t in zip(car.frames[n:], car.times[n:]) if f.get("N") == 29]
        gap = max((b - a for a, b in zip(ts, ts[1:])), default=9.0)
        check(len(ts) >= 15 and gap < 0.35, f"bearings keep flowing while distance polls wait ({len(ts)} in 3 s, longest gap {gap:.2f} s)")
        time.sleep(0.7)
        n = len(car.frames)
        try:
            post("/cmd", {"frame": {"N": 22, "D1": 0}, "wait": 0.3})
            refused = False
        except urllib.error.HTTPError as e:
            refused = e.code == 409
        time.sleep(0.2)
        check(refused and not any(f.get("N") == 22 for f in car.frames[n:]) and get("/follow")["on"],
              "line-sensor query refused while following (it would drop the UNO out of follow mode)")
        n = len(car.frames)
        post("/cmd", {"frame": {"N": 100}, "wait": 0.3})
        time.sleep(0.6)
        after = [f for f in car.frames[n:] if f.get("N") == 29]
        check(not get("/follow")["on"] and not after, "stop hands the car back: no more bearings")
        # page deadman: start again, then stop checking in
        n = len(car.frames)
        post("/follow/start", {})
        time.sleep(3.0)
        after = car.frames[n:]
        stops = [i for i, f in enumerate(after) if f.get("N") == 100]
        check(not get("/follow")["on"] and bool(stops) and not any(f.get("N") == 29 for f in after[stops[0]:]),
              "follow stops itself when the page goes quiet (stop sent, no bearings after it)")
    except OSError as e:
        check(False, f"controller unreachable: {e}")
    finally:
        proc.terminate()
        try:
            out = proc.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            proc.kill(); out = proc.communicate()[0]
        if not ok:
            print(out[-2000:])
    return ok


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    steps = {"firmware": firmware, "tracker": tracker, "controller": controller}
    run = steps if mode == "all" else {mode: steps[mode]}
    ok = all([fn() for fn in run.values()])
    print(f"follow {mode} verification passed" if ok else f"follow {mode} verification FAILED")
    sys.exit(0 if ok else 1)
