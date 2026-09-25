#!/usr/bin/env python3
"""Browser controller for the car.

Serves controller.html on localhost and proxies its commands to the car's
TCP port 100, so the page gets real replies (distance, line sensors, acks).
The board serves one video viewer, so this server is that viewer: it relays
the MJPEG stream to the page at /stream and feeds the same frames to the
person follower (tools/follow.py).

  tools/controller.py [--host elegoo-car.local] [--port 8765]
"""
import argparse, json, os, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car, load_pan_center, parse  # noqa: E402
from follow import Follower, Stream  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "..", "calibration.json")


def load_cal():
    try:
        return json.load(open(CAL))
    except (OSError, ValueError):
        return {}


def save_trim(left, right):
    cal = load_cal(); cal["trim_left"] = round(left, 3); cal["trim_right"] = round(right, 3)
    json.dump(cal, open(CAL, "w"), indent=2)


def save_front(center, dist_cm):
    """Store the raw servo angle that points the sonar straight ahead."""
    import time
    cal = load_cal(); cal["servo_center_deg"] = center
    cal["servo_center_note"] = (f"front: raw servo angle that points the sonar straight ahead, "
                                f"set in the controller {time.strftime('%Y-%m-%d')}; "
                                f"read {dist_cm if dist_cm is not None else 'no echo'} cm when saved")
    json.dump(cal, open(CAL, "w"), indent=2)


class Link:
    """One TCP session to the car, rebuilt on demand, one command at a time.
    Forward drive uses independent wheel speeds with the straight-line trim,
    renewed by the page while the button is held; a deadman thread stops the
    car if no renewal arrives within 0.7 s."""

    def __init__(self, host, port=100):
        self.host, self.port = host, port
        self.car = None
        self.lock = threading.Lock()
        cal = load_cal()
        self.trim = [cal.get("trim_left", 1.0), cal.get("trim_right", 1.0)]
        self.pan_center = load_pan_center()
        self.diff_until = 0.0
        threading.Thread(target=self._deadman, daemon=True).start()

    def set_pan_center(self, center):
        self.pan_center = center
        with self.lock:
            if self.car is not None:
                self.car.set_pan_center(center)

    def _deadman(self):
        import time
        while True:
            time.sleep(0.15)
            if self.diff_until and time.monotonic() > self.diff_until:
                self.diff_until = 0.0
                try:
                    self.send({"N": 100}, 0.3)
                except OSError:
                    pass

    def send(self, frame, wait):
        import time
        # forward pulses become trimmed wheel-speed drive, renewed while held
        if frame.get("N") == 2 and frame.get("D1") == 3:
            v = int(frame.get("D2", 120))
            frame = {"N": 4, "D1": max(0, min(255, round(v * self.trim[0]))), "D2": max(0, min(255, round(v * self.trim[1])))}
            self.diff_until = time.monotonic() + 0.7
        elif frame.get("N") == 100:
            self.diff_until = 0.0
        with self.lock:
            self._connect()
            try:
                return self.car.send(frame, wait=wait)
            except OSError:
                self.car.close()
                self.car = None
                raise

    def _connect(self):
        if self.car is None or not self.car.alive:
            self.car = Car(self.host, self.port, timeout=3.0, pan_center=self.pan_center)
            # the UNO boots with the servo at raw 90; point it at the saved front
            self.car.send({"N": 5, "D1": 1, "D2": 90}, wait=0.8)

    def post(self, frame):
        """A frame with no reply (follow bearings). It must not wait behind the
        page's distance polls, which hold the lock for up to 0.6 s each: the UNO
        drops a bearing older than 0.5 s, so a queued bearing stops the car."""
        car = self.car
        if car is None or not car.alive:
            with self.lock:
                self._connect()
                car = self.car
        try:
            car.post(frame)
        except OSError:
            car.close()
            raise


class Handler(BaseHTTPRequestHandler):
    link = None
    car_host = ""
    stream = None
    follower = None

    def log_message(self, *a):  # keep the terminal quiet
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "controller.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/trim":
            self._json(200, {"left": self.link.trim[0], "right": self.link.trim[1]})
        elif self.path == "/center":
            self._json(200, {"center": self.link.pan_center})
        elif self.path == "/config":
            self._json(200, {"host": self.car_host, "stream": "/stream",
                             "capture": f"http://{self.car_host}/capture"})
        elif self.path.startswith("/stream"):
            self._relay_stream()
        elif self.path == "/follow":
            f = self.follower
            f.touch()
            self._json(200, {**f.state, "unavailable": f.unavailable, "video_error": self.stream.error})
        else:
            self._json(404, {"error": "not found"})

    def _relay_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        count = 0
        try:
            while True:
                count, jpeg = self.stream.wait(count, 5.0)
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg + b"\r\n")
        except OSError:                 # the page went away
            pass

    def _follow(self, req):
        f = self.follower
        if f.unavailable:
            return self._json(503, {"error": f"person follow needs Apple Vision: {f.unavailable}"})
        op = self.path.rsplit("/", 1)[-1]
        if op == "select":
            box = f.select(float(req["x"]), float(req["y"]))
            if box is None:
                return self._json(404, {"error": "no person there"})
            if req.get("start"):
                f.start()
            return self._json(200, f.state)
        if op == "start":
            f.start()
        elif op == "stop":
            f.stop()
        elif op == "forget":
            f.forget()
        else:
            return self._json(404, {"error": "not found"})
        return self._json(200, f.state)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        if self.path.startswith("/follow/"):
            try:
                return self._follow(json.loads(self.rfile.read(n) or b"{}"))
            except (ValueError, KeyError, TypeError) as e:
                return self._json(400, {"error": str(e)})
            except OSError as e:
                return self._json(502, {"error": f"car unreachable: {e}"})
        if self.path == "/trim":
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
                left, right = float(req.get("left", 1.0)), float(req.get("right", 1.0))
                left, right = max(0.5, min(1.5, left)), max(0.5, min(1.5, right))
                self.link.trim = [left, right]; save_trim(left, right)
                return self._json(200, {"left": left, "right": right})
            except (ValueError, KeyError) as e:
                return self._json(400, {"error": str(e)})
        if self.path == "/center":
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
                center = int(req["center"])
                if not 10 <= center <= 170:
                    raise ValueError("center must be 10..170")
                dist = req.get("dist")
                save_front(center, int(dist) if dist is not None else None)
                self.link.set_pan_center(center)
                return self._json(200, {"center": center})
            except (ValueError, KeyError, TypeError) as e:
                return self._json(400, {"error": str(e)})
        if self.path != "/cmd":
            return self._json(404, {"error": "not found"})
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
            frame = req["frame"] if "frame" in req else parse(req["cmd"])
            wait = float(req.get("wait", 0.6))
            if frame.get("N") in MANUAL:    # any drive or mode input takes the car back from the follower
                self.follower.release()
            elif frame.get("N") in LEAVES_MODE and self.follower.on:
                return self._json(409, {"error": "paused while following: this query switches the car out of follow mode"})
            elif frame.get("N") == 21 and self.follower.on:
                # one sonar pinger while following (two hear each other's echoes):
                # answer from the telemetry stream instead of pinging again
                car = self.link.car
                t = car.telemetry if car else None
                if t and time.monotonic() - t["mono"] < 0.5:
                    return self._json(200, {"sent": None, "reply": "{0_%d}" % (t["dist"] or 400)})
            line, reply = self.link.send(frame, wait)
            self._json(200, {"sent": line, "reply": reply})
        except (ValueError, KeyError) as e:
            self._json(400, {"error": str(e)})
        except OSError as e:
            self._json(502, {"error": f"car unreachable: {e}"})


# stop, timed and wheel-speed drive, joystick, servo, and the UNO's own modes
MANUAL = {100, 2, 3, 4, 5, 101, 102}
# Read-only on paper, but ELEGOO's handlers switch the UNO to programming mode:
# N=22 (line sensors) ended every follow within 2.5 s on 2026-09-25.
LEAVES_MODE = {22}


def use_venv():
    """Person follow needs pyobjc's Vision; the system Python has none. Rerun
    under the repo's .venv when that one does."""
    venv = os.path.normpath(os.path.join(HERE, "..", ".venv"))
    py = os.path.join(venv, "bin", "python")
    if os.path.normpath(sys.prefix) == venv or not os.path.exists(py):
        return
    try:
        import Vision  # noqa: F401
    except ImportError:
        os.execv(py, [py, os.path.abspath(sys.argv[0])] + sys.argv[1:])


def main():
    use_venv()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local", help="car hostname or IP")
    ap.add_argument("--port", type=int, default=8765, help="local port for the page")
    ap.add_argument("--car-port", type=int, default=100, help="the car's JSON command port")
    ap.add_argument("--video-port", type=int, default=81, help="the car's MJPEG stream port")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    Handler.link = Link(a.host, a.car_port)
    Handler.car_host = a.host
    Handler.stream = Stream(f"http://{a.host}:{a.video_port}/stream")
    cal = load_cal()
    Handler.follower = Follower(Handler.stream,
                                lambda f: Handler.link.post(f) if f.get("N") == 29 else Handler.link.send(f, 0.4),
                                front_raw=lambda: Handler.link.pan_center,
                                mirror=bool(cal.get("camera_mirror", False)),
                                hfov=float(cal.get("camera_hfov_deg", 62.0)),
                                trace=os.path.join(HERE, "..", "build", "follow.jsonl"),
                                telemetry=lambda: Handler.link.car.telemetry if Handler.link.car else None,
                                yaw_left_sign=int(cal.get("yaw_left_sign", -1)))
    if Handler.follower.unavailable:
        print(f"person follow off: {Handler.follower.unavailable}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://127.0.0.1:{a.port}/"
    print(f"controller at {url}  (car: {a.host})  Ctrl-C to stop", flush=True)
    if not a.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.follower.release()
        if Handler.link.car:
            try:
                Handler.link.car.send({"N": 100}, wait=0.3)
            except OSError:
                pass
            Handler.link.car.close()


if __name__ == "__main__":
    main()
