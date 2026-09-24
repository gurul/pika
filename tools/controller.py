#!/usr/bin/env python3
"""Browser controller for the car.

Serves controller.html on localhost and proxies its commands to the car's
TCP port 100, so the page gets real replies (distance, line sensors, acks).
Video is embedded straight from the board's MJPEG stream.

  tools/controller.py [--host elegoo-car.local] [--port 8765]
"""
import argparse, json, os, sys, threading, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car, load_pan_center, parse  # noqa: E402

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
            if self.car is None or not self.car.alive:
                self.car = Car(self.host, self.port, timeout=3.0, pan_center=self.pan_center)
                # the UNO boots with the servo at raw 90; point it at the saved front
                self.car.send({"N": 5, "D1": 1, "D2": 90}, wait=0.8)
            try:
                return self.car.send(frame, wait=wait)
            except OSError:
                self.car.close()
                self.car = None
                raise


class Handler(BaseHTTPRequestHandler):
    link = None
    car_host = ""

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
            self._json(200, {"host": self.car_host,
                             "stream": f"http://{self.car_host}:81/stream",
                             "capture": f"http://{self.car_host}/capture"})
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
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
            line, reply = self.link.send(frame, wait)
            self._json(200, {"sent": line, "reply": reply})
        except (ValueError, KeyError) as e:
            self._json(400, {"error": str(e)})
        except OSError as e:
            self._json(502, {"error": f"car unreachable: {e}"})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local", help="car hostname or IP")
    ap.add_argument("--port", type=int, default=8765, help="local port for the page")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    Handler.link = Link(a.host)
    Handler.car_host = a.host
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
        if Handler.link.car:
            try:
                Handler.link.car.send({"N": 100}, wait=0.3)
            except OSError:
                pass
            Handler.link.car.close()


if __name__ == "__main__":
    main()
