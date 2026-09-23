#!/usr/bin/env python3
"""Read-only radar display for a controller's angle-tagged JSONL telemetry.

Example: python tools/sonar_feed.py --run-dir build/runs/vfh-... --port 8767
The controller remains the only client connected to the car. This process only
tails its log and serves the display on the computer's loopback interface.
"""
import argparse
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


MAX_READ = 1024 * 1024
MAX_LINE = 65536
HISTORY_S = 2.0
STALE_S = 0.75
HTML_PATH = Path(__file__).resolve().parents[1] / "web" / "sonar-feed.html"


def finite(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def wrap(angle):
    return (angle + 180) % 360 - 180


def project(angle, distance):
    """Centimeters in radar coordinates: +x right, +y forward, 90° forward."""
    theta = math.radians(angle)
    return {"x_cm": distance * math.cos(theta), "y_cm": distance * math.sin(theta)}


@dataclass(frozen=True)
class Sample:
    stamp: float
    pan: float
    distance: float
    yaw: float
    seq: int
    settled: bool
    yaw_left_sign: int = -1

    @property
    def valid_return(self):
        return self.settled and 0 < self.distance < 400


def parse_sample(row):
    """Only controller-accepted v4 ticks may provide displayed observations."""
    if not isinstance(row, dict) or row.get("event") != "tick" or row.get("accepted") is not True:
        return None
    t = row.get("telemetry")
    if not isinstance(t, dict):
        return None
    for field in ("t", "pan", "dist", "yaw"):
        if not finite(t.get(field)):
            return None
    if not (t["t"] > 0 and 10 <= t["pan"] <= 170 and 0 <= t["dist"] <= 400):
        return None
    if type(t.get("seq")) is not int or not 0 <= t["seq"] <= 65535:
        return None
    if type(t.get("settled")) is not bool:
        return None
    sign = row.get("yaw_left_sign", -1)
    if type(sign) is not int or sign not in (-1, 1):
        return None
    return Sample(t["t"], t["pan"], t["dist"], t["yaw"], t["seq"], t["settled"], sign)


class RadarFeed:
    def __init__(self, run_dir, clock=time.time):
        self.run_dir = Path(run_dir).resolve()
        self.path = self.run_dir / "log.jsonl"
        self.clock = clock
        self.lock = threading.Lock()
        self.identity = None
        self.offset = 0
        self.pending = b""
        self.discard_line = False
        self.available = False
        self.error = None
        self._reset()

    def _reset(self):
        self.latest = None
        self.returns = deque(maxlen=240)
        self.arrivals = deque(maxlen=100)
        self.decision = {}
        self.observe = None
        self.ended = False
        self.end_reason = None
        self.samples = 0
        self.rejected = 0
        self.malformed = 0

    def ingest(self, row):
        if not isinstance(row, dict):
            self.malformed += 1
            return
        if row.get("event") == "end":
            self.ended = True
            self.end_reason = str(row.get("error") or "Run completed")
            return
        if row.get("event") == "abort":
            self.end_reason = str(row.get("reason", "Run interrupted"))
            return
        sample = parse_sample(row)
        if sample is None:
            if row.get("event") == "tick":
                self.rejected += 1
            return
        if self.latest is not None:
            if sample.seq == self.latest.seq or sample.stamp <= self.latest.stamp:
                return
        self.latest = sample
        self.samples += 1
        self.arrivals.append(sample.stamp)
        self.ended = False
        self.observe = row.get("observe") if type(row.get("observe")) is bool else None
        self.decision = {key: row[key] if finite(row.get(key)) else None
                         for key in ("left", "right", "look", "spin")}
        self.decision["reason"] = str(row.get("reason", ""))[:120]
        # Gyro sign follows the controller's left-positive heading convention.
        # A new settled ray supersedes its recent angular neighborhood. A no-echo
        # sample revokes the old return there instead of drawing it as free space.
        if sample.settled:
            bearing = sample.pan + sample.yaw_left_sign * sample.yaw
            self.returns = deque((r for r in self.returns
                                  if abs(wrap((r.pan + r.yaw_left_sign * r.yaw) - bearing)) > 5), maxlen=240)
            if sample.valid_return:
                self.returns.append(sample)

    def _read(self):
        try:
            with self.path.open("rb") as stream:
                stat = os.fstat(stream.fileno())
                identity = (stat.st_dev, stat.st_ino)
                if identity != self.identity or stat.st_size < self.offset:
                    self.identity, self.offset, self.pending = identity, 0, b""
                    self.discard_line = False
                    self._reset()
                if stat.st_size - self.offset > MAX_READ:
                    self.offset = stat.st_size - MAX_READ
                    self.pending = b""
                    self.discard_line = True
                    self._reset()
                stream.seek(self.offset)
                chunk = stream.read(MAX_READ)
                self.offset = stream.tell()
        except FileNotFoundError:
            self.available, self.error = False, None
            return
        except OSError as exc:
            self.available, self.error = False, str(exc)
            return
        self.available, self.error = True, None
        lines = (self.pending + chunk).split(b"\n")
        self.pending = lines.pop()
        for line in lines:
            if self.discard_line:
                self.discard_line = False
                continue
            if not line.strip():
                continue
            if len(line) > MAX_LINE:
                self.malformed += 1
                continue
            try:
                self.ingest(json.loads(line))
            except (ValueError, UnicodeDecodeError):
                self.malformed += 1
        if len(self.pending) > MAX_LINE:
            self.pending = b""
            self.discard_line = True
            self.malformed += 1

    def snapshot(self):
        with self.lock:
            self._read()
            now, latest = self.clock(), self.latest
            age = None if latest is None else now - latest.stamp
            status = "waiting"
            if not self.available:
                status = "missing"
            elif self.ended:
                status = "ended"
            elif latest is not None:
                if age < -2:
                    status = "clock_error"
                elif age > STALE_S:
                    status = "stale"
                elif not latest.settled:
                    status = "settling"
                elif not latest.valid_return:
                    status = "unknown"
                else:
                    status = "live"
            points = []
            if latest is not None and self.available:
                for sample in self.returns:
                    sample_age = now - sample.stamp
                    if not 0 <= sample_age <= HISTORY_S:
                        continue
                    angle = sample.pan + wrap(sample.yaw_left_sign * sample.yaw - latest.yaw_left_sign * latest.yaw)
                    if not 0 <= angle <= 180:
                        continue
                    points.append({"angle": angle, "distance_cm": sample.distance,
                                   "age_s": round(sample_age, 3), "seq": sample.seq,
                                   **project(angle, sample.distance)})
            recent = [stamp for stamp in self.arrivals if 0 <= now - stamp <= 3]
            hz = ((len(recent) - 1) / (recent[-1] - recent[0])
                  if len(recent) > 1 and recent[-1] > recent[0] else 0)
            return {
                "status": status, "run": self.run_dir.name, "server_time": now,
                "age_s": None if age is None else round(max(0, age), 3),
                "history_s": HISTORY_S, "stale_s": STALE_S,
                "sample_hz": round(hz, 1), "sample_count": self.samples,
                "latest": None if latest is None else {
                    "pan": latest.pan, "distance_cm": latest.distance,
                    "settled": latest.settled, "valid_return": latest.valid_return,
                    "yaw": latest.yaw, "seq": latest.seq,
                },
                "points": points, "decision": self.decision, "observe": self.observe,
                "end_reason": self.end_reason, "error": self.error,
                "rejected_count": self.rejected, "malformed_count": self.malformed,
            }


def make_server(feed, port=8767):
    html = HTML_PATH.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            host = self.headers.get("Host", "").split(":", 1)[0]
            if host not in ("127.0.0.1", "localhost"):
                self.send_error(403, "Local display only")
                return
            route = urlsplit(self.path).path
            if route == "/api/state":
                body = json.dumps(feed.snapshot(), allow_nan=False).encode()
                content_type = "application/json; charset=utf-8"
            elif route in ("/", "/sonar-feed.html"):
                body, content_type = html, "text/html; charset=utf-8"
            elif route == "/health":
                body, content_type = b'{"ok":true}', "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="Controller run directory containing log.jsonl; may be created later")
    parser.add_argument("--port", default=8767, type=int)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    server = make_server(RadarFeed(args.run_dir), args.port)
    print(f"Live sonar: http://127.0.0.1:{args.port}/  |  Reading {args.run_dir / 'log.jsonl'}", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
