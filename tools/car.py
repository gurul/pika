#!/usr/bin/env python3
"""Drive the ELEGOO Smart Robot Car V4 over Wi-Fi.

Talks to the camera module's TCP command port (100) and relays the stock
JSON frames to the UNO. Answers the module's {Heartbeat} so the link stays up.

Usage:
  car.py [--host elegoo-car.local] CMD [CMD ...]     run commands, then exit
  car.py [--host elegoo-car.local]                   interactive prompt

Commands (shortcuts or raw JSON):
  fwd|back|left|right [speed=120] [ms=800]   timed move
  stop                                       standby
  dist                                       ultrasonic distance, cm
  line                                       line sensors L M R
  servo <angle>                              gimbal 0..180
  mode track|avoid|follow|off                autonomous modes
  {"N":21,"D1":2}                            any raw frame
"""
import argparse, json, os, socket, sys, threading, time

CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")
DIR = {"left": 1, "right": 2, "fwd": 3, "back": 4}
MODE = {"track": 1, "avoid": 2, "follow": 3}


def parse_telemetry(frame, now=None):
    """Legacy seven-field or v4 ten-field telemetry; receipt uses monotonic time.

    V4 adds signed commanded pan (negative while settling), sequence and MCU
    sample milliseconds. A zero distance means unknown, not clear space.
    """
    if not frame.startswith("{T_") or not frame.endswith("}"):
        return None
    parts = frame[3:-1].split("_")
    if len(parts) not in (6, 7, 10):
        return None
    try:
        values = list(map(int, parts))
    except ValueError:
        return None
    d, yaw10, l, m, r, ground = values[:6]
    if not (0 <= d <= 400 and ground in (0, 1) and all(0 <= v <= 1023 for v in (l, m, r))):
        return None
    sample = {"t": time.time(), "mono": time.monotonic() if now is None else now,
              "dist": d, "yaw": yaw10 / 10.0, "line": (l, m, r), "ground": bool(ground),
              "mv": values[6] if len(values) >= 7 else None}
    if len(values) == 10:
        pan, seq, tick = values[7:]
        if not (10 <= abs(pan) <= 170 and 0 <= seq <= 65535 and 0 <= tick <= 65535):
            return None
        sample.update(pan=abs(pan), settled=pan > 0, seq=seq, sample_ms=tick)
    return sample


def load_pan_center():
    """The raw servo angle that points the sonar straight ahead."""
    try:
        return int(json.load(open(CAL)).get("servo_center_deg", 90))
    except (OSError, ValueError, TypeError):
        return 90


class Car:
    """Pan angles are logical: 90 is straight ahead. send() adds the saved
    center offset to outgoing N=5/N=28 angles, and telemetry reports the
    logical angle back. A frame with "raw": true skips the offset."""

    def __init__(self, host, port=100, timeout=5.0, pan_center=None):
        self.pan_center = load_pan_center() if pan_center is None else pan_center
        self._pan_sent = {}            # raw angle -> logical angle it was sent for
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(0.2)
        self.seq = 0
        self.replies = []
        self.telemetry = None          # latest {T_...} frame from the modified UNO firmware, parsed
        self.telemetry_count = 0
        self.lock = threading.Condition()
        self.send_lock = threading.Lock()     # one command in flight at a time, across threads
        self.write_lock = threading.Lock()    # don't interleave heartbeats with JSON frames
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = ""
        while self.alive:
            try:
                chunk = self.sock.recv(1024).decode("utf-8", "replace")
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while "}" in buf:
                frame, buf = buf.split("}", 1)
                frame += "}"
                frame = frame[frame.rfind("{"):] if "{" in frame else frame
                if frame == "{Heartbeat}":
                    try:
                        with self.write_lock:
                            self.sock.sendall(b"{Heartbeat}")
                    except OSError:
                        pass
                    continue
                if frame.startswith("{T_"):
                    sample = parse_telemetry(frame)
                    if sample is not None and "pan" in sample:
                        raw = sample["pan"]
                        sample["pan"] = self._pan_sent.get(raw, raw - (self.pan_center - 90))
                    if sample is not None:
                        self.telemetry = sample
                        self.telemetry_count += 1
                    continue
                with self.lock:
                    self.replies.append(frame)
                    self.replies = self.replies[-64:]
                    self.lock.notify_all()
        self.alive = False

    def set_pan_center(self, center):
        self.pan_center = int(center)
        self._pan_sent.clear()

    def _pan(self, obj):
        key = {5: "D2", 28: "D1"}.get(obj.get("N"))
        if obj.get("raw") or key is None or key not in obj:
            return {k: v for k, v in obj.items() if k != "raw"}
        logical = int(obj[key])
        raw = max(10, min(170, logical + self.pan_center - 90))
        self._pan_sent[raw] = logical
        return {**obj, key: raw}

    def send(self, obj, wait=1.0):
        obj = self._pan(obj)
        with self.send_lock:
            self.seq += 1
            obj = {"H": str(self.seq), **obj}
            line = json.dumps(obj, separators=(",", ":"))
            with self.write_lock:
                self.sock.sendall(line.encode())
            tag = "{" + str(self.seq) + "_"
            deadline = time.monotonic() + wait
            with self.lock:
                while time.monotonic() < deadline:
                    for r in self.replies:
                        if r.startswith(tag) or (r == "{ok}" and obj.get("N") == 100):
                            self.replies.remove(r)
                            return line, r
                    self.lock.wait(max(0, deadline - time.monotonic()))
            return line, None

    def close(self):
        self.alive = False
        self.sock.close()


def parse(cmd):
    """Turn a shortcut or raw JSON string into a frame dict."""
    if cmd.startswith("{"):
        return json.loads(cmd)
    parts = cmd.split()
    op, args = parts[0], parts[1:]
    if op in DIR:
        speed = int(args[0]) if args else 120
        ms = int(args[1]) if len(args) > 1 else 800
        return {"N": 2, "D1": DIR[op], "D2": speed, "T": ms}
    if op == "stop":
        return {"N": 100}
    if op == "dist":
        return {"N": 21, "D1": 2}
    if op == "servo":
        return {"N": 5, "D1": 1, "D2": int(args[0])}
    if op == "mode":
        return {"N": 100} if args[0] == "off" else {"N": 101, "D1": MODE[args[0]]}
    raise ValueError(f"unknown command: {cmd}")


def run(car, cmd):
    if cmd.strip() == "line":
        for i, name in enumerate("LMR"):
            line, r = car.send({"N": 22, "D1": i})
            print(f"{name}: {r}")
        return
    line, r = car.send(parse(cmd))
    print(f"{line} -> {r if r else 'no reply'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local")
    ap.add_argument("--port", type=int, default=100)
    ap.add_argument("cmds", nargs="*")
    a = ap.parse_args()
    car = Car(a.host, a.port)
    try:
        if a.cmds:
            for c in a.cmds:
                run(car, c)
            return
        print(f"connected to {a.host}:{a.port}. Ctrl-D to quit.")
        while True:
            try:
                c = input("car> ").strip()
            except EOFError:
                break
            if c:
                try:
                    run(car, c)
                except (ValueError, KeyError, IndexError) as e:
                    print(e)
    finally:
        try:
            car.send({"N": 100}, wait=0.5)
        except OSError:
            pass
        car.close()


if __name__ == "__main__":
    main()
