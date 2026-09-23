#!/usr/bin/env python3
"""Fast autonomous drive on firmware v3: telemetry stream at 12 Hz, gyro
heading hold, clearance-scaled speed, closed-loop turns when blocked.

  tools/dash.py [--duration 120] [--vmax 230]
"""
import argparse, json, os, random, re, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car
from safety import Safety, DIR_BACK
from explore import keyboard_estop

CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")
D_STOP, D_SLOW, D_CLEAR = 45.0, 70.0, 130.0      # cm ahead
D_CURVE = 110.0                                   # start bending away from an obstacle here
SIDE_MIN = 70.0                                   # a side counts as open beyond this


class Dash:
    def __init__(self, a):
        self.a = a
        self.cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
        self.left_sign = self.cal.get("yaw_left_sign", -1)
        self.trim = (self.cal.get("trim_left", 1.0), self.cal.get("trim_right", 1.0))
        self.t0 = time.monotonic()
        self.car = Car(a.host)
        self.safety = Safety(self.car, self.log, max_speed=255)
        self.safety.trim = self.trim
        self.stats = {"cmds": 0, "turns": 0, "backups": 0, "max_v": 0, "stops": 0}
        self.heading_ref = None
        self.recent = []          # last distance readings, for the median
        self.v_cmd = 0.0          # current commanded speed, slew limited
        self.open_side = 1        # +1 curve left, -1 curve right: the side last seen more open

    def log(self, ev, **f):
        print(f"[{time.monotonic() - self.t0:6.1f}] {ev} {json.dumps(f) if f else ''}", flush=True)

    def rep(self, frame, wait=0.6):
        return self.car.send(frame, wait=wait)[1]

    def tele(self, max_age=0.5):
        t = self.car.telemetry
        if t and time.time() - t["t"] <= max_age:
            self.safety.mark_alive(); self.safety.update_distance(t["dist"], ahead=True)
            return t

    def yaw(self):
        t = self.tele(0.4)
        if t: return t["yaw"]
        r = self.rep({"N": 24}, 0.5); m = re.search(r"_(-?\d+)\}", r or "")
        return int(m.group(1)) / 10.0 if m else None

    def servo(self, deg):
        self.rep({"N": 5, "D1": 1, "D2": deg}, 0.5)

    def range_at(self, deg):
        self.servo(deg); time.sleep(0.15)
        r = self.rep({"N": 21, "D1": 2}, 0.7); m = re.search(r"_(\d+)\}", r or "")
        return int(m.group(1)) if m else None

    # ---------------------------------------------------------------- moves
    def turn(self, deg, tol=3.0):
        """Closed-loop turn by deg, positive = left, on the gyro."""
        y0 = self.yaw()
        if y0 is None: return 0.0
        target = y0 + self.left_sign * deg; t0 = time.time()
        while time.time() - t0 < 6 and not self.safety.estop:
            y = self.yaw()
            if y is None: continue
            rem = (target - y) * self.left_sign
            if abs(rem) <= tol: break
            d1 = 1 if rem > 0 else 2
            if abs(rem) > 60: self.rep({"N": 2, "D1": d1, "D2": 160, "T": 220}, 0.05); time.sleep(0.24)
            elif abs(rem) > 25: self.rep({"N": 2, "D1": d1, "D2": 140, "T": 110}, 0.05); time.sleep(0.25)
            elif abs(rem) > 10: self.rep({"N": 2, "D1": d1, "D2": 120, "T": 70}, 0.05); time.sleep(0.25)
            else: self.rep({"N": 2, "D1": d1, "D2": 110, "T": 40}, 0.05); time.sleep(0.25)
        self.rep({"N": 100}, 0.2); y1 = self.yaw()
        self.stats["turns"] += 1
        return ((y1 - y0) * self.left_sign) if (y1 is not None) else 0.0

    def drive(self, v, yaw_err):
        """Wheel speeds for forward speed v with a heading-hold correction (deg error, positive = need left).
        Speed is slew limited: up 25 PWM per command (0 to 230 in about a second), down 60 (brakes faster)."""
        self.v_cmd = min(v, self.v_cmd + 25) if v > self.v_cmd else max(v, self.v_cmd - 60)
        v = self.v_cmd
        if abs(yaw_err) < 1.0: yaw_err = 0.0                      # deadband: gyro resolution is 0.1 deg
        k = 1.6                                                   # PWM per degree
        corr = max(-30.0, min(30.0, k * yaw_err))
        left, right = v - corr, v + corr                          # steer left = slow the left wheel
        left = max(0, min(255, left)); right = max(0, min(255, right))
        if self.safety.drive_diff(left, right):
            self.stats["cmds"] += 1; self.stats["max_v"] = max(self.stats["max_v"], int(v))

    def blocked(self, d):
        self.safety.stop("blocked"); self.stats["stops"] += 1
        left = self.range_at(140) or 0; right = self.range_at(40) or 0; self.servo(90)
        self.log("blocked", ahead=d, left=left, right=right)
        self.open_side = 1 if left >= right else -1
        if max(left, right) < SIDE_MIN:
            self.safety.pulse(DIR_BACK, 150, 450); time.sleep(0.6); self.stats["backups"] += 1; self.recent = []
            deg = 180 if abs(left - right) < 15 else (150 if left > right else -150)
        else:
            deg = random.choice((70, 95)) * (1 if left >= right else -1)
        ach = self.turn(deg); self.log("turned", asked=deg, achieved=round(ach, 1))
        self.heading_ref = None

    # ------------------------------------------------------------------ run
    def run(self):
        keyboard_estop(self.safety)
        self.log("start", vmax=self.a.vmax, duration=self.a.duration, left_sign=self.left_sign, trim=self.trim)
        self.servo(90); self.log("recal", reply=self.rep({"N": 26}, 3.0))
        self.rep({"N": 25, "D1": 70}, 0.5); time.sleep(0.4)
        stuck_since = None; last_d = None; last_cmd = 0.0
        try:
            while time.monotonic() - self.t0 < self.a.duration and not self.safety.estop:
                t = self.tele()
                if t is None:
                    if self.safety.stale(): self.safety.stop("stale"); self.log("abort", reason="no telemetry"); break
                    time.sleep(0.05); continue
                raw, y, mv = t["dist"], t["yaw"], t.get("mv")
                self.recent = (self.recent + [raw])[-5:]
                d = sorted(self.recent)[len(self.recent) // 2]        # median: speed decisions
                d_emerg = min(self.recent[-2:])                       # two lowest recent: emergency stop
                if not t["ground"]:
                    self.safety.stop("lifted"); self.log("abort", reason="lifted / cliff"); break
                if self.a.min_mv and mv and mv < self.a.min_mv:
                    self.safety.stop("battery"); self.log("abort", reason="battery low", mv=mv); break
                if d <= D_STOP or d_emerg <= D_STOP - 15:
                    self.blocked(d); stuck_since = None; self.recent = []; self.v_cmd = 0.0; continue
                # stuck: driving but the distance is not falling
                if last_d is not None and abs(d - last_d) < 2 and d < 200:
                    stuck_since = stuck_since or time.monotonic()
                    if time.monotonic() - stuck_since > 2.5:
                        self.log("stuck", d=d); self.blocked(d); stuck_since = None; continue
                else:
                    stuck_since = None
                last_d = d
                if self.heading_ref is None: self.heading_ref = y
                yaw_err = (self.heading_ref - y) * self.left_sign
                if d < D_CURVE:
                    # bend away early instead of stopping: ask for a heading offset toward the open side,
                    # growing as the obstacle nears (up to 25 deg), and let the heading hold steer it
                    bend = 25.0 * (D_CURVE - d) / (D_CURVE - D_STOP)
                    self.heading_ref = y + self.left_sign * self.open_side * bend
                    yaw_err = self.open_side * bend
                else:
                    self.heading_ref = y            # clear ahead: hold whatever heading we have
                    yaw_err = 0.0
                v = self.a.vmax if d >= D_CLEAR else (80 + (self.a.vmax - 80) * (d - D_STOP) / (D_CLEAR - D_STOP)) if d > D_SLOW else 80
                if time.monotonic() - last_cmd >= 0.12:
                    self.drive(v, yaw_err); last_cmd = time.monotonic()
                time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        finally:
            self.safety.stop("end"); self.rep({"N": 25, "D1": 0}, 0.4); self.servo(90)
            self.log("end", **self.stats, frames=self.car.telemetry_count); self.car.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local"); ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--vmax", type=int, default=230)
    ap.add_argument("--min-mv", type=int, default=0, help="abort below this battery voltage; 0 disables")
    Dash(ap.parse_args()).run()


if __name__ == "__main__":
    main()
