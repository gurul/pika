#!/usr/bin/env python3
"""Precision motion test on the modified UNO firmware v3 (gyro N=24, recal N=26).

Closed-loop turns on the gyro, closed-loop distance on the sonar. Uses the
firmware's own moves: N=2 left/right for spins, N=4 wheel speeds for straight
driving with the calibrated trim.

  tools/precision.py probe          recalibrate, learn the yaw sign of a left turn, check forward is forward
  tools/precision.py turn 90        closed-loop turn (positive = left), report error
  tools/precision.py drive 40       closed-loop distance toward whatever is ahead
  tools/precision.py square 40      4 x (drive, turn 90 left), report closure
"""
import argparse, json, os, re, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car

CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")


class Precision:
    def __init__(self, host):
        self.c = Car(host)
        self.cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
        self.left_sign = self.cal.get("yaw_left_sign", 1)
        self.trim = (self.cal.get("trim_left", 1.0), self.cal.get("trim_right", 1.0))
        self.log = []

    def rep(self, frame, wait=0.6):
        return self.c.send(frame, wait=wait)[1]

    def yaw(self):
        for _ in range(3):
            r = self.rep({"N": 24}, 0.5); m = re.search(r"_(-?\d+)\}", r or "")
            if m: return int(m.group(1)) / 10.0

    def dist(self):
        for _ in range(3):
            r = self.rep({"N": 21, "D1": 2}, 0.7); m = re.search(r"_(\d+)\}", r or "")
            if m: return int(m.group(1))

    def stop(self): self.rep({"N": 100}, 0.3)
    def recal(self): return self.rep({"N": 26}, 3.0)

    def spin(self, left, speed, ms):
        self.rep({"N": 2, "D1": 1 if left else 2, "D2": speed, "T": ms}, 0.05)

    def straight(self, speed):
        self.rep({"N": 4, "D1": int(speed * self.trim[0]), "D2": int(speed * self.trim[1])}, 0.05)

    # ---------------------------------------------------------------- probe
    def probe(self):
        self.rep({"N": 5, "D1": 1, "D2": 90}, 0.8); time.sleep(0.5)
        print("gyro recalibration (car still):", self.recal())
        y0 = self.yaw(); self.spin(True, 140, 400); time.sleep(0.9); dy = self.yaw() - y0
        print(f"firmware LEFT spin 400 ms: yaw {dy:+.1f} deg")
        self.left_sign = 1 if dy > 0 else -1
        y0 = self.yaw(); self.spin(False, 140, 400); time.sleep(0.9); print(f"firmware RIGHT spin 400 ms: yaw {self.yaw() - y0:+.1f} deg")
        d0 = self.dist(); y0 = self.yaw(); self.straight(120); time.sleep(0.6); self.stop(); time.sleep(0.6); d1 = self.dist(); dy = self.yaw() - y0
        print(f"wheel-speed forward 0.6 s: distance ahead {d0} -> {d1}, heading change {dy:+.1f} deg")
        fwd_ok = d0 is not None and d1 is not None and d1 < d0
        self.cal["yaw_left_sign"] = self.left_sign; json.dump(self.cal, open(CAL, "w"), indent=2)
        print(f"saved yaw_left_sign={self.left_sign}; forward is {'forward' if fwd_ok else 'NOT confirmed (no obstacle ahead or moved away)'}")

    # ------------------------------------------------------------- closed loop
    def turn(self, deg, tol=1.5):
        """Turn by deg, positive = left. Coarse spin, then shrinking pulses."""
        y0 = self.yaw(); target = y0 + self.left_sign * deg; t0 = time.time()
        while time.time() - t0 < 12:
            y = self.yaw()
            if y is None: continue
            rem = (target - y) * self.left_sign          # degrees still to turn, positive = need left
            if abs(rem) <= tol: break
            left = rem > 0
            if abs(rem) > 40: self.spin(left, 150, 250); time.sleep(0.28)
            elif abs(rem) > 15: self.spin(left, 120, 120); time.sleep(0.3)
            elif abs(rem) > 5: self.spin(left, 110, 70); time.sleep(0.3)
            else: self.spin(left, 110, 45); time.sleep(0.3)
        self.stop(); time.sleep(0.5); ach = (self.yaw() - y0) * self.left_sign
        print(f"turn {deg:+.0f}: achieved {ach:+.1f} deg, error {ach - deg:+.1f} deg, {time.time() - t0:.1f} s")
        self.log.append(("turn", deg, ach)); return ach

    def drive(self, cm, tol=2):
        d0 = self.dist()
        if d0 is None or d0 >= 400 or d0 - cm < 22: sys.exit(f"need an obstacle ahead between {cm + 22} and 399 cm, have {d0}")
        target = d0 - cm; t0 = time.time(); y0 = self.yaw(); last = d0; agree = 0
        while time.time() - t0 < 20:
            d = self.dist()
            if d is None or d >= 400 or abs(d - last) > 30:      # sonar glitch: ignore one-off jumps
                continue
            last = d; rem = d - target
            agree = agree + 1 if rem <= tol else 0
            if agree >= 2: break                                   # two consecutive readings at the target
            if rem > 20: self.straight(120); time.sleep(0.15)
            else: self.straight(85); time.sleep(0.07); self.stop(); time.sleep(0.3)
        self.stop(); time.sleep(0.5); ds = sorted(x for x in (self.dist(), self.dist(), self.dist()) if x is not None); d1 = ds[len(ds) // 2]; moved = d0 - d1; dy = (self.yaw() - y0) * self.left_sign
        print(f"drive {cm}: moved {moved} cm, error {moved - cm:+d} cm, heading drift {dy:+.1f} deg, {time.time() - t0:.1f} s")
        self.log.append(("drive", cm, moved)); return moved

    def square(self, side):
        y_start = self.yaw(); d_start = self.dist()
        for i in range(4):
            print(f"-- leg {i + 1}"); self.drive(side); self.turn(90)
        print(f"square closure: heading {(self.yaw() - y_start) * self.left_sign:+.1f} deg (0 is perfect), sonar ahead {d_start} -> {self.dist()} cm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local"); ap.add_argument("cmd"); ap.add_argument("value", nargs="?", type=float)
    a = ap.parse_args(); p = Precision(a.host)
    try:
        if a.cmd == "probe": p.probe()
        elif a.cmd == "turn": p.turn(a.value or 90)
        elif a.cmd == "drive": p.drive(int(a.value or 40))
        elif a.cmd == "square": p.square(int(a.value or 40))
        elif a.cmd == "recal": print(p.recal())
    finally:
        p.stop(); p.c.close()


if __name__ == "__main__":
    main()
