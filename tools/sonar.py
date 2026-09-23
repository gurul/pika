#!/usr/bin/env python3
"""Sonar sweeps and motion calibration for the car.

The ultrasonic sensor rides on the camera's pan servo, so a servo sweep turns
one range sensor into a coarse range scan. The UNO firmware takes the servo
angle in degrees but its driver moves in 10-degree steps between 10 and 170,
and blocks for 500 ms per move, so a sweep is 17 positions at best.

  tools/sonar.py sweep [--step 15]          one sweep, printed as a bar chart
  tools/sonar.py calibrate-forward          drive at a wall, measure cm/s
  tools/sonar.py calibrate-turn             turn in place, measure deg/s
"""
import argparse, json, math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car  # noqa: E402

SERVO_SCALE = 1           # D2 is degrees; the firmware divides by 10 because its driver counts in 10-degree units
SERVO_CENTER = 90         # degrees, straight ahead
SETTLE_S = 0.12           # the firmware already blocks 500 ms per servo move before it acks
SERVO_MIN, SERVO_MAX = 10, 170
MAX_RANGE = 150           # firmware caps at 150 cm; 150 means "no echo"
CAL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")


class Sonar:
    """Servo-swept ultrasonic ranger over one TCP link."""

    def __init__(self, car, on_reading=None):
        self.car = car
        self.angle = None
        self.on_reading = on_reading      # callable(angle_deg, range_cm) for every live reading

    def servo(self, deg):
        deg = max(SERVO_MIN, min(SERVO_MAX, int(round(deg / 10.0)) * 10))
        for attempt in (1, 2):
            line, r = self.car.send({"N": 5, "D1": 1, "D2": deg * SERVO_SCALE}, wait=1.0)
            if r is not None:
                break
        else:
            raise OSError(f"servo {deg}: no ack after 2 tries")
        time.sleep(SETTLE_S)
        self.angle = deg

    def range_cm(self, samples=1):
        """One reading, or the median of several. 0 is a failed echo and is dropped."""
        vals = []
        for _ in range(samples):
            line, r = self.car.send({"N": 21, "D1": 2}, wait=0.8)
            v = _num(r)
            if v is not None and v > 0:
                vals.append(v)
        if not vals:
            return None
        vals.sort()
        v = vals[len(vals) // 2]
        if self.on_reading:
            self.on_reading(self.angle if self.angle is not None else SERVO_CENTER, v)
        return v

    def sweep(self, step=10, samples=1, lo=SERVO_MIN, hi=SERVO_MAX):
        """Return [(angle_deg, range_cm|None)] from lo to hi in 10-degree steps.
        Angle 90 is ahead; angles above 90 look to the car's left."""
        step = max(10, int(round(step / 10.0)) * 10)
        angles = list(range(lo, hi + 1, step))
        if self.angle is not None and abs(self.angle - hi) < abs(self.angle - lo):
            angles.reverse()                     # start from the nearer end
        out = []
        for a in angles:
            self.servo(a)
            out.append((a, self.range_cm(samples)))
        out.sort()
        return out

    def center(self):
        self.servo(SERVO_CENTER)


def _num(reply):
    if not reply:
        return None
    i, j = reply.rfind("_"), reply.rfind("}")
    try:
        return int(reply[i + 1:j])
    except ValueError:
        return None


def bar_chart(scan):
    for a, r in scan:
        if r is None:
            print(f"{a:4d}°  {'?':>4}")
        else:
            bar = "█" * int(r / 3)
            print(f"{a:4d}°  {r:4d} {bar}{'  (no echo)' if r >= MAX_RANGE else ''}")


def load_cal():
    try:
        with open(CAL_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_cal(update):
    cal = load_cal()
    cal.update(update)
    with open(CAL_PATH, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"saved {CAL_PATH}: {json.dumps(update)}")


# ---------------------------------------------------------------- calibrate

def calibrate_forward(car, sonar, speed=120, pulse_ms=400, pulses=3):
    """Face a wall 80..140 cm away. Drives forward in pulses and measures the
    distance change with the sonar. Records cm per second at this speed."""
    sonar.center()
    d0 = sonar.range_cm(samples=3)
    if d0 is None or d0 >= MAX_RANGE or d0 < 60:
        sys.exit(f"need a wall 60..149 cm ahead, got {d0}")
    print(f"start {d0} cm")
    total_ms = 0
    for i in range(pulses):
        car.send({"N": 2, "D1": 3, "D2": speed, "T": pulse_ms}, wait=0.2)
        time.sleep(pulse_ms / 1000 + 0.6)
        total_ms += pulse_ms
        d = sonar.range_cm(samples=3)
        print(f"after pulse {i + 1}: {d} cm")
        if d is None or d < 25:
            break
    d1 = sonar.range_cm(samples=3)
    car.send({"N": 100}, wait=0.3)
    moved = d0 - d1
    cm_per_s = moved / (total_ms / 1000)
    print(f"moved {moved} cm in {total_ms} ms -> {cm_per_s:.1f} cm/s at speed {speed}")
    save_cal({"forward_speed": speed, "forward_cm_per_s": round(cm_per_s, 1),
              "forward_pulse_ms": pulse_ms})


def circular_shift_deg(before, after, step):
    """Best rotation (deg) that aligns `after` onto `before`, by circular
    cross-correlation of two sweeps taken at the same angles. Positive means
    the car turned left (scene shifted toward lower servo angles)."""
    import numpy as np
    b = np.array([min(r or MAX_RANGE, MAX_RANGE) for _, r in before], float)
    a = np.array([min(r or MAX_RANGE, MAX_RANGE) for _, r in after], float)
    b -= b.mean(); a -= a.mean()
    best, best_k = -1e9, 0
    n = len(b)
    for k in range(-(n - 3), n - 2):          # keep at least 3 overlapping beams
        if k >= 0:
            x, y = b[k:], a[:n - k]
        else:
            x, y = b[:n + k], a[-k:]
        if len(x) < 3:
            continue
        score = float(np.dot(x, y)) / len(x)
        if score > best:
            best, best_k = score, k
    return best_k * step


def calibrate_turn(car, sonar, speed=140, pulse_ms=500, step=10):
    """Sweep, turn left for one pulse, sweep again, and recover the rotation
    by correlating the two sweeps. Records degrees per second at this speed.
    Works best with distinct features around (furniture, a doorway)."""
    before = sonar.sweep(step=step, samples=2)
    bar_chart(before)
    car.send({"N": 2, "D1": 1, "D2": speed, "T": pulse_ms}, wait=0.2)
    time.sleep(pulse_ms / 1000 + 0.8)
    car.send({"N": 100}, wait=0.3)
    after = sonar.sweep(step=step, samples=2)
    bar_chart(after)
    deg = circular_shift_deg(before, after, step)
    dps = deg / (pulse_ms / 1000)
    print(f"estimated turn {deg} deg in {pulse_ms} ms -> {dps:.0f} deg/s at speed {speed}")
    if abs(deg) < step:
        print("turn too small to measure; raise --pulse-ms")
    save_cal({"turn_speed": speed, "turn_deg_per_s": round(dps, 1), "turn_pulse_ms": pulse_ms})
    sonar.center()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sweep"); s.add_argument("--step", type=int, default=10); s.add_argument("--samples", type=int, default=1)
    s.add_argument("--json", action="store_true")
    f = sub.add_parser("calibrate-forward"); f.add_argument("--speed", type=int, default=120); f.add_argument("--pulse-ms", type=int, default=400)
    t = sub.add_parser("calibrate-turn"); t.add_argument("--speed", type=int, default=140); t.add_argument("--pulse-ms", type=int, default=500)
    a = ap.parse_args()
    car = Car(a.host)
    sonar = Sonar(car)
    try:
        if a.cmd == "sweep":
            t0 = time.time()
            scan = sonar.sweep(step=a.step, samples=a.samples)
            dt = time.time() - t0
            if a.json:
                print(json.dumps({"scan": scan, "seconds": round(dt, 2)}))
            else:
                bar_chart(scan)
                print(f"{len(scan)} readings in {dt:.1f} s")
            sonar.center()
        elif a.cmd == "calibrate-forward":
            calibrate_forward(car, sonar, a.speed, a.pulse_ms)
        elif a.cmd == "calibrate-turn":
            calibrate_turn(car, sonar, a.speed, a.pulse_ms)
    finally:
        car.send({"N": 100}, wait=0.3)
        car.close()


if __name__ == "__main__":
    main()
