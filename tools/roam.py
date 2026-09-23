#!/usr/bin/env python3
"""Fast, smooth reactive roaming: VFH-lite on a servo-scanned sonar.

Method (Borenstein and Koren's Vector Field Histogram, in the VFH+ form):
  1. Keep a polar histogram of obstacle density over the front 180 degrees in
     10-degree sectors. Each sonar reading raises the density of its sector
     and of the neighbours inside the robot's enlargement angle
     asin((robot_radius + safety) / range). Densities decay over time so the
     round-robin scan stays fresh.
  2. Binarise with hysteresis (tau_high blocks, tau_low frees).
  3. Choose the steering sector by cost
       g = mu1 * |sector - target| + mu2 * |sector - heading| + mu3 * |sector - previous|
     with mu1 > mu2 + mu3 so the goal wins but heading and the last choice
     smooth the path. The target is straight ahead, with a slow random wander.
  4. Linear speed shrinks with the density ahead; steering becomes a
     differential wheel-speed pair (smooth arc), never a stop-and-spin, unless
     every sector is blocked, in which case the car backs up and spins by a
     calibrated angle toward the freest side.

Scan pattern: the servo looks straight ahead every step and alternates one
side glance every other step (the UNO blocks 0.5 s per servo move, which is
the loop's only real cost).

Safety: tools/safety.py drive_diff() with a 1 s software deadman plus the
camera bridge's 3 s heartbeat cutoff; cliff/lift check every 2 s; e-stop on
q or space.

  tools/roam.py [--duration 240] [--vmax 200]
"""
import argparse, json, math, os, random, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car import Car
from sonar import Sonar, MAX_RANGE
from safety import Safety, DIR_BACK, DIR_LEFT, DIR_RIGHT, MAX_PULSE_MS
from explore import keyboard_estop

SECTOR = 10
SECTORS = list(range(10, 171, SECTOR))          # servo-reachable headings, 90 = ahead
ROBOT_R, SAFETY_D = 13.0, 12.0                  # cm: half width of the car, clearance
TAU_HIGH, TAU_LOW = 0.55, 0.30                  # binary histogram hysteresis
MU1, MU2, MU3 = 5.0, 2.0, 2.0                   # cost weights (VFH+: mu1 > mu2 + mu3)
D_STOP, D_SLOW, D_CLEAR = 26.0, 45.0, 110.0     # cm ahead
D_NEAR, D_FAR = 25.0, 120.0                     # density 1 at D_NEAR, 0 at D_FAR (the sonar caps at 150)
GLANCES = [70, 110, 50, 130, 70, 110, 30, 150]  # near-path angles twice as often as the wide ones


class Histogram:
    def __init__(self):
        self.h = {s: 0.0 for s in SECTORS}
        self.blocked = {s: False for s in SECTORS}
        self.stamp = {s: 0.0 for s in SECTORS}

    def decay(self, dt):
        f = math.exp(-dt / 2.5)                    # forget in a few seconds
        for s in SECTORS:
            self.h[s] *= f

    def add(self, angle, r, now):
        """Density from one reading: high when close, spread over the
        enlargement angle so the car keeps its width clear."""
        if r is None:
            return
        self.stamp[angle] = now
        if r >= MAX_RANGE:
            self.h[angle] = min(self.h[angle], 0.1)
            return
        mag = max(0.0, min(1.0, (D_FAR - r) / (D_FAR - D_NEAR)))   # blocked below ~68 cm, free above ~90 cm
        enl = math.degrees(math.asin(min(1.0, (ROBOT_R + SAFETY_D) / max(r, ROBOT_R + SAFETY_D + 1))))
        for s in SECTORS:
            if abs(s - angle) <= enl:
                self.h[s] = max(self.h[s], mag)
        for s in SECTORS:
            if self.h[s] >= TAU_HIGH:
                self.blocked[s] = True
            elif self.h[s] <= TAU_LOW:
                self.blocked[s] = False

    def choose(self, target, heading, previous):
        best, best_g = None, 1e9
        for s in SECTORS:
            if self.blocked[s]:
                continue
            g = MU1 * abs(s - target) + MU2 * abs(s - heading) + MU3 * abs(s - previous)
            if g < best_g:
                best, best_g = s, g
        return best

    def freest(self):
        return min(SECTORS, key=lambda s: (self.h[s], abs(s - 90)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="elegoo-car.local")
    ap.add_argument("--duration", type=float, default=240)
    ap.add_argument("--vmax", type=int, default=160, help="wheel PWM at full clearance (sensing keeps up to about here)")
    ap.add_argument("--vmin", type=int, default=70, help="wheel PWM that still moves the car")
    ap.add_argument("--wander", type=float, default=12.0, help="seconds between random target changes")
    a = ap.parse_args()
    t0 = time.monotonic()
    def log(ev, **f):
        print(f"[{time.monotonic() - t0:6.1f}] {ev} {json.dumps(f) if f else ''}", flush=True)
    car = Car(a.host)
    safety = Safety(car, log, max_speed=255)
    hist = Histogram()
    def on_reading(angle, cm):
        safety.mark_alive(); safety.update_distance(cm, ahead=abs(angle - 90) <= 10)
        hist.add(angle, cm, time.monotonic())
    sonar = Sonar(car, on_reading=on_reading)
    keyboard_estop(safety)
    try:
        cal = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")))
    except Exception:
        cal = {}
    turn_dps = cal.get("turn_deg_per_s", 120.0)
    safety.trim = (cal.get("trim_left", 1.0), cal.get("trim_right", 1.0))
    log("trim", left=safety.trim[0], right=safety.trim[1])

    def spin(deg):
        """Calibrated spin, positive = left."""
        direction = DIR_LEFT if deg > 0 else DIR_RIGHT
        remaining = abs(deg) / turn_dps * 1000.0
        while remaining >= 60 and not safety.estop:
            ms = min(remaining, MAX_PULSE_MS)
            if not safety.pulse(direction, 150, ms):
                break
            time.sleep(ms / 1000.0 + 0.15); remaining -= ms

    target, previous = 90, 90
    last_wander = time.monotonic(); last_floor = 0.0; last_t = time.monotonic()
    step = 0; glance_i = 0; stalls = 0; last_d = None; same = 0
    stats = {"diff_cmds": 0, "spins": 0, "backups": 0}
    try:
        sonar.servo(90)
        while time.monotonic() - t0 < a.duration and not safety.estop:
            now = time.monotonic(); hist.decay(now - last_t); last_t = now
            # -- sense: ahead every step, one side glance every step when moving fast, else every other
            if step % 2 == 1 or (last_d is not None and last_d > D_CLEAR):
                ang = GLANCES[glance_i % len(GLANCES)]; glance_i += 1
                sonar.servo(ang); sonar.range_cm(); sonar.servo(90)
            d = sonar.range_cm()
            step += 1
            if d is None:
                safety.stop("no_echo"); continue
            if now - last_floor > 2.0:
                last_floor = now
                line, r = car.send({"N": 23}, wait=0.6)
                if r and "_true" in r:
                    safety.stop("lifted"); log("lifted"); time.sleep(1.0); continue
            if now - last_wander > a.wander:
                last_wander = now; target = random.choice([60, 70, 80, 90, 90, 100, 110, 120])
                log("wander", target=target)
            # -- stuck: distance not changing while we think we are moving
            if last_d is not None and d < 120 and abs(d - last_d) < 3 and safety.moving_until > now:
                same += 1
            else:
                same = 0
            last_d = d
            # -- decide
            sector = hist.choose(target, 90, previous)
            if d <= D_STOP or sector is None or same >= 4:
                reason = "too_close" if d <= D_STOP else ("all_blocked" if sector is None else "stuck")
                safety.stop(reason); stalls += 1
                if d <= D_STOP + 6:
                    safety.pulse(DIR_BACK, 140, 400); time.sleep(0.65); stats["backups"] += 1
                # full look to both sides before choosing
                for ang in (30, 60, 120, 150):
                    sonar.servo(ang); sonar.range_cm()
                sonar.servo(90)
                free = hist.freest()
                deg = free - 90 if abs(free - 90) >= 20 else (90 if random.random() < 0.5 else -90)
                log("spin", reason=reason, to_sector=free, deg=deg, hist={s: round(v, 2) for s, v in hist.h.items() if v > 0.05})
                spin(deg); stats["spins"] += 1
                previous, same = 90, 0
                for s in SECTORS:
                    hist.h[s] *= 0.3                    # the world rotated under the histogram
                continue
            # -- smooth steering: speed from clearance, curvature from the chosen sector
            v = a.vmin + (a.vmax - a.vmin) * max(0.0, min(1.0, (d - D_SLOW) / (D_CLEAR - D_SLOW)))
            steer = max(-1.0, min(1.0, (sector - 90) / 40.0))   # 40 degrees off = full differential
            k = 1.0                                      # inner wheel down to 0 at full steer
            left = v * (1.0 - k * max(0.0, steer)) ; right = v * (1.0 - k * max(0.0, -steer))
            left = max(a.vmin * 0.35, left); right = max(a.vmin * 0.35, right)
            if safety.drive_diff(left, right):
                stats["diff_cmds"] += 1
                if step % 6 == 0:
                    log("drive", d=d, sector=sector, v=round(v), L=round(left), R=round(right))
            previous = sector
    except KeyboardInterrupt:
        pass
    finally:
        safety.stop("end")
        try: sonar.servo(90)
        except Exception: pass
        log("end", stalls=stalls, estop=safety.estop, **stats)
        car.close()


if __name__ == "__main__":
    main()
