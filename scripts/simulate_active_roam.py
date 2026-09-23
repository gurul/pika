"""Independent 2D geometry and differential-drive plant for active-roam checks.

Models sonar as nearest of rays across a 30-degree cone, pan settling, 10 Hz
telemetry and 20 Hz host ticks. Ideal planar acoustics are NOT hardware proof.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from active_roam import ActivePlanner, Config


class World:
    def __init__(self, obstacles=()):
        self.obstacles = [(0, 600, -5, 0), (0, 600, 300, 305), (-5, 0, 0, 300), (600, 605, 0, 300), *obstacles]

    def contact(self, x, y, radius=13):
        return any(math.hypot(x - max(x0, min(x, x1)), y - max(y0, min(y, y1))) <= radius
                   for x0, x1, y0, y1 in self.obstacles)

    def ray(self, x, y, heading):
        dx, dy = math.cos(heading), math.sin(heading)
        nearest = 400.0
        for x0, x1, y0, y1 in self.obstacles:
            lo, hi = 0, 400
            for pos, direction, lower, upper in ((x, dx, x0, x1), (y, dy, y0, y1)):
                if abs(direction) < 1e-9:
                    if not lower <= pos <= upper:
                        hi = -1; break
                else:
                    a, b = (lower - pos) / direction, (upper - pos) / direction
                    lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
            if 0 <= lo <= hi:
                nearest = min(nearest, lo)
        return nearest

    def sonar(self, x, y, heading):
        return min(self.ray(x, y, heading + math.radians(a)) for a in range(-15, 16, 5))


def run(name, obstacles=(), duration=30, noise=False, seed=17, wheel_gain=.24):
    world, planner, rng = World(obstacles), ActivePlanner(Config()), random.Random(seed)
    x, y, yaw = 40.0, 150.0, 0.0
    pan, pan_ready = 90, 0.0
    seq, last_reason, previous_turn = 0, None, 0
    trace = []
    stats = dict(name=name, contacts=0, distance_cm=0.0, stops=0, steering_reversals=0,
                 glances=0, max_pwm=0.0, max_step_up=0.0, min_x=x, max_x=x)
    last_l = last_r = 0
    for step in range(int(duration / .05)):
        now = step * .05
        if step % 2 == 0:
            seq += 1
            d = world.sonar(x, y, yaw + math.radians(pan - 90))
            if noise:
                d += rng.uniform(-2, 2)
                if seq % 31 == 0:
                    d += 85
            settled = now >= pan_ready
            planner.ingest(dict(seq=seq, mono=now, sample_ms=int(now * 1000) & 65535,
                                yaw=-math.degrees(yaw), pan=pan, settled=settled,
                                dist=d if d < 400 and settled else 0,
                                mv=7900, ground=True), now)
        decision = planner.step(now)
        if decision.look != pan:
            pan_ready = now + .08 + .003 * abs(decision.look - pan)
            pan = decision.look; stats["glances"] += 1
        l, r = decision.left, decision.right
        stats["max_pwm"] = max(stats["max_pwm"], l, r)
        stats["max_step_up"] = max(stats["max_step_up"], l-last_l, r-last_r)
        if decision.reason != "drive" and last_reason == "drive":
            stats["stops"] += 1
        turn = 1 if r-l > 5 else -1 if l-r > 5 else 0
        if turn and previous_turn and turn != previous_turn:
            stats["steering_reversals"] += 1
        if turn:
            previous_turn = turn
        # Independent plant: 0.24 cm/s per PWM, 18 cm effective track width.
        # No stiction here; hardware testing must establish the low-speed floor.
        vl, vr = l * wheel_gain, r * wheel_gain
        v, omega = (vl + vr) / 2, (vr - vl) / 18
        if decision.spin:
            omega = decision.spin * .8  # average of short turning pulses
        x += v * math.cos(yaw + omega * .025) * .05
        y += v * math.sin(yaw + omega * .025) * .05
        yaw += omega * .05
        stats["distance_cm"] += v * .05
        stats["max_x"] = max(stats["max_x"], x)
        trace.append(dict(t=round(now, 3), x=x, y=y, yaw=math.degrees(yaw), **asdict(decision)))
        if world.contact(x, y):
            stats["contacts"] += 1
            break
        last_l, last_r, last_reason = l, r, decision.reason
    stats.update(end_x=x, end_y=y, last_reason=last_reason)
    return stats, trace, world


def verify():
    out = ROOT / "build/active-roam-simulation"
    out.mkdir(parents=True, exist_ok=True)
    # Positive/negative controls for the collision oracle, independent of policy.
    control = World([(190, 230, 120, 180)])
    assert control.contact(210, 150) and not control.contact(40, 150)
    scenarios = [("hallway", ((350, 355, 0, 300),), False),
                 ("offset_obstacle", ((190, 230, 120, 180),), False),
                 ("noisy_obstacle", ((190, 230, 120, 180),), True)]
    results = []
    failures = []
    for name, obstacles, noise in scenarios:
        stats, trace, world = run(name, obstacles, noise=noise)
        results.append(stats)
        (out / f"{name}.json").write_text(json.dumps(dict(stats=stats, trace=trace, obstacles=world.obstacles), indent=2))
        print(json.dumps(stats))
        if stats["contacts"]:
            failures.append(f"{name}: collision")
        if stats["distance_cm"] < 150:
            failures.append(f"{name}: insufficient progress")
        if obstacles and stats["max_x"] < 260:
            failures.append(f"{name}: did not pass obstacle")
        if stats["max_pwm"] > 110 or stats["max_step_up"] > 5.00001:
            failures.append(f"{name}: slew/speed limit violated")
    (out / "summary.json").write_text(json.dumps(results, indent=2))
    assert not failures, "; ".join(failures)


if __name__ == "__main__":
    verify()
