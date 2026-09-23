#!/usr/bin/env python3
"""Offline behavior and independent geometry/stiction checks. No device I/O."""
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from active_roam import ActivePlanner, Config
from stream_roam import VfhConfig, VfhPlanner
from simulate_active_roam import World


def simulate(name, planner, obstacles=(), duration=35, noisy=False, seed=17):
    """20Hz host, 10Hz sonar, finite pan slew, a nonzero wheel deadband.

    Range comes from independent geometry, not a scripted controller-friendly
    sequence. The 30-degree cone is inherited from World. No echo at max range
    remains zero. This is not a model of specular reflection or wheel slip.
    """
    world, rng = World(obstacles), random.Random(seed)
    x, y, yaw, pan, ready = 40.0, 150.0, 0.0, 90, 0.0
    seq, last_moving, previous_turn, moving_streak = 0, False, 0, 0
    trace, reasons = [], Counter()
    stats = dict(name=name, duration_s=duration, contacts=0, distance_cm=0.0,
                 net_displacement_cm=0.0, max_x=x, min_y=y, max_y=y,
                 drive_ticks=0, stopped_ticks=0, weak_ticks=0, inner_weak_ticks=0, arc_ticks=0,
                 stop_transitions=0, steering_reversals=0, glances=0,
                 max_pwm=0.0, unknown_samples=0, valid_samples=0,
                 longest_continuous_drive_s=0.0,
                 stiction_pwm=55, wheel_gain_cm_s_per_pwm=.24)
    for step in range(round(duration / .05)):
        now = round(step * .05, 6)
        if step % 2 == 0:
            seq += 1
            distance = world.sonar(x, y, yaw + math.radians(pan - 90))
            settled = now >= ready
            if noisy and distance < 400:
                distance += rng.uniform(-2, 2)
                if seq % 37 == 0:
                    distance = min(399, distance + 65)
            distance = max(0, distance) if distance < 400 and settled else 0
            if settled:
                stats["valid_samples" if distance else "unknown_samples"] += 1
            planner.ingest(dict(seq=seq, mono=now, t=now, sample_ms=int(round(now * 1000)) & 65535,
                                yaw=-math.degrees(yaw), pan=pan, settled=settled,
                                dist=distance, ground=True, mv=7900), now)
        decision = planner.step(now)
        reasons[decision.reason] += 1
        if decision.look != pan:
            ready = now + .08 + .003 * abs(decision.look - pan)
            pan = decision.look
            stats["glances"] += 1
        left, right = decision.left, decision.right
        assert math.isfinite(left) and math.isfinite(right)
        assert min(left, right) >= 0, "forward-only planner emitted negative drive"
        stats["max_pwm"] = max(stats["max_pwm"], left, right)
        moving = max(left, right) >= 55
        moving_streak = moving_streak + 1 if moving else 0
        stats["longest_continuous_drive_s"] = max(stats["longest_continuous_drive_s"], moving_streak * .05)
        stats["drive_ticks" if moving else "stopped_ticks"] += 1
        stats["weak_ticks"] += int(0 < max(left, right) < 55)
        # A stalled inner wheel turns a commanded curve into a pivot.
        stats["inner_weak_ticks"] += int(moving and 0 < min(left, right) < 55)
        stats["arc_ticks"] += int(moving and abs(left - right) >= 10)
        stats["stop_transitions"] += int(last_moving and not moving)
        turn = 1 if right - left >= 8 else -1 if left - right >= 8 else 0
        if turn and previous_turn and turn != previous_turn:
            stats["steering_reversals"] += 1
        if turn:
            previous_turn = turn
        # Stiction independently kills commands below the wheel's useful floor.
        vl, vr = (.24 * pwm if pwm >= 55 else 0.0 for pwm in (left, right))
        speed, omega = (vl + vr) / 2, (vr - vl) / 18
        if decision.spin:
            omega = .8 * decision.spin  # short-turn command duty-cycle average
        # Substeps ensure collision detection does not skip thin obstacles.
        contact = False
        for _ in range(5):
            x += speed * math.cos(yaw + omega * .005) * .01
            y += speed * math.sin(yaw + omega * .005) * .01
            yaw += omega * .01
            stats["distance_cm"] += speed * .01
            if world.contact(x, y):
                contact = True
                break
        stats["max_x"] = max(stats["max_x"], x)
        stats["min_y"], stats["max_y"] = min(stats["min_y"], y), max(stats["max_y"], y)
        trace.append(dict(t=now, x=x, y=y, yaw=math.degrees(yaw), **asdict(decision)))
        if contact:
            stats["contacts"] += 1
            break
        last_moving = moving
    stats.update(end_x=x, end_y=y, end_yaw=math.degrees(yaw),
                 net_displacement_cm=math.hypot(x - 40, y - 150), reasons=dict(reasons),
                 measured_elapsed_s=trace[-1]["t"] + .05)
    return dict(stats=stats, trace=trace, obstacles=world.obstacles)


def verify_simulation():
    output = ROOT / "build/vfh-restore-simulation"
    output.mkdir(parents=True, exist_ok=True)
    oracle = World([(190, 230, 120, 180)])
    assert oracle.contact(210, 150), "positive control must detect an obstacle contact"
    assert oracle.contact(177, 150), "contact oracle must include the 13cm body radius"
    assert not oracle.contact(40, 150), "initial pose must be clear"
    assert 149 < oracle.sonar(40, 150, 0) < 151, "forward test obstacle must be observed"
    scenarios = [
        ("offset_obstacle", ((190, 230, 120, 180),), False),
        ("noisy_obstacle", ((190, 230, 120, 180),), True),
        ("hallway_end", ((350, 355, 0, 300),), False),
    ]
    summaries, failures = [], []
    for name, obstacles, noisy in scenarios:
        data = simulate(name, VfhPlanner(VfhConfig()), obstacles, noisy=noisy)
        stats = data["stats"]
        summaries.append(stats)
        (output / f"{name}.json").write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps(stats, sort_keys=True))
        if stats["contacts"]:
            failures.append(f"{name}: geometry contact")
        if stats["distance_cm"] < 150:
            failures.append(f"{name}: traveled less than 150cm under stiction")
        if name != "hallway_end" and stats["max_x"] < 260:
            failures.append(f"{name}: failed to pass the obstacle")
        if stats["weak_ticks"] > 3:
            failures.append(f"{name}: repeated below-stiction commands ({stats['weak_ticks']})")
        if stats["inner_weak_ticks"] > 3:
            failures.append(f"{name}: inner wheel below stiction ({stats['inner_weak_ticks']})")
        if stats["stop_transitions"] > 6:
            failures.append(f"{name}: stop-go driving ({stats['stop_transitions']} stops)")
        if stats["arc_ticks"] < 10:
            failures.append(f"{name}: lacks useful continuous differential steering")
        if stats["max_pwm"] > 110 or stats["valid_samples"] < 20:
            failures.append(f"{name}: invalid speed or missing geometry observations")
    # Comparison is recorded, never used as the correctness oracle.
    baseline = simulate("active_baseline", ActivePlanner(Config()),
                        ((190, 230, 120, 180),), noisy=False)
    (output / "active_baseline.json").write_text(json.dumps(baseline, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(dict(
        model="10Hz sonar, 20Hz host, 80ms + 3ms/degree pan latency, 55PWM stiction, 13cm body radius",
        geometry_contact_positive_control=True, restored=summaries,
        active_baseline=baseline["stats"], failures=failures), indent=2) + "\n")
    if failures:
        raise AssertionError("; ".join(failures))


def main():
    mode = sys.argv[1] if len(sys.argv) == 2 else ""
    if mode == "tests":
        from tests.test_stream_roam import GyroStartupTests, StreamBehaviorTests, StreamSessionTests
        suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(case)
                                   for case in (StreamBehaviorTests, StreamSessionTests, GyroStartupTests))
        if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
            return 1
    elif mode == "simulation":
        verify_simulation()
    else:
        raise SystemExit("usage: check_stream_roam.py {tests|simulation}")
    print(f"stream roam {mode} passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
