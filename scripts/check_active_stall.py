#!/usr/bin/env python3
"""Offline reproduction of the physical scan lock and adversarial recovery cases."""
from pathlib import Path
import argparse
import json
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from tests.test_active_stall import StallTests
from active_roam import ActivePlanner, Config

parser = argparse.ArgumentParser()
parser.add_argument('--replay', type=Path)
args = parser.parse_args()

suite = unittest.defaultTestLoader.loadTestsFromTestCase(StallTests)
if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
    sys.exit(1)
if args.replay:
    rows = [json.loads(line) for line in args.replay.read_text().splitlines()]
    frozen = [r for r in rows if r['event'] == 'tick' and r['t'] >= 40]
    assert len(frozen) >= 100, 'recorded stationary segment missing'
    assert {r['look'] for r in frozen} == {70, 90}, 'original two-angle lock not reproduced'
    assert all(r['reason'] == 'braking_margin' and r['left'] == r['right'] == 0 for r in frozen)
    p = ActivePlanner(Config(vmax=140))
    p.cruise_heading = -frozen[0]['telemetry']['yaw'] + frozen[0]['heading']
    requested = set()
    for row in frozen:
        telemetry = row['telemetry']
        now = telemetry['mono']
        assert p.ingest(telemetry, now), 'recorded telemetry rejected'
        d = p.step(now)
        requested.add(d.look)
        assert d.left == d.right == d.spin == 0, 'frozen blocked observations cannot authorize motion'
    assert any(abs(angle-90) > 20 for angle in requested), 'replay never requests wider search'
    print(json.dumps(dict(replay_samples=len(frozen), original_angles=[70,90], requested_angles=sorted(requested))))
    # This is a sensing-policy replay, not a counterfactual driving simulation:
    # new requested angles have no fabricated measurements in the recorded log.
print('active stall verification passed')
