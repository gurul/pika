"""Regressions derived from the 2026-09-22 physical scan lock."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from active_roam import ActivePlanner, ActiveSession, Config
from safety import Safety
from tests.test_active_roam import FakeCar, sample


def stationary_scan(ranges, seconds=12, goal=-20, vmax=140):
    """Independent servo/echo feedback; wheels stay stationary until an action.

    The physical trace measured 44..47 cm at 70/90 degrees. Other bearings
    here are explicit scenario fixtures, not inferred from that trace.
    """
    planner = ActivePlanner(Config(vmax=vmax))
    planner.goal, planner.goal_until = goal, 3
    planner.previous = goal or 0
    # The recorded gyro changed from 112.5 to 78.5 degrees during recovery;
    # the original cruise heading remained 34 degrees left of the new heading.
    planner.cruise_heading = -34
    pan, ready, seq = 90, 0.0, 0
    decisions, observed = [], []
    for step in range(round(seconds / .05)):
        now = step * .05
        if step % 2 == 0:
            seq += 1
            settled = now >= ready
            distance = ranges(pan) if settled else 0
            planner.ingest(dict(seq=seq, mono=now, sample_ms=round(now*1000),
                                yaw=0, pan=pan, settled=settled, dist=distance,
                                mv=6500, ground=True), now)
            if settled:
                observed.append(pan)
        decision = planner.step(now)
        decisions.append(decision)
        if decision.look != pan:
            ready = now + .08 + .003 * abs(decision.look - pan)
            pan = decision.look
        if decision.spin or decision.left or decision.right:
            break
    return planner, decisions, observed


class StallTests(unittest.TestCase):
    def test_rejected_goal_expands_search_to_observed_opening(self):
        p, decisions, observed = stationary_scan(lambda pan: 220 if pan <= 30 else 44)
        self.assertIn(30, observed, "scan remained locked to an unusable heading")
        self.assertTrue(any(d.spin == -1 for d in decisions), "confirmed side opening never used")
        self.assertTrue(all(d.left == d.right == 0 for d in decisions),
                        "44 cm front corridor must not authorize forward motion")

    def test_enclosed_car_searches_both_sides_without_driving(self):
        p, decisions, observed = stationary_scan(lambda pan: 44)
        self.assertLessEqual(min(observed), 30)
        self.assertGreaterEqual(max(observed), 150)
        self.assertTrue(all(d.left == d.right == d.spin == 0 for d in decisions))

    def test_glance_speed_budget_cannot_relock_a_centered_feasible_goal(self):
        for distance in (50, 55, 60):
            with self.subTest(distance=distance):
                p, decisions, observed = stationary_scan(
                    lambda pan: 220 if pan <= 30 else distance)
                if decisions[-1].reason == 'drive':
                    # At 60 cm a straight route can support the full scan
                    # budget; selecting it over an unexecutable curve is valid.
                    d = decisions[-1]
                    ahead = p.memory.view(p.heading, p.last_step, p.travel, p.cfg.ahead_s)
                    self.assertGreaterEqual(p._speed(d.clearance, ahead, d.heading, 110), p.cfg.min_pwm)
                else:
                    self.assertIn(30, observed)
                    self.assertTrue(any(d.spin == -1 for d in decisions))
                    self.assertTrue(all(d.left == d.right == 0 for d in decisions))
                self.assertIn(p.pan_request, (None, decisions[-1].look))

    def test_unknown_front_still_collects_side_information_without_driving(self):
        p, decisions, observed = stationary_scan(lambda pan: 0, goal=None)
        self.assertLessEqual(min(observed), 30)
        self.assertGreaterEqual(max(observed), 150)
        self.assertTrue(all(d.left == d.right == d.spin == 0 for d in decisions))

    def test_close_obstacle_never_authorizes_recovery(self):
        p, decisions, observed = stationary_scan(lambda pan: 220 if pan <= 30 else 15)
        self.assertTrue(all(d.left == d.right == d.spin == 0 for d in decisions))

    def test_missing_pan_ack_uses_fresh_angle_confirmation_while_stopped(self):
        car, clock = FakeCar(), [0.0]
        safety = Safety(car, lambda *a, **k: None, clock=lambda: clock[0])
        session = ActiveSession(car, ActivePlanner(), safety, lambda *a, **k: None)
        car.send = lambda frame, wait=.5: (car.sent.append(frame) or '{}', None)
        car.telemetry = sample(1, 0)
        session.tick(0)
        self.assertIsNotNone(session.pending_pan)
        self.assertFalse(safety.continuous)
        car.sent.clear()
        clock[0] = .1
        session.tick(.1)  # duplicate pre-command telemetry cannot confirm it
        self.assertIsNotNone(session.pending_pan)
        self.assertFalse(car.sent)
        angle = session.pending_pan[0]
        car.telemetry = sample(2, .2, pan=angle, settled=False)
        clock[0] = .2
        session.tick(.2)
        self.assertIsNone(session.pending_pan)
        self.assertFalse(safety.estop)
        self.assertEqual(session.sent_pan, angle)

    def test_decision_keeps_input_snapshot_when_transport_receives_new_frame(self):
        car = FakeCar()
        safety = Safety(car, lambda *a, **k: None, clock=lambda: 0)
        session = ActiveSession(car, ActivePlanner(), safety, lambda *a, **k: None)
        original = sample(1, 0)
        car.telemetry = original
        def send(frame, wait=.5):
            car.telemetry = sample(2, .1, distance=12)
            return '{}', '{1_ok}'
        car.send = send
        session.tick(0)
        self.assertIs(session.input_telemetry, original)
        self.assertTrue(session.input_accepted)
        self.assertEqual(session.planner.last_seq, 1)
        self.assertEqual(car.telemetry['seq'], 2)


if __name__ == '__main__':
    unittest.main()
