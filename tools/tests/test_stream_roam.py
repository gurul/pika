"""Behavioral checks for streamed VFH roaming; these never connect to a device."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from active_roam import ActiveSession
from safety import Safety
from stream_roam import VfhPlanner, stationary_heading_check


def sample(seq, now, *, pan=90, distance=120, yaw=0, settled=True,
           ground=True, mv=8000, sample_ms=None):
    return dict(seq=seq, mono=now, t=now, pan=pan, dist=distance, yaw=yaw,
                settled=settled, ground=ground, mv=mv, line=(500, 500, 500),
                sample_ms=int(round(now * 1000)) & 65535 if sample_ms is None else sample_ms)


def static_scan(planner, duration=6, distance=120, noisy=False):
    """Independent stationary sensor harness with actual commanded pan delay.

    Geometry motion belongs in the separate simulator. Here a broad surface at
    a constant range isolates scan scheduling and command continuity.
    """
    pan, ready, seq = 90, 0.0, 0
    records = []
    for step in range(int(duration / .05)):
        now = round(step * .05, 6)
        if step % 2 == 0:
            seq += 1
            d = distance(pan, now) if callable(distance) else distance
            if noisy:
                d += (-1.5, 1.2, -.4, .9, -1.1)[seq % 5]
            planner.ingest(sample(seq, now, pan=pan, distance=d if now >= ready else 0,
                                  settled=now >= ready), now)
        decision = planner.step(now)
        records.append((now, decision))
        if decision.look != pan:
            ready = now + .08 + .003 * abs(decision.look - pan)
            pan = decision.look
    return records


class StreamBehaviorTests(unittest.TestCase):
    def ready(self, distance=120, cfg=None):
        p = VfhPlanner(cfg)
        for seq in range(1, 5):
            now = seq / 10
            p.ingest(sample(seq, now, distance=distance), now)
            p.step(now)
        return p

    def test_confirmed_clear_path_has_useful_wheel_power(self):
        records = static_scan(VfhPlanner(), duration=3, distance=200)
        settled = [d for t, d in records if t >= .8]
        self.assertGreaterEqual(sum(max(d.left, d.right) >= 70 for d in settled), .95 * len(settled))
        self.assertTrue(all(max(d.left, d.right) <= 110 for _, d in records))

    def test_scanning_and_driving_overlap_without_repeated_stiction(self):
        records = static_scan(VfhPlanner(), duration=6, distance=120)
        cruising = [d for t, d in records if t >= 1]
        self.assertTrue(any(d.look != 90 for d in cruising), "positive control: a scan actually occurs")
        self.assertTrue(any(d.look != 90 and max(d.left, d.right) >= 70 for d in cruising),
                        "must drive during a side glance")
        weak = [d for d in cruising if 0 < max(d.left, d.right) < 55]
        self.assertLessEqual(len(weak), 2, "must not repeatedly reset below physical stiction")
        stopped = [d for d in cruising if max(d.left, d.right) == 0 and not d.spin]
        self.assertLessEqual(len(stopped), .15 * len(cruising), "ample-clearance scanning must remain mostly continuous")

    def test_launch_crosses_stiction_then_accelerates_smoothly(self):
        planner = VfhPlanner()
        records = static_scan(planner, duration=2, distance=200)
        moving = [(t, d) for t, d in records if max(d.left, d.right) > 0]
        self.assertTrue(moving)
        self.assertAlmostEqual(max(moving[0][1].left, moving[0][1].right), planner.cfg.min_pwm)
        for (before_t, before), (after_t, after) in zip(moving, moving[1:]):
            self.assertLessEqual(max(after.left, after.right) - max(before.left, before.right),
                                 planner.cfg.accel_pwm_s * (after_t - before_t) + 1e-6)

    def test_observed_side_opening_produces_a_useful_forward_arc(self):
        planner, seq = VfhPlanner(), 0
        for pan, distance, count in ((90, 60, 4), (130, 220, 4), (90, 60, 2)):
            for _ in range(count):
                seq += 1
                now = seq / 10
                planner.ingest(sample(seq, now, pan=pan, distance=distance), now)
                decision = planner.step(now)
        self.assertEqual(decision.reason, "drive")
        self.assertEqual(decision.spin, 0)
        self.assertGreater(decision.heading, 0)
        self.assertGreaterEqual(decision.right, 70)
        self.assertGreater(decision.right - decision.left, 10)

    def test_small_side_range_changes_do_not_flip_turn_direction(self):
        planner, seq, headings = VfhPlanner(), 0, []
        for cycle in range(8):
            for pan in (50, 50, 90, 90, 130, 130, 90, 90):
                seq += 1
                now = seq / 10
                distance = 60 if pan == 90 else 220 + (1 if cycle % 2 else -1) * (pan - 90) / 40
                planner.ingest(sample(seq, now, pan=pan, distance=distance), now)
                decision = planner.step(now)
                if cycle >= 1 and decision.reason == "drive" and abs(decision.heading) >= 20:
                    headings.append(1 if decision.heading > 0 else -1)
        self.assertGreater(len(headings), 5, "positive control: repeated steering decisions exist")
        self.assertLessEqual(sum(a != b for a, b in zip(headings, headings[1:])), 1)

    def test_clear_returns_eventually_release_a_previous_obstacle(self):
        planner = VfhPlanner()
        decisions = []
        for seq in range(1, 151):
            now = seq / 10
            planner.ingest(sample(seq, now, distance=45 if now <= 1 else 200), now)
            decisions.append(planner.step(now))
        self.assertTrue(any(d.left == d.right == 0 for d in decisions[:10]),
                        "positive control: the initial obstacle blocks movement")
        self.assertTrue(all(max(d.left, d.right) >= 70 for d in decisions[-10:]),
                        "expired obstacle density must not latch blocked forever after clear returns")

    def test_overdue_unsettled_scan_cannot_extend_its_driving_budget(self):
        planner, seq, requested, launched = VfhPlanner(), 0, None, None
        records = []
        for step in range(50):
            now = round(step * .05, 6)
            if step % 2 == 0:
                seq += 1
                planner.ingest(sample(seq, now, pan=requested or 90,
                                      settled=requested is None,
                                      distance=200 if requested is None else 0), now)
            decision = planner.step(now)
            if requested is None and decision.look != 90:
                requested, launched = decision.look, now
            if launched is not None and now - launched >= 1.0:
                records.append(decision)
        self.assertIsNotNone(launched, "positive control: a scan was requested")
        self.assertTrue(records)
        self.assertTrue(all(d.left == d.right == d.spin == 0 for d in records),
                        "fresh transport packets cannot extend an uncompleted scan's bounded blind interval")

    def test_small_range_noise_does_not_chatter_between_drive_and_stop(self):
        records = static_scan(VfhPlanner(), duration=6, distance=90, noisy=True)
        cruising = [d for t, d in records if t >= 1]
        moving = [max(d.left, d.right) >= 55 for d in cruising]
        self.assertGreaterEqual(sum(moving), .8 * len(moving))
        transitions = sum(a != b for a, b in zip(moving, moving[1:]))
        self.assertLessEqual(transitions, 2, "small range changes must not repeatedly restart the motors")

    def test_every_driving_wheel_is_at_the_moving_floor_or_zero(self):
        planner, seq = VfhPlanner(), 0
        wheels = []
        for pan, distance, count in ((90, 60, 4), (130, 220, 4), (90, 60, 30)):
            for _ in range(count):
                seq += 1
                now = seq / 10
                planner.ingest(sample(seq, now, pan=pan, distance=distance), now)
                d = planner.step(now)
                if d.reason == "drive":
                    wheels += [d.left, d.right]
        self.assertTrue(any(w == 0 for w in wheels) or len(set(wheels)) > 1,
                        "positive control: a steered arc was commanded")
        self.assertFalse([w for w in wheels if 0 < w < planner.cfg.min_pwm])

    def test_no_moving_glance_while_steering_hard(self):
        p = self.ready(200)
        p.pan_request, p.blind_until = None, 0.0
        p.steer = .8
        p.center_ready_at = -10.0
        self.assertEqual(p._scan(1.0, (200, .95), moving=True), 90)
        p.steer, p.pan_request, p.blind_until = 0.0, None, 0.0
        self.assertNotEqual(p._scan(1.0, (200, .95), moving=True), 90,
                            "positive control: a straight car does glance")

    def test_launch_waits_for_the_sonar_to_face_forward(self):
        p = self.ready(200)
        p.left = p.right = 0.0
        p.pan_request, p.pan_requested_at = 170, .4
        p.ingest(sample(5, .5, pan=170, distance=200), .5)
        d = p.step(.5)
        self.assertEqual((d.left, d.right, d.reason, d.look), (0.0, 0.0, "centering", 90))

    def test_histogram_rotates_obstacles_with_a_left_turn(self):
        p = VfhPlanner()
        p.ingest(sample(1, .1, pan=140, distance=60), .1)
        before = {a for a, b in p.hist.blocked.items() if b}
        self.assertIn(140, before)
        # yaw_left_sign is -1: a raw yaw of -30 is a 30-degree left turn, so
        # every blocked sector moves 30 degrees to the right (toward 90).
        p.ingest(sample(2, .2, pan=90, distance=0, yaw=-30), .2)
        after = {a for a, b in p.hist.blocked.items() if b}
        self.assertEqual(after, {a - 30 for a in before if 10 <= a - 30 <= 170})

    def test_passed_side_obstacle_does_not_block_the_way_ahead(self):
        p = VfhPlanner()
        p.ingest(sample(1, .1, pan=20, distance=60), .1)
        p.ingest(sample(2, .2, pan=20, distance=60), .2)
        # 50cm of straight travel: the -70 degree obstacle is now beside/behind.
        clearance = p.memory.clearance
        p.travel = 50.0
        for seq, now in ((3, .3), (4, .4)):
            p.ingest(sample(seq, now, pan=90, distance=200), now)
        self.assertGreater(clearance(0, .4, p.travel), p.cfg.stop_cm + 10)

    def test_scored_gaze_prefers_the_stale_side_the_path_turns_toward(self):
        p = self.ready(200)
        now = 1.0
        # Fresh reading at 70 (right); nothing seen at 110 (left) recently.
        p.memory.add(p.heading - 20, 150, now - .05, p.travel)
        self.assertEqual(p._best_look(now, (70, 110), selected=20), 110)
        # Positive control: once the left is fresh and the right stale, it flips.
        p.memory.add(p.heading + 20, 150, now, p.travel)
        p.memory.rays = type(p.memory.rays)([r for r in p.memory.rays
                                              if abs(r.bearing - (p.heading - 20)) > 1], maxlen=160)
        self.assertEqual(p._best_look(now + .5, (70, 110), selected=0), 70)

    def test_close_return_stops_on_first_sample(self):
        p = self.ready(200)
        self.assertGreater(max(p.left, p.right), 0)
        p.ingest(sample(5, .5, distance=12), .5)
        d = p.step(.5)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))
        self.assertEqual(d.reason, "close_obstacle")

    def test_stale_ground_and_estop_stop_immediately(self):
        for fault in ("stale", "ground", "estop"):
            with self.subTest(fault=fault):
                p = self.ready(200)
                self.assertGreater(max(p.left, p.right), 0)
                now = 1.0 if fault == "stale" else .5
                if fault != "stale":
                    p.ingest(sample(5, now, distance=200, ground=fault != "ground"), now)
                d = p.step(now, estop=fault == "estop")
                self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_no_echo_revokes_previously_clear_forward_path(self):
        p = self.ready(200)
        self.assertGreater(max(p.left, p.right), 0)
        p.ingest(sample(5, .5, distance=0), .5)
        d = p.step(.5)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_unseen_side_never_counts_as_an_opening(self):
        p = self.ready(40)
        d = p.step(.41)
        self.assertEqual(d.spin, 0)
        self.assertEqual((d.left, d.right), (0, 0))

    def test_one_far_spike_does_not_open_a_blocked_heading(self):
        p = self.ready(40)
        p.ingest(sample(5, .5, pan=150, distance=28), .5)
        p.ingest(sample(6, .6, pan=150, distance=300), .6)
        d = p.step(.6)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_side_samples_cannot_keep_forward_clearance_alive(self):
        p = self.ready(200)
        for seq in range(5, 30):
            now = seq / 10
            p.ingest(sample(seq, now, pan=150, distance=200), now)
            d = p.step(now)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_duplicate_and_delayed_packets_do_not_refresh_motion_permission(self):
        p = self.ready(200)
        old = sample(4, .4, distance=200)
        self.assertFalse(p.ingest(old, .8))
        self.assertFalse(p.ingest(sample(5, 1.0, sample_ms=500), 1.0))
        d = p.step(1.0)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_unsettled_pan_return_cannot_open_a_side_path(self):
        p = self.ready(40)
        for seq in (5, 6):
            p.ingest(sample(seq, seq / 10, pan=150, distance=250, settled=False), seq / 10)
        d = p.step(.6)
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 0))

    def test_low_or_missing_battery_telemetry_is_not_a_drive_cutoff(self):
        for mv in (6500, 6100, 0, None):
            with self.subTest(mv=mv):
                p = VfhPlanner()
                for seq in range(1, 11):
                    now = seq / 10
                    p.ingest(sample(seq, now, distance=200, mv=mv), now)
                    d = p.step(now)
                self.assertGreaterEqual(max(d.left, d.right), 70)


class FakeCar:
    def __init__(self):
        self.sent = []
        self.telemetry = None
        self.fail_pan = False

    def send(self, frame, wait=.5):
        self.sent.append(dict(frame))
        return "{}", None if self.fail_pan and frame["N"] == 28 else "{1_ok}"


class StreamSessionTests(unittest.TestCase):
    def session(self, observe=False):
        car, planner, clock = FakeCar(), VfhPlanner(), [0.0]
        safety = Safety(car, lambda *args, **kwargs: None, max_speed=110, clock=lambda: clock[0])
        safety.DEADMAN_S = .4
        session = ActiveSession(car, planner, safety, lambda *args, **kwargs: None, observe)
        for seq in range(1, 6):
            clock[0] = seq / 10
            car.telemetry = sample(seq, clock[0], distance=200)
            session.tick(clock[0])
        return car, planner, safety, clock, session

    def test_faults_stop_previously_commanded_wheels(self):
        for fault in ("stale", "ground", "close", "estop"):
            with self.subTest(fault=fault):
                car, _, safety, clock, session = self.session()
                self.assertTrue(any(frame["N"] == 4 for frame in car.sent))
                car.sent.clear()
                clock[0] = 1.2 if fault == "stale" else .6
                if fault != "stale":
                    car.telemetry = sample(6, .6, distance=12 if fault == "close" else 200,
                                           ground=fault != "ground")
                safety.estop = fault == "estop"
                session.tick(clock[0])
                self.assertTrue(any(frame["N"] == 100 for frame in car.sent))
                self.assertFalse(any(frame["N"] in (1, 2, 3, 4) for frame in car.sent))

    def test_observe_mode_never_sends_motor_frames(self):
        car, _, _, _, _ = self.session(observe=True)
        self.assertFalse(any(frame["N"] in (1, 2, 3, 4) for frame in car.sent))
        self.assertTrue(any(frame["N"] == 28 for frame in car.sent))

    def test_unconfirmed_pan_timeout_stops_and_latches_estop(self):
        car, _, safety, clock, session = self.session()
        self.assertTrue(any(frame["N"] == 4 for frame in car.sent))
        car.fail_pan = True
        session.sent_pan = None
        clock[0] = .6
        car.telemetry = sample(6, .6, distance=200)
        session.tick(.6)
        car.sent.clear()
        clock[0] = 1.2
        session.tick(1.2)
        self.assertTrue(safety.estop)
        self.assertFalse(any(frame["N"] in (1, 2, 3, 4) for frame in car.sent))


class GyroStartupTests(unittest.TestCase):
    class GyroCar:
        def __init__(self, drift=0, corrected_drift=0, *, data=True, stop_ack=True):
            self.now = 0.0
            self.drift, self.corrected_drift = drift, corrected_drift
            self.data, self.stop_ack = data, stop_ack
            self.sent = []

        @property
        def telemetry(self):
            if not self.data:
                return None
            seq = int((self.now + 1e-8) * 10)
            return dict(seq=seq, mono=seq / 10, yaw=seq / 10 * self.drift)

        def send(self, frame, wait=.5):
            self.sent.append(dict(frame))
            if frame["N"] == 26:
                self.drift = self.corrected_drift
                return "{}", "{1_ok}"
            return "{}", "{ok}" if self.stop_ack else None

        def clock(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    def check(self, car, cancelled=lambda: False):
        events = []
        stationary_heading_check(car, lambda name, **fields: events.append((name, fields)),
                                 cancelled, clock=car.clock, sleep=car.sleep)
        return events

    def test_stable_gyro_does_not_recalibrate(self):
        car = self.GyroCar(drift=.3)
        events = self.check(car)
        self.assertEqual(car.sent, [{"N": 100}])
        self.assertTrue(any(name == "gyro_check" and fields["stable"] for name, fields in events))

    def test_drift_recalibrates_once_and_requires_stable_recheck(self):
        car = self.GyroCar(drift=43, corrected_drift=.1)
        events = self.check(car)
        self.assertEqual(car.sent, [{"N": 100}, {"N": 26}])
        checks = [fields["stable"] for name, fields in events if name == "gyro_check"]
        self.assertEqual(checks, [False, True])

    def test_persistent_drift_aborts(self):
        car = self.GyroCar(drift=43, corrected_drift=43)
        with self.assertRaisesRegex(RuntimeError, "gyro unstable"):
            self.check(car)
        self.assertEqual(car.sent, [{"N": 100}, {"N": 26}])

    def test_missing_telemetry_aborts(self):
        car = self.GyroCar(data=False)
        with self.assertRaisesRegex(RuntimeError, "insufficient fresh telemetry"):
            self.check(car)
        self.assertEqual(car.sent, [{"N": 100}])

    def test_unacknowledged_stop_aborts_before_calibration(self):
        car = self.GyroCar(drift=43, stop_ack=False)
        with self.assertRaisesRegex(RuntimeError, "stop not acknowledged"):
            self.check(car)
        self.assertEqual(car.now, 0)
        self.assertEqual(car.sent, [{"N": 100}])

    def test_cancel_aborts_before_calibration(self):
        car = self.GyroCar(drift=43)
        with self.assertRaisesRegex(RuntimeError, "estop"):
            self.check(car, cancelled=lambda: True)
        self.assertEqual(car.sent, [{"N": 100}])


if __name__ == "__main__":
    unittest.main()
