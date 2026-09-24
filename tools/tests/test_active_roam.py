import math
from pathlib import Path
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from active_roam import ActivePlanner, ActiveSession, Config, PolarMemory
from car import Car, parse_telemetry
from safety import Safety


def sample(seq, now, pan=90, distance=200, yaw=0, settled=True, mv=8000, ground=True, tick=None):
    return dict(seq=seq, mono=now, t=now, pan=pan, dist=distance, yaw=yaw,
                settled=settled, mv=mv, ground=ground,
                sample_ms=int(now * 1000) & 65535 if tick is None else tick, line=(500, 500, 500))


class PlannerTests(unittest.TestCase):
    def ready(self, distance=200):
        p = ActivePlanner()
        p.ingest(sample(1, 0, distance=distance), 0); p.step(0)
        p.ingest(sample(2, .1, distance=distance), .1)
        return p

    def test_positive_control_known_forward_drives(self):
        p = self.ready()
        d = p.step(.1)
        self.assertGreater(d.left, 0)
        self.assertEqual(d.left, d.right)

    def test_unknown_is_not_a_free_valley(self):
        p = self.ready(90)
        d = p.step(.1)
        self.assertEqual(d.heading, 0)
        self.assertNotEqual(d.look, 90)  # looks early, without blindly bending
        self.assertIsNone(p.memory.clearance(40, .1, 0))

    def test_search_looks_outward_and_centers_between_glances(self):
        p = self.ready(100)
        self.assertEqual(p.step(.1).look, 110)
        p.ingest(sample(3, .2, pan=110, distance=95), .2)
        p.ingest(sample(4, .3, pan=110, distance=95), .3)
        self.assertEqual(p.step(.3).look, 90)
        p.ingest(sample(5, .4, distance=100), .4)
        p.ingest(sample(6, .5, distance=100), .5)
        self.assertEqual(p.step(.5).look, 130)

    def test_confirmed_opening_selected_not_unseen_side(self):
        p = self.ready(90)
        for i, t in enumerate((.2, .3), 3):
            p.ingest(sample(i, t, pan=130, distance=250), t)
        d = p.step(.3)
        self.assertGreater(d.heading, 0)
        self.assertGreaterEqual(d.right, d.left)

    def test_single_far_spike_does_not_create_a_path(self):
        p = self.ready(90)
        p.ingest(sample(3, .2, pan=130, distance=35), .2)
        p.ingest(sample(4, .3, pan=130, distance=300), .3)
        self.assertLessEqual(p.memory.view(40, .3, 0)[0], 35)
        self.assertLessEqual(p.step(.3).heading, 0)  # never steer toward that unconfirmed right/left opening

    def test_one_close_return_brakes_immediately(self):
        p = self.ready()
        self.assertGreater(p.step(.1).left, 0)
        p.ingest(sample(3, .2, distance=12), .2)
        d = p.step(.2)
        self.assertEqual((d.left, d.right, d.reason), (0, 0, "close_obstacle"))

    def test_expiry_travel_and_yaw_invalidate_free_space(self):
        p = self.ready()
        self.assertIsNotNone(p.memory.view(0, .1, 0))
        self.assertIsNone(p.memory.view(0, 2.5, 0))
        self.assertIsNone(p.memory.view(60, .1, 0))
        self.assertLess(p.memory.view(0, .1, 50)[0], p.memory.view(0, .1, 0)[0])

    def test_side_returns_never_refresh_forward(self):
        p = self.ready()
        for i in range(3, 20):
            p.ingest(sample(i, i / 10, pan=150, distance=200), i / 10)
        d = p.step(1.9)
        self.assertEqual(d.reason, "unknown_ahead")
        self.assertEqual(d.left, 0)

    def test_unknown_echo_revokes_previous_clearance(self):
        p = self.ready()
        p.ingest(sample(3, .2, distance=0), .2)
        self.assertEqual(p.step(.2).left, 0)

    def test_latency_and_blind_time_reduce_speed(self):
        p = ActivePlanner()
        self.assertLess(p.speed_limit(65, .9), p.speed_limit(65, 0))
        self.assertLess(p.speed_limit(45), p.speed_limit(150))

    def test_recovery_only_turns_toward_a_confirmed_opening(self):
        p = self.ready(48)
        self.assertEqual(p.step(.1).spin, 0)
        for i, t in enumerate((.2, .3), 3):
            p.ingest(sample(i, t, pan=170, distance=220), t)
        d = p.step(.3)
        self.assertEqual(d.reason, "recover_turn")
        self.assertEqual((d.left, d.right, d.spin), (0, 0, 1))
        p.ingest(sample(5, .4, distance=15), .4)
        self.assertEqual(p.step(.4).spin, 0)

    def test_acceleration_bounded_and_estop_immediate(self):
        p = self.ready()
        last = 0
        for i in range(2, 9):
            now = i * .1
            p.ingest(sample(i + 1, now), now)
            d = p.step(now)
            self.assertLessEqual(d.left - last, p.cfg.accel_pwm_s * .2 + 1e-6)
            self.assertLessEqual(max(d.left, d.right), p.cfg.vmax)
            last = d.left
        d = p.step(.81, estop=True)
        self.assertEqual((d.left, d.right), (0, 0))


class FakeCar:
    def __init__(self):
        self.sent = []; self.telemetry = None
    def send(self, frame, wait=.5):
        self.sent.append(dict(frame))
        return "{}", "{1_ok}"


class IntegrationTests(unittest.TestCase):
    def test_parse_legacy_and_extended(self):
        a = parse_telemetry("{T_100_-20_500_501_502_1_8000}", 10)
        self.assertEqual(a["yaw"], -2)
        self.assertNotIn("pan", a)
        b = parse_telemetry("{T_0_-20_500_501_502_1_8000_-130_4_65000}", 10)
        self.assertFalse(b["settled"])
        self.assertEqual(b["pan"], 130)
        self.assertIsNone(parse_telemetry("{T_10_bad_500_501_502_1_8000}"))
        self.assertIsNone(parse_telemetry("{T_100_0_500_501_502_1_8000_180_4_100}"))

    def test_socket_reader_fragmentation_heartbeat_and_tagged_ack(self):
        server = socket.socket(); server.bind(("127.0.0.1", 0)); server.listen()
        failure = []
        def peer():
            try:
                s, _ = server.accept()
                with s:
                    s.settimeout(2)
                    s.sendall(b"noise{T_100_0_500_500_")
                    s.sendall(b"500_1_8000_90_1_100}{Heartbeat}")
                    buf = b""
                    while b'"N":27' not in buf:
                        buf += s.recv(1024)
                    s.sendall(b"{ok}{1_8000}")
            except Exception as exc:
                failure.append(exc)
        th = threading.Thread(target=peer); th.start()
        car = Car("127.0.0.1", server.getsockname()[1], pan_center=90)  # no offset: this tests the reader
        try:
            self.assertEqual(car.send({"N": 27})[1], "{1_8000}")
            deadline = time.monotonic() + 1
            while car.telemetry is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(car.telemetry["pan"], 90)
        finally:
            car.close(); th.join(2); server.close()
        self.assertFalse(failure)

    def test_duplicate_stale_unsettled_and_out_of_order(self):
        p = ActivePlanner()
        a = sample(1, 0)
        self.assertTrue(p.ingest(a, 0))
        self.assertFalse(p.ingest(a, .1))
        self.assertFalse(p.ingest(sample(0, .2), .2))
        self.assertFalse(p.ingest(sample(2, .1), 1))
        self.assertTrue(p.ingest(sample(2, .1, pan=130, settled=False), .1))
        self.assertEqual(len(p.memory.rays), 1)
        self.assertEqual(p.step(.6).reason, "stale_telemetry")

    def test_sequence_and_tick_wrap(self):
        p = ActivePlanner()
        self.assertTrue(p.ingest(sample(65535, 0, tick=65500), 0))
        self.assertTrue(p.ingest(sample(0, .1, tick=64), .1))

    def test_buffered_frames_do_not_refresh_liveness(self):
        p = ActivePlanner()
        p.ingest(sample(1, 0, tick=100), 0)
        self.assertFalse(p.ingest(sample(2, .7, tick=200), .7))
        self.assertFalse(p.ingest(sample(3, .8, tick=300), .8))
        self.assertEqual(p.last_receipt, 0)

    def session(self, observe=False):
        c, p, clock = FakeCar(), ActivePlanner(), [0.0]
        s = Safety(c, lambda *a, **kw: None, max_speed=110, clock=lambda: clock[0])
        s.DEADMAN_S = .4
        session = ActiveSession(c, p, s, lambda *a, **kw: None, observe)
        for i in range(1, 5):
            clock[0] = i / 10
            c.telemetry = sample(i, clock[0]); session.tick(clock[0])
        return c, p, s, clock, session

    def test_known_clear_path_sends_both_wheels_and_failure_stops(self):
        for fault in ("ground_signal", "stale_telemetry", "estop"):
            c, p, s, clock, run = self.session()
            self.assertTrue(any(f["N"] == 4 for f in c.sent))  # positive control
            c.sent.clear(); clock[0] = 1 if fault == "stale_telemetry" else .5
            if fault != "stale_telemetry":
                c.telemetry = sample(5, .5, ground=fault != "ground_signal")
            s.estop = fault == "estop"
            d = run.tick(clock[0])
            self.assertEqual(d.reason, fault)
            self.assertFalse(any(f["N"] in (1, 2, 3, 4) for f in c.sent))
            self.assertTrue(any(f["N"] == 100 for f in c.sent))

    def test_battery_reading_does_not_block_drive(self):
        for mv in (6561, 6500, 0, None):
            with self.subTest(mv=mv):
                c, p, s, clock, run = self.session()
                c.sent.clear()
                for i in range(5, 9):
                    clock[0] = i / 10
                    c.telemetry = sample(i, clock[0], mv=mv)
                    d = run.tick(clock[0])
                self.assertGreater(d.left, 0)
                self.assertGreater(d.right, 0)
                self.assertTrue(any(f["N"] == 4 for f in c.sent))
                self.assertEqual(p.mv, mv)

    def test_observe_mode_has_no_motor_frames(self):
        c, _, _, _, _ = self.session(observe=True)
        self.assertFalse(any(f["N"] in (1, 2, 3, 4) for f in c.sent))
        self.assertTrue(any(f["N"] == 28 for f in c.sent))

    def test_pan_command_failure_latches_estop(self):
        c, p, s, clock, run = self.session()
        c.send = lambda frame, wait=.5: ("{}", None)
        run.sent_pan = None
        clock[0] = .5; c.telemetry = sample(5, .5)
        run.tick(.5)
        self.assertFalse(s.continuous)  # stop while the pan command is unconfirmed
        clock[0] = 1.01
        run.tick(1.01)
        self.assertTrue(s.estop)
        self.assertFalse(s.continuous)

    def test_safety_deadman_runs_after_expiry_and_close_pivot_denied(self):
        c, _, s, clock, _ = self.session()
        clock[0] = 1.1
        s.mark_alive()
        self.assertTrue(s.watchdog())
        self.assertEqual(c.sent[-1]["N"], 100)
        s.update_distance(10)
        self.assertFalse(s.drive_diff(0, 100))


if __name__ == "__main__":
    unittest.main()
