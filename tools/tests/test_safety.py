import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from safety import Safety, DIR_FWD, DIR_BACK, DIR_LEFT, MAX_PULSE_MS, MIN_FORWARD_CM


class FakeLink:
    def __init__(self): self.sent = []
    def send(self, frame, wait=0.5): self.sent.append(dict(frame)); return "{}", "{ok}"
    def motion(self): return [f for f in self.sent if f.get("N") == 2]
    def stops(self): return [f for f in self.sent if f.get("N") == 100]


class Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.link, self.clock, self.events = FakeLink(), Clock(), []
        self.s = Safety(self.link, lambda ev, **f: self.events.append((ev, f)), max_speed=150, clock=self.clock)
        self.s.update_distance(80)

    def test_permitted_move_is_sent(self):          # positive control
        self.assertTrue(self.s.pulse(DIR_FWD, 120, 400))
        m = self.link.motion(); self.assertEqual(len(m), 1)
        self.assertEqual((m[0]["D1"], m[0]["D2"], m[0]["T"]), (3, 120, 400))

    def test_pulse_length_and_speed_are_capped(self):
        self.s.pulse(DIR_FWD, 999, 5000)
        m = self.link.motion()[0]
        self.assertEqual(m["T"], MAX_PULSE_MS); self.assertEqual(m["D2"], 150)

    def test_forward_vetoed_when_too_close(self):
        self.s.update_distance(MIN_FORWARD_CM - 1)
        self.assertFalse(self.s.pulse(DIR_FWD, 120, 400))
        self.assertEqual(self.link.motion(), [])
        self.assertTrue(self.s.pulse(DIR_BACK, 120, 400))    # backing away is allowed

    def test_stale_sensor_refuses_and_stops(self):
        self.assertTrue(self.s.pulse(DIR_FWD, 120, 400))
        self.clock.t += 1.5                                    # no new reading
        self.assertTrue(self.s.stale())
        self.assertFalse(self.s.pulse(DIR_FWD, 120, 400))
        self.assertEqual(len(self.link.motion()), 1)
        self.s.moving_until = self.clock.t + 0.3               # pretend still moving
        self.assertTrue(self.s.watchdog())
        self.assertEqual(len(self.link.stops()), 1)

    def test_side_readings_keep_feed_alive_but_not_forward_veto(self):
        self.clock.t += 3.0
        self.s.update_distance(120, ahead=False)              # a sweep reading at some angle
        self.assertFalse(self.s.stale())
        self.assertFalse(self.s.pulse(DIR_FWD, 120, 400))    # ahead reading is 3 s old: refused
        self.assertTrue(self.s.pulse(DIR_LEFT, 120, 400))     # turning is fine
        self.s.update_distance(80)
        self.assertTrue(self.s.pulse(DIR_FWD, 120, 400))

    def test_watchdog_quiet_when_fresh(self):
        self.s.pulse(DIR_FWD, 120, 400)
        self.assertFalse(self.s.watchdog())
        self.assertEqual(self.link.stops(), [])

    def test_estop_latches(self):
        self.s.raise_estop("test")
        self.assertEqual(len(self.link.stops()), 1)
        for d in (DIR_FWD, DIR_BACK, DIR_LEFT):
            self.assertFalse(self.s.pulse(d, 120, 400))
        self.assertEqual(self.link.motion(), [])
        self.assertIn(("estop", {"reason": "test"}), self.events)

    def test_dry_run_sends_no_motion(self):
        s = Safety(self.link, lambda ev, **f: None, dry_run=True, clock=self.clock); s.update_distance(80)
        self.assertTrue(s.pulse(DIR_FWD, 120, 400))
        self.assertEqual(self.link.motion(), [])


if __name__ == "__main__":
    unittest.main()
