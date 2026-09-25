"""Person follow: bearing geometry, target choice, lost-person policy, the
bearing loop, and Apple Vision on a real photo (tools/tests/fixtures,
CC0 from Wikimedia Commons: "Young people walking in the street, Hungary 2011")."""
import os, sys, threading, time, unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import follow  # noqa: E402
from follow import (Avoider, TURN_DEG, bearing_deg, choose, frame_age_ms, hint_for, jumped, pick_at,  # noqa: E402
                    range_cm, wrap, MATCH, MATCH_NEAR)

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures", "people-cc0.jpg")


class Geometry(unittest.TestCase):
    def test_bearing_sign(self):
        self.assertAlmostEqual(bearing_deg((0.4, 0.2, 0.2, 0.6)), 0.0)
        self.assertGreater(bearing_deg((0.0, 0.2, 0.2, 0.6)), 0)        # left half of the image is left
        self.assertLess(bearing_deg((0.7, 0.2, 0.2, 0.6)), 0)
        self.assertAlmostEqual(bearing_deg((0.0, 0, 0.0, 0), hfov=60), 30.0)
        self.assertAlmostEqual(bearing_deg((0.0, 0.2, 0.2, 0.6), mirror=True), -bearing_deg((0.0, 0.2, 0.2, 0.6)))

    def test_pick_at(self):
        big, small = (0.1, 0.1, 0.6, 0.8), (0.3, 0.3, 0.1, 0.2)
        self.assertEqual(pick_at([big, small], 0.35, 0.4), small)        # overlap: the smaller one
        self.assertEqual(pick_at([big, small], 0.15, 0.15), big)
        self.assertEqual(pick_at([small], 0.45, 0.4), small)             # just outside, still near
        self.assertIsNone(pick_at([small], 0.9, 0.9))
        self.assertIsNone(pick_at([], 0.5, 0.5))


class Choice(unittest.TestCase):
    last = (0.4, 0.2, 0.2, 0.6)

    def test_rejects_a_stranger(self):
        self.assertIsNone(choose([((0.1, 0.2, 0.2, 0.6), MATCH + 0.05)], self.last))

    def test_continuity_buys_slack(self):
        near = (0.42, 0.2, 0.2, 0.6)
        self.assertEqual(choose([(near, MATCH + 0.05)], self.last), 0)
        self.assertIsNone(choose([(near, MATCH_NEAR + 0.01)], self.last))

    def test_best_match_wins(self):
        cands = [((0.1, 0.2, 0.2, 0.6), 0.30), ((0.7, 0.2, 0.2, 0.6), 0.20)]
        self.assertEqual(choose(cands, None), 1)

    def test_nearby_beats_slightly_closer_look_far_away(self):
        cands = [((0.8, 0.2, 0.15, 0.6), 0.25), ((0.41, 0.2, 0.2, 0.6), 0.32)]
        self.assertEqual(choose(cands, self.last), 1)


class LostPolicy(unittest.TestCase):
    def test_never_seen(self):
        self.assertEqual(hint_for(None, 0.0), (0, 0))

    def test_short_miss_holds(self):
        self.assertEqual(hint_for(4.4, 0.1), (4, 1))

    def test_left_frame_sideways_searches_that_way_in_pulses(self):
        turn, look = follow.HOLD_S + 0.1, follow.HOLD_S + follow.SEARCH_TURN_S + 0.1
        self.assertEqual(hint_for(25.0, turn), (30, 2))
        self.assertEqual(hint_for(-25.0, turn), (-30, 2))
        self.assertEqual(hint_for(25.0, look), (30, 0))          # still, so the camera can look
        cycle = follow.SEARCH_TURN_S + follow.SEARCH_LOOK_S
        self.assertEqual(hint_for(25.0, turn + cycle), (30, 2))

    def test_lost_in_the_middle_stops(self):
        self.assertEqual(hint_for(3.0, 1.0), (0, 0))

    def test_search_gives_up(self):
        self.assertEqual(hint_for(25.0, follow.SEARCH_S + 0.1), (0, 0))


class FakeStream:
    def __init__(self):
        self.count, self.error, self.at = 0, None, time.monotonic()
        self.cond = threading.Condition()

    def push(self):
        with self.cond:
            self.count += 1
            self.at = time.monotonic()
            self.cond.notify_all()

    def wait(self, after, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.count > after, timeout)
            return (self.count, b"jpeg") if self.count > after else (after, None)


class CameraRange(unittest.TestCase):
    def test_range_from_box_height(self):
        self.assertAlmostEqual(range_cm((0, 0, 0.1, 0.35)), 100.0)
        self.assertAlmostEqual(range_cm((0, 0, 0.1, 0.28)), 125.0)      # measured 129 cm by sonar
        self.assertEqual(range_cm((0, 0, 1, 1.0)), 35.0)
        self.assertEqual(range_cm((0, 0, 0.01, 0.01)), 400.0)            # clamped


class Avoiding(unittest.TestCase):
    """The sonar is only for something close ahead: go around it."""
    ahead = {"bearing": 0.0, "range": 120.0, "abs": 10.0, "fresh": True}

    def test_far_or_single_pings_are_ignored(self):
        a = Avoider()
        self.assertEqual(a.step(0, (120, 90), self.ahead, 0.0), "follow")
        self.assertEqual(a.step(0, (0, 20), self.ahead, 0.0), "follow")      # no-echo breaks agreement
        self.assertEqual(a.step(0, (80, 20), self.ahead, 0.0), "follow")     # one close ping: stray

    def test_the_person_close_ahead_is_not_an_obstacle(self):
        a = Avoider()
        me = {"bearing": 2.0, "range": 30.0, "abs": 0.0, "fresh": True}
        self.assertEqual(a.step(0, (25, 28), me, 0.0), "follow")

    def test_obstacle_goes_around_on_the_persons_side_then_rejoins(self):
        a = Avoider()
        left = {"bearing": 8.0, "range": 120.0, "abs": 18.0, "fresh": True}
        self.assertEqual(a.step(0.0, (22, 24), left, 10.0), "detour")
        self.assertEqual(a.target, 18.0 + TURN_DEG)
        self.assertEqual(a.drive_range, follow.DRIVE_RANGE_CM)
        self.assertEqual(a.step(0.5, (22, 24), dict(left, fresh=False), 40.0), "detour")   # still blocked
        self.assertEqual(a.step(0.6, (90, 120), dict(left, fresh=False), 60.0), "detour")  # clear, keep going
        self.assertEqual(a.step(0.6 + follow.DETOUR_S + 0.01, (90, 120), dict(left, fresh=False), 68.0), "rejoin")
        self.assertEqual(a.target, 18.0)                                                   # back to where they were
        self.assertEqual(a.step(2.0, (90, 120), left, 30.0), "follow")                     # seen again
        self.assertIsNone(a.target)

    def test_blocked_again_soon_tries_the_other_side(self):
        a = Avoider()
        ahead = dict(self.ahead, fresh=False)
        a.step(0.0, (20, 20), ahead, 0.0)
        side = a.side
        a.reset()
        a.step(2.0, (20, 20), ahead, 0.0)
        self.assertEqual(a.side, -side)

    def test_gives_up_when_it_never_clears(self):
        a = Avoider()
        a.step(0.0, (20, 20), dict(self.ahead, abs=None), 0.0)
        self.assertEqual(a.step(follow.GIVE_UP_S + 0.1, (20, 20), dict(self.ahead, abs=None), 0.0), "follow")

    def test_needs_the_heading(self):
        self.assertEqual(Avoider().step(0, (20, 20), self.ahead, None), "follow")

    def test_wrap(self):
        self.assertEqual(wrap(190), -170)
        self.assertEqual(wrap(-190), 170)


class FrameAge(unittest.TestCase):
    def test_age_adds_transport_and_clamps(self):
        self.assertEqual(frame_age_ms(10.0, 10.2), 200 + follow.TRANSPORT_MS)
        self.assertEqual(frame_age_ms(10.0, 9.0), 0)
        self.assertEqual(frame_age_ms(0.0, 100.0), 2000)


class Jumps(unittest.TestCase):
    """A match far from the track needs a second frame to agree (buddy's follow.py lesson)."""

    class Vis:
        def __init__(self, boxes):
            self.frames = list(boxes)
            self.pool = lambda: __import__("contextlib").nullcontext()
        def image(self, jpeg): return object()
        def people(self, cg): return [self.frames.pop(0)]
        def prints(self, cg, boxes): return [[1.0, 0.0] for _ in boxes]

    def track(self, boxes):
        t = follow.PersonTracker(self.Vis(boxes))
        t.bank, t.last_box = [[1.0, 0.0]], boxes[0]
        return [t.update(b"j") for _ in boxes]

    def test_small_moves_track_every_frame(self):
        boxes = [(0.40, 0.2, 0.2, 0.6), (0.43, 0.2, 0.2, 0.6), (0.46, 0.2, 0.2, 0.6)]
        self.assertTrue(all(self.track(boxes)))

    def test_a_jump_waits_for_a_second_frame(self):
        a, far = (0.40, 0.2, 0.2, 0.6), (0.05, 0.2, 0.2, 0.6)
        got = self.track([a, far, far, far])
        self.assertIsNotNone(got[0]); self.assertIsNone(got[1]); self.assertEqual(got[2][0], far)
        self.assertTrue(jumped(a, far)); self.assertFalse(jumped(a, (0.45, 0.2, 0.2, 0.6)))

    def test_a_one_frame_blip_is_ignored(self):
        a, far = (0.40, 0.2, 0.2, 0.6), (0.05, 0.2, 0.2, 0.6)
        got = self.track([a, far, a])
        self.assertIsNone(got[1]); self.assertEqual(got[2][0], a)


class FakeTracker:
    """Sees the person at a fixed box until told they are gone."""
    def __init__(self):
        self.box, self.boxes, self.locked = (0.0, 0.2, 0.2, 0.6), [], True

    def update(self, jpeg):
        self.boxes = [self.box] if self.box else []
        return (self.box, 0.1) if self.box else None


class Loop(unittest.TestCase):
    def setUp(self):
        self.sent, self.logs = [], []
        self.stream = FakeStream()
        orig = follow.vision_available
        follow.vision_available = lambda: (False, "test")      # no Vision, no loop thread
        try:
            self.f = follow.Follower(self.stream, self.sent.append, front_raw=lambda: 95, log=self.logs.append)
        finally:
            follow.vision_available = orig
        self.f.tracker, self.f.unavailable = FakeTracker(), None
        self.stop = False
        self.pump = threading.Thread(target=self._pump, daemon=True)

    def _pump(self):
        while not self.stop:
            self.stream.push(); time.sleep(0.03)

    def tearDown(self):
        self.stop = True
        self.f.close()

    def run_loop(self, seconds):
        threading.Thread(target=self.f._run, daemon=True).start()
        self.pump.start()
        time.sleep(seconds)

    def test_start_sends_mode_with_front_then_bearings(self):
        self.f.start()
        self.run_loop(0.8)
        self.assertEqual(self.sent[:2], [{"N": 25, "D1": 100, "D2": 1}, {"N": 101, "D1": 3, "D2": 95}])
        hints = [m for m in self.sent if m["N"] == 29]
        self.assertGreaterEqual(len(hints), 4)
        self.assertTrue(all(m["D2"] == 1 and m["D1"] == 25 for m in hints), hints)   # box centre 0.1: (0.5 - 0.1) * 62
        self.assertTrue(all(m["D4"] == 58 for m in hints), hints)                    # box height 0.6: 35 / 0.6
        self.assertTrue(all(follow.TRANSPORT_MS <= m["D3"] < follow.TRANSPORT_MS + 200 for m in hints), hints)
        # at most SEND_HZ per second
        self.assertLessEqual(len(hints), follow.SEND_HZ * 0.8 + 2)

    def test_release_stops_bearings_without_a_stop_frame(self):
        self.f.start(); self.run_loop(0.3)
        self.f.release(); n = len(self.sent); time.sleep(0.4)
        self.assertEqual(len(self.sent), n)
        self.assertNotIn({"N": 100}, self.sent)

    def test_quiet_page_stops_the_car(self):
        self.f.start(); self.run_loop(0.2)
        self.f.touched -= follow.PAGE_S + 1
        time.sleep(0.3)
        self.assertFalse(self.f.on)
        self.assertIn({"N": 100}, self.sent)
        self.assertEqual(self.logs, ["follow: controller page went quiet, stopping"])

    def test_obstacle_in_the_way_sends_a_detour(self):
        car = {"dist": 120, "yaw": 0.0}
        self.f.telemetry = lambda: {"mono": time.monotonic(), "dist": car["dist"], "yaw": car["yaw"],
                                    "pan": 90, "settled": True}
        self.f.tracker.box = (0.35, 0.2, 0.2, 0.6)          # a little left: bearing about +3
        self.f.start(); self.run_loop(0.4)
        self.assertEqual([m for m in self.sent if m["N"] == 29][-1]["D4"], 58)
        car["dist"] = 20                                      # something 20 cm ahead
        time.sleep(0.6)
        last = [m for m in self.sent if m["N"] == 29][-1]
        self.assertEqual(self.f.avoider.state, "detour")
        self.assertEqual((last["D2"], last["D4"]), (1, follow.DRIVE_RANGE_CM))
        self.assertGreater(last["D1"], 40)                    # steering off to the left, around it

    def test_person_leaves_left_then_car_searches_left(self):
        self.f.start(); self.run_loop(0.3)
        self.f.tracker.box = None
        time.sleep(follow.HOLD_S + 0.2)
        last = [m for m in self.sent if m["N"] == 29][-1]
        self.assertEqual((last["D1"], last["D2"]), (30, 2))


@unittest.skipUnless(follow.vision_available()[0], "Apple Vision (pyobjc) not installed")
class AppleVision(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vis = follow.Vis()
        cls.jpeg = open(FIXTURE, "rb").read()
        cls.cg = cls.vis.image(cls.jpeg)
        cls.boxes = cls.vis.people(cls.cg)

    def test_finds_the_people(self):
        self.assertEqual(len(self.boxes), 3)
        for x, y, w, h in self.boxes:
            self.assertTrue(0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1)

    def test_boxes_are_top_left_based(self):
        # the small far-off person stands with their head lower in the frame than the two near ones
        small = min(self.boxes, key=lambda b: b[2] * b[3])
        self.assertGreater(small[1], max(b[1] for b in self.boxes if b != small))

    def test_appearance_separates_people(self):
        fps = self.vis.prints(self.cg, self.boxes)
        for i in range(3):
            for j in range(3):
                if i != j:
                    self.assertGreater(follow.distance(fps[i], fps[j]), MATCH_NEAR)
            b = self.boxes[i]
            nudged = (b[0] + 0.02 * b[2], b[1] + 0.03 * b[3], b[2] * 0.95, b[3] * 0.95)
            self.assertLess(follow.distance(fps[i], self.vis.prints(self.cg, [nudged])[0]), MATCH)

    def test_click_locks_then_tracks_the_same_person(self):
        t = follow.PersonTracker(self.vis)
        for target in self.boxes:
            cx, cy = target[0] + target[2] / 2, target[1] + target[3] / 2
            got = t.select(self.jpeg, cx, cy)
            self.assertEqual(got, pick_at(self.boxes, cx, cy))
            box, dist = t.update(self.jpeg)
            self.assertEqual(box, got)
            self.assertLess(dist, 0.05)


if __name__ == "__main__":
    unittest.main()
