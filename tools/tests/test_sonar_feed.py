"""Contract tests for real JSONL tailing, projection, freshness and HTTP refresh."""
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest

from tools.sonar_feed import MAX_LINE, MAX_READ, RadarFeed, make_server, parse_sample, project


def tick(stamp=1000, seq=1, pan=90, dist=80, yaw=0, settled=True, accepted=True):
    return {"event": "tick", "accepted": accepted, "left": 80, "right": 100,
            "reason": "drive", "look": pan, "telemetry": {
                "t": stamp, "pan": pan, "dist": dist, "yaw": yaw,
                "seq": seq, "settled": settled}}


class FeedTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.path = self.root / "log.jsonl"
        self.now = 1000.2
        self.feed = RadarFeed(self.root, clock=lambda: self.now)

    def tearDown(self):
        self.tmp.cleanup()

    def append(self, row):
        with self.path.open("a") as stream:
            stream.write(json.dumps(row) + "\n")

    def test_projection_cardinal_and_oblique(self):
        self.assertAlmostEqual(project(90, 120)["x_cm"], 0)
        self.assertAlmostEqual(project(90, 120)["y_cm"], 120)
        self.assertEqual(project(0, 120), {"x_cm": 120, "y_cm": 0})
        self.assertAlmostEqual(project(150, 120)["x_cm"], -60 * math.sqrt(3))
        self.assertAlmostEqual(project(150, 120)["y_cm"], 60)

    def test_only_finite_accepted_angle_tagged_ticks(self):
        self.assertIsNotNone(parse_sample(tick()))
        for changes in ({"accepted": False}, {"event": "command"}):
            bad = tick(); bad.update(changes)
            self.assertIsNone(parse_sample(bad))
        for field, value in (("pan", 0), ("pan", 171), ("dist", -1),
                             ("dist", float("nan")), ("yaw", float("inf")),
                             ("yaw", 10**1000), ("settled", 1), ("seq", True), ("t", 0)):
            bad = tick(); bad["telemetry"][field] = value
            self.assertIsNone(parse_sample(bad), (field, value))

    def test_live_tail_refresh_and_partial_line(self):
        self.assertEqual(self.feed.snapshot()["status"], "missing")
        self.append(tick())
        first = self.feed.snapshot()
        self.assertEqual(first["status"], "live")
        self.assertEqual(first["points"][0]["distance_cm"], 80)
        self.assertEqual(first["decision"]["left"], 80)
        raw = json.dumps(tick(stamp=1000.1, seq=2, dist=76))
        with self.path.open("a") as stream: stream.write(raw[:20])
        self.assertEqual(self.feed.snapshot()["sample_count"], 1)
        with self.path.open("a") as stream: stream.write(raw[20:] + "\n")
        second = self.feed.snapshot()
        self.assertEqual(second["latest"]["distance_cm"], 76)
        self.assertEqual(second["sample_count"], 2)
        self.assertAlmostEqual(second["sample_hz"], 10)

    def test_unknown_revokes_old_ray_and_unsettled_never_plots(self):
        self.append(tick())
        self.assertEqual(len(self.feed.snapshot()["points"]), 1)
        self.append(tick(stamp=1000.1, seq=2, dist=0))
        unknown = self.feed.snapshot()
        self.assertEqual(unknown["status"], "unknown")
        self.assertEqual(unknown["points"], [])
        self.append(tick(stamp=1000.2, seq=3, pan=130, dist=40, settled=False))
        settling = self.feed.snapshot()
        self.assertEqual(settling["status"], "settling")
        self.assertEqual(settling["latest"]["pan"], 130)
        self.assertEqual(settling["points"], [])
        self.append(tick(stamp=1000.3, seq=4, dist=400))
        self.now = 1000.3
        self.assertEqual(self.feed.snapshot()["status"], "unknown")
        self.assertEqual(self.feed.snapshot()["points"], [])

    def test_observe_mode_and_heading_sign_are_preserved(self):
        first = tick(pan=60, yaw=5)
        first.update(observe=True, yaw_left_sign=1)
        self.append(first)
        second = tick(stamp=1000.1, seq=2, pan=120, yaw=25)
        second.update(observe=True, yaw_left_sign=1)
        second["left"] = float("nan")
        self.append(second)
        state = self.feed.snapshot()
        self.assertTrue(state["observe"])
        self.assertEqual(state["points"][0]["angle"], 40)
        self.assertIsNone(state["decision"]["left"])
        json.dumps(state, allow_nan=False)

    def test_rejected_and_duplicate_frames_do_not_refresh(self):
        self.append(tick())
        self.feed.snapshot()
        self.now = 1001
        self.append(tick(stamp=1001, seq=1, dist=12))
        self.append(tick(stamp=1001, seq=2, accepted=False))
        state = self.feed.snapshot()
        self.assertEqual(state["status"], "stale")
        self.assertEqual(state["latest"]["distance_cm"], 80)
        self.assertEqual(state["sample_count"], 1)

    def test_heading_reprojection_history_expiry_and_future_clock(self):
        self.append(tick(pan=60, yaw=5))
        self.append(tick(stamp=1000.1, seq=2, pan=120, yaw=25))
        state = self.feed.snapshot()
        self.assertEqual(state["points"][0]["angle"], 80)
        self.now = 1003
        self.assertEqual(self.feed.snapshot()["points"], [])
        self.assertEqual(self.feed.snapshot()["status"], "stale")
        self.append(tick(stamp=2000, seq=3))
        self.assertEqual(self.feed.snapshot()["status"], "clock_error")

    def test_replaced_log_resets_end_and_samples(self):
        self.append(tick())
        self.append({"event": "end", "error": None})
        self.assertEqual(self.feed.snapshot()["status"], "ended")
        replacement = self.root / "new.jsonl"
        replacement.write_text(json.dumps(tick(seq=42)) + "\n")
        replacement.replace(self.path)
        state = self.feed.snapshot()
        self.assertEqual(state["status"], "live")
        self.assertEqual(state["latest"]["seq"], 42)
        self.assertEqual(state["sample_count"], 1)

    def test_large_history_malformed_and_oversized_lines_are_bounded(self):
        self.path.write_bytes(b"x" * (MAX_READ + 80) + b"\nnot json\n")
        self.append(tick())
        state = self.feed.snapshot()
        self.assertEqual(state["latest"]["distance_cm"], 80)
        self.assertGreater(state["malformed_count"], 0)
        with self.path.open("ab") as stream: stream.write(b"x" * (MAX_LINE + 20))
        self.feed.snapshot()
        self.assertLessEqual(len(self.feed.pending), MAX_LINE)
        with self.path.open("ab") as stream: stream.write(b"tail\n")
        self.append(tick(stamp=1000.1, seq=2, dist=64))
        self.assertEqual(self.feed.snapshot()["latest"]["distance_cm"], 64)

    def test_http_serves_real_log_changes_and_original_page(self):
        self.append(tick())
        server = make_server(self.feed, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        try:
            connection.request("GET", "/")
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            page = response.read().decode()
            self.assertIn("fetch('/api/state'", page)
            self.assertIn("Blank space is unknown", page)
            connection.request("GET", "/api/state")
            self.assertEqual(json.loads(connection.getresponse().read())["latest"]["distance_cm"], 80)
            self.append(tick(stamp=1000.1, seq=2, dist=45))
            connection.request("GET", "/api/state")
            self.assertEqual(json.loads(connection.getresponse().read())["latest"]["distance_cm"], 45)
            connection.request("GET", "/api/state", headers={"Host": "untrusted.invalid"})
            response = connection.getresponse(); response.read()
            self.assertEqual(response.status, 403)
        finally:
            connection.close()
            server.shutdown(); server.server_close(); thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
