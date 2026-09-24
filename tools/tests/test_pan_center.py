"""The saved sonar front offsets outgoing pan angles and maps telemetry back."""
import json
import socket
import threading
import time
import unittest

from tools.car import Car
from tools.controller import Link


class FakeCar:
    """Loopback TCP peer: records frames, acks each, can push telemetry."""

    def __init__(self):
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.frames = []
        self.conn = None
        self.connected = threading.Event()
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        self.conn, _ = self.srv.accept()
        self.connected.set()
        buf = ""
        while True:
            try:
                chunk = self.conn.recv(1024).decode()
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while "}" in buf:
                frame, buf = buf.split("}", 1)
                obj = json.loads(frame + "}")
                self.frames.append(obj)
                self.conn.sendall(("{" + obj["H"] + "_ok}").encode())

    def push(self, text):
        self.connected.wait(2)
        self.conn.sendall(text.encode())

    def close(self):
        if self.conn:
            self.conn.close()
        self.srv.close()


class PanCenterTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeCar()
        self.car = Car("127.0.0.1", self.fake.port, timeout=2, pan_center=95)

    def tearDown(self):
        self.car.close()
        self.fake.close()

    def test_outgoing_angles_add_the_offset(self):
        self.car.send({"N": 5, "D1": 1, "D2": 90}, wait=1)
        self.car.send({"N": 28, "D1": 60}, wait=1)
        self.assertEqual(self.fake.frames[0]["D2"], 95)
        self.assertEqual(self.fake.frames[1]["D1"], 65)

    def test_clamps_to_servo_range(self):
        self.car.send({"N": 28, "D1": 170}, wait=1)
        self.assertEqual(self.fake.frames[0]["D1"], 170)

    def test_raw_frames_skip_the_offset_and_drop_the_flag(self):
        self.car.send({"N": 5, "D1": 1, "D2": 90, "raw": True}, wait=1)
        self.assertEqual(self.fake.frames[0]["D2"], 90)
        self.assertNotIn("raw", self.fake.frames[0])

    def test_other_frames_are_untouched(self):
        self.car.send({"N": 4, "D1": 100, "D2": 100}, wait=1)
        self.assertEqual((self.fake.frames[0]["D1"], self.fake.frames[0]["D2"]), (100, 100))

    def telemetry_pan(self, raw):
        before = self.car.telemetry_count
        self.fake.push(f"{{T_80_0_500_500_500_1_7400_{raw}_1_100}}")
        deadline = time.monotonic() + 1
        while self.car.telemetry_count == before and time.monotonic() < deadline:
            time.sleep(0.01)
        return self.car.telemetry["pan"]

    def test_telemetry_reports_the_logical_angle_even_when_clamped(self):
        self.car.send({"N": 28, "D1": 170}, wait=1)   # raw 170, clamped
        self.assertEqual(self.telemetry_pan(170), 170)
        self.car.send({"N": 28, "D1": 90}, wait=1)
        self.assertEqual(self.telemetry_pan(95), 90)

    def test_unsent_telemetry_angles_subtract_the_offset(self):
        self.assertEqual(self.telemetry_pan(100), 95)

    def test_changing_the_center_applies_immediately(self):
        self.car.set_pan_center(85)
        self.car.send({"N": 28, "D1": 90}, wait=1)
        self.assertEqual(self.fake.frames[0]["D1"], 85)


class ControllerConnectTests(unittest.TestCase):
    def test_connecting_points_the_sonar_at_the_saved_front(self):
        fake = FakeCar()
        link = Link("127.0.0.1", fake.port)
        link.pan_center = 95
        try:
            link.send({"N": 21, "D1": 2}, 1)
            self.assertEqual([(f["N"], f.get("D2")) for f in fake.frames], [(5, 95), (21, None)])
        finally:
            link.car.close()
            fake.close()


if __name__ == "__main__":
    unittest.main()
