#!/usr/bin/env python3
"""U5: gyro heading. Drift under 2 deg over 20 s at rest after N=26 recalibration; two identical
500 ms left spins agree within 15% and each exceeds 30 deg; the turn rate they imply is saved
to calibration.json (replacing the sweep-correlation estimate). Over Wi-Fi, car on the floor."""
import json, os, re, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from car import Car
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")
cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
c = Car(os.environ.get("CAR_HOST", "elegoo-car.local"))
def yaw():
    for _ in range(3):
        line, r = c.send({"N": 24}, wait=0.6); m = re.search(r"_(-?\d+)\}", r or "")
        if m: return int(m.group(1)) / 10.0
ok = True
def check(cond, m):
    global ok; print(("ok:   " if cond else "FAIL: ") + m); ok = ok and cond
print("recalibrate:", c.send({"N": 26}, wait=3.0)[1])
y0 = yaw(); time.sleep(20); y1 = yaw()
check(abs(y1 - y0) < 2.0, f"drift at rest over 20 s: {y1 - y0:+.1f} deg")
spins = []
for i in range(2):
    a = yaw(); c.send({"N": 2, "D1": 1, "D2": 140, "T": 450}, wait=0.15); time.sleep(0.5); c.send({"N": 2, "D1": 1, "D2": 140, "T": 50}, wait=0.15); time.sleep(0.9)
    spins.append(yaw() - a); time.sleep(0.5)
c.send({"N": 100}, wait=0.4); c.close()
mag = [abs(x) for x in spins]
check(min(mag) > 30, f"two 500 ms left spins: {spins[0]:+.1f}, {spins[1]:+.1f} deg")
check(abs(mag[0] - mag[1]) <= 0.15 * max(mag), f"spins agree within 15%: {abs(mag[0] - mag[1]):.1f} deg apart")
check(spins[0] * spins[1] > 0, "same sign both times")
if ok:
    dps = round(sum(mag) / len(mag) / 0.5, 1)
    cal.update({"turn_deg_per_s": dps, "turn_speed": 140, "yaw_left_sign": 1 if spins[0] > 0 else -1, "turn_rate_source": "gyro N=24"})
    json.dump(cal, open(CAL, "w"), indent=2); print(f"turn rate from gyro: {dps} deg/s at speed 140, saved")
print("uno yaw verification passed" if ok else "uno yaw check failed"); sys.exit(0 if ok else 1)
