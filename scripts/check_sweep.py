#!/usr/bin/env python3
"""G3: two live sonar sweeps, timing and repeatability."""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from car import Car
from sonar import Sonar, MAX_RANGE, bar_chart

car = Car(os.environ.get("CAR_HOST", "elegoo-car.local"))
sonar = Sonar(car)
try:
    t0 = time.time(); a = sonar.sweep(step=10); ta = time.time() - t0
    t0 = time.time(); b = sonar.sweep(step=10); tb = time.time() - t0
    sonar.center()
finally:
    car.send({"N": 100}, wait=0.3); car.close()
bar_chart(a); print(f"sweep A: {len(a)} readings in {ta:.1f} s"); print(f"sweep B: {len(b)} readings in {tb:.1f} s")
ok = True
if len(a) != 17 or len(b) != 17: print("FAIL: expected 17 readings"); ok = False
if max(ta, tb) >= 20: print("FAIL: sweep slower than 20 s"); ok = False
missing = sum(1 for _, r in a + b if r is None)
if missing > 2: print(f"FAIL: {missing} failed echoes"); ok = False
pairs = [(ra, rb) for (_, ra), (_, rb) in zip(a, b) if ra and rb and (ra < MAX_RANGE or rb < MAX_RANGE)]
if pairs:
    diffs = sorted(abs(ra - rb) for ra, rb in pairs); med = diffs[len(diffs) // 2]
    print(f"echoing beams: {len(pairs)}, median |A-B| = {med} cm")
    if med >= 15: print("FAIL: sweeps disagree"); ok = False
else:
    print("FAIL: no echoing beams at all; put the car within 150 cm of something"); ok = False
if ok: print("sweep verification passed")
else: sys.exit(1)
