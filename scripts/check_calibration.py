#!/usr/bin/env python3
"""G4: calibration.json exists with sane, measured values."""
import json, os, sys
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration.json")
try:
    cal = json.load(open(p))
except (OSError, ValueError) as e:
    sys.exit(f"no calibration.json: {e}. Run tools/sonar.py calibrate-forward and calibrate-turn")
ok = True
def need(key, lo, hi):
    global ok
    v = cal.get(key)
    if v is None or not (lo <= v <= hi):
        print(f"FAIL: {key} = {v}, expected {lo}..{hi}"); ok = False
    else:
        print(f"ok:   {key} = {v}")
need("forward_cm_per_s", 8, 80)      # ELEGOO V4 at speed 120 drives roughly 20..50 cm/s
need("turn_deg_per_s", 40, 400)      # spin in place at speed 140
need("forward_speed", 60, 255); need("turn_speed", 60, 255)
print("calibration verification passed" if ok else "calibration missing or out of range")
sys.exit(0 if ok else 1)
