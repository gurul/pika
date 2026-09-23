#!/usr/bin/env python3
"""G10: vision module on a saved frame. Uses the key from the config file."""
import glob, json, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from vision import Vision, ROOMS, HAZARDS

frames = sorted(glob.glob("build/runs/*/frames/*.jpg")) or sorted(glob.glob("build/runs/*/frame.jpg"))
if not frames:
    sys.exit("no saved frame under build/runs/*/frames; run a dry run first")
frame = frames[-1]
events = []
v = Vision(log=lambda *a, **k: events.append(k))
if not v.enabled:
    sys.exit(f"vision disabled: {events}")
print(f"model {v.model}, frame {frame}")
t0 = time.time(); out = v.describe(open(frame, "rb").read()); dt = time.time() - t0
print(json.dumps(out, indent=1)); print(f"{dt:.1f} s")
ok = True
def check(c, m):
    global ok
    print(("ok:   " if c else "FAIL: ") + m); ok = ok and c
check(out is not None, "model returned parseable JSON")
if out:
    check(out["room"] in ROOMS, f"room label in the allowed list: {out['room']}")
    check(all(h["type"] in HAZARDS and h.get("where") in ("ahead", "left", "right") for h in out["hazards"]), f"hazards well formed: {out['hazards']}")
    check(all(d.get("where") in ("ahead", "left", "right") and isinstance(d.get("open"), bool) for d in out["doorways"]), f"doorways well formed: {out['doorways']}")
    check(dt < 30, f"answered in {dt:.1f} s")
print("vision verification passed" if ok else "vision check failed"); sys.exit(0 if ok else 1)
