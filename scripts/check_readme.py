#!/usr/bin/env python3
"""G9: README covers the mapping mode."""
import os, sys
t = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "README.md")).read()
need = ["## Map the room", "tools/explore.py", "safety", "calibrate-forward", "calibrate-turn", "log.jsonl", "map.png", "map.npy", "poses.jsonl", "--fault", "stale-at", "estop-at", "--dry-run"]
if "--uno" in sys.argv:
    need = ["## UNO firmware", "N=24", "N=25", "400 cm", "uno-factory", "avrdude", "avr-gcc@9", "restore", "deadman"]
missing = [n for n in need if n not in t]
for n in need: print(("ok:   " if n not in missing else "FAIL: ") + n)
if missing: sys.exit(1)
print("readme verification passed")
