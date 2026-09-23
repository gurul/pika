# Gates: room map

OWNS: tools/explore.py, tools/mapping.py, tools/sonar.py, tools/tests/**, scripts/**, calibration.json, GATES.md, README.md

Scope: the car explores the room on its own and produces a digital occupancy map of it, with a safety layer that owns the motors, sonar-sweep localization, frontier exploration, and run logs with frames

- [ ] G1: the mapping core is correct on a simulated room: the inverse sonar model marks walls occupied and the interior free, the scan matcher recovers a perturbed pose to within 5 cm and 5 degrees, and the frontier planner picks a reachable unknown boundary
  CHECK: .venv/bin/python scripts/check_mapping.py
  EXPECT: mapping verification passed
  EVIDENCE: pending

- [ ] G2: the safety layer refuses unsafe motion and permits safe motion (unit tests with a fake link: pulse length capped, forward vetoed below the minimum distance, stale sensors force a stop, e-stop latches, a permitted move passes as the positive control)
  CHECK: .venv/bin/python scripts/check_safety.py
  EXPECT: safety verification passed
  EVIDENCE: pending

- [ ] G2b: a 25 s dry run against the live car sweeps, plans, would have moved, captures frames, and sends no motion frame
  CHECK: .venv/bin/python tools/explore.py --dry-run --duration 25 --no-viewer --run-dir build/runs/dry && .venv/bin/python scripts/check_run.py build/runs/dry --mode dry
  EXPECT: run verification passed
  EVIDENCE: pending

- [ ] G3: a live sonar sweep returns 17 readings in under 20 s and a repeat sweep agrees with it (median absolute difference under 15 cm over echoing beams)
  CHECK: .venv/bin/python scripts/check_sweep.py
  EXPECT: sweep verification passed
  EVIDENCE: pending

- [ ] G4: motion is calibrated on the live car: forward cm/s measured against a wall and turn deg/s measured by sweep correlation, both inside sane ranges, saved to calibration.json
  CHECK: .venv/bin/python scripts/check_calibration.py
  EXPECT: calibration verification passed
  EVIDENCE: pending

- [ ] G5: with the sensor feed frozen 24 s into a live run (after the first sweep and first move), the car is sent an explicit stop within 1.5 s of the freeze (0.8 s staleness plus the 0.3 s watchdog tick; the UNO itself halts 450 ms after any pulse) and no motion frame follows
  CHECK: .venv/bin/python tools/explore.py --duration 45 --fault stale-at=24 --run-dir build/runs/stale && .venv/bin/python scripts/check_run.py build/runs/stale --mode stale
  EXPECT: stale verification passed
  EVIDENCE: pending

- [ ] G6: with e-stop raised 24 s into a live run (after the first sweep and first move), the car is sent an explicit stop within 1.5 s and no motion frame follows
  CHECK: .venv/bin/python tools/explore.py --duration 45 --fault estop-at=24 --run-dir build/runs/estop && .venv/bin/python scripts/check_run.py build/runs/estop --mode estop
  EXPECT: estop verification passed
  EVIDENCE: pending

- [ ] G7: a live mapping run of at least 4 minutes completes 12 or more scan-move cycles, writes map.png, map.npy, poses.jsonl and frames, keeps every localization correction inside the search window, and ends with a stop and a summary
  CHECK: .venv/bin/python tools/explore.py --duration 240 --run-dir build/runs/map && .venv/bin/python scripts/check_run.py build/runs/map --mode map
  EXPECT: map verification passed
  EVIDENCE: pending

- [ ] G8: the person watching the G7 run confirms the car did not collide with anything and that map.png shows the room's walls and large objects where they are
  EVIDENCE: pending

- [ ] G9: the README documents the mapping mode, its safety envelope, the calibration steps, the run log format, the map files, and the fault-injection flags
  CHECK: .venv/bin/python scripts/check_readme.py
  EXPECT: readme verification passed
  EVIDENCE: pending

- [ ] G10: on a saved camera frame the vision module returns a room label from the allowed list and well-formed hazard and doorway lists, using the key from the config file
  CHECK: .venv/bin/python scripts/check_vision.py
  EXPECT: vision verification passed
  EVIDENCE: pending

