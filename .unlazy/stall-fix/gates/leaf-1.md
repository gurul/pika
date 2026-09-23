# Gates: planner repair

OWNS: tools/active_roam.py, tools/tests/test_active_stall.py, tools/tests/test_active_roam.py, scripts/check_active_stall.py, README.md, HANDOVER.md, GATES-active-roam.md

- [x] L1: Stall regression and search recovery checks pass.
  CHECK: .venv/bin/python scripts/check_active_stall.py --replay build/runs/active-20260922-160415/log.jsonl
  EXPECT: active stall verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output=Ran 7 tests in 0.047s | OK
