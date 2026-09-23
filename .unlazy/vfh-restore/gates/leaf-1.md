# Gates: controller integration
OWNS: tools/stream_roam.py, tools/active_roam.py, README.md, HANDOVER.md, todo.md

- [ ] G1: Existing baseline regressions pass and new entrypoint runs help without a device.
  CHECK: .venv/bin/python tools/stream_roam.py --help && .venv/bin/python scripts/check_active_roam.py planner
  EXPECT: active planner verification passed
  EVIDENCE: pending
