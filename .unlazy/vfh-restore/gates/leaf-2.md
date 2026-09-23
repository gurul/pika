# Gates: independent behavior verification
OWNS: tools/tests/test_stream_roam.py, scripts/check_stream_roam.py, build/vfh-restore-simulation/**

- [ ] G1: Behavioral and fault tests pass.
  CHECK: .venv/bin/python scripts/check_stream_roam.py tests
  EXPECT: stream roam tests passed
  EVIDENCE: pending

- [ ] G2: Geometry and stiction simulations pass.
  CHECK: .venv/bin/python scripts/check_stream_roam.py simulation
  EXPECT: stream roam simulation passed
  EVIDENCE: pending
