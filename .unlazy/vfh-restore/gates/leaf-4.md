# Gates: live sonar display
OWNS: tools/sonar_feed.py, web/sonar-feed.html, tools/tests/test_sonar_feed.py

- [ ] G1: Feed correctly projects actual angle-tagged telemetry, rejects unsettled/unknown as ranges, and labels stale data.
  CHECK: .venv/bin/python -m unittest discover -s tools/tests -p test_sonar_feed.py -v
  EXPECT: OK
  EVIDENCE: pending

- [ ] G2: Live local UI verified with real telemetry; no independent car socket, and remains usable while driving.
  EVIDENCE: pending
