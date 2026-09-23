# Gates: restored VFH stream controller

- [x] G1: Known clear motion and arcs survive scans without repeated sub-stiction restarts; immediate stop protections and invalid telemetry rejection hold.
  CHECK: .venv/bin/python scripts/check_stream_roam.py tests
  EXPECT: stream roam tests passed
  EVIDENCE: 2026-09-22 18:2x re-run: 'OK' + 'stream roam tests passed'.

- [ ] G2: Independent geometry simulation with wheel stiction demonstrates progress without contacts, including an offset obstacle and noisy readings.
  CHECK: .venv/bin/python scripts/check_stream_roam.py simulation
  EXPECT: stream roam simulation passed
  EVIDENCE: UNMET after review. Gates tightened (inner wheel below stiction, stops <= 6). Before fixes: inner-weak 41-64 ticks, 10-15 stops. After: inner-weak 0 in every run, 0 contacts. Over 10 seeds: offset 10/10 passed, 6 stops; hallway 10/10, 7 stops (over the 6 limit); noisy 6/10 passed the obstacle, stops median 6.5, max 12. The remaining stops are turns into no-echo space, which is unknown by design.

- [x] G3: Existing transport, safety, active baseline and stall regressions pass.
  CHECK: .venv/bin/python scripts/check_active_roam.py planner && .venv/bin/python scripts/check_active_roam.py integration && .venv/bin/python scripts/check_active_stall.py --replay build/runs/active-20260922-160415/log.jsonl
  EXPECT: active stall verification passed
  EVIDENCE: 2026-09-22 re-run: planner OK, integration OK, stall replay 162 samples -> angles 10..170; 'active stall verification passed'.

- [ ] G4: Independent review findings resolved, entrypoint documented, and any physical retest is bounded with stop evidence and honest limitations.
  EVIDENCE: Review build/vfh-restore-review.md. Fixed: #1 no glance while |steer|>.3 or turning fast; #2 geometric ray aging (VfhMemory); #3 launch only with the sonar settled forward; #4 wheel at or above the floor, or 0; #6 steering kept through stops; #7 tests added (78 OK); #8 pan rounded to a sector. Not changed: #9 fading stale returns (labelled). README documents the entrypoint. Physical retest pending: the car is off.

- [x] G5: Live sonar feed displays the same controller's real readings with correct angles, unknown and stale states; verified in the browser.
  CHECK: .venv/bin/python -m unittest discover -s tools/tests -p test_sonar_feed.py -v
  EXPECT: OK
  EVIDENCE: 2026-09-22: 10 tests OK. Browser (Chrome) replay of real log active-20260922-175119 through sonar_feed: LIVE state, 8.6 Hz, red returns at 130/150/90 deg, no console errors; after stopping replay the page showed STALE DATA / 'No fresh sonar data' with no points. Replay, not a live car run.
