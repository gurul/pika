# Gates: person follow

OWNS: firmware/uno_v4_mod/FollowDrive.h, firmware/uno_v4_mod/ApplicationFunctionSet_xxx0.cpp, tools/follow.py, tools/controller.py, tools/controller.html, tools/tests/test_follow.py, tools/tests/fixtures/**, scripts/check_follow.py, README.md, todo.md, HANDOVER.md, GATES-follow.md

Scope: Pick a person by clicking them in the controller's video; the Mac tracks them with Apple Vision and streams bearing and camera range; the UNO holds the gap on the camera range, steers on its gyro, brakes and backs off on its sonar, and stops when bearings stop; the Mac steers a detour around obstacles the sonar finds.

- [x] F1: The UNO follow drive behaves as specified in 30 host checks (camera-range gap band, back-off only when ahead, sonar reflex needing two agreeing pings, turn in place, search, stale bearing, lifted, gyro hold, frame-age latency, millis wrap), and v5 builds within the UNO's flash on AVR core 1.8.3.
  CHECK: .venv/bin/python scripts/check_follow.py firmware
  EXPECT: follow firmware verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/gurucharan/Documents/personal/car; path=574d30059456/19 entries; output=follow wired into the sketch: True | follow firmware verification passed

- [x] F2: Tracker logic and Apple Vision: bearing geometry, camera range, click picking, stranger rejection, jump confirmation, lost-person policy, obstacle detour (sides, rejoin, give up), bearing loop (rate, frame age, range, detour, release, quiet-page stop), and on a real photo: 3 people found, prints separate people and match the same person, click then track.
  CHECK: .venv/bin/python scripts/check_follow.py tracker
  EXPECT: follow tracker verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/gurucharan/Documents/personal/car; path=574d30059456/19 entries; output=Apple Vision tests ran: True | follow tracker verification passed

- [x] F3: The controller end to end against a fake car and camera: relays /stream, a click locks and follows, sends follow mode with the saved front, streams seen bearings of about 10 deg right with frame age, bearings keep flowing (gap < 0.35 s) while slow distance polls hold the link, the line-sensor query is refused while following, stop hands the car back, a quiet page stops the car.
  CHECK: .venv/bin/python scripts/check_follow.py controller
  EXPECT: follow controller verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/gurucharan/Documents/personal/car; path=574d30059456/19 entries; output=ok   follow stops itself when the page goes quiet (stop sent, no bearings after it) | follow controller verification passed

- [x] F4: No regressions: all unit tests, README checks, and the active-roam firmware gate (host ActivePan + UNO build).
  CHECK: .venv/bin/python -m unittest discover -s tools/tests -t . && .venv/bin/python scripts/check_readme.py && .venv/bin/python scripts/check_readme.py --uno && .venv/bin/python scripts/check_active_roam.py firmware
  EXPECT: active firmware verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/gurucharan/Documents/personal/car; path=574d30059456/19 entries; output=Ran 119 tests in 4.639s | OK

- [x] F5: The page works in Chrome: Follow shows the people to pick, a click follows with an aligned outline, a second click switches, Space stops.
  EVIDENCE: Manual, 2026-09-25, Chrome against controller.py with the fake car and fake MJPEG camera from scripts/check_follow.py (fixture photo). First pass found two defects, both fixed and re-checked: (1) picking showed "0 people in view" because detection ran only after a lock; Follower now scans while the page watches; (2) boxes and clicks were mapped to the element, not the letterboxed picture; picture() now maps both. Re-check: Follow -> "3 people in view · click one"; click on the right-hand person -> "following · 10° right", teal box on them; click on the left-hand person -> "following · 9° left", teal box moved; Space -> fake car received {"N":100}, no N=29 after it, /follow on=false locked=true. Bearings carried D3=93 ms.

- [ ] F6: On the car, with v5 flashed: bearing sign and turn direction are right, it holds the gap, backs off, searches and stops as specified (todo.md task f).
  EVIDENCE: pending. Flash part done 2026-09-25: 32164 bytes written and verified by avrdude read-back; USB static checks (queries, N=29 accepted, 41 telemetry frames, MCU clock 1628 -> 6068 ms monotonic). No motion yet.
