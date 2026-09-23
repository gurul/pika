# Handover: ELEGOO car project, 2026-09-22

## 2026-09-22 evening: restored VFH controller + live radar (Claude Code session)

`tools/stream_roam.py` (VFH restored on v4 telemetry) and `tools/sonar_feed.py` +
`web/sonar-feed.html` (read-only radar, tails the controller's log) are built.
The radar was checked in Chrome on a replayed real log, with live and stale states. It has not yet been checked against a live car.
An independent review (`build/vfh-restore-review.md`) found no safety defect and 4 medium stop-go
causes; all 4 are fixed (see `.unlazy/vfh-restore/GATES.md` G4). 78 unit tests pass. The simulation gate
G2 is still UNMET under its tightened limits: 6–7 stops per 35 s, and the noisy case gets past the obstacle in 6/10 seeds.
Physical run not yet done; the car was powered off by the user. Next: observe run with radar, then
a short supervised drive.


Latest: lidar bench bring-up explicitly paused by user; exact hardware state, corrected numbered wiring, unverified draft firmware and remaining work are saved in `todo.md`. No Mega firmware was flashed. User now asks why the older autodrive worked better.

## Current direction: user will install lidar first

User chose to install lidar before further navigation development. Pause sonar-
controller changes and physical runs until they resume. No source or firmware
changes were made in response to this latest feedback.

The requested four-minute run `build/runs/active-20260922-175119/` was interrupted
after **145.577 active seconds**, not completed. User reported short forward
movements separated by swivels and stops, and poor straight driving. Logs agree
with repeated acceleration resets: 420 consumed telemetry decisions were drive,
370 braking_margin, 214 unknown_ahead, 197 no_confirmed_path, 53 close_obstacle,
37 recover_turn, and 8 awaiting_pan. These are decision counts, not distance or
proof of physical progress. Maximum commanded PWM was 134.5.

The controller was stopped gracefully using SIGINT. Its `estop` result was the
intentional interruption; final N100 stop returned `{ok}`, stream-off and sonar-
center commands also acknowledged. Controller exited; TCP slot released.

Next integration should address dense obstacle coverage plus continuous steering,
motor deadband/straight-line calibration and stationary gyro validation. Lidar
does not itself fix drive-control behavior. Model/interface not yet supplied.
Smooth physical navigation remains unverified; retain that limitation and keep
the user-requested removal of the battery cutoff.

## Latest physical update after power cycle

Latest straight-line speed probe after the user's repositioning: five consistent
forward readings were 62 cm, so the existing braking model limited the requested
255 PWM to **123 PWM**. One N4 command ran for a 0.3-second host window, followed
by acknowledged stop. Median forward range changed 62 -> 53 cm (about 9 cm sonar-
estimated progress, not external odometry). Stream disabled and exit stop also
acknowledged. Evidence: `build/fast-burst-20260922-175012.json`. This verifies a
short forward response; full-speed travel and smooth roaming remain unverified.

Wi-Fi and command acknowledgements recovered. Run `active-20260922-174335`
completed 20 active seconds but the user reported no movement; gyro readings
were unusable (over 800 degrees apparent rotation). Stationary measurement then
confirmed -43.664 deg/s drift. N26 recalibration reduced the immediate measurement
to -1.56 deg/s; a subsequent two-second stationary interval had zero recorded
heading change. Evidence: `build/stationary-gyro-recalibration.json`.

A supervised 150 ms, 140 PWM left pulse then produced about 26 degrees of measured
turn, confirmed by the user as a small movement. Stop acknowledged. Evidence:
`build/stall-motor-response.json`. Do not equate earlier gyro drift with motion.

After recalibration, low-speed run `active-20260922-174536` completed with only
limited turns and no sustained forward motion. Requested fast run
`active-20260922-174634` completed 20 active seconds with zero wheel commands:
52 of 57 settled forward samples were no-echo; other pan angles had valid echoes.
User confirmed open space ahead. End stop and stream-off both acknowledged.

User then placed the car facing an object. Three stationary N21 queries each
returned **13 cm**, confirming the sonar does see that object. Battery query
returned 7371 mV. No full-speed burst was sent after this repositioning because
the object is close. Need a clear test lane with the target farther away for
the requested speed test. The software currently rejects no-echo space, which
blocks open-room travel; resolving this sensing limitation remains outstanding.
Persistent startup gyro validation has not yet been implemented. G5 remains unmet.

## Latest: scan-loop fix verified; physical link failure unresolved

The host planner no longer lets an infeasible direction monopolize sonar scanning.
It evaluates the side-glance braking budget before committing a goal, widens the
scan while stopped (including unknown front echoes), and preserves all stop rules.
The user's removal of the battery cutoff remains in place. No firmware was changed.

Pan ACK loss now holds motion stopped and can be confirmed by a newer accepted
angle-tagged telemetry frame; failure to confirm still latches estop. Main-loop
logs contain the actual decision input, ingest acceptance/age, and command replies
and latency. Estop exits are errors. Requested duration now starts after initialization.

Evidence: 37 tests pass (7 stall/transport regressions, 12 planner, 18 integration/
safety); all 3 existing independent-geometry simulations pass. Replay of 162 real
stationary samples changes requested scan angles from {70,90} to all nine angles
10..170 in 20-degree steps, without authorizing motion from that blocked evidence.
These checks were re-executed through `.unlazy/stall-fix/GATES.md`.

Physical observe run `build/runs/active-20260922-161352/` completed 12 active seconds,
88 consumed frames, zero wheel commands, and confirmed a missing pan ACK via
telemetry. Driving attempts `active-20260922-161416/` and `active-20260922-161535/`
lost all UNO replies after N2 and N4 respectively. Stop attempts in those failed
runs were NOT acknowledged. A read-only probe between runs acknowledged stop and
reported yaw 75.2 vs 78.5 before the prior turn. That does not prove cause of failure.
At the last reachability check, elegoo-car.local resolved to 10.0.0.43 but TCP 100
and TCP 80 both timed out. No further driving process is running. User was asked
what the car's power/lights/wheels are doing; answer pending. Physical gate remains
unmet, not abandoned. Do not report smooth driving or full task completion.

Research swarm synthesis: `RESEARCH-stall-fix.md`. Three independent reports are
in `build/stall-review.md`, `build/sonar-methods-research.md`, and
`build/navigation-methods-research.md`. Next: restore/diagnose the physical link,
then verify motion. The next navigation architecture to evaluate is a small set
of calibrated curves with gyro feedback and observation scheduling.

## Active sonar follow-up — v4 flashed and stationary checks passed

Physical test update: after USB disconnection and the user's camera-switch
confirmation, Wi-Fi observe mode passed. A requested 20-second run at cap 110
aborted at 11.37 seconds with stale telemetry during recovery turns (9 motion
frames). The subsequent requested 60-second run at cap 140 completed in 60.25
seconds with 443 telemetry frames and 16 motion frames, but stalled scanning:
350 of 443 sampled decisions were `braking_margin`. Peak planned differential
PWM was only 54.29; recovery turn pulses were 100 PWM. This did not demonstrate
high-speed travel or smooth navigation. Logs: `build/runs/active-20260922-160346/`
and `build/runs/active-20260922-160415/`. Final clearance was 44 cm at heading
-20 degrees. The 60-second duration includes connection and initialization.

V4 was flashed on 2026-09-22 through `/dev/cu.usbserial-3130` using hwlog and
avrdude; all 31,546 flash bytes were read back and verified. Stationary USB
checks received 58 extended frames, verified advancing sequence/sample time,
negative moving and positive settled pan tags at 140 degrees, and recentered
to 90 degrees. Evidence: `build/active-roam-flash-static.json`.
No wheel commands were sent. Streaming is off, standby was commanded, and
hwlog and the serial connection are closed. Battery telemetry was **6.561 V**.
At the user's request, the Mac active-roam controller's 7.0 V cutoff was removed;
battery telemetry is retained. This change does not require another firmware flash.
USB disconnection and returning the UART switch to camera mode remain manual
steps before any driving. Physical smoothness remains unverified.

The source now includes v4 nonblocking pan (`N=28 D1=angle`) and optional
ten-field telemetry (`N=25 D1=100 D2=1`). The running UNO now has v4.
User confirmed charged battery, USB disconnected and a clear level floor for
supervised tests. A subsequent request to connect UNO USB and set upload mode
is pending; no USB serial port was present at the last check. No motor command
or firmware flash was sent in this follow-up.

New `tools/active_roam.py` implements the stream-based replacement behavior:
progressive sensor glances, unknown-space rejection, gyro bearing memory,
clearance-dependent speed, chosen-heading persistence and bounded recovery
turns toward confirmed openings. It requires v4 and has an `--observe` mode.
`tools/car.py` accepts both telemetry formats and serializes heartbeat writes.
`tools/safety.py` now catches expired differential renewals, stops on revoked
forward clearance and applies the close-obstacle veto to one-wheel forward arcs.

Read `RESEARCH-active-roam.md` and `GATES-active-roam.md` for research and current
evidence. Three main simulations pass, but speed/noise variation still exposes
stalls. **Do not describe it as seamless or physically verified.** The LiDAR
in the user's photographs is TF-Luna, a single-beam sensor; the previous note
below that its model is unknown is superseded.

V4 compiles on AVR core 1.8.3 with the existing build script. To fit flash,
four integer `sprintf` calls were replaced with equivalent `itoa` calls;
legacy features were retained. Pan timing uses the existing compensated
`_millis()` clock because this shield changes Timer0's prescaler.
Pre-edit v3 source files are in `build/active-roam-v3-backup/`.
A rebuilt v3 rollback image is `build/uno_v3_rollback/uno_v4_mod.ino.hex`;
it is a rebuild from the saved sources, not a readback from the device.

Next: charge the battery and confirm USB disconnection/camera switch before
any motion. The flash and stationary stream/pan checks are complete. Observe first,
then a supervised low-speed run. Do not run `scripts/check_uno_protocol.py`
on USB: the inherited script contains a motor command despite its name.

---

For the next agent. Everything here was verified in the session that wrote it unless marked otherwise. Read `README.md` for the how-to; this file is the state, the judgement calls, and the traps.

## What this is

An ELEGOO Smart Robot Car Kit V4 (UNO R3 + ESP32-S3 camera board) driven from a Mac over Wi-Fi. Owner: Guru. Personal project, no Era governance, not a git repo yet. Project root `~/Documents/personal/car`, Python env `.venv/` (uv, Python 3.12: numpy, pillow, openai 3.17.0, pyserial).

Goal as stated by the owner, in order of arrival: control the car from the Mac, then autonomous roaming as fast and smooth as possible, then a digital map of every open room in the house. A lidar is available to attach (model not yet named); that is the intended path to a real map.

## Current hardware state

| Part | State |
|---|---|
| Camera board (ESP32-S3-WROOM-1, OV3660) | flashed with `firmware/camera_s3` (community-patched ELEGOO sketch): joins Wi-Fi `<home Wi-Fi SSID>` as `elegoo-car.local` (was 10.0.0.43), keeps hotspot `ELEGOO-<board MAC>`. TCP 100 = JSON bridge with `{Heartbeat}`; HTTP 80 `/capture`, 81 `/stream`. One TCP client and one video viewer at a time. Factory backup `firmware/build-archive/camera_s3-factory-20260922.bin`. |
| UNO R3 | flashed with `firmware/uno_v4_mod` **v3** (see below). Factory backup `firmware/build-archive/uno-factory-20260922.hex`. |
| Shield | the **2020 TB6612 board**: standby on pin 3 must be HIGH, PWMA 5, PWMB 6, AIN_1 7, BIN_1 8, group A direction polarity inverted relative to ELEGOO's V0 source (which targets the newer DRV8835 shield). |
| Battery | 2000 mAh Li-ion, was at 6.2 to 6.6 V under load at the end of the session, i.e. nearly flat. The firmware warns (red LED) below 7.0 V. **A flat battery blocks the motors while the UNO, sonar, gyro and camera keep working; it looks exactly like broken motor code.** Charge it before believing any motion result. |
| Shield UART switch | camera position = Wi-Fi control, USB commands to the UNO blocked. USB position = flashing and USB serial, camera cut off. It must be flipped by hand for every reflash. |

## UNO firmware v3 (`firmware/uno_v4_mod`)

ELEGOO `SmartRobotCarV4.0_V0_20210104` plus, each marked `mod:` in the source:

- sonar cap 150 to 400 cm, 30 ms echo timeout (400 = no echo)
- `N=24` gyro heading, tenths of a degree; integrated every 10 ms (every-loop integration is silently zeroed by the firmware's 0.05 deg deadband)
- `N=25 D1=<ms>` telemetry stream `{T_<dist>_<yaw*10>_<L>_<M>_<R>_<ground>_<mV>}`; request 70 ms, the camera bridge drops about a quarter so about 12 frames/s arrive
- `N=26` gyro zero recalibration; the car must be truly still, not just set down (that gave 7 deg/s drift)
- `N=27` battery millivolts
- servo `N=5 D2` in plain degrees 10..170, 200 ms settle (the stock driver moved in 10 deg steps with a 500 ms block)
- `N=4` wheel-speed drive stops itself 1.5 s after the last renewal
- TB6612 pin map with standby and the group A polarity fix (`-DSHIELD_DRV8835` restores ELEGOO's map)

Gyro sign: **a left turn decreases yaw** (`yaw_left_sign = -1` in `calibration.json`). Turn rate from the gyro: 189.5 deg/s at speed 140. Forward: 22.5 cm/s at speed 120 (older sweep-based number, probably low; re-measure with the gyro-era firmware).

Protocol traps: `N=1` (per-motor) zeroes the *other* group's PWM, so it cannot drive both sides; use `N=2` (timed moves, left/right are spins in place) or `N=4`. `N=2` timed moves ack at timer expiry, not at receipt. `N=23` prints `_false` while ON the ground.

Ledger `GATES-uno.md`: all 8 gates met with evidence from real runs. Build is `scripts/check_uno_build.py`.

### Building and flashing the UNO (the hard-won part)

- Arduino ships its AVR compiler and avrdude for macOS as x86 binaries; this Mac has no Rosetta. Native toolchain: `brew trust osx-cross/avr && brew install avr-gcc@8 avrdude`; `build/avr-bin8/` symlinks compiler and binutils into one dir that arduino-cli's `compiler.path` build property points at; `-mcall-prologues` and `-Wl,--relax` make it fit (31.6 KB of 32 KB). gcc 9 and 12 overflow.
- **The sketch only runs when built on Arduino AVR core 1.8.3.** On 1.8.8 the untouched stock sketch jumps back to reset about a second into init, with no reset cause and no unhandled interrupt, on every compiler tried. Diagnosed over 11 flash cycles; do not repeat that. `arduino-cli core list` must show `arduino:avr 1.8.3`.
- Servo library 1.1.8 (ELEGOO era). FastLED 3.2.10 comes from ELEGOO's `addLibrary` zip, installed into `~/Documents/Arduino/libraries/FastLED`.
- Flash: UNO on its own USB (shows as `/dev/cu.usbserial-31x0`; the `usbmodemSN234567892` port is the dock's billboard, ignore it), switch on the USB side, `avrdude -p m328p -c arduino -P <port> -b 115200 -U flash:w:build/uno_v4_mod/uno_v4_mod.ino.hex:i`. Then static checks only. **Never send a motion command while the USB cable is in; it is short.** Unplug, flip the switch back, test motion over Wi-Fi.
- The buddy daemon (`com.github.cc-buddy-bridge.daemon`) grabs any ESP32-S3 on USB and toggles its boot straps. Bootout it before the camera board ever goes on USB again; the widget relaunches it.

## Mac-side tools (`tools/`)

| Tool | State |
|---|---|
| `car.py` | TCP client; answers heartbeats; parses telemetry into `car.telemetry`; one command in flight at a time across threads. Sound. |
| `controller.py` + `controller.html` | browser controller at `http://127.0.0.1:8765/`: video, hold-to-drive pad, speed, camera pan, straight trim slider (saved to `calibration.json`), distance streaming while driving. Works. Forward uses `N=4` with trim and a 0.7 s server deadman. Holds the car's single TCP slot: stop it before any autonomous tool. |
| `sonar.py` | servo sweeps (17 positions, 10 deg steps, about 10 s) and the older sweep-based calibration. Works. |
| `safety.py` | owns motion frames: pulse cap 450 ms, forward veto below 25 cm or with a stale "ahead" reading, liveness from any UNO reply (0.8 s), e-stop latch, `drive_diff` with trim and a 1 s software deadman. 8 unit tests pass. |
| `precision.py` | closed-loop turns on the gyro and drives on the sonar. **Measured: turns within 0.3 to 1.5 deg on 90 and 180; drives within 0 to 4 cm on 30 to 50 cm.** The one solid demonstration of what the platform can do. |
| `roam.py` | VFH-lite reactive roaming (Borenstein and Koren's Vector Field Histogram, in the VFH+ form; arXiv 2011.05228 was the reference): decaying polar histogram from servo glances, cost-function steering, clearance-scaled speed. **The owner preferred how this one drove.** Written before firmware v3, so it still polls distance instead of using the stream and uses timed spins instead of gyro turns. |
| `dash.py` | fast mode on firmware v3: 12 Hz stream, heading hold, slew-limited speed, gyro turns when blocked, then a "curve away early" behaviour added last. **The owner called the result terrible.** Honest reading: the curve-away doubled the block count (38 in 90 s vs 17 before it) because it bends into the unseen side; the stop-glance-turn cycle every three seconds in a small room is the real jerkiness. The precise turns and the stream are good; the behaviour on top is not. |
| `explore.py` + `mapping.py` + `vision.py` | stop-and-scan sonar mapping: log-odds occupancy grid (Moravec and Elfes sonar model), beam-model scan matcher (about 6 to 8 cm and 3 to 6 deg per step on the simulator, 12 poses), frontier exploration, OpenAI room/hazard labels. Ran once for four minutes on the old firmware: the map was a blob (see `build/runs/map/map.png`), because the car barely moved (vision vetoed on clutter, since fixed), sonar saw chair legs within 150 cm, heading drifted. Gate G7 in `GATES.md` is recorded **unmet**. Not yet adapted to firmware v3 (gyro heading, 400 cm sonar, stream). |

Vision: `tools/vision.py` reads `OPENAI_API_KEY` from `~/.config/car/env`, falling back to `~/.config/cc-buddy-bridge/env` (both exist, never print them). Model default `gpt-5-mini`. Verified on real frames. Room labels flip-flop; a 2-of-3 vote is in place. Only stairs, people, pets and liquid may veto a move.

## Recommended next steps, in order

1. **Charge the battery fully** and re-run `tools/precision.py probe` to confirm forward and the yaw sign; then `tools/precision.py square 40` in a space with 60 cm or more on every side, which never completed here for lack of room.
2. **Bring the VFH roam onto firmware v3.** Keep `roam.py`'s histogram and cost function (that is what felt right); replace its distance polling with the telemetry stream, its timed spins with gyro turns (`precision.py`'s `turn` is the pattern), and re-scale its density to the 400 cm sonar. Drop `dash.py`'s curve-away; keep its slew limit and heading hold if useful. Median-filter the sonar while moving: single readings jump 50 to 100 cm under motor load.
3. **Map with the lidar**, not the sonar. When the owner names the lidar model and a spare ESP32 dev board, build a lidar-to-Wi-Fi bridge (UART in, UDP out) and run scan matching on the Mac; `mapping.py`'s matcher generalises to 360-degree scans with the beam count changed. The sonar map is a dead end for "reconstruct my room"; the 150 cm cap is gone but the 15-degree beam and floor-level view remain.
4. Optional firmware v4 ideas: telemetry could include a servo-angle field so sweeps can stream; the camera bridge could stop dropping frames (`Serial2.setRxBufferSize`), but that means reflashing the ESP32 (USB-C on the back of the camera board, download mode via BOOT+RST, `--after watchdog-reset`, back up with chunked `--no-stub read-flash`).

## Other threads touched in this session

- **buddy** (`~/Documents/personal/buddy`): Telegram message formatting rewritten (HTML parse mode, one formatter module, 17 new tests, suite 1767 passed). Merged into `main` as `79bb646`, daemon restarted, owner confirmed it reads well on the phone. **Not pushed.**
- **arduino-cli on this Mac**: Arduino's bundled `ctags` is x86-only too; a native build of arduino/ctags now sits at `~/Library/Arduino15/packages/builtin/tools/ctags/5.8-arduino11/ctags` (original beside it as `.x86_64.bak`). A core update may bring the x86 one back.

## Files that must not be committed

`firmware/camera_s3/secrets.h`, `firmware/camera_sta/secrets.h` (Wi-Fi password), `firmware/build-archive/` (backups, one holds nothing sensitive but is 8 MB), `.venv/`, `build/`. `.gitignore` already lists them. `calibration.json` is fine to commit.
