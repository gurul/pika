# Car project TODO

## Next up

Open tasks, in no fixed order. Each has enough context to resume cold.

### a) Sonar servo center calibration

The servo's commanded 90° is not physically straight, so every "ahead" reading
is taken slightly off-axis.

Built 2026-09-23: the controller's Sonar front panel (1° and 5° arrow nudges
in raw degrees, live distance, "Save as front" to `servo_center_deg` in
`calibration.json`), and one offset in `tools/car.py` for outgoing `N=5`/`N=28`
and incoming telemetry pan (`tools/tests/test_pan_center.py`).

Front saved by the owner on 2026-09-23: 95° raw (21 cm to the target at save
time). Still open: see whether roaming's "ahead" readings improve with it.

### b) Kitchen island go-around

Classic Bug-style obstacle avoidance to a goal on the far side of the island.

1. Car points at the goal. Drive with gyro heading hold until the island is
   about 30 cm ahead.
2. 90° gyro turn (`tools/precision.py` turn pattern).
3. Drive along the island's end with the sonar pointed sideways until the side
   range jumps; continue about 25 cm margin.
4. Turn back parallel to the goal line; follow the long side until it clears;
   continue the margin.
5. Turn in and drive back the same lateral offset, face the goal, drive the
   remaining distance.

Distance comes from time × 22.5 cm/s (`forward_cm_per_s` at speed 120 in
`calibration.json`; an older sweep-based number, probably low).
Open questions for the owner: goal distance, which side (or auto-pick),
island size, start distance.

### c) `roam.py` spins at random

Seen in `build/runs/roam-20260922-183714/`. Causes found in `tools/roam.py`:

- The pre-spin full look saturates the histogram: close readings are enlarged
  by `asin(25/r)` and together cover every sector, so `freest()` picks whichever
  sector returned no-echo (density 0.1). The choice is effectively random.
- When the freest sector is near center, a coin flip picks ±90°.
- A random wander target every 12 s (`--wander`).
- Spins are timed pulses at 150 PWM, while the turn calibration was measured
  at 140 PWM; not gyro-closed.
- False "lifted" stops at about 51 s and 111 s of that run.

Fix: pick the spin side by the median of real ranges per side (ignore
no-echo), remove the coin flip and the wander, close turns on the gyro
(`precision.py` pattern), and find why the lift check trips.

### d) `stream_roam.py` freezes in the real room

A 4-minute drive (`build/runs/vfh-live/`) froze on `unknown_ahead`: 171 of 215
forward readings were no-echo. `stream_roam.py` treats no-echo as unknown;
`roam.py` treats it as open, which is why `roam.py` moves. Decide how to treat
no-echo (e.g. open beyond a range, or open only when neighbouring beams agree).
Simulation gate G2 in `.unlazy/vfh-restore/GATES.md` is still unmet. The
scored gaze (Finean et al., RA-L 2021, arXiv 2109.04721) is implemented but
untested on hardware.

### e) Radar feed on a live drive

`tools/sonar_feed.py` + `web/sonar-feed.html` are verified only on a replayed
log. Run it beside a live `stream_roam.py --observe` run (shared `--run-dir`)
and confirm the live and stale states.

## Paused: TF-Luna lidar bring-up — 2026-09-22

Paused explicitly by user. Do not continue programming, wiring tests or flashing until they resume this work. User now wants to understand why the older autodrive was better.

### Hardware state

- Photo IMG_1134.HEIC shows an ELEGOO Mega 2560 R3 with TF-Luna. This is a separate bench board, not the car UNO.
- Mega initially did not appear over USB. Following disconnection of the lidar wires and the barrel power cable, USB-only power worked.
- User explicitly confirmed **USB only; lidar wires disconnected**. Last USB identity: `/dev/cu.usbmodem31301`, VID/PID `2341:0042`, serial `24230313032351606131`.
- Four-second passive read at 115200 received 523 zero bytes, with no valid lidar data. This was with the sensor disconnected; it does not diagnose sensor damage or prove the existing firmware is suitable. Serial port closed afterwards.
- No new firmware has been flashed. No car motor commands were sent during lidar work. Car was previously stopped with acknowledgment.

### Correct the wiring advice before reconnecting

I previously supplied instructions using wire colours and connecting Mega TX1/pin18 directly to lidar RX. That was not adequately verified. The TF-Luna communication level is LVTTL 3.3V, while Mega outputs use 5V logic. Do not repeat that direct TX connection.

Manufacturer-numbered receive-only UART test:

| TF-Luna connector pin | Function | Mega connection |
| --- | --- | --- |
| 1 | Power | 5V |
| 2 | RX | Leave disconnected |
| 3 | TX | RX1 / digital19 |
| 4 | Ground | GND |
| 5 | Mode | Leave disconnected for UART |
| 6 | Output | Leave disconnected |

Power off before rewiring. Verify connector orientation and trace each wire; the existing photo is insufficient to certify colours or endpoints. Sensor supply is 3.7–5.2V and lacks reverse-polarity/overvoltage protection. Keep Mega RX internal pull-up off and hardware TX1 disabled. A 3.3V nominal sensor TX exceeds the Mega's 3.0V VIH threshold at a 5.0V supply; the available sensor manual does not establish worst-case output margin.

Primary sources: [TF-Luna manual, §§5–6 / Figure4](https://en.benewake.com/uploadfiles/2025/04/20250430174515390.pdf), [Mega pinout](https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf). Official manual materialized by `pcb scan` at `~/.pcb/cache/datasheets/materialized/6e1cfb21-5ed7-5f76-998a-e2593a890231/20250430174515390.md`; connector figure is `images/img-5.jpeg` beside it. User photo converted for inspection to `/private/tmp/car-lidar-connection.png` (temporary; original remains in Messages attachments).

### Unfinished software and next steps

- Draft only: `firmware/mega_lidar/mega_lidar.ino` and `TfLunaParser.h`. No tests, build, flash or live sensor verification yet. Treat both files as unverified.
- Draft configures RX1 at115200 without enabling TX1 or pull-ups, parses default9-byte5959 frames with checksum, reports signal/range validity and limits USB reporting to20Hz. All frame distances assume factory cm mode, which must be checked against a measured target because units are not encoded in these packets.
- Acceptance ledger `.unlazy/lidar-bringup/GATES.md` has all3 gates unmet: firmware checks, independent electrical review, physical range capture. `scripts/check_mega_lidar.py` named by the ledger **has not been created**.
- Independent electrical reviewer was interrupted at the user's pause request; preliminary findings are incorporated above, no report file was written.
- On resume: verify numbered connector orientation, complete parser and receive-only tests, compile for Mega using existing AVR1.8.3/native gcc8, preserve existing Mega flash before replacement, then perform a short stationary capture and compare two measured target distances. Never interpret a powered board alone as a working lidar.
- TF-Luna is a fixed single-beam sensor, not a360° scanner. Lidar integration must still solve coverage/scanning and steering; it does not automatically fix jerky autodrive.

## Autodrive investigation

User preferred the earlier `tools/roam.py` VFH-lite behavior. `HANDOVER.md` explicitly recommended preserving its histogram and cost function while updating transport/gyro control. Current `tools/active_roam.py` replaced more behavior and physically regressed to stop–scan–move. No controller rollback or new drive test is authorized by the current explanatory question; compare and explain first.
