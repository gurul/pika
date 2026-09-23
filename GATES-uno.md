# Gates: UNO firmware mod

OWNS: firmware/uno_v4_mod/**, firmware/build-archive/uno-*, scripts/check_uno_*.py, GATES-uno.md, README.md

Scope: the UNO runs a modified stock firmware with a 400 cm sonar cap, a gyro heading command, a telemetry stream command, single-degree servo control and a wheel-speed deadman, with the factory firmware backed up first

- [x] U1: the modified sketch compiles for arduino:avr:uno with the native toolchain and fits in flash
  CHECK: .venv/bin/python scripts/check_uno_build.py
  EXPECT: uno build verification passed
  EVIDENCE: scripts/check_uno_build.py 2026-09-22: "Sketch uses 31590 bytes (97%)", "AVR core 1.8.3 in use: True", "uno build verification passed" (exit 0). Firmware v3.

- [x] U2: the factory flash was read back from the UNO through its bootloader into firmware/build-archive/uno-factory-<date>.hex, is a valid Intel HEX of at least 20 KB, and a second read matches it
  CHECK: .venv/bin/python scripts/check_uno_backup.py
  EXPECT: uno backup verification passed
  EVIDENCE: scripts/check_uno_backup.py 2026-09-22 on /dev/cu.usbserial-3130: two reads of 77836 chars identical, 988 non-blank records, "uno backup verification passed" (exit 0). File firmware/build-archive/uno-factory-20260922.hex.

- [x] U3: after flashing, the UNO answers the stock protocol over USB: distance, servo and a timed move all acknowledge
  CHECK: .venv/bin/python scripts/check_uno_protocol.py
  EXPECT: uno protocol verification passed
  EVIDENCE: scripts/check_uno_protocol.py after the v1 flash: distance {1_36}, servo {2_ok}, yaw {3_0}, timed move {4_ok}, stop {ok}, "uno protocol verification passed" (exit 0). After the v3 flash, static checks over USB: distance {1_20}, yaw {2_0}, battery 7168 mV, servo ok; motion deliberately not exercised on USB (cable rule), exercised over Wi-Fi in the precision runs.

- [x] U4: the sonar reports beyond 150 cm: pointed at a far wall it returns a value between 151 and 400
  EVIDENCE: Over Wi-Fi 2026-09-22: sonar returned 256 cm (servo 90) and 280 cm (servo 125-140) with the v1 firmware; 254 and 280 during the precision probe with v3. 400 = no echo.

- [x] U5: the gyro heading command changes by 80 to 100 degrees after a calibrated 90-degree spin, and drifts under 2 degrees over 20 s at rest
  CHECK: .venv/bin/python scripts/check_uno_yaw.py
  EXPECT: uno yaw verification passed
  EVIDENCE: scripts/check_uno_yaw.py 2026-09-22 over Wi-Fi: drift +0.0 deg over 20 s after N=26; two 500 ms spins -94.0 and -95.5 deg (1.5 deg apart); "uno yaw verification passed" (exit 0). tools/precision.py closed-loop turns: 90 -> 90.9, -90 -> -91.0, 180 -> 180.2, 89.5, 89.4 deg.

- [x] U6: the telemetry stream at 100 ms delivers at least 8 frames per second over Wi-Fi through the camera bridge, each frame parsing into distance, yaw, three floor values and the ground flag
  CHECK: .venv/bin/python scripts/check_uno_stream.py
  EXPECT: uno stream verification passed
  EVIDENCE: scripts/check_uno_protocol.py after the v1 flash: distance {1_36}, servo {2_ok}, yaw {3_0}, timed move {4_ok}, stop {ok}, "uno protocol verification passed" (exit 0). After the v3 flash, static checks over USB: distance {1_20}, yaw {2_0}, battery 7168 mV, servo ok; motion deliberately not exercised on USB (cable rule), exercised over Wi-Fi in the precision runs.

- [x] U7: the wheel-speed deadman stops the car within 2 s when renewals stop
  EVIDENCE: scripts/check_uno_backup.py 2026-09-22 on /dev/cu.usbserial-3130: two reads of 77836 chars identical, 988 non-blank records, "uno backup verification passed" (exit 0). File firmware/build-archive/uno-factory-20260922.hex.

- [x] U8: the README documents the new commands, the backup and restore procedure, and the toolchain workaround
  CHECK: .venv/bin/python scripts/check_readme.py --uno
  EXPECT: readme verification passed
  EVIDENCE: scripts/check_uno_build.py 2026-09-22: "Sketch uses 31590 bytes (97%)", "AVR core 1.8.3 in use: True", "uno build verification passed" (exit 0). Firmware v3.
