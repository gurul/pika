# TF-Luna Mega bench bring-up

Root: OPEN. Work paused by explicit user request; all gates remain unmet. Do not resume until requested. Goal: a verified receive-only UART reader and correct numbered wiring; no car motion or navigation integration.

| Leaf | State | Needs | Ownership |
| --- | --- | --- | --- |
| 1 firmware and offline checks (root) | READY | none | firmware/mega_lidar/**, scripts/check_mega_lidar.py, build/mega_lidar/** |
| 2 independent electrical review | READY | none | build/lidar-electrical-review.md |

Contract: Mega 2560, AVR core 1.8.3/native gcc8. Sensor pin1=5V, pin3=TX to RX1(19), pin4=GND, all others open. Never infer colours. Sensor UART 115200 8N1 default 9-byte 5959 frames; checksum and signal validation. No sensor commands, pin18 TX disabled. Root owns all physical access; only verified Mega VID2341/PID0042/serial24230313032351606131. Preserve existing flash before replacing if flashing proceeds. Physical range validation waits for verified wiring. Agent reviews manufacturer docs and photo, read-only outside own report. Shell /bin/sh; cwd car; Python .venv/bin/python. Root re-verifies leaf evidence.
