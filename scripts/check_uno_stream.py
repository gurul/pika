#!/usr/bin/env python3
"""U6: telemetry stream at 100 ms over Wi-Fi: >= 8 frames/s, every frame parses."""
import os, re, socket, sys, time
host = os.environ.get("CAR_HOST", "elegoo-car.local")
s = socket.create_connection((host, 100), timeout=5); s.settimeout(0.3)
s.sendall(b'{"H":"1","N":25,"D1":70}')
buf = b""; frames = []; t0 = time.time()
while time.time() - t0 < 5.0:
    try: d = s.recv(4096)
    except socket.timeout: continue
    if not d: break
    buf += d
    while b"}" in buf:
        fr, buf = buf.split(b"}", 1); fr += b"}"
        if fr == b"{Heartbeat}": s.sendall(b"{Heartbeat}"); continue
        frames.append(fr.decode("utf-8", "replace"))
s.sendall(b'{"H":"2","N":25,"D1":0}'); time.sleep(0.5); s.close()
tele = [f for f in frames if f.startswith("{T_")]
pat = re.compile(r"^\{T_(\d+)_(-?\d+)_(\d+)_(\d+)_(\d+)_([01])(?:_(\d+))?\}$")   # 7th field: battery mV (firmware v2+)
parsed = [pat.match(f) for f in tele]
ok = True
def check(c, m):
    global ok; print(("ok:   " if c else "FAIL: ") + m); ok = ok and c
rate = len(tele) / 5.0
check(rate >= 8.0, f"telemetry rate {rate:.1f} frames/s (need >= 8)")
check(all(parsed), f"all {len(tele)} frames parse (first: {tele[:2]})")
if parsed and all(parsed):
    dists = [int(m.group(1)) for m in parsed]; check(all(0 <= d <= 400 for d in dists), f"distance range {min(dists)}..{max(dists)} within 0..400")
print("uno stream verification passed" if ok else "uno stream check failed"); sys.exit(0 if ok else 1)
