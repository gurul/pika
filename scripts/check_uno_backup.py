#!/usr/bin/env python3
"""U2: read the UNO's flash twice through its bootloader; both reads must be
valid Intel HEX of at least 20 KB and identical. Needs the UNO on USB with
the shield's UART switch in the USB position. Port from CAR_UNO_PORT or the
only /dev/cu.usbserial-* present."""
import glob, os, subprocess, sys, time
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
port = os.environ.get("CAR_UNO_PORT") or (glob.glob("/dev/cu.usbserial-*") or [None])[0]
if not port:
    sys.exit("no /dev/cu.usbserial-* port: plug the UNO's USB in")
out_dir = os.path.join(root, "firmware", "build-archive"); os.makedirs(out_dir, exist_ok=True)
stamp = time.strftime("%Y%m%d")
files = [os.path.join(out_dir, f"uno-factory-{stamp}.hex"), os.path.join(root, "build", "uno-factory-verify.hex")]
for f in files:
    r = subprocess.run(["avrdude", "-p", "m328p", "-c", "arduino", "-P", port, "-b", "115200", "-U", f"flash:r:{f}:i"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-800:]); sys.exit(f"avrdude read failed on {port}")
    print(f"read {f}: {os.path.getsize(f)} bytes")
a, b = open(files[0]).read(), open(files[1]).read()
ok = True
def check(c, m):
    global ok; print(("ok:   " if c else "FAIL: ") + m); ok = ok and c
check(a.startswith(":") and a.rstrip().endswith(":00000001FF"), "first read is Intel HEX with an EOF record")
check(len(a) >= 20000, f"first read size {len(a)} >= 20000 chars")
check(a == b, "second read identical to the first")
data_lines = [l for l in a.splitlines() if l.startswith(":") and l[7:9] == "00"]
nonblank = sum(1 for l in data_lines if set(l[9:-2]) != {"F"})
check(nonblank >= 500, f"{nonblank} non-blank data records (a real program, not erased flash)")
print("uno backup verification passed" if ok else "uno backup check failed"); sys.exit(0 if ok else 1)
