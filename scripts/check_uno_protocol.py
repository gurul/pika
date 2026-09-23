#!/usr/bin/env python3
"""U3: stock protocol over the UNO's USB after flashing: distance, servo, timed move."""
import glob, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import serial
port = os.environ.get("CAR_UNO_PORT") or (glob.glob("/dev/cu.usbserial-*") or [None])[0]
if not port: sys.exit("no /dev/cu.usbserial-* port")
s = serial.Serial(port, 9600, timeout=1); time.sleep(2.5); banner = s.read(200).decode("utf-8", "replace").strip()
print("banner:", repr(banner))
def cmd(frame, wait=1.0):
    s.reset_input_buffer(); s.write(frame.encode()); time.sleep(wait); return s.read(200).decode("utf-8", "replace").strip()
ok = True
def check(c, m):
    global ok; print(("ok:   " if c else "FAIL: ") + m); ok = ok and c
r = cmd('{"H":"1","N":21,"D1":2}'); check(r.startswith("{1_") and r.endswith("}"), f"distance query answered: {r}")
r = cmd('{"H":"2","N":5,"D1":1,"D2":90}'); check(r == "{2_ok}", f"servo answered: {r}")
r = cmd('{"H":"3","N":24}'); check(r.startswith("{3_") and r[3:-1].lstrip("-").isdigit(), f"yaw query answered: {r}")
r = cmd('{"H":"4","N":2,"D1":3,"D2":90,"T":300}', 1.2); check("_ok" in r, f"timed move acknowledged: {r}")
r = cmd('{"H":"5","N":100}'); check(r == "{ok}", f"stop answered: {r}")
s.close()
print("uno protocol verification passed" if ok else "uno protocol check failed"); sys.exit(0 if ok else 1)
