#!/usr/bin/env python3
"""U1: the modified UNO sketch compiles with the native toolchain and fits."""
import os, re, subprocess, sys
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
# gcc 8 (closest native match to Arduino's gcc 7.3) plus -mcall-prologues: 31.3 KB; gcc 9 overflows the 32 KB flash
FLAGS = "-mcall-prologues"
cmd = ["arduino-cli", "compile", "--fqbn", "arduino:avr:uno", "--build-path", "build/uno_v4_mod",
       "--build-property", "compiler.path=" + os.path.join(root, "build", "avr-bin8") + "/",
       "--build-property", "compiler.cpp.extra_flags=" + FLAGS, "--build-property", "compiler.c.extra_flags=" + FLAGS,
       "--build-property", "compiler.c.elf.extra_flags=-Wl,--relax", "firmware/uno_v4_mod"]
r = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
out = re.sub(r"\x1b\[[0-9;]*m", "", r.stdout + r.stderr)
print("\n".join(l for l in out.splitlines() if "Sketch uses" in l or "Global variables" in l or "error" in l.lower() or "Used platform" in l or "arduino:avr" in l))
# the sketch silently restarts in init when built on AVR core 1.8.8; 1.8.3 (ELEGOO's era) is required
core_list = subprocess.run(["arduino-cli", "core", "list"], capture_output=True, text=True).stdout
core_ok = re.search(r"arduino:avr\s+1\.8\.3\b", core_list) is not None
print("AVR core 1.8.3 in use:", core_ok)
m = re.search(r"Sketch uses (\d+) bytes .* Maximum is (\d+)", out)
g = re.search(r"Global variables use (\d+) bytes .* Maximum is (\d+)", out)
ok = r.returncode == 0 and m and g and int(m.group(1)) < int(m.group(2)) and int(g.group(1)) < 0.85 * int(g.group(2))
hexf = os.path.join(root, "build/uno_v4_mod/uno_v4_mod.ino.hex")
ok = ok and os.path.isfile(hexf) and os.path.getsize(hexf) > 20000
mods = sum(open(os.path.join(root, "firmware/uno_v4_mod", f)).read().count("mod:") for f in ("ApplicationFunctionSet_xxx0.cpp", "DeviceDriverSet_xxx0.cpp", "ApplicationFunctionSet_xxx0.h"))
print(f"mod markers in source: {mods} (need >= 9)")
ok = ok and mods >= 9 and core_ok
print("uno build verification passed" if ok else "uno build failed"); sys.exit(0 if ok else 1)
