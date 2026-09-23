#!/usr/bin/env python3
"""G2: safety layer unit tests; prints the marker only when all pass."""
import os, subprocess, sys
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
r = subprocess.run([sys.executable, "-m", "unittest", "-v", "tools.tests.test_safety"], cwd=root, capture_output=True, text=True)
print(r.stdout + r.stderr)
if r.returncode == 0 and "OK" in r.stderr and " ... ok" in r.stderr:
    print("safety verification passed")
else:
    sys.exit(1)
