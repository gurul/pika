#!/usr/bin/env python3
"""Offline gates only. No device connections, flashing, or motor commands."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from tests.test_active_roam import PlannerTests, IntegrationTests
from tests.test_safety import SafetyTests


def main():
    mode = sys.argv[1]
    if mode in ("planner", "integration"):
        cases = (PlannerTests,) if mode == "planner" else (IntegrationTests, SafetyTests)
        suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in cases)
        if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
            return 1
    elif mode == "firmware":
        code = r'''
#include <cassert>
#include "ActivePan.h"
int main() {
  ActivePan p;
  assert(p.move(130, 1000) == 130);
  assert(p.tag(1000) == -130);
  assert(p.busy(1199)); assert(!p.busy(1200));
  assert(p.tag(1200) == 130);
  p.move(10, 2000); p.move(170, 2010);
  assert(p.busy(2569)); assert(!p.busy(2570));
  assert(p.move(-100, 3000) == 10);
  assert(p.move(300, 4000) == 170);
  p.move(90, 0xffffff00UL);
  assert(p.busy(0x00000020UL));
  assert(!p.busy(0x00000080UL));
}
'''
        with tempfile.TemporaryDirectory() as td:
            source = Path(td) / "pan.cpp"; exe = Path(td) / "pan"
            source.write_text(code)
            subprocess.run(["c++", "-std=c++11", "-I", str(ROOT / "firmware/uno_v4_mod"), str(source), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
        subprocess.run([sys.executable, str(ROOT / "scripts/check_uno_build.py")], check=True, cwd=ROOT)
    elif mode == "simulation":
        from simulate_active_roam import verify
        verify()
    else:
        raise ValueError(mode)
    print(f"active {mode} verification passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
