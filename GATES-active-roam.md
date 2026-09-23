# Gates: active sonar roaming

Scope: Research active perception through alphaXiv, implement observation-aware continuous roaming and nonblocking pan telemetry, verify offline, then test supervised low-speed motion if the hardware can be prepared.

- [x] R1: Research distinguishes published findings from adaptations for this single-sonar car.
  EVIDENCE: Reviewed RESEARCH-active-roam.md against alphaXiv/arXiv 2010.04977 and 2011.05228 abstracts and 2403.13929 primary full text. Drone/stereo/CBF results are explicitly not claimed for this car; adaptation, simulation assumptions and the unresolved faster-wheel stall are separate.

- [x] R2: Planner scenarios exercise early glances, confirmed openings, unknown and expired directions, noisy sonar, stopping distance, and smooth steering.
  CHECK: .venv/bin/python scripts/check_active_roam.py planner
  EXPECT: active planner verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output=Ran 12 tests in 0.001s | OK

- [x] R3: Stream integration rejects old, duplicate and moving-servo measurements; lost telemetry and emergency stop prevent continued motion; battery readings do not impose a driving cutoff (user-requested change); legacy safety tests pass.
  CHECK: .venv/bin/python scripts/check_active_roam.py integration
  EXPECT: active integration verification passed
  EVIDENCE: Re-executed after removing the battery cutoff on 2026-09-22; exit=0; cwd=~/Documents/personal/car; output=Ran 18 tests in 0.003s | OK | active integration verification passed. Regression checks confirm wheel commands remain permitted at 6561 mV, 6500 mV, zero and missing battery readings with a clear observed path. Planner suite also re-executed: 12 tests passed.

- [x] R4: Nonblocking servo timing and telemetry state pass host-compiled checks; UNO firmware compiles within flash and RAM limits using AVR core 1.8.3.
  CHECK: .venv/bin/python scripts/check_active_roam.py firmware
  EXPECT: active firmware verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output=uno build verification passed | active firmware verification passed

- [x] R5: Deterministic closed-loop simulations measure path progress, contacts, stops and steering changes using the implemented controller and independent world geometry.
  CHECK: .venv/bin/python scripts/check_active_roam.py simulation
  EXPECT: active simulation verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output={"name": "noisy_obstacle", "contacts": 0, "distance_cm": 549.2614987096447, "stops": 10, "steering_reversals": 7, "glances": 69, "max_pwm": 110.0, "max_step_up": 5.000000000000078, "min_x": 40.0, "max_x": 537.1696841101449, "end_x": 537.169

- [ ] R6: Supervised physical run verifies valid angle-tagged telemetry, early look-before-turn behavior, bounded speed and stop on exit; report actual outcomes without substituting simulation.
  EVIDENCE: Original 60-second run stalled (`build/runs/active-20260922-160415/`). Scan-loop repair passes 37 tests, recorded-input replay and 3 simulations; see `.unlazy/stall-fix/GATES.md`. Updated observe run `active-20260922-161352` passed for 12 active seconds and 88 frames. Motion retests `active-20260922-161416` and `active-20260922-161535` lost all replies after N2 and N4; stop sends were unacknowledged. Subsequent TCP100 and TCP80 probes timed out. Physical motion verification remains unmet.

Work order: research and protocol diagnosis -> firmware and measurement integrity -> planner and runner -> fault tests and simulation -> supervised hardware run -> documentation and evidence.
