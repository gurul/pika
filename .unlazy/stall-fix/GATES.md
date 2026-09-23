# Gates: physical stall fix

- [x] G1: Original stationary lock reproduces and the fixed planner expands search to a usable observed opening without driving into rejected clearance.
  CHECK: .venv/bin/python scripts/check_active_stall.py --replay build/runs/active-20260922-160415/log.jsonl
  EXPECT: active stall verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output=Ran 7 tests in 0.047s | OK

- [x] G2: Existing planner and transport safety regressions pass, including low battery remaining permitted.
  CHECK: .venv/bin/python scripts/check_active_roam.py planner && .venv/bin/python scripts/check_active_roam.py integration
  EXPECT: active integration verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output=Ran 18 tests in 0.003s | OK

- [x] G3: Independent geometry simulation preserves progress, collision avoidance, speed and acceleration bounds.
  CHECK: .venv/bin/python scripts/check_active_roam.py simulation
  EXPECT: active simulation verification passed
  EVIDENCE: exit=0; shell=/bin/sh; cwd=~/Documents/personal/car; path=421131676988/21 entries; output={"name": "noisy_obstacle", "contacts": 0, "distance_cm": 603.9725758779958, "stops": 6, "steering_reversals": 4, "glances": 60, "max_pwm": 110.0, "max_step_up": 5.000000000000078, "min_x": 40.0, "max_x": 538.380279634351, "end_x": 538.38027

- [x] G4: Independent review findings are integrated or resolved with evidence.
  EVIDENCE: Reviewed build/stall-review.md and independently inspected source/logs. Initial44cm fix extended to50/55/60cm side-glance feasibility cases; pending pan state matches emitted recovery look; snapshot/command logs added; lost-ACK confirmation and estop error reporting verified. Independent reviewer reran7 regressions, persistent13s fixtures and7 adversarial pan cases. Remaining motion-time connection failure is explicitly carried by G5 rather than attributed without evidence.

- [ ] G5: Bounded physical retest demonstrates escape from the recorded scan stall and finishes stopped; report measured motion and any remaining limitation.
  EVIDENCE: Physical navigation remains UNMET. Link recovered after user power cycle. User reported car stationary despite gyro rotation in run174335; stationary test measured -43.664 deg/s drift. N26 recalibration plus follow-up showed stable stationary heading; user confirmed a small movement from150ms/140PWM pulse, about26deg gyro change (build/stall-motor-response.json). Low-speed run174536 only made limited turns. Fast run174634 sent zero wheel commands because52/57 forward samples were no-echo; exit stop acknowledged. User then placed object ahead: three N21 replies13cm. Sonar target detection confirmed; no fast burst sent at that close distance. Need a clear measured lane for speed test and a solution for open-space no-echo behavior. Startup gyro validation remains outstanding. Latest four-minute run active-20260922-175119 was deliberately interrupted after145.577 active seconds following user report of jerky stop-swivel-move behavior; final N100 stop, stream-off and center acknowledged. User now plans to install lidar first; navigation work paused at their direction. No smooth physical behavior claimed.

- [x] G6: Research swarm findings are synthesized into ranked, source-supported methods with hardware assumptions and measurable next steps.
  EVIDENCE: RESEARCH-stall-fix.md synthesizes three independent reports; primary agent opened alphaXiv ASAA, original Curved Openspace, DWA and CVM, verifying scan-rate and admissibility claims. Method table separates implementations/adaptations and missing hardware assumptions; ranked follow-up prioritizes link reliability, measured motor response, arc selection and scan deadlines. No paper performance is claimed for this car.
