# Fix active-roam stall

Contract: preserve v4 wire protocol, angle/sequence freshness, unknown-space rejection,
emergency stop, and the user's removal of the battery cutoff. No firmware changes
unless host-side evidence proves one necessary. Python 3.12 at .venv/bin/python;
checks run from project root with /bin/sh. No Git repository exists to sync.

Root OPEN: integrate verified diagnosis, planner fix, regression checks, simulations,
and an authorized bounded physical retest if the car is reachable and USB is absent.

- leaf-1 VERIFIED, Needs none: primary owns tools/active_roam.py,
  tools/tests/test_active_stall.py, tools/tests/test_active_roam.py, scripts/check_active_stall.py, README.md,
  HANDOVER.md, GATES-active-roam.md. Reproduce the recorded lock, ensure search
  progresses beyond rejected headings, and validate unchanged stop behavior.
- leaf-2 VERIFIED, Needs none: reviewer owns build/stall-review.md only. Independently
  inspect original run and transport/control timing. Read-only code review;
  root receives evidence and reviews conclusions before integration.
- leaf-3 VERIFIED, Needs none: research agent owns build/sonar-methods-research.md;
  investigate active sonar scanning and sensor scheduling from alphaXiv and primary
  research, with applicability to a single panning sonar, gyro and no wheel encoders.
- leaf-4 VERIFIED, Needs none: research agent owns build/navigation-methods-research.md;
  compare VFH+, dynamic-window and alternative reactive controllers for this car,
  rank improvements and identify assumptions that this hardware cannot satisfy.

User explicitly requested a research swarm during the fix. Each research leaf must
cite primary source links, distinguish papers from proposed adaptations, and provide
falsifiable evaluation criteria. Root integrates findings in RESEARCH-stall-fix.md.

Shared interfaces: ActivePlanner.ingest/step, Decision and Config retain compatibility
with ActiveSession and scripts/simulate_active_roam.py. Diagnostic suggestions are
not approval to drive; only primary may connect to hardware.

Work order: evidence and reproduction -> policy fix -> regression/simulation ->
review integration -> physical verification -> docs and measured report.

Latest state: root remains OPEN. All four software/research leaves verified. G5
physical retest failed on lost replies; both camera/control ports now time out.
User hardware-state question pending. No acceptance gates abandoned.
