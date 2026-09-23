# Restore VFH behavior with v4 sensing
Root: OPEN. Current lidar work stays paused. No firmware flash required.

| Leaf | State | Needs | Ownership |
| --- | --- | --- | --- |
| 1 controller and integration (root) | IN-FLIGHT | none | tools/stream_roam.py, tools/active_roam.py, README.md, HANDOVER.md, todo.md |
| 2 independent behavior tests | READY | none | tools/tests/test_stream_roam.py, scripts/check_stream_roam.py, build/vfh-restore-simulation/** |
| 3 independent review | IN-FLIGHT | none | build/vfh-restore-review.md |
| 4 live sonar feed | READY | none | tools/sonar_feed.py, web/sonar-feed.html, tools/tests/test_sonar_feed.py |

Interface: tools.stream_roam exports VfhPlanner(cfg=None) and VfhConfig. API matches ActivePlanner: ingest(t,now), step(now,estop=False) -> Decision(left,right,look,reason,heading,clearance,spin), plus same fields consumed by ActiveSession. Uses existing angle-tagged ingestion and command adapter. Entry python tools/stream_roam.py --observe/--duration/--vmax uses shared active runner with injected planner/config factory; old active planner remains comparison baseline and old roam.py unchanged.

Restore VFH density hysteresis and cost 5*target+2*heading+2*previous, continuous differential curves and useful outer-wheel floor. Preserve immediate close/stale/ground/estop and unknown-space rejection, no battery cutoff. Scanning must reserve an actual bounded round-trip time and consume that budget rather than charging a fresh blind period on every tick. Wide stationary search and recovery only toward observed clearance. Finite-duration cleanup and active pan telemetry retain transport protections. Do not turn tests into old exact-behavior assertions. Root owns device access only.

Test toolchain .venv/bin/python, shell /bin/sh, cwd car. No Git repository available to sync. Independent simulation includes servo settling, 10Hz feedback, wheel stiction (e.g. below55 PWM), geometry contact oracle with positive control. Report real-run evidence separately; successful simulation is not smooth hardware proof. Reviewer may read draft while implementation develops, then review final patch. No automatic physical long run.

User added live radar view inspired by https://github.com/TechTalkies/YouTube/tree/41f0d4115bbe2ca86aa9ff43073dbd843450151b/19%20Radar . Feed must tail shared log.jsonl telemetry, not open another car TCP socket. Existing tick telemetry fields: t wall-time, mono, pan10..170, settled, dist0=unknown, yaw, seq; tick decision includes reason,left,right,look. Feed owns its local HTTP endpoint and browser display only, root starts it and verifies real UI. Bind127.0.0.1 and no motor controls. Indicate stale/unsettled/missing data honestly. CLI --run-dir path and --port. Can keep latest points briefly but age them and mark stale; use actual pan angle for sweeping arm. Root may adapt interface after discussion.
