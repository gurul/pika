#!/usr/bin/env python3
"""Verify an explore run directory. Modes: dry, stale, estop, map."""
import json, os, sys

run_dir, mode = sys.argv[1], sys.argv[2].split("=")[-1] if len(sys.argv) > 2 else "map"
if sys.argv[2:3] == ["--mode"]:
    mode = sys.argv[3]
fails = []
def check(cond, msg):
    (print("ok:   " + msg) if cond else (fails.append(msg), print("FAIL: " + msg)))

log = [json.loads(l) for l in open(os.path.join(run_dir, "log.jsonl"))]
summary = json.load(open(os.path.join(run_dir, "summary.json")))
ev = lambda name: [r for r in log if r["event"] == name]
pulses = [r for r in ev("pulse") if not r.get("dry")]
dry_pulses = [r for r in ev("pulse") if r.get("dry")]
scans = ev("scan"); ends = ev("end")
check(len(ends) == 1 and ev("start"), "log has a start and one end")
check(any(r["event"] == "stop" and r.get("reason") == "end" for r in log), "run ended with an explicit stop")
check(os.path.isfile(os.path.join(run_dir, "map.png")) and os.path.isfile(os.path.join(run_dir, "map.npy")), "map.png and map.npy written")
check(os.path.isfile(os.path.join(run_dir, "poses.jsonl")) and sum(1 for _ in open(os.path.join(run_dir, "poses.jsonl"))) >= 1, "poses.jsonl has entries")

if mode == "dry":
    check(len(scans) >= 1, f"at least one sweep: {len(scans)}")
    check(len(pulses) == 0 and summary["motion_frames"] == 0, f"no motion frame sent (dry run): {summary['motion_frames']}")
    check(len(dry_pulses) >= 1, f"the planner would have moved: {len(dry_pulses)} dry pulses")
    check(summary.get("frames", 0) >= 3, f"camera frames captured: {summary.get('frames')}")
    marker = "run verification passed"
elif mode in ("stale", "estop"):
    faults = ev("fault")
    check(len(faults) == 1, "fault was injected once")
    if faults:
        tf = faults[0]["t"]
        before = [p for p in pulses if p["t"] < tf]; after = [p for p in pulses if p["t"] >= tf]
        check(len(before) >= 1, f"car was driving before the fault: {len(before)} pulses")
        check(len(after) == 0, f"no motion frame after the fault: {len(after)}")
        stops = [r for r in ev("stop") if r["t"] >= tf and r.get("reason") in (mode, "estop", "stale")]
        check(stops and stops[0]["t"] - tf <= 1.5, f"stop within 1.5 s of the fault: {round(stops[0]['t'] - tf, 2) if stops else 'none'} s")
        check(summary["reason"] in (mode, "estop", "stale"), f"run ended because of the fault: {summary['reason']}")
    marker = f"{mode} verification passed"
else:
    check(len(scans) >= 12, f"scan-move cycles: {len(scans)} (need >= 12)")
    check(summary["motion_frames"] >= 12, f"motion frames: {summary['motion_frames']}")
    check(summary.get("max_correction_cm", 99) <= 25 and summary.get("max_correction_deg", 99) <= 25,
          f"localization corrections inside the search window: max {summary.get('max_correction_cm')} cm, {summary.get('max_correction_deg')} deg")
    frames = os.listdir(os.path.join(run_dir, "frames"))
    check(len(frames) >= 12, f"frames saved: {len(frames)}")
    check(summary["occupied_cells"] >= 40 and summary["free_cells"] >= 400, f"map has structure: {summary['occupied_cells']} occupied, {summary['free_cells']} free cells")
    check(not summary["estop"], "no e-stop during the run")
    check(summary["reason"] in ("duration", "complete"), f"ended by time or completion: {summary['reason']}")
    marker = "map verification passed"

if fails:
    sys.exit(f"{len(fails)} check(s) failed")
print(marker)
