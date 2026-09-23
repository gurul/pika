#!/usr/bin/env python3
"""G1: mapping core against a simulated room. Prints the marker only if every
assertion holds."""
import math, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import numpy as np
from mapping import Grid, Matcher, RoomSim, next_waypoint, frontiers, render

rng = np.random.default_rng(7)
room = RoomSim(w=300, h=240, boxes=[(40, -100, 90, -50)])   # a box in the room
fails = []

def check(cond, msg):
    if not cond:
        fails.append(msg); print("FAIL:", msg)
    else:
        print("ok:  ", msg)

# --- build a map from known poses (lawnmower inside the room) --------------
grid = Grid(size_cm=600, cell_cm=5)
poses = [(x, y, th) for y in (-80, 0, 80) for x in (-100, -40, 20, 80) for th in (0, 90)]
t0 = time.time()
for p in poses:
    grid.integrate_scan(p, room.sweep(p, step=10, noise=3.0, rng=rng))
print(f"integrated {len(poses)} sweeps in {time.time() - t0:.2f} s")
P = grid.prob()

def at(x, y):
    r, c = grid.to_cell(x, y); return P[r, c]

# walls: points along each wall that lie within 150 cm of some pose
wall_pts = [(-150, y) for y in range(-100, 101, 20)] + [(150, y) for y in range(-100, 101, 20)] \
         + [(x, -120) for x in range(-140, 141, 20)] + [(x, 120) for x in range(-140, 141, 20)]
seen = [pt for pt in wall_pts if any(math.hypot(pt[0]-p[0], pt[1]-p[1]) < 130 for p in poses)]
occ_frac = np.mean([at(x, y) > 0.6 or max(at(x+dx, y+dy) for dx in (-5, 0, 5) for dy in (-5, 0, 5)) > 0.6 for x, y in seen])
check(occ_frac >= 0.6, f"wall points marked occupied: {occ_frac:.2f} of {len(seen)} (need >= 0.60)")
interior = [(x, y) for x in range(-120, 121, 20) for y in range(-90, 91, 20) if not (30 <= x <= 100 and -110 <= y <= -40)]
free_frac = np.mean([at(x, y) < 0.35 for x, y in interior])
check(free_frac >= 0.8, f"interior points marked free: {free_frac:.2f} (need >= 0.80)")
box_edge = [(65, -50), (65, -100), (40, -75), (90, -75)]
box_frac = np.mean([max(at(x+dx, y+dy) for dx in (-5, 0, 5) for dy in (-5, 0, 5)) > 0.6 for x, y in box_edge])
check(box_frac >= 0.5, f"box edges marked occupied: {box_frac:.2f} (need >= 0.50)")

# --- scan matcher recovers a perturbed pose ---------------------------------
m = Matcher(grid)
hits = 0; exy_all = []; eth_all = []; worse_all = []; trials = [(-60, -40, 0), (0, 40, 90), (60, 0, 180), (-20, -60, 270), (40, 60, 45), (-90, 20, 135), (100, -60, 300),
          (-120, 80, 20), (120, 90, 200), (0, -90, 100), (-50, 60, 315), (110, -20, 60)]
t0 = time.time()
for true in trials:
    scan = room.sweep(true, step=10, noise=3.0, rng=rng)
    prior = (true[0] + 8, true[1] - 6, true[2] + 10)
    est, s, ps = m.match(prior, scan)
    err_xy = math.hypot(est[0]-true[0], est[1]-true[1]); err_th = abs((est[2]-true[2]+180) % 360 - 180)
    ok = err_xy <= 8.0 and err_th <= 6.0
    hits += ok; exy_all.append(err_xy); eth_all.append(err_th)
    worse_all.append(err_xy > 10.0 or err_th > 10.0)
    print(f"  match true={true} prior_err=(8,-6,10) -> err {err_xy:.1f} cm, {err_th:.1f} deg  {'ok' if ok else 'MISS'}")
print(f"matching {len(trials)} poses took {time.time() - t0:.2f} s")
check(hits >= 0.5 * len(trials), f"matcher recovered {hits}/{len(trials)} poses within 8 cm / 6 deg (need >= 50%)")
check(np.mean(exy_all) <= 8.0 and np.mean(eth_all) <= 6.0, f"mean localization error {np.mean(exy_all):.1f} cm, {np.mean(eth_all):.1f} deg (need <= 8 cm, 6 deg)")
check(sum(worse_all) <= 0.2 * len(trials), f"estimates worse than the prior (10 cm, 10 deg): {sum(worse_all)}/{len(trials)} (need <= 20%)")

# --- negative control: matcher must not "improve" on a map with no structure -
empty = Grid(); est, s, ps = Matcher(empty).match((10, 10, 10), room.sweep((10, 10, 10)))
check(est == (10, 10, 10), "matcher keeps the prior on an empty map (negative control)")

# --- frontier planner on a partial map ---------------------------------------
part = Grid(size_cm=600, cell_cm=5)
p0 = (-100, -80, 0)
part.integrate_scan(p0, room.sweep(p0, step=10))
wp, path, front = next_waypoint(part, p0)
check(front.any(), f"partial map has frontier cells: {int(front.sum())}")
check(wp is not None and path is not None, f"planner found a reachable frontier waypoint: {wp}")
if wp is not None:
    d = math.hypot(wp[0]-p0[0], wp[1]-p0[1])
    check(5 <= d <= 60, f"waypoint is a short hop away: {d:.0f} cm")
    r, c = part.to_cell(*wp)
    check(part.logodds[r, c] < 0, "waypoint lies in free space")
# full map should have far fewer frontiers than the partial one
_, _, front_full = next_waypoint(grid, poses[0])
check(front_full.sum() < front.sum(), f"frontiers shrink as the map fills: {int(front_full.sum())} < {int(front.sum())}")

os.makedirs("build", exist_ok=True)
render(grid, [(p[0], p[1], p[2]) for p in poses], out_path="build/sim_map.png")
render(part, [p0], path=path, front=front, out_path="build/sim_partial.png")
print("rendered build/sim_map.png and build/sim_partial.png")

if fails:
    sys.exit(f"{len(fails)} assertion(s) failed")
print("mapping verification passed")
