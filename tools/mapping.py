"""Occupancy-grid mapping and localization from servo-swept sonar.

Tuned on the simulator (scripts/check_mapping.py): 17 beams at 10-degree
steps, occupied evidence in a 6-degree cone, matcher sigma 10 cm. Measured
accuracy of one localization step: about 6-8 cm and 3-6 degrees. Heading is
the weak axis; the UNO's gyro would fix that but needs UNO firmware work.

Map: a log-odds occupancy grid (Moravec and Elfes, 1985). Each sonar reading
is a cone of half-angle BEAM_HALF_DEG around the beam axis. Cells inside the
cone and short of the measured range get free evidence, cells on the arc at
the measured range get occupied evidence, both weighted down toward the edge
of the cone. A no-echo reading marks the near part of the cone weakly free.

Pose: the car has no odometry, so each move is predicted from calibrated
timing and then corrected by matching the next sweep against the map over a
small brute-force window (correlative scan matching on a smoothed map).

Frontiers: free cells that touch unknown cells. Exploration drives to the
nearest reachable one until none is left.

Frame: x forward from the start pose, y to the car's left, headings in
degrees counter-clockwise, 0 = the start heading. Servo 90 is straight ahead
and angles above 90 look left, so beam heading = pose heading + servo - 90.
"""
import math
from collections import deque

import numpy as np

BEAM_HALF_DEG = 15.0
OCC_HALF_DEG = 6.0         # narrower cone for occupied evidence (echo is usually near the axis)
MAX_RANGE = 150.0          # firmware cap; a reading of 150 means no echo
NO_ECHO_FREE_CM = 110.0    # how far a no-echo reading is trusted as free
NO_ECHO_W = 0.8            # weight of a no-echo reading relative to an echo
RANGE_TOL = 6.0            # thickness of the occupied arc, cm
L_FREE, L_OCC = 0.55, 1.4  # log-odds increments (occupied is rarer, so stronger)
L_MIN, L_MAX = -5.0, 5.0
UNKNOWN_BAND = 0.25        # |log-odds| below this counts as unknown
FREE_THRESH, OCC_THRESH = -0.45, 0.8


def beam_heading(pose, servo_deg):
    return pose[2] + (servo_deg - 90.0)


class Grid:
    def __init__(self, size_cm=600, cell_cm=5):
        self.cell = float(cell_cm)
        self.n = int(size_cm // cell_cm)
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.origin = self.n // 2      # world (0,0) sits at cell (origin, origin)
        ys, xs = np.mgrid[0:self.n, 0:self.n]
        self._cx = (xs - self.origin) * self.cell      # world x of each cell centre
        self._cy = (ys - self.origin) * self.cell

    # -- coordinates ------------------------------------------------------
    def to_cell(self, x, y):
        return int(round(y / self.cell)) + self.origin, int(round(x / self.cell)) + self.origin

    def in_bounds(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    # -- inverse sensor model ---------------------------------------------
    def integrate_beam(self, pose, servo_deg, r):
        if r is None:
            return
        x0, y0 = pose[0], pose[1]
        hdg = math.radians(beam_heading(pose, servo_deg))
        no_echo = r >= MAX_RANGE
        reach = NO_ECHO_FREE_CM if no_echo else r + RANGE_TOL + self.cell
        # bounding box of the cone
        rad = int(reach / self.cell) + 2
        rc, cc = self.to_cell(x0, y0)
        r0, r1 = max(0, rc - rad), min(self.n, rc + rad + 1)
        c0, c1 = max(0, cc - rad), min(self.n, cc + rad + 1)
        if r0 >= r1 or c0 >= c1:
            return
        dx = self._cx[r0:r1, c0:c1] - x0
        dy = self._cy[r0:r1, c0:c1] - y0
        d = np.hypot(dx, dy)
        ang = np.degrees(np.arctan2(dy, dx) - hdg)
        ang = (ang + 180.0) % 360.0 - 180.0
        w = 1.0 - (np.abs(ang) / BEAM_HALF_DEG) ** 2        # Elfes-style angular falloff
        in_cone = (np.abs(ang) <= BEAM_HALF_DEG) & (d > self.cell)
        block = self.logodds[r0:r1, c0:c1]
        if no_echo:
            free = in_cone & (d < NO_ECHO_FREE_CM)
            block[free] -= NO_ECHO_W * L_FREE * w[free]
        else:
            free = in_cone & (d < r - 0.5 * self.cell)
            occ = (np.abs(ang) <= OCC_HALF_DEG) & (d > self.cell) & (d >= r - 0.5 * self.cell) & (d <= r + RANGE_TOL)   # surface at r, band behind it
            dist_w = 1.0 - 0.5 * (r / MAX_RANGE)              # far echoes are less certain
            block[free] -= L_FREE * w[free]
            block[occ] += L_OCC * w[occ] * dist_w
        np.clip(block, L_MIN, L_MAX, out=block)

    def integrate_scan(self, pose, scan):
        for servo_deg, r in scan:
            self.integrate_beam(pose, servo_deg, r)

    # -- classification ---------------------------------------------------
    def occupied(self, thresh=OCC_THRESH):
        return self.logodds > thresh

    def free(self, thresh=FREE_THRESH):
        return self.logodds < thresh

    def unknown(self):
        return np.abs(self.logodds) < UNKNOWN_BAND

    def coverage(self):
        """Fraction of cells within 150 cm of any visited pose that are classified."""
        return float(np.mean(~self.unknown()))


# ------------------------------------------------------------------ matcher

class Matcher:
    """Correlative scan matching with a beam model. A candidate pose is scored
    by predicting each sonar reading from the map the way the sensor works
    (nearest occupied cell anywhere in the beam cone) and comparing it with
    the measured range. This avoids the bias of endpoint matching against
    wide sonar arcs. Coarse-to-fine search; a quadratic prior keeps directions
    the sweep cannot observe (sliding along a bare wall) at the prediction."""

    CONE = np.array([-12.0, -6.0, 0.0, 6.0, 12.0])
    RANGES = np.arange(5.0, MAX_RANGE + 0.1, 5.0)

    def __init__(self, grid, xy_window=15.0, th_window=20.0, sigma_cm=10.0,
                 prior_xy_cm=12.0, prior_th_deg=8.0):
        self.grid = grid
        self.xy_window, self.th_window = xy_window, th_window
        self.sigma = sigma_cm
        self.prior_xy, self.prior_th = prior_xy_cm, prior_th_deg

    def predict(self, cand, servos, occ):
        """Expected range per (candidate, beam) from ray casts through occ."""
        hdg = cand[:, 2][:, None, None] + (servos[None, :, None] - 90.0) + self.CONE[None, None, :]
        hr = np.radians(hdg).astype(np.float32)                        # (N,B,C)
        cos, sin = np.cos(hr)[..., None], np.sin(hr)[..., None]
        xs = cand[:, 0, None, None, None] + self.RANGES * cos          # (N,B,C,R)
        ys = cand[:, 1, None, None, None] + self.RANGES * sin
        rr = np.rint(ys / self.grid.cell).astype(np.int32) + self.grid.origin
        cc = np.rint(xs / self.grid.cell).astype(np.int32) + self.grid.origin
        inb = (rr >= 0) & (rr < self.grid.n) & (cc >= 0) & (cc < self.grid.n)
        hit = np.zeros(rr.shape, dtype=bool)
        hit[inb] = occ[rr[inb], cc[inb]]
        first = np.where(hit.any(-1), self.RANGES[np.argmax(hit, axis=-1)], MAX_RANGE)
        return first.min(-1)                                            # (N,B)

    def score_many(self, cand, scan, occ):
        servos = np.array([s for s, r in scan if r is not None], dtype=np.float32)
        meas = np.array([r for s, r in scan if r is not None], dtype=np.float32)
        exp_r = self.predict(cand, servos, occ)
        echo = meas < MAX_RANGE
        diff = exp_r - meas[None, :]
        like_echo = np.exp(-(diff ** 2) / (2 * self.sigma ** 2))
        # measured no-echo: fine if the map also predicts nothing near; penalise a
        # predicted obstacle, more so the closer it is
        like_none = np.exp(-((MAX_RANGE - exp_r) ** 2) / (2 * 45.0 ** 2))
        like = np.where(echo[None, :], like_echo, like_none)
        return like.sum(1)

    def score(self, pose, scan, occ=None):
        occ = self.grid.occupied() if occ is None else occ
        return float(self.score_many(np.array([pose], dtype=np.float32), scan, occ)[0])

    def _search(self, prior, scan, occ, xy_w, xy_s, th_w, th_s):
        dth, dx, dy = np.meshgrid(np.arange(-th_w, th_w + 1e-6, th_s),
                                  np.arange(-xy_w, xy_w + 1e-6, xy_s),
                                  np.arange(-xy_w, xy_w + 1e-6, xy_s), indexing="ij")
        dx, dy, dth = dx.ravel(), dy.ravel(), dth.ravel()
        cand = np.stack([prior[0] + dx, prior[1] + dy, prior[2] + dth], 1).astype(np.float32)
        s = self.score_many(cand, scan, occ)
        s -= 0.5 * ((dx ** 2 + dy ** 2) / self.prior_xy ** 2 + (dth / self.prior_th) ** 2)
        i = int(np.argmax(s))
        return (float(cand[i, 0]), float(cand[i, 1]), float(cand[i, 2])), float(s[i])

    def match(self, prior, scan):
        """Return (pose, score, prior_score). Keeps the prior when the map has
        too little structure to say otherwise."""
        occ = self.grid.occupied()
        if float(occ.sum()) < 25:                 # too little structure to argue with the prediction
            return tuple(prior), 0.0, 0.0
        prior_score = self.score(prior, scan, occ)
        coarse, _ = self._search(prior, scan, occ, self.xy_window, 5.0, self.th_window, 5.0)
        fine, s = self._search(coarse, scan, occ, 5.0, 2.5, 5.0, 2.5)
        # the prior penalty was measured from `coarse` in the fine pass; re-score
        # against the original prior so the comparison is honest
        dx, dy, dth = fine[0] - prior[0], fine[1] - prior[1], fine[2] - prior[2]
        s = self.score(fine, scan, occ) - 0.5 * ((dx ** 2 + dy ** 2) / self.prior_xy ** 2 + (dth / self.prior_th) ** 2)
        if s <= prior_score + 0.75:               # demand a clear win before overriding the prediction
            return tuple(prior), prior_score, prior_score
        return fine, s, prior_score


# ---------------------------------------------------------------- planning

def inflate(mask, cells):
    out = mask.copy()
    for _ in range(cells):
        m = out.copy()
        m[1:, :] |= out[:-1, :]; m[:-1, :] |= out[1:, :]
        m[:, 1:] |= out[:, :-1]; m[:, :-1] |= out[:, 1:]
        out = m
    return out


def frontiers(grid, robot_radius_cm=16.0):
    """Cells that are free, not too close to obstacles, and touch unknown."""
    occ = inflate(grid.occupied(), int(robot_radius_cm / grid.cell))
    free = grid.free() & ~occ
    unk = grid.unknown()
    touch = np.zeros_like(unk)
    touch[1:, :] |= unk[:-1, :]; touch[:-1, :] |= unk[1:, :]
    touch[:, 1:] |= unk[:, :-1]; touch[:, :-1] |= unk[:, 1:]
    return free & touch, free


def bfs_path(free, start, goal_mask):
    """Shortest 4-connected path over `free` from start to any goal cell."""
    n = free.shape[0]
    prev = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if goal_mask[cur]:
            path = []
            while cur is not None:
                path.append(cur); cur = prev[cur]
            return path[::-1]
        r, c = cur
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < n and 0 <= nc < n and free[nr, nc] and (nr, nc) not in prev:
                prev[(nr, nc)] = cur
                q.append((nr, nc))
    return None


def next_waypoint(grid, pose, lookahead_cm=35.0):
    """Pick the nearest frontier and return (waypoint_xy, path_cells, frontier_mask).
    waypoint is None when the map has no reachable frontier left."""
    front, free = frontiers(grid)
    start = grid.to_cell(pose[0], pose[1])
    # the robot's own footprint is free by definition
    r0, c0 = start
    free[max(0, r0 - 3):r0 + 4, max(0, c0 - 3):c0 + 4] = True
    if not front.any():
        return None, None, front
    path = bfs_path(free, start, front)
    if not path:
        return None, None, front
    k = min(len(path) - 1, max(1, int(lookahead_cm / grid.cell)))
    rr, cc = path[k]
    return ((cc - grid.origin) * grid.cell, (rr - grid.origin) * grid.cell), path, front


# ---------------------------------------------------------------- rendering

def render(grid, trail, path=None, front=None, out_path=None, scale=2):
    """Draw the map to a PNG: white free, black occupied, grey unknown,
    frontier cells in cyan, path in orange, trail in blue, robot in red."""
    from PIL import Image, ImageDraw
    p = grid.prob()
    img = (255 * (1.0 - p)).astype(np.uint8)
    rgb = np.stack([img, img, img], axis=-1)
    unk = grid.unknown()
    rgb[unk] = (150, 150, 150)
    if front is not None:
        rgb[front] = (0, 200, 220)
    im = Image.fromarray(rgb[::-1, :, :])          # y up
    im = im.resize((grid.n * scale, grid.n * scale), Image.NEAREST)
    d = ImageDraw.Draw(im)
    def px(x, y):
        r, c = grid.to_cell(x, y)
        return c * scale + scale // 2, (grid.n - 1 - r) * scale + scale // 2
    if path:
        pts = [px((c - grid.origin) * grid.cell, (r - grid.origin) * grid.cell) for r, c in path]
        if len(pts) > 1:
            d.line(pts, fill=(255, 140, 0), width=2)
    if len(trail) > 1:
        d.line([px(x, y) for x, y, _ in trail], fill=(60, 90, 255), width=2)
    if trail:
        x, y, th = trail[-1]
        cx, cy = px(x, y)
        d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], outline=(230, 30, 30), width=2)
        ex, ey = px(x + 20 * math.cos(math.radians(th)), y + 20 * math.sin(math.radians(th)))
        d.line([(cx, cy), (ex, ey)], fill=(230, 30, 30), width=2)
    if out_path:
        im.save(out_path)
    return im


# ---------------------------------------------------------------- simulator

class RoomSim:
    """Rectangular room with boxes, for tests. Walls are axis-aligned segments."""

    def __init__(self, w=300, h=240, boxes=()):
        self.segs = [((-w / 2, -h / 2), (w / 2, -h / 2)), ((w / 2, -h / 2), (w / 2, h / 2)),
                     ((w / 2, h / 2), (-w / 2, h / 2)), ((-w / 2, h / 2), (-w / 2, -h / 2))]
        for (x0, y0, x1, y1) in boxes:
            self.segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]

    def _ray(self, x, y, hdg_deg):
        h = math.radians(hdg_deg)
        dx, dy = math.cos(h), math.sin(h)
        best = MAX_RANGE
        for (ax, ay), (bx, by) in self.segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((ax - x) * ey - (ay - y) * ex) / den
            u = ((ax - x) * dy - (ay - y) * dx) / den
            if t > 0 and 0 <= u <= 1 and t < best:
                best = t
        return best

    def sweep(self, pose, step=15, noise=0.0, rng=None):
        """Sonar returns the nearest surface anywhere inside the cone."""
        out = []
        for servo in range(10, 171, step):
            hdg = beam_heading(pose, servo)
            r = min(self._ray(pose[0], pose[1], hdg + a) for a in (-12, -6, 0, 6, 12))
            if r < MAX_RANGE and noise and rng is not None:
                r = max(5.0, r + rng.normal(0, noise))
            out.append((servo, int(round(min(r, MAX_RANGE)))))
        return out
