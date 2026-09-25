"""Follow one chosen person with the car's camera.

The car's camera serves a single MJPEG viewer, so `Stream` is that viewer and
hands each frame on to the browser and to the tracker. `PersonTracker` finds
people with Apple's Vision framework and keeps hold of the one that was
clicked, by appearance (an image feature print of their crop) and by where
they were last. `Follower` turns the tracked box into a bearing and sends it
to the UNO as N=29 several times a second; the UNO holds the gap on its own
sonar and stops when the bearings stop (firmware/uno_v4_mod/FollowDrive.h).

Needs pyobjc-framework-Vision and pyobjc-framework-Quartz (see README).
"""
import json, math, threading, time, urllib.request
from array import array

HFOV_DEG = 62.0          # OV3660 horizontal field of view, approximate
SEND_HZ = 8              # bearing rate; the UNO drops a bearing older than 0.5 s
HOLD_S = 0.3             # a detector miss this short keeps the last bearing
SEARCH_S = 3.0           # after that, look toward where they left, for this long
SEARCH_TURN_S = 0.35     # searching turns in pulses: this long turning,
SEARCH_LOOK_S = 0.5      # then this long still, so the camera gets a sharp frame
PAGE_S = 2.0             # stop following when the controller page stops checking in
EDGE_DEG = 12            # a person last seen this far off-axis left the frame sideways
TRANSPORT_MS = 80        # capture on the board to arrival here, estimated; the UNO
                         # turns a bearing into a gyro heading as of (arrival - this)
JUMP = 0.2               # a match this far (image widths) from the last one needs a second frame to agree


# --- geometry and decisions (pure, unit tested) ------------------------------

def bearing_deg(box, hfov=HFOV_DEG, mirror=False):
    """Box (x, y, w, h), normalized with x to the right. + = left of the camera axis."""
    cx = box[0] + box[2] / 2
    b = (0.5 - cx) * hfov
    return -b if mirror else b


def iou(a, b):
    ax2, ay2, bx2, by2 = a[0] + a[2], a[1] + a[3], b[0] + b[2], b[1] + b[3]
    ix = max(0.0, min(ax2, bx2) - max(a[0], b[0]))
    iy = max(0.0, min(ay2, by2) - max(a[1], b[1]))
    inter = ix * iy
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def jumped(a, b):
    """The centres are more than JUMP apart."""
    return math.hypot(a[0] + a[2] / 2 - b[0] - b[2] / 2, a[1] + a[3] / 2 - b[1] - b[3] / 2) > JUMP


def contains(box, x, y):
    return box[0] <= x <= box[0] + box[2] and box[1] <= y <= box[1] + box[3]


def pick_at(boxes, x, y):
    """The box under a click; the smallest wins when people overlap, else the nearest."""
    hit = [b for b in boxes if contains(b, x, y)]
    if hit:
        return min(hit, key=lambda b: b[2] * b[3])
    if not boxes:
        return None
    near = min(boxes, key=lambda b: (b[0] + b[2] / 2 - x) ** 2 + (b[1] + b[3] / 2 - y) ** 2)
    cx, cy = near[0] + near[2] / 2, near[1] + near[3] / 2
    return near if abs(cx - x) < 0.15 and abs(cy - y) < 0.25 else None


# Distances between unit feature prints of upper-body boxes, 0 to 2. Measured
# 2026-09-25: one person over 30 frames of the car's camera, 0.12 to 0.39 from
# the first frame; different people on the test photo (tools/tests/fixtures),
# 0.70 to 0.97. Continuity (overlap with the last box) buys a looser threshold,
# because pose and light change frame to frame.
MATCH = 0.50
MATCH_NEAR = 0.62


def choose(cands, last_box):
    """cands: [(box, appearance distance)]. Returns the index of the chosen person or None."""
    best, best_score = None, None
    for i, (box, dist) in enumerate(cands):
        near = last_box is not None and iou(box, last_box) > 0.3
        if dist > (MATCH_NEAR if near else MATCH):
            continue
        score = dist - (0.15 if near else 0.0)
        if best_score is None or score < best_score:
            best, best_score = i, score
    return best


# --- camera range and the sonar reflex (pure, unit tested) ---------------------
#
# The camera is the sensor for the person: it always knows which thing is them.
# Their upper-body box height gives the range: box height x distance measured
# 32 to 41 across four floor runs, 2026-09-25 (0.28 at 129 cm, 0.30 at 114,
# 0.25 at 166, 0.33 at 97), so range = K_RANGE / height. The sonar is a narrow
# beam that also hits walls, furniture and the floor, so it never measures the
# person: it is only the reflex for something close ahead, on the UNO (brake
# and back off) and here (steer around it).

K_RANGE = 35.0           # box height x distance, cm
RANGE_SMOOTH = 0.5       # weight of the newest estimate
TELEMETRY_S = 0.5        # car samples older than this are not used

AVOID_CM = 30            # two front pings this close: something is in the way
PERSON_AHEAD_DEG = 12    # ... unless it is the person, straight ahead
PERSON_TOL_CM = 20       # ... at about the camera's range to them
TURN_DEG = 50            # go around it this far to one side
DETOUR_S = 1.0           # keep that heading this long after the way clears
DRIVE_RANGE_CM = 70      # the range sent while going around: a moderate speed
REJOIN_S = 2.0           # then steer back to where they were, this long at most
GIVE_UP_S = 6.0          # a detour that never clears ends here
FLIP_S = 5.0             # blocked again this soon: try the other side


def range_cm(box):
    """Camera range to a person from their upper-body box: cm, 15 to 400."""
    return max(15.0, min(400.0, K_RANGE / max(box[3], 1e-3)))


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class Avoider:
    """Going around something the sonar finds in the way.

    Headings are absolute, degrees, + = left (the gyro, sign-corrected). While
    it is active, `target` is the heading to steer for and `drive_range` the
    range to send, and the person's own bearing is set aside."""

    def __init__(self):
        self.state = "follow"
        self.target = None
        self.drive_range = None
        self.side = 1
        self.t0 = 0.0
        self.clear_at = None
        self.last_side, self.last_t = None, -1e9

    @staticmethod
    def blocked(pings, person):
        """Two agreeing close pings, and not explained by the person straight ahead."""
        a, b = pings
        if not (a and b and max(a, b) <= AVOID_CM):
            return False
        if person and person["fresh"] and abs(person["bearing"]) <= PERSON_AHEAD_DEG \
                and abs(person["range"] - max(a, b)) <= PERSON_TOL_CM:
            return False
        return True

    def step(self, now, pings, person, yaw):
        """pings: the last two front sonar readings (0 = no echo). person: None or
        {bearing, range, abs, fresh}. yaw: the car's heading now, or None."""
        blocked = self.blocked(pings, person)
        if self.state == "follow":
            if blocked and yaw is not None:
                if person and person["fresh"] and abs(person["bearing"]) >= 5:
                    side = 1 if person["bearing"] > 0 else -1       # go round on their side
                elif self.last_side is not None and now - self.last_t < FLIP_S:
                    side = -self.last_side                            # that side was blocked too
                else:
                    side = 1
                base = person["abs"] if person and person.get("abs") is not None else yaw
                self.state, self.side, self.t0, self.clear_at = "detour", side, now, None
                self.target, self.drive_range = wrap(base + side * TURN_DEG), DRIVE_RANGE_CM
                self.last_side, self.last_t = side, now
        elif self.state == "detour":
            self.clear_at = None if blocked else (self.clear_at if self.clear_at is not None else now)
            done = self.clear_at is not None and now - self.clear_at >= DETOUR_S
            if done or now - self.t0 > GIVE_UP_S:
                if person and person.get("abs") is not None:
                    self.state, self.t0 = "rejoin", now
                    self.target, self.drive_range = person["abs"], person["range"]
                else:
                    self.reset()
        elif self.state == "rejoin":
            if (person and person["fresh"]) or now - self.t0 > REJOIN_S:
                self.reset()
            elif blocked:
                self.state = "follow"
                return self.step(now, pings, person, yaw)
        return self.state

    def reset(self):
        self.state, self.target, self.drive_range = "follow", None, None


def frame_age_ms(arrived, now):
    """How long ago the frame was taken, for the UNO's heading history (at most 2 s)."""
    return max(0, min(2000, round((now - arrived) * 1000) + TRANSPORT_MS))


def hint_for(last_bearing, since_seen):
    """What to tell the car when this frame did not find them: (bearing, seen)."""
    if last_bearing is None:
        return 0, 0
    if since_seen <= HOLD_S:
        return round(last_bearing), 1
    if since_seen <= SEARCH_S and abs(last_bearing) >= EDGE_DEG:
        side = 30 if last_bearing > 0 else -30
        phase = (since_seen - HOLD_S) % (SEARCH_TURN_S + SEARCH_LOOK_S)
        return (side, 2) if phase < SEARCH_TURN_S else (side, 0)
    return 0, 0


# --- camera stream -----------------------------------------------------------

class Stream:
    """The one MJPEG connection to the car. Keeps the latest JPEG."""

    def __init__(self, url):
        self.url = url
        self.jpeg = None
        self.count = 0
        self.at = 0.0            # monotonic arrival of the latest frame
        self.error = None
        self.cond = threading.Condition()
        self.alive = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.alive:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    self.error = None
                    buf = b""
                    while self.alive:
                        chunk = r.read(16384)
                        if not chunk:
                            break
                        buf += chunk
                        while True:
                            a = buf.find(b"\xff\xd8")
                            z = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                            if a < 0 or z < 0:
                                buf = buf[a:] if a >= 0 else buf[-1:]
                                break
                            with self.cond:
                                self.jpeg = buf[a:z + 2]
                                self.at = time.monotonic()
                                self.count += 1
                                self.cond.notify_all()
                            buf = buf[z + 2:]
            except OSError as e:
                self.error = str(e)
            time.sleep(1.0)

    def wait(self, after, timeout=2.0):
        """The first frame newer than count `after`: (count, jpeg), or (after, None) on timeout."""
        with self.cond:
            self.cond.wait_for(lambda: self.count > after or not self.alive, timeout)
            return (self.count, self.jpeg) if self.count > after else (after, None)


# --- Vision -------------------------------------------------------------------

def vision_available():
    try:
        import Vision, Quartz  # noqa: F401
        return True, ""
    except ImportError as e:
        return False, f"{e}; install with: uv pip install --python .venv/bin/python pyobjc-framework-Vision pyobjc-framework-Quartz"


class Vis:
    """Thin wrapper over Vision. Boxes come back top-left based: (x, y, w, h), x right, y down.

    Carried over from buddy's bridge (cc_buddy_bridge/identity.py, vision.py): options
    are native nil, never a Python dict, which can throw on an absent Vision option;
    feature prints are pinned to revision 2 (768 unit-norm floats); and a long-running
    loop drains an autorelease pool per frame (`pool`) so Vision objects do not pile up.
    """

    def __init__(self):
        import objc, Vision, Quartz
        from Foundation import NSData
        self.V, self.Q, self.NSData = Vision, Quartz, NSData
        self.pool = objc.autorelease_pool

    def image(self, jpeg):
        data = self.NSData.dataWithBytes_length_(jpeg, len(jpeg))
        src = self.Q.CGImageSourceCreateWithData(data, None)
        return self.Q.CGImageSourceCreateImageAtIndex(src, 0, None) if src else None

    def people(self, cg):
        # Upper body, not full body: the camera tilts up, so a person close to the
        # car is head and shoulders. On the car's camera, upper body found the
        # person in 30 of 30 frames and full body in 22 (2026-09-25).
        req = self.V.VNDetectHumanRectanglesRequest.alloc().init()
        req.setUpperBodyOnly_(True)
        h = self.V.VNImageRequestHandler.alloc().initWithCGImage_options_(cg, None)
        ok, _ = h.performRequests_error_([req], None)
        out = []
        for o in (req.results() or []) if ok else []:
            r = o.boundingBox()
            x, y, w, hh = r.origin.x, r.origin.y, r.size.width, r.size.height
            out.append((x, 1.0 - y - hh, w, hh))
        return out

    def prints(self, cg, boxes):
        """One appearance vector per box, over the box's own pixels, unit length."""
        h = self.V.VNImageRequestHandler.alloc().initWithCGImage_options_(cg, None)
        reqs = []
        for (x, y, w, hh) in boxes:
            req = self.V.VNGenerateImageFeaturePrintRequest.alloc().init()
            req.setRevision_(self.V.VNGenerateImageFeaturePrintRequestRevision2)
            req.setRegionOfInterest_(((x, 1.0 - y - hh), (w, hh)))
            reqs.append(req)
        if not reqs:
            return []
        ok, _ = h.performRequests_error_(reqs, None)
        out = []
        for r in reqs:
            o = (r.results() or [None])[0] if ok else None
            out.append(unit(array("f", bytes(o.data()))) if o is not None and o.elementType() == 1 else None)
        return out


# pyobjc does not hand back computeDistance's float out-parameter, so the
# distance is computed here, as Vision does: Euclidean over the raw floats
# (unit length already at revision 2), 0 to 2.
def unit(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def distance(a, b):
    return math.sqrt(max(0.0, sum((x - y) ** 2 for x, y in zip(a, b))))


class PersonTracker:
    """Holds one person: a bank of their feature prints and their last box."""

    BANK = 5

    def __init__(self, vis):
        self.vis = vis
        self.bank = []
        self.last_box = None
        self.pending = None      # a match that jumped away from the track, awaiting a second frame
        self.boxes = []          # everyone in the latest frame, for the overlay

    @property
    def locked(self):
        return bool(self.bank)

    def clear(self):
        self.bank, self.last_box, self.pending = [], None, None

    def select(self, jpeg, x, y):
        """Lock onto the person under a click at normalized (x, y). Returns their box or None."""
        with self.vis.pool():
            return self._select(jpeg, x, y)

    def _select(self, jpeg, x, y):
        cg = self.vis.image(jpeg)
        if cg is None:
            return None
        self.boxes = self.vis.people(cg)
        box = pick_at(self.boxes, x, y)
        if box is None:
            return None
        fp = self.vis.prints(cg, [box])[0]
        if fp is None:
            return None
        self.bank, self.last_box, self.pending = [fp], box, None
        return box

    def scan(self, jpeg):
        """Before anyone is picked: just find the people, for the page to offer."""
        with self.vis.pool():
            cg = self.vis.image(jpeg)
            self.boxes = self.vis.people(cg) if cg is not None else []
        return self.boxes

    def update(self, jpeg):
        """Find them in a new frame. Returns (box, distance) or None."""
        with self.vis.pool():
            return self._update(jpeg)

    def _update(self, jpeg):
        cg = self.vis.image(jpeg)
        if cg is None:
            return None
        self.boxes = self.vis.people(cg)
        if not self.bank or not self.boxes:
            return None
        fps = self.vis.prints(cg, self.boxes)
        cands = [(b, min(distance(fp, t) for t in self.bank) if fp is not None else 9.0)
                 for b, fp in zip(self.boxes, fps)]
        i = choose(cands, self.last_box)
        if i is None:
            return None
        box, dist = cands[i]
        # Follow a person, not a sighting (buddy's follow.py): a match far from where
        # they just were is an outlier until the next frame puts them there too.
        if self.last_box is not None and jumped(box, self.last_box):
            confirmed = self.pending is not None and not jumped(box, self.pending)
            self.pending = None if confirmed else box
            if not confirmed:
                return None
        self.pending = None
        self.last_box = box
        # learn new poses only from confident matches, keeping the clicked print
        if dist < MATCH * 0.7 and fps[i] is not None:
            self.bank = [self.bank[0]] + (self.bank[1:] + [fps[i]])[-(self.BANK - 1):]
        return box, dist


# --- the loop -----------------------------------------------------------------

class Follower:
    """Runs the tracker on every frame and streams bearings to the car while on.

    Keeps the person as an absolute heading (the gyro at the moment the frame
    was taken, plus their bearing in it), so the car can steer back to them
    while they are out of the picture, and hands the sonar pings to the Avoider."""

    def __init__(self, stream, send, front_raw, mirror=False, hfov=HFOV_DEG, log=print, trace=None,
                 telemetry=None, yaw_left_sign=-1):
        self.stream, self.send, self.front_raw = stream, send, front_raw
        self.mirror, self.hfov, self.log = mirror, hfov, log
        self.trace = trace       # path of a JSONL file: one line per bearing sent
        self.telemetry = telemetry or (lambda: None)   # the car's latest {T_...} sample, if streaming
        self.yaw_left_sign = yaw_left_sign
        ok, why = vision_available()
        self.tracker = PersonTracker(Vis()) if ok else None
        self.unavailable = None if ok else why
        self.on = False
        self.lock = threading.Lock()
        self.state = {"on": False, "locked": False, "seen": False, "box": None, "boxes": [],
                      "bearing": None, "match": None, "hint": None, "fps": 0.0, "range": None,
                      "sonar": None, "avoid": "follow"}
        self.last_seen = None
        self.last_bearing = None
        self.seen_arrival = 0.0
        self.range = None
        self.person_abs = None
        self.avoider = Avoider()
        self.yaws = []           # (monotonic, heading + = left) from telemetry, last 2 s
        self.pings = (0, 0)
        self._last_mono = None
        self.touched = time.monotonic()
        self.alive = True
        if self.tracker:
            threading.Thread(target=self._run, daemon=True).start()

    def select(self, x, y):
        _, jpeg = self.stream.wait(self.stream.count - 1, 2.0)
        if jpeg is None:
            raise OSError("no video from the car")
        with self.lock:
            box = self.tracker.select(jpeg, x, y)
            self.state.update(boxes=self.tracker.boxes, box=box, locked=box is not None)
            if box is not None:
                self.last_seen, self.last_bearing = time.monotonic(), bearing_deg(box, self.hfov, self.mirror)
                self.seen_arrival = self.stream.at
                self.range = range_cm(box)
        return box

    def touch(self):
        """The controller page is still open and watching."""
        self.touched = time.monotonic()

    def start(self):
        if not self.tracker or not self.tracker.locked:
            raise ValueError("click the person to follow first")
        self.touch()
        self.avoider.reset()
        # the stream is the one sonar pinger while following: the UNO's reflex and
        # the avoider here hear the same pings, 10 a second
        self.send({"N": 25, "D1": 100, "D2": 1})
        self.send({"N": 101, "D1": 3, "D2": self.front_raw()})
        self.on = True
        self.state["on"] = True

    def release(self):
        """Stop sending bearings; the caller is taking the car back with its own command."""
        self.on = False
        self.state["on"] = False

    def stop(self):
        was = self.on
        self.release()
        if was:
            self.send({"N": 100})

    def forget(self):
        self.stop()
        with self.lock:
            self.tracker.clear()
            self.last_seen = self.last_bearing = self.range = self.person_abs = None
            self.state.update(locked=False, box=None, seen=False, bearing=None, match=None)

    def close(self):
        self.alive = False

    # --- car state from telemetry ---------------------------------------------

    def _telemetry(self, now):
        """Fold in a new car sample; return (heading now or None, its age s)."""
        t = self.telemetry()
        if not t or now - t["mono"] > TELEMETRY_S:
            return None, 9.0
        if t["mono"] != self._last_mono:
            self._last_mono = t["mono"]
            self.yaws.append((t["mono"], self.yaw_left_sign * t["yaw"]))
            self.yaws = [(m, y) for m, y in self.yaws if now - m < 2.0]
            if t.get("pan", 90) == 90 and t.get("settled", True):   # front pings only
                self.pings = (self.pings[1], t["dist"])
            self.state["sonar"] = t["dist"]
        return self.yaws[-1][1] if self.yaws else None, now - t["mono"]

    def _yaw_at(self, mono):
        """The heading at a past moment, from the telemetry history (nearest sample)."""
        if not self.yaws:
            return None
        return min(self.yaws, key=lambda s: abs(s[0] - mono))[1]

    def _trace(self, hint, age, found):
        if not self.trace:
            return
        box = found[0] if found else None
        row = {"t": round(time.time(), 3), "bearing": hint[0], "seen": hint[1], "age_ms": age,
               "range": hint[2] if len(hint) > 2 else None, "avoid": self.avoider.state,
               "match": round(found[1], 3) if found else None,
               "box": [round(v, 3) for v in box] if box else None, "people": len(self.state["boxes"])}
        t = self.telemetry()
        if t and time.monotonic() - t["mono"] < 1.0:
            row.update(dist=t["dist"], yaw=t["yaw"], pan=t.get("pan"), mv=t.get("mv"))
        try:
            with open(self.trace, "a") as f:
                f.write(json.dumps(row) + "\n")
        except OSError:
            pass

    def _run(self):
        count, sent_at, frames, t0 = 0, 0.0, 0, time.monotonic()
        while self.alive:
            count, jpeg = self.stream.wait(count, 1.0)
            arrived = self.stream.at
            now = time.monotonic()
            yaw, yaw_age = self._telemetry(now)
            found = None
            watched = self.on or now - self.touched < PAGE_S    # no Vision work for nobody
            if jpeg is not None and watched and self.tracker.locked:
                with self.lock:
                    try:
                        found = self.tracker.update(jpeg)
                    except Exception as e:      # a Vision failure must not kill the loop
                        self.log(f"follow: tracker error {e}")
                        found = None
                    self.state["boxes"] = self.tracker.boxes
                    if found:
                        box, dist = found
                        b = bearing_deg(box, self.hfov, self.mirror)
                        self.last_seen, self.last_bearing, self.seen_arrival = now, b, arrived
                        r = range_cm(box)
                        self.range = r if self.range is None else RANGE_SMOOTH * r + (1 - RANGE_SMOOTH) * self.range
                        then = self._yaw_at(arrived - TRANSPORT_MS / 1000.0)
                        if then is not None:
                            self.person_abs = wrap(then + b)
                        self.state.update(seen=True, box=box, bearing=round(b, 1), match=round(dist, 3),
                                          range=round(self.range))
                    else:
                        self.state.update(seen=False)
            elif jpeg is not None and watched:
                with self.lock:                  # the page is open: show who can be picked
                    try:
                        self.state["boxes"] = self.tracker.scan(jpeg)
                    except Exception as e:
                        self.log(f"follow: scan error {e}")
            if jpeg is not None and watched:
                frames += 1
                if now - t0 >= 2.0:
                    self.state["fps"] = round(frames / (now - t0), 1)
                    frames, t0 = 0, now
            if self.on and now - self.touched > PAGE_S:
                self.log("follow: controller page went quiet, stopping")
                try:
                    self.stop()
                except OSError:
                    self.release()
            if self.on and now - sent_at >= 1.0 / SEND_HZ:
                fresh = self.last_seen is not None and now - self.last_seen <= HOLD_S
                person = None
                if self.last_bearing is not None and self.range is not None:
                    person = {"bearing": self.last_bearing, "range": self.range, "abs": self.person_abs, "fresh": fresh}
                pings = self.pings if yaw is not None else (0, 0)       # stale car state: no reflex here
                self.state["avoid"] = self.avoider.step(now, pings, person, yaw)
                if self.avoider.target is not None and yaw is not None:
                    b = max(-60, min(60, round(wrap(self.avoider.target - yaw))))
                    hint = (b, 1, round(self.avoider.drive_range))
                    age = round(yaw_age * 1000)
                else:
                    if found:
                        hint = (round(self.last_bearing), 1)
                    else:
                        since = now - self.last_seen if self.last_seen is not None else 1e9
                        hint = hint_for(self.last_bearing, since)
                    hint = hint + (round(self.range) if hint[1] == 1 and self.range else 0,)
                    age = frame_age_ms(self.seen_arrival, time.monotonic()) if hint[1] == 1 else 0
                self.state["hint"] = hint
                try:
                    self.send({"N": 29, "D1": hint[0], "D2": hint[1], "D3": age, "D4": hint[2]})
                    sent_at = now
                    self._trace(hint, age, found)
                except OSError as e:
                    self.log(f"follow: send failed {e}")
