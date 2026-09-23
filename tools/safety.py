"""The safety layer owns the motors. Nothing else sends a motion frame.

Every move is a short timed pulse (the UNO stops itself when the timer runs
out), forward is refused when the last distance reading is too close or too
old, and an e-stop latches until the process exits. All decisions are logged
through the `log` callback so a run log can prove what happened.
"""
import time

MAX_PULSE_MS = 450
MIN_FORWARD_CM = 25
STALE_S = 0.8                 # no reply from the UNO of any kind for this long
AHEAD_FRESH_S = 2.5           # the "ahead" distance must be at most this old for a forward pulse
DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT = 3, 4, 1, 2
DIR_NAME = {1: "left", 2: "right", 3: "forward", 4: "back"}


class Safety:
    def __init__(self, link, log, max_speed=150, dry_run=False, clock=time.monotonic):
        self.link = link              # object with send(frame, wait) -> (line, reply)
        self.log = log                # callable(event: str, **fields)
        self.max_speed = max_speed
        self.dry_run = dry_run
        self.clock = clock
        self.estop = False
        self.dist = None              # last distance straight ahead, cm
        self.dist_ts = None           # when it was read
        self.sample_ts = None         # last live sample of any kind (any servo angle)
        self.moving_until = 0.0
        self.motion_frames = 0
        self.stops = 0
        self.trim = (1.0, 1.0)        # (left, right) wheel scale, from calibration.json
        self.continuous = False

    # -- sensor feed --------------------------------------------------------
    def mark_alive(self):
        """Any reply from the UNO proves the link and the board are alive."""
        self.sample_ts = self.clock()

    def update_distance(self, cm, ahead=True):
        """Any successful reading proves the sensor feed is alive; only a
        reading straight ahead updates the forward-veto distance."""
        if cm is not None and cm > 0:
            self.sample_ts = self.clock()
            if ahead:
                self.dist, self.dist_ts = cm, self.sample_ts

    def stale(self):
        return self.sample_ts is None or (self.clock() - self.sample_ts) > STALE_S

    def ahead_fresh(self):
        return self.dist_ts is not None and (self.clock() - self.dist_ts) <= AHEAD_FRESH_S

    # -- e-stop -------------------------------------------------------------
    def raise_estop(self, reason="user"):
        if not self.estop:
            self.estop = True
            self.log("estop", reason=reason)
        self.stop("estop")

    # -- motion -------------------------------------------------------------
    def stop(self, reason):
        self.stops += 1
        self.moving_until = 0.0
        self.continuous = False
        self.link.send({"N": 100}, wait=0.4)
        self.log("stop", reason=reason)

    def pulse(self, direction, speed, ms):
        """Request a timed move. Returns True if it was sent."""
        ms = min(int(ms), MAX_PULSE_MS)
        speed = max(0, min(int(speed), self.max_speed))
        if self.estop:
            self.log("deny", why="estop", dir=DIR_NAME[direction]); return False
        if self.stale():
            self.log("deny", why="stale", dir=DIR_NAME[direction])
            if self.clock() < self.moving_until:
                self.stop("stale")
            return False
        if direction == DIR_FWD and not self.ahead_fresh():
            self.log("deny", why="no_fresh_ahead_reading"); return False
        if direction == DIR_FWD and self.dist is not None and self.dist < MIN_FORWARD_CM:
            self.log("deny", why="too_close", dist=self.dist); return False
        if self.dry_run:
            self.log("pulse", dry=True, dir=DIR_NAME[direction], speed=speed, ms=ms); return True
        self.motion_frames += 1
        self.continuous = False
        self.moving_until = self.clock() + ms / 1000.0
        self.link.send({"N": 2, "D1": direction, "D2": speed, "T": ms}, wait=0.25)
        self.log("pulse", dir=DIR_NAME[direction], speed=speed, ms=ms, dist=self.dist)
        return True

    # -- continuous differential drive (roam mode) ----------------------------
    DEADMAN_S = 1.0

    def drive_diff(self, left, right):
        """Set left/right wheel speeds (forward only, 0..max_speed). Untimed
        on stock firmware. V3 has a 1.5 s hardware timeout; v4 active stream
        shortens it to 0.5 s. Call watchdog() even after a renewal expires."""
        left = max(0, min(int(round(left * self.trim[0])), self.max_speed))
        right = max(0, min(int(round(right * self.trim[1])), self.max_speed))
        if self.estop:
            self.log("deny", why="estop", dir="diff"); self.stop("estop"); return False
        if self.stale():
            self.log("deny", why="stale", dir="diff"); self.stop("stale"); return False
        if not self.ahead_fresh():
            self.log("deny", why="no_fresh_ahead_reading", dir="diff"); self.stop("no_fresh_ahead_reading"); return False
        if self.dist is not None and self.dist < MIN_FORWARD_CM and max(left, right) > 0:
            self.log("deny", why="too_close", dist=self.dist); self.stop("too_close"); return False
        if self.dry_run:
            return True
        self.motion_frames += 1
        self.continuous = bool(left or right)
        self.moving_until = self.clock() + self.DEADMAN_S
        self.link.send({"N": 4, "D1": left, "D2": right}, wait=0.25)
        return True

    def watchdog(self):
        """Call every tick. Stops the car if it is moving on stale data."""
        if self.continuous and self.clock() >= self.moving_until:
            self.stop("renewal_expired")
            return True
        if (self.continuous or self.clock() < self.moving_until) and self.stale():
            self.stop("stale")
            return True
        return False
