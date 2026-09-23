#!/usr/bin/env python3
"""Active sonar roaming: look before steering, with v4 angle-tagged telemetry.

Run --observe first (no wheel commands), then a supervised --duration 20 run.
This is a local reactive controller, not room mapping or guaranteed collision
avoidance. Defaults deliberately cap first trials to 110 PWM.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from car import Car
from safety import Safety, DIR_LEFT, DIR_RIGHT


def wrap(a):
    return (a + 180) % 360 - 180


@dataclass
class Config:
    vmax: int = 110
    min_pwm: int = 70
    memory_s: float = 1.8
    ahead_s: float = 1.15
    glance_after_s: float = 0.40
    stale_s: float = 0.40
    stop_cm: float = 30.0
    radius_cm: float = 20.0  # conservative body circle including clearance
    look_early_cm: float = 170.0
    # Assumptions, not measured guarantees. Validate on charged battery/floor.
    cm_per_pwm_s: float = 0.30
    braking_cm_s2: float = 45.0
    latency_s: float = 0.25
    accel_pwm_s: float = 100.0
    yaw_left_sign: int = -1


@dataclass
class Ray:
    bearing: float
    distance: float
    stamp: float
    travel: float


@dataclass
class Decision:
    left: float = 0.0
    right: float = 0.0
    look: int = 90
    reason: str = "waiting"
    heading: float = 0.0
    clearance: float = 0.0
    spin: int = 0


class PolarMemory:
    """Recent observations in gyro world bearings; unknown never decays to free.

    Distance ages by a conservative travel budget. This avoids pretending
    timed motor commands provide accurate translational odometry.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.rays = deque(maxlen=160)

    def add(self, bearing, distance, now, travel):
        self.rays.append(Ray(wrap(bearing), distance, now, travel))

    def view(self, bearing, now, travel, max_age=None):
        age_limit = self.cfg.memory_s if max_age is None else max_age
        rays = [r for r in self.rays if 0 <= now - r.stamp <= self.cfg.memory_s
                and abs(wrap(r.bearing - bearing)) <= 12]
        if not rays:
            return None
        newest = rays[-1]
        if now - newest.stamp > age_limit:
            return None
        # An invalid latest echo revokes old clearance at that bearing.
        if not 0 < newest.distance < 400:
            return None
        valid = [r for r in rays[-3:] if 0 < r.distance < 400]
        if len(valid) < 2:
            return None
        a, b = valid[-2:]
        # A single far spike cannot open a path. A close observation brakes now.
        distance = min(a.distance, b.distance)
        # On disagreement retain the smaller range; two new far readings are
        # needed before clearance can increase. Do not discard a usable near
        # estimate merely because a single far spike arrived.
        return (max(0, distance - (travel - min(a.travel, b.travel))), newest.stamp)

    def clearance(self, bearing, now, travel):
        observed = self.view(bearing, now, travel)
        if observed is None:
            return None
        distance = observed[0]
        for r in self.rays:
            if now - r.stamp > self.cfg.memory_s or not 0 < r.distance < 400:
                continue
            d = max(1, r.distance - (travel - r.travel))
            # Sonar obstacle direction is uncertain within its broad cone.
            enlargement = 15 + math.degrees(math.asin(min(1, self.cfg.radius_cm / d)))
            if abs(wrap(r.bearing - bearing)) <= enlargement:
                distance = min(distance, d)
        return distance


class ActivePlanner:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.memory = PolarMemory(self.cfg)
        self.heading = 0.0
        self.travel = 0.0
        self.last_seq = None
        self.last_tick = None
        self.last_receipt = None
        self.clock_offset = None
        self.sensor_time = 0.0
        self.ground = False
        self.mv = None
        self.pan = 90
        self.settled = False
        self.pan_samples = 0
        self.pan_request = None
        self.pan_requested_at = 0.0
        self.look_side = 1
        self.search_index = 0
        self.previous = 0.0
        self.goal = None
        self.goal_until = 0.0
        self.cruise_heading = None
        self.last_step = None
        self.left = self.right = 0.0
        self.emergency_until = 0.0

    def ingest(self, t, now):
        if t is None or "seq" not in t or now - t["mono"] > self.cfg.stale_s:
            return False
        sensor_time = self.sensor_time
        if self.last_seq is not None:
            delta_seq = (t["seq"] - self.last_seq) & 0xffff
            delta_tick = (t["sample_ms"] - self.last_tick) & 0xffff
            if not 0 < delta_seq < 32768 or not 0 < delta_tick < 32768:
                return False
            sensor_time += delta_tick / 1000
        offset = t["mono"] - sensor_time
        self.clock_offset = offset if self.clock_offset is None else min(self.clock_offset, offset)
        # Reject buffered old frames (MCU sample clock lags receipt progression).
        if offset - self.clock_offset > 0.30:
            return False
        self.sensor_time = sensor_time
        self.last_seq, self.last_tick = t["seq"], t["sample_ms"]
        self.last_receipt = t["mono"]
        self.heading = wrap(t["yaw"] * self.cfg.yaw_left_sign)
        if self.cruise_heading is None:
            self.cruise_heading = self.heading
        self.ground, self.mv = t["ground"], t.get("mv")
        if self.pan != t["pan"] or not t["settled"]:
            self.pan_samples = 0
        self.pan, self.settled = t["pan"], t["settled"]
        if not self.settled:
            return True
        self.pan_samples += 1
        bearing = self.heading + self.pan - 90
        d = t["dist"]
        self.memory.add(bearing, d, t["mono"], self.travel)
        # Any close return in the front quadrant overrides filtering immediately.
        if abs(self.pan - 90) <= 45 and 0 < d <= self.cfg.stop_cm:
            self.emergency_until = now + 0.35
        return True

    def speed_limit(self, distance, blind_s=0.0):
        # v*latency + v²/(2a) <= available clearance.
        a, tau = self.cfg.braking_cm_s2, self.cfg.latency_s + blind_s
        budget = max(0.0, distance - self.cfg.stop_cm)
        v = max(0.0, math.sqrt((a * tau)**2 + 2 * a * budget) - a * tau)
        return min(self.cfg.vmax, v / self.cfg.cm_per_pwm_s)

    def _look(self, now, ahead, selected, searching=False):
        # Finish a glance before issuing another command; do not starve it by
        # re-centering on every fast host tick while the servo is still moving.
        if self.pan_request is not None:
            if self.pan == self.pan_request and self.settled and self.pan_samples >= 2:
                self.pan_request = None
            elif now - self.pan_requested_at <= 0.9:
                return self.pan_request
            else:
                self.pan_request = None
        if self.pan != 90:
            desired = 90
        elif ahead is None and not (searching and self.settled and self.pan_samples >= 2):
            desired = 90
        elif ahead is not None and now - ahead[1] > self.cfg.glance_after_s:
            desired = 90
        elif not searching and self.goal is not None and abs(selected) > 10:
            desired = 90 + int(round(selected / 10) * 10)
        elif searching or ahead[0] < self.cfg.look_early_cm:
            # Search outward on the preferred side, then the other side.
            offsets = (20, 40, 60, 80, -20, -40, -60, -80)
            offset = offsets[self.search_index % len(offsets)] * self.look_side
            self.search_index += 1
            desired = 90 + offset
        elif abs(selected) > 10:
            desired = 90 + int(round(selected / 10) * 10)
        else:
            desired = 90
        desired = max(10, min(170, desired))
        if desired != self.pan:
            self.pan_request, self.pan_requested_at = desired, now
        return desired

    def _speed(self, clearance, ahead, selected, look=90):
        if ahead is None:
            return 0.0
        speed = min(self.speed_limit(clearance),
                    self.speed_limit(ahead[0], 0.75 if look != 90 else 0.2))
        return speed * max(0.6, 1 - abs(selected) / 150)

    def step(self, now, estop=False):
        dt = 0.0 if self.last_step is None else max(0, min(0.5, now - self.last_step))
        self.last_step = now
        self.travel += max(self.left, self.right) * self.cfg.cm_per_pwm_s * dt
        ahead = self.memory.view(self.heading, now, self.travel, self.cfg.ahead_s)
        reason = None
        if estop:
            reason = "estop"
        elif self.last_receipt is None or now - self.last_receipt > self.cfg.stale_s:
            reason = "stale_telemetry"
        elif not self.ground:
            reason = "ground_signal"
        elif now < self.emergency_until:
            reason = "close_obstacle"
        elif ahead is None:
            reason = "unknown_ahead"
        elif ahead[0] <= self.cfg.stop_cm:
            reason = "close_obstacle"

        candidates, feasible = [], []
        near = [r.distance - (self.travel-r.travel) for r in self.memory.rays
                if now-r.stamp < self.cfg.memory_s and 0 < r.distance < 400]
        can_turn = reason is None and near and min(near) > self.cfg.stop_cm
        if self.goal is not None and (now > self.goal_until or abs(wrap(self.goal - self.heading)) < 8):
            self.goal = None
        target = wrap((self.cruise_heading if self.goal is None else self.goal) - self.heading) if self.cruise_heading is not None else 0
        for angle in range(-80, 81, 10):
            clearance = self.memory.clearance(self.heading + angle, now, self.travel)
            if clearance is None or clearance <= self.cfg.stop_cm + 10:
                continue
            # Preference for long clear paths, small turns and previous choice.
            switch = 18 if angle * self.previous < 0 and abs(self.previous) >= 15 else 0
            cost = 0.65 * abs(angle - target) + 0.18 * abs(angle - self.previous) + switch + 4500 / max(1, clearance - 25)
            candidate = (cost, angle, clearance)
            candidates.append(candidate)
            # An attractive heading is not useful if neither driving nor a
            # bounded recovery turn can reach it. Rank executable choices first.
            # Include the sensing action needed to maintain this route. Using
            # centered speed here would relatch a goal that fails on each glance.
            needs_glance = abs(angle) > 10 or (ahead is not None and ahead[0] < self.cfg.look_early_cm)
            if (self._speed(clearance, ahead, angle, 110 if needs_glance else 90) >= self.cfg.min_pwm or
                    (can_turn and abs(angle) >= 20 and clearance >= 80)):
                feasible.append(candidate)
        selected, clearance = 0.0, 0.0
        if candidates:
            _, selected, clearance = min(feasible or candidates)
            if not feasible:
                self.goal = None
            elif self.goal is None and abs(selected) >= 20:
                self.goal, self.goal_until = wrap(self.heading + selected), now + 3
                self.look_side = 1 if selected > 0 else -1
        elif reason is None:
            reason = "no_confirmed_path"
        searching = not feasible or reason is not None
        look = self._look(now, ahead, selected, searching=searching)
        if reason is not None:
            self.left = self.right = 0.0
            return Decision(look=look, reason=reason, heading=selected, clearance=clearance)

        # Keep the forward envelope valid while looking away; reduce speed
        # before spending sensing time on side glances.
        speed = self._speed(clearance, ahead, selected, look) if feasible else 0.0
        if speed < self.cfg.min_pwm:
            self.left = self.right = 0.0
            # Recovery: a confirmed side opening can be reached with short
            # gyro-observed turns in place instead of blind reverse. Require
            # clearance around the current body; no spin on an emergency return.
            if abs(selected) >= 20 and clearance >= 80 and can_turn:
                # _look may have proposed a side target; only the emitted center
                # command may remain pending while the body turns.
                if self.pan != 90:
                    if self.pan_request != 90:
                        self.pan_requested_at = now
                    self.pan_request = 90
                else:
                    self.pan_request = None
                return Decision(look=90, reason="recover_turn", heading=selected,
                                clearance=clearance, spin=1 if selected > 0 else -1)
            self.goal = None
            return Decision(look=look, reason="braking_margin", heading=selected, clearance=clearance)
        correction = max(-35, min(35, selected * 0.75))
        desired_left = max(0, min(self.cfg.vmax, speed - correction))
        desired_right = max(0, min(self.cfg.vmax, speed + correction))
        # Rate-limit increases; braking and safety reductions are immediate.
        dl, dr = desired_left - self.left, desired_right - self.right
        fraction = min(1, self.cfg.accel_pwm_s * dt / max(1e-9, dl, dr))
        self.left += dl * fraction if dl > 0 else dl
        self.right += dr * fraction if dr > 0 else dr
        self.previous = selected
        return Decision(self.left, self.right, look, "drive", selected, clearance)


class ActiveSession:
    """Transport adapter shared by real runs and fake-transport tests."""
    def __init__(self, car, planner, safety, log, observe=False):
        self.car, self.planner, self.safety, self.log = car, planner, safety, log
        self.observe = observe
        self.last_command = -1e9
        self.sent_pan = None
        self.last_reason = None
        self.last_decision = Decision()
        self.pending_pan = None
        self.input_telemetry = None
        self.input_accepted = False

    def tick(self, now):
        p, s = self.planner, self.safety
        self.input_telemetry = self.car.telemetry
        accepted = self.input_accepted = p.ingest(self.input_telemetry, now)
        if accepted:
            s.mark_alive()
        if self.pending_pan is not None:
            angle, previous_seq, deadline = self.pending_pan
            t = self.input_telemetry
            if accepted and t['seq'] != previous_seq and t['pan'] == angle:
                self.sent_pan = angle
                self.pending_pan = None
                self.log('pan_confirmed', angle=angle, via='telemetry', seq=t['seq'])
            elif now >= deadline or s.estop:
                self.pending_pan = None
                s.raise_estop('pan_command_timeout')
            else:
                # The command may have arrived even if its small ACK was lost.
                # Remain stopped until fresh MCU angle telemetry confirms it.
                p.left = p.right = 0.0
                self.last_decision = Decision(look=angle, reason='awaiting_pan')
                return self.last_decision
        decision = p.step(now, s.estop)
        if self.observe:
            p.left = p.right = 0.0  # no fictitious travel while the wheels are disabled
        self.last_decision = decision
        if decision.reason != self.last_reason:
            self.log("state", reason=decision.reason)
            self.last_reason = decision.reason
        if decision.reason in ("estop", "stale_telemetry", "ground_signal"):
            if s.continuous or s.moving_until > now:
                s.stop(decision.reason)
            return decision
        if decision.spin:
            if s.continuous:
                s.stop("recover_turn")
            if not self.observe and now - self.last_command >= 0.18:
                s.pulse(DIR_LEFT if decision.spin > 0 else DIR_RIGHT, 100, 60)
                self.last_command = now
        elif decision.left or decision.right:
            if not self.observe and now - self.last_command >= 0.12:
                front = p.memory.view(p.heading, now, p.travel, p.cfg.ahead_s)
                if front is not None:
                    s.update_distance(front[0])
                    if not s.drive_diff(decision.left, decision.right):
                        p.left = p.right = 0
                self.last_command = now
        elif s.continuous:
            s.stop(decision.reason)
        elif s.moving_until > now and not decision.spin:
            s.stop(decision.reason)  # interrupt a recovery pulse if clearance was revoked
        # N28 only; N5 would cancel wheel mode and block the UNO.
        if decision.look != self.sent_pan:
            _, reply = self.car.send({"N": 28, "D1": decision.look}, wait=0.15)
            if reply and reply.endswith("_ok}"):
                self.sent_pan = decision.look
            else:
                seq = self.input_telemetry.get('seq') if self.input_telemetry else None
                self.pending_pan = (decision.look, seq, s.clock() + .5)
                p.left = p.right = 0.0
                if s.continuous or s.moving_until > s.clock():
                    s.stop('pan_unconfirmed')
                decision = self.last_decision = Decision(look=decision.look, reason='awaiting_pan')
            self.log("look", angle=decision.look)
        s.watchdog()
        return decision


class LoggedCar(Car):
    def __init__(self, host, log):
        self.command_log = log
        self.last_stop_reply = None
        super().__init__(host)

    def send(self, obj, wait=1.0):
        started = time.monotonic()
        line, reply = super().send(obj, wait)
        if obj.get('N') == 100:
            self.last_stop_reply = reply
        self.command_log('command', command=obj, reply=reply,
                         elapsed_ms=round((time.monotonic()-started)*1000, 1))
        return line, reply


def main(planner_factory=ActivePlanner, config_factory=Config, description=None,
         run_prefix='active', startup_check=None):
    ap = argparse.ArgumentParser(description=description or __doc__)
    ap.add_argument("--host", default="elegoo-car.local")
    ap.add_argument("--duration", type=float, default=20)
    ap.add_argument("--vmax", type=int, default=110)
    ap.add_argument("--observe", action="store_true", help="move only the sonar; no wheel commands")
    ap.add_argument("--run-dir", type=Path)
    args = ap.parse_args()
    if not 70 <= args.vmax <= 140 or not 0 < args.duration <= 600:
        ap.error("vmax must be 70..140; duration must be 0..600 seconds")
    cfg = config_factory(vmax=args.vmax)
    calpath = Path(__file__).resolve().parent.parent / "calibration.json"
    cal = json.loads(calpath.read_text()) if calpath.exists() else {}
    cfg.yaw_left_sign = cal.get("yaw_left_sign", -1)
    if cfg.yaw_left_sign not in (-1, 1):
        ap.error("calibration yaw_left_sign must be -1 or 1")
    run_dir = args.run_dir or Path("build/runs") / time.strftime(run_prefix + "-%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    stats = {"frames": 0, "drive_ticks": 0, "stop_ticks": 0, "max_pwm": 0, "observe": args.observe}
    with (run_dir / "log.jsonl").open("w") as log_file:
        def log(event, **fields):
            entry = {"t": round(time.monotonic() - started, 4), "event": event, **fields}
            log_file.write(json.dumps(entry) + "\n"); log_file.flush()
            if event not in ("tick", "command"):
                print(json.dumps(entry), flush=True)
        car = LoggedCar(args.host, log)
        safety = Safety(car, log, max_speed=args.vmax)
        safety.DEADMAN_S = 0.40
        safety.trim = (cal.get("trim_left", 1), cal.get("trim_right", 1))
        # A signal only sets the latch; socket I/O remains on the control thread.
        old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        for sig in old_handlers:
            signal.signal(sig, lambda *_: setattr(safety, "estop", True))
        error = None
        try:
            safety.stop("startup")
            car.send({"N": 25, "D1": 100, "D2": 1}, wait=0.5)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if car.telemetry and "seq" in car.telemetry:
                    break
                time.sleep(0.03)
            else:
                raise RuntimeError("UNO v4 angle-tagged telemetry required; no motion sent")
            center_sent = time.monotonic()
            _, center_reply = car.send({"N": 28, "D1": 90}, wait=.3)
            if not center_reply or not center_reply.endswith("_ok}"):
                raise RuntimeError("pan initialization not acknowledged")
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                t = car.telemetry
                if t and t["mono"] > center_sent + .6 and t.get("pan") == 90 and t.get("settled"):
                    break
                time.sleep(.02)
            else:
                raise RuntimeError("pan did not produce fresh centered telemetry")
            if startup_check is not None:
                startup_check(car, log, lambda: safety.estop)
            planner = planner_factory(cfg)
            session = ActiveSession(car, planner, safety, log, args.observe)
            last_seq = None
            run_started = time.monotonic()
            while time.monotonic() - run_started < args.duration and not safety.estop:
                now = time.monotonic()
                dec = session.tick(now)
                if session.input_telemetry and session.input_telemetry.get("seq") != last_seq:
                    last_seq = session.input_telemetry.get("seq"); stats["frames"] += 1
                    log("tick", **dec.__dict__, telemetry=session.input_telemetry,
                        accepted=session.input_accepted, accepted_seq=planner.last_seq,
                        accepted_age_s=None if planner.last_receipt is None else now-planner.last_receipt,
                        observe=args.observe, yaw_left_sign=cfg.yaw_left_sign)
                stats["drive_ticks" if dec.left or dec.right else "stop_ticks"] += 1
                stats["max_pwm"] = max(stats["max_pwm"], dec.left, dec.right)
                if dec.reason in ("ground_signal", "stale_telemetry") and now - started > 3:
                    raise RuntimeError(dec.reason)
                time.sleep(0.02)
            stats['active_elapsed_s'] = round(time.monotonic() - run_started, 3)
            if safety.estop:
                raise RuntimeError('estop')
        except (OSError, RuntimeError) as exc:
            error = str(exc); log("abort", reason=error)
        finally:
            stop_acknowledged = False
            try:
                safety.stop("exit")
                stop_acknowledged = car.last_stop_reply == '{ok}'
                car.send({"N": 25, "D1": 0}, wait=0.3)
                car.send({"N": 28, "D1": 90}, wait=0.3)
            except OSError:
                pass
            if not stop_acknowledged and error is None:
                error = 'exit stop not acknowledged'
            car.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
            stats.update(error=error, elapsed_s=round(time.monotonic() - started, 3),
                         motion_frames=safety.motion_frames, stop_acknowledged=stop_acknowledged)
            (run_dir / "summary.json").write_text(json.dumps(stats, indent=2) + "\n")
            log("end", **stats)
    return 1 if error else 0


if __name__ == "__main__":
    sys.exit(main())
