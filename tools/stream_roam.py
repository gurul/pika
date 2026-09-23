#!/usr/bin/env python3
"""VFH roaming with continuous curves and v4 nonblocking sonar telemetry.

Uses the older roam.py histogram and steering cost, with angle-tagged evidence,
bounded sensing time and the shared stop/transport protections. No echo is
unknown. Run --observe to view live sonar without wheel commands.
"""
from dataclasses import dataclass
import math
import time

from active_roam import ActivePlanner, Config, Decision, PolarMemory, main as run_controller, wrap
from roam import Histogram, SECTORS, MU1, MU2, MU3, TAU_HIGH, TAU_LOW


@dataclass
class VfhConfig(Config):
    stop_cm: float = 26.0
    ahead_s: float = 1.25
    center_dwell_s: float = .18
    steering_slew_s: float = 2.5
    # Gaze while driving: 'scored' picks each side look by staleness, weighted
    # toward the chosen path (Finean et al., RA-L 2021, arXiv 2109.04721);
    # 'interleave' is the fixed forward/left/forward/right pattern. Both search
    # outward in a fixed spiral while stopped.
    gaze: str = 'scored'
    path_weight: float = 3.0
    path_stale_s: float = .3


def stationary_heading_check(car, log, cancelled, clock=time.monotonic, sleep=time.sleep):
    """Reject the stationary gyro drift that invalidated previous physical runs."""
    _, reply = car.send({'N': 100}, wait=.5)
    if reply != '{ok}':
        raise RuntimeError('stationary check stop not acknowledged')
    for attempt in range(2):
        start, frames, seq = clock(), [], None
        while clock()-start < 1.3:
            if cancelled():
                raise RuntimeError('estop')
            t = car.telemetry
            if t and t.get('seq') != seq and t['mono'] >= start and clock()-t['mono'] < .3:
                seq = t['seq']
                frames.append((t['mono'], t['yaw']))
            sleep(.02)
        if len(frames) < 8 or frames[-1][0]-frames[0][0] < .8:
            raise RuntimeError('insufficient fresh telemetry for stationary gyro check')
        span = [wrap(yaw-frames[0][1]) for _, yaw in frames]
        drift = span[-1]/(frames[-1][0]-frames[0][0])
        stable = abs(drift) <= 2 and max(span)-min(span) <= 3
        log('gyro_check', samples=len(frames), drift_deg_s=round(drift, 3), stable=stable)
        if stable:
            return
        if attempt:
            raise RuntimeError('gyro unstable while stopped; no drive started')
        log('gyro_recalibration', reason='stationary_drift')
        _, reply = car.send({'N': 26}, wait=4)
        if not reply or not reply.endswith('_ok}'):
            raise RuntimeError('gyro recalibration not acknowledged')
        sleep(.25)


class VfhMemory(PolarMemory):
    """Candidate clearance with rays aged by geometry, not by full travel.

    Subtracting all forward travel from a side ray shrinks it until its
    enlargement cone covers straight ahead and every heading looks blocked.
    Travel direction is the mean of the heading when the ray was taken and now,
    which is exact for straight travel and second-order for a steady curve.
    The forward view() keeps the conservative full-travel subtraction.
    """
    def __init__(self, cfg, planner):
        super().__init__(cfg)
        self.planner = planner  # heading is already updated when add() runs

    def add(self, bearing, distance, now, travel):
        super().add(bearing, distance, now, travel)
        self.rays[-1].heading = self.planner.heading

    def clearance(self, bearing, now, travel):
        observed = self.view(bearing, now, travel)
        if observed is None:
            return None
        distance = observed[0]
        for r in self.rays:
            if now - r.stamp > self.cfg.memory_s or not 0 < r.distance < 400:
                continue
            s = travel - r.travel
            course = math.radians(r.heading + wrap(self.planner.heading - r.heading) / 2)
            dx = r.distance*math.cos(math.radians(r.bearing)) - s*math.cos(course)
            dy = r.distance*math.sin(math.radians(r.bearing)) - s*math.sin(course)
            d = max(1, math.hypot(dx, dy))
            ray_bearing = math.degrees(math.atan2(dy, dx))
            # Sonar obstacle direction is uncertain within its broad cone.
            enlargement = 15 + math.degrees(math.asin(min(1, self.cfg.radius_cm / d)))
            if abs(wrap(ray_bearing - bearing)) <= enlargement:
                distance = min(distance, d)
        return distance


class VfhPlanner(ActivePlanner):
    def __init__(self, cfg=None):
        super().__init__(cfg or VfhConfig())
        self.hist = Histogram()
        self.hist_heading = None
        self.steer = 0.0
        self.blind_until = 0.0
        self.center_ready_at = None
        self.glance_index = 0
        self.stationary_index = 0
        self.memory = VfhMemory(self.cfg, self)
        self.yaw_rate = 0.0
        self._rate_ref = None

    def ingest(self, t, now):
        if not super().ingest(t, now):
            return False
        if self._rate_ref is not None and now - self._rate_ref[0] >= .15:
            self.yaw_rate = wrap(self.heading-self._rate_ref[1]) / (now-self._rate_ref[0])
            self._rate_ref = (now, self.heading)
        elif self._rate_ref is None:
            self._rate_ref = (now, self.heading)
        if self.hist_heading is None:
            self.hist_heading = self.heading
        # Rotate the old body-relative histogram in whole sectors; accumulate
        # fractional heading changes rather than losing them each sample.
        shift = int(round(wrap(self.heading - self.hist_heading) / 10)) * 10
        if shift:
            old_h, old_b, old_t = self.hist.h.copy(), self.hist.blocked.copy(), self.hist.stamp.copy()
            for angle in SECTORS:
                self.hist.h[angle] = old_h.get(angle + shift, 0.0)
                self.hist.blocked[angle] = old_b.get(angle + shift, False)
                self.hist.stamp[angle] = old_t.get(angle + shift, 0.0)
            self.previous -= shift
            self.hist_heading = wrap(self.hist_heading + shift)
        if self.settled and 0 < t['dist'] < 400:
            # Histogram sectors are multiples of 10 degrees.
            self.hist.add(max(10, min(170, int(round(self.pan/10))*10)), t['dist'], now)
            # Old firmware used 150 as its no-echo sentinel. On v4 a valid
            # 150..399cm return must also release the histogram's blocked latch.
            for sector in SECTORS:
                if self.hist.h[sector] >= TAU_HIGH:
                    self.hist.blocked[sector] = True
                elif self.hist.h[sector] <= TAU_LOW:
                    self.hist.blocked[sector] = False
        return True

    def _request(self, angle, now):
        if angle != self.pan:
            self.pan_request, self.pan_requested_at = angle, now
        return angle

    def _scan(self, now, ahead, moving, selected=0):
        if self.pan_request is not None:
            if self.pan == self.pan_request and self.settled and self.pan_samples >= 2:
                self.pan_request = None
            elif now - self.pan_requested_at <= .9:
                return self.pan_request
            else:
                # A late/missing pan never extends the original blind budget.
                self.pan_request = None
                return self._request(90, now)
        if self.pan != 90:
            self.center_ready_at = None
            if not self.blind_until:
                self.blind_until = now + .08 + .003 * abs(self.pan-90) + .32
            return self._request(90, now)
        if not self.settled or self.pan_samples < 2:
            return 90
        self.blind_until = 0.0
        if self.center_ready_at is None:
            self.center_ready_at = now
        if now - self.center_ready_at < self.cfg.center_dwell_s:
            return 90
        if moving:
            # Near-path glances only; wide sweeps cost too much blind time in
            # motion. Every glance returns to 90 before the next one.
            if ahead is None:
                return 90
            # A stopped car about to launch needs the sonar forward; a glance
            # started now would only be cancelled by the launch check.
            if max(self.left, self.right) < self.cfg.min_pwm:
                return 90

            def affordable(angle):
                duration = self._glance_s(angle)
                # While turning, the heading leaves the last forward ray's
                # 12-degree window before a glance returns.
                if abs(self.steer) > .3 or abs(self.yaw_rate)*duration > 8:
                    return False
                if now-ahead[1]+duration > self.cfg.ahead_s:
                    return False
                # Do not launch a glance that would immediately demand a stop.
                return self.speed_limit(ahead[0], duration) >= max(self.cfg.min_pwm+5, self.left, self.right)

            if self.cfg.gaze == 'scored':
                angle = self._best_look(now, (50, 70, 110, 130), selected, affordable)
            else:
                offsets = (-20, 20, -40, 40, -20, 20)
                angle = 90 + offsets[self.glance_index % len(offsets)]
                angle = angle if affordable(angle) else None
            if angle is None:
                return 90
            self.glance_index += 1
            self.blind_until = now + self._glance_s(angle)
        else:
            # Stopped there is no path to watch, so search outward instead:
            # the paper's gaze scoring applies to motion along a path.
            offsets = (-20, 20, -40, 40, -60, 60, -80, 80)
            angle = 90 + offsets[self.stationary_index % len(offsets)]
            self.stationary_index += 1
        self.center_ready_at = None
        return self._request(angle, now)

    @staticmethod
    def _glance_s(angle):
        """Out-and-back servo time plus two settled samples and ACK slack."""
        return 2 * (.08 + .003 * abs(angle-90)) + .64

    def _staleness(self, now, relative):
        """Seconds since any settled reading near this body-relative angle."""
        bearing = self.heading + relative
        ages = [now - r.stamp for r in self.memory.rays
                if abs(wrap(r.bearing - bearing)) <= 10 and now >= r.stamp]
        return min(ages) if ages else 2 * self.cfg.memory_s

    def _best_look(self, now, angles, selected, allowed=None):
        """Greedy gaze: look where it has gone longest unseen per second of
        blindness, tripled on the heading the planner is steering toward."""
        best, best_score = None, 0.0
        for angle in angles:
            if allowed is not None and not allowed(angle):
                continue
            stale = self._staleness(now, angle-90)
            on_path = abs((angle-90) - selected) <= 15 and stale >= self.cfg.path_stale_s
            score = (self.cfg.path_weight if on_path else 1.0) * stale / self._glance_s(angle)
            if score > best_score:
                best, best_score = angle, score
        return best

    def _stop(self, look, reason, selected=0.0, clearance=0.0, spin=0):
        self.left = self.right = 0.0
        return Decision(look=look, reason=reason, heading=selected,
                        clearance=clearance, spin=spin)

    def step(self, now, estop=False):
        dt = 0.0 if self.last_step is None else max(0, min(.5, now-self.last_step))
        self.last_step = now
        # Differential translation uses the mean wheel command. This remains a
        # conservative command-based estimate, not measured odometry.
        self.travel += (self.left+self.right) * .5 * self.cfg.cm_per_pwm_s * dt
        self.hist.decay(dt)
        ahead = self.memory.view(self.heading, now, self.travel, self.cfg.ahead_s)
        reason = None
        if estop:
            reason = 'estop'
        elif self.last_receipt is None or now-self.last_receipt > self.cfg.stale_s:
            reason = 'stale_telemetry'
        elif not self.ground:
            reason = 'ground_signal'
        elif now < self.emergency_until or (ahead is not None and ahead[0] <= self.cfg.stop_cm):
            reason = 'close_obstacle'
        elif ahead is None:
            reason = 'unknown_ahead'
        if reason in ('estop', 'stale_telemetry', 'ground_signal'):
            return self._stop(90, reason)
        if (self.blind_until and now > self.blind_until and
                not (self.pan == 90 and self.settled and self.pan_samples >= 2)):
            self.pan_request = None
            self._request(90, now)
            return self._stop(90, 'scan_timeout')

        candidates = []
        for sector in SECTORS:
            if self.hist.blocked[sector]:
                continue
            angle = sector-90
            clearance = self.memory.clearance(self.heading+angle, now, self.travel)
            if clearance is None or clearance <= self.cfg.stop_cm+10:
                continue
            cost = MU1*abs(angle) + MU2*abs(angle) + MU3*abs(angle-self.previous)
            candidates.append((cost, angle, clearance))
        selected, clearance = 0.0, 0.0
        if candidates:
            _, selected, clearance = min(candidates)
        elif reason is None:
            reason = 'no_confirmed_path'
        look = self._scan(now, ahead, moving=reason is None, selected=selected)
        if reason:
            return self._stop(look, reason, selected, clearance)

        # The reserved sensing interval shrinks as time passes. Charging a new
        # full glance delay on each tick caused the former stop-start loop.
        blind = max(0.0, self.blind_until-now)
        cap = min(self.speed_limit(ahead[0], blind), self.speed_limit(clearance))
        if cap < self.cfg.min_pwm:
            near = [r.distance-(self.travel-r.travel) for r in self.memory.rays
                    if now-r.stamp < self.cfg.memory_s and 0 < r.distance < 400]
            if abs(selected) >= 20 and clearance >= 80 and near and min(near) > self.cfg.stop_cm+2:
                self.pan_request = None
                self._request(90, now)
                return self._stop(90, 'recover_turn', selected, clearance, 1 if selected > 0 else -1)
            return self._stop(look, 'braking_margin', selected, clearance)

        # Restore the old useful speed floor and clearance-scaled speed. A
        # launch begins at the floor; subsequent increases remain slew-limited.
        desired = self.cfg.min_pwm + (self.cfg.vmax-self.cfg.min_pwm)*max(0, min(1, (ahead[0]-45)/65))
        outer = min(desired, cap)
        current = max(self.left, self.right)
        target_steer = max(-1.0, min(1.0, selected/40))
        if current >= self.cfg.min_pwm:
            outer = min(outer, current+self.cfg.accel_pwm_s*dt)
            delta = self.cfg.steering_slew_s*dt
            self.steer += max(-delta, min(delta, target_steer-self.steer))
        else:
            # Launch only with the sonar settled forward; a side glance still
            # in flight would otherwise drive blind without being charged.
            if not (self.pan == 90 and self.settled and self.pan_samples >= 2):
                self.pan_request = None
                self._request(90, now)
                return self._stop(90, 'centering', selected, clearance)
            outer = self.cfg.min_pwm
            # Relaunch on the chosen curve, not straight at what stopped us.
            self.steer = target_steer
        # Each wheel is either at or above the moving floor, or deliberately 0.
        # A wheel between the two stalls and turns a curve into a pivot.
        mag = abs(self.steer)
        need = self.cfg.min_pwm/(1-mag) if mag < 1 else math.inf
        if mag < .05:
            inner = outer
        elif need <= cap:
            outer = max(outer, need)
            inner = outer*(1-mag)
        else:
            inner = 0.0  # explicit pivot about the inner wheel
        self.left, self.right = (inner, outer) if self.steer > 0 else (outer, inner)
        self.previous = selected
        return Decision(self.left, self.right, look, 'drive', selected, clearance)


if __name__ == '__main__':
    raise SystemExit(run_controller(planner_factory=VfhPlanner, config_factory=VfhConfig,
                                    description=__doc__, run_prefix='vfh',
                                    startup_check=stationary_heading_check))
