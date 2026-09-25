#ifndef FOLLOW_DRIVE_H
#define FOLLOW_DRIVE_H
#include <stdint.h>

// mod: person follow. The Mac finds the chosen person in the video and sends
// their bearing and range (N=29) several times a second, with how old the frame
// was. The camera sets the course and the speed: the bearing is fixed to the
// gyro heading the car had when the frame was taken (`aim`), and the range holds
// the gap. The sonar is only a reflex: two pings in a row agreeing that
// something is close ahead stop forward motion, and closer still back the car
// off. No fresh bearing means no motion: a lost person, a dead Mac or a dropped
// link all stop the car.
struct FollowDrive {
  static const uint16_t kGapCm = 30;       // hold about this far behind the person (owner asked for closer than 50)
  static const uint16_t kBand = 8;         // no fore-aft motion within +-kBand of the gap
  static const uint16_t kTooCloseCm = 18;  // back off below this camera range, if the person is ahead
  static const uint16_t kBrakeCm = 22;     // sonar reflex: nothing forward with anything this close ahead
  static const uint16_t kBackCm = 14;      // ... and back off below this
  static const uint16_t kHintMs = 500;     // an older bearing counts as none
  static const uint8_t kStepMs = 60;       // heading history spacing
  static const int kVmin = 70;             // wheel PWM that still moves the car
  // Calm on purpose: the camera sees about +-31 deg and runs ~0.2 s behind, so
  // a fast turn loses the person (first floor run, 2026-09-25: 150 top speed,
  // 90 spin and 85 search went too fast and lost them).
  static const int kVmax = 110;
  static const int kSpin = 80;             // turn in place toward an off-axis person
  static const int kSearch = 75;           // turn in place toward where they left the frame
  static const int kTurnMax = 40;          // arc steering; kVmax + kTurnMax must stay under 256

  // A left turn decreases this car's gyro heading (yaw_left_sign in
  // calibration.json, measured by tools/precision.py; see HANDOVER.md).
  static const int8_t kYawLeft = -1;

  int16_t aim = 0;         // gyro heading the person was at, degrees x10
  int8_t bearing = 0;      // as sent, degrees, + = left; only its side matters for a search
  uint8_t seen = 0;        // 0 lost, 1 seen, 2 search toward the bearing's side
  uint16_t range = 0;      // camera range to the person, cm; 0 = unknown
  uint16_t sonarNow = 0;   // the last two sonar pings, cm; 0 = no echo or none
  uint16_t sonarPrev = 0;
  uint32_t hint_ms = 0;
  bool hinted = false;
  int16_t hist[8] = {0};   // gyro heading x10, one per kStepMs, hist[head] newest
  uint8_t head = 0;
  uint32_t hist_ms = 0;

  // Headings are integer tenths of a degree (yaw10), which keeps float code
  // out of the UNO's last few hundred bytes of flash.
  void sample(int16_t yaw10, uint32_t now) {
    if (uint32_t(now - hist_ms) < kStepMs) return;
    hist_ms = now;
    head = (head + 1) & 7;
    hist[head] = yaw10;
  }

  // Gyro heading about age_ms ago (within one step); beyond the history, its
  // oldest entry. hist[head - k] was sampled k to k + 1 steps ago.
  int16_t yawAgo(uint16_t age_ms, int16_t yaw10) const {
    uint16_t k = age_ms / kStepMs;
    if (k == 0) return yaw10;
    if (k > 7) k = 7;
    return hist[(head - k) & 7];
  }

  // Every sonar ping, from the telemetry stream: the one pinger while following,
  // because two pingers hear each other's echoes.
  void sonar(uint16_t d) {
    sonarPrev = sonarNow;
    sonarNow = d;
  }

  void hint(int b, uint8_t s, uint16_t age_ms, uint16_t r, int16_t yaw10, uint32_t now) {
    bearing = b < -60 ? -60 : b > 60 ? 60 : b;
    seen = s;
    range = r;
    aim = yawAgo(age_ms, yaw10) + bearing * 10 * kYawLeft;
    hint_ms = now;
    hinted = true;
  }

  // Signed wheel PWM, + = forward. yaw10: gyro heading now.
  void wheels(bool on_ground, int16_t yaw10, uint32_t now, int &left, int &right) const {
    left = right = 0;
    if (!on_ground || !hinted || seen == 0 || uint32_t(now - hint_ms) >= kHintMs) return;
    if (seen == 2) {
      right = bearing >= 0 ? kSearch : -kSearch;
      left = -right;
      return;
    }
    int err = (aim - yaw10) / 10 * kYawLeft;           // degrees still to turn, + = left
    err = err < -90 ? -90 : err > 90 ? 90 : err;
    const bool ahead = err >= -15 && err <= 15;
    int v = 0;
    if (range == 0) v = 0;                             // no range: turn to them, don't drive
    else if (range < kTooCloseCm) v = ahead ? -kVmin : 0;
    else if (range > kGapCm + kBand) {
      v = kVmin + (range - kGapCm - kBand);           // 70 at 38 cm up to kVmax at 78 cm
      if (v > kVmax) v = kVmax;
    }
    // Sonar reflex: both of the last two pings close, so not a stray echo.
    const uint16_t nearest = sonarNow > sonarPrev ? sonarNow : sonarPrev;
    if (sonarNow && sonarPrev && nearest < kBrakeCm) {
      if (v > 0) v = 0;
      if (nearest < kBackCm) v = -kVmin;
    }
    if (v == 0) {
      if (err > 6 || err < -6) {
        right = err > 0 ? kSpin : -kSpin;
        left = -right;
      }
      return;
    }
    int turn = v < 0 ? 0 : 2 * err;                    // reverse straight
    turn = turn < -kTurnMax ? -kTurnMax : turn > kTurnMax ? kTurnMax : turn;
    left = v - turn;
    right = v + turn;
  }
};
#endif
