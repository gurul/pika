#ifndef ACTIVE_PAN_H
#define ACTIVE_PAN_H
#include <stdint.h>

// mod: command angle, not encoder feedback. Wrap-safe settling deadline.
struct ActivePan {
  uint8_t angle = 90;
  uint32_t started = 0;
  uint16_t settle_ms = 0;
  bool attached = false;

  uint8_t move(int requested, uint32_t now) {
    uint8_t next = requested < 10 ? 10 : requested > 170 ? 170 : requested;
    int delta = int(next) - angle;
    if (delta < 0) delta = -delta;
    // Interrupted travel has an unknown starting position: allow a full sweep.
    settle_ms = busy(now) ? 560 : 80 + 3 * delta;
    angle = next;
    started = now;
    attached = true;
    return angle;
  }
  bool busy(uint32_t now) const { return uint32_t(now - started) < settle_ms; }
  int tag(uint32_t now) const { return busy(now) ? -int(angle) : int(angle); }
};
#endif
