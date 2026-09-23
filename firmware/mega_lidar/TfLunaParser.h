#pragma once
#include <stdint.h>
#include <string.h>

struct TfLunaSample {
  uint16_t distance_raw, strength, temperature_raw;
  uint32_t stamp_ms;
  // Assumes the manufacturer's factory 9-byte/cm output format. These frames
  // do not identify their units; verify against a measured target on the bench.
  const char* status() const {
    if (strength < 100) return "weak_signal";
    if (strength >= 32768) return "overexposed";
    if (distance_raw < 20) return "below_range";
    if (distance_raw > 800) return "above_range";
    return "ok";
  }
  bool usableDefaultCm() const {
    return strength >= 100 && strength < 32768 &&
           distance_raw >= 20 && distance_raw <= 800;
  }
};

class TfLunaParser {
 public:
  uint32_t bytes = 0, frames = 0, bad_crc = 0;
  TfLunaSample last = {};
  bool feed(uint8_t b, uint32_t now) {
    ++bytes;
    if (n && uint32_t(now - last_byte_ms) > 50) n = 0;
    last_byte_ms = now;
    data[n++] = b;
    while (n && data[0] != 0x59) discard();
    if (n >= 2 && data[1] != 0x59) { discard(); return false; }
    if (n < 9) return false;
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) sum += data[i];
    if (sum != data[8]) {
      ++bad_crc;
      discard(); // Preserve a possible overlapping header after corruption.
      return false;
    }
    last = {word(2), word(4), word(6), now};
    ++frames;
    n = 0;
    return true;
  }
 private:
  uint8_t data[9] = {}, n = 0;
  uint32_t last_byte_ms = 0;
  uint16_t word(uint8_t i) const { return uint16_t(data[i]) | uint16_t(data[i+1]) << 8; }
  void discard() { --n; memmove(data, data + 1, n); }
};
