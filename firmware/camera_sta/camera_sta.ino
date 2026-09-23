// Station-mode firmware for the ELEGOO Smart Robot Car V4 camera module.
//
// The stock firmware runs its own Wi-Fi access point, so a laptop that joins
// it loses internet. This build joins an existing 2.4 GHz network instead and
// keeps the stock wire protocol:
//
//   TCP port 100   JSON command frames, relayed to the UNO over UART.
//                  The module sends {Heartbeat} every second and drops a
//                  client that misses three in a row. Reply with {Heartbeat}.
//   HTTP port 81   /stream  MJPEG video
//   HTTP port 80   /capture single JPEG
//
// If the network is unreachable for 20 s at boot, the module falls back to
// the stock access point (ELEGOO-<chip id>, open) so the phone app still works.
//
// Board: ESP32 Wrover Module (esp32:esp32:esp32wrover), partition huge_app.

// Implementation: camera_sta.cpp
