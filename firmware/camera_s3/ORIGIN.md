# Origin

Copied 2026-09-22 from github.com/ekulkisnek/elegoo-car-custom-tools,
`esp32-s3/ESP32_CameraServer_AP_2023_V1.3/` at commit 05461d9. That tree is a
patched copy of ELEGOO's stock `ESP32_CameraServer_AP_2023_V1.3` camera sketch
for the ESP32-S3-WROOM-1 camera module shipped with newer Smart Robot Car V4
kits. Patches add station mode (joins home Wi-Fi from `secrets.h`), keep the
stock `ELEGOO-<mac>` soft AP, mDNS `elegoo-car.local`, and a `/drive` page.
The .ino was renamed to `camera_s3.ino` so the folder name matches.
