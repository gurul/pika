#!/usr/bin/env bash
# Flash the station-mode firmware to the ELEGOO ESP32-S3 camera module through
# hwlog, then prove it booted and joined Wi-Fi.
#
#   tools/flash_camera.sh /dev/cu.usbmodemXXXX
#
# The port is the camera board's own USB-C, in download mode: hold BOOT, tap
# RST, release BOOT. The first run also backs up the factory flash.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT=${1:?usage: tools/flash_camera.sh /dev/cu.usbmodemXXXX}
FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashMode=qio,FlashSize=8M,PartitionScheme=default_8MB,PSRAM=opi,CPUFreq=240,UploadSpeed=921600"
SKETCH=firmware/camera_s3
BUILD=build/camera_s3
ESPTOOL=$(ls -d "$HOME"/Library/Arduino15/packages/esp32/tools/esptool_py/*/esptool | tail -1)
BACKUP=firmware/build-archive/camera_s3-factory-$(date +%Y%m%d).bin

[ -f "$SKETCH/secrets.h" ] || { echo "missing $SKETCH/secrets.h (copy secrets.h.example)" >&2; exit 2; }

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD" "$SKETCH"

if ! ls firmware/build-archive/camera_s3-factory-*.bin >/dev/null 2>&1; then
  mkdir -p firmware/build-archive
  echo "backing up factory flash to $BACKUP (8 MB)"
  "$ESPTOOL" --port "$PORT" --baud 921600 read-flash 0 0x800000 "$BACKUP"
fi

# hwlog owns the port; flash runs inside its pause lease and archives the ELF.
hwlog start --port "$PORT" --baud 115200 || true
hwlog flash --port "$PORT" -- \
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" --input-dir "$BUILD" "$SKETCH"

echo "waiting for Wi-Fi join (up to 40 s)"
if hwlog wait --pattern "STA OK  http://[0-9.]+" --timeout 40; then
  hwlog logs --boot -1 --tail 20
  echo "joined. try: tools/car.py dist"
else
  echo "no Wi-Fi join; last boot log:" >&2
  hwlog logs --boot -1 --tail 30 >&2
  exit 1
fi
