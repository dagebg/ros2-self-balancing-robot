#!/usr/bin/env bash
# Merges bootloader, partition table, OTA data and application into one image that flashes at 0x0:
#
#   tools/merge_firmware.sh [env] [output]       e.g. tools/merge_firmware.sh esp32 robot-firmware.bin
#   esptool.py --chip esp32 write_flash 0x0 robot-firmware.bin
#
# Run `pio run -e <env>` first. Offsets are the Arduino-ESP32 defaults for this board (4 MB, DIO).
set -euo pipefail

ENV="${1:-esp32}"
OUT="${2:-firmware-${ENV}-merged.bin}"
ROOT="$(git rev-parse --show-toplevel)"
BUILD="$ROOT/firmware/esp32/robot-firmware/.pio/build/$ENV"
PIO_HOME="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
PYTHON="${PYTHON:-python3}"

"$PYTHON" "$PIO_HOME/packages/tool-esptoolpy/esptool.py" --chip esp32 merge_bin \
  -o "$OUT" --flash_mode dio --flash_size 4MB \
  0x1000 "$BUILD/bootloader.bin" \
  0x8000 "$BUILD/partitions.bin" \
  0xe000 "$PIO_HOME/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" \
  0x10000 "$BUILD/firmware.bin"
