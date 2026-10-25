#!/bin/bash
# esp32fc: host -fsyntax-only check of main/*.c against test/host_stubs (NOT an IDF build).
# Variants: ESC = PWM (default) / OneShot125 / DShot600, IMU = phone (default) / local ICM-42688.
cd "$(dirname "$0")/.." || exit 1
fail=0
for esc in 0 1 2; do
  for imu in 0 1; do
    for f in main/*.c; do
      extra="-DFC_ESC_PROTOCOL=$esc -DFC_IMU_SOURCE=$imu"
      if ! gcc -std=gnu11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -Werror -I test/host_stubs -I main \
          -I components/fc_safety/include -I components/fc_core/include -I components/viop/include $extra $f; then
        echo "FAILED $f esc=$esc imu=$imu"; fail=1
      fi
    done
  done
done
echo "syntax check done, fail=$fail"
