#!/bin/bash
# esp32fc: host -fsyntax-only check of main/*.c against test/host_stubs (NOT an IDF build)
cd "$(dirname "$0")/.." || exit 1
fail=0
for esc in 0 2; do
  for clk in 0 1; do
    for f in main/*.c; do
      extra="-DFC_CLOCK_IMU_DRDY=$clk"
      [ $esc = 2 ] && extra="$extra -DFC_ESC_PROTOCOL=2"
      if ! gcc -std=gnu11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -Werror -I test/host_stubs -I main \
          -I components/fc_safety/include -I components/fc_core/include -I components/viop/include $extra $f; then
        echo "FAILED $f esc=$esc clk=$clk"; fail=1
      fi
    done
  done
done
echo "syntax check done, fail=$fail"
