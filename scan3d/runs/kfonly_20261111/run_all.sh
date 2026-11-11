#!/bin/bash
# kfonly 11-11: the 11 H recordings (README 1), smallest first; progress to the session progress file.
export PROG=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/kfonly.progress.txt
H=$(cd "$(dirname "$0")" && pwd)
"$H/queue.sh" ${@:-20261022_134204 20261022_133820 20261022_134550 20261020_221052 20261020_224323 20261021_213341 20261020_223815 20261021_214550 20261020_221925 20261021_215348 20261020_222747} > "$H/queue.log" 2>&1
