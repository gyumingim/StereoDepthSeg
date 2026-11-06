#!/bin/bash
# coordinator 11-06: pause controls until ~/scanner/night1107.done exists (3rd pause) (and the running control has finished), then resume
cd "$(dirname "$0")"
while pgrep -f "[v]iba_work/venv/bin/python scan3d/viba/viba.py" >/dev/null || [ ! -f $HOME/scanner/night1107.done ]; do sleep 60; done
echo RESUME $(date +%T)
./ctl_run.sh 20261020_221052 20261022_133820 20261022_134550 20261020_224323 20261020_223815 20261021_215348 20261021_214550 20261021_213341 20261020_221925 20261020_222747 20261020_214138
