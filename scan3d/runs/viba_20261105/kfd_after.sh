#!/bin/bash
cd "$(dirname "$0")"; while pgrep -f "[k]fba_from_dump.py scan3d/runs/viba_20261105/ba/20261021_215348" >/dev/null; do sleep 60; done
./kfd_run.sh 20261020_214138 20261020_222747 20261020_224323
