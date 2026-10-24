#!/usr/bin/env bash
# 2026-10-24: VIO with the ASSUMED std60 align (baseA / audA) on the given recordings, then score all four conditions.
cd "$(dirname "$0")/../.."
for R in "$@"; do
  STD60_REC=$R STD60_CONDS=baseA,audA ../venv_ffs/bin/python imu_learn/std60_board_run.py run
  STD60_REC=$R STD60_CONDS=base,aud,baseA,audA ../venv_ffs/bin/python imu_learn/std60_board_run.py score 2>&1 | grep "^o"
done
