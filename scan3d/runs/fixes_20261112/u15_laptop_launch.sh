#!/usr/bin/env bash
# fixes 11-13 unit 15: laptop jobs one after another (board 165506, then office 121801), from the desktop WSL. Run detached:
#   setsid nohup bash u15_laptop_launch.sh [board|office|both] > /dev/null 2>&1 < /dev/null &
cd /home/system/scanner/StereoDepthSeg-fixes
L=scan3d/runs/fixes_20261112/u15_laptop_logs; mkdir -p $L
W=${1:-both}
# audit 11-13: log the code state here and pass the commit to the laptop run (u15_laptop.sh also logs the laptop file shas)
GITC="$(git rev-parse --short HEAD)$(git diff --quiet HEAD -- scan3d/scanA/scan_offline.py scan3d/motion_stereo.py || echo -dirty)"
echo "$(date '+%m-%d %T') launch $W git $GITC scan_offline $(sha256sum scan3d/scanA/scan_offline.py | cut -c1-16) motion_stereo $(sha256sum scan3d/motion_stereo.py | cut -c1-16)" >> $L/launch.txt
C="cmd:SCANNER_COMMIT=$GITC bash scan3d/runs/fixes_20261112/u15_laptop.sh"
if [ "$W" != office ]; then
  bash scan3d/tools/remote_laptop.sh job 165506 "$C board \$R \$O \$P" --tag fix_u15 \
    --send scan3d/runs/desktop/20261024_165506_std60/fix_u13c_pt > $L/board.log 2>&1
  echo "board rc $?" >> $L/launch.txt
fi
if [ "$W" != board ]; then
  bash scan3d/tools/remote_laptop.sh job 121801 "cmd:VARIANTS='${VARIANTS:-base sig farnone farguard farp2}' OFFICE_EXTRA='${OFFICE_EXTRA:-}' ${C#cmd:} office \$R \$O \$P" --tag ${OTAG:-fix_u15} \
    --send scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/red/hs_20261112_121801_std60_kf \
    --send scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/ba/poses_ba.jsonl.gz > $L/office.log 2>&1
  echo "office rc $?" >> $L/launch.txt
fi
echo "launch done" >> $L/launch.txt
