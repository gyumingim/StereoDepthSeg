#!/bin/bash
# round 2 (18:30, resumed ovacc): confirmation of the 2 best o0-screen settings on all 42 HS cells (o1/o2 added),
# + LS on the std60 set (wrapper was missing at freeze time). Keeps the 6-job cap: waits for the std run.
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
while ! grep -q STD_DONE /tmp/ovacc/round_std.log; do sleep 20; done
B=$(ls /tmp/ovacc/ate/bin/ov_bridge_* | head -1)
[ -f /tmp/ovacc/ate/bin/br_LS.sh ] || { printf '#!/bin/bash\nexec %s "$@"\n' "$B" > /tmp/ovacc/ate/bin/br_LS.sh; chmod 755 /tmp/ovacc/ate/bin/br_LS.sh; }
$PY acc_std.py run LS --par 6
$PY acc_run.py run NC05,G85 --par 6
echo ROUND2_DONE
