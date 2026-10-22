#!/bin/bash
# judge round 2 item 11: rms of "k1 shared" vs "k1 per focus" on the SAME views.
# 1) both fits as before (own 2.5x rejection) -> kept views; 2) intersection; 3) both fits on the intersection (no rejection).
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
mkdir -p logs
nice -n 5 $P joint.py --groups focus --model k5 --boot 0 --tag _sv1 > logs/joint_focus_k5_sv1.log 2>&1 &
nice -n 5 $P joint.py --groups focus --model k5 --k1-per-group --boot 0 --tag _sv1 > logs/joint_focus_k5_k1g_sv1.log 2>&1 &
wait
$P - <<'EOF'
import json
a = json.load(open("results/joint_focus_k5_sv1.json"))["kept_views"]
b = json.load(open("results/joint_focus_k5_k1g_sv1.json"))["kept_views"]
sa, sb = {tuple(x) for x in a}, {tuple(x) for x in b}
inter = sorted(sa & sb)
json.dump(dict(views=[list(x) for x in inter], n_k1shared=len(sa), n_k1g=len(sb), n_inter=len(inter),
               note="intersection of kept views of joint_focus_k5_sv1 and joint_focus_k5_k1g_sv1"),
          open("results/views_same_k5_vs_k1g.json", "w"))
print("kept", len(sa), len(sb), "intersection", len(inter))
EOF
nice -n 5 $P joint.py --groups focus --model k5 --boot 0 --tag _same --views-from results/views_same_k5_vs_k1g.json > logs/joint_focus_k5_same.log 2>&1 &
nice -n 5 $P joint.py --groups focus --model k5 --k1-per-group --boot 0 --tag _same --views-from results/views_same_k5_vs_k1g.json > logs/joint_focus_k5_k1g_same.log 2>&1 &
wait
cat logs/joint_focus_k5_sv1.log logs/joint_focus_k5_k1g_sv1.log logs/joint_focus_k5_same.log logs/joint_focus_k5_k1g_same.log
