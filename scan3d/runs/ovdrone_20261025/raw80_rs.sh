# raw80 (hs_20261021_213341 first 80 s, 1920x1080 frames + phone remap = live-like path, remap-row RS) : RS 2.94 ms / flip vs off
set -u
H=~/scanner/StereoDepthSeg-ovdrone/scan3d
O=/tmp/ovdrone/ident
B=/tmp/ovdrone/ate/bin/ov_bridge_b76c78e10e94
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
for v in "r294:all_exact,rs_derot=2940" "r294f:all_exact,rs_derot=-2940" "r221:all_exact,rs_derot=2205"; do
  n=${v%%:*}; o=${v#*:}
  $B /tmp/ovdrone/cfg/c4/estimator_config.yaml $BASE --remap /tmp/ovopt/remap_1920x1080.bin --ovopt $o < /tmp/ovopt/raw80.bin > $O/$n.jsonl 2> $O/$n.err
  echo "== $n rc $? $(grep 'rs_derot: readout' $O/$n.err | tail -1)"
done
cd $O && python3 $H/runs/ovopt_20261025/ate_raw80.py r_mine_ex r294 r294f r221 r_rs r_rsflip
