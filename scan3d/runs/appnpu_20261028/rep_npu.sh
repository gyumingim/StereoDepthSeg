set -u
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
R=~/scanner/StereoDepthSeg/scan3d/recordings
C=~/scanner/tmp_lens05/new_gen/c4f60
O=~/scanner/tmp_appnpu/rep; mkdir -p $O; cd $O
N=~/scanner/StereoDepthSeg-appnpu/phone_stereo/arlat/npu
D=~/scanner/StereoDepthSeg-appnpu/scan3d/runs/appnpu_20261028
B=~/scanner/tmp_appnpu/ident/ov_bridge_new
WR=/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/L6_a16mc/lg_wr.npy
CFG=xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1
export PYTHONPATH=/home/system/scanner/tmp_ovxfeat/pylib
srv() { ~/scanner/tmp_appnpu/pc/xfnpu_pc $1 pc:xfsoft pc:lg512 $WR $O/$1.npu.jsonl "$PY $N/npu_emu.py" 512 x512 2 2> $O/$1.srv.err & SP=$!
  for i in $(seq 1 120); do grep -q "ready\|failed" $O/$1.srv.err 2>/dev/null && break; sleep 1; done; }
date
# A: with the server (FLGc gates), B: no server (connect fails -> BRIEF fallback), C: server killed after 12 s
srv apA; nice -n 5 $PY $D/appnpu_replay.py $R/hs_20261024_165506_std60 $C $B $O/A --extra "--loop-xf unix:@apA --loop-cfg $CFG" > A.log 2>&1; kill $SP; wait $SP
nice -n 5 $PY $D/appnpu_replay.py $R/hs_20261024_165506_std60 $C $B $O/B --extra "--loop-xf unix:@apNONE --loop-cfg $CFG" > B.log 2>&1
srv apC; (sleep 12; kill -9 $SP) & nice -n 5 $PY $D/appnpu_replay.py $R/hs_20261024_165506_std60 $C $B $O/C --extra "--loop-xf unix:@apC --loop-cfg $CFG" > C.log 2>&1; wait
for x in A B C; do echo "== $x"; tail -1 $x.log; grep -i "xfeat\|xf_down\|loop:" $x.bridge.err | head -5; done
date
