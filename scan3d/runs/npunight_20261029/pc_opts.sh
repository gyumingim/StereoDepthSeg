#!/bin/bash
# npunight: PC full-recording check of npu_opts (server C++ = new xfnpu_pc, NPU replaced by the torch fp32 stand-in npu_emu.py,
# like appnpu rep_npu.sh A): default vs nn_fast=1,lg_post=3 -> loop.jsonl / poses compared (map fields included).
set -u
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
R=~/scanner/StereoDepthSeg/scan3d/recordings
C=~/scanner/tmp_lens05/new_gen/c4f60
O=~/scanner/tmp_npunight/pcopts; mkdir -p $O; cd $O
N=~/scanner/StereoDepthSeg-appnpu/phone_stereo/arlat/npu
D=~/scanner/StereoDepthSeg-appnpu/scan3d/runs/appnpu_20261028
B=~/scanner/tmp_appnpu/ident/ov_bridge_new
WR=/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/L6_a16mc/lg_wr.npy
CFG=xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1
export PYTHONPATH=/home/system/scanner/tmp_ovxfeat/pylib
for v in def fast; do
  if [ $v = fast ]; then export XFNPU_OPTS=nn_fast=1,lg_post=3; else unset XFNPU_OPTS; fi
  ~/scanner/tmp_npunight/pctest/xfnpu_pc np$v pc:xfsoft pc:lg512 $WR $O/$v.npu.jsonl "$PY $N/npu_emu.py" 512 x512 2 2> $O/$v.srv.err & SP=$!
  for i in $(seq 1 120); do grep -q "ready\|failed" $O/$v.srv.err 2>/dev/null && break; sleep 1; done
  nice -n 5 $PY $D/appnpu_replay.py $R/hs_20261024_165506_std60 $C $B $O/$v --extra "--loop-xf unix:@np$v --loop-cfg $CFG" > $v.log 2>&1
  kill $SP; wait $SP
done
python3 - <<"PY"
import json
def L(p): return [l for l in open(p)]
for f in ("loop.jsonl","poses.jsonl"):
    a=L("def."+f); b=L("fast."+f)
    strip=lambda l: json.dumps({k:v for k,v in json.loads(l).items() if k not in ("ms","remap_ms","t_wall_ms")}, sort_keys=True)
    sa=[strip(x) for x in a]; sb=[strip(x) for x in b]
    print(f, len(a), len(b), "same" if sa==sb else "DIFFERENT (%d lines differ)" % sum(1 for x,y in zip(sa,sb) if x!=y))
PY
grep -h "opts" def.srv.err fast.srv.err
