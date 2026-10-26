cd ~/scanner/StereoDepthSeg-ovloop/scan3d/runs/ovloop_20261026
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export OVLOOP_OUT=/home/system/scanner/tmp_ovloop/ate_det
mkdir -p $OVLOOP_OUT/bin; cp ~/scanner/tmp_ovloop/ate/bin/* $OVLOOP_OUT/bin/
sed -i "s#tmp_ovloop/ate/#tmp_ovloop/ate_det/#g" $OVLOOP_OUT/bin/br.sh
mkdir -p $OVLOOP_OUT/vpr; cp -r ~/scanner/tmp_ovloop/ate/vpr/boq $OVLOOP_OUT/vpr/
cat > /tmp/ovl_det_jobs.txt <<J
134204_o0_B30
221925_o2_B30
221925_o2_Q30
s165506_o0_B30
J
for j in $(cat /tmp/ovl_det_jobs.txt); do $PY ovloop_run.py job $j 2>/dev/null & done; wait
for j in $(cat /tmp/ovl_det_jobs.txt); do c=${j##*_}; a=~/scanner/tmp_ovloop/ate/$c/$j.loop.jsonl; b=$OVLOOP_OUT/$c/$j.loop.jsonl
  python3 - "$a" "$b" <<'P'
import json,sys
def L(f): return [json.loads(x) for x in open(f)]
A,B=L(sys.argv[1]),L(sys.argv[2])
strip=lambda x:{k:v for k,v in x.items() if k not in ("ms",)}
la=[strip(x) for x in A if x.get("type") in ("loop","kf")]; lb=[strip(x) for x in B if x.get("type") in ("loop","kf")]
print(sys.argv[1].split("/")[-1], "lines", len(la), len(lb), "identical(excl ms)", la==lb, "loops", sum(x["type"]=="loop" for x in A), sum(x["type"]=="loop" for x in B))
P
done
