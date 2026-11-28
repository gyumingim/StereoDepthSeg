#!/bin/bash
# 11-28: newrec 20-kf rehearsal with the vendored kf_select (newrec/kfsel), same settings as run_c4_ba.sh, stop after FFS + check B.
# Compare with c4_ba (fixes kf_select): keyframe list must be identical.
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/rehearsal30
REC=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/rec_hs_20261112_121801_std30
[ -e $D/c4_ba_kfvend ] && { echo "c4_ba_kfvend exists, not overwriting"; exit 1; }
bash $HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/newrec/newrec.sh $REC $D/c4_ba_kfvend --ffs-kf 20 --stop-after ffs > $D/newrec_c4_ba_kfvend.out 2>&1
echo "rc $?" >> $D/newrec_c4_ba_kfvend.out
python3 - $D/c4_ba $D/c4_ba_kfvend > $D/kfvend_compare.json <<'PY'
import json, sys, glob, hashlib, time
from pathlib import Path
a, b = Path(sys.argv[1]), Path(sys.argv[2])
def kfdir(r): return Path(glob.glob(str(r / "scan/red/*_kf"))[0])
def man(r): return json.load(open(kfdir(r) / "manifest.json"))
def ksel(r): return json.load(open(kfdir(r) / "kf_select.json"))
ma, mb = man(a), man(b)
def kfl(m):
    for k in ("keyframes", "kf", "frames"):
        if k in m: return m[k]
    return None
la, lb = kfl(ma), kfl(mb)
def sha_dir(d):
    h = hashlib.sha256()
    for f in sorted(Path(d).glob("*.jpg")): h.update(f.name.encode()); h.update(f.read_bytes())
    return h.hexdigest()
ca = json.load(open(a / "check_b.json")); cb = json.load(open(b / "check_b.json"))
r = dict(made=time.strftime("%F %T"), a=str(a), b=str(b), manifest_keys=sorted(ma.keys()),
         n_kf=[ma.get("n_keyframes"), mb.get("n_keyframes")], kf_list_identical=(la == lb) if la is not None else None,
         kf_select_json_identical=ksel(a) == ksel(b), kf_select_cfg=[ma.get("kf_select"), mb.get("kf_select")] if ma.get("kf_select") != mb.get("kf_select") else ma.get("kf_select"),
         frames_kf_sha=[sha_dir(kfdir(a) / "frames_kf"), sha_dir(kfdir(b) / "frames_kf")],
         run_ffs20_cams_identical=open(a / "run_ffs20/dump_ba/cams.npz", "rb").read() == open(b / "run_ffs20/dump_ba/cams.npz", "rb").read(),
         check_b=[{k: ca.get(k) for k in ("PASS", "fails", "share_kf_over_line", "kf_med_px_median")}, {k: cb.get(k) for k in ("PASS", "fails", "share_kf_over_line", "kf_med_px_median")}])
r["vio_sha256_identical"] = ma["vio_sha256"] == mb["vio_sha256"]; r["frames_kf_identical"] = r["frames_kf_sha"][0] == r["frames_kf_sha"][1]
print(json.dumps(r, indent=1, default=str))
PY
