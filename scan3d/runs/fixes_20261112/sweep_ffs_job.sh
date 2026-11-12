#!/usr/bin/env bash
# FFS settings sweep 11-12 (desktop RTX 5080 only). One factor at a time from the desktop default (TRT fp16, long side 960,
# iters 8, max_disp 192), same BA poses / TSDF / sigma (--depth-sigma ba numbers), no far pairs.
#   board 165506: fix_u13c_pt keyframes + BA poses (mesh) and its board-scaled poses (mesh_board)
#   office 121801: laptop fix_u8 keyframes + BA poses, fix_u8 mesh command (as trt13d_full.sh / far14 baseline)
# Engines per config: ffs_trt.py build (same export recipe, only --iters / --max-disp / sizes change) into ENGROOT/<cfg>.
# Code frozen at start: git show HEAD:scan3d/scanA/scan_offline.py -> SNAP (another agent edits the worktree copy).
# Run detached: setsid nohup bash sweep_ffs_job.sh [cfg ...] > LOG 2>&1 &
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
unset SCAN3D_FFS_BACKEND
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
R=$WT/scan3d/runs/desktop/sweep_ffs_20261112          # small result files (git)
K=/home/system/scanner/sweep_ffs                      # meshes / engines / snapshot (not git)
ENGROOT=$K/engines
SNAP=$K/snap/scanA/scan_offline.py
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_sweep.txt
mkdir -p "$R" "$K/runs" "$ENGROOT" "$(dirname "$SNAP")"
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$R/progress.txt" >> "$LOGW"; }
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m+0}' "$1"; }
[ -f "$SNAP" ] || git show HEAD:scan3d/scanA/scan_offline.py > "$SNAP"
echo "{\"made\": \"$(date '+%F %T')\", \"snapshot\": \"$SNAP\", \"from\": \"git show $(git rev-parse --short HEAD):scan3d/scanA/scan_offline.py\", \"sha256\": \"$(sha256sum "$SNAP" | cut -c1-16)\"}" > "$R/snapshot.json"

U13C=$WT/scan3d/runs/desktop/20261024_165506_std60/fix_u13c_pt
BREC=$U13C/red/hs_20261024_165506_std60_kf
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
OREC=$U8/red/hs_20261112_121801_std60_kf
COMMON="--compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --ffs-backend trt"
DYN960="128x128,384x672,544x960;128x128,672x384,960x544"

# config -> "iters max_disp scale engine_dir sizes dynamic"
cfg() { case $1 in
  base_a|base_b) echo "8 192 0.5 /home/system/scanner/ffs_trt/engines_v3 - -";;
  base_rb) echo "8 192 0.5 $ENGROOT/base_rb 544x960,960x544 $DYN960";;
  it4)  echo "4 192 0.5 $ENGROOT/it4 544x960,960x544 $DYN960";;
  it12) echo "12 192 0.5 $ENGROOT/it12 544x960,960x544 $DYN960";;
  it16) echo "16 192 0.5 $ENGROOT/it16 544x960,960x544 $DYN960";;
  md256) echo "8 256 0.5 $ENGROOT/md256 544x960,960x544 $DYN960";;
  r1280) echo "8 192 0.666667 $ENGROOT/r1280 736x1280,1280x736 128x128,512x896,736x1280;128x128,896x512,1280x736";;
  *) echo "";; esac; }

build() {   # cfg -> engines (skip if built + check PASS)
  local c=$1; read -r it md sc ed sz dy <<< "$(cfg "$c")"
  [ "$sz" = - ] && return 0
  if [ -f "$ed/manifest.json" ] && [ -f "$R/$c/check.log" ] && tail -n1 "$R/$c/check.log" | grep -q PASS; then return 0; fi
  mkdir -p "$R/$c"; local t0=$SECONDS m
  m=$(gpumon "$R/$c/gpu_build.csv")
  for ws in 4 8; do
    $P scan3d/ffs_trt.py build --out "$ed" --sizes "$sz" --dynamic "$dy" --iters "$it" --max-disp "$md" --workspace-gb $ws \
      > "$R/$c/build_ws$ws.log" 2>&1 && break
    note "$c build with workspace $ws GB failed (tail: $(tail -n 2 "$R/$c/build_ws$ws.log" | tr '\n' ' ' | cut -c1-200))"
  done
  kill $m 2>/dev/null
  [ -f "$ed/manifest.json" ] && cp "$ed/manifest.json" "$R/$c/engine_manifest.json"
  $P scan3d/ffs_trt.py check --engines "$ed" --iters "$it" --max-disp "$md" > "$R/$c/check.log" 2>&1
  note "$c engines built $((SECONDS - t0)) s (GPU peak $(peak "$R/$c/gpu_build.csv") MiB), check: $(tail -n1 "$R/$c/check.log" | cut -c1-120)"
  tail -n1 "$R/$c/check.log" | grep -q PASS
}

one() {     # one scan_offline run: cfg name rec outdir extra-args...
  local c=$1 name=$2 rec=$3 o=$4; shift 4
  read -r it md sc ed sz dy <<< "$(cfg "$c")"
  local xs=""; [ "$sc" != 0.5 ] && xs="--scale $sc --allow-fullres"
  rm -rf "$o"; mkdir -p "$o"
  local m t0=$SECONDS rc=0
  m=$(gpumon "$o.gpu.csv")
  SWEEP_CFG=$c SWEEP_MAX_DISP=$md $P scan3d/runs/fixes_20261112/sweep_ffs_wrap.py "$SNAP" "$rec" --out "$o" $COMMON \
    --iters "$it" --ffs-engines "$ed" $xs "$@" > "$o.log" 2>&1 || rc=$?
  kill $m 2>/dev/null
  local wall=$((SECONDS - t0)) gp; gp=$(peak "$o.gpu.csv")
  mkdir -p "$R/$c"
  cp "$o.log" "$R/$c/$name.log"; cp "$o.gpu.csv" "$R/$c/$name.gpu.csv"
  for f in summary.json keyframes.json sweep_meta.json; do [ -f "$o/$f" ] && cp "$o/$f" "$R/$c/${name}_$f"; done
  echo "{\"cfg\": \"$c\", \"run\": \"$name\", \"rc\": $rc, \"wall_s\": $wall, \"gpu_peak_mib\": $gp, \"ended\": \"$(date '+%F %T')\"}" > "$R/$c/${name}_wall.json"
  echo "$name rc $rc wall $wall s GPU $gp MiB"
  return $rc
}

runcfg() {
  local c=$1 t0=$SECONDS s1 s2 s3
  if ! build "$c"; then note "$c SKIPPED: engines did not build / check failed (see $R/$c/)"; return 1; fi
  s1=$(one "$c" board_mesh "$BREC" "$K/runs/$c/board_mesh" --gt imu_gt_spl --vio "$U13C/ba/poses_ba.jsonl.gz")
  s2=$(one "$c" board_meshb "$BREC" "$K/runs/$c/board_meshb" --gt imu_gt_spl --vio "$U13C/board/poses_board.jsonl.gz")
  s3=$(one "$c" office "$OREC" "$K/runs/$c/office" --gt lensonly --vio "$U8/ba/poses_ba.jsonl.gz")
  local b; b=$(python3 -c "
import json
try:
    s=json.load(open('$R/$c/board_mesh_summary.json')); o=json.load(open('$R/$c/office_summary.json'))
    print('board p90 %.3f kfp90 %.3f cm | office v %d ffs %.1f s runtime %.1f s full %s' % (s['pooled_absdZ_cm']['p90'], s['kf_board_absdZ_p90_cm_median'], o['mesh_vertices'], o['ffs_s_total'], o['runtime_s'], o['tsdf_full']))
except Exception as e: print('summary missing', e)")
  DONE=$((DONE + 1)); local dt=$((SECONDS - t0)); TOT=$((TOT + dt))
  local left=$((NCFG - DONE)); local eta=$(date -d "+$(( left * 600 + 300 )) sec" '+%H:%M')
  note "$c done in $dt s: $s1; $s2; $s3; $b. $DONE/$NCFG configs, ETA ~$eta (est. ~10 min per config incl. engine build + eval)"
}

CFGS=${*:-base_a base_b it4 it16 r1280 md256 base_rb it12}
NCFG=$(echo $CFGS | wc -w); DONE=0; TOT=0
note "sweep start, git $(git rev-parse --short HEAD), snapshot $(sha256sum "$SNAP" | cut -c1-16), configs: $CFGS"
for c in $CFGS; do runcfg "$c" || true; done
note "all configs run; evaluation next"
$P scan3d/runs/fixes_20261112/sweep_ffs_eval.py > "$R/eval.log" 2>&1 && note "eval done: $R/sweep_results.json" || note "eval FAILED (see $R/eval.log)"
note "sweep job done"
