#!/bin/bash
# One command for a NEW recording (night 11-27 item 4): VIO -> keyframes -> pair plan -> stage A check -> FFS -> stage B check -> fuse.
# No 121801 paths. Resumable (each step skips when its output exists). One heavy job at a time (power-offs): CPU nice 19, 8 threads;
# FFS (GPU) waits until no other GPU / heavy python job runs.
#
#   bash newrec.sh REC OUT [--cfg auto|c4f60|fps30] [--ffs-kf N] [--stop-after plan|check|ffs|fuse] [--force]
#     REC = phone recording folder (cam5.mp4, frames_cam5.csv, imu.csv, meta.json)
#     --cfg auto  = VIO config by fps (60 -> c4f60; 30 -> chosen by the 11-28 rehearsal, see CFG30 below)
#     --ffs-kf N  = FFS only on N keyframes spread over the recording (rehearsal / quick look); 0 = all keyframes (default)
#     --force     = continue to FFS even if stage A fails
# Pair profile is always "new" (low-angle backdoors off). kf-win = round(4 * fps / 60) (2 at 30 fps, 4 at 60 fps).
# Outputs: OUT/scan (run_scanner, VIO + lensonly + reduce), OUT/run/dump_ba/cams.npz (VIO poses), OUT/plan/plan.json, OUT/check_a.json,
#          OUT/bag (partner-frame BA: poses_all.npz, run/dump_ba/cams.npz with BA poses, ffs/{pairs,s1,s12}), OUT/check_b.json, OUT/newrec.log
#          (--ffs-kf N: OUT/run_ffsN, OUT/plan_ffsN, OUT/bag_ffsN)
# Not included yet (next steps after fuse, as for 121801): 2DGS (gsm_train --hold 8 scoring build), mesh, texture.
set -uo pipefail
REC=$(realpath "$1"); OUT=$(realpath -m "$2"); shift 2
CFGSEL=auto; FFSKF=0; STOP=fuse; FORCE=0
while [ $# -gt 0 ]; do case $1 in
  --cfg) CFGSEL=$2; shift;; --ffs-kf) FFSKF=$2; shift;; --stop-after) STOP=$2; shift;; --force) FORCE=1;;
  *) echo "unknown arg $1" >&2; exit 2;; esac; shift; done
G=$HOME/scanner/StereoDepthSeg-gsmap; N=$G/scan3d/gsmap/newrec; RS=$HOME/scanner/StereoDepthSeg-fixes/scan3d/scanA/run_scanner.sh
PY="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
CFG30=c4        # decided 11-27 night (runs/night_20261127/rehearsal30/vio_choice.json): c4 = c4f60 with track 30 Hz; on the 30 fps video c4 == c4f60 (identical poses), fps30 (640x360) ATE 8.3 vs 3.4 cm to the 60 fps VIO
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 PYTHONUNBUFFERED=1
mkdir -p "$OUT"; L=$OUT/newrec.log
log() { echo "$(date '+%F %T') $*" | tee -a "$L"; }
wait_gpu_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|kf_ba|colmap|xatlas' | grep -vc grep)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return 0
    sleep 30; n=$((n+1)); [ $n -gt 120 ] && { log "FAIL wait for free GPU/CPU > 60 min"; exit 1; }
  done
}
FPS=$(python3 -c "import json,sys; m=json.load(open('$REC/meta.json')); print(int(round(float(m.get('save_fps') or m.get('fps')))))")
KW=$(( (4 * FPS + 30) / 60 )); [ $KW -lt 1 ] && KW=1
case $CFGSEL in auto) [ $FPS -le 40 ] && CFG=$CFG30 || CFG=c4f60;; *) CFG=$CFGSEL;; esac
case $CFG in /*) CFGD=$CFG;; *) CFGD=$HOME/scanner/build_livepose/cfg_gen/$CFG;; esac   # name under cfg_gen or an absolute dir
[ -e "$CFGD/remap_1920x1080.bin" ] || { echo "VIO config $CFGD has no remap_1920x1080.bin (lp_replay needs it)" >&2; exit 2; }
VIOARGS=""; [ $FPS -le 40 ] && VIOARGS="--fps $FPS"
log "START newrec REC $REC fps $FPS kf-win $KW vio cfg $CFG ffs-kf $FFSKF stop-after $STOP"
# 1. VIO + lensonly + keyframes (CPU)
if [ ! -e "$OUT/scan/scan_summary.json" ]; then
  log "step scan (VIO + keyframes)"
  nice -n 19 bash "$RS" "$REC" "$OUT/scan" --stop-after reduce --cfg "$CFGD" --kf-win $KW ${VIOARGS:+--vio-args "$VIOARGS"} > "$OUT/scan.log" 2>&1 || { log "FAIL scan (see scan.log)"; exit 1; }
fi
VIO=$OUT/scan/vio/B_o0.poses.jsonl.gz; RED=$(ls -d "$OUT"/scan/red/*_kf | head -1)
[ -e "$VIO" ] && [ -d "$RED" ] || { log "FAIL no VIO poses or keyframe folder"; exit 1; }
# 2. cams (VIO poses, no BA) + plan (CPU)
[ -e "$OUT/run/dump_ba/cams.npz" ] || $PY "$N/mkcams.py" "$RED" "$REC" "$VIO" "$OUT/run" >> "$L" 2>&1 || { log "FAIL mkcams"; exit 1; }
if [ ! -e "$OUT/plan/plan.json" ]; then
  log "step plan"
  (cd "$G" && $PY scan3d/gsmap/gsm_ffs_pairs2.py plan "$OUT/run" --vio "$VIO" --rec "$REC" --out "$OUT/plan" --min-perp 40 --dir-deg 10 --n-pairs 4 --k 2 --profile new > "$OUT/plan.log" 2>&1) \
    || { log "FAIL plan (see plan.log)"; exit 1; }
fi
[ "$STOP" = plan ] && { log "STOP after plan"; exit 0; }
# 3. stage A check (CPU, before FFS)
$PY "$N/postrec_check.py" plan --plan "$OUT/plan/plan.json" --rec "$REC" --vio "$VIO" --scan "$OUT/scan" --out "$OUT/check_a.json" > "$OUT/check_a.log" 2>&1; rc=$?
if [ $rc = 0 ]; then log "CHECK A PASS"; elif [ $rc = 3 ]; then log "CHECK A FAIL: $(python3 -c "import json; print('; '.join(json.load(open('$OUT/check_a.json'))['fails']))")"
  [ $FORCE = 1 ] || [ $FFSKF -gt 0 ] || { log "STOP: stage A failed (use --force to continue)"; exit 3; }; else log "FAIL check_a crashed rc $rc"; exit 1; fi
[ "$STOP" = check ] && { log "STOP after check"; exit 0; }
# 4. BA WITH the partner frames (ba_partners_generic.py; 11-27 night: FFS on VIO-relative partner poses disagrees ~1.4-1.7 px per keyframe,
#    with partner BA 0.33-0.41 px, runs/night_20261127/bafps) then FFS with BA poses for keyframe AND partner (GPU)
FR=$OUT/run; PLANF=$OUT/plan/plan.json; BAO=$OUT/bag
if [ $FFSKF -gt 0 ]; then
  [ $rc = 3 ] && log "rehearsal mode (--ffs-kf $FFSKF): stage A FAIL ignored"
  FR=$OUT/run_ffs$FFSKF; PLANF=$OUT/plan_ffs$FFSKF/plan.json; BAO=$OUT/bag_ffs$FFSKF
  [ -e "$FR/dump_ba/cams.npz" ] || $PY "$N/mkcams.py" "$RED" "$REC" "$VIO" "$FR" --subset $FFSKF --plan "$OUT/plan/plan.json" >> "$L" 2>&1 || { log "FAIL mkcams subset"; exit 1; }
  [ -e "$PLANF" ] || (cd "$G" && $PY scan3d/gsmap/gsm_ffs_pairs2.py plan "$FR" --vio "$VIO" --rec "$REC" --out "$(dirname $PLANF)" --min-perp 40 --dir-deg 10 --n-pairs 4 --k 2 --profile new > "$OUT/plan_ffs$FFSKF.log" 2>&1) || { log "FAIL subset plan"; exit 1; }
fi
X="--run $FR --plan $PLANF --rec $REC --vio $VIO --out $BAO"; mkdir -p "$BAO"
for st in frames ba post ffs; do
  case $st in frames) [ -e $BAO/manifest.json ] && continue;; ba) [ -e $BAO/ba/ba_state.npz ] && continue;; post) [ -e $BAO/poses_all.npz ] && continue;; ffs) [ -e $BAO/ffs/s1/ffs_info.json ] && continue;; esac
  [ $st = ba ] || [ $st = ffs ] && wait_gpu_free
  log "step $st (partner BA)"
  (cd "$G" && source $HOME/scanner/host.env 2>/dev/null; FFS_ROOT=$HOME/scanner/StereoDepthSeg nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore "$N/ba_partners_generic.py" $st $X > "$BAO/$st.log" 2>&1)     || { log "FAIL $st (see $BAO/$st.log)"; exit 1; }
done
log "partner BA: $(cat $BAO/post.json | tr -d '
 ' | cut -c1-220)"
# 5. stage B check (CPU)
$PY "$N/postrec_check.py" ffs --pairs "$BAO/ffs/pairs" --out "$OUT/check_b.json" > "$OUT/check_b.log" 2>&1; rc=$?
if [ $rc = 0 ]; then log "CHECK B PASS"; elif [ $rc = 3 ]; then log "CHECK B FAIL: $(python3 -c "import json; print('; '.join(json.load(open('$OUT/check_b.json'))['fails']))")"; else log "FAIL check_b crashed rc $rc"; exit 1; fi
[ "$STOP" = ffs ] && { log "STOP after ffs"; exit 0; }
A="$BAO/run --vio $VIO --rec $REC --out $BAO/ffs --min-perp 40 --dir-deg 10 --n-pairs 4 --k 2 --profile new"
# 6. fuse (CPU)
if [ ! -e "$BAO/ffs/s12/fuse_info.json" ]; then
  log "step fuse"; (cd "$G" && $PY scan3d/gsmap/gsm_ffs_pairs2.py fuse $A > "$OUT/fuse.log" 2>&1) || { log "FAIL fuse (see fuse.log)"; exit 1; }
fi
log "DONE newrec (fused FFS: $BAO/ffs/s12, cams with BA poses: $BAO/run/dump_ba/cams.npz)"
