#!/usr/bin/env bash
# fixes 11-13 unit 15 on the LAPTOP (RTX 4060, PyTorch FFS; coordinator rule 11-13: compute only on the laptop).
# Run THERE through remote_laptop.sh (cwd = WT, $R $O $P set):
#   remote_laptop.sh job 165506 'cmd:bash scan3d/runs/fixes_20261112/u15_laptop.sh board' --tag fix_u15 --send <fix_u13c_pt>
#   remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/u15_laptop.sh office' --tag fix_u15 --send <fix_u8 red kf> --send <poses_ba>
# Variants run one after another inside this one job (one laptop lock). Every run: --ffs-backend pytorch (explicit, no host.env).
set -uo pipefail
what=$1; R=$2; O=$3; P=$4          # remote_laptop.sh run.sh vars are not exported -> passed as arguments
WT=$(pwd)
export PYTHONUNBUFFERED=1
unset SCAN3D_FFS_BACKEND
mkdir -p "$O"
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$O/u15_times.txt"; }
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
note "u15 laptop $what start, gpu $(nvidia-smi --query-gpu=name,memory.used,utilization.gpu --format=csv,noheader)"
# audit 11-13 (u15b_verify.md): record the code that actually runs here (the laptop copy pushed by remote_laptop.sh has no .git)
note "code sha256[:16] (laptop copy): $(for f in scan3d/scanA/scan_offline.py scan3d/motion_stereo.py scan3d/tsdf_map.py scan3d/tsdf_evict.py \
  scan3d/scanA/run_scanner.sh scan3d/runs/fixes_20261112/u15_laptop.sh; do printf '%s=%s ' "${f##*/}" "$(sha256sum "$f" 2>/dev/null | cut -c1-16)"; done)\
 desktop git ${SCANNER_COMMIT:-unknown}"
if [ "$what" = board ]; then
  D=$WT/scan3d/runs/desktop/20261024_165506_std60
  for v in base sig; do
    sa=off; [ $v = sig ] && sa=on
    t0=$SECONDS; m=$(gpumon "$O/gpu_board_$v.csv")
    SCANNER_COMMIT=${SCANNER_COMMIT:-unknown} bash scan3d/scanA/run_scanner.sh "$R" "$WT/$O/board_$v" --reuse "$D/fix_u13c_pt" --board auto \
      --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000 --ffs-backend pytorch --sigma-scale-aware $sa" > "$O/board_$v.log" 2>&1; rc=$?
    kill $m 2>/dev/null
    note "board $v rc $rc $((SECONDS - t0)) s GPU peak $(peak "$O/gpu_board_$v.csv") MiB $($P -c "
import json;s=json.load(open('$O/board_$v/mesh/summary.json'));b=json.load(open('$O/board_$v/mesh_board/summary.json'))
print('mesh p90',round(s['pooled_absdZ_cm']['p90'],4),'kf p90 med',round(s['kf_board_absdZ_p90_cm_median'],4),'mesh_board p90',round(b['pooled_absdZ_cm']['p90'],4),
 'z rms',round(s.get('mesh_board_z_rms_cm',-1),4),'cover',round(s.get('mesh_board_cover_frac',-1),3),'v',s['mesh_vertices'],'backend',s['ffs_backend'],
 'aware',s['depth_sigma'].get('scale_aware'),'down',s['depth_sigma'].get('near_ffs_calls_downscaled'),'/',s['depth_sigma'].get('near_ffs_calls'))" 2>&1)"
  done
else
  U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
  ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 600000 --ffs-backend pytorch ${OFFICE_EXTRA:-}"
  # OFFICE_EXTRA: e.g. '--tsdf-evict-gpu-blocks 60000' (unit 12: mesh bit-identical to off) so the far runs fit the 8 GB laptop GPU
  # (first try without it: farnone / farguard CUDA OOM under the gpu_guard 0.75 cap, 11-13 10:30)
  F="--far-pairs wide --far-baseline 0.35"
  for v in ${VARIANTS:-base sig farnone farguard farp2}; do
    case $v in
      base) x="--sigma-scale-aware off";;
      sig) x="--sigma-scale-aware on";;
      # every far flag explicit (audit 11-13: farnone / farguard of the unit 15 run had --far-partner2 off only by the then default)
      farnone) x="--sigma-scale-aware on $F --far-guard none --far-repl-min-z off --far-partner2 off";;
      farguard) x="--sigma-scale-aware on $F --far-guard lr+sat --far-lr-tol 1.0 --far-repl-min-z on --far-partner2 off";;
      # farp2: the unit 15 run used the OLD p2 rule of b20ad79 (next candidate); the current code runs the 15b rule instead
      farp2) x="--sigma-scale-aware on $F --far-guard lr+sat --far-lr-tol 1.0 --far-repl-min-z on --far-partner2 on";;
      # unit 15b
      farnone_sigoff) x="--sigma-scale-aware off $F --far-guard none --far-repl-min-z off --far-partner2 off";;
      farp2dir) x="--sigma-scale-aware on $F --far-guard lr+sat --far-repl-min-z on --far-partner2 on --far-p2-min-dir-deg 20 --far-lr-tol 1.0";;
      farlr15) x="--sigma-scale-aware on $F --far-guard lr+sat --far-lr-tol 1.5 --far-repl-min-z on --far-partner2 off";;
    esac
    t0=$SECONDS; m=$(gpumon "$O/gpu_office_$v.csv")
    $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/mesh_$v" $ARGS $x > "$O/mesh_$v.log" 2>&1; rc=$?
    kill $m 2>/dev/null
    note "office $v rc $rc $((SECONDS - t0)) s GPU peak $(peak "$O/gpu_office_$v.csv") MiB $($P -c "
import json;s=json.load(open('$O/mesh_$v/summary.json'));fp=s.get('far_pairs') or {}
print('v',s['mesh_vertices'],'runtime_s',s['runtime_s'],'ffs_s',s['ffs_s_total'],'blocks',s['tsdf_blocks'],'full',s['tsdf_full'],'backend',s['ffs_backend'],
 'aware',s['depth_sigma'].get('scale_aware'),'far',{k:fp.get(k) for k in ('kf_with_partner','px_added','px_replaced','px_conflict_kept_near','px_wide_only_cand','px_wide_only_rejected','px_rej_lr','px_rej_sat','px_rej_p2','px_repl_below_min_z','kf_partner2','kf_no_partner2','ffs_s','ffs2_s')} if fp else None)" 2>&1)"
  done
fi
note "u15 laptop $what done"
