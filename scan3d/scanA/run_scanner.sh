#!/usr/bin/env bash
# scanner-defaults 2026-11-11: ONE entry point for the offline scanner (scan3d/scanA/README.md).
# Defaults = user decision 11-11:
#   (1) keyframe-only input: kf_ba / FFS / TSDF read only REC/frames_kf/*.jpg (kf_frames.py refuses a cam5.mp4 next to it)
#   (2) keyframes sent as 1920x1080 JPEG q90, the PC downscales to 960x540 (INTER_AREA) for all compute (kf_ba, FFS, TSDF)
#   (3) bundle adjustment on GPU (kf_ba defaults: --ba-device cuda, --sift-device cpu, --fast-tracks)
#
#   run_scanner.sh REC OUT [options]          (cwd = anywhere; paths absolute or relative to cwd)
# REC = a full recording (cam5.mp4 ...)  -> steps vio, reduce, kfba, mesh [, board]
#       a reduced recording (frames_kf/ + phone_live_poses.jsonl.gz, what the phone app will send) -> kfba, mesh [, board]
# Steps
#   vio     until the phone sends live poses: PC replay lp_replay.py with the phone's live config c4f60 (causal, per-frame output)
#           -> OUT/vio/B_o0.poses.jsonl.gz      (skipped with --vio FILE or a reduced REC)
#   reduce  until the phone app sends keyframes: make_reduced.py (kfonly) = the kf_ba keyframe rule on the live poses, keyframes
#           decoded from cam5.mp4, 1920x1080 JPEG q90 -> OUT/red/<rec>_kf/  (no video inside)
#   kfba    kf_ba.py SIFT 6000, seed 0, no loop, --kf-from-vio --match-ms 1, BA on GPU -> OUT/ba/poses_ba.jsonl.gz
#   mesh    scan_offline.py FFS 960x540 + TSDF 5 mm, --compress --kf-step 1, on the BA poses -> OUT/mesh/ (summary.json, mesh.ply)
#   board   (only with --board FILE|auto) board_scale.py: metric scale from the ChArUco board -> OUT/board/; then mesh again on the
#           board-scaled poses -> OUT/mesh_board/ (skip the 2nd mesh with --board-no-mesh)
# Options (old behaviour by flags)
#   --full-video        old path: no reduce, kf_ba + mesh decode REC/cam5.mp4 (A in kfonly)
#   --kf-res 1920|960   resolution of the sent keyframes (default 1920; 960 = kfonly B)      --kf-quality Q (default 90)
#   --ba-device cuda|cpu (default cuda; cpu = featloop/kfonly BA)   --sift-device cpu|auto (default cpu; auto = old GPU SIFT)
#   --gt NAME   GT/lens dir in REC (default: imu_gt_spl_cal1022 if present, else imu_gt_spl). Lens K/dist + scoring only.
#   --cfg DIR   VIO config (default: build_livepose/cfg_gen_hs222/c4f60 when the GT is cal1022 (2.22 dpt H), else cfg_gen/c4f60)
#   --vio FILE  use these live poses (skip the vio step)
#   --kfba-args '...'  --mesh-args '...'  --vio-args '...'  extra arguments for that step
#   --cache DIR  kf_ba image cache root (default OUT/cache, deleted at the end unless --keep-cache)
# Env: P (python, default ~/scanner/StereoDepthSeg/venv_ffs/bin/python), FFS_ROOT (default ~/scanner/StereoDepthSeg).
# Writes OUT/scan_summary.json (made, inputs, git commit, per-step seconds, |dZ| p90, k, ...) and prints it.
set -euo pipefail
WT=$(cd "$(dirname "$0")/../.." && pwd)
MAIN=/home/system/scanner/StereoDepthSeg
P=${P:-$MAIN/venv_ffs/bin/python}
export FFS_ROOT=${FFS_ROOT:-$MAIN}
B=/home/system/scanner/build_livepose
[ $# -ge 2 ] || { sed -n '2,/^set -euo/p' "$0" | grep '^#'; exit 1; }
REC=$(realpath -m "$1"); OUT=$(realpath -m "$2"); shift 2
FULLV=0; KRES=1920; KQ=90; BADEV=cuda; SIFTDEV=cpu; G=""; CFG=""; VIO=""; BOARD=""; BOARD_MESH=1
KFX=""; MSX=""; VIX=""; CACHE=""; KEEPC=0
while [ $# -gt 0 ]; do case $1 in
  --full-video) FULLV=1;; --kf-res) KRES=$2; shift;; --kf-quality) KQ=$2; shift;;
  --ba-device) BADEV=$2; shift;; --sift-device) SIFTDEV=$2; shift;; --gt) G=$2; shift;; --cfg) CFG=$2; shift;;
  --vio) VIO=$(realpath -m "$2"); shift;; --board) BOARD=$2; shift;; --board-no-mesh) BOARD_MESH=0;;
  --kfba-args) KFX=$2; shift;; --mesh-args) MSX=$2; shift;; --vio-args) VIX=$2; shift;;
  --cache) CACHE=$(realpath -m "$2"); shift;; --keep-cache) KEEPC=1;;
  *) echo "bad option $1" >&2; exit 1;; esac; shift; done
case $KRES in 1920|960) ;; *) echo "--kf-res must be 1920 or 960" >&2; exit 1;; esac
mkdir -p "$OUT"; CACHE=${CACHE:-$OUT/cache}
TIMES=$OUT/times.txt; : > "$TIMES"
T() {  # T NAME LOG CMD...: run CMD > LOG 2>&1, print + record wall ms; stop the script on failure
  local n=$1 lg=$2; shift 2; local t0 rc=0; t0=$(date +%s%N)
  echo "$(date +%T) step $n start (log $lg)"
  "$@" > "$lg" 2>&1 || rc=$?
  local ms=$(( ($(date +%s%N) - t0) / 1000000 )); echo "$n $ms" >> "$TIMES"
  echo "$(date +%T) step $n done $((ms / 1000)).$(( (ms % 1000) / 100 )) s (rc $rc)"
  [ $rc = 0 ] || { echo "step $n failed rc $rc, tail of $lg:" >&2; tail -n 15 "$lg" >&2; exit $rc; }; }

REDUCED=0; [ -d "$REC/frames_kf" ] && REDUCED=1
if [ $REDUCED = 1 ]; then
  [ -e "$REC/cam5.mp4" ] && { echo "$REC has frames_kf/ and cam5.mp4 -- not a keyframe-only recording" >&2; exit 1; }
  [ $FULLV = 1 ] && { echo "--full-video needs a full recording (cam5.mp4), $REC is keyframe-only" >&2; exit 1; }
fi
if [ -z "$G" ]; then [ -d "$REC/imu_gt_spl_cal1022" ] && G=imu_gt_spl_cal1022 || G=imu_gt_spl; fi
if [ -z "$CFG" ]; then [ "$G" = imu_gt_spl_cal1022 ] && CFG=$B/cfg_gen_hs222/c4f60 || CFG=$B/cfg_gen/c4f60; fi
echo "== run_scanner $(date '+%F %T'): REC $REC ($([ $REDUCED = 1 ] && echo keyframe-only || echo full recording)) -> $OUT"
echo "   gt $G, input $([ $FULLV = 1 ] && echo 'full video (old)' || echo "keyframes ${KRES} JPEG q${KQ}"), BA $BADEV, SIFT $SIFTDEV, git $(git -C "$WT" rev-parse --short HEAD 2>/dev/null || echo ?)"

# ---- vio (stand-in for the phone's live poses)
if [ $REDUCED = 1 ]; then
  VIO=${VIO:-$REC/phone_live_poses.jsonl.gz}
elif [ -z "$VIO" ]; then
  [ -d "$CFG" ] || { echo "VIO config $CFG missing" >&2; exit 1; }
  mkdir -p "$OUT/vio"
  cd "$WT/scan3d/runs/livepose_20261030"
  T vio "$OUT/vio.log" $P lp_replay.py "$REC" "$CFG" "$OUT/vio/B_o0" --bridge $B/bridge/ov_bridge \
    --vocab "$WT/scan3d/ov_bridge/loop/voc_k10L4.voc" $VIX
  cd "$WT"
  VIO=$OUT/vio/B_o0.poses.jsonl.gz
fi
[ -f "$VIO" ] || { echo "live poses $VIO missing" >&2; exit 1; }

# ---- reduce (stand-in for the phone app sending keyframes)
IN=$REC
if [ $REDUCED = 0 ] && [ $FULLV = 0 ]; then
  IN=$OUT/red/$(basename "$REC")_kf
  XR=""; [ "$KRES" = 1920 ] && XR=--full-res
  T reduce "$OUT/reduce.log" $P "$WT/scan3d/runs/kfonly_20261111/make_reduced.py" "$REC" "$VIO" --out "$IN" --quality "$KQ" $XR --gt "$G"
  [ -e "$IN/cam5.mp4" ] && { echo "reduced folder holds a video" >&2; exit 9; }
  VIO=$IN/phone_live_poses.jsonl.gz
fi
if [ -d "$IN/frames_kf" ]; then
  echo "[input] keyframe-only: $IN/frames_kf ($(ls "$IN/frames_kf" | wc -l) jpg, $($P -c "import cv2,glob;f=cv2.imread(sorted(glob.glob('$IN/frames_kf/*.jpg'))[0]);print(f'{f.shape[1]}x{f.shape[0]}')"), $(du -sh "$IN/frames_kf" | cut -f1)), cam5.mp4 absent: $([ -e "$IN/cam5.mp4" ] && echo NO || echo yes)"
else
  echo "[input] full video $IN/cam5.mp4 (old path)"
fi

# ---- kf_ba (GPU BA)
T kfba "$OUT/kfba.log" $P "$WT/scan3d/scanA/kf_ba.py" "$IN" --vio "$VIO" --out "$OUT/ba" --gt "$G" --kf-from-vio --match-ms 1 \
  --feat sift --loop none --seed 0 --cache-root "$CACHE" --ba-device "$BADEV" --sift-device "$SIFTDEV" $KFX
grep -m1 '^\[device\]' "$OUT/kfba.log" || true
rm -f "$OUT/ba/db.db"

# ---- FFS 960 + TSDF
T mesh "$OUT/mesh.log" $P "$WT/scan3d/scanA/scan_offline.py" "$IN" --out "$OUT/mesh" --gt "$G" --vio "$OUT/ba/poses_ba.jsonl.gz" \
  --compress --kf-step 1 $MSX

# ---- optional board scale
if [ -n "$BOARD" ]; then
  if [ "$BOARD" = auto ]; then case $(basename "$REC") in
    *165506*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_5x3.json;; *20261020_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_6x6.json;;
    *20261021_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_fit_6x6.json;; *20261022_*) BOARD=$MAIN/scan3d/config/charuco/board_monitor_6x6.json;;
    *) echo "--board auto: no board known for $(basename "$REC")" >&2; exit 1;; esac; fi
  [ "$KRES" = 960 ] && [ $FULLV = 0 ] && { echo "board_scale needs 1920x1080 keyframes (--kf-res 1920)" >&2; exit 1; }
  T board "$OUT/board.log" $P "$WT/scan3d/scanA/board_scale.py" "$IN" --poses "$OUT/ba/poses_ba.jsonl.gz" --board "$BOARD" \
    --lens "$IN/$G/gt.npz" --out "$OUT/board"
  if [ $BOARD_MESH = 1 ]; then
    T mesh_board "$OUT/mesh_board.log" $P "$WT/scan3d/scanA/scan_offline.py" "$IN" --out "$OUT/mesh_board" --gt "$G" \
      --vio "$OUT/board/poses_board.jsonl.gz" --compress --kf-step 1 $MSX
  fi
fi
[ $KEEPC = 1 ] || rm -rf "$CACHE"

# ---- summary
$P - "$OUT" "$REC" "$IN" "$VIO" "$G" "$CFG" "$KRES" "$KQ" "$FULLV" "$(git -C "$WT" rev-parse HEAD 2>/dev/null || echo ?)" <<'EOF'
import json, sys, time
from pathlib import Path
out, rec, inp, vio, g, cfg, kres, kq, fullv, commit = sys.argv[1:]
out = Path(out)
times = {l.split()[0]: round(float(l.split()[1]) / 1000, 1) for l in (out / "times.txt").read_text().split("\n") if l.strip()}
def mesh(d):
    p = out / d / "summary.json"
    if not p.exists():
        return None
    m = json.loads(p.read_text()); s = m["poses"]["sim3_scale_would_be"]
    return dict(absdZ_p90_cm=m["pooled_absdZ_cm"]["p90"], k_pct=(1 / s - 1) * 100, mesh_board_z_rms_cm=m.get("mesh_board_z_rms_cm"),
                n_kf=m.get("n_kf"), n_kf_depth=m.get("n_kf_depth"), n_kf_board_eval=m.get("n_kf_board_eval"))
dev = next((l.strip() for l in open(out / "kfba.log") if l.startswith("[device]")), None)
ba = json.loads((out / "ba" / "ba.json").read_text()) if (out / "ba" / "ba.json").exists() else {}
bs = json.loads((out / "board" / "board_scale.json").read_text()) if (out / "board" / "board_scale.json").exists() else None
man = json.loads((Path(inp) / "manifest.json").read_text()) if (Path(inp) / "manifest.json").exists() else None
r = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), commit=commit, rec=rec, input=inp, keyframe_only=(Path(inp) / "frames_kf").is_dir(),
         kf_res=None if fullv == "1" else int(kres), kf_quality=None if fullv == "1" else int(kq), live_poses=vio, gt=g, vio_cfg=cfg,
         kfba_device_line=dev, step_s=times, total_s=round(sum(times.values()), 1),
         sent=None if man is None else dict(n_keyframes=man["n_keyframes"], image_size=man["image_size"], jpeg_bytes=man["jpeg_bytes"],
                                            duration_s=man["duration_s"], jpeg_MB_per_min=man["jpeg_bytes"] / 1e6 / man["duration_s"] * 60),
         ba=dict(timing=ba.get("timing")), mesh=mesh("mesh"), board=bs, mesh_board=mesh("mesh_board"))
(out / "scan_summary.json").write_text(json.dumps(r, indent=1))
print(json.dumps({k: v for k, v in r.items() if k != "ba"}, indent=1))
EOF
echo "== run_scanner done $(date +%T)"
