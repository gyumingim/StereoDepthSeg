#!/bin/bash
# Printed-board lens calibration, one command (lenskit 2026-10-27). PC only; never installs on the phone.
#   bash scan3d/tools/calcap/lenskit_run.sh SET_A SET_B [--g SET_G] [--board board_print_A1_12x9] [--out DIR]
#        [--app-focus 0.5] [--boot 30] [--sub-boot 20] [--no-app] [--true-lens true_lens.json]
#        [--mrcal-args "--accept-small --flatness-mm 0.05"]   (extra lenskit_mrcal.py options; both are human choices)
# 1. OpenCV k12_notan / k1k2p per set and A+B (calcap_calib.py)       -> OUT/opencv_{A,B,AB}.json
# 2. corners -> mrcal caches (lenskit_dump.py)                         -> OUT/corners_*.vnl, meta_*.json
# 3. mrcal OPENCV4 + 2-axis board warp, criteria C1-C8 (lenskit_mrcal.py) -> OUT/criteria.json, criteria_ko.txt
# 4. only if all PASS: app lens json + phone_ov_main set via the lens05 generator (lenskit_make_app_lens.py)
# Exit: 0 all pass (+ app files), 2 criteria failed, other = error.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
S3D="$(cd "$HERE/../.." && pwd)"
VPY="${VPY:-$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python}"
MRCAL_ENV="${MRCAL_ENV:-$S3D/runs/lenscal_20261027/mrcal_env.sh}"
A=""; B=""; G=""; BOARD="board_print_A1_12x9"; OUT=""; FOCUS="0.5"; BOOT=30; SUBBOOT=20; NOAPP=0; TRUE=""; MARGS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --g) G="$2"; shift 2;;
    --board) BOARD="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --app-focus) FOCUS="$2"; shift 2;;
    --boot) BOOT="$2"; shift 2;;
    --sub-boot) SUBBOOT="$2"; shift 2;;
    --no-app) NOAPP=1; shift;;
    --true-lens) TRUE="$2"; shift 2;;
    --mrcal-args) MARGS="$2"; shift 2;;
    *) if [ -z "$A" ]; then A="$1"; elif [ -z "$B" ]; then B="$1"; else echo "unknown arg $1"; exit 9; fi; shift;;
  esac
done
[ -n "$A" ] && [ -n "$B" ] || { sed -n 2,9p "$0"; exit 9; }
A="$(cd "$A" && pwd)"; B="$(cd "$B" && pwd)"; [ -n "$G" ] && G="$(cd "$G" && pwd)"
[ -n "$OUT" ] || OUT="$(dirname "$A")/lenskit_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
LOG="$OUT/run_log.txt"
exec > >(tee -a "$LOG") 2>&1
echo "lenskit_run $(date '+%F %T')  A=$A  B=$B  G=${G:-none}  board=$BOARD  app_focus=$FOCUS  out=$OUT"
echo "git $(git -C "$S3D" rev-parse --short HEAD 2>/dev/null) $(git -C "$S3D" rev-parse --abbrev-ref HEAD 2>/dev/null)"
TAG="lenskit_$(date +%Y%m%d_%H%M%S)"
TL=(); [ -n "$TRUE" ] && TL=(--true-lens "$TRUE")
cal() {  # name sets...
  local nm="$1"; shift
  "$VPY" "$HERE/calcap_calib.py" "$@" --boards "$BOARD" --dsplit 1.15 --focus-dpt "$FOCUS" --tag "${TAG}_$nm" "${TL[@]}" | tail -12
  cp "$1/calib/calib_${TAG}_$nm.json" "$OUT/opencv_$nm.json"
}
echo "== 1. OpenCV (calcap_calib)"
cal A "$A"; cal B "$B"; cal AB "$A" "$B"
echo "== 2. corners for mrcal"
"$VPY" "$HERE/lenskit_dump.py" "$OUT" A "$A" --boards "$BOARD"
"$VPY" "$HERE/lenskit_dump.py" "$OUT" B "$B" --boards "$BOARD"
GF=()
if [ -n "$G" ]; then "$VPY" "$HERE/lenskit_dump.py" "$OUT" G "$G" --boards "$BOARD"; GF=(--with-g); fi
echo "== 3. mrcal + criteria C1-C8"
( source "$MRCAL_ENV"; python3 "$HERE/lenskit_mrcal.py" "$OUT" --boot "$BOOT" --sub-boot "$SUBBOOT" "${GF[@]}" $MARGS )
PASS=$(python3 -c "import json;print(int(json.load(open('$OUT/criteria.json'))['all_pass']))")
if [ "$PASS" != "1" ]; then echo "RESULT: FAIL (criteria) - no app lens made. $OUT/criteria_ko.txt"; exit 2; fi
if [ "$NOAPP" = "1" ]; then echo "RESULT: PASS (app lens not requested)"; exit 0; fi
echo "== 4. app lens files (NOT installed)"
"$VPY" "$HERE/lenskit_make_app_lens.py" "$OUT" --app-focus "$FOCUS" --python "$VPY"
echo "RESULT: PASS - app lens files in $OUT/app_lens (not installed)"
