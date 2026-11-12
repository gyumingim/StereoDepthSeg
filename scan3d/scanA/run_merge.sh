#!/usr/bin/env bash
# merge 2026-11-12: "continue recording" = several recordings of one scene -> ONE mesh (scan3d/scanA/README.md 8).
#
#   run_merge.sh OUT REC_A REC_B [REC_C ...] [options]
#
# Steps
#   parts   every recording on its own, exactly as run_scanner.sh does today: vio (replay) [+ lensonly] + reduce
#           (run_scanner.sh REC OUT/partN --stop-after reduce) -> OUT/partN/red/<rec>_kf
#   align   merge_align.py: cross-recording SIFT matches (subsampled keyframes, all pairs) -> PnP of the new recording's keyframes
#           on points triangulated with the earlier recording's VIO poses -> 4-DoF transform (yaw + translation, both VIO worlds
#           gravity aligned and metric; Sim(3) scale ratio and tilt only reported) -> OUT/merged/<A>_merged_kf = one keyframe
#           folder (B's poses moved into A's world, frames renumbered, B's times shifted 100 s past A's end).
#           Not enough cross matches (no overlap) -> "MERGE FAILED", exit 6, nothing else runs.
#   scan    run_scanner.sh on that folder: ONE kf_ba over all keyframes (pairs within a recording as today; cross pairs = VIO
#           covisibility after the alignment + merge_align's verified cross pairs via kf_ba --extra-pairs), FFS depth partners
#           only within one recording (time gap > the 30 s partner cap), one TSDF, one mesh in A's world -> OUT/scan/
# Options
#   --vio-args-of N 'ARGS'   extra lp_replay arguments for the N-th recording (1 = A), e.g. '--max-secs 35' or
#                            '--start-off 25 --yaml init_dyn_use=true' (the split test of 121801, README 8)
#   --align-args '...'       extra merge_align.py arguments (--max-sub 80 --min-kf 5 --min-pnp 30 ...)
#   all other options go to the final run_scanner.sh (e.g. --depth-sigma ba --mesh-args '--tsdf-max-blocks 250000' --kfba-args ...)
#   --lens FILE              lens json for every part and the merged scan (default: the std 0.5 dpt app lens, like run_scanner)
# Writes OUT/merge_summary.json (made, commit, inputs, per-step seconds, merge.json numbers, final scan status) and prints it.
set -euo pipefail
WT=$(cd "$(dirname "$0")/../.." && pwd)
MAIN=/home/system/scanner/StereoDepthSeg
P=${P:-$MAIN/venv_ffs/bin/python}
[ $# -ge 3 ] || { sed -n '2,/^set -euo/p' "$0" | grep '^#'; exit 1; }
OUT=$(realpath -m "$1"); shift
RECS=(); declare -A VA=(); ALX=""; SX=(); LENS=$WT/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json
while [ $# -gt 0 ]; do case $1 in
  --vio-args-of) VA[$2]=$3; shift 3;;
  --align-args) ALX=$2; shift 2;;
  --lens) LENS=$(realpath -m "$2"); shift 2;;
  --*) SX+=("$1"); case $1 in --full-video|--board-no-mesh|--keep-cache) shift;; *) SX+=("$2"); shift 2;; esac;;
  *) RECS+=("$(realpath -m "$1")"); shift;;
esac; done
[ ${#RECS[@]} -ge 2 ] || { echo "run_merge.sh needs at least two recordings" >&2; exit 1; }
[ -f "$LENS" ] || { echo "lens $LENS missing" >&2; exit 1; }
[ -n "$OUT" ] && [ "$OUT" != / ] || { echo "bad OUT" >&2; exit 1; }
mkdir -p "$OUT"; TIMES=$OUT/merge_times.txt; : > "$TIMES"
COMMIT=${SCANNER_COMMIT:-$(git -C "$WT" rev-parse HEAD 2>/dev/null || echo unknown)}
echo "== run_merge $(date '+%F %T'): ${#RECS[@]} recordings -> $OUT (git ${COMMIT:0:12})"
T() { local n=$1; shift; local t0 rc=0; t0=$(date +%s%N); echo "$(date +%T) $n start"; "$@" || rc=$?
  local ms=$(( ($(date +%s%N) - t0) / 1000000 )); echo "$n $ms $rc" >> "$TIMES"; echo "$(date +%T) $n done $((ms / 1000)) s (rc $rc)"; return $rc; }
msum() {  # msum STATUS
$P - "$OUT" "$COMMIT" "$1" "${RECS[@]}" <<'EOF'
import json, sys, time
from pathlib import Path
out, commit, status, recs = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4:]
t = {l.split()[0]: dict(s=round(int(l.split()[1]) / 1000, 1), rc=int(l.split()[2])) for l in (out / "merge_times.txt").read_text().splitlines() if l.strip()}
mj = next(iter(sorted(out.glob("merged/*_align/merge.json"))), None)
ss = out / "scan" / "scan_summary.json"
s = json.loads(ss.read_text()) if ss.exists() and "scan" in t else None
r = dict(made=time.strftime("%F %T"), commit=commit, status=status, recordings=recs, steps=t,
         merge=json.loads(mj.read_text()) if mj else None,
         scan=None if s is None else dict(status=s.get("status"), ba_quality=s.get("ba_quality"), mesh=s.get("mesh"), step_s=s.get("step_s")))
(out / "merge_summary.json").write_text(json.dumps(r, indent=1))
print(json.dumps(dict(status=status, steps=t, n_kf=(r["merge"] or {}).get("n_keyframes_per_part"),
                      parts=[{k: p.get(k) for k in ("n_cross_verified", "n_pnp_ok", "n_consensus", "yaw_deg", "centre_err_cm_median",
                                                     "tilt_deg_median", "sim3_scale_ratio")} for p in (r["merge"] or {}).get("parts", [])],
                      mesh_vertices=((r["scan"] or {}).get("mesh") or {}).get("mesh_vertices")), indent=1))
EOF
}
REDS=()
for n in "${!RECS[@]}"; do
  k=$((n + 1)); R=${RECS[$n]}
  # shellcheck disable=SC2086
  if ! T part$k bash "$WT/scan3d/scanA/run_scanner.sh" "$R" "$OUT/part$k" --stop-after reduce --lens "$LENS" \
       ${VA[$k]:+--vio-args "${VA[$k]}"} > "$OUT/part$k.log" 2>&1; then
    tail -n 15 "$OUT/part$k.log" >&2; msum "failed: part$k"; exit 7; fi
  REDS+=("$OUT/part$k/red/$(basename "$R")_kf")
done
MD=$OUT/merged/$(basename "${RECS[0]}")_merged_kf
rc=0
# shellcheck disable=SC2086
T align $P "$WT/scan3d/scanA/merge_align.py" "${REDS[@]}" --out "$MD" $ALX > "$OUT/align.log" 2>&1 || rc=$?
if [ $rc != 0 ]; then
  tail -n 5 "$OUT/align.log" >&2
  msum "failed: align rc $rc$(grep -m1 'MERGE FAILED' "$OUT/align.log" | sed 's/^/ -- /' || true)"
  echo "!!!! run_merge: the recordings could not be merged (align rc $rc). Log $OUT/align.log" >&2; exit $rc
fi
grep '^\[part\|^\[merge' "$OUT/align.log" || true
KX="--extra-pairs $MD/extra_pairs.txt"
NS=(); KFX_SET=0
for ((j = 0; j < ${#SX[@]}; j++)); do
  if [ "${SX[$j]}" = --kfba-args ]; then NS+=(--kfba-args "${SX[$((j + 1))]} $KX"); KFX_SET=1; j=$((j + 1)); else NS+=("${SX[$j]}"); fi
done
[ $KFX_SET = 1 ] || NS+=(--kfba-args "$KX")
rc=0
T scan bash "$WT/scan3d/scanA/run_scanner.sh" "$MD" "$OUT/scan" --lens "$LENS" "${NS[@]}" > "$OUT/scan.log" 2>&1 || rc=$?
[ $rc = 0 ] && st=ok || { st="failed: scan rc $rc"; tail -n 15 "$OUT/scan.log" >&2; }
msum "$st"
echo "== run_merge done $(date +%T) ($st)"
exit $rc
