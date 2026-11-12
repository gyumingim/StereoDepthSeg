#!/usr/bin/env bash
# scanner-defaults 2026-11-11: ONE entry point for the offline scanner (scan3d/scanA/README.md).
# Defaults = user decision 11-11:
#   (1) keyframe-only input: kf_ba / FFS / TSDF read only REC/frames_kf/*.jpg (kf_frames.py refuses a cam5.mp4 next to it)
#   (2) keyframes sent as 1920x1080 JPEG q90, the PC downscales to 960x540 (INTER_AREA) for all compute (kf_ba, FFS, TSDF)
#   (3) bundle adjustment on GPU (kf_ba defaults: --ba-device cuda, --sift-device cpu, --fast-tracks)
#   (4) TSDF dense block allocation (ghost 11-11, scan_offline --tsdf-alloc dense): no ghost double layer on the board
#   (5) motion keyframes (kfsel 11-11): new keyframe after 3 cm or 4 deg, lowest-gyro frame of a 4-frame window (rollback --kf-select interval)
#   (6) FFS partners from all keyframes (dt <= 30 s, dy check), 2 images per depth map (ffsviews 11-11: --ffs-partners all --ffs-views 2;
#       rollback --ffs-partners near --ffs-views 3)
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
#   mesh    scan_offline.py FFS 960x540 + TSDF 5 mm (dense block allocation), --compress --kf-step 1, on the BA poses -> OUT/mesh/ (summary.json, mesh.ply)
#   board   (only with --board FILE|auto) board_scale.py: metric scale from the ChArUco board -> OUT/board/; then mesh again on the
#           board-scaled poses -> OUT/mesh_board/ (skip the 2nd mesh with --board-no-mesh)
#   redscreen (only with --scale-ref redscreen) redscreen_scale.py: metric scale from a laptop screen showing a full-screen solid red page
#           (redscreen 11-11) -> OUT/redscreen/ (s, sigma, edge-fit stats, poses_redscreen.jsonl.gz = BA positions / s); then mesh again on the
#           scaled poses -> OUT/mesh_red/. If the screen is not found or a quality gate fails: the step FAILS LOUDLY (message, summary
#           scale_status 'failed: ...', exit code 3 at the end) and only the unscaled OUT/mesh/ is left.
# Options (old behaviour by flags)
#   --full-video        old path: no reduce, kf_ba + mesh decode REC/cam5.mp4 (A in kfonly)
#   --kf-res 1920|960   resolution of the sent keyframes (default 1920; 960 = kfonly B)      --kf-quality Q (default 90)
#   --ba-device cuda|cpu (default cuda; cpu = featloop/kfonly BA)   --sift-device cpu|auto (default cpu; auto = old GPU SIFT)
#   --gt NAME   GT/lens dir in REC (default: imu_gt_spl_cal1022 if present, else imu_gt_spl). Lens K/dist + scoring, AND its 'ok' flag drops
#               keyframes from the mesh input (M1; scan_offline --keep-gt-not-ok via --mesh-args turns that off)
#   --lens FILE lens json (K, dist) for a recording WITHOUT a GT folder: a lens-only stand-in REC/lensonly/ is written after the vio step
#               (lensonly_gt.py; t_ns = frames_cam5.csv, poses = the live VIO poses, NOT a ground truth, no board scoring).
#               Without --lens and without a GT folder: std recordings at 0.5 dpt (meta focus_dpt_request 0.5) get the 0.5 dpt app lens
#               scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json automatically (the lens of cfg_gen/c4f60); anything else stops.
#   --scale-ref redscreen  metric scale from the red laptop screen (step redscreen above)   --red-size WxH  red area in mm
#               (default 302.0x188.75 = 14in Lenovo 83K9 panel, 1920x1200 px x 0.15729 mm EDID pitch: assumption, not measured)
#   --cfg DIR   VIO config (default: build_livepose/cfg_gen_hs222/c4f60 when the GT is cal1022 (2.22 dpt H), else cfg_gen/c4f60)
#   --vio FILE  use these live poses (skip the vio step)
#   --tsdf-alloc dense|sparse  TSDF block allocation (default dense; sparse = old 3-point allocation, ghost layer)
#   --kf-select motion|interval  keyframe rule in reduce (default motion = kfsel 11-11 user decision, kf_select.py: new keyframe after
#               --kf-dist CM (default 3) or --kf-rot DEG (default 4) since the last one, the frame of a --kf-win N (default 4) VIO-frame
#               window with the lowest gyro rate (--kf-blur gyro|lap|none, default gyro); kf_ba then reads the manifest list.
#               interval = old rule, every 10th VIO frame (rollback)
#   --ffs-views N  images per FFS depth map = reference keyframe + N-1 partner keyframes (default 2 since kfsel merge 11-11; 3 = old: scan_offline --n-pairs 2;
#               2 = one pair; 4, 5 = extra partners + median fusion; README 6)   --ffs-fuse auto|mean|median (default auto)
#   --ffs-partners near|all  partner search: all (default since 11-11) = every keyframe, dt <= 30 s + dy pair check;
#               near (old) = time neighbours dt <= 2.5 s (README 6). Rollback to the old FFS: --ffs-partners near --ffs-views 3
#   --reuse DIR  take vio/reduce/kfba from an earlier run_scanner OUT (DIR/red, DIR/ba): only mesh [, board] run (same BA poses)
#   --kfba-args '...'  --mesh-args '...'  --vio-args '...'  extra arguments for that step
#   --cache DIR  kf_ba image cache root (default OUT/cache, deleted at the end unless --keep-cache)
#   --min-mesh-vertices N  quality gate (fixes 11-12 M3): a mesh with fewer vertices -> exit 5 + summary status 'failed: ...' (default 1000)
# Checks added 11-12 (results unchanged): failed step / VIO bridge crash -> summary status 'failed: ...' + non-zero exit (H3);
#   earlier outputs in OUT are moved to OUT/_prev and the summary reads only steps run now (M2); explicit --gt folder missing -> stop (M5);
#   --reuse + ignored flags -> warning (M6); frames_kf size must match --kf-res (M9); summary records vio crash, BA keyframes on VIO pose,
#   TSDF block limit, GT-ok drops, commit dirty / MAIN commit / lens sha (M7). GT 'ok' still drops keyframes from the mesh (M1):
#   --mesh-args '--keep-gt-not-ok' turns that off.
# Env: SCANNER_COMMIT (commit to record, default git rev-parse), P (python, default ~/scanner/StereoDepthSeg/venv_ffs/bin/python), FFS_ROOT (default ~/scanner/StereoDepthSeg).
# Writes OUT/scan_summary.json (made, inputs, git commit, per-step seconds, |dZ| p90, k, ...) and prints it.
set -euo pipefail
WT=$(cd "$(dirname "$0")/../.." && pwd)
MAIN=/home/system/scanner/StereoDepthSeg
P=${P:-$MAIN/venv_ffs/bin/python}
export FFS_ROOT=${FFS_ROOT:-$MAIN}
B=/home/system/scanner/build_livepose
COMMIT=${SCANNER_COMMIT:-$(git -C "$WT" rev-parse HEAD 2>/dev/null || echo unknown)}
MAIN_COMMIT=${SCANNER_MAIN_COMMIT:-$(git -C "$MAIN" rev-parse HEAD 2>/dev/null)$(git -C "$MAIN" diff --quiet HEAD 2>/dev/null || echo -dirty)}   # lib_ffs / FFS come from MAIN (M7)   # remote_laptop.sh pushes code without .git -> passes it
[ $# -ge 2 ] || { sed -n '2,/^set -euo/p' "$0" | grep '^#'; exit 1; }
REC=$(realpath -m "$1"); OUT=$(realpath -m "$2"); shift 2
FULLV=0; KRES=1920; KQ=90; BADEV=cuda; SIFTDEV=cpu; G=""; CFG=""; VIO=""; BOARD=""; BOARD_MESH=1
LENS=""; SREF=""; REDSIZE=302.0x188.75; LENSONLY=0
KFX=""; MSX=""; VIX=""; CACHE=""; KEEPC=0; TALLOC=dense; KFSEL=motion; KFD=3; KFR=4; KFW=4; KFB=gyro; FV=2; FF=auto; FP=all; REUSE=""
G_SET=0; CFG_SET=0; PRE_REUSE=""; MINV=1000     # fixes 11-12: --gt given explicitly (M5), flags ignored by --reuse (M6), mesh vertex gate (M3)
while [ $# -gt 0 ]; do case $1 in
  --full-video) FULLV=1;; --kf-res) KRES=$2; shift;; --kf-quality) KQ=$2; shift;;
  --ba-device) BADEV=$2; shift;; --sift-device) SIFTDEV=$2; shift;; --gt) G=$2; G_SET=1; shift;; --cfg) CFG=$2; CFG_SET=1; shift;;
  --min-mesh-vertices) MINV=$2; shift;;
  --vio) VIO=$(realpath -m "$2"); shift;; --board) BOARD=$2; shift;; --board-no-mesh) BOARD_MESH=0;;
  --kfba-args) KFX=$2; shift;; --mesh-args) MSX=$2; shift;; --vio-args) VIX=$2; shift;;
  --tsdf-alloc) TALLOC=$2; shift;;
  --kf-select) KFSEL=$2; shift;; --kf-dist) KFD=$2; shift;; --kf-rot) KFR=$2; shift;; --kf-win) KFW=$2; shift;; --kf-blur) KFB=$2; shift;;
  --ffs-views) FV=$2; shift;; --ffs-fuse) FF=$2; shift;; --ffs-partners) FP=$2; shift;; --reuse) REUSE=$(realpath -m "$2"); shift;;
  --cache) CACHE=$(realpath -m "$2"); shift;; --keep-cache) KEEPC=1;;
  --lens) LENS=$(realpath -m "$2"); shift;; --scale-ref) SREF=$2; shift;; --red-size) REDSIZE=$2; shift;;
  *) echo "bad option $1" >&2; exit 1;; esac
  case $1 in --full-video|--kf-res|--kf-quality|--ba-device|--sift-device|--kfba-args|--vio-args|--kf-select|--kf-dist|--kf-rot|--kf-win|--kf-blur|--vio|--cfg)
    PRE_REUSE="$PRE_REUSE $1";; esac; shift; done
case $KRES in 1920|960) ;; *) echo "--kf-res must be 1920 or 960" >&2; exit 1;; esac
case $TALLOC in dense|sparse) ;; *) echo "--tsdf-alloc must be dense or sparse" >&2; exit 1;; esac
case $KFSEL in interval|motion) ;; *) echo "--kf-select must be interval or motion" >&2; exit 1;; esac
[ $KFSEL = motion ] && [ $FULLV = 1 ] && { echo "--kf-select motion needs the keyframe path (not --full-video)" >&2; exit 1; }
case $FV in 2|3|4|5|6|7|8) ;; *) echo "--ffs-views must be 2..8" >&2; exit 1;; esac
case $FF in auto|mean|median) ;; *) echo "--ffs-fuse must be auto, mean or median" >&2; exit 1;; esac
case $FP in near|all) ;; *) echo "--ffs-partners must be near or all" >&2; exit 1;; esac
case $SREF in ""|redscreen) ;; *) echo "--scale-ref must be redscreen" >&2; exit 1;; esac
[[ $REDSIZE =~ ^[0-9.]+x[0-9.]+$ ]] || { echo "--red-size must be WxH in mm, e.g. 302.0x188.75" >&2; exit 1; }
if [ -n "$SREF" ]; then
  [ "$KRES" = 960 ] && [ $FULLV = 0 ] && { echo "--scale-ref redscreen needs 1920x1080 keyframes (--kf-res 1920)" >&2; exit 1; }
  [ $FULLV = 1 ] && { echo "--scale-ref redscreen needs the keyframe path (not --full-video)" >&2; exit 1; }
fi
FFSX=""; { [ "$FV" = 3 ] && [ "$FF" = auto ]; } || FFSX="--ffs-views $FV --ffs-fuse $FF"   # views 3 + fuse auto: scan_offline gets the old command line
[ "$FP" = near ] || FFSX="$FFSX --ffs-partners $FP"
if [ -n "$REUSE" ] && [ ! -f "$REUSE/ba/poses_ba.jsonl.gz" ]; then echo "--reuse $REUSE: no ba/poses_ba.jsonl.gz" >&2; exit 1; fi
[ -n "$REUSE" ] && [ -n "$PRE_REUSE" ] && echo "!! WARNING --reuse $REUSE: vio/reduce/kfba are NOT run, these flags have no effect:$PRE_REUSE (M6)" >&2
[[ $MINV =~ ^[0-9]+$ ]] || { echo "--min-mesh-vertices must be an integer" >&2; exit 1; }
[ -n "$OUT" ] && [ "$OUT" != / ] || { echo "bad OUT '$OUT'" >&2; exit 1; }
mkdir -p "$OUT"; CACHE=${CACHE:-$OUT/cache}
# M2 (fixes 11-12): results of an earlier run in the same OUT must not enter this summary -> move away the per-run outputs that
# later steps would only read "if present" (vio / red / ba are rewritten by their own step; with --reuse they are read from DIR).
# Moved to OUT/_prev (replaced each run), not deleted; the summary also reads only the steps of THIS run (times.txt).
PREV=$OUT/_prev; rm -rf "${PREV:?}"; mkdir -p "$PREV"
for d in scan_summary.json mesh board mesh_board redscreen mesh_red mesh.log board.log mesh_board.log redscreen.log mesh_red.log; do
  [ -e "$OUT/$d" ] && mv "$OUT/$d" "$PREV/"; done
rmdir "$PREV" 2>/dev/null || echo "[M2] earlier outputs in $OUT moved to $PREV (not used)"
TIMES=$OUT/times.txt; : > "$TIMES"
STATUS=ok; SSTAT=""
T() {  # T NAME LOG CMD...: run CMD > LOG 2>&1, print + record wall ms; stop the script on failure
  local n=$1 lg=$2; shift 2; local t0 rc=0; t0=$(date +%s%N)
  echo "$(date +%T) step $n start (log $lg)"
  "$@" > "$lg" 2>&1 || rc=$?
  local ms=$(( ($(date +%s%N) - t0) / 1000000 )); echo "$n $ms" >> "$TIMES"
  echo "$(date +%T) step $n done $((ms / 1000)).$(( (ms % 1000) / 100 )) s (rc $rc)"
  [ $rc = 0 ] || { echo "step $n failed rc $rc, tail of $lg:" >&2; tail -n 15 "$lg" >&2; STATUS="failed: step $n rc $rc"; summary || true; exit $rc; }; }
summary() {  # writes OUT/scan_summary.json (also on a failed step: status 'failed: ...'; fixes 11-12 H3/M3)
$P - "$OUT" "$REC" "$IN" "$VIO" "$G" "$CFG" "$KRES" "$KQ" "$FULLV" "$COMMIT" "$TALLOC" "$BADIR" "$FV" "$FF" "$REUSE" "$FP" "$SREF" "$SSTAT" "$LENS" \
   "$STATUS" "$MAIN_COMMIT" "$WT" "$MINV" <<'EOF'
import hashlib, json, sys, time
from pathlib import Path
out, rec, inp, vio, g, cfg, kres, kq, fullv, commit, talloc, badir, fv, ff, reuse, fp, sref, sstat, lens, status, main_commit, wt, minv = sys.argv[1:]
out = Path(out)
times = {l.split()[0]: round(float(l.split()[1]) / 1000, 1) for l in (out / "times.txt").read_text().split("\n") if l.strip()}
ran = set(times)                 # M2 (fixes 11-12): only steps of THIS run are read (a failed step is in times.txt too)
def sha(p):
    try:
        return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
    except Exception:
        return None
def mesh(d):
    p = out / d / "summary.json"
    if d not in ran or not p.exists():
        return None
    m = json.loads(p.read_text()); s = m["poses"]["sim3_scale_would_be"]
    return dict(absdZ_p90_cm=(m.get("pooled_absdZ_cm") or {}).get("p90"), k_pct=(1 / s - 1) * 100, mesh_board_z_rms_cm=m.get("mesh_board_z_rms_cm"),
                n_kf=m.get("n_kf"), n_kf_depth=m.get("n_kf_depth"), n_kf_board_eval=m.get("n_kf_board_eval"),
                kf_absdZ_p90_cm_median=m.get("kf_board_absdZ_p90_cm_median"), board_cover_frac=m.get("mesh_board_cover_frac"),
                ffs_views=m.get("ffs_views", 3), ffs_fuse=m.get("ffs_fuse"), ffs_partners=m.get("ffs_partners"), ffs_s_total=m.get("ffs_s_total"),
                n_kf_by_pairs=m.get("n_kf_by_pairs"), n_kf_no_pair=m.get("n_kf_no_pair"),
                mesh_vertices=m.get("mesh_vertices"), tsdf_full=m.get("tsdf_full"), tsdf_blocks=m.get("tsdf_blocks"),
                n_kf_gt_not_ok_dropped=m.get("n_kf_gt_not_ok_dropped"), gt_ok_filter=m.get("gt_ok_filter"),
                depth_sigma=m.get("depth_sigma"), mesh_cam_depth_m=m.get("mesh_cam_depth_m"), lib_ffs=m.get("lib_ffs"))
kl = (Path(reuse) if reuse else out) / "kfba.log"
dev = next((l.strip() for l in open(kl) if l.startswith("[device]")), None) if kl.exists() else None
ba = json.loads((Path(badir) / "ba.json").read_text()) if (("kfba" in ran or reuse) and (Path(badir) / "ba.json").exists()) else {}
bs = json.loads((out / "board" / "board_scale.json").read_text()) if ("board" in ran and (out / "board" / "board_scale.json").exists()) else None
vj = out / "vio" / "B_o0.json"
vr = json.loads(vj.read_text()) if ("vio" in ran and vj.exists()) else None
man = json.loads((Path(inp) / "manifest.json").read_text()) if (Path(inp) / "manifest.json").exists() else None
r = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), commit=commit, rec=rec, input=inp, keyframe_only=(Path(inp) / "frames_kf").is_dir(),
         kf_res=None if fullv == "1" else int(kres), kf_quality=None if fullv == "1" else int(kq), live_poses=vio, gt=g, vio_cfg=cfg, tsdf_alloc=talloc,
         ffs_views=int(fv), ffs_fuse=ff, ffs_partners=fp, reuse=reuse or None, ba_dir=badir,
         kfba_device_line=dev, step_s=times, total_s=round(sum(times.values()), 1),
         sent=None if man is None else dict(n_keyframes=man["n_keyframes"], image_size=man["image_size"], jpeg_bytes=man["jpeg_bytes"],
                                            duration_s=man["duration_s"], jpeg_MB_per_min=man["jpeg_bytes"] / 1e6 / man["duration_s"] * 60,
                                            kf_per_min=man["n_keyframes"] / man["duration_s"] * 60, kf_select=man.get("kf_select")),
         ba=dict(timing=ba.get("timing")), mesh=mesh("mesh"), board=bs, mesh_board=mesh("mesh_board"))
r["status"] = status             # ok | failed: ... (fixes 11-12: written also when a step fails)
r["steps_run"] = sorted(ran)
r["vio"] = None if vr is None else {k: vr.get(k) for k in ("crashed", "bridge_rc", "frames_selected", "frames_fed", "poses", "wall_s")}
r["vio_crashed"] = None if vr is None else vr.get("crashed")
r["ba_quality"] = {k: ba.get(k) for k in ("n_kf", "kf_obs_min", "n_kf_vio_pose", "fixed_kf", "final_med_px", "final_rmse_px",
                                          "frac_inlier_1px", "sim3_rms_cm")} if ba else None
r["min_mesh_vertices"] = int(minv)
gtz = Path(inp) / g / "gt.npz"
r["provenance"] = dict(commit=commit, commit_dirty=commit.endswith("-dirty"), main_commit=main_commit or None, worktree=wt,
                       lens_file=(lens or str(gtz)), lens_sha=sha(lens) if (g == "lensonly" and lens) else sha(gtz),
                       vio_cfg_sha={f.name: sha(f) for f in sorted(Path(cfg).glob("*.yaml"))} if Path(cfg).is_dir() else None)
r["lens_only"] = (lens or None) if g == "lensonly" else None
if sref:
    rp = out / "redscreen" / "redscreen_scale.json"
    rj = json.loads(rp.read_text()) if ("redscreen" in ran and rp.exists()) else {}
    keys = ("s", "sigma", "scale_error_pct", "sigma_pct", "boot_sd", "thr_spread", "W_mm", "H_mm", "red_size_mm", "red_size_note", "ratio_WH",
            "ratio_ref", "n_kf", "n_kf_red_quad", "n_views_used", "line_rms_px_median", "bundle_med_abs_px", "bundle_rms_px", "corner_angle_deg",
            "excl", "status", "runtime_s")
    r["redscreen"] = {k: rj.get(k) for k in keys}
    r["scale_status"] = sstat
    r["scale_source"] = "redscreen" if sstat == "ok" else None
    r["scale_applied"] = (1 / rj["s"]) if sstat == "ok" else None
    r["mesh_red"] = mesh("mesh_red")
    r["final_mesh"] = str(out / ("mesh_red" if sstat == "ok" else "mesh") / "mesh.ply")
    if "redscreen" not in ran:
        r["redscreen"] = None
if g == "lensonly":     # stand-in GT = live VIO poses: k_pct / board numbers are not GT numbers
    for mk in ("mesh", "mesh_board", "mesh_red"):
        if r.get(mk):
            r[mk]["note"] = "lens-only stand-in GT: k_pct and board numbers meaningless"
(out / "scan_summary.json").write_text(json.dumps(r, indent=1))
print(json.dumps({k: v for k, v in r.items() if k != "ba"}, indent=1))
EOF
}
mesh_gate() {  # mesh_gate DIR: empty / near-empty mesh -> failure (M3; the TSDF writes an empty mesh.ply without complaint)
  local nv; nv=$($P -c "import json;print(json.load(open('$1/summary.json'))['mesh_vertices'])" 2>/dev/null || echo 0)
  if [ "$nv" -lt "$MINV" ]; then
    echo "!!!! MESH GATE: $1/mesh.ply has $nv vertices (< --min-mesh-vertices $MINV)" >&2
    STATUS="failed: $(basename "$1") $nv vertices < $MINV"; summary || true; exit 5
  fi
}

REDUCED=0; [ -d "$REC/frames_kf" ] && REDUCED=1
if [ $REDUCED = 1 ]; then
  [ -e "$REC/cam5.mp4" ] && { echo "$REC has frames_kf/ and cam5.mp4 -- not a keyframe-only recording" >&2; exit 1; }
  [ $FULLV = 1 ] && { echo "--full-video needs a full recording (cam5.mp4), $REC is keyframe-only" >&2; exit 1; }
fi
if [ -z "$G" ] && [ -z "$LENS" ]; then [ -d "$REC/imu_gt_spl_cal1022" ] && G=imu_gt_spl_cal1022 || G=imu_gt_spl; fi
if [ $G_SET = 1 ] && [ -z "$LENS" ] && [ ! -f "$REC/$G/gt.npz" ]; then echo "--gt $G: $REC/$G/gt.npz missing (typo?) -- stopping (M5)" >&2; exit 1; fi
if [ -n "$LENS" ] || [ ! -f "$REC/$G/gt.npz" ]; then         # no GT folder: lens-only stand-in (redscreen 11-11)
  if [ -n "$LENS" ] && [ -n "$G" ]; then echo "--gt $G and --lens both given" >&2; exit 1; fi
  if [ -z "$LENS" ]; then
    FD=$($P -c "import json;print(json.load(open('$REC/meta.json')).get('focus_dpt_request'))" 2>/dev/null || echo none)
    case "$(basename "$REC")|$FD" in
      *std*"|0.5") LENS=$WT/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json
                   echo "[lens] no GT folder $REC/$G -> std 0.5 dpt recording: lens-only fallback $LENS";;
      *) echo "no GT folder $REC/$G and no --lens: lens unknown (focus_dpt_request $FD, $(basename "$REC")); pass --lens FILE" >&2; exit 1;;
    esac
  fi
  [ -f "$LENS" ] || { echo "--lens $LENS missing" >&2; exit 1; }
  [ $CFG_SET = 1 ] || echo "!! lens-only: VIO config is cfg_gen/c4f60 (0.5 dpt app lens) whatever --lens says; pass --cfg for another lens (M5)" >&2
  G=lensonly; LENSONLY=1; CFG=${CFG:-$B/cfg_gen/c4f60}
fi
if [ -z "$CFG" ]; then [ "$G" = imu_gt_spl_cal1022 ] && CFG=$B/cfg_gen_hs222/c4f60 || CFG=$B/cfg_gen/c4f60; fi
echo "== run_scanner $(date '+%F %T'): REC $REC ($([ $REDUCED = 1 ] && echo keyframe-only || echo full recording)) -> $OUT"
echo "   gt $G, input $([ $FULLV = 1 ] && echo 'full video (old)' || echo "keyframes ${KRES} JPEG q${KQ}"), BA $BADEV, SIFT $SIFTDEV, TSDF alloc $TALLOC, kf $KFSEL$([ $KFSEL = motion ] && echo " ${KFD}cm/${KFR}deg win $KFW $KFB"), git ${COMMIT:0:12}"
echo "   gt $G, input $([ $FULLV = 1 ] && echo 'full video (old)' || echo "keyframes ${KRES} JPEG q${KQ}"), BA $BADEV, SIFT $SIFTDEV, TSDF alloc $TALLOC, git ${COMMIT:0:12}"
echo "   ffs views $FV (fuse $FF, partners $FP)${REUSE:+, reuse vio/reduce/kfba from $REUSE}"

# ---- vio (stand-in for the phone's live poses)
BADIR=$OUT/ba; IN=$REC
if [ -n "$REUSE" ]; then
  BADIR=$REUSE/ba
  if [ $REDUCED = 1 ]; then VIO=$REC/phone_live_poses.jsonl.gz
  elif [ $FULLV = 1 ]; then VIO=${VIO:-$REUSE/vio/B_o0.poses.jsonl.gz}
  else IN=$REUSE/red/$(basename "$REC")_kf; VIO=$IN/phone_live_poses.jsonl.gz; fi
  [ -d "$IN" ] || { echo "--reuse: input $IN missing" >&2; exit 1; }
elif [ $REDUCED = 1 ]; then
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
if [ $LENSONLY = 1 ]; then                                    # lens-only stand-in GT in REC/lensonly (needs the live poses)
  T lensonly "$OUT/lensonly.log" $P "$WT/scan3d/scanA/lensonly_gt.py" "$REC" "$VIO" "$LENS" lensonly
fi

# ---- reduce (stand-in for the phone app sending keyframes)
if [ -z "$REUSE" ] && [ $REDUCED = 0 ] && [ $FULLV = 0 ]; then
  IN=$OUT/red/$(basename "$REC")_kf
  XR=""; [ "$KRES" = 1920 ] && XR=--full-res
  XS=""; [ $KFSEL = motion ] && XS="--kf-select motion --kf-dist $($P -c "print($KFD/100)") --kf-rot $KFR --kf-win $KFW --kf-blur $KFB"
  T reduce "$OUT/reduce.log" $P "$WT/scan3d/runs/kfonly_20261111/make_reduced.py" "$REC" "$VIO" --out "$IN" --quality "$KQ" $XR --gt "$G" $XS
  [ -e "$IN/cam5.mp4" ] && { echo "reduced folder holds a video" >&2; exit 9; }
  VIO=$IN/phone_live_poses.jsonl.gz
fi
if [ -d "$IN/frames_kf" ]; then
  KSZ=$($P -c "import cv2,glob;f=cv2.imread(sorted(glob.glob('$IN/frames_kf/*.jpg'))[0]);print(f'{f.shape[1]}x{f.shape[0]}')")
  case "$KRES|$KSZ" in "1920|1920x1080"|"960|960x540") ;; *)                 # M9: lens K is for 1920x1080; refuse a size we did not ask for
    echo "frames_kf images are $KSZ but --kf-res is $KRES (expected $([ "$KRES" = 1920 ] && echo 1920x1080 || echo 960x540)); stopping (M9)" >&2; exit 1;; esac
  echo "[input] keyframe-only: $IN/frames_kf ($(ls "$IN/frames_kf" | wc -l) jpg, $($P -c "import cv2,glob;f=cv2.imread(sorted(glob.glob('$IN/frames_kf/*.jpg'))[0]);print(f'{f.shape[1]}x{f.shape[0]}')"), $(du -sh "$IN/frames_kf" | cut -f1)), cam5.mp4 absent: $([ -e "$IN/cam5.mp4" ] && echo NO || echo yes)"
else
  echo "[input] full video $IN/cam5.mp4 (old path)"
fi

# ---- kf_ba (GPU BA); a motion-selected keyframe folder -> kf_ba takes its manifest list (also for a phone-sent folder)
KL=""
if [ -f "$IN/manifest.json" ] && [ "$($P -c "import json;print((json.load(open('$IN/manifest.json')).get('kf_select') or {}).get('rule','interval'))")" = motion ]; then
  KL="--kf-list $IN/manifest.json"; echo "[kf] motion-selected keyframes: kf_ba --kf-list $IN/manifest.json"
fi
if [ -n "$REUSE" ]; then
  echo "[reuse] BA poses $BADIR/poses_ba.jsonl.gz (kfba skipped)"
else
  T kfba "$OUT/kfba.log" $P "$WT/scan3d/scanA/kf_ba.py" "$IN" --vio "$VIO" --out "$OUT/ba" --gt "$G" --kf-from-vio --match-ms 1 \
    --feat sift --loop none --seed 0 --cache-root "$CACHE" --ba-device "$BADEV" --sift-device "$SIFTDEV" $KL $KFX
  grep -m1 '^\[device\]' "$OUT/kfba.log" || true
  rm -f "$OUT/ba/db.db"
fi

# ---- FFS 960 + TSDF
T mesh "$OUT/mesh.log" $P "$WT/scan3d/scanA/scan_offline.py" "$IN" --out "$OUT/mesh" --gt "$G" --vio "$BADIR/poses_ba.jsonl.gz" \
  --compress --kf-step 1 --tsdf-alloc "$TALLOC" $FFSX $MSX
mesh_gate "$OUT/mesh"

# ---- optional board scale
if [ -n "$BOARD" ]; then
  if [ "$BOARD" = auto ]; then case $(basename "$REC") in
    *165506*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_5x3.json;; *20261020_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_6x6.json;;
    *20261021_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_fit_6x6.json;; *20261022_*) BOARD=$MAIN/scan3d/config/charuco/board_monitor_6x6.json;;
    *) echo "--board auto: no board known for $(basename "$REC")" >&2; exit 1;; esac; fi
  [ "$KRES" = 960 ] && [ $FULLV = 0 ] && { echo "board_scale needs 1920x1080 keyframes (--kf-res 1920)" >&2; exit 1; }
  T board "$OUT/board.log" $P "$WT/scan3d/scanA/board_scale.py" "$IN" --poses "$BADIR/poses_ba.jsonl.gz" --board "$BOARD" \
    --lens "$IN/$G/gt.npz" --out "$OUT/board"
  if [ $BOARD_MESH = 1 ]; then
    T mesh_board "$OUT/mesh_board.log" $P "$WT/scan3d/scanA/scan_offline.py" "$IN" --out "$OUT/mesh_board" --gt "$G" \
      --vio "$OUT/board/poses_board.jsonl.gz" --compress --kf-step 1 --tsdf-alloc "$TALLOC" $FFSX $MSX
    mesh_gate "$OUT/mesh_board"
  fi
fi

# ---- optional red-screen scale (redscreen 11-11): fail loudly, keep the unscaled mesh
SSTAT=""
if [ "$SREF" = redscreen ]; then
  LZ=$IN/$G/gt.npz; [ $LENSONLY = 1 ] && LZ=$LENS
  t0=$(date +%s%N); rc=0; echo "$(date +%T) step redscreen start (log $OUT/redscreen.log)"
  $P "$WT/scan3d/scanA/redscreen_scale.py" "$IN" --poses "$BADIR/poses_ba.jsonl.gz" --lens "$LZ" --out "$OUT/redscreen" \
    --red-size "$REDSIZE" > "$OUT/redscreen.log" 2>&1 || rc=$?
  ms=$(( ($(date +%s%N) - t0) / 1000000 )); echo "redscreen $ms" >> "$TIMES"; echo "$(date +%T) step redscreen done $((ms / 1000)) s (rc $rc)"
  if [ $rc = 0 ]; then
    SSTAT=ok
    T mesh_red "$OUT/mesh_red.log" $P "$WT/scan3d/scanA/scan_offline.py" "$IN" --out "$OUT/mesh_red" --gt "$G" \
      --vio "$OUT/redscreen/poses_redscreen.jsonl.gz" --compress --kf-step 1 --tsdf-alloc "$TALLOC" $FFSX $MSX
    mesh_gate "$OUT/mesh_red"
  else
    SSTAT="failed: $(grep -m1 'REDSCREEN SCALE FAILED' "$OUT/redscreen.log" | sed 's/^REDSCREEN SCALE FAILED: //' || true) (rc $rc, log $OUT/redscreen.log)"
    echo "!!!! RED-SCREEN SCALE FAILED -- the result is NOT scaled (only OUT/mesh). $SSTAT" >&2; tail -n 5 "$OUT/redscreen.log" >&2
    STATUS="scale failed (unscaled mesh only)"
  fi
fi
[ $KEEPC = 1 ] || rm -rf "$CACHE"

# ---- summary
summary
echo "== run_scanner done $(date +%T)"
if [ -n "$SREF" ] && [ "$SSTAT" != ok ]; then echo "!!!! scale reference $SREF FAILED: $SSTAT -- unscaled result in $OUT/mesh" >&2; exit 3; fi
