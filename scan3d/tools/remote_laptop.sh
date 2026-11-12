#!/usr/bin/env bash
# Laptop worker for the offline scanner pipeline (docs/laptop_worker.md). Run from THIS PC's WSL.
# Laptop = HP Victus 15 (Ryzen 7 8845HS, RTX 4060 Laptop 8 GB), WSL Ubuntu-22.04 user `system`, same absolute paths as here.
#
#   tools/remote_laptop.sh push [WT]                  sync code of worktree WT (default: the one holding this script) -> same path there
#   tools/remote_laptop.sh job REC STEP [opts]        ONE job: take the laptop lock, push code, send REC (+ --send files),
#                                                     run STEP there, pull results back here, delete REC + caches there, unlock
#   tools/remote_laptop.sh status                     lock holder + tail of the last job log
#   tools/remote_laptop.sh pull RID[/TAG]             copy scan3d/runs/laptop/RID[/TAG] back again (e.g. after a dropped link)
#   tools/remote_laptop.sh sh 'cmd'                   one command there (cwd = WT/scan3d)
#   tools/remote_laptop.sh unlock                     remove the lock (refuses while a laptop job runs; FORCE=1 overrides)
#
# REC  = recording dir name under scan3d/recordings or a unique part of it (165506 -> hs_20261024_165506_std60).
# STEP = vio | kfba | mesh | board | all (= vio kfba mesh) | scan | 'cmd:<shell command>' (cwd = WT, $R $O $P $G $CACHE set)
#   scan  scan3d/scanA/run_scanner.sh (needs a WT that has it: scanner-defaults-20261111) = the new scanner defaults (11-11):
#         VIO c4f60 replay -> keyframes 1920 JPEG q90 (no video) -> kf_ba GPU BA -> FFS 960 + TSDF [-> board with --args '--board auto']
#         -> O/scan/ (scan_summary.json). --gt / --cfg only when given (run_scanner picks cal1022 + hs222 for H); other flags via --args
#   vio   lp_replay.py with the live config c4f60 (0.5 dpt lens)            -> O/vio/B_o0.{poses.jsonl.gz,json,...}
#   kfba  kf_ba.py SIFT, seed 0, no loop (featloop S)                          -> O/ba_S/   (input: --vio FILE or O/vio)
#   mesh  scan_offline.py FFS 960x540 + TSDF 5 mm on the BA poses             -> O/mesh_S/ (input: --poses FILE or O/ba_S)
#   board board_scale.py (needs a WT that has scan3d/scanA/board_scale.py, e.g. the boardscale worktree) -> O/board/
# O = scan3d/runs/laptop/<REC short id>[/<--tag>] inside WT (same relative path here and there; pull OVERWRITES same names).
# opts: --gt imu_gt_spl   --cfg DIR (VIO config, default build_livepose/cfg_gen/c4f60)   --vio FILE   --poses FILE
#       --board FILE (board json, default by recording)   --send FILE|DIR (repeatable; copied to the same absolute path there)
#       --tag NAME (results under O/NAME)   --keep (do not delete the recording there)   --args 'extra args for the step script'
# Env: LAPTOP=user@host (default LAN cable 169.254.78.233, falls back to Wi-Fi 192.168.1.76), LOCK_WAIT_S (default 6 h), POLL_MAX_S (default 12 h),
#      JOB_TIMEOUT_S (default 0 = off): kill a hung laptop job after that many s (rc 124; e.g. 14400 = 4 h; a normal scan takes 3-10 min).
# Lock (fixes 11-12 H2/M8): holder = host:pid:date:job id. A lock whose orchestrator here is gone is removed only when no laptop
# job (laptop_jobs/*/run.sh) runs; the EXIT trap does not wipe the recording / cache or unlock while its laptop job still runs
# (SIGTERM / SIGHUP / dropped link) -> the orphan job finishes, the next job waits for it. Cache = laptop_cache/<job id> (per job).
set -euo pipefail
LAN=a3162@169.254.78.233; WIFI=a3162@192.168.1.76
HERE=$(cd "$(dirname "$0")/../.." && pwd)                 # worktree root that holds this script
WT=${WT:-$HERE}
MAIN=/home/system/scanner/StereoDepthSeg                  # holds venv_ffs, tools/Fast-FoundationStereo, recordings
RECS=$MAIN/scan3d/recordings
P=$MAIN/venv_ffs/bin/python
JOBS=/home/system/scanner/laptop_jobs                     # there: one dir per job (log, rc, cmd)
LOCK=/home/system/scanner/laptop_gpu.lock.d               # there: mkdir lock = one job at a time
CACHE=/home/system/scanner/laptop_cache                   # there: kf_ba image cache (deleted after each job)

host() {
  if [ -n "${LAPTOP:-}" ]; then echo "$LAPTOP"; return; fi
  for h in $LAN $WIFI; do
    ssh -o BatchMode=yes -o ConnectTimeout=4 -o LogLevel=ERROR "$h" "echo ok" >/dev/null 2>&1 && { echo "$h"; return; }
  done
  echo "laptop unreachable (LAN $LAN, Wi-Fi $WIFI)" >&2; exit 2
}
H=$(host)
SSHO=(-o BatchMode=yes -o LogLevel=ERROR -o ServerAliveInterval=30)
rsh() { ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system -- bash -s" | tr -d '\0'; }    # bash script on stdin
rin() { ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system -- bash -c \"cd / && tar xzf -\"" | tr -d '\0'; }  # tar.gz on stdin

push() {  # push [WT]: code only (no venv, recordings, runs, builds, weights)
  local wt=${1:-$WT} t0=$SECONDS
  wt=$(cd "$wt" && pwd)
  (cd / && tar --exclude="${wt#/}/venv_ffs" --exclude="${wt#/}/scan3d/recordings" --exclude="${wt#/}/scan3d/runs" \
     --exclude="${wt#/}/scan3d/ov_bridge/android" --exclude="${wt#/}/scan3d/ov_bridge/build" \
     --exclude="${wt#/}/tools/Fast-FoundationStereo" --exclude="${wt#/}/phone_stereo" --exclude='*.pt' --exclude='*.pth' \
     --exclude='__pycache__' --exclude='.git' -cf - "${wt#/}" | gzip -1) | rin
  # scripts that live under runs/ (e.g. runs/livepose_20261030/lp_replay.py): .py/.sh only, no data
  (cd / && find "${wt#/}/scan3d/runs" \( -name '*.py' -o -name '*.sh' \) -size -2M -print0 | tar --null -T - -cf - | gzip -1) | rin
  rsh <<EOF
mkdir -p $wt/scan3d/runs $RECS
[ -e $wt/scan3d/recordings ] || ln -s $RECS $wt/scan3d/recordings
EOF
  echo "push $wt done $((SECONDS - t0)) s"
}

send() {  # send ABS_PATH... : same absolute paths there
  local t0=$SECONDS
  (cd / && tar -cf - "${@#/}" | gzip -1) | rin
  echo "send $* done $((SECONDS - t0)) s"
}

resolve_rec() {
  local m; m=$(cd "$RECS" && ls -d *"$1"* 2>/dev/null || true)
  [ "$(echo "$m" | grep -c .)" = 1 ] || { echo "recording '$1': need exactly one match in $RECS, got: $m" >&2; exit 1; }
  echo "$m"
}

ALIVE_RE="$JOBS/[^ ]*/run\\.sh"                            # any laptop job (fixes 11-12 H2: the lock follows the laptop job, not only this pid)
holder_alive_here() {  # holder_alive_here HOLDER: 0 if its orchestrator is a live remote_laptop.sh on this host
  local hn pid; hn=${1%%:*}; pid=$(echo "$1" | cut -d: -f2)
  [ "$hn" = "$(hostname)" ] && [ -n "$pid" ] || return 2          # other host: unknown
  tr '\0' ' ' 2>/dev/null < "/proc/$pid/cmdline" | grep -q remote_laptop.sh   # kill -0 alone is fooled by a reused pid (M8); full cmdline, not ps (cut at $COLUMNS, N5)
}
lock() {  # mkdir lock there; holder = this host + pid + date + job id; waits up to LOCK_WAIT_S
  local t0=$SECONDS me="$(hostname):$$:$(date +%F_%T):$1"
  while :; do
    r=$(echo "mkdir $LOCK 2>/dev/null && echo '$me' > $LOCK/holder && echo GOT || cat $LOCK/holder 2>/dev/null" | rsh)
    [ "${r%%$'\n'*}" = GOT ] && { LOCK_ME=$me; return 0; }
    rc_h=0; holder_alive_here "$r" || rc_h=$?
    if [ -n "$r" ] && [ $rc_h = 1 ]; then
      # orchestrator gone: remove ONLY if the holder line is unchanged AND no laptop job runs (one command there = no read/remove gap)
      v=$(echo "cur=\$(cat $LOCK/holder 2>/dev/null); if [ \"\$cur\" = '$r' ] && ! pgrep -f '$ALIVE_RE' >/dev/null; then
            oid=\${cur##*:}; [ -f $JOBS/\$oid/cleanup ] && bash $JOBS/\$oid/cleanup; rm -rf $LOCK; echo REMOVED; else echo KEPT; fi" | rsh || true)
      case "$v" in
        *REMOVED*) echo "stale lock ($r): orchestrator gone, no laptop job alive -> removed (+ its cache)"; continue;;
        *) echo "$(date +%T) lock holder ($r) has no orchestrator here but its laptop job still runs -> waiting";;
      esac
    else
      echo "$(date +%T) laptop busy ($r), waiting"
    fi
    [ $((SECONDS - t0)) -gt "${LOCK_WAIT_S:-21600}" ] && { echo "lock wait timeout, holder: $r" >&2; exit 3; }
    sleep 60
  done
}
unlock() {  # remove the lock only when it is ours (LOCK_ME) and our laptop job is not running
  echo "cur=\$(cat $LOCK/holder 2>/dev/null); [ \"\$cur\" = '${LOCK_ME:-}' ] && rm -rf $LOCK && echo unlocked || echo \"lock not ours (\$cur), kept\"" | rsh
}
job_alive() {  # job_alive ID -> ALIVE | DONE | NOTSTARTED (prints UNKNOWN when the link is down)
  echo "if [ -f $JOBS/$1/rc ]; then echo DONE; elif pgrep -f '$JOBS/$1/run\\.sh' >/dev/null; then echo ALIVE;
        elif [ -f $JOBS/$1/started ]; then sleep 3; [ -f $JOBS/$1/rc ] && echo DONE || { pgrep -f '$JOBS/$1/run\\.sh' >/dev/null && echo ALIVE || echo DONE; }; else echo NOTSTARTED; fi" \
    | rsh 2>/dev/null | head -n 1 || true
}
cleanup() {  # EXIT trap of job(): never wipe cache / recording or unlock while the laptop job still runs (H2)
  set +e
  echo "== cleanup"
  [ -n "${LOCK_ME:-}" ] || return 0                         # never got the lock: nothing of ours there
  local st; st=$(job_alive "$JOB_ID"); st=${st:-UNKNOWN}
  if [ "$st" = ALIVE ] || [ "$st" = UNKNOWN ]; then
    echo "!! laptop job $JOB_ID is $st: lock, cache $JC and recording kept. It keeps running there; get results later with" >&2
    echo "!!   remote_laptop.sh pull <RID>[/TAG]; the next job waits for it, then removes the lock and this job's cache itself." >&2
    return 0
  fi
  # N6 (11-12): one command there; the recording is deleted (and the lock removed) only while the lock is still ours.
  # The per-job cache $JC is only ours -> always removed.
  echo "rm -rf $JC; cur=\$(cat $LOCK/holder 2>/dev/null); if [ \"\$cur\" = '${LOCK_ME:-}' ]; then $( [ $KEEP = 1 ] || echo "rm -rf $R;" ) rm -rf $LOCK && echo unlocked; else echo \"lock not ours (\$cur): recording $R and lock kept\"; fi" | rsh
}

step_cmd() {  # step_cmd STEP -> shell line run there (cwd WT); uses vars below
  case $1 in
    vio)  echo "mkdir -p $O/vio && cd scan3d/runs/livepose_20261030 && /usr/bin/time -f 'TIME vio %e s maxrss %M kB' $P lp_replay.py $R $CFG $WT/$O/vio/B_o0 --bridge $BRIDGE --vocab $WT/scan3d/ov_bridge/loop/voc_k10L4.voc $XARGS > $WT/$O/vio.log 2>&1" ;;
    kfba) echo "/usr/bin/time -f 'TIME kfba %e s maxrss %M kB' $P scan3d/scanA/kf_ba.py $R --vio ${VIO:-$O/vio/B_o0.poses.jsonl.gz} --out $O/ba_S --gt $G --feat sift --loop none --cache-root $JC --seed 0 --keep-db $XARGS > $O/kfba.log 2>&1" ;;
    mesh) echo "/usr/bin/time -f 'TIME mesh %e s maxrss %M kB' $P scan3d/scanA/scan_offline.py $R --out $O/mesh_S --gt $G --vio ${POSES:-$O/ba_S/poses_ba.jsonl.gz} $XARGS > $O/mesh.log 2>&1" ;;
    scan) echo "SCANNER_COMMIT=$(git -C "$WT" rev-parse HEAD)$(git -C "$WT" diff --quiet HEAD || echo -dirty) SCANNER_MAIN_COMMIT=$(git -C "$MAIN" rev-parse HEAD 2>/dev/null)$(git -C "$MAIN" diff --quiet HEAD 2>/dev/null || echo -dirty) bash scan3d/scanA/run_scanner.sh $R $WT/$O/scan --cache $JC/scan $( [ $GT_SET = 1 ] && echo --gt $G ) $( [ $CFG_SET = 1 ] && echo --cfg $CFG ) $( [ -n "$VIO" ] && echo --vio $(cd "$WT" && realpath -m "$VIO") ) $XARGS > $O/scan.log 2>&1" ;;
    board) echo "/usr/bin/time -f 'TIME board %e s maxrss %M kB' $P scan3d/scanA/board_scale.py $R --poses ${POSES:-$O/ba_S/poses_ba.jsonl.gz} --board $BOARD --lens $R/$G/gt.npz --out $O/board $( [ -d $R/imu_gt_rs ] && echo --check-gt-corners $R/imu_gt_rs/gt.npz ) $XARGS > $O/board.log 2>&1" ;;
    cmd:*) echo "${1#cmd:}" ;;
    *) echo "unknown step $1" >&2; exit 1 ;;
  esac
}

job() {
  local rec=$1 st=$2; shift 2
  G=imu_gt_spl; CFG=/home/system/scanner/build_livepose/cfg_gen/c4f60; BRIDGE=/home/system/scanner/build_livepose/bridge/ov_bridge
  VIO=""; POSES=""; BOARD=""; TAG=""; KEEP=0; XARGS=""; SENDS=(); GT_SET=0; CFG_SET=0
  while [ $# -gt 0 ]; do case $1 in
    --gt) G=$2; GT_SET=1; shift;; --cfg) CFG=$2; CFG_SET=1; shift;; --vio) VIO=$2; shift;; --poses) POSES=$2; shift;; --board) BOARD=$2; shift;;
    --send) SENDS+=("$(cd "$WT" && realpath -m "$2")"); shift;; --tag) TAG=$2; shift;; --keep) KEEP=1;; --args) XARGS=$2; shift;;
    *) echo "bad option $1"; exit 1;; esac; shift; done
  local name; name=$(resolve_rec "$rec"); R=$RECS/$name
  local rid; rid=$(echo "$name" | sed -E 's/^hs_//')
  O=scan3d/runs/laptop/$rid${TAG:+/$TAG}
  if [ -z "$BOARD" ]; then case $name in *165506*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_5x3.json;;
    hs_20261020_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_6x6.json;; hs_20261021_*) BOARD=$MAIN/scan3d/config/charuco/board_laptop_fit_6x6.json;;
    hs_20261022_*) BOARD=$MAIN/scan3d/config/charuco/board_monitor_6x6.json;; esac; fi
  for f in "$VIO" "$POSES"; do [ -n "$f" ] && SENDS+=("$(cd "$WT" && realpath -m "$f")"); done
  if [ "$st" = scan ]; then                    # VIO configs run_scanner may pick (the laptop has only cfg_gen/ by default)
    [ -f "$WT/scan3d/scanA/run_scanner.sh" ] || { echo "WT $WT has no scan3d/scanA/run_scanner.sh (use WT=~/scanner/StereoDepthSeg-defaults)" >&2; exit 1; }
    for c in /home/system/scanner/build_livepose/cfg_gen_hs222/c4f60 "$CFG"; do [ -d "$c" ] && SENDS+=("$c"); done
  fi
  local steps=("$st"); [ "$st" = all ] && steps=(vio kfba mesh)
  # N3 (11-12): unique id (time + rec + step + tag + pid + random) -> two same-second jobs never share a job dir / rc file / cache
  local tg=${TAG//[^A-Za-z0-9_.-]/_}
  JOB_ID=$(date +%Y%m%d_%H%M%S)_${rid}_${st%%:*}${tg:+_$tg}_$$_$(od -An -N2 -tx2 /dev/urandom | tr -dc 0-9a-f); local id=$JOB_ID
  JC=$CACHE/$id                                # per-job cache dir there (fixes 11-12 H2: an orphan job keeps its own cache)
  echo "== job $id on $H: $name, steps '${steps[*]}', out $O"
  lock "$id"
  trap cleanup EXIT
  local t0=$SECONDS
  push "$WT"
  (cd "$RECS" && tar --exclude="$name/scanA_cache" --exclude="$name/cache" -cf - "$name") \
    | ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system -- bash -c \"cd $RECS && tar xf -\"" | tr -d '\0'
  echo "send rec $((SECONDS - t0)) s ($(du -sh --exclude=scanA_cache --exclude=cache "$R" | cut -f1))"
  [ ${#SENDS[@]} -gt 0 ] && send "${SENDS[@]}"
  { echo "set -e; D=$JOBS/$id; mkdir -p \$D $JC; cd $WT; mkdir -p $O"
    echo "echo '$( [ $KEEP = 1 ] && echo "rm -rf $JC" || echo "rm -rf $R $JC")' > \$D/cleanup; touch \$D/started"
    echo "export FFS_ROOT=$MAIN SCAN3D_GPU_POLL_S=3 PATH=/usr/local/cuda/bin:\$PATH"
    echo "cat > \$D/run.sh <<'__RUN__'"
    echo "cd $WT; set -e; R=$R; O=$O; P=$P; G=$G; CACHE=$JC; WT=$WT"
    for s in "${steps[@]}"; do echo "echo \"\$(date +%T) step ${s%%:*} start\"; ( $(step_cmd "$s") ); echo \"\$(date +%T) step ${s%%:*} done\""; done
    echo "__RUN__"
    # setsid -f: a plain nohup/& child is killed when this wsl.exe session ends (tested 11-11)
    # N4 (11-12): JOB_TIMEOUT_S > 0 -> the job is killed after that many seconds (timeout: TERM to its process group, KILL 60 s later), rc 124/137
    local to=""; [ "${JOB_TIMEOUT_S:-0}" -gt 0 ] && to="timeout -k 60 ${JOB_TIMEOUT_S}"
    echo "setsid -f bash -c '$to bash $JOBS/$id/run.sh > $JOBS/$id/job.log 2>&1; echo \$? > $JOBS/$id/rc' >/dev/null 2>&1 < /dev/null"
  } | rsh
  echo "running (log there: $JOBS/$id/job.log)"
  while :; do                                   # poll (ssh may drop; the job keeps running there under setsid)
    r=$(echo "cat $JOBS/$id/rc 2>/dev/null || { pgrep -f '$JOBS/$id/run.sh' >/dev/null || { sleep 5; [ -f $JOBS/$id/rc ] && cat $JOBS/$id/rc || echo 97; }; }" | rsh || true)
    rc=$(echo "$r" | head -n 1); [[ "$rc" =~ ^[0-9]+$ ]] && break   # 97 = job process gone without rc (killed / WSL restarted)
    if [ $((SECONDS - t0)) -gt "${POLL_MAX_S:-43200}" ]; then    # M8: no endless poll; the EXIT trap keeps the lock while the job runs
      echo "poll timeout after ${POLL_MAX_S:-43200} s: job $id still not done there (log $JOBS/$id/job.log)" >&2; exit 4; fi
    sleep 20
  done
  echo "== job rc $rc ($((SECONDS - t0)) s total)"; echo "cat $JOBS/$id/job.log" | rsh | tail -n 20
  local tp=$SECONDS
  # binary stream: NOT through rsh (its tr -d '\0' would delete the NUL bytes of the tar and corrupt it; seen 11-11)
  ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system --cd $WT -- tar -cf - $O" | (cd "$WT" && tar xf -)
  ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system -- cat $JOBS/$id/job.log" | tr -d '\0' > "$WT/$O/job_$id.log" || true
  echo "pulled $O ($((SECONDS - tp)) s) -> $WT/$O"
  return "$rc"
}

case "${1:-}" in
  push) push "${2:-$WT}" ;;
  job) shift; job "$@" ;;
  status) echo "cat $LOCK/holder 2>/dev/null || echo 'lock free'; d=\$(ls -td $JOBS/*/ 2>/dev/null | head -1); [ -n \"\$d\" ] && { echo \"last job \$d rc \$(cat \$d/rc 2>/dev/null || echo running)\"; tail -n 5 \$d/job.log; }; nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader" | rsh ;;
  pull) ssh "${SSHO[@]}" "$H" "wsl -d Ubuntu-22.04 -u system --cd $WT -- tar -cf - scan3d/runs/laptop/$2" | (cd "$WT" && tar xvf - | wc -l | sed 's/^/files pulled: /') ;;
  sh) echo "cd $WT/scan3d && $2" | rsh ;;
  unlock) [ "${FORCE:-0}" = 1 ] || { a=$(echo "pgrep -af '$ALIVE_RE' || true" | rsh); [ -z "$a" ] || { echo "a laptop job still runs ($a): not unlocking (FORCE=1 to override)" >&2; exit 1; }; }
          echo "rm -rf $LOCK" | rsh; echo unlocked ;;
  *) sed -n '2,/^set -euo/p' "$0" | grep '^#'; exit 1 ;;
esac
