#!/usr/bin/env bash
# Run VIO bench cells on the second PC "entropy" (docs/entropy_worker.md). Run this from THIS PC's WSL (scan3d dir).
#
#   tools/remote_entropy.sh push                         # sync code (repo minus venv/recordings/runs/big dirs) -> entropy
#   tools/remote_entropy.sh run TAG CMDS.txt [-j 8]      # push CMDS.txt, run each line on entropy (cwd scan3d), N parallel;
#                                                        #   blocks until all lines finish (run it with nohup/& if long)
#   tools/remote_entropy.sh status TAG                   # done/failed counts + last log lines
#   tools/remote_entropy.sh pull TAG runs/DIR [...]      # copy files under these dirs written since TAG started -> here
#   tools/remote_entropy.sh sh 'cmd'                     # one shell command on entropy (cwd scan3d)
#
# CMDS.txt: one shell command per line, e.g.
#   ../venv_ffs/bin/python imu_learn/lens_switch_ab.py job 134550_o0_new
# Each line gets nice -n 5; its stdout+stderr go to ~/scanner/entropy_jobs/TAG/NNN.log on entropy, exit code to NNN.rc.
# Recordings on entropy have NO videos: only the small files + caches copied on purpose. A cell whose frame cache is
# missing fails loudly (cache dirs are read-only, cam5.mp4 is a placeholder) - check status for rc != 0.
set -euo pipefail
HOST=${ENTROPY:-a3162@100.91.12.2}
SSH=${SSH:-ssh.exe}
REPO=/home/system/scanner/StereoDepthSeg
S3=$REPO/scan3d
JOBS=/home/system/scanner/entropy_jobs

rsh() {  # rsh <<'EOF' ... EOF : bash script on entropy WSL (stdin), as user system
  "$SSH" -o BatchMode=yes -o LogLevel=ERROR "$HOST" "wsl -d Ubuntu-22.04 -u system -- bash -s" | tr -d '\0'
}

push() {
  local t0=$SECONDS
  (cd / && tar --exclude="${REPO#/}/venv_ffs" --exclude="${S3#/}/recordings" --exclude="${S3#/}/runs" \
     --exclude="${S3#/}/ov_bridge/android" --exclude="${S3#/}/ov_bridge/build" --exclude="${S3#/}/tools/ov_bridge_*/build" \
     --exclude="${REPO#/}/phone_stereo" --exclude='*.pt' --exclude='__pycache__' -cf - "${REPO#/}" | gzip -1) \
    | "$SSH" -o BatchMode=yes -o LogLevel=ERROR "$HOST" "wsl -d Ubuntu-22.04 -u system -- bash -c \"cd / && tar xzf -\"" \
    | tr -d '\0'
  echo "push done $((SECONDS - t0)) s"
}

run() {
  local tag=$1 cmds=$2; shift 2
  local par=8
  [ "${1:-}" = "-j" ] && par=$2
  [ -f "$cmds" ] || { echo "no $cmds"; exit 1; }
  push
  { echo "set -e; D=$JOBS/$tag; mkdir -p \$D; cd \$D; touch started"
    echo "cat > cmds.txt <<'__CMDS__'"; grep -v '^\s*\(#\|$\)' "$cmds"; echo "__CMDS__"
    cat <<EOF
n=\$(wc -l < cmds.txt); echo "\$(date +%F\ %T) start \$n cmds, $par parallel" | tee -a sched.log
cat > one.sh <<'__ONE__'
i=\$(printf %03d \$1); c=\$(sed -n "\$1p" $JOBS/$tag/cmds.txt)
( cd $S3 && nice -n 5 bash -c "\$c" ) > $JOBS/$tag/\$i.log 2>&1; rc=\$?
echo \$rc > $JOBS/$tag/\$i.rc; echo "\$(date +%T) \$i rc \$rc \$c" >> $JOBS/$tag/sched.log
__ONE__
seq 1 \$n | xargs -P $par -n 1 bash one.sh || true
echo "\$(date +%F\ %T) end; failed: \$(grep -L '^0\$' $JOBS/$tag/*.rc | wc -l)" | tee -a sched.log
EOF
  } | rsh
}

status() {
  rsh <<EOF
D=$JOBS/$1; cd \$D || exit 1
echo "cmds \$(wc -l < cmds.txt)  done \$(ls *.rc 2>/dev/null | wc -l)  failed \$(grep -L '^0\$' *.rc 2>/dev/null | wc -l)"
tail -n 5 sched.log
EOF
}

pull() {
  local tag=$1; shift
  local t0=$SECONDS
  { echo "cd $S3 && find $* -type f -newer $JOBS/$tag/started -print0 | tar --null -T - -cf - | gzip -1"; } \
    | "$SSH" -o BatchMode=yes -o LogLevel=ERROR "$HOST" "wsl -d Ubuntu-22.04 -u system -- bash -s" \
    | (cd "$S3" && tar xzvf - | wc -l | sed 's/^/files pulled: /')
  echo "pull done $((SECONDS - t0)) s"
}

case "${1:-}" in
  push) push ;;
  run) shift; run "$@" ;;
  status) status "$2" ;;
  pull) shift; pull "$@" ;;
  sh) echo "cd $S3 && $2" | rsh ;;
  *) sed -n '2,17p' "$0"; exit 1 ;;
esac
