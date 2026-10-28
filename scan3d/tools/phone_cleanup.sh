#!/usr/bin/env bash
# phone_cleanup.sh (appfix 2026-10-29): SAFE phone storage cleanup. Default = dry run (table + sizes, nothing deleted).
#   bash scan3d/tools/phone_cleanup.sh [-s SERIAL]                       list candidates
#   bash scan3d/tools/phone_cleanup.sh [-s SERIAL] --delete NAME...      delete these (basename or full phone path)
#   bash scan3d/tools/phone_cleanup.sh [-s SERIAL] --all-safe            delete every SAFE recording
#   bash scan3d/tools/phone_cleanup.sh [-s SERIAL] --all-scratch         delete every SCRATCH item (test dirs, replay copies)
# Env: ADB (default adb; Git Bash: adb.exe is fine, MSYS path conversion is switched off here), PC_REC (PC copies,
#      default ~/scanner/StereoDepthSeg/scan3d/recordings), ANDROID_SERIAL (= -s).
# Candidates:
#   REC     recordings: arlat files/hs_* files/calcap* files/ov/run_*, com.camera.dualstream files/rec_*
#           SAFE only if the phone file list is NON-EMPTY and EVERY phone file (relative name + size in bytes) exists in
#           $PC_REC/<name> with the same size (extra PC files are fine). Else KEEP (reason shown).
#   SCRATCH our test dirs/files in /data/local/tmp (name allowlist below) and arlat files/replay/* (stream copies pushed
#           from the PC). Not compared; deleted only by name or --all-scratch.
#   Everything else (configs files/ov/<cfg>, NPU models files/npu, other apps' files, unknown /data/local/tmp names) is never
#   listed as a candidate and never deleted.
# Any adb error (device offline / unauthorized / several devices / no end marker / unexpected line) ABORTS the whole run
# with exit 3 before anything (more) is deleted: an unreadable phone can never look "identical" (a previous ad-hoc script
# did that). Each delete re-lists and re-checks that one candidate right before its rm, then checks it is gone.
# Exit: 0 ok, 2 usage / refused target, 3 adb error (aborted).
set -u
export MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*'
ADB=${ADB:-adb}
PC_REC=${PC_REC:-$HOME/scanner/StereoDepthSeg/scan3d/recordings}
SER=${ANDROID_SERIAL:-}
MODE=dry; TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -s) SER=$2; shift 2 ;;
    --delete) MODE=names; shift; while [ $# -gt 0 ] && [ "${1#--}" = "$1" ] && [ "$1" != -s ]; do TARGETS+=("$1"); shift; done ;;
    --all-safe) MODE=allsafe; shift ;;
    --all-scratch) MODE=allscratch; shift ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
[ "$MODE" = names ] && [ ${#TARGETS[@]} -eq 0 ] && { echo "--delete needs at least one NAME" >&2; exit 2; }

E0=/storage/emulated/0/Android/data
ARL=$E0/com.scan3d.arlat/files
DUAL=$E0/com.camera.dualstream/files
TMP=/data/local/tmp
ROOTS="$TMP $ARL $ARL/ov $ARL/replay $DUAL"
TMP_OURS='^(ov[a-z0-9_]*|integ[0-9]*|imuprio|idlespin|timing[0-9]+|kb[0-9]+|replay|npu[a-z0-9_]*|xf[a-z0-9_]*|lg[a-z0-9_]*|imulog[A-Za-z0-9_.]*|imu_[A-Za-z0-9_.]*|mag\.csv|pf_th\.txt)$'
SAFE_CHARS='^[A-Za-z0-9_./-]+$'

ADBS=("$ADB"); [ -n "$SER" ] && ADBS+=(-s "$SER")
die() { echo "ABORT (adb): $*" >&2; echo "nothing (more) deleted" >&2; exit 3; }

# one adb shell call; stdout = lines, must end with __END_OK__ (removed); any adb failure -> abort
ash() {
  local out rc
  out=$("${ADBS[@]}" shell "$1; echo __END_OK__" 2>&1); rc=$?
  out=${out//$'\r'/}
  [ $rc -eq 0 ] || die "adb shell rc $rc: $(echo "$out" | head -3)"
  [ "$(printf '%s\n' "$out" | tail -1)" = __END_OK__ ] || die "no end marker (output: $(echo "$out" | head -3 | tr '\n' ' '))"
  printf '%s\n' "$out" | sed '$d'
}

check_device() {
  local st rc
  st=$("${ADBS[@]}" get-state 2>&1); rc=$?
  st=${st//$'\r'/}
  [ $rc -eq 0 ] && [ "$st" = device ] || die "get-state rc $rc: $st"
}

# remote listing: "D|path" per entry, "F|size|path" per file, ERR on find/stat failure
# $1 = shell words for the entries to list (globs expanded on the phone)
remote_list() {
  ash 'for e in '"$1"'; do [ -e "$e" ] || continue; echo "D|$e"; if [ -d "$e" ]; then find "$e" -type f -exec stat -c "F|%s|%n" {} + || echo "ERR|find $e"; else stat -c "F|%s|%n" "$e" || echo "ERR|stat $e"; fi; done'
}

category() {   # path -> REC | SCRATCH | "" (not a candidate)
  local p=$1 b=${1##*/} d=${1%/*}
  case "$d" in
    "$TMP") [[ "$b" =~ $TMP_OURS ]] && echo SCRATCH ;;
    "$ARL") case "$b" in hs_*|calcap*) echo REC ;; esac ;;
    "$ARL/ov") case "$b" in run_*) echo REC ;; esac ;;
    "$ARL/replay") echo SCRATCH ;;
    "$DUAL") case "$b" in rec_*) echo REC ;; esac ;;
  esac
}

declare -A CAT NF BYTES STATUS WHY FILES
ORDER=()
parse_listing() {   # stdin = remote_list output -> fills the arrays (only candidates)
  local line kind rest size path cur=""
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    kind=${line%%|*}; rest=${line#*|}
    case "$kind" in
      D) cur=""; local c; c=$(category "$rest")
         if [ -n "$c" ]; then
           [[ "$rest" =~ $SAFE_CHARS ]] && [[ "$rest" != *..* ]] || die "unexpected characters in phone path $rest"
           cur=$rest; CAT[$cur]=$c; NF[$cur]=0; BYTES[$cur]=0; FILES[$cur]=""; ORDER+=("$cur")
         fi ;;
      F) size=${rest%%|*}; path=${rest#*|}
         [[ "$size" =~ ^[0-9]+$ ]] || die "bad size line: $line"
         [ -z "$cur" ] && continue
         case "$path" in "$cur"|"$cur"/*) ;; *) die "file $path outside $cur" ;; esac
         NF[$cur]=$(( ${NF[$cur]} + 1 )); BYTES[$cur]=$(( ${BYTES[$cur]} + size ))
         FILES[$cur]+="${size}|${path#"$cur"}"$'\n' ;;
      ERR) die "phone listing error: $rest" ;;
      *) die "unexpected adb output line: $line" ;;
    esac
  done
}

judge() {   # $1 = candidate path -> STATUS / WHY
  local p=$1 n name pc l size rel bad=0 first=""
  name=${p##*/}
  if [ "${CAT[$p]}" = SCRATCH ]; then STATUS[$p]=SCRATCH; WHY[$p]="test/replay copy (not compared)"; return; fi
  pc=$PC_REC/$name
  if [ "${NF[$p]}" -eq 0 ]; then STATUS[$p]=KEEP; WHY[$p]="phone file list empty"; return; fi
  if [ ! -d "$pc" ]; then STATUS[$p]=KEEP; WHY[$p]="no PC copy $pc"; return; fi
  declare -A pcs=()
  while IFS='|' read -r size rel; do pcs["/$rel"]=$size; done < <(find "$pc" -type f -printf '%s|%P\n')
  while IFS= read -r l; do
    [ -z "$l" ] && continue
    size=${l%%|*}; rel=${l#*|}
    if [ "${pcs[$rel]-MISSING}" != "$size" ]; then
      bad=$((bad + 1)); [ -z "$first" ] && first="$rel phone $size PC ${pcs[$rel]-missing}"
    fi
  done <<< "${FILES[$p]}"
  if [ $bad -eq 0 ]; then STATUS[$p]=SAFE; WHY[$p]="all ${NF[$p]} files match $pc"
  else STATUS[$p]=KEEP; WHY[$p]="$bad of ${NF[$p]} files differ (first: $first)"; fi
}

mb() { awk -v b="$1" 'BEGIN { printf "%.1f", b / 1048576 }'; }

# ---------- scan
check_device
# 1) entry names only (other apps' / unknown dirs are never walked), 2) file lists of the candidates, 30 per adb call
ENT=$(ash 'for e in '"$TMP/* $ARL/* $ARL/ov/* $ARL/replay/* $DUAL/*"'; do [ -e "$e" ] && echo "E|$e"; done') || exit 3
CANDS=()
while IFS= read -r line; do
  [ -z "$line" ] && continue
  [ "${line%%|*}" = E ] || die "unexpected adb output line: $line"
  e=${line#E|}
  [ -n "$(category "$e")" ] || continue
  [[ "$e" =~ $SAFE_CHARS ]] && [[ "$e" != *..* ]] || die "unexpected characters in phone path $e"
  CANDS+=("'$e'")
done <<< "$ENT"
for ((i = 0; i < ${#CANDS[@]}; i += 30)); do
  LIST=$(remote_list "${CANDS[*]:i:30}") || exit 3
  parse_listing <<< "$LIST"
done
for p in "${ORDER[@]}"; do judge "$p"; done

echo "phone_cleanup $(date '+%Y-%m-%d %H:%M:%S') device ${SER:-default} PC copies $PC_REC mode $MODE"
printf '%-8s %-8s %9s %6s  %s  -- %s\n' STATUS CAT MB FILES PATH WHY
tot_safe=0; tot_scr=0; tot_keep=0
for p in "${ORDER[@]}"; do
  printf '%-8s %-8s %9s %6s  %s  -- %s\n' "${STATUS[$p]}" "${CAT[$p]}" "$(mb "${BYTES[$p]}")" "${NF[$p]}" "$p" "${WHY[$p]}"
  case "${STATUS[$p]}" in SAFE) tot_safe=$((tot_safe + BYTES[$p])) ;; SCRATCH) tot_scr=$((tot_scr + BYTES[$p])) ;; *) tot_keep=$((tot_keep + BYTES[$p])) ;; esac
done
echo "total: SAFE $(mb $tot_safe) MB, SCRATCH $(mb $tot_scr) MB, KEEP $(mb $tot_keep) MB (${#ORDER[@]} candidates)"
[ "$MODE" = dry ] && { echo "dry run: nothing deleted (use --delete NAME... | --all-safe | --all-scratch)"; exit 0; }

# ---------- pick targets (all checked before the first rm)
DEL=()
case "$MODE" in
  allsafe) for p in "${ORDER[@]}"; do [ "${STATUS[$p]}" = SAFE ] && DEL+=("$p"); done ;;
  allscratch) for p in "${ORDER[@]}"; do [ "${STATUS[$p]}" = SCRATCH ] && DEL+=("$p"); done ;;
  names)
    for t in "${TARGETS[@]}"; do
      hit=""
      for p in "${ORDER[@]}"; do [ "$p" = "$t" ] || [ "${p##*/}" = "$t" ] && hit="$hit $p"; done
      hit=${hit# }
      [ -z "$hit" ] && { echo "REFUSED: $t is not a candidate" >&2; exit 2; }
      [ "$hit" != "${hit% *}" ] && { echo "REFUSED: $t matches several paths ($hit), give the full path" >&2; exit 2; }
      case "${STATUS[$hit]}" in SAFE|SCRATCH) DEL+=("$hit") ;; *) echo "REFUSED: $hit is ${STATUS[$hit]} (${WHY[$hit]})" >&2; exit 2 ;; esac
    done ;;
esac
[ ${#DEL[@]} -eq 0 ] && { echo "nothing to delete"; exit 0; }

# ---------- delete: re-list + re-judge each one right before its rm
n_del=0; b_del=0
for p in "${DEL[@]}"; do
  want=${STATUS[$p]}
  unset "CAT[$p]" "NF[$p]" "BYTES[$p]" "FILES[$p]" "STATUS[$p]" "WHY[$p]"; ORDER=()
  check_device
  L1=$(remote_list "'$p'") || exit 3
  parse_listing <<< "$L1"
  if [ -z "${CAT[$p]-}" ]; then echo "skip $p: gone or no longer a candidate"; continue; fi
  judge "$p"
  if [ "${STATUS[$p]}" != "$want" ]; then echo "skip $p: re-check says ${STATUS[$p]} (${WHY[$p]})"; continue; fi
  ash "rm -rf -- '$p'" > /dev/null || exit 3
  gone=$(ash "[ -e '$p' ] && echo STILL || echo GONE") || exit 3
  [ "$gone" = GONE ] || die "rm did not remove $p ($gone)"
  n_del=$((n_del + 1)); b_del=$((b_del + BYTES[$p]))
  echo "deleted $p ($(mb "${BYTES[$p]}") MB, ${NF[$p]} files, $want)"
done
echo "deleted $n_del item(s), $(mb $b_del) MB"
