#!/usr/bin/env bash
# phone_cleanup.sh tests with the fake adb (2026-10-29). Prints PASS/FAIL per case; exit 1 on any FAIL.
set -u
H=$(cd "$(dirname "$0")" && pwd)
T=${1:?path to phone_cleanup.sh}
W=$(mktemp -d); [ -n "${KEEP_W:-}" ] && echo "work dir $W"
export ADB=$H/fake_adb.sh FAKE_ROOT=$W/phone PC_REC=$W/pc
fails=0
ok() { echo "PASS $1"; }
bad() { echo "FAIL $1"; fails=$((fails + 1)); }
mkfs() {   # fresh fake phone + PC
  rm -rf "$W/phone" "$W/pc"; mkdir -p "$W/pc"
  A=$W/phone/storage/emulated/0/Android/data/com.scan3d.arlat/files; D=$W/phone/storage/emulated/0/Android/data/com.camera.dualstream/files
  mkdir -p "$A/ov/c4f60" "$A/replay" "$A/npu" "$D" "$W/phone/data/local/tmp"
  # match: same names + sizes (PC has one extra file)
  mkdir -p "$A/hs_20261020_100000_std60/sub" "$PC_REC/hs_20261020_100000_std60/sub"
  head -c 1000 /dev/zero > "$A/hs_20261020_100000_std60/a.mp4"; head -c 1000 /dev/zero > "$PC_REC/hs_20261020_100000_std60/a.mp4"
  head -c 10 /dev/zero > "$A/hs_20261020_100000_std60/sub/imu.csv"; head -c 10 /dev/zero > "$PC_REC/hs_20261020_100000_std60/sub/imu.csv"
  echo x > "$PC_REC/hs_20261020_100000_std60/extra_pc_only.txt"
  # size mismatch
  mkdir -p "$D/rec_20261016_153012" "$PC_REC/rec_20261016_153012"
  head -c 500 /dev/zero > "$D/rec_20261016_153012/v.mp4"; head -c 499 /dev/zero > "$PC_REC/rec_20261016_153012/v.mp4"
  # missing file on PC
  mkdir -p "$A/hs_20261021_100000_std60" "$PC_REC/hs_20261021_100000_std60"
  head -c 5 /dev/zero > "$A/hs_20261021_100000_std60/a.mp4"; head -c 5 /dev/zero > "$A/hs_20261021_100000_std60/b.csv"
  head -c 5 /dev/zero > "$PC_REC/hs_20261021_100000_std60/a.mp4"
  # empty phone dir (PC copy exists and is empty too -> must NOT be safe)
  mkdir -p "$A/calcap_20261025_100000" "$PC_REC/calcap_20261025_100000"
  # no PC copy
  mkdir -p "$A/ov/run_20261028_201451_main60_c4f60"; echo hi > "$A/ov/run_20261028_201451_main60_c4f60/meta.json"
  # scratch + never-candidates
  mkdir -p "$W/phone/data/local/tmp/ovopt" "$W/phone/data/local/tmp/frida"; echo 1 > "$W/phone/data/local/tmp/ovopt/x"; echo 1 > "$W/phone/data/local/tmp/frida/y"
  echo s > "$A/replay/hs_x.bin"; echo m > "$A/npu/x960p1mc.nnc"; echo c > "$A/ov/c4f60/estimator_config.yaml"
  rm -f "$W/phone/.calls"
}
st() { grep -E "^[A-Z]+ +[A-Z]+ .* $1  --" "$W/out" | awk '{print $1}'; }
P=/storage/emulated/0/Android/data/com.scan3d.arlat/files
Q=/storage/emulated/0/Android/data/com.camera.dualstream/files

mkfs; FAKE_MODE=ok bash "$T" > "$W/out" 2>&1; rc=$?
[ $rc = 0 ] && [ "$(st $P/hs_20261020_100000_std60)" = SAFE ] && ok "match -> SAFE" || { bad "match -> SAFE (rc $rc)"; cat "$W/out"; }
[ "$(st $Q/rec_20261016_153012)" = KEEP ] && ok "size mismatch -> KEEP" || bad "size mismatch -> KEEP"
[ "$(st $P/hs_20261021_100000_std60)" = KEEP ] && ok "file missing on PC -> KEEP" || bad "missing -> KEEP"
[ "$(st $P/calcap_20261025_100000)" = KEEP ] && grep -q "phone file list empty" "$W/out" && ok "empty phone list -> KEEP" || bad "empty list -> KEEP"
[ "$(st $P/ov/run_20261028_201451_main60_c4f60)" = KEEP ] && ok "no PC copy -> KEEP" || bad "no PC copy -> KEEP"
[ "$(st /data/local/tmp/ovopt)" = SCRATCH ] && [ "$(st $P/replay/hs_x.bin)" = SCRATCH ] && ok "test dir + replay -> SCRATCH" || bad "scratch"
! grep -q "frida\|npu/x960\|c4f60/estimator" "$W/out" && ok "foreign / config / model never listed" || bad "foreign listed"
[ -d "$P" ] ; [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "dry run deletes nothing" || bad "dry run deleted"

mkfs; FAKE_MODE=offline bash "$T" > "$W/out" 2>&1; rc=$?
[ $rc = 3 ] && grep -q "ABORT" "$W/out" && ! grep -q SAFE "$W/out" && ok "offline -> abort rc 3, no SAFE" || { bad "offline (rc $rc)"; cat "$W/out"; }
mkfs; FAKE_MODE=noauth bash "$T" --all-safe > "$W/out" 2>&1; rc=$?
[ $rc = 3 ] && [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "unauthorized -> abort, nothing deleted" || bad "noauth (rc $rc)"
mkfs; FAKE_MODE=empty bash "$T" --all-safe > "$W/out" 2>&1; rc=$?
[ $rc = 3 ] && [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "silent adb (rc 0, no output) -> abort, nothing deleted" || { bad "empty adb (rc $rc)"; cat "$W/out"; }

mkfs; FAKE_MODE=ok bash "$T" --all-safe > "$W/out" 2>&1; rc=$?
[ $rc = 0 ] && [ ! -e "$W/phone$P/hs_20261020_100000_std60" ] && [ -e "$W/phone$Q/rec_20261016_153012/v.mp4" ] && [ -e "$W/phone$P/hs_20261021_100000_std60" ] \
  && [ -e "$W/phone$P/calcap_20261025_100000" ] && [ -e "$W/phone/data/local/tmp/ovopt" ] && [ -e "$PC_REC/hs_20261020_100000_std60/a.mp4" ] \
  && ok "--all-safe deletes only the SAFE one (PC copy untouched)" || { bad "--all-safe (rc $rc)"; cat "$W/out"; }

mkfs; FAKE_MODE=ok bash "$T" --delete rec_20261016_153012 > "$W/out" 2>&1; rc=$?
[ $rc = 2 ] && grep -q REFUSED "$W/out" && [ -e "$W/phone$Q/rec_20261016_153012/v.mp4" ] && ok "--delete of a KEEP item refused" || bad "--delete KEEP (rc $rc)"
mkfs; FAKE_MODE=ok bash "$T" --delete hs_20261020_100000_std60 rec_20261016_153012 > "$W/out" 2>&1; rc=$?
[ $rc = 2 ] && [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "one bad name -> nothing deleted" || bad "mixed names (rc $rc)"
mkfs; FAKE_MODE=ok bash "$T" --delete ovopt hs_x.bin > "$W/out" 2>&1; rc=$?
[ $rc = 0 ] && [ ! -e "$W/phone/data/local/tmp/ovopt" ] && [ ! -e "$W/phone$P/replay/hs_x.bin" ] && [ -e "$W/phone/data/local/tmp/frida/y" ] && ok "--delete scratch by name" || { bad "--delete scratch (rc $rc)"; cat "$W/out"; }

# phone changes between the scan and the rm (before shell call 3 = the re-list): the re-check must skip it
mkfs; FAKE_MODE=ok FAKE_MUTATE_AT=3 FAKE_MUTATE="head -c 1001 /dev/zero > '$W/phone$P/hs_20261020_100000_std60/a.mp4'" bash "$T" --all-safe > "$W/out" 2>&1; rc=$?
[ $rc = 0 ] && grep -q "re-check says KEEP" "$W/out" && [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "changed after scan -> re-check skips, not deleted" || { bad "changed (rc $rc)"; cat "$W/out"; }
# offline right after the scan (scan = 1 entries call + 1 list call): re-check must abort, nothing deleted
mkfs; FAKE_MODE=offline_after:2 bash "$T" --all-safe > "$W/out" 2>&1; rc=$?
[ $rc = 3 ] && [ -e "$W/phone$P/hs_20261020_100000_std60/a.mp4" ] && ok "offline before the re-check -> abort, nothing deleted" || { bad "offline_after (rc $rc)"; cat "$W/out"; }
[ -n "${KEEP_W:-}" ] || rm -rf "$W"
[ $fails = 0 ] && echo "ALL PASS" || { echo "$fails FAIL"; exit 1; }
