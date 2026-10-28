#!/usr/bin/env bash
# fake adb for phone_cleanup tests. FAKE_ROOT = fake phone fs, FAKE_MODE = ok | offline | empty | offline_after:N | noauth
# offline_after:N -> the first N shell calls work, then "error: device offline"
[ "$1" = -s ] && shift 2
cnt=$FAKE_ROOT/.calls; n=$(cat "$cnt" 2>/dev/null || echo 0)
off() { echo "error: device offline" >&2; exit 1; }
case "$FAKE_MODE" in
  offline) off ;;
  noauth) echo "error: device unauthorized." >&2; exit 1 ;;
  empty) exit 0 ;;   # adb that says nothing and returns 0 (must NOT look like "identical")
esac
if [ "$1" = get-state ]; then echo device; exit 0; fi
[ "$1" = shell ] || { echo "fake: $*" >&2; exit 1; }
shift
echo $((n + 1)) > "$cnt"
case "$FAKE_MODE" in offline_after:*) [ "$n" -ge "${FAKE_MODE#offline_after:}" ] && off ;; esac
[ -n "${FAKE_MUTATE_AT:-}" ] && [ $((n + 1)) = "$FAKE_MUTATE_AT" ] && eval "$FAKE_MUTATE"   # phone changes before this call
cmd="$*"
cmd=${cmd//\/storage\/emulated\/0/$FAKE_ROOT/storage/emulated/0}
cmd=${cmd//\/data\/local\/tmp/$FAKE_ROOT/data/local/tmp}
bash -c "$cmd" 2>&1 | sed "s#$FAKE_ROOT##g"
