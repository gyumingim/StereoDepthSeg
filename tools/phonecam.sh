#!/usr/bin/env bash
# phonecam.sh — S24 카메라를 /dev/video9 가상 웹캠으로 띄운다.
#
#   ./tools/phonecam.sh list        연결 확인 + 사용 가능한 카메라 목록
#   ./tools/phonecam.sh run [ID]    카메라를 /dev/video9 로 내보낸다 (기본 ID 0)
#
# 사전 조건
#   1) 폰: 설정 > 휴대전화 정보 > 소프트웨어 정보 > 빌드번호 7번 탭
#          -> 개발자 옵션 > USB 디버깅 ON, 연결 시 "USB 디버깅 허용" 수락
#   2) PC: sudo modprobe v4l2loopback video_nr=9 card_label=PhoneCam
#          ** exclusive_caps=1 을 주면 안 된다 **  주면 장치가 Video Capture/Output
#          능력을 아예 announce 하지 않아(Capabilities 0x85200000) ffmpeg 의
#          ioctl(VIDIOC_G_FMT) 가 Invalid argument 로 실패한다. 빼면 0x85200003.
#
# 왜 초광각(보통 camera-id 2)을 권하는가
#   scrcpy 에는 초점 고정 옵션이 없다. 메인 카메라는 오토포커스라 초점이 바뀌면
#   fx 가 같이 바뀌는데, 캘리브레이션은 내부 파라미터가 고정이라고 전제한다.
#   갤럭시의 초광각은 보통 고정초점이라 이 문제가 없다.
#   -> list 로 확인한 뒤 초광각 ID 를 골라 쓸 것. (실제 고정초점인지는
#      서로 다른 거리에서 두 번 캘리해 fx 가 같은지 보면 검증된다)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ADB="$HERE/platform-tools/adb"
SCRCPY="$(find "$HERE" -maxdepth 2 -name scrcpy -type f | head -1)"
VIDEO_NR=9
DEV="/dev/video$VIDEO_NR"

die() { echo "  $*" >&2; exit 1; }

[ -x "$ADB" ]    || die "adb 없음: $ADB"
[ -x "$SCRCPY" ] || die "scrcpy 없음"

check_device() {
  local n
  n=$("$ADB" devices | tail -n +2 | grep -cw device)
  if [ "$n" -eq 0 ]; then
    echo "  폰이 안 잡힌다. 확인할 것:"
    echo "    - USB 디버깅이 켜져 있는가"
    echo "    - 폰 화면의 'USB 디버깅을 허용하시겠습니까?' 를 수락했는가"
    echo "    - 케이블이 데이터 전송용인가 (충전 전용 케이블은 안 됨)"
    "$ADB" devices | tail -n +2 | grep -v '^$'
    exit 1
  fi
}

case "${1:-list}" in
  list)
    check_device
    echo "  연결됨:"; "$ADB" devices -l | tail -n +2 | grep -v '^$'
    echo
    echo "  사용 가능한 카메라:"
    "$SCRCPY" --list-cameras 2>&1 | grep -E "^\s+--camera-id|fps" || true
    ;;
  run)
    check_device
    ID="${2:-0}"
    if [ ! -e "$DEV" ]; then
      echo "  $DEV 이 없다. 먼저 아래를 실행할 것 (sudo 필요, 한 번만):"
      echo
      echo "    sudo modprobe v4l2loopback video_nr=$VIDEO_NR card_label=PhoneCam"
      echo
      echo "  exclusive_caps=1 은 넣지 말 것 — 넣으면 ffmpeg 가 헤더를 못 쓴다."
      exit 1
    fi
    echo "  카메라 $ID -> $DEV  (중지: Ctrl+C)"
    # 삼성 + Android 14+ 에서 scrcpy 카메라가 "Camera disconnected" 로 불시에 끊기는
    # 미해결 이슈가 있다 (scrcpy #4865, #5977). 확인된 우회책은 재시작뿐이라
    # 끊기면 자동으로 다시 붙인다. /dev/video9 는 v4l2loopback 이 유지하므로
    # 읽는 쪽(OpenCV)은 잠깐 멈췄다 이어진다.
    # 화면이 꺼지면 백그라운드 카메라 접근이 회수되는 것으로 보여 화면을 켜둔다.
    # ── 끊김의 진짜 원인과 대책 (dumpsys media.camera 로 직접 확인) ──────────
    # HAL 이 동시에 열 수 있는 조합은 {0 1} {0 3} 뿐 → 초광각(2)은 누구와도 공존 불가.
    # 삼성 서비스 둘이 전면 카메라를 잠깐씩 여는데, 그 순간 우리(카메라 2)가 EVICT 된다:
    #   smartface = Smart Stay (intelligent_sleep_mode)      13초 주기
    #   sead      = 환경 적응형 디스플레이 (ead_enabled)      우리가 카메라를 여는 순간 반응
    # 두 설정을 끄면 90초/60초 실험에서 무중단, sead 이벤트 0건. pm disable 은 효과 없음.
    # 되돌리기: settings put system intelligent_sleep_mode 1 / ead_enabled 1
    "$ADB" shell settings put system intelligent_sleep_mode 0 >/dev/null 2>&1
    "$ADB" shell settings put system ead_enabled 0 >/dev/null 2>&1
    "$ADB" shell svc power stayon usb >/dev/null 2>&1
    "$ADB" shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1
    trap 'echo; echo "  중지"; exit 0' INT TERM
    n=0
    while true; do
      "$SCRCPY" --video-source=camera --camera-id="$ID" \
           --camera-size=1280x720 --camera-fps=30 \
           --v4l2-sink="$DEV" --no-playback --no-audio
      n=$((n+1))
      echo "  scrcpy 종료 (${n}회째) — 3초 후 재시작. 중지: Ctrl+C"
      sleep 3
      while ! "$ADB" devices | tail -n +2 | grep -qw device; do
        echo "  폰 대기 중... (USB 디버깅 허용 여부 확인)"; sleep 3
      done
    done
    ;;
  *) die "사용법: $0 [list|run [카메라ID]]" ;;
esac
