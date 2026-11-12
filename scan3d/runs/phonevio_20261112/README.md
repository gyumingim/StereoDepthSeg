# phonevio_20261112 — 폰에서 VIO (녹화 끝난 뒤, 프레임 안 버림)

상태 (11-12 13:40): **앱 코드만 됨, 폰 실행 안 함** — 폰 무선 디버깅이 꺼져 adb 연결 실패 (ping 은 됨, 39097 응답 없음, mDNS 없음).

## 만든 것 (앱 0.6.40, vc 54, 브랜치 phonevio-20261112)
- `phone_stereo/arlat/src/com/scan3d/arlat/ReplayVio.java`, `MainActivity` mode `replay`.
  `adb shell am start -n com.scan3d.arlat/.MainActivity --es mode replay --es rec hs_20261112_121801_std60` (선택: `--es cfg c4f60`, `--ef max_battery_c 42`)
- PC `lp_replay.py` (run_scanner vio 단계) 를 그대로 옮김: 같은 브리지 인자 (`--profile rec60 --drop-late 0 --loop 1 --loop-sync 1 ...`),
  같은 프레임 선택 (pick_by_time), 같은 IMU (35/16 v0..2, acc 를 gyro 시각에 선형 보간, 프레임 +5 ms 까지), 카메라 안 엶.
- 영상: MediaCodec 으로 cam5.mp4 디코드 → Y 평면 → LUT round((Y-16)·255/219). PC 는 OpenCV BGR→GRAY.
  121801 3 프레임 PC 확인: 차이 평균 1.4 계조 (색이 진한 곳에서 최대 14) — 입력 차이로 남음.
- 결과: `REC/phonevio_<시각>/` poses.jsonl.gz (브리지 pose 줄 그대로), loop.jsonl, bridge.err, temps.csv (2 초마다 배터리·발열), run.json.
  배터리 42 °C 면 멈춤.
- 브리지: 이 워크트리 소스 (= PC build_livepose 브리지 소스 scanplan f6b368e, ov_bridge 폴더 diff 없음) 로 새로 빌드
  (`BUILD_DIR=~/scanner/phonevio_work/ovb_android bash scan3d/ov_bridge/android/build_android.sh bridge`), strip 뒤 sha c8293ded.
  0.6.39 APK 안 브리지 (d5c0e585) 는 어느 빌드인지 기록 없음.
- 폰 files/ov/c4f60 yaml 3 개·remap·voc sha = PC cfg_gen/c4f60 와 같음 (확인함).

## 비교 (폰 실행 뒤)
`python cmp_phonevio.py REC 폰결과폴더 PC접두어 out.json` — PC 기준 = `runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0` (fixes 워크트리).
SE3 정렬 ATE, Sim3 축척비, 회전 차이, init 시각, 루프, 자세 수, 시간·온도.

## 다시 시작
1. 폰 무선 디버깅 켜기 (사용자), `adb connect 192.168.1.31:<포트>`; 앞 앱이 com.scan3d.arlat 이면 기다림.
2. `adb install -r ~/scanner/phonevio_work/apk/arlat.apk` (WSL 경로; 다시 빌드: build.sh, OVB=위 브리지, B=~/scanner/phonevio_work/apk)
3. 위 am start, 끝나면 `am force-stop com.scan3d.arlat`, 결과 폴더 pull, cmp_phonevio.py.
