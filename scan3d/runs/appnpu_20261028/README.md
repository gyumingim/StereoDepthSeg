# appnpu 2026-10-28 — 앱 0.6.32: XFeat + LighterGlue (S24 NPU) 루프 클로저 경로 (기본 꺼짐, 대체 경로 없음)

브랜치 `appnpu-20261028` (WSL `~/scanner/StereoDepthSeg-appnpu`) = 폰에 깔린 0.6.29 (`lens05-20261027` 9ef84a0) 위. 계획 = `PLAN.md` (코딩 전 커밋 4432f68).
**폰에 설치 안 함, adb 안 씀 (폰 연결 없음).** 폰 시간·정확도는 전부 미측정 → 아래 숫자 중 폰 값은 모두 **추정**.

## 결론
- APK `scratchpad\team\appnpu\apk\arlat_0.6.32_appnpu.apk` (vc 38, `0.6.32-appnpu-20261028`, sha ad4cd962…, 같은 개발 인증서 CN=PhoneStereo Development;
  libov_bridge.so 7642d942…, libxfnpu.so 1c3c286f…, 코드 = 이 README 를 넣은 커밋). `SHA256SUMS.txt` 같은 폴더.
  롤백 = `scratchpad\team\appnpu\apk\arlat_0.6.29_rollback.apk` (= lens05 사본, sha 5feb3e53…, vc 35). 0.6.30 (5507a270) 은 대체 경로가 있어 지웠다.
- intent `npu_loop 1` 일 때만 켜짐. 안 주면 0.6.29 와 같은 동작 (다리 인자 같음, libxfnpu.so 안 읽음). 단 APK 안에서 다른 것:
  **libov_bridge.so 새 빌드** (코드 추가, 꺼짐 경로 PC 동일 — 아래), **libarloop.so** sha 3a271cec ≠ 0.6.29 49e390a2 (소스 diff 없음, 같은 크기,
  20 바이트만 다르고 문자열 차이 없음 → 빌드 ID 차이로 추정), **매니페스트 `uses-native-library libenn_public_api_cpp.so required=false`** (항상 들어감:
  로드는 npu_loop 1 일 때만이지만 앱 링커 네임스페이스 허용 목록이 바뀜), assets/npu/ 모델 4 개 (약 9 MB, 꺼짐이면 복사 안 함), meta app_version 문자열.
- **사용자 규칙 (10-28): NPU 를 못 쓰면 돌리지 않는다. BRIEF/CPU 대체 경로 없음.**
  - 시작: ENN 열기·모델 열기·버퍼·워밍업·자산 복사 중 하나라도 실패, 또는 시간 초과 (자바 15 s / 네이티브 10 s), 또는 loop 0 →
    **실행 거부**: 다리·카메라 안 켬, run 폴더엔 meta.json ("npu_loop":"refused: 이유", "run_valid":false) + npu_start.txt 만, logcat `NPU LOOP REFUSED: …`, 앱 finish().
    같은 프로세스에서 이전 서버가 ENN 안에서 멈춰 남아 있으면 다음 시작도 거부 ("previous NPU server stuck; restart the app").
  - 다리가 서버에 연결 못 함 → 다리 시작 실패 (CMD 모드와 같게 치명).
  - 실행 중: 서버 오류는 **명시적 오류 코드**로 답함 (개수 자리에 −1 NPU 실행 실패, −2 영상 크기, −3 키프레임 캐시 없음/버려짐, −4 잘못된 mode; "0 개" 로 보내지 않음) 뒤 연결을 닫음.
    다리는 음수 코드·끊김·3 s 무응답을 모두 "서버 잃음" 으로 보고 **루프 클로저를 그 실행 동안 멈춤** (BRIEF 안 씀, 새 루프 없음; 지도 보정은 마지막 값 유지).
    **VIO 는 계속** (안전). loop.jsonl 에 `{"xf_down":이유,"errno":..,"t_wall_ms":..}`; 앱은 종료 때 이것 또는 서버 실패 기록을 보고
    meta.json `"run_valid": false`, `"npu_failed": {"bridge": xf_down 줄, "server": 이유 + 시각}` 를 쓴다. 문제 없으면 `"run_valid": true`.
- 켜면: 앱 프로세스 안 ENN(`libenn_public_api_cpp.so` dlopen)으로 XFeat `x960p1mc`(매 키프레임) + LighterGlue `L6_a16mc`(BoW 후보에만),
  다리 루프 스레드가 추상 유닉스 소켓으로 요청 (VIO 스레드는 키프레임 복사만, 기다리지 않음). 상호 NN·LG 후처리는 CPU 계산 (모델 아님).

## PC 에서 확인한 것 (폰 아님, 녹화 1 개, 각 1 회, `--loop-sync 1`)
| 확인 | 방법 | 결과 |
|---|---|---|
| 꺼짐 동일성 (다리) | lens05 소스 PC 빌드 vs 이 브랜치 최종 PC 빌드, std60 165506 + 폰 c4f60 새 렌즈 폴더, `--profile rec60 --drop-late 0 --loop 1 --loop-sync 1` (`appnpu_replay.py`, `ident_check.py`) | 포즈 1690 줄 · loop.jsonl 59 줄 **같음** (시간 칸 ms/remap_ms 만 뺌) `ident_pc_result.txt`. 폰(비동기 loop-sync 0, Android 바이너리)은 미확인 |
| Android 다리 | lens05 소스 Android 빌드 vs 폰 0.6.29 libov_bridge: 크기 같음(13,678,288 B), /home 경로 문자열(-lens05 vs -integ2) 7 개 다름 | 경로 외 문자열 차이 없음만 확인 (기계어 비교 아님 → "재현" 은 미확인) `android_bridge_repro.txt` |
| 서버 C++ vs 파이썬 기준 | PC 시험 빌드 `xfnpu_pc` (NPU 대신 torch fp32 대역 `npu_emu.py`) + `test_server.py` (같은 작성자; 좌표 정규화·채움 가정 공유. 공식 XFeat 후처리 대비는 독립): 라이브 경로 프레임 2 장, 질의점 150 | 키포인트 512/512 동일, 상호 NN 251/251, LG 175/175 동일, topk 200·333 도 동일. 없는 키프레임 요청 → 오류 코드 −3. fp16 버퍼: 키포인트 집합 99.8 % `test_server_*.json` |
| LG 점 부족 시 채움 | `pad_check.py` PC fp32 (a16 양자화 모델 아님), 5 쌍 × 8 조합 | 반복+복사합 Jaccard 0.94~1.00 → 기본 lg_pad 2 `pad_check.json` |
| 다리+서버 연결 | 같은 녹화, FLGc 문 (max_cand=1), 서버 = PC torch fp32 대역 (`rep_npu.sh`) | A 서버 있음: 검증 12 / 채택 10 — **녹화 1 개 1 회, PC 대역, 같은 재방문 구간에 몰림 (독립 루프 10 개 아님), 채택된 루프가 맞는지(지도 정확도) 모름**. 0.6.29 BRIEF(max_cand 3) 와 비교한 것 아님. B 서버 없음 → 다리 시작 실패 (포즈 0). C 도중 강제 종료 → xf_down, 이후 검증 0 (BRIEF 없음), **A·C VIO 포즈(지도 칸 뺀) = 꺼짐과 같음** `vio_same.txt`, `rep_npu_summary.txt` |
| Java 도우미 | `jt/TNpu.java` (CPU 고르기, 다리 설정 문자열) | 8/8 통과 `jt/run_t.log` (거부·INVALID 경로는 폰 없이 미시험, 코드 읽기만) |
- PC 대역 시간(ms)은 torch CPU 값이라 폰과 무관.

## 0.6.30 → 0.6.32 바뀐 것 (검토자용 목록)
1. OvRunner: npu 시작 실패/시간 초과/loop 0 → `NpuRefused` (meta.json 쓰고 throw, 다리·카메라 전); MainActivity 가 받아 finish(). `npuStartTimed` = 별도 스레드 + 15 s.
2. OvRunner.stop: `npuDownLine(loop.jsonl)` + `NpuLoop.stop()` (= JNI nStop 이 서버 실패 문자열 반환) → `run_valid` / `npu_failed`.
3. xfnpu_core.h: 오류 코드 −1..−4 + 연결 닫기, `fail()` 첫 실패 기록, `failure()`, 시작 handshake `wait_for(10 s)` → 시간 초과 시 detach + `stuck()`; stop 이 3 s 안에 못 끝내도 `stuck`.
4. xfnpu_jni.cpp: stuck 서버는 일부러 해제 안 함 + `g_stuck` → 이후 nStart 거부; nStop 은 실패 문자열 반환.
5. ovloop.cpp/h: unix 연결 실패 = 치명 (BRIEF 대체 삭제); 실행 중 잃음 = 검증 안 함 (why "xf_down", BRIEF 삭제); 음수 코드 해석; xf_down 줄에 t_wall_ms.
6. 버전 vc 38 / 0.6.32, VERSION "ov-20261028-appnpu-0.6.32".

## 실행 안내 (나중에 사용자가)
1. 설치: `adb install -r arlat_0.6.32_appnpu.apk` (같은 인증서 → 데이터·files/ov 유지). 렌즈 세트(files/ov/c4f60 …)는 그대로 씀.
2. 온도 기록(SKIN/AP)을 원하면 (HsRecorder 와 같음): `adb shell pm grant com.scan3d.arlat android.permission.DUMP`. 없으면 npu.jsonl 의 skin/ap = null (배터리·thermal status 는 기록).
3. 실행 (기존 스크립트에 extras 만 추가): `bash scan3d/tools/ov_phone_run.sh run <ip:port> c4f60 <secs> --es profile rec60 --ei npu_loop 1`
   - rec60 이 loop 1 을 켠다 (loop 0 이면 실행 거부).
   - 선택: `--es npu_xf x960f1mc` (post_off 판), `--es npu_lg_in trk768` (마스크 768 모델이 생기면 + `--es npu_lg <이름>`), `--ei npu_lg_pad 0|1|2`,
     `--ei npu_topk 512`, `--es npu_cpus 0-3|none|…` (기본: 다리 cpus 가 있으면 그 여집합, 없으면 0-3), `--ei npu_nice 10`, `--ei npu_perf <ENN perf mode>`,
     `--ei npu_cache_mb 300`, `--es npu_cfg max_cand=2,…` (다리 --loop-cfg 뒤에 붙음), `--ei npu_timeout_ms 3000`.
   - 새 모델: `adb push NAME.nnc /sdcard/Android/data/com.scan3d.arlat/files/npu/` 후 `--es npu_lg NAME` (LG 는 `lg_wr_NAME.npy` 도 같이). 앱이 내부 files/npu/ 로 복사해 ENN 에 경로로 줌.
4. 결과 (run 폴더): `meta.json` "npu_loop" = "on" 또는 "refused: 이유", **"run_valid"** (false 면 "npu_failed" 확인), `npu_start.txt` (ENN 열기·버퍼 크기·워밍업 µs), `npu.jsonl` (줄마다:
   F = prep/npu/post_q/store/tot/recv µs, post 단계 4 개, nk, nq, skin/ap/bat/tstat, 캐시; M = mode 1/2, nA/nB, nv(LG 실제 점 수), 짝 수, pack/npu/post µs),
   `loop.jsonl` (kf 줄 "xf" = [키포인트, 3D 점, 삼각, 서버 ms feat/tri/match], 후보 줄 "xn" = 짝 수, "xf_down" 줄 = 서버 잃음 → 그 뒤 루프 없음, 실행 INVALID),
   `bridge_stderr.log` ("XFeat server … connected", 끝에 "keyframes pushed … dropped …" = 큐 버림).
5. 롤백: `adb install -r -d arlat_0.6.29_rollback.apk` (vc 35). files/npu/ 는 남아도 0.6.29 는 안 읽음.

## 독립 검토자에게: 위험한 곳 (확인 부탁)
1. **ENN 이 앱 프로세스에서 실제로 열리는지** — nncrun 은 instrument(화면 없음) 프로세스에서 성공. 이 앱은 Activity + 카메라 + ARCore 라이브러리가 같이 있는 프로세스. 매니페스트 `uses-native-library` 를 넣음 (nncrun 과 같음). 미검증.
2. **ENN 동시 실행**: 같은 앱 안 다른 NPU 사용자 없음(추정). ENN 호출은 서버 작업 스레드 하나에서만 (init·open·exec·close). EnnExecuteModel 이 오래 막히면 다리는 3 s 뒤 루프를 멈추고 실행 INVALID. 앱 종료 때 서버 스레드가 3 s 안에 안 끝나면 detach + 객체를 일부러 해제하지 않음
   (`xfnpu_core.h` stop(3000), `xfnpu_jni.cpp` nStop) → ENN 정리 안 됨 (프로세스 종료 때 OS 가 회수, 추정).
3. **NumberOfBuffersInfo/EnnBuffer 구조**를 nncrun 과 같은 방어적 읽기로 가정 (in 1 / out 1 아니면 끔). x960p1mc 버퍼가 fp32 인지 fp16 인지 모름 → 크기로 판별 (둘 다 처리, fp16 은 PC 대역으로만 시험).
4. **XFeat 입력 정규화**: 영상 전체(544 줄, 복제 4 줄 포함) 평균·분산 — npuopt10 `xf960_eval.py run960` 과 같다고 봄. 모델이 안에서 다시 정규화하지 않는다고 가정 (noadd 그래프). 폰 출력으로 미확인.
5. **x960p1mc sigmoid(heat) 잘림** (npu README 11차: 3ch 골든에서 작은 heat 가 0) — 1ch 는 44.5 dB 로 덜하지만 실제 폰 출력 영향 미측정.
6. **LG 입력이 PC 시제품과 다름**: PC FLGc 는 [추적점 + XFeat 512] 를 LG 에 넣고 XFeat 은 640×360 축소 영상, LG 가지치기 있음. 여기 기본 x512 = XFeat 512 점만 (960×544 원 해상도), 고정 N=512, 가지치기 없음. 루프 수·정확도 영향 미측정.
   또 x512 에서 옛 키프레임 XFeat 점의 3D 는 이전 키프레임과의 NN 삼각측량에서만 생김 (추적점 3D 는 LG 짝에 안 쓰임).
7. **L6_a16mc 양자화(a16w8) 정확도** 는 PC fp32 대역으로만 봄. 폰 실제 짝 품질 미측정 (lgpost README: 폰 CPU 후처리 약 6.4 ms 1 회 측정).
8. **in-model 판(L6p, N×3) / 마스크 판(N×513) 고리**: lgpost README 사전 기준만 보고 짬 (출력 3 칸을 float 로 읽음, 마스크 하나를 양쪽 공유 → 두 쪽 모두 min(nA,nB) 점). 실제 모델로 미시험.
9. **CPU 고정 기본 0-3 (작은 코어)**: VIO 를 피하려는 선택. 후처리·상호 NN(약 700×700×64)·LG 후처리가 작은 코어에서 얼마나 걸릴지 미측정 (추정 키프레임당 수십~150 ms). 늦으면 다리 큐(6) 버림 증가 → loop.jsonl 로 확인. nncrun 6차: cpu 고정이 오히려 느렸던 기록 있음.
10. **기술자 캐시 fp16 + 300 MB 상한 (오래된 것부터 버림)**: 버려진 키프레임이 후보가 되면 오류 −3 → 루프 정지 + 실행 INVALID (긴 비행에서 생길 수 있음, 약 3000 키프레임 추정). 버림 수는 npu.jsonl "evicted".
11. **소켓 이름** `@scan3d_xfnpu_<앱 pid>` (추상). 같은 uid 의 다른 프로세스는 접속 가능 (인증 없음). 서버는 한 번에 한 연결.
12. **다리 바이너리 교체**: 꺼짐 동일성은 PC 빌드로만 확인 (Android 빌드는 같은 소스, 재현성은 경로 문자열 차이만 확인). 폰에서 0.6.29 대비 포즈 동일성 미확인.
13. **MainActivity 의 npu intent 파싱**은 `npu_loop` 이 0 이 아니면만 실행. `npu_cache_mb` 를 int 로 읽음.

## 파일
`PLAN.md`, `appnpu_replay.py` (PC 재생), `ident_check.py` / `ident_pc.sh` / `ident_pc_result.txt`, `vio_same.py` / `vio_same.txt`, `rep_npu.sh` / `rep_npu_summary.txt` /
`rep_A_npu_example.jsonl` (PC 대역 npu.jsonl 예), `pad_check.py` / `pad_check.json`, `test_server_*.json` / `run_test*.sh`, `jt/`, `build_apk.sh` / `apk_build.txt`,
`build_android_bridge.sh`, `android_bridge_repro.txt`.
코드: `scan3d/ov_bridge/loop/ovloop.{h,cpp}` (lgopt xf_* 그대로 + unix 모드), `phone_stereo/arlat/npu/` (`xfnpu_core.h` 서버, `enn_backend.h`, `xfnpu_jni.cpp`, PC 시험 `xfnpu_pc.cpp` · `npu_emu.py` · `test_server.py`,
npuopt10 `xfpost_soft.h` / `xfpost_off.h` 복사), `phone_stereo/arlat/src/com/scan3d/arlat/NpuLoop.java`, `OvRunner.java`, `MainActivity.java`, `AndroidManifest.xml`, `build.sh` (NPU_ASSETS).
.nnc 는 git 밖: APK assets/npu/ (x960p1mc md5 05b4f94f…, x960f1mc 2a958cb7…, L6_a16mc 67936e1e…, lg_wr 38ed0c3b…) = `C:\Users\a3162\scan3d_handoff\npu_results\` 의 원본과 md5 같음.
빌드 원본: WSL `~/scanner/tmp_appnpu/` (build_pc, build_android_new, apk, npu_assets, ident, rep, pc).

## 남은 위험 (0.6.32 에서 새로 생기거나 그대로인 것)
- **UI 스레드 대기**: OvRunner.start 는 UI 스레드에서 불리고, NPU 시작은 별도 스레드지만 UI 스레드가 최대 15 s join 으로 기다린다 (첫 실행 자산 복사 약 9 MB + ENN 열기). 그동안 입력이 오면 ANR 가능 (화면은 안 씀, 추정 위험 낮음). 비동기 시작으로 바꾸려면 OvRunner.start 구조 변경 필요.
- 시작 시간 초과 뒤 남은 네이티브 스레드는 계속 ENN 안에 있을 수 있음 → 앱 재시작 전까지 npu_loop 실행은 모두 거부.
- 거부·INVALID 경로는 PC 에서 다리 쪽(B, C)과 서버 오류 코드(−3)만 시험. 앱(Java) 쪽은 폰 없이 코드 읽기만.

## 폰 실행 1 회 (2026-10-28 20:01, 사용자 승인, 0.6.32 vc 38 설치, `phone_run.sh`, 결과 `phone/`)
- 설치 성공 (vc 35 → 38), DUMP 권한 부여. preflight OK (SKIN 32.1, thermal 0, 배터리 45 %, 충전 안 함). `--es profile rec60 --ei npu_loop 1`, 120 s 요청.
- **NPU 열림 (앱 프로세스)**: logcat `NpuUserDriver::OpenSubGraph/CommitSubGraph`, `Operator[1] (hw: 4) ENN_NPU` (+ `ENN_UNIFIED_DEVICE hw:16` 줄 1 개) `phone/logcat_npu_proof.txt`.
  npu_start.txt: x960p1mc 입력·출력 **f32 버퍼** (fp16 아님), L6_a16mc N 512 A|BT|z0|z1, 워밍업 XFeat 21.4 ms / LG 18.5 ms (1 회, 첫 실행), 작업 스레드 0-3 고정 OK.
  다리 `XFeat server … connected`. meta: npu_loop "on", **run_valid true, bridge_exit 0**, npu_failed 없음, xf_down 없음.
- **그러나 키프레임 0 개 → 키프레임당 XFeat/LG 시간, 큐 버림, VIO 시간 비교 없음.** 이유 두 가지:
  1. 실행이 11 s 에 "stop user" 로 끝남: 다른 앱이 앞으로 와서(logcat noteStopComponent, `phone/stop_cause.txt`) onPause → 정지. 사용자가 폰을 쓰는 중이었던 것으로 보임.
  2. 폰을 책상에 그대로 두면 VIO 가 초기화되지 않는다 (458 프레임 모두 INIT; 동적 초기화는 움직임 필요). 초기화돼도 키프레임 규칙이 움직임(8 cm / 8°)을 요구 → **정지한 폰으로는 NPU 루프 경로를 잴 수 없음.**
- 온도 (`phone/temps_pc.txt`, 5 s): SKIN 32.1 → 34.3, **AP 38.0~49.0** (시작 전부터 44.5, 기준 50 바로 아래), 배터리 45 %. 안전 정지 안 걸림.
- 끝난 뒤 카메라 클라이언트 [] , ov_bridge 프로세스 없음 (`phone/after_check.txt`). 0.6.32 는 그대로 설치 (이상 동작 없음).
- 다음: 손에 들고 천천히 움직이는 2 분 (init 후 키프레임 생김) + 사용자가 폰을 안 쓸 때. 측정값은 모두 미검증.

## 오차 시험 (손에 들고 "출발점으로 돌아오기") — 사전 등록 규칙 (실행 전 커밋)
- 명령: `bash scan3d/runs/appnpu_20261028/run_err.sh <tag> <npu 0|1> <secs>` (0.6.32 rec60 + loop 1 [+ npu_loop 1], 실시간 보기 켬, 안전 관문:
  시작 SKIN < 40 (ov_phone_run 점검은 SKIN < 36), 정지 SKIN ≥ 45 / AP ≥ 50 / 배터리 < 20 % / thermal ≥ 2, 끝나면 카메라 닫힘 확인, 가져오기, 채점).
  채점 `python3 score_err.py err/<tag>`, 비교 `python3 score_err.py --abba err/A1 err/B1 err/B2 err/A2`.
- 순서 ABBA: A1 = npu 1, B1 = npu 0, B2 = npu 0, A2 = npu 1. 같은 경로, 바닥 테이프 표시에서 시작·끝. 시작 3 s·끝 3 s 가만히.
- 1차 지표 = 지도(루프 보정) 끝점 오차 / 경로 길이 (%). 끝점 = 마지막 2 s 평균, 시작 = 추적 첫 0.5 s 평균, 지도 보정 = 마지막 최적화 목표 (yaw, t).
  보조: VIO 끝점 오차 cm, 높이 최대 차, 루프 채택/거절, NPU 시간, 온도.
- 판정: 네 실행 모두 유효 (bridge_exit 0, npu 실행은 run_valid true, xf_down 없음, 안전 정지 없음) 일 때만.
  두 NPU 실행 값이 모두 두 BRIEF 실행 값보다 작으면 "NPU 쪽이 낫다 (4 회, 추정)", 반대면 "BRIEF 쪽이 낫다", 겹치면 "차이 보이지 않음". 무효가 있으면 판정 안 함.
- 한계: 참값 없음 (표시로 돌아온 거리만), 손 경로는 매번 다름, 4 회뿐. BRIEF 쪽 (npu 0) = 0.6.29 와 같은 BoW+BRIEF 루프 설정 (max_cand 3), NPU 쪽은 FLGc 문 (max_cand 1).

### 사용자 안내 (5 줄)
1. 바닥에 테이프로 출발 표시를 하고, 폰을 그 위에 들고 화면은 만지지 않습니다 (다른 앱으로 바꾸지 않기).
2. 코디네이터가 "시작" 하면 3 초 가만히 들고 있다가 천천히 걷기 시작합니다.
3. 방을 한 바퀴 (매번 같은 길) 천천히 돌고, 처음 보던 방향으로 출발 표시에 다시 섭니다.
4. 표시 위에서 3 초 이상 가만히 있으면 끝 — 앱이 정해진 시간 뒤 스스로 멈춥니다.
5. 폰이 뜨겁게 느껴지거나 이상하면 바로 내려놓고 알려 주세요 (자동 안전 정지도 있음).

### A1 (0.6.32, 20:14) = 연습, 무효 — ABBA 는 0.6.33 으로 다시 시작
- 앱이 20:14:04 / 20:14:05 두 번 시작됨 (run 폴더 3 개: 201404 = 바로 끝, 201405 = 약 30 s 1804 프레임, meta 없음; 201451 = 약 27 s, meta 없음).
  두 번 모두 **AP ≥ 50 안전 정지** (20:14:44 AP 50.7, 20:15:27 AP 50.0; 시작 때 AP 27.4 → 약 40 s 만에 50). 왜 두 번째 실행(20:14:51)이 시작됐는지는 모름.
  키프레임 XFeat 2 개: NPU 14.1 ms, 후처리+질의 21.6 ms, 상호 NN 70.5 ms (작은 코어 0-3, 1 회씩, 미검증). LG 0 회, 루프 0.
- 사용자: 화면에 아무것도 안 보여 시작·움직일 때를 몰랐음 → 0.6.33 에 화면 안내 추가. 조건이 바뀌므로 A1 은 연습으로만 두고 ABBA 는 0.6.33 에서 처음부터.
- **AP 가 문제**: 정지 기준 AP 50 에 40 s 안에 닿음 (NPU 실행). 설치 직후 쉬는 상태에서도 AP 56.6 을 봄 (20:21, 사용자 사용 중일 수 있음).
  run_err.sh 시작 관문에 AP < 50 추가 (그렇지 않으면 감시가 곧바로 멈춤).

### 0.6.33 (vc 39, sha f9ddd3b7…, 20:21 설치): 화면 안내 (RunCue)
- secs 가 있고 guide 가 없는 mode ov 실행에서 큰 글자: "준비 중 — 가만히" (첫 프레임 뒤 3 s; OpenVINS 정적 초기화 = 가만히 있다가 첫 움직임에 시작)
  → "출발 — 천천히 한 바퀴 + 남은 초" (출발 6 s 뒤에도 추적 아님: "초기화 안 됨 — 멈췄다 다시 출발") → 마지막 15 s "출발 표시로 돌아와 멈추기"
  → 마지막 3 s "끝 — 가만히 3초" → "저장 중" → "완료". 아래 작은 줄: VIO 추적 중/초기화 전/잃음, NPU 꺼짐/열림/거부/무효. 2 Hz, 소리·진동 없음, npu 0/1 같음.
  주의: 요청 문구 "VIO 초기화 + 3 s 뒤 출발" 은 정적 초기화가 움직임을 기다리므로 막힘 → "첫 프레임 + 3 s" 로 바꿈.
- NPU "무효" 표시용: 서버가 Q 없이 끊긴 연결을 실패로 기록, 다리 xf_down 이 소켓을 닫음 (실패 경로만 바뀜). PC 꺼짐 동일성 다시 IDENTICAL.

### R1 (0.6.33, 20:26, npu 1) = 무효 — ABBA 는 0.6.34 로 R1 부터 다시
- VIO 가 실행 내내 init (logcat 상태 줄) — 화면 "준비 중 — 가만히" 를 보고 사용자가 계속 가만히 있었던 것으로 보임 (정적 초기화는 첫 움직임에서 시작).
- 앱 재시작 원인 (logcat): 20:27:00 우리 안전 감시가 AP 50.1 로 force-stop → 20:27:05·20:27:08 **런처(uid 10161, recents/아이콘)가 작업을 다시 열어**
  옛 intent (extras 포함, FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY) 를 다시 전달 → 같은 설정으로 새 실행 (run_20261028_202708). 감시가 또 정지 (20:27:06, 20:28:48 AP 50.0).
  끝은 run_err 시간 초과 force-stop. 즉 R1 에도 AP ≥ 50 안전 정지가 있었다.

### 0.6.34 (vc 40, sha 15fe1576…, 20:34 설치)
- 화면: 첫 프레임 + 3 s 뒤 **항상** "출발 — 폰을 위아래로 살짝 흔든 뒤 천천히 걷기"; 20 s 넘게 초기화 안 되면 "폰을 조금 더 크게 흔들기";
  초기화되면 초록 "VIO 시작됨" 3 s → 남은 시간은 **VIO 초기화부터** 셈 (intent run_secs). 마지막 15 s "출발 표시로 돌아와 멈추기", 마지막 3 s "끝 — 가만히 3초", 끝나면 앱이 스스로 끝냄.
  강제 상한 = intent secs (run_err.sh: secs = run_secs + 60, PC 쪽 시간 초과 = 그 + 30).
- 재시작 막기: mode ov 는 기록(recents)에서 다시 열린 intent 로는 시작 안 함, 한 activity 에서 한 번만.
- 네이티브 (다리·libxfnpu) 는 0.6.33 과 같은 바이너리 (sha 2ed40e6b / c6d54e83).
- run_err.sh: `<secs>` = VIO 초기화부터의 실행 길이. vc 40 확인.
