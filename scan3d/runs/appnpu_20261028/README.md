# appnpu 2026-10-28 — 앱 0.6.30: XFeat + LighterGlue (S24 NPU) 루프 클로저 경로 (기본 꺼짐)

브랜치 `appnpu-20261028` (WSL `~/scanner/StereoDepthSeg-appnpu`) = 폰에 깔린 0.6.29 (`lens05-20261027` 9ef84a0) 위. 계획 = `PLAN.md` (코딩 전 커밋 4432f68).
**폰에 설치 안 함, adb 안 씀 (폰 연결 없음).** 폰 시간·정확도는 전부 미측정 → 아래 숫자 중 폰 값은 모두 **추정**.

## 결론
- APK `scratchpad\team\appnpu\apk\arlat_0.6.30_appnpu.apk` (vc 36, `0.6.30-appnpu-20261028`, sha 5507a270…, 같은 개발 인증서 CN=PhoneStereo Development;
  libov_bridge.so afd3aeba…, libxfnpu.so 8f118c94…, 커밋 = 이 README 를 넣은 커밋의 코드).
  롤백 = `scratchpad\team\appnpu\apk\arlat_0.6.29_rollback.apk` (= lens05 사본, sha 5feb3e53…, vc 35).
- intent `npu_loop 1` 일 때만 켜짐. 안 주면 0.6.29 와 같은 동작 (다리 인자 같음, libxfnpu.so 안 읽음). 단 **libov_bridge.so 는 새 빌드**(코드 추가, 꺼짐 경로 동일 — 아래 확인).
- 켜면: 앱 프로세스 안 ENN(`libenn_public_api_cpp.so` dlopen)으로 XFeat `x960p1mc`(매 키프레임) + LighterGlue `L6_a16mc`(BoW 후보에만) 를 돌리고,
  다리의 루프 스레드가 추상 유닉스 소켓으로 요청 (VIO 스레드는 키프레임 복사만, 기다리지 않음). ENN/모델 열기 실패 → 기능 끔 + 이유 기록, 다리는 0.6.29 인자 그대로.
  실행 중 서버가 죽거나 3 s 응답 없음 → 그 실행 동안 BRIEF(기존) 검증으로 되돌아감.

## PC 에서 확인한 것 (폰 아님)
| 확인 | 방법 | 결과 |
|---|---|---|
| 꺼짐 동일성 (다리) | lens05 소스 PC 빌드 vs 이 브랜치 PC 빌드, std60 165506 + 폰 c4f60 새 렌즈 폴더, `--profile rec60 --drop-late 0 --loop 1 --loop-sync 1` (`appnpu_replay.py`, `ident_check.py`) | 포즈 1690 줄 · loop.jsonl 59 줄 **같음** (시간 칸 ms/remap_ms 만 뺌) `ident_pc_result.txt` |
| Android 다리 재현성 | lens05 소스 Android 빌드 = 폰 0.6.29 libov_bridge 와 크기 같음(13,678,288 B), 차이 = 내장 소스 경로 문자열(-lens05 vs -integ2) 7 개 | sha 는 다름 (경로 때문, 추정) `android_bridge_repro.txt` |
| 서버 C++ = 독립 파이썬 기준 | PC 시험 빌드 `xfnpu_pc` (NPU 대신 torch fp32 대역 `npu_emu.py`) + `test_server.py`: 실제 라이브 경로 프레임 2 장 (cam5 → c4f60 remap 960×540), 질의점 150 | 키포인트 512/512 **동일**, 상호 NN 짝 251/251 동일, LG 짝 175/175 동일. topk 200·333 (채움 경로)도 동일. fp16 버퍼 경로: 키포인트 집합 99.8 % `test_server_*.json` |
| LG 점 부족 시 채움 (마스크 없는 N=512 모델) | `pad_check.py` PC fp32, 5 쌍 × 8 점수 조합, 기준 = 같은 망을 실제 점만으로 | 반복+복사합(lg_pad 2) Jaccard 0.94~1.00 (N 이 배수면 1.00), 0 채움 0.64~0.96, 반복만 0~0.97 → **기본 lg_pad 2** `pad_check.json` |
| 다리+서버 연결 (PC 재생) | 같은 녹화, FLGc 문, 서버 = PC 대역 (`rep_npu.sh`) | A 서버 있음: 검증 12 / 채택 10 (BRIEF 만: 5 / 3). B 서버 없음: 연결 실패 → BRIEF, C 서버 도중 강제 종료 → "lost" → BRIEF. **세 경우 모두 VIO 포즈(지도 칸 뺀) 같음** `vio_same.txt`, `rep_npu_summary.txt` |
| Java 도우미 | `jt/TNpu.java` (CPU 고르기, 다리 설정 문자열) | 8/8 통과 `jt/run_t.log` |
- PC 대역 시간(ms)은 torch CPU 값이라 폰과 무관. 정확도 판정 아님 (한 녹화, 1 회).

## 실행 안내 (나중에 사용자가)
1. 설치: `adb install -r arlat_0.6.30_appnpu.apk` (같은 인증서 → 데이터·files/ov 유지). 렌즈 세트(files/ov/c4f60 …)는 그대로 씀.
2. 온도 기록(SKIN/AP)을 원하면 (HsRecorder 와 같음): `adb shell pm grant com.scan3d.arlat android.permission.DUMP`. 없으면 npu.jsonl 의 skin/ap = null (배터리·thermal status 는 기록).
3. 실행 (기존 스크립트에 extras 만 추가): `bash scan3d/tools/ov_phone_run.sh run <ip:port> c4f60 <secs> --es profile rec60 --ei npu_loop 1`
   - rec60 이 loop 1 을 켠다 (loop 0 이면 npu 는 "off: loop 0" 로 꺼짐).
   - 선택: `--es npu_xf x960f1mc` (post_off 판), `--es npu_lg_in trk768` (마스크 768 모델이 생기면 + `--es npu_lg <이름>`), `--ei npu_lg_pad 0|1|2`,
     `--ei npu_topk 512`, `--es npu_cpus 0-3|none|…` (기본: 다리 cpus 가 있으면 그 여집합, 없으면 0-3), `--ei npu_nice 10`, `--ei npu_perf <ENN perf mode>`,
     `--ei npu_cache_mb 300`, `--es npu_cfg max_cand=2,…` (다리 --loop-cfg 뒤에 붙음), `--ei npu_timeout_ms 3000`.
   - 새 모델: `adb push NAME.nnc /sdcard/Android/data/com.scan3d.arlat/files/npu/` 후 `--es npu_lg NAME` (LG 는 `lg_wr_NAME.npy` 도 같이). 앱이 내부 files/npu/ 로 복사해 ENN 에 경로로 줌.
4. 결과 (run 폴더): `meta.json` "npu_loop" = "on" 또는 "off: 이유", `npu_start.txt` (ENN 열기·버퍼 크기·워밍업 µs), `npu.jsonl` (줄마다:
   F = prep/npu/post_q/store/tot/recv µs, post 단계 4 개, nk, nq, skin/ap/bat/tstat, 캐시; M = mode 1/2, nA/nB, nv(LG 실제 점 수), 짝 수, pack/npu/post µs),
   `loop.jsonl` (kf 줄 "xf" = [키포인트, 3D 점, 삼각, 서버 ms feat/tri/match], 후보 줄 "xn" = 짝 수, 경로 "xf"/"brief"; "xf_down" 줄 = 서버 잃음),
   `bridge_stderr.log` ("XFeat server … connected", 끝에 "keyframes pushed … dropped …" = 큐 버림).
5. 롤백: `adb install -r -d arlat_0.6.29_rollback.apk` (vc 35). files/npu/ 는 남아도 0.6.29 는 안 읽음.

## 독립 검토자에게: 위험한 곳 (확인 부탁)
1. **ENN 이 앱 프로세스에서 실제로 열리는지** — nncrun 은 instrument(화면 없음) 프로세스에서 성공. 이 앱은 Activity + 카메라 + ARCore 라이브러리가 같이 있는 프로세스. 매니페스트 `uses-native-library` 를 넣음 (nncrun 과 같음). 미검증.
2. **ENN 동시 실행**: 같은 앱 안 다른 NPU 사용자 없음(추정). ENN 호출은 서버 작업 스레드 하나에서만 (init·open·exec·close). EnnExecuteModel 이 오래 막히면 다리는 3 s 뒤 BRIEF 로. 앱 종료 때 서버 스레드가 3 s 안에 안 끝나면 detach + 객체를 일부러 해제하지 않음
   (`xfnpu_core.h` stop(3000), `xfnpu_jni.cpp` nStop) → ENN 정리 안 됨 (프로세스 종료 때 OS 가 회수, 추정).
3. **NumberOfBuffersInfo/EnnBuffer 구조**를 nncrun 과 같은 방어적 읽기로 가정 (in 1 / out 1 아니면 끔). x960p1mc 버퍼가 fp32 인지 fp16 인지 모름 → 크기로 판별 (둘 다 처리, fp16 은 PC 대역으로만 시험).
4. **XFeat 입력 정규화**: 영상 전체(544 줄, 복제 4 줄 포함) 평균·분산 — npuopt10 `xf960_eval.py run960` 과 같다고 봄. 모델이 안에서 다시 정규화하지 않는다고 가정 (noadd 그래프). 폰 출력으로 미확인.
5. **x960p1mc sigmoid(heat) 잘림** (npu README 11차: 3ch 골든에서 작은 heat 가 0) — 1ch 는 44.5 dB 로 덜하지만 실제 폰 출력 영향 미측정.
6. **LG 입력이 PC 시제품과 다름**: PC FLGc 는 [추적점 + XFeat 512] 를 LG 에 넣고 XFeat 은 640×360 축소 영상, LG 가지치기 있음. 여기 기본 x512 = XFeat 512 점만 (960×544 원 해상도), 고정 N=512, 가지치기 없음. 루프 수·정확도 영향 미측정.
   또 x512 에서 옛 키프레임 XFeat 점의 3D 는 이전 키프레임과의 NN 삼각측량에서만 생김 (추적점 3D 는 LG 짝에 안 쓰임).
7. **L6_a16mc 양자화(a16w8) 정확도** 는 PC fp32 대역으로만 봄. 폰 실제 짝 품질 미측정 (lgpost README: 폰 CPU 후처리 약 6.4 ms 1 회 측정).
8. **in-model 판(L6p, N×3) / 마스크 판(N×513) 고리**: lgpost README 사전 기준만 보고 짬 (출력 3 칸을 float 로 읽음, 마스크 하나를 양쪽 공유 → 두 쪽 모두 min(nA,nB) 점). 실제 모델로 미시험.
9. **CPU 고정 기본 0-3 (작은 코어)**: VIO 를 피하려는 선택. 후처리·상호 NN(약 700×700×64)·LG 후처리가 작은 코어에서 얼마나 걸릴지 미측정 (추정 키프레임당 수십~150 ms). 늦으면 다리 큐(6) 버림 증가 → loop.jsonl 로 확인. nncrun 6차: cpu 고정이 오히려 느렸던 기록 있음.
10. **기술자 캐시 fp16 + 300 MB 상한 (오래된 것부터 버림)**: 긴 비행에서 출발점 키프레임이 먼저 버려짐 → 그 루프를 못 찾음. 버림 수는 npu.jsonl "evicted".
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
