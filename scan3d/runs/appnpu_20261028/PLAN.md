# appnpu 2026-10-28 — 폰 앱에 XFeat + LighterGlue (S24 NPU) 루프 클로저 경로 넣기 (계획, 코딩 전 커밋)

브랜치 `appnpu-20261028` = `lens05-20261027` 9ef84a0 (폰에 깔린 앱 0.6.29 vc 35, 렌즈 0.5 dpt calcap k12_notan) 위.
폰 연결 없음 → 설치·adb 안 함. 산출물 = APK + 롤백 APK(0.6.29) + 실행 안내. 독립 검토자가 따로 본다.

## 0. 기준 확인 (읽은 것)
- 폰에 깔린 것 = 0.6.29 (`runs/lens05_20261027/README.md` 4·5절, 09:14:35 설치, versionName 0.6.29-lens05-20261027).
  libov_bridge.so = integ2 android 빌드 (sha 6f29ce70…), 네이티브 소스 = lens05 9ef84a0 의 `scan3d/ov_bridge` (lens05 는 네이티브 안 바꿈).
- 다른 브랜치: integ2(0.6.28, 더 옛), ovrefine(오프라인 평가만), fcarm/gyrocheck(ESP32·분석) → 앱 기준 아님.
- PC 시제품: `lgopt-20261027` (xfopt 위, 부모 = lens05 9ef84a0). 다리(ov_bridge)의 `--loop-xf CMD` + `--loop-cfg xf_*`
  (기본 꺼짐) 가 루프 스레드에서 특징/짝 서버(xf_server.py)와 stdin/stdout 이진 규약으로 이야기한다:
  `'F' id W H img nq q[]` → `nk kps[] ms`, `'M' a b mode` → `n (ia ib score)[] ms`, `'Q'`.
  추천 조건 FLGc = xf_mode 2 (LG), topk 512, `xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1` (+서버 LG 가지치기).
  XFeat 은 **매 키프레임**(옛 키프레임 기술자가 있어야 짝을 찾음), LG 는 **BoW 후보(≥3 s, ≥1 m)에만 1 번**, 삼각측량용 짝은 상호 NN.
- NPU: `runs/npu_20261027/nncrun` (ovxfeat, 읽기만) — 앱 프로세스(untrusted_app)에서 `dlopen("libenn_public_api_cpp.so")`
  (vendor public.libraries.txt, 매니페스트 `<uses-native-library required=false>`) → EnnInitialize/OpenModel/AllocateAllBuffers/ExecuteModel.

## 1. 구조 (결정)
- **NPU 는 앱 프로세스 안**에서 돈다 (지시: nncrun 과 같은 방식). VIO 다리(ov_bridge)는 앱이 띄운 별도 실행 파일이라
  vendor 라이브러리 dlopen 이 링커 이름공간 때문에 안 될 가능성이 크다 (추정, 시험 안 함) → 다리에는 ENN 코드를 넣지 않는다.
- 새 JNI 라이브러리 `libxfnpu.so` (앱이 **기능이 켜졌을 때만** System.loadLibrary) = "NPU 특징/짝 서버":
  추상 유닉스 소켓 `@scan3d_xfnpu_<앱pid>` 에서 위 xf_server 규약을 그대로 받는다. 서버 스레드 1 개(작업 스레드), nice 10, CPU 고정(아래).
- 다리: `--loop-xf unix:@NAME` 이면 자식 프로세스 대신 그 소켓에 연결 (나머지 xf_* 코드는 lgopt 그대로).
  소켓 읽기/쓰기 시간 제한(기본 3 s). 서버가 죽거나 시간 초과 → 그 실행 동안 xf 끔, **BRIEF 경로로 되돌아감**(unix 모드만; PC CMD 모드는 lgopt 그대로).
  연결 실패도 치명 아님(BRIEF 로 계속). VIO 스레드는 키프레임 복사만 (기존 ovloop 그대로) → VIO 프레임은 절대 기다리지 않는다.
- 앱: intent `npu_loop 1` (기본 0). loop 가 켜진 실행(rec60 기본)에서만 의미. 시작 순서: ENN 열기·모델 열기·버퍼 할당·워밍업 1 회 →
  성공이면 `--loop-xf unix:@… --loop-cfg <npu 설정>` 을 다리 인자에 붙임. **실패면 기능 끔 + events/meta/npu.log 에 이유, 다리 인자는 0.6.29 와 같음.**
  모델에 CPU/GPU 대체 경로 없음 (ENN 실패 = 끔).

## 2. 모델 · 전후처리
- XFeat 기본 `x960p1mc` (960×544 1ch, softmax/sigmoid 모델 안, `npu_results/xf960soft/x960p1mc/*.nnc`), 후처리 = npuopt10 `xfpost_soft.h`
  (공식 bilinear 점수 · bicubic 기술자, ymax 540). 선택 `x960f1mc` + `xfpost_off.h` (출력 130 채널이면 자동). 어느 쪽인지는 출력 버퍼 크기로 판별.
  입력 = 키프레임 영상(다리가 보낸 것 = VIO 가 보는 remap 된 960×540 회색) → 아래 4 줄 복제(BORDER_REPLICATE)로 544 → /255 →
  영상 전체 평균·분산 정규화 (eps 1e-5, xf960_eval.py run960 과 같음) → float32 NCHW.
  queries(추적점) 기술자 = 같은 bicubic 식으로 실수 좌표에서 샘플 (xf_server: 같은 조밀 지도 + bicubic).
- LighterGlue 기본 `L6_a16mc` (x [1,512,512] → out [1,512,1026] = A|BT|z0|z1), CPU 후처리 = nncrun lgpost 2 (P=A·BTᵀ, log + logsig, 상호 argmax, exp(max)>0.1).
  키포인트 정규화 = kornia (k − [W/2,H/2]) / (max(W,H)/2), W×H = 다리가 보낸 영상 크기 (xf_server 와 같음). 위치 인코딩 W = lg_wr (2×48) 을 앱 자산에.
  고리(hook): 모델 출력이 N×3 이면 "모델 안 짝 결정" 판(L6p / 마스크 L6pm768, 다른 에이전트가 만드는 중)으로 보고 O(N) 마무리만,
  입력이 N×513 이면 마스크 칸(실제 1 / 채움 0) 채움. 규격은 lgpost README 사전 기준대로 (미검증, 모델 나오면 확인).
- 입력 모드 플래그 `npu_lg_in`: `x512` (기본: XFeat 512 점만 LG 에) / `trk768` (추적점 + XFeat, 768 마스크 모델 필요; 마스크 모델이 아니면 시작 시 기능 끔).
  ※ PC FLGc 는 LG 에 [추적점 + XFeat 512] 를 넣었다 → 기본 x512 는 PC 시제품과 **다른 입력** (짝 수·정확도 영향 미측정).
  N<512 일 때 채움 방식은 PC 에서 L6 fp32 로 비교해 정한다 (마스크 없는 모델의 한계, 위험 목록).
- 상호 NN (삼각측량용, xf_tri_mode 1) = CPU, cos ≥ 0.82 (xf_server 기본). 기술자 캐시 = 키프레임별 fp16, 용량 제한(기본 300 MB, 넘으면 오래된 것부터 버림 + 기록).
- .nnc 는 삼성 산출물이라 git 에 넣지 않는다. APK assets 에 넣고 앱이 `files/npu/` 로 복사(크기 다르면 다시) → ENN 에 파일 경로.
  `files/npu/` 에 사용자가 adb push 한 파일이 있으면 그것을 쓰는 이름 지정도 가능 (intent npu_xf / npu_lg).

## 3. 플래그 (전부 기본 꺼짐 = 0.6.29 와 같은 동작)
- intent `npu_loop` 0|1, `npu_xf` (파일 이름, 기본 x960p1mc), `npu_lg` (기본 L6_a16mc), `npu_lg_in` x512|trk768, `npu_topk` 512,
  `npu_cpus` (서버 스레드 CPU, 기본: 다리 `cpus` 가 있으면 그 여집합, 없으면 0-3 = 작은 코어; "none" = 고정 안 함), `npu_cfg` (다리 --loop-cfg 덧붙임),
  `npu_perf` (EnnSetPreferencePerfMode, 기본 -1 = 안 건드림).
- 다리 기본 npu 설정 = FLGc 문: `xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1` (+ npu_cfg).
- 기록 `<run>/npu.jsonl` : 키프레임마다 정규화+입력 / NPU / 출력 / 후처리 / 질의 기술자 µs, 키포인트 수, LG 마다 pack / NPU / post µs, N 실제 수,
  SKIN/AP (2 s 마다 thermalservice dump, DUMP 권한 있을 때) + 배터리 + thermal status, 캐시 크기·버림, 오류. `<run>/npu_start.txt` = ENN 열기 기록.
  다리 `loop.jsonl` 의 kf 줄 "xf" (서버 ms) · 큐 버림(dropped) 은 그대로.

## 4. 확인 순서
1. 꺼짐 동일성: PC 재생(std60 165506, c4f60 새 렌즈, `--profile rec60 --drop-late 0`, loop 1 `--loop-sync 1`)으로
   lens05 소스 빌드 vs 이 브랜치 빌드 → 포즈 줄 + loop.jsonl (ms 빼고) 같아야 함. 앱 쪽은 npu_loop 0 이면 인자·동작 같음을 코드로 확인 (Java 시험).
2. 기능 시험 (PC, ENN 대신 torch 대역): 같은 C++ 서버 코드를 PC 로 빌드, NPU 호출만 파이썬 torch 대역(공식 XFeat + lgnpu L6 fp32)으로 바꿔
   다리 재생과 연결 → 규약·인덱스·전후처리 동작 확인 (정확도 판정 아님). 서버 C++ 후처리 = 기존 검증된 헤더 그대로.
3. APK 빌드 (vc 36, 0.6.30-appnpu-20261028), 서명 같은 개발 인증서, 롤백 = 0.6.29 APK (lens05 사본 sha 5feb3e53…).
4. 독립 검토용 위험 목록 + 실행 안내 (README).
측정 안 한 시간·정확도는 모두 "추정".
