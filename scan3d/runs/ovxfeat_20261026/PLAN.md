# ovxfeat 2026-10-26 — XFeat + LighterGlue 로 루프 기하 확인 (계획, 평가 셋 사전 선언)

브랜치 `ovxfeat-20261026` (ovloop-20261026 0d71a86 에서). 사용자 결정 10-26 (고정): 루프 닫기 기하 확인에 XFeat (점+기술자) + LighterGlue (매칭기).
목표: 흰 벽 방에서 BoW 가 맞는 후보를 찾고도 매칭 0–11 개로 거절되던 것을 고친다. 60 fps (rec60) 만.

## 출처 (다운로드 기록)
- XFeat + LighterGlue: https://github.com/verlab/accelerated_features commit e92685f57f8318b18725c5c8c0bd28c7fe188d9a (2025-01-15),
  LICENSE = Apache-2.0 (저장소 LICENSE 파일 확인). 가중치 저장소 안 `weights/xfeat.pt` (sha256 0f5187fd…), `weights/xfeat-lighterglue.pt` (sha256 766102df…).
  LighterGlue 코드 = kornia `kornia.feature.lightglue` (kornia 0.7.2, Apache-2.0; 원본 LightGlue cvg/LightGlue Apache-2.0).
  가중치 학습 데이터 = MegaDepth (glue-factory) — 가중치 자체 라이선스는 저장소 Apache-2.0 로 배포됨 (데이터셋 조건은 따로 확인 안 함 = 가정).
- 폰 벤치: TFLite `benchmark_model` android_aarch64 nightly (storage.googleapis.com/tensorflow-nightly-public/.../android_aarch64_benchmark_model, sha256 5d45116e…).

## 구조 (PC 시제품)
- 후보 찾기 = BoW 그대로 (B 와 같은 후보, max_cand 3). 바뀌는 것 = 후보 쌍의 점 매칭만. PnP RANSAC/LM, roll-pitch/drift/yaw 문, 시간 일관성, 그래프 모두 동일.
- 브리지 `--loop-xf "CMD"`: 루프 스레드가 XFeat 서버(파이썬, PC GPU) 를 자식 프로세스로 띄워 파이프로 요청 (`--loop-sync 1` 이면 결정적).
  키프레임마다: 영상 + 추적점(3D 가 있을 수 있는 OpenVINS 점) 위치 → 서버가 XFeat 점 top-k 검출 + 추적점 위치에서 기술자 샘플링.
  후보 쌍마다: 서버가 매칭 (mode NN = 상호 최근접 cos ≥ 0.82, mode LG = LighterGlue min_conf 0.1) → 인덱스 쌍.
  방향 A: 옛 KF 의 3D 점 ↔ 현재 2D, 방향 B: 현재 3D ↔ 옛 2D (기존과 같은 규칙).
- **사후 추가 (19:55, 첫 결과를 본 뒤)**: XFeat 매칭은 많아도(250–520) 3D 가 있는 점이 9–23 개뿐이라 손 루프가 안 닫힘 → `xf_tri` 추가.
  따라서 `xf_tri` 는 사전 선언이 아니다. 섞인 변수를 가르려고 대조군 `B60t` (box-BRIEF + 같은 삼각측량), 시차 변형 `*t3` (xf_tri_deg 3) 추가 (감독 20:05).
- 옵션 `xf_tri` (필요할 때만): XFeat 점을 앞 키프레임과 매칭해 VIO 자세로 삼각측량 → 3D 점을 늘림 (흰 벽에서 OpenVINS 3D 점이 적을 때).
- 기본값: `--loop-xf` 없으면 기존 BRIEF 경로 그대로 (동일성 확인).

## 평가 (ovloop README 와 같은 자, 단 60 fps: `--profile rec60 --drop-late 0 --loop 1 --loop-sync 1`, histogram NONE, 960x540, pts 200, clones 15)
사전 선언 셋 (바꾸지 않음):
1. 손 루프 142148 / 142610 (흰 벽, 핵심): 끝→시작 루프가 닫히나, 시작–끝 오차 VIO → 지도 (cm), 후보별 매칭/인라이어 수. (151646 은 60 fps 에서 VIO 발산 = 참고만)
2. std60 165506 o0–o2: 지도 ATE (GT), 루프 에지 GT 대조.
3. HS 42칸 (ovloop 과 같은 칸, DROP 제외): 루프 에지 수, 틀린 루프 (>10 cm 또는 >3°; 3–10 cm 경계도 따로 보고), recall (recall.py 같은 자격), ATE 중앙값.
조건 (주 비교, 세 셋 모두):
- `B60`  BoW 후보 + box-BRIEF 매칭 (기준선)
- `N1k`  BoW 후보 + XFeat sparse top-k 1024, 입력 640x360, 상호 NN
- `L1k`  같은 XFeat + LighterGlue
사전 선언 변형 목록 (조정자 요청 19:2x: "점을 더 쓰면 나아지나, 어디서 멈추나"; 손 루프 + std60 먼저, 고른 것은 HS 도):
- top-k 512 / 1024 / 2048 / 4096 × {NN, LG}, 입력 640x360
- XFeat* (semi-dense, `detectAndComputeDense` 다중 스케일 + 정제 없이 거친 점) top-k 4096 / 8000 × NN (LighterGlue 는 sparse 점으로 학습됨 → XFeat* + LG 는 참고만)
- 입력 960x540 (KF 원본) top-k 4096 × {NN, LG} = "최대" 후보
보고: 후보당 매칭 수, 루프 닫힘, 틀린 루프, PC 시간 (GPU 공유 → 참고).
사용자 결정 (조정자 19:3x): 폰에서 키프레임당 총 ~150 ms (XFeat NPU + LighterGlue NPU/GPU) 안에 드는 가장 무거운 변형을 기본으로 권장; 정확도 스윕은 "최대가 나쁘지 않음(틀린 루프 ~0)" 확인용.

HS 조건 목록 (사후, std60 기준 선별, 20:25 — HS 결과를 보기 전에 기록): B60, B60t, L1k, N1k, L1kt, N1kt, L1kt2, L512t, N4kt, L4kFt, N4kFt (11개 모두 HS 표에 보고).
틀린 루프 판정 (감독 21:31, XFeat HS 결과를 보기 전에 기록): HS 42칸 (DROP 3칸 제외, 모든 조건 같은 42칸) 에서 사전 규칙 (>10 cm 또는 >3°, 문턱 그대로) 의
**개수**가 B60 개수 이하일 것 (비율 = 틀림/GT 확인 에지도 같이 보고). 통과하는 변형이 없으면 **기본값 = B60 (지금 그대로)**.
HS 는 이 브랜치 브리지 c9a32937 (L60 VIO, gyro_klt 없음) 기준. GL60 (integ2 + 패치, aacfd992) 은 손 루프/std60 만, 가능하면 고른 변형 1개 + B60 만 HS 재실행.
기본값 선택 규칙 (사전, 조정자 19:3x): 폰 측정 후 150 ms 안의 가장 무거운 변형 중 HS 틀린 루프 ≤ B60 인 것. 폰 측정 전에는 고르지 않는다.
std60 = 녹화 1개 × 시작 오프셋 3 (독립 반복 아님). s173432 는 모든 조건에서 VIO 초기화 실패/발산 (ov60eval 와 같음) → 점수 없음.
자 메모: 손 루프 시작–끝 오차는 ov60eval L60 과 같은 `loop_eval` (이 브랜치 = ovloop 기반, gyro_klt 없음) → B60 4.48 / 11.2 cm = ov60eval L60 값과 같음.
integ2/dpvo 의 GL60 (gyro_klt=5 브리지, 9.14/16.54 cm) 과는 VIO 가 달라서 숫자가 다르다. 시작–끝 오차는 틀린 루프로도 줄 수 있으므로 판정은 GT (std60/HS) 로.
채택 문턱 (사전): 손 루프 중 1개 이상 닫힘 + 오차 감소, HS 틀린 루프 ≤ B60 (+경계 사례 보고), HS/std ATE 중앙 나빠짐 ≤ 0.02 cm.
반복: 결정적 (sync), 칸당 1회; GPU 결정성은 같은 칸 2회 재실행으로 확인.

## 폰 (카메라 없음, 폰 락 + calcap 세션 없을 때만, 짧게, SKIN/AP 기록)
- XFeat 백본(model.py XFeatModel) → ONNX 고정 입력 1x1xHxW (640x352? 실제 32 배수: 640x352, 480x256, 960x544) → TFLite (onnx2tf) fp32/fp16.
- LighterGlue → 고정 점 수 N (512/1024/2048) ONNX → TFLite (가능하면).
- `benchmark_model` 로 CPU (4 스레드), GPU delegate, NNAPI (→ Samsung enn NPU) ms. N 에 따른 LighterGlue 비용 (~N²).
- 안 되면 이유와 다음 단계를 정확히 적는다.
