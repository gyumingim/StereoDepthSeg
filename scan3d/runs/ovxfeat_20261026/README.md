# ovxfeat 2026-10-26/27 — XFeat (+LighterGlue) 로 루프 기하 확인

브랜치 `ovxfeat-20261026` (ovloop 0d71a86 기반). 계획·사전 선언·사후 기록 = `PLAN.md`. 모든 기능은 플래그 뒤 (기본 = 기존과 동일).

## 결론 (먼저)
- **사전 규칙 판정: 통과한 변형 없음 → 기본값은 B60 (BoW + box-BRIEF) 그대로. 병합 권장 안 함.**
  규칙 (PLAN, HS XFeat 결과 채점 전 기록): HS 42칸 같은 칸에서 틀린 루프 (>10 cm 또는 >3°) **개수** ≤ B60 개수.
- 흰 벽 손 루프는 XFeat + 삼각측량(xf_tri, **사후 추가**)으로 닫힌다 (닫힘, GT 없음 = 미검증). 그러나 HS 에서 틀린 루프가 7–11 배.
- 폰 NPU 는 NNAPI 로 못 씀 (Android 16 S24: TFLite NNAPI 위임이 그래프 거부 → CPU, ORT NNAPI 176 ms = NPU 아님, LighterGlue NNAPI OP_FAILED). 다음 단계 = Samsung ENN SDK.
- LighterGlue 는 폰 CPU 에서 N=512 66 ms / 1024 195 ms (참고 측정) → 키프레임당 4번이면 150 ms 초과. NN 매칭 경로만 예산 안.

## HS 42칸 (브리지 c9a32937, **L60 VIO 기준**, gyro_klt 없음; 반복 1회, 결정적)
| 조건 | 루프 에지 (GT 확인) | 틀린 루프 (사전 규칙) | 비율 | 3–10 cm 경계 | 최대 오차 | 지도 ATE 중앙 (VIO 0.680) |
|---|---|---|---|---|---|---|
| **B60** (기준) | 4502 (4391) | **2** | 0.05 % | 8 | 3.6 cm / 3.6° | 0.700 |
| N1k (XFeat NN, 삼각 없음) | 6756 (6405) | 3 | 0.05 % | 13 | 4.0 / 3.8° | 0.680 |
| B60t (BRIEF + 삼각, 대조군) | 9055 (8640) | 9 | 0.10 % | 20 | 4.2 / 3.4° | 0.660 |
| N4kFt | 8304 (7780) | 14 | 0.18 % | 19 | 5.6 / 7.1° | 0.690 |
| L1kt / L1kt2 / L512t | 8492 / 8465 / 8568 | 16 / 16 / 16 | 0.20 % | 31 / 45 / 18 | 6.4 / 4.0–4.8° | 0.665–0.680 |
| N1kt | 8247 (7740) | 19 | 0.25 % | 14 | 4.3 / 3.9° | 0.715 |
| L1k (LG, 삼각 없음) | 7733 (7269) | 20 | 0.28 % | 33 | 5.0 / 3.6° | 0.645 |
| L4kFt / N4kt | 8465 / 8202 | 21 / 21 | 0.27 % | 28 / 21 | 5.9 / 7.5° , 4.9 / 4.3° | 0.690 / 0.710 |
| N2kFt (폰 예산 후보) | 8330 (7808) | 22 | 0.28 % | 20 | 5.3 / **10.9°** | 0.700 |

- 틀린 루프는 모두 회전 >3° (위치 >10 cm 는 0). HS 지도 ATE 는 어느 조건도 뚜렷한 이득 없음 (칸별 차 중앙 −0.015…+0.025 cm).
- HS 조건 목록은 std60 을 보고 사후에 골랐다 (PLAN 에 HS 채점 전 기록). 규칙 기록(21:48) 때 결과 파일은 이미 40–45칸 있었으나 채점 전.

## 흰 벽 손 루프 (시작–끝 cm, GT 없음; 1회) — 자 = ov60eval L60 (B60 = L60 값과 같음)
- B60: 142148 4.48 / 142610 11.2, 루프 0. XFeat 만 (L1k/N1k): 루프 0 — 매칭은 250–520 개지만 3D 있는 점이 9–23 개.
- XFeat + 삼각: 142610 은 모든 변형 11.2 → 4.1–6.7 cm, 142148 은 루프는 생기지만 3.1–6.4 cm (±, 1.4 cm 이하는 잡음 수준). B60t: 142610 6.22 (루프 3), 142148 루프 0.
- GL60 (integ2 4035d59 + 패치, aacfd992): B60 9.14 / 16.54 → L1kt 8.10 / 11.34, N1kt 8.86 / 11.24, L4kFt 6.94 / 13.55, N4kFt 6.36 / 12.02. **HS 는 GL60 위 미검증.**

## std60 165506 (녹화 1개 × 시작 오프셋 3, 독립 반복 아님; s173432 는 VIO 초기화 실패 → 점수 없음)
지도 ATE B60 [1.58, 1.46, 1.47] → L1k [1.32, 1.27, 1.41] (틀린 0/66), N1k [1.44, 1.32, 1.36] (0), 삼각 변형 약 1.25–1.55 (틀린 0–8 개, 사전 규칙).
변수 분리: XFeat 매칭만 = std 좋아짐·틀린 0 이지만 손 루프 못 닫음 / 삼각측량 = 손 루프 닫지만 경계·틀린 에지 늘어남. tri_deg 3 은 일관되게 낫지 않음.

## 폰 (카메라 없음, 잠금·온도 규칙: 시작 SKIN<36, 항목마다 기록). `phone/phone_r4.txt` (판정용), `phone_r3.txt` (1차 참고)
판정용 (list4, 23:41–23:51): 충전 중, 화면 켜짐 유지(충전 중 stay-on, Brave 화면에 떠 있었음, 마지막 입력 >5 분 규칙으로 시작), 절전 ON (사용자 설정, 그대로), SKIN 30.4–31.2, 핵심 항목 2회. VIO 동시 실행 없음.
- XFeat 백본 TFLite GPU (fp16 허용 기본): 640x352 9.2 / 8.9 ms, 960x544 p50 17.7 / 19.0 ms (평균 17.6 / 19.1, 반복 차 7–8 %). 충전·절전 상태는 실행 시작 때만 기록 (항목별 기록은 배터리·화면·온도). CPU 4 스레드 640x352 13.9 / 13.9 ms.
- **실측 키프레임 총합 (kf_bench, CPU 1 스레드: XFeat ORT + 후처리 + NN 4번)**: 640/512점 51 ms, 640/1k 75 ms, 640/2k 182 ms, 960/2k 225–229 ms, 960/4k 730–736 ms (4 스레드 960/2k 103 ms).
- **추정 (따로 잰 값의 합: GPU XFeat + 후처리 + NN 4번)**: 640/512 ≈ 23, 640/1k ≈ 47, 640/2k ≈ 153, 960/2k ≈ 159, 960/4k ≈ 665 ms (4 스레드 960/2k ≈ 78). 삼각측량·PnP 비용, VIO 동시 실행 빠짐.
- 정확도: 같은 흰 벽 프레임에서 ORT NNAPI-fp16 경로 (176 ms, NPU 아님) top-1024 겹침 99.8 %, 기술자 cos 최소 1.000; 폰 ORT CPU 100 %. **GPU 경로 정확도는 폰에서 미확인** (benchmark_model 출력 덤프 안 됨; PC TFLite fp16 가중치 최대 오차 0.02).
  → 정확도 확인된 주 숫자 = CPU 경로 640/1k **75 ms**.
- NN 매칭 N≈2250 이 4번이면 ~137 ms (1 스레드, 순진한 ORT MatMul+ArgMax) → 2k 점부터 예산 초과. 가장 무거운 150 ms 안 변형 = 640x352 / 1k 점 / NN (실측 75 ms).
- PC 시제품은 640x360 으로 줄여서 XFeat 안에서 640x352, 폰 모델은 640x352 고정. PC LighterGlue 는 kornia 기본 가지치기 켬, 폰 고정 모델은 6층 전부.

## 동일성·결정성 (`ident/ident.txt`)
기준 빌드(ovloop 0d71a86) vs 이 브랜치 B60: 9/9 poses+map+루프 로그 IDENTICAL. XFeat/삼각 실행의 VIO 필드 vs B60: 747/747 IDENTICAL. tri_one 리팩터 전후 루프 로그 129 IDENTICAL. GPU 재실행: 손 루프 2 × 2조건 IDENTICAL (HS 칸 결정성은 미확인).

## 출처
XFeat/LighterGlue: github.com/verlab/accelerated_features e92685f (Apache-2.0, 가중치 저장소 안; 학습 데이터 MegaDepth 조건은 가정). LightGlue 코드 = kornia 0.7.2 (Apache-2.0).
폰: ONNX Runtime Android 1.20.0 (Maven), TFLite benchmark_model nightly android_aarch64 (sha 5d45116e…), onnx2tf 1.26.3 / TF 2.17.1 (InstanceNorm 은 CPU 로, unfold 는 one-hot stride-8 conv 로 바꿔 변환, fp32 ≤1e-5 일치).

## 다음 (하려면)
1. 틀린 루프 원인 (삼각측량 3D 정확도, XFeat 1/8 해상도 위치, PnP 문턱) — 사후 조정은 새 사전 선언 후.  2. Samsung ENN SDK 로 NPU.  3. NN 매칭 fp16/GPU 행렬곱.  4. GL60 위 HS.

## 파일
`ovxfeat_run.py` (평가), `xf_server.py`, `summary.py` → `summary.json` (L60), `summary_ate_gl60.json`, `score.json`, `edges.py`, `cand_dist.py`, `ident_check.py`, `export_*.py`, `phone/` (ort_bench, kf_bench, phone_bench.sh, list3/4, parse_phone.py, acc_compare.py, 결과 json).
