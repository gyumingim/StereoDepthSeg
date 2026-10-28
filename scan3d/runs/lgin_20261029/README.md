# lgin 2026-10-29 — LighterGlue 입력 고르기: (A) XFeat 512 점만 vs (B) OpenVINS 추적점 + XFeat (PC, 라이브 경로)

브랜치 `lgin-20261029` (WSL `~/scanner/StereoDepthSeg-lgin`, npunight-20261029 6f6e69e 위). PC 만 (폰·adb 안 씀).
배경: `audit_pipefit_20261028.md` 1·3번 (NPU LG 는 XFeat 512 만, PC 설계 FLGc 는 추적점 + XFeat ≈ 674 점), 앱 0.6.36 의 기본 `npu_lg_in x512`.
**이 README 의 "사전 등록" 절은 본 실행(main) 전에 커밋한다.** 결과 절은 실행 뒤 추가.

## 사전 등록 (본 실행 전 커밋)

### 비교하는 것 (한 가지만 바뀜)
| 조건 | LG 입력 | 서버 옵션 (`xf_server_lgin.py`) |
|---|---|---|
| **A** | XFeat 512 점만 (= 앱 `npu_lg_in x512`, `xfnpu_core.h` pick) | `--lg-in x512 --pad32 1` |
| **B** | 추적점(쿼리) + XFeat 512, 모든 점 (= FLGc 입력, 중앙 ≈ 674 점) | `--lg-in trk --pad32 1` |

공통 (A·B 같음):
- 다리 = `~/scanner/tmp_appnpu/ident/ov_bridge_new` (npunight PC 기준과 같은 PC 빌드), `--profile rec60 --drop-late 0 --loop 1`,
  루프 문 = 앱 `NpuLoop.BRIDGE_CFG` 그대로 (FLGc: `xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1`) + `log_cand=1` (기록만).
- 입력 = 폰 c4f60 설정 폴더 (`~/scanner/tmp_lens05/new_gen/c4f60`, 960x540 왜곡 편 영상, 다리 안 remap) = 라이브 경로. 재생 = `appnpu_replay.py` (npunight `mkstream.py` 와 같은 방법, tee 없음).
- 서버 = PC fp32 (CUDA, 결정적 설정), XFeat 공식 후처리 top-512, 문턱 0.05, 960x540 → 아래 4 줄 복제 960x544 (앱 x960 모델 입력과 같은 크기), y ≥ 540 점 버림.
  추적점 기술자 = 같은 dense 맵 bicubic. 상호 NN (삼각측량용, mode 1) 은 두 조건 모두 [추적점 + XFeat] 전체 (앱 `nn()` 과 같음), cos ≥ 0.82.
- LG = kornia LighterGlue fp32 6 층, **가지치기 끔 · 조기 종료 끔** (NPU 고정 그래프와 같게; FLGc 의 가지치기와는 다름), 짝 문턱 0.1.

### 데이터 (GT 있는 녹화, npunight PC 기준과 같은 창)
| 녹화 | 창 | 초점 | 비고 |
|---|---|---|---|
| hs_20261024_165506_std60 | 전체 28 s | **0.5 dpt** (std60, 60 fps 녹화) | 라이브 설정과 같음 |
| hs_20261021_213341 | 처음 40 s | **HS 2.22 dpt** (240 fps → 60 fps) | 폰 npunight 에도 씀 |
| hs_20261021_214550 | 40 s | HS 2.22 | 폰 npunight |
| hs_20261021_215348 | 30 s | HS 2.22 | PC 만 (BRIEF 루프 0) |
| hs_20261020_221925 | 30 s | HS 2.22 | 폰 npunight |
| hs_20261020_222747 | 30 s | HS 2.22 | PC 만 |
| hs_20261020_223815 | 30 s | HS 2.22 | 폰 npunight |
| hs_20261020_224323 | 30 s | HS 2.22 | PC 만 |
- 0.5 dpt GT 녹화는 165506 하나뿐 (173432 는 VIO 초기화 안 됨, npunight). HS 녹화는 0.5 dpt remap 으로 들어감 → 내부 값 약 1.4 % 차이 (**가정**, npunight 와 같음).
- 빠진 GT 녹화: 173432 (VIO 실패), 213955 (발산), 짧은 녹화들 (hs_20261022_* 등, 창 < 30 s 또는 npunight 기준 없음).

### 실행 순서 (`lgin_run.py run main`)
- 녹화마다 차례로: sync 0 A1 → B1 → B2 → A2 (ABBA, `--loop-sync 0` = 앱과 같은 비동기 루프 스레드), 그 뒤 sync 1 A, B (결정적 기준 1 회씩). 녹화 4 개씩 병렬.

### 지표 (채점 = npunight `score_rep.py pc` 그대로 + `cand_stats.py`)
- **1 차 = 지도(루프 보정) 포즈 ATE RMSE (cm)**, sync 0 run.
- 보조: VIO ATE, 끝점 오차, 채택 루프 수, 맞는 루프 (GT 카메라 중심 < 0.5 m · 광축 < 30°) 수·비율, 틀린 루프 수,
  후보 단계 (`cand_stats.py`): LG 로 간 후보 수, GT 재방문 후보 수, 기하 검증 통과 수와 정밀도(통과 중 GT 재방문 비율)·재현율,
  채택 루프의 상대 포즈 오차 (회전 deg · 이동 cm, GT 축 규칙 **가정**; 맞는 루프 회전 오차 중앙 > 10° 이면 "규칙 의심" 으로 표시만), LG 점 수·PC 시간.

### 유효 run
exit 0, `.score.json` 에 지도 ATE 있음, loop.jsonl 에 xf_down 없음, 서버 살아 있음. `keyframes pushed … dropped …` 는 기록 (sync 0 에서 버림이 있으면 표에 적음, 무효 아님).

### 판정 규칙
1. 녹화마다 (sync 0 네 run 모두 유효일 때만): A 두 값이 B 두 값보다 모두 작으면 "A 쪽이 낫다", 모두 크면 "B 쪽이 낫다", 겹치면 "차이 없음".
2. 전체: 한쪽이 녹화 2 개 이상이고 반대쪽이 0 개면 그쪽, 아니면 "차이 보이지 않음" (npunight 규칙과 같음).
3. 안전 검사: main 의 모든 run (sync 0 + 1) 에서 GT 로 틀린 채택 루프 수 합. B 가 A 보다 많으면 B 는 고르지 않는다 (반대도 같음).
4. **선택**: 2 가 "B" 이고 3 을 통과할 때만 B. 그 밖에는 **A** (앱 현재 기본, NPU 모델 L6_a16mc 가 이미 있음, 512 점이라 싸다;
   B 용 마스크 768 모델은 lgpost 에서 eais 변환 실패 → 아직 없음). 2 가 "A" 이고 3 에서 A 가 더 많으면 "A 지만 틀린 루프 주의" 로 적는다.
- 반복이 거의 결정적이라 녹화당 표본은 사실상 1 개 → 결과는 **추정**. 섞인 변수: 없음 (LG 입력만 다름). 단 sync 0 은 스레드 타이밍에 따라 run 끼리 달라질 수 있음.

### 탐색 (판정에 안 씀, `lgin_run.py run expl`, main 뒤)
- `Bm` = 앱 `trk768` 마스크 모델 방식 (추적점 먼저, 최대 768, 양쪽을 min(nA, nB) 점으로 자름), sync 1.
- `A640` / `B640` = FLGc 의 XFeat 해상도 640x360 에서 같은 A/B, sync 1.
- HS 녹화 7 개 **전체 길이** (약 137–219 s) A/B sync 1 — 더 긴 재방문.
- 재방문 거리 목록 (`revisit_list.py`): 기존 PC 실행의 VIO 포즈로 녹화별 크기·가장 긴 재방문 경로.

### 실행 전에 본 것 (정직하게)
- 배관 시험 1 회: 165506 처음 12 s, A·B sync 1 각 1 회 (`~/scanner/tmp_lgin_smoke`) — 서버·다리 연결, LG 점 수 (A 512/512, B 651/696), nk 512 만 확인. ATE·루프 값은 보지 않음.
- npunight 결과 (BRIEF vs NPU x512 = 사실상 A, 5 녹화 "차이 보이지 않음") 와 lgopt FLGc (B, 개발 분할) 결과는 이미 알고 있음.

## 파일
`xf_server_lgin.py` (lgopt `xf_server.py` + `--lg-in/--pad32/--lg-cap`), `appnpu_replay.py` (appnpu 사본), `lgin_run.py` (작업·관문), `cand_stats.py` (후보·RPE).
원자료 WSL `~/scanner/tmp_lgin/<조건>/<녹화>_w<창>_s<sync>_r<반복>.*`.
