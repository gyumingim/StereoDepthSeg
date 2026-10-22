# ZUPT 수정 A' 시험 — 사전 등록 (2026-10-22)

이 파일은 A' 를 한 번도 돌리기 전에 커밋한다.
- 판정 기준·사건 정의·녹화·칸은 `docs/zupt_fixA_plan.md` 의 것을 **바꾸지 않고** 쓴다.
- 아래 "판정 담당 시험 계획" 절은 그 파일(= `runs/zupt_slam_review/judge.md`)의 절을 글자 그대로 옮겼다.
- 계획 안의 "A" 는 이번 시험에서 "A'" 로 읽는다. 바꿔 읽는 곳: "모두 만족해야 A 채택", 기준 4 "A에서는 …", 판정 규칙 "A를 채택".
- "구현 세부 (A 계획)" 절도 그대로 적용한다. 다만 "A' 에서 달라지는 것" 절이 앞선다.
- 참고 기준은 A (`runs/zupt_fixA/summary.json`) 와 C (`runs/zupt_fixC/summary.json`) 둘이다. 같은 표에 나란히 적되, 판정에는 쓰지 않는다.

## A' 는 무엇이고 왜 하나

- A' = A (ZUPT disparity 기준점 = 가장 늦은 클론 = 마지막 일반 갱신) + C 의 시작 정지 면제.
  - 첫 일반 갱신 전 (`has_moved_since_zupt == false`) 에는 base 와 같이 바로 앞 프레임과 비교한다.
  - 상한(C)은 넣지 않는다. D 도 넣지 않는다. 변형은 하나만 돌린다.
- 근거 (`docs/zupt_fixA_result.md`, `runs/zupt_fixC/diag_A.md`, `docs/zupt_fixC_result.md`):
  - 표적 133820 을 살린 것은 A 뿐이다 (C 는 389–444 cm).
  - A 의 부작용 중 213955 o1 의 시작 정지 변화 (첫 추적 11.5 → 3.0 s) 는 C 의 면제 방식으로 막을 수 있음을 보였다 (C 에서 첫 추적 시각이 75/75 같음).
- 미리 적는 기대 (추정):
  - 154320 o0 는 첫 일반 갱신 (2.14 s) 이 base 와 A 에서 같다. 그 뒤 구간에서는 A' = A 다. 그래서 A' 도 시작 7 s 의 짧은 ZUPT 연속을 쪼갤 것이고, A 처럼 붕괴할 것으로 예상한다.
  - 214550 o1 은 첫 일반 갱신이 2.65 s 다. 그래서 A' 는 A 와 거의 같을 것으로 예상한다.
  - 따라서 기준 2·3 은 실패할 가능성이 높다. 그래도 하는 이유: 시작 정지 효과를 A 에서 분리해, 213955 계열과 나머지 회귀를 가르기 위해서다.

## A' 에서 달라지는 것 (구현 세부)
- 코드 `tools/ov_patches/zuptAp.diff` (= `open_vins_zbase` ↔ `open_vins_zuptAp`):
  - `UpdaterZeroVelocity` 에 공개 멤버 `zuptAp_anchor` 를 둔다. `VioManager.cpp:295` 의 ZUPT 블록이 `try_update` 직전에 여기에 `has_moved_since_zupt` 를 넣는다.
  - disparity 기준 시각 = `zuptAp_anchor && 클론 있음` 이면 가장 늦은 클론 시각 (A 와 같음), 아니면 `state->_timestamp` (base 와 같음).
- 빌드: `~/scanner/src/open_vins_zuptAp`, `ovbuild_zuptAp`, 브리지 `tools/ov_bridge_zuptAp`. 방식은 A·C 와 같다.
- 러너 `imu_learn/zupt_fixAp.py` (= `zupt_fixC.py` 에서 조건 이름과 경로만 바꿈):
  - 칸 75 개, 인자는 같다.
  - base 는 `runs/zupt_fixA` 의 것을 다시 쓴다. 먼저 base 2 개 (`BR_134204_o0`, `I3_215348_o1`) 를 다시 돌려 비트 단위로 같은지 본다. 다르면 멈춘다.
  - 동시 실행 ≤ 5, threads 2.
- 분석 `imu_learn/zupt_fixAp_analyse.py` 와 독립 검사 `imu_learn/zupt_fixAp_check.py`: C 판의 복사본이다. 조건 이름과 경로만 바꿨고, 기준은 그대로다.

## 보고만 하는 항목 (판정 규칙을 바꾸지 않음)
`imu_learn/zupt_fixAp_extra.py` → `runs/zupt_fixAp/extra.json`. base / A / C / A' 모두에 대해 계산하고, 결과 문서에 표로만 적는다.

1. **1 s 창 전멸 (`wipe1s`).** 계획의 SLAM 전멸 정의에서 창만 0.2 s → 1.0 s 로 넓힌 것이다. C 에서 133820 의 느린 SLAM 소멸 (0.7 s 동안 44 → 0) 을 0.2 s 창이 놓쳤기 때문에 넣는다.
2. **Sim3 ATE.** 벤치의 `ate_sim3_cm` 이다.
3. **1 s 상대 이동 오차 (`rel1s_cm`).** 0.1 s 격자에서 |VIO 1 s 이동 거리 − GT 1 s 이동 거리| 의 평균 (cm) 이다. 좌표 정렬이 필요 없다. 214550 처럼 SE3 정렬이 불안정한 녹화를 보조로 보기 위해 넣는다.

## 사전 등록 진단: 154320 o0 (보고만 함)
- 구간: 첫 추적 포즈 ~ 7.5 s. 이 녹화는 이 구간에 SLAM/MSCKF 특징이 없다.
- 잴 것: ZUPT 수락 프레임 수, 가장 긴 연속 (프레임), 연속 길이 중앙값.
- "쪼갬" 의 정의: 가장 긴 연속 ≤ 3 프레임 **그리고** 수락 수 ≤ base 의 0.8 배.
- 예측 규칙: 쪼갬 → 이 run 은 붕괴 (계획의 run 붕괴 정의). 쪼개지 않음 → 붕괴 아님.
- 이미 있는 값 (실행 전에 잼, `zupt_fixAp_extra.one`):

  | 조건 | 수락 | 최장 연속 | 쪼갬 | 붕괴 | 예측 맞음 |
  |---|---|---|---|---|---|
  | base | 53 | 15 | 아님 | 아님 (11.1 cm) | 맞음 |
  | A | 36 | 2 | 쪼갬 | 붕괴 (501 cm) | 맞음 |
  | C | 53 | 15 | 아님 | ATE 2.1 cm (규칙상 440 s 전멸로 붕괴 분류) | — |

- A' 의 예측 (실행 전): 쪼갬 → 붕괴. 결과 문서에 이 예측이 맞았는지 적는다.
- 표본은 run 1 개뿐이다. 맞아도 인과의 증거로는 약하다.

---

## 판정 담당 시험 계획 (judge.md 에서 그대로 옮김)

## 사전 등록 시험 계획 (돌리기 전에 고정)

**같은 조건**
- 같은 빌드(`open_vins_diag` + 수정 1개), 같은 설정 파일, 같은 채점 스크립트를 쓴다. 기준선(수정 없음)도 **같은 날 같은 빌드 트리에서 다시** 돌린다. 옛 수치와 섞지 않는다.
- 한 번에 바꾸는 것은 하나. A와 C를 같이 바꾼 결과는 결론 근거로 쓰지 않는다.
- 각 녹화는 o0/o1/o2 3회 모두 돌린다. 판정은 3/3 기준이다.

**녹화**

| 묶음 | 녹화 | 역할 |
|---|---|---|
| 표적 | hs_20261022_133820 o0–o2 | 붕괴 사례, 막아야 함 |
| 표적 | hs_20261022_134550 o0–o2 | o2 붕괴, o0/o1 전멸 후 회복 |
| 대조 | hs_20261022_134204 o0–o2 | 중간 정지 없음, 멀쩡해야 함 |
| 회귀 | 214550 base o0–o2 | 긴 정지(ZUPT 이득)와 87 s 반정지 |
| 회귀 | 이전 i3/G1.0 비교에 쓴 나머지 녹화 전부 | 다른 곳에서 새 붕괴가 없는지 |

**사건 정의 (로그 기준)**
- 전부 거절 갱신: SLAM `in ≥ 10` 이고 `rej/in ≥ 0.9`인 일반 갱신 프레임.
- SLAM 전멸: SLAM 개수가 0.2 s 안에 20개 이상에서 3개 이하로 떨어짐.
- 붕괴: 전멸 뒤 5 s 안의 최대 이동이 0.5 m를 넘거나, 해당 run의 ATE가 50 cm를 넘음.

**통과 기준 (모두 만족해야 A 채택)**
1. 막기: 133820 o0–o2와 134550 o0–o2에서 전부 거절 갱신 0회, SLAM 전멸 0회. 133820 각 run의 ATE ≤ 10 cm(기준선 280–386 cm). 134550 o2 ATE ≤ 5 cm(기준선 18.5 cm). 10 cm와 5 cm는 134204 수준(1.1–1.3 cm)에 여유를 둔 값이며 근거는 추정이다.
2. 새 붕괴 없음: 회귀 묶음 모든 run에서 기준선에 없던 전멸이나 붕괴가 0회.
3. 회귀 없음:
   - 134204 각 run ATE ≤ 기준선 × 1.2 + 0.5 cm.
   - 214550 각 run ATE ≤ 기준선 × 1.1(현재 22/18/21 cm).
   - GT 속도 < 0.05 cm/s인 진짜 정지 구간의 ZUPT 수락 프레임 수 ≥ 기준선의 90 % (정지 이득 유지).
4. 기전 확인(반증 가능):
   - A에서는 표적 반정지 구간 안에서 ZUPT가 0.72 s보다 일찍 끊겨야 한다(일반 갱신이 끼어듦).
   - ZUPT 틈이 사라졌는데도 전멸이 남으면 "ZUPT 중 자세 고정" 가설은 기각한다. 그때는 다른 원인(예: 반정지 중 삼각측량 실패와 특징 교체)을 조사한다.
   - 반대로 ZUPT가 끊겼는데 전멸만 사라지고 ATE가 여전히 크면, 붕괴의 주원인은 MSCKF 부재 쪽으로 판정한다.

**판정 규칙**
- 1–4를 모두 만족하면 A를 채택한다.
- 1은 만족하고 2·3 중 하나가 실패하면 C로 같은 시험을 반복한다.
- 1이 실패하면 기전 재조사이며, D 추가는 그 뒤에 한다.
- 결과 파일에는 만든 시각, 빌드 해시, 설정 해시, 녹화 목록을 남긴다.

---

## 구현 세부 (A 계획, 그대로 적용 — 위 "A' 에서 달라지는 것" 이 앞섬)

### 빌드 (기존 `open_vins_diag` / `ovbuild_diag` 는 건드리지 않음)
- 소스 복사 2벌: `~/scanner/src/open_vins_zbase` (수정 없음), `~/scanner/src/open_vins_zuptA` (수정 A 한 개).
  둘 다 `cp -a open_vins_diag` 로 만든다.
- 빌드 디렉터리 `ovbuild_zbase`, `ovbuild_zuptA`. `ovbuild_diag` 의 CMakeCache 와 같은 옵션:
  `cmake -S <src>/ov_msckf -B <build> -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=ON
  -DCMAKE_PREFIX_PATH=$HOME/scanner/local` (컴파일러 `/usr/lib/ccache/c++`, GNU 11.4), `cmake --build <build> --target ov_msckf_lib -j4`.
- 브리지 복사 2벌: `tools/ov_bridge_zbase`, `tools/ov_bridge_zuptA` (`main.cpp` 는 `tools/ov_bridge_diag/main.cpp` 와 같은 바이트,
  `CMakeLists.txt` 는 프로젝트 이름과 `OV_SRC`/`OV_BUILD` 기본값만 다름 — `tools/ov_bridge_g10` 와 같은 방식).
- 수정 내용은 `tools/ov_patches/zuptA.diff` (zbase ↔ zuptA 소스 diff) 로 저장소에 남긴다.
  구현: `time0_cam` = `state->_clones_IMU` 의 가장 늦은 클론 시각 (ZUPT 수락 프레임은 클론을 만들지 않으므로 이것이
  "ZUPT 연속 구간 전 마지막 일반 갱신 시각"이다. 연속 구간 첫 ZUPT 프레임에서는 지금 코드의 `state->_timestamp` 와 같은 값).
  클론이 없으면 지금처럼 `state->_timestamp`.
- 기준선 동일성 검사: zbase 브리지로 `i3_confirm` 215348 base o1 과 `board_rotation` 134204 base o0 를 같은 인자로 다시 돌려
  포즈 스트림(`poses.jsonl.gz` 의 `t`, `status`, `p`, `q`, `v`, `T_world_cam0`)과 `ate_cm` 이 옛 결과와 **완전히 같은지** 본다.
  하나라도 다르면 여기서 멈추고 보고한다 (본 실행을 시작하지 않음).

### 실행
- 러너 `imu_learn/zupt_fixA.py` (`run` / `job` / `analyse`). `job` 은 `c5_run` 을 그대로 쓰고 `c5_run.DIAG_BRIDGE` 만
  조건별 브리지로 바꾼다 (`g10.py` 와 같은 방식). 공통 설정 = i3_confirm base: `zr.common(rec, off, lead, max_secs, raw, rebias=False)`
  (static init strict 1.0 s / 15 px, clones 11, noise cfg, 640x360, 60 fps, decode threads 2, `--no-gyro-rebias`),
  `num_opencv_threads=2`, still-hold 끔, `a.gyro_rebias = False` 를 job 에서도 명시. 동시 실행 ≤ 5, `nice -n 5`.
- 조건: `base` (zbase 브리지) / `A` (zuptA 브리지). 같은 run 이름 쌍: `<cell>_base`, `<cell>_A`.
- 칸 (cell) 목록 — 녹화별 시작 오프셋은 원래 비교와 같게:
  - 표적·대조: 133820, 134550, 134204 o0–o2 = `board_rotation.py` 오프셋 0/1/2 s, 전체 길이, raw IMU, lead 2.0.
  - 회귀 i3 묶음: 213341, 213955, 214550, 215348 o0–o2 = `runs/i3_confirm/inputs/<rec>.json` 의 `offsets_s`, 전체 길이, raw, lead 2.0.
  - 회귀 10-20/10-19 묶음 = `z2_regress.jobs()` 의 base 칸 전부 (G1.0 회귀 R 칸의 상위 집합): F (6 녹화 × o0–o2, 전체 길이, raw),
    T (5 녹화 × S1/S2 × d−0.25/0/+0.25, 60 s, lead 0, raw), O (131326, 154320 × o0–o2, 전체 길이, **raw 아님** — z2_regress 그대로).
  - 빠진 것: i3_takeoff/G1.0 의 B·N 칸 (같은 녹화의 짧은 클립 변형). 녹화 자체는 위 전체 길이 run 으로 모두 덮는다.
- 순서: 동일성 검사 2개 → (통과 시) 표적·대조·214550 의 base/A → 나머지 회귀. 출력 `runs/zupt_fixA/`, 로그 `runs/zupt_fixA.log`.
- 각 결과 json 에 만든 시각, git HEAD, 러너/벤치 blob, 브리지·라이브러리 sha256 (c5_run 이 기록), 설정 파일 sha256 (c5_run 이 기록) 을 남긴다.

### 로그에서 사건 읽는 법 (`<run>.stderr.log.gz`, ANSI 색 코드 제거 후)
- 프레임 = `[DIAGT] t` 줄부터 다음 `[DIAGT]` 전까지. `t` 는 클립 시각 (포즈의 `t` 와 같음).
- ZUPT 수락 프레임 = 그 안에 `[ZUPT]: accepted` 줄이 있음.
- 일반 갱신 프레임 = ZUPT 수락이 아니고 `seconds for SLAM update (N feats)` 줄이 있는 프레임. SLAM 개수 = N.
  같은 프레임의 `[CHI2] slam in X rej Y` 줄은 모두 더한다 (in = ΣX, rej = ΣY).
- 전부 거절 갱신: 일반 갱신 프레임 중 in ≥ 10 이고 rej/in ≥ 0.9.
- SLAM 전멸: 일반 갱신 프레임 j 에서 N_j ≤ 3 이고, t_j − 0.2 s ≤ t_i < t_j 인 일반 갱신 프레임 i 중 N_i ≥ 20 인 것이 있음.
  한 번 센 뒤에는 N 이 다시 20 이상이 될 때까지 다음 전멸을 세지 않는다 (한 사건 = 한 번).
- 붕괴 (전멸별): 전멸 시각 t 부터 t+5 s 사이 추적 포즈 위치 `p` 의 첫 포즈 대비 최대 거리 > 0.5 m.
  run 붕괴 = 전멸별 붕괴가 하나라도 있거나 `ate_cm` > 50. 결과가 없는 run (오류·ATE 없음) 은 따로 "결과 없음" 으로 적고,
  기준선에 결과가 있는데 A 에 없으면 기준 2 실패로 본다.
- 기준 2 의 "기준선에 없던": 같은 칸의 base run 에 ±1.0 s 안의 전멸이 없는 A 의 전멸 = 새 전멸. base 붕괴 아님 → A 붕괴 = 새 붕괴.
  기준 2 는 표적 6 run 을 뺀 모든 run (대조 134204 포함) 에 적용한다.
- 기준 3 의 진짜 정지 ZUPT 수 (run 별): ZUPT 수락 프레임 중 GT 속도 < 0.05 cm/s 인 것의 수.
  GT 속도 = `recordings/<rec>/imu_gt_spl/gt.npz` (ok & |p_bc| < 1.2) 의 카메라 중심 c = −R_bcᵀ p_bc 로
  |c(t+0.25) − c(t−0.25)| / 0.5 s. 두 끝의 GT 표본이 각각 0.02 s 안에 있어야 하고, 아니면 그 프레임은 셈에서 뺀다.
  GT 시각 → 클립 시각: `(t_ns_gt − (pose0.t_ns − pose0.t·1e9 + align_dt_ms·1e6)) / 1e9`.
  판정: 기준선 수가 30 이상인 run 마다 A ≥ 0.9 × 기준선. 30 미만이거나 GT 가 없는 run 은 목록만 적는다.
- 기준 4 의 "표적 반정지 구간": base run 의 ZUPT 수락 연속 구간을 0.3 s 이하 틈으로 이어 붙인 정지 구간 중 길이 ≥ 1 s 이고
  그 안 또는 끝 뒤 2 s 안에 base 의 SLAM 전멸이 있는 구간 [a, b]. A run 의 [a, b] 안에서 연속한 두 일반 갱신 프레임 사이 시간
  (구간 경계 앞뒤의 가장 가까운 일반 갱신 포함) 의 최댓값 < 0.72 s 이면 통과. 6 run 모두 통과해야 기준 4 첫 줄 통과.
  둘째·셋째 줄은 해석 규칙이므로 숫자 (전멸 수, ATE) 와 함께 문장으로 판정한다.
- 기준 1 ATE 는 벤치 `ate_cm` (기존 표와 같은 값). 214550 / 134204 의 기준선 = **이번에 다시 돌린 base** (옛 22/18/21 cm 아님).
- 추가 기록 (판정에 안 씀): ZUPT 수락 총 수, `[ZUPT]: passed disparity (… N features)` 의 특징 수가 20 이하인 프레임 수
  (judge.md 주의: 기준 시각 특징 < 20 이면 disparity 검사가 꺼짐 — 214550 긴 정지에서 확인).

### 분석
- `imu_learn/zupt_fixA_analyse.py` → `runs/zupt_fixA/summary.json`.
- 독립 검사: `imu_learn/zupt_fixA_check.py` (분석 스크립트를 import 하지 않음, 다른 담당이 이 계획만 보고 작성) → `runs/zupt_fixA/check.json`.
  두 결과의 기준 1–4 판정과 핵심 수치가 다르면 결과 문서에 둘 다 적고 원인을 밝힌다.
- 결과 문서 `docs/zupt_fixA_result.md`.
