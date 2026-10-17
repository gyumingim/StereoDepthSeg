# ChArUco 보드로 축척 확인·보정 (aruco_scale)

벽에 크기를 아는 ChArUco 보드를 붙여 두면, 보드가 보이는 키프레임마다 "정답 깊이"가 생긴다.
그걸로 아래를 한다.

1. **깊이 축척 확인·보정**: FFS 시차가 전체적으로 밀리는 양(오프셋 Δd)과 비례(s)를 재서 고친다.
   (실측: 3분 사이 시차가 +0.08~0.2 px 밀려 깊이가 약 1.5 % 변함, 열 때문으로 추정)
2. **두 렌즈 자세 잔차 (정보)**: 정류 오른쪽 영상(rect1)에서도 보드가 보이면 yaw·pitch·roll 잔차를 잰다.
3. **VIO 축척 확인 (정보)**: 보드 기준 카메라 이동 거리 vs VIO 이동 거리의 비.
4. **위치 기준점 (정보)**: 보드의 월드 위치가 처음보다 얼마나 떠내려갔는지 (VIO 드리프트).

렌즈 내부값(초점거리·왜곡) 전체 캘리는 하지 않는다.

## 먼저 고칠 것: lib_ffs.infer 축소 버그 (이번 시험에서 발견)

합성 시험에서 FFS 시차가 **화면 가로 위치에 따라 0.06 → 0.71 px** 커지는 치우침이 나왔다 (1440x1080, 기본 축소 0.444).
원인: `lib_ffs.infer` 가 왼쪽 영상은 `cv2.resize(l, None, fx=scale, fy=scale)` (실제 비율 = scale 0.444),
오른쪽 영상은 `cv2.resize(r, (폭, 높이))` (실제 비율 = 639/1440 = 0.44375) 로 줄여서 두 영상의 화소 위치가
가로 위치에 비례해 어긋난다. 1440·scale 이 정수가 아닐 때마다 생긴다.

| 축소 scale (FFS 입력 폭) | 가로 위치별 시차 치우침 px (x = 60 / 400 / 700 / 1000 / 1320) |
|---|---|
| 0.444 (639, 지금 기본) | +0.06 / +0.22 / +0.38 / +0.56 / +0.71 |
| 0.4445 (640) | +0.02 / +0.06 / +0.08 / +0.13 / +0.15 |
| 0.5, 0.6 | 전체 중앙 −0.04 ~ +0.02 (치우침 거의 없음) |

1 m 에서 시차는 약 16 px 이므로 0.7 px = 깊이 4 % (화면 오른쪽일수록 가깝게 나옴).
**고치는 법 (lib_ffs.py, 통합 담당)**: 왼쪽도 크기를 직접 준다.
`Wd, Hd = round(W0*scale), round(H0*scale); l = cv2.resize(l, (Wd, Hd)); r = cv2.resize(r, (Wd, Hd))`.
보드 보정은 전체 오프셋·비례만 고치므로 이 가로 방향 치우침은 못 고친다 → 이걸 먼저 고친 뒤 보드 확인을 할 것.

## 1. 보드 인쇄·붙이기

### 새로 인쇄 (권장: 1.5 m 까지 쓰려면)
```
./venv_ffs/bin/python scan3d/tools/make_charuco.py            # A4 가로, DICT_4X4_50, 7x5 칸, 칸 32 mm
./venv_ffs/bin/python scan3d/tools/make_charuco.py --paper A3 # A3, 칸 50 mm (2 m 까지)
```
- 결과: `scan3d/config/charuco/charuco_A4.pdf/png`, `board_A4.json`.
- 인쇄 대화상자에서 **"실제 크기 100 %"**, "용지에 맞춤" 끄기.
- 칸이 클수록 멀리서 잡힌다. 1440x1080 합성 시험: A4 4X4 (칸 32 mm) 는 1.5 m 정면까지, 2 m 는 안 됨.

### 이미 인쇄한 보드 (사용자 것: A4 세로, 5x7 칸, 마커 0.6, "용지에 맞춤")
- 그림 `charucoboard.png` (1160x1560 px, 칸 200 px, 여백 80 px) 는 **DICT_6X6_250** 로 확인됨 (모서리 24개 모두 검출).
- 명세 파일: `scan3d/config/charuco/board_user_A4_6x6.json` (칸 크기 의도 값 34.5 mm 는 추정, 꼭 재서 넣을 것).
- 6X6 마커가 칸의 0.6 이라 마커 한 칸(비트)이 작다 → 합성 시험에서 **1 m 까지만** 잘 잡힘 (1.5 m 는 검출 0).
  먼 거리도 쓰려면 위의 4X4 보드를 하나 더 인쇄할 것.

### 붙이기
- 평평하고 단단한 판(폼보드, 아크릴, 두꺼운 박스)에 붙인 뒤 벽에. 종이가 휘거나 울면 안 된다.
- 반짝이지 않게 (형광등 반사가 칸을 가리면 검출이 안 됨).
- 마커 id 가 같은 다른 보드(실내 드론 DICT_4X4_50 마커 등)가 같은 화면에 있지 않게.

## 2. 실제 칸 크기 재기

축척 확인이 믿는 건 "보드 실제 크기" 하나다 (초점거리 오차와는 거의 무관: d_true ≈ B·보드 화소 크기 / 보드 실제 크기).
그래서 이 값이 0.5 % 틀리면 보정도 0.5 % 틀린다.

### 자로 재기
- 위쪽 눈금 0 ~ 7 (보드 전체 가로), 왼쪽 눈금 0 ~ 5 (세로) 를 0.5 mm 까지 재서
  `board_A4.json` 의 `measured_width_mm`, `measured_height_mm` 에 넣는다 (또는 칸 1개를 `square_mm` 에).

### 자 없이: 사진 1장으로 (A4 종이 또는 카드 기준)
```
./venv_ffs/bin/python scan3d/tools/make_charuco.py --measure 사진.jpg --board scan3d/config/charuco/board_user_A4_6x6.json
./venv_ffs/bin/python scan3d/tools/make_charuco.py --measure a.jpg b.jpg c.jpg --ref both --board ... --write
```
- 원리: 종이 네 모서리(또는 카드 네 모서리)를 찾아 사진 → 실제 평면(mm) 변환을 만들고, 보드 모서리를 mm 로 옮겨 칸 크기를 맞춘다.
  모서리는 변마다 밝기 기울기 최대점으로 직선을 맞춰 서브화소로 구한다.
- 찍는 법:
  - 보드가 인쇄된 **A4 종이를 벽에서 떼어** (아직 판에 안 붙였으면) 종이보다 **어두운 바닥**(책상, 어두운 천) 위에 평평하게.
    종이 네 모서리가 모두 보이고, 손가락·그림자가 모서리를 가리지 않게.
  - 카드 기준(`--ref card`): 신용·교통 카드 (85.60 x 53.98 mm) 를 **종이 옆, 같은 바닥 위**에. 종이 위에 올리면 카드 두께
    0.76 mm 만큼 커 보인다 (40 cm 에서 약 +0.2 %).
  - 폰을 40 cm 쯤 위에서 거의 수직으로. **2배 망원(렌즈 왜곡 적음)** 을 쓰고 보드를 화면 가운데에. 3장 이상 찍어 중앙값.
  - 판에 이미 붙였으면: 판째로 바닥에 놓고 같은 방법 (종이 가장자리가 판보다 밝아야 함).
- 결과: 칸 크기 mm, 가로/세로 칸 크기 차이(프린터 비율 차이), 잔차. `--write` 면 json `square_mm` 에 기록.
  확인 그림 `<사진>_measure.jpg` (빨강 = 종이, 파랑 = 카드, 초록 = 보드 모서리).
- 합성 시험 정확도 (4000x3000 사진, 기울기 0~30°, `eval_aruco_scale.py measure-synth`):

| 기준 | 왜곡 없음 | 방사 왜곡 k1 = −0.03 (광각 폰 정도) |
|---|---|---|
| A4 종이 | 칸 오차 0.01 ~ 0.04 % | 0.16 ~ 0.24 % |
| 카드 | 0.10 ~ 0.28 % | 0.37 ~ 0.82 % |

  실제는 종이 크기 오차(보통 ±0.5 mm 안팎으로 추정 = ±0.2 %, ISO 216 허용은 ±2 mm)가 더 들어간다.
  A4 와 카드를 둘 다 찍어(`--ref both`) 두 값이 0.3 % 안으로 맞으면 믿어도 된다.

## 3. 찍는 법 (녹화)

- 보드를 **여러 거리 (0.5 / 1 / 1.5 m)** 와 **각도 (정면, 좌우 20~40°)** 에서 **각각 2~3 초씩 폰을 거의 멈추고**.
  - 렌즈 2·5 는 약 30 ms 어긋나 찍혀 움직이면 움직임 보정 정류의 남은 오차가 Δd 에 섞인다 → 멈춘 순간이 중요.
  - 거리가 여러 개여야 오프셋과 비례를 가를 수 있다 (한 거리면 오프셋만).
- 보드가 화면의 1/6 이상 크기로 보이게. 화면 가운데·가장자리 둘 다.
- 열 드리프트 확인: 스캔 시작 때 한 번, 3분 뒤 한 번 보드를 다시 비춘다.
- 처리:
```
./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py rec scan3d/recordings/rec_YYYYMMDD_HHMMSS \
    --board scan3d/config/charuco/board_A4.json            # FFS 포함 (GPU)
... --no-ffs                                               # 자세·렌즈 자세 잔차·VIO 축척만 (GPU 안 씀)
```
  VIO 축척 비는 녹화 폴더 안 replay 결과 `recon_*/vio_poses.json` 이 있을 때만 (먼저 replay 를 한 번 돌릴 것).
  결과 `/tmp/aruco_eval_<녹화>/frames.jsonl`, `status.json`.

## 4. 원리와 숫자

- 보드 자세: rect0(정류 왼쪽)에서 ChArUco 모서리 검출 → solvePnP (IPPE + LM, 정류 K, 왜곡 없음).
- 표본: 검출된 모서리 범위 안 보드 평면 점(칸의 1/4 간격)을 투영 → d_true = f·B / Z_board, d_ffs = 그 화소 FFS 시차.
- 한 키프레임: 이상치(중앙값 ± max(0.3 px, 4·MAD)) 빼고 요약 (평균 d_true, 평균 d_ffs, 가중치).
  가중치 = 모서리 수 · 보드 화소 크기 · cos(각도) / (1 + (재투영/0.3)²) / (1 + (움직임 회전/0.1°)² + (이동/0.3 mm)²).
- 누적: 시간 감쇠 가중 (τ = 60 s, EWMA 와 같은 뜻) Huber 회귀 d_ffs = a·d_true + b.
  d_true 범위가 1.4 배 이상이면 a 도, 아니면 a = 1. 보정 d_corr = s·d_ffs + Δd (s = 1/a, Δd = −b/a).
- 적용 조건(ready): 관측 3장 이상, |Δd| ≤ 2 px, 0.9 ≤ s ≤ 1.1.
- 거르는 기준: 모서리 ≥ 8, 보드 화소 크기 ≥ 60 px, 각도 ≤ 60°, 재투영 RMS ≤ 0.8 px, 거리 0.25~3 m.

### 합성 시험 결과 (`eval_aruco_scale.py synth`, 실제 녹화 rec_20261016_165645 배경, 1440x1080, f = 1041.6 px, B = 15.53 mm)
시차 모형 = a·d_true + b + 상관 잡음 σ 0.05 px + 이상치 2 %, 영상 흐림·잡음·JPEG.
- E1 한 장 (A4 4X4 칸 32 mm): 0.5~1 m 에서 한 장 오프셋 오차 중앙 0.001~0.02 px (깊이 0.01~0.14 %),
  PnP 거리 오차 ≤ 0.1 %. 1.5 m 는 정면·잡음 적을 때만, 2 m 는 검출 안 됨.
  사용자 보드 (6X6, 칸 34.5 mm): 1 m 까지 오차 ≤ 0.013 px, 1.5 m 이상 검출 안 됨.
- E2 누적: 0.5/1/1.5 m 24장, 주입 s = 0.9901, Δd = −0.149 → s = 0.9906, Δd = −0.159.
  깊이 오차 보정 전 −1.4 / −1.9 / −2.8 % (0.5 / 1 / 2 m) → 보정 후 −0.01 / +0.02 / +0.08 %.
  1 m 만 8장 (오프셋만): 주입 Δd 0.12 → 0.118. 잡음 σ 0.15 px: s 1.0101 → 1.0101, Δd −0.101 → −0.095.
- E3 열 드리프트 (0 → 0.2 px / 180 s, 5 s 마다 관측): 보정 후 1 m 시차 오차 중앙 τ 30 s 0.026, τ 60 s 0.040,
  τ 120 s 0.046 px (τ 가 길수록 늦게 따라감; 실제 잡음이 크면 τ 를 늘릴 것).
- E4 렌즈 자세 잔차 주입 yaw 0.1 / pitch 0.2 / roll 0.5 mrad → 추정 0.103 / 0.209 / 0.504 mrad (12장 중앙,
  한 장 yaw 표준편차 0.04 mrad = 0.04 px). PnP 로 잰 기선 15.66 mm (참값 15.53, 참고만).
- E5 VIO 축척 주입 1.03 → 1.031 (89 쌍, 퍼짐 0.004).
- E6 실제 FFS (GPU, 5 경우 x 3장): 위 lib_ffs 축소 버그 때문에 주입 0 에서도 +0.38~0.54 px 가 나옴.
  주입 0.3 px 와 0 의 차이 = +0.27~0.31 px 로 변화는 잘 잡힘. 모서리로 잰 시차 오프셋 (FFS 무관) 은 주입값과
  ±0.03 px 안으로 맞음. → 축소 버그를 고친 뒤 E6 를 다시 볼 것 (`synth --tests x --ffs`).
- 속도 (CPU): 보드 검출 약 11~12 ms (1440x1080), observe + rect1 확인 약 30 ms.

## 5. 통합 방법 (메인 담당이 할 일)

이 모듈은 cv2·numpy 만 쓰고 scan3d 를 import 하지 않는다.

1. **scan3d.py 인자**: `--aruco-board PATH` (보드 json, 없으면 끔), `--aruco-apply` (보정을 실제로 적용, 기본은 관측만).
   `__init__` 에서 `self.aruco = aruco_scale.make(a.aruco_board, log=self.log) if a.aruco_board else None`.
   해상도(세션 크기)가 바뀌면 `self.aruco.reset()` (f 가 바뀜). 맵 리셋 때는 지우지 않는다 (열 상태는 이어짐).
2. **depth_seg.DepthSeg.__call__**: `disp = lib_ffs.infer(...)` 바로 다음
   ```python
   disp_raw = disp
   if self.disp_fix is not None:            # = aruco.apply_disp (--aruco-apply 일 때만 넣음)
       disp = self.disp_fix(disp, rp)
   ```
   그 뒤 `valid_mask(disp, ...)` 는 보정된 시차로. 반환 dict 에 `disp_raw=disp_raw`, `rectR=rectR` 추가
   (rectR 은 이미 `lr.rectify_pair` 에서 만들어짐).
3. **scan3d._process_keyframe**: `r = self.ds(fL.bgr, fR.bgr, rp=rp, scale=...)` 바로 다음
   ```python
   if self.aruco is not None:
       T_cam = np.array(T, float)
       if R_fix is not None:
           T_cam[:3, :3] = T_cam[:3, :3] @ np.asarray(R_fix, float)    # 쌍별 정류 rect0' 의 월드 자세
       self.aruco.observe(r["rectL"], r["disp_raw"], rp, t, valid=r["valid"], T_world_cam=T_cam,
                          motion=info, rect1=r.get("rectR"))
   ```
   - 반드시 **보정 전 시차 (disp_raw)** 와 **이 쌍의 rp** (움직임 보정 정류) 를 넘긴다.
   - `info` = `_pair_rect` 가 준 dict (rot_deg, trans_mm) → 움직이는 키프레임 가중치를 낮춤.
   - 깊이 워커 스레드에서 약 30 ms 추가 (보드 없으면 검출 11 ms). 무거우면 rect1 을 빼거나 2 키프레임마다.
4. **적용**: `--aruco-apply` 면 `self.ds.disp_fix = self.aruco.apply_disp`. 준비 전(`ready=False`)엔 그대로 돌려준다.
   점·TSDF 모두 보정된 Z 를 쓰게 된다. (지도에 이미 쌓인 점은 다시 고치지 않음.)
   다른 경로가 깊이 Z 만 갖고 있으면 `apply_depth(Z, rp)` 또는 배율 `depth_factor(Z, rp)`.
5. **상태**: 상태 JSON 에 `aruco = self.aruco.status(t)` (board_seen, delta_d_px, s, mode, ready, n_obs, last{...},
   depth_change_pct{0.5m,1m,2m}, extrinsic{yaw/pitch/roll_mrad, corner_offset_px, dy_px, ffs_minus_corner_px},
   vio_scale{ratio_median, n_pairs}, warn). 웹은 "보드 보임 · Δd · 1 m 깊이 변화 % · VIO 축척" 정도만 보여 주면 됨.
   켜고 끄기: `self.aruco.enabled = False` (관측은 계속).
6. **replay**: replay 도 `_process_keyframe` 를 지나므로 따로 할 일 없음.

읽는 법
- `ffs_minus_corner_px` ≈ 0 이고 `corner_offset_px` ≈ Δd → 렌즈 자세(열로 yaw 변화) 탓. 보정 파일 R 을 다듬을 일.
- `corner_offset_px` ≈ 0 인데 Δd ≠ 0 → FFS 쪽 치우침.
- `vio_scale.ratio_median` 이 1 에서 1 % 넘게 벗어나면 VIO 축척 문제 (지도 크기 틀림).
- `last.board_drift_m` 이 커지면 VIO 위치 드리프트.

## 6. 한계

- 보드 실제 크기 오차가 그대로 축척 오차가 된다 (자 0.5 mm / 224 mm = 0.2 %).
- 보정은 **전체** 오프셋·비례만. 화면 위치마다 다른 오차(위 lib_ffs 버그, 렌즈 왜곡 잔차)는 못 고친다.
  보드를 화면 가운데·가장자리에서 다 보여 주면 평균은 맞지만 위치별 차이는 남는다.
- 보드 무늬(흑백 큰 칸)에서의 FFS 치우침이 일반 벽·물건에서와 같다는 보장은 없다 (확인 안 됨).
- 움직이면 렌즈 2·5 촬영 시각 차이(약 30 ms) 때문에 정류 잔차가 Δd 에 섞인다 → 멈춰서 찍기, 가중치로 일부 줄임.
- 보드가 오래 안 보이면 마지막 값이 유지된다 (τ 감쇠는 상대 가중치일 뿐). 열이 계속 변하면 가끔 다시 비출 것.
- 거리: 1440x1080 에서 A4 4X4 는 약 1.5 m, 사용자 6X6 A4 는 약 1 m 까지 (합성 시험 기준, 실제 흐림·노출에 따라 더 짧을 수 있음).
- 사진 칸 재기: 폰 사진 왜곡이 남아 있으면 0.2 % 정도 틀릴 수 있다 (합성 k1 = −0.03). 보드를 가운데, 망원으로.
- 실제 보드 녹화로는 아직 검증 안 됨 (합성만).

## 7. 실제 보드 녹화 결과 (2026-10-18) → 보정 v2

녹화 rec_20261018_002349 (0.45~0.7 m) · rec_20261018_004937 (0.24~1.2 m, 종이를 더 평평하게). 보드 칸 37.0 mm (사용자 자:
7칸 259 mm, 5칸 185 mm, ±0.4 %). 처리 `eval_aruco_scale.py rec ... --calib <파일> --vio-poses none`.

- 쌍마다 모서리 시차·세로 어긋남이 크게 흔들린 원인 = 두 렌즈 촬영 시각 차 (+22.7 / −15.6 ms) 동안의 **이동** (회전은 자이로로
  보정되지만 이동은 안 됨). 보드 PnP 로 구한 카메라 속도로 예측한 f·v·dt/Z 와 상관 0.96 (시차) · 0.93 (dy). 이것을 빼면 남는
  고정 값이 시간·거리·각도 구간마다 거의 같다.
- 옛 보정 (calib_phone_pair.json) 잔차: 모서리 시차 −1.53 / −1.62 px (yaw +1.47 / +1.56 mrad), dy −2.80 / −2.64 px
  (pitch +2.8 mrad), FFS − 모서리 ≈ 0 (FFS 는 정류 기하를 그대로 따름). → 깊이가 0.5 m 에서 약 5 %, 1 m 에서 약 7~9 % 길게 나옴.
- 보드 없이 독립 확인: SfM 성긴 점 (165645, 175446) 깊이 대비 보정 정류 시차 오프셋 −1.82 / −1.52 px (yaw 1.7 / 1.4 mrad),
  dy −1.2 / −1.5 px (pitch 1.2~1.4 mrad). SfM 상대 회전만 비교하면 yaw 0.1~0.8 mrad 로 작게 나오는데, SfM 기선 방향에
  z 성분 21 % 가 섞여 있어 (렌즈 5 초점 0.3 % 차이로 추정) yaw 와 엉킨 것으로 본다 (시차 오프셋 쪽을 믿음).
- 칸 크기에 대한 민감도: 칸 ±5 % → 오프셋 ∓1.5 px. 처음 34.5 mm (추정) 로 낸 −3.5 px · 3.45 mrad 는 칸 크기 탓이 대부분.
- **보정 v2** = `calib_phone_pair_v2.json` (렌즈 5 를 정류 좌표에서 yaw 1.5 · pitch 2.0 mrad 돌림, `tools/calib_rot_fix.py`,
  근거 숫자는 파일 안 `rotation_fix`). 2+6 은 같은 장면 쌍으로 yaw 를 v2 기준으로 다시 맞춘 `calib_phone_pair_2_6_v2.json`.
  scan3d·sfm_recon·make_ov_config 기본 = v2 (`calib_scale.default_calib`), 옛 파일은 `--calib-v1` 또는 `SCAN3D_CALIB=v1`.
- v2 잔차: 모서리 시차 +0.04 / +0.10 px, dy −0.61 / −0.51 px. 보드 기준 깊이 오차 0.5 m −5.3 → −0.5 % (두 녹화 모두),
  1 m −7.4 → +2.7 % (004937 affine; 1 m 넘는 쪽은 관측이 적어 외삽 — 믿지 말 것). 남은 비례 s ≈ 1.03~1.04 는
  기선 길이·보드 크기 어느 쪽인지 못 가름 (적용 안 함).
- 검출: `minMarkerPerimeterRate 0.01` + `tryRefineMarkers` 로 두 렌즈 모두 보인 쌍 97 → 105 (+8 %, 16 → 24 ms/장).
  적응 임계 창을 넓히면 108 쌍인데 40 ms 넘게 걸려 안 씀. 첫 녹화는 0.45~0.7 m 만 있어서 검출기를 바꿔도 거리 범위는 안 넓어짐.
- scan3d 통합: `--aruco-board` (기본 auto = config/charuco/board_user_A4_6x6.json) 로 키프레임마다 관측, 웹 상태표 줄 "보드 (ArUco)".
  `--aruco-apply` 는 기본 꺼짐 (1 m 넘는 곳 보정이 아직 확인 안 됨, 위 외삽 문제).
