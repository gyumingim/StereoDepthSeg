# 정지 초기화 자이로 바이어스를 사전값으로 — 계획 (2026-10-23, 실행 전 커밋)

앞 시험 `docs/initgyro_result.md` (자이로 문턱으로 초기화 막기): 131326 은 고쳐졌지만 막힌 3 칸이 늦게 초기화되어 발산 → 불통과.
이번 변형 (메인 세션 결정): **초기화 시각은 V4 그대로 (막지 않음)**, 대신 초기화 창 자이로 평균을 bg0 로 쓰지 않는다.

## 바꾼 것
- 새 설정 키 `init_bg_mode` (0 = 끔 = 원본, 1 = bg0 = 0, 2 = bg0 = `init_bg_prior` [x, y, z] rad/s).
  정지 초기화의 `bg = 옛 반 자이로 평균` 한 줄 뒤에서만 바꿈. 초기화 시각·중력 방향·ba·v=0·초기 공분산은 그대로.
- 코드: `~/scanner/src/open_vins` (V4 원본) 의 복사본 `~/scanner/src/open_vins_initbg` + `tools/ov_initbg_patch.py`
  (diff `tools/ov_patches/initbg.diff`, ov_init 두 파일). 라이브러리 `~/scanner/src/ovbuild_initbg`, 브리지 = 원본 `ov_bridge/main.cpp` 를
  `~/scanner/src/ovbuild_initbg_bridge` 에 다시 빌드. 실행 `imu_learn/initbg_run.py` (V4 와 같은 인자, 설정 끝에 두 줄 덧붙임).
- **바이어스 관련 기본값 (그대로 둠)**: 정지 초기화의 초기 공분산 bg σ = **0.02 rad/s** (`StaticInitializer.cpp`, 모든 상태 0.02², q 0.02², p 0.05², v 0.01²),
  자이로 random walk 1.0e-4 rad/s²/√Hz, 잡음 1.0e-3 rad/s/√Hz (`config/mono/kalibr_imu_chain.yaml` = make_ov_config.IMU_NOISE).
  σ 0.02 는 작지 않음: 원시 IMU 참 바이어스 크기 ≈0.006 → P1(0) 의 오차는 0.3σ. 131326 의 V4 bg0 0.125 는 6σ 였다.
  주의: 131326 은 초기화 순간 실제로 움직이고 있었으므로 **v = 0 (σ 0.01 m/s) 과 중력 방향(회전 중 가속도 평균) 도 틀린 채** 남는다 → bg 만 고쳐서 살아날지는 모름.

## 사전값 P2 (leave-one-recording-out, 실행 전 계산)
`imu_learn/initbg_prior.py` → `runs/initbg/prior.json`. 정지 창 = initgyro_thresh 와 같은 이름표 (0.5 s 창, GT 0.1 s 기준 회전 속도 p90 < 0.02 rad/s, GT 커버 ≥ 80 %), 0.25 s 간격.
녹화마다 정지 창 자이로 평균의 축별 중앙값 → 녹화 R 의 사전값 = **R 을 뺀, 같은 IMU 종류(원시 / 안드로이드 보정) 녹화들** 값의 축별 중앙값.
213955 도 원천으로 씀 (점수만 제외). 정지 창이 없는 녹화(221925, 224323, 131326)는 원천이 못 되지만 사전값은 받음.

| IMU 종류 | 원천 녹화 | 녹화별 중앙값 범위 (rad/s) | 사전값 |
|---|---|---|---|
| 원시 16/35 (13 녹화) | 11 | x 0.0027–0.0033, y 0.0036–0.0051, z 0.0019–0.0025 | ≈ [0.0031, 0.0042, 0.0023] (자기 값과 차 0.0002–0.0010) |
| 보정 1/4 (131326, 154320) | 154320 만 (정지 창 1 개) | [0.0071, 0.0011, 0.0035] | 131326 ← 154320 값; **154320 ← 원천 없음 → 0 (= P1 과 같음)** |
- 보정 IMU 쪽 사전값은 근거가 약함 (창 1 개). 131326 의 참 바이어스(ZUPT 켠 base 의 bg0 [0.0084, −0.0048, 0.0009])와 차 ≈0.006 — 참고만, 사전값 만들 때 안 씀.

## 비트 확인 (완료, 실행 전)
`init_bg_mode: 0`, `init_bg_prior: [0,0,0]` 줄을 넣고 새 브리지 131326 o0 전 구간 → V4 포즈 **5518 행 모두 같음** ('ms' 만 뺌). `runs/initbg/bitcheck.json`.

## 비교 (o0 만, 15 칸 × 2 변형, 칸당 1 회, 동시 ≤ 12)
- 변형 P1 (mode 1), P2 (mode 2). 기준 = `runs/phone60/<rec>_o0_V4` 재사용. 점수 = `zupt_ob_score.run_metrics` (V4 와 같은 발산 판정).
- **기준 (변형마다 따로, 셋 다 통과해야 "효과 있음")**
  (a) 213955 제외 o0 14 칸 발산 수 ≤ V4 o0 (= 2: 131326, 224323), **그리고 131326 o0 발산 안 함**.
  (b) 두 쪽 다 정상인 o0 칸의 창 ATE 중앙값 ≤ 1.2 × V4 + 0.3 cm.
  (c) **초기화 시각(첫 tracking 포즈 t_ns)이 15 칸 모두 V4 와 같을 것** (아니면 패치가 시각을 바꾼 것 = 실험 무효).
- 함께 보고: 칸별 bg0 (V4 / P1 / P2), 끝 bg.
- **약점: 오프셋 하나, 칸당 1 회 → 반복 없음.** 131326 의 결과는 사건 하나. P2 의 보정 IMU 사전값은 창 1 개 근거.

## 재현
```bash
mkdir ~/scanner/src/open_vins_initbg && cp -a ~/scanner/src/open_vins/{ov_core,ov_init,ov_msckf} ~/scanner/src/open_vins_initbg/
python3 tools/ov_initbg_patch.py ~/scanner/src/open_vins_initbg
mkdir ~/scanner/src/ovbuild_initbg && cd ~/scanner/src/ovbuild_initbg && cmake ../open_vins_initbg/ov_msckf -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=ON -DCMAKE_PREFIX_PATH=$HOME/scanner/local && make -j12 ov_msckf_lib
cmake -S ~/scanner/StereoDepthSeg/scan3d/ov_bridge -B ~/scanner/src/ovbuild_initbg_bridge -DOV_SRC=$HOME/scanner/src/open_vins_initbg -DOV_BUILD=$HOME/scanner/src/ovbuild_initbg && make -C ~/scanner/src/ovbuild_initbg_bridge
cd ~/scanner/StereoDepthSeg/scan3d
../venv_ffs/bin/python imu_learn/initbg_prior.py
../venv_ffs/bin/python imu_learn/initbg_run.py bitcheck 131326_o0
../venv_ffs/bin/python imu_learn/initbg_run.py run && ../venv_ffs/bin/python imu_learn/initbg_run.py score
```
