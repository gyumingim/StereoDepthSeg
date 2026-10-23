# 정지 초기화에 자이로 정지 조건 추가 — 계획 (2026-10-23, 실행 전 커밋)

배경 = `docs/collapse_nozupt_20261023.md` 1 절: try_zupt=false 이면 OpenVINS 는 "정지 창 + 흔들림(jerk)" 정지 초기화를 쓰고,
정지 판정은 **가속도 흔들림만** 본다 (`StaticInitializer.cpp`). 131326 은 폰이 ≈16°/s 로 도는 중에 초기화되어
그 창 자이로 평균을 바이어스로 저장 (bg0 0.125 rad/s, 참 ≈0.01) → o0·o1·o2 모두 발산.

## 바꾼 것 (한 가지)
- 새 설정 키 `init_max_gyro` (rad/s, 없으면 0 = 끔).
- 정지 초기화에서 기존 가속도 검사 통과 뒤, **바이어스 창** (init 창의 옛 반 = bg·중력·v=0 을 뽑는 0.5 s) 을 0.1 s 칸 5 개로 나눠
  g = max(칸별 |자이로 평균 벡터|). g > init_max_gyro 이면 초기화 안 함 → 다음 프레임 창에서 다시 시도.
  (칸별 평균의 최댓값: 앞뒤로 흔드는 회전이 평균에서 지워지는 것을 막음. 0.1 s 평균이라 자이로 잡음은 작게 남음.)
- 코드: `~/scanner/src/open_vins` (V4 브리지가 쓴 원본, `ov_bridge/build/ov_bridge` → `ovbuild/libov_msckf_lib.so`) 의 **복사본**
  `~/scanner/src/open_vins_initgyro` 에 `tools/ov_initgyro_patch.py` 적용 (diff = `tools/ov_patches/initgyro.diff`, ov_init 두 파일만).
  라이브러리 `~/scanner/src/ovbuild_initgyro` (Release, ROS 끔, ARUCO 켬 = ovbuild 와 같은 설정), 브리지 = 원본 `ov_bridge/main.cpp` 를
  밖에서 빌드 `~/scanner/src/ovbuild_initgyro_bridge/ov_bridge` (옵션 구조체가 바뀌므로 LD_LIBRARY_PATH 바꿔치기 대신 헤더째 새로 빌드).
  open_vins·open_vins_diag 는 안 건드림.
- 실행 = `imu_learn/initgyro_run.py` (phone60_run V4 와 같은 인자: config/mono + try_zupt=false, clones 11, strict, init 창 1.0, 시차 15,
  프레임 캐시, gyro_rebias 끔, opencv 스레드 2). 설정 파일 끝에 `init_max_gyro: <값>` 한 줄만 덧붙임.

## 비트 확인 (완료, 실행 전)
`init_max_gyro: 0` 으로 새 브리지 131326 o0 전 구간 → V4 저장 포즈와 **5518 행 모두 같음** ('ms' 만 뺌), ATE 249.85 = 249.85.
`runs/initgyro/bitcheck.json`.

## 문턱값 = 0.05 rad/s (2.9°/s) — 비교 전에 데이터로 정함
`imu_learn/initgyro_thresh.py` → `runs/initgyro/thresh.json`. 15 녹화 전부, 0.05 s 마다 0.5 s 창, 브리지가 받는 자이로 그대로
(새 녹화 = 원시 16/35, 131326·154320 = 안드로이드 보정 1/4; 바이어스 포함). 이름표는 GT(보드) 카메라 회전 속도 (0.1 s 기준선, GT 커버 ≥ 80 %):
정지 = GT 속도 p90 < 0.02 rad/s, 움직임 = GT 속도 중앙값 > 0.10 rad/s. (GT 정지 구간 속도는 ≈0.001 rad/s 로 움직임과 확실히 갈림.)
213955(낙하) 제외.

| 창 | 개수 | g 1 % | 50 % | 99 % | 최대 |
|---|---|---|---|---|---|
| 정지 (GT) | 3065 | 0.0045 | 0.0060 | 0.0156 | **0.024** |
| 움직임 중 가속도 조건(std < 0.3)도 통과한 창 = 이 문턱이 막아야 할 창 | 220 | **0.120** | 0.232 | 0.795 | 0.813 |
| 움직임 전체 | 24856 | 0.176 | 0.853 | 3.94 | 9.56 |

| 문턱 | 정지 창 막힘 | 가속도 통과 움직임 창 통과 |
|---|---|---|
| 0.02 | 0.46 % | 0 % |
| 0.03–0.10 | 0 % | 0 % |
| 0.15 | 0 % | 10.9 % |

- 정지 최댓값 0.024 과 막을 창 최솟값 0.120 사이. **0.05 = 정지 최댓값의 2 배, 막을 창 최솟값의 0.4 배** (양쪽 여유).
  보정 IMU 녹화(131326·154320)의 정지 창은 0.017–0.024 로 원시보다 높음 (표본 7 개뿐) → 0.03 은 여유가 작아 안 씀.
- 131326 의 V4 초기화 순간 바이어스 창 g = 0.34–0.69 (창 위치 ±0.2 s 불확실 범위) → 0.05 면 막힘.
- 다른 녹화의 V4 o0 초기화 창 g (같은 범위): 0.03–0.20 이 섞인 곳이 있음 (134550, 213341, 222747, 224323, 221925 등) → 그 칸들은 초기화가 늦어질 수 있다 (비용, 아래 보고).
- 한계: 정지 이름표는 GT(보드)가 보일 때만 → 내려놓은 정지(보드 안 보임)는 표본에 없다. 온도에 따른 바이어스 변화는 이 녹화 범위만.

## 비교 (사용자 결정 2026-10-23: 시작 오프셋 o0 하나만, 15 칸, 칸당 1 회)
- 조건 = ig (init_max_gyro 0.05) 15 칸. 기준 = `runs/phone60/<rec>_o0_V4` 재사용 (비트 확인 통과). 프레임 캐시, 동시 ≤ 12.
- 점수 = `zupt_ob_score.run_metrics` (V4 와 같은 발산 판정: 창 ATE > 50 cm 또는 rel5 > 30 cm).
- **기준 (둘 다 통과해야 "효과 있음")**
  (a) 213955 제외 o0 14 칸 발산 수 ≤ V4 o0 의 발산 수 (= 2: 131326, 224323), **그리고 131326 o0 가 발산하지 않을 것**.
  (b) 두 조건 모두 정상인 o0 칸의 창 ATE 중앙값 ≤ 1.2 × V4 + 0.3 cm.
- 함께 보고: 칸별 초기화 시각 차 (ig − V4, 늦어짐 = 비용), 첫 추적 bg0 크기, 224323 은 원인이 다름(끝 정지, 시각 갱신 0) → 이 수정으로 안 고쳐질 것으로 예상 (추정).
- **약점: 오프셋 하나, 칸당 1 회 → 반복이 없다.** V4 에서 131326 은 세 오프셋이 같은 사건이었고 (같은 초기화 순간) 이번에도 o0 한 번만 본다.
  발산 1 칸 차이는 잡음일 수 있다. 결과는 "o0 에서 이랬다" 까지만 말한다.

## 재현
```bash
mkdir ~/scanner/src/open_vins_initgyro && cp -a ~/scanner/src/open_vins/{ov_core,ov_init,ov_msckf} ~/scanner/src/open_vins_initgyro/
python3 tools/ov_initgyro_patch.py ~/scanner/src/open_vins_initgyro
mkdir ~/scanner/src/ovbuild_initgyro && cd ~/scanner/src/ovbuild_initgyro && cmake ../open_vins_initgyro/ov_msckf -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=ON -DCMAKE_PREFIX_PATH=$HOME/scanner/local && make -j12 ov_msckf_lib
cmake -S ~/scanner/StereoDepthSeg/scan3d/ov_bridge -B ~/scanner/src/ovbuild_initgyro_bridge -DOV_SRC=$HOME/scanner/src/open_vins_initgyro -DOV_BUILD=$HOME/scanner/src/ovbuild_initgyro && make -C ~/scanner/src/ovbuild_initgyro_bridge
cd ~/scanner/StereoDepthSeg/scan3d
../venv_ffs/bin/python imu_learn/initgyro_thresh.py
../venv_ffs/bin/python imu_learn/initgyro_run.py bitcheck 131326_o0
../venv_ffs/bin/python imu_learn/initgyro_run.py run && ../venv_ffs/bin/python imu_learn/initgyro_run.py score
```
