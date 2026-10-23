# OpenVINS 설정 조사: 폰(모노+폰 IMU)·소형 드론은 어떻게 쓰나 (2026-10-23)

작성: 조사 에이전트. 입력: 웹 출처(아래 URL), 로컬 업스트림 `~/scanner/src/open_vins` (commit 6948812, 2025-11-30),
우리 벤치 `imu_learn/vio_bench_hs.py` → `write_config()` → 템플릿 `config/mono/estimator_config.yaml`,
IMU 잡음 `make_ov_config.py IMU_NOISE`, Allan 결과 `runs/gyro_bias_stability.json`.
"추정" = 측정·출처 없이 내가 판단한 것.

## 0. 결론 먼저

1. **가장 좋은 기준선 = OpenVINS 저자(goldbattle)가 2025-11 에 올린 안드로이드 폰 예제 설정** (`open_vins_mobile`, 모노 640x480, Camera2 + SensorManager).
   거기서는 `try_zupt: false`, 초기화 `init_window_time 1.0 / init_imu_thresh 0.4 / init_max_disparity 2.0`,
   `up_msckf_sigma_px 2 / chi2_multipler 5`, 온라인 보정 셋 다 켬(extrinsics·intrinsics·timeoffset), IMU 잡음 = EuRoC 값 그대로.
   https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app_device/config/estimator_config.yaml
2. **ZUPT 는 "계속 켜기"가 아니라 "처음에만"이 관례.** 손으로 들고 다니는 업스트림 설정(tum_vi, rpng_aruco, rpng_plane)과 폰 예제,
   찾은 드론 설정 2개는 모두 `zupt_only_at_beginning: true`. 계속 켜는 건 지상 차량(kaist, rpng_ironsides) 뿐.
   우리는 `zupt_only_at_beginning: false` + 시차만(chi2 0) → 업스트림 어디에도 없는 조합.
3. **60 fps 에서 ZUPT 시차 0.5 px/프레임은 느슨하다.** 업스트림 0.5 px 는 20~30 Hz 기준(= 10~15 px/s). 60 fps 에서는 30 px/s
   (f≈430 px 이면 약 4°/s 또는 1 m 앞 4 cm/s) 보다 느린 움직임을 "정지"로 판정 → 그 프레임의 시각 갱신을 건너뜀(코드 확인). 느린 스캔 동작에서 오판 위험 (추정).
4. **60 fps + max_clones 11 = 창 0.18 s 뿐.** 업스트림은 11 clone 을 20~31 Hz 로 써서 창 0.35~0.5 s. 모노 MSCKF 는 창 안의 시차로 삼각측량하므로
   창이 짧으면 깊이·스케일 정보가 줄어든다 (원리는 확실, 영향 크기는 추정). → 30 Hz 로 추적하거나 clone 을 늘리는 A/B 권장.
5. **IMU 잡음은 이미 관례 범위.** 공식 문서 "Allan 값의 10~20배". 우리 값은 Allan 대비 자이로 ×15, 가속 ×25 → 대체로 맞음. 바꿀 1순위 아님.
6. **롤링 셔터는 OpenVINS 가 모델링하지 않는다.** 저자: 판독 시간이 짧으면 글로벌 셔터와 비슷하게 동작. 우리 2.9 ms 는 짧은 편.
   대신 시각을 노출 중앙에 맞추고 `calib_cam_timeoffset: true`, 픽셀 잡음 1→1.5~2 가 관례(폰 예제 2).

## 1. 우리 현재 값 (파일에서 확인)

| 항목 | 값 | 출처(로컬) |
|---|---|---|
| 카메라 | 모노 640x360 @60 fps, S24 주 카메라, RS skew ≈2.9 ms (사용자 제공) | 과제 설명 |
| f (640x360) | ≈ 430 px (1920x1080 K fx 1267~1304 ÷3, 추정 환산) | `config/lens/hs_1920x1080_*.json` |
| IMU | 원시 ≈500 Hz, `update_rate` 템플릿 400 | `config/mono/kalibr_imu_chain.yaml` |
| 초기화 | `init_dyn_use false`; strict = `init_window_time 1.5`, `init_imu_thresh 0.3`, `init_max_disparity = f·tan 0.3° ≈ 2.25 px`; 과제에 적힌 실행값 1.0 s / 15 px 는 `--init-window/--init-disp` 덮어쓰기 또는 loose(15·W/640) | `make_ov_config.init_params`, `vio_bench_hs.init_values` |
| ZUPT | `try_zupt true`, `zupt_chi2_multipler 0`, `zupt_max_velocity 0.1`, `zupt_noise_multiplier 10`, `zupt_max_disparity 0.5`, `zupt_only_at_beginning false` | 템플릿 |
| 창·특징 | `max_clones 11`, `max_slam 50`, `num_pts 200`, `fast_threshold 15`, `grid 5x5`, `min_px_dist 10`, KLT, HISTOGRAM | 템플릿·`--npts/--clones` 기본 |
| 온라인 보정 | extrinsics true, intrinsics false(정류·핀홀 고정), timeoffset true | 템플릿 |
| 측정 잡음 | `up_msckf_sigma_px 1`, `chi2 1` | 템플릿 |
| IMU 잡음 (cfg) | gyro ND 1.0e-3, gyro RW 1.0e-4, acc ND 1.0e-2, acc RW 1.0e-3 | `make_ov_config.IMU_NOISE` |
| Allan (43 분 정지) | gyro ND ≈ 6.6e-5 rad/s/√Hz (3.8e-3 °/s/√Hz), gyro RW ≈ 2.7e-6, acc ND ≈ 4.0e-4, acc RW 2e-5(x)~1.4e-3(z, τ=0.6 s 라 의심) | `runs/gyro_bias_stability.json` |
| 배율 (cfg/Allan) | gyro ND ×15, acc ND ×25, gyro RW ×37, acc RW ×1~50 | 계산 |

## 2. 공식 문서·코드에서 확인한 것

- **IMU 잡음 부풀리기**: "Typically these noise values should be inflated by maybe 10-20x their values to take into account unmodelled errors". 같은 페이지가 extrinsic·시간 오프셋의 **온라인 추정을 권장**, 내부 파라미터는 오프라인 보정이 우선. 롤링 셔터 언급 없음.
  https://docs.openvins.com/gs-calibration.html
- 저자 답변: 원시 잡음을 그대로 쓰면 "too small … inflate by 1-2 orders" (issue #172). https://github.com/rpng/open_vins/issues/172
- **ZUPT 문서**: 주 목적은 "모노 + SLAM 특징 없음"에서 정지 시 갱신 불가 문제와 정지 중 움직이는 물체(신호 대기 차) 대응. 실제로는 a=0, ω=0 합성 측정이라 **등속도에서도 오판 가능**(속도 크기 검사로 보완).
  IMU 기반 판정은 R 을 **50~100 배** 부풀려야 했다고 기록. 시차 기반은 "동적 환경에서 실패할 수 있어 IMU 와 같이 쓰면 좋다".
  https://docs.openvins.com/update-zerovelocity.html
- **ZUPT 코드 동작** (`UpdaterZeroVelocity.cpp`): 시차 통과 조건 = 평균 시차 < `zupt_max_disparity` **그리고** 특징 > 20개. 시차 통과 못 하면 `chi2 > chi2_multipler·χ²` 또는 `|v| > zupt_max_velocity` 일 때 거절.
  → `chi2_multipler 0` 이면 **시차만으로 결정**(IMU·속도 검사 무시). ZUPT 가 받아들여지면 `VioManager` 는 그 프레임의 특징 추적·갱신을 **건너뛰고 return**.
  https://github.com/rpng/open_vins/blob/master/ov_msckf/src/update/UpdaterZeroVelocity.cpp ,
  https://github.com/rpng/open_vins/blob/master/ov_msckf/src/core/VioManager.cpp
- **초기화 코드 동작** (`InertialInitializer.cpp`, `VioManagerHelper.cpp`): `wait_for_jerk = (ZUPT 꺼짐)`.
  ZUPT 켜면 "정지 상태"(두 반창 모두 시차 < `init_max_disparity`)에서 바로 정적 초기화. ZUPT 끄면 "앞 반은 정지, 뒷 반은 움직임 + 가속 표준편차 > `init_imu_thresh`"(들어올리는 순간)를 기다린다.
  즉 **try_zupt 를 끄면 초기화 방식도 바뀐다** — 따로 떼어 비교할 수 없는 섞인 변수. 안드로이드 빌드에서는 동적 초기화가 꺼져 있음(`#ifndef __ANDROID__`, Ceres 없음).
  https://github.com/rpng/open_vins/blob/master/ov_init/src/init/InertialInitializer.cpp
- 모노는 "정지 후 움직임"이 있어야 초기화 (저자, issue #342). https://github.com/rpng/open_vins/issues/342
- `init_max_disparity` 주석: "dependent on resolution" (euroc 752x480 에서 10). https://github.com/rpng/open_vins/blob/master/config/euroc_mav/estimator_config.yaml

## 3. 다른 사람들의 설정

### 3.1 업스트림 14개 (로컬 commit 6948812 에서 직접 읽음)
https://github.com/rpng/open_vins/tree/master/config

| 설정 | 플랫폼 | try_zupt | zupt chi2/vel/noise/disp/only_begin | init win/imu/disp | track Hz | num_pts | fast | sigma_px | calib ext/int/dt | gyro ND / acc ND |
|---|---|---|---|---|---|---|---|---|---|---|
| euroc_mav | MAV | false | 0/0.1/10/0.5/F | 2.0/1.5/10 | 21 | 200 | 20 | 1 | T/T/T | 1.7e-4 / 2.0e-3 |
| uzhfpv_indoor(_45) | 고속 FPV 드론 | false | 0/0.5/20/0.5/F | 2.0/0.30/2.0 | 31 | 200 | 50 | 1.5 | F/T/T | 1.7e-4 / 2.0e-3 |
| uzhfpv_outdoor | 고속 FPV 드론 | false | 0/0.5/20/0.5/F | 2.0/0.30/2.0 | 31 | 200 | 25 | 1.5 | F/T/T | 1.7e-4 / 2.0e-3 |
| tum_vi | 손 | false | 0/0.1/50/2.0/**T** | 1.5/0.45/15 | 21 | 200 | 20 | 1 | T/T/T | 1.6e-4 / 2.8e-3 |
| rpng_aruco | 손 | false | 0/0.1/50/0.5/**T** | 2.0/1.2/2.0 | 21 | 150 | 30 | 1.5 | T/T/T | 1.7e-4 / 2.0e-3 |
| rpng_plane | 손, 모노 | false | 0/0.1/50/1.5/**T** | 2.0/0.5/4.0 | 31 | 200 | 30 | 1 | T/T/T | 2.1e-4 / 2.1e-3 |
| rs_d455 / rs_t265 | 손 | false | 0/0.1/10/0.5/F | 2.0/1.5/10 | 31 | 200 | 30 | 1 | T/T/T | 2.1e-4·5.4e-4 / 2.1e-3·7.3e-3 |
| kaist | 차 | **true** | 0.5/0.1/1/0.4/F | 2.0/0.5/1.5 | 31 | 200 | 30 | 1.5 | F/T/T | 1.7e-4 / 5.9e-3 |
| rpng_ironsides | 지상 로봇 | **true** | 1/0.5/10/0.4/F | 2.0/0.5/1.5 | 31 | 100 | 30 | 2 | F/T/F | 1.1e-4 / 2.7e-3 |
| kaist_vio | 드론 | false | 0/0.02/10/0.2/F | 2.0/0.6/5.0 | 31 | 200 | 30 | 1.2 | F/T/F | 1e-3 / 7e-2 |
| rpng_sim | 시뮬 | false | 1/0.1/1/0/F | 2.0/1.0/1.5 | 21 | 250 | 15 | 1 | T/T/T | 1.7e-4 / 2.0e-3 |

공통: 전부 `max_clones 11`, `use_klt true`, `histogram_method HISTOGRAM`, `max_slam 50`(rpng_plane 25). try_zupt true 는 14개 중 2개(지상 차량).

### 3.2 폰
- **OpenVINS 저자의 안드로이드 앱 `open_vins_mobile`** (2025-11-17 갱신, Galaxy 폰 Kalibr 결과 포함). https://github.com/goldbattle/open_vins_mobile
  - 설정: 모노, `calib ext/int/dt = T/T/T`, `max_clones 11`, `max_slam 50`, `try_zupt false` (남은 값: chi2 0, vel 0.1, noise 50, disp 1.5, only_begin **true**),
    `init 1.0 s / 0.4 / 2.0 px`, `init_max_features 25`, `num_pts 200`, `fast 20`, `min_px_dist 10`, `track_frequency 31`, HISTOGRAM, KLT,
    **`up_*_sigma_px 2`, `up_*_chi2_multipler 5`**. https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app_device/config/estimator_config.yaml
  - IMU 잡음: EuRoC 값 그대로 (gyro ND 1.6968e-4, gyro RW 1.9393e-5, acc ND 2.0e-3, acc RW 3.0e-3, 200 Hz). Kalibr imu.yaml 도 같은 값 → 폰을 따로 Allan 측정하지 않은 것으로 보임(추정).
    https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app_device/config/kalibr_imu_chain.yaml
  - 카메라: 640x480, Kalibr `timeshift_cam_imu = 0.0188 s`(18.8 ms), 재투영 오차 평균 0.91 px. https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app_device/config/kalibr_imucam_chain.yaml
  - 앱: Camera2 `SENSOR_TIMESTAMP`(부팅 시계), **AF 끔·초점 무한대 고정**, AE 켬 + 노출 보정 −2, `TYPE_ACCELEROMETER`/`TYPE_GYROSCOPE` `SENSOR_DELAY_FASTEST`, 가속·자이로는 "둘 다 오면 하나로 묶는" 단순 짝짓기(TODO 주석으로 동기 문제 인정). 롤링 셔터 처리 없음.
    https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app/src/main/java/com/openvins/android/Camera2ResView.kt ,
    https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app/src/main/java/com/openvins/android/MainActivity.kt
- 폰 데이터 실패 사례: #486 (MARS logger, 30 fps/240 Hz, 발산 → 저자 "초기화 문제일 수도"), #326 (S22 Ultra 모노, Kalibr extrinsic 이 1 m 넘게 틀려 실패 → 보정 고친 뒤 해결).
  https://github.com/rpng/open_vins/issues/486 , https://github.com/rpng/open_vins/issues/326
- MARS logger 위키(폰 VIO 권장값, OKVIS 기준): gyro ND 2.4e-3, acc ND 1.6e-2, gyro RW 2e-5, acc RW 5.5e-5; 폰 왜곡 0 가능, RS 판독시간은 메타데이터 또는 1/30 s, 일부 폰은 카메라·IMU 시계가 다름. https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki
- ADVIO(폰 VIO 데이터셋, iPhone/Pixel/Tango, 롤링 셔터): OpenVINS 공식 설정은 찾지 못함. https://arxiv.org/pdf/1807.09828
- 폰 대상 리뷰: 특징 기반 방법은 RS 로 큰 손해가 없고, 폰의 좁은 화각이 MSCKF 등의 강인성을 떨어뜨린다고 보고. https://www.cjig.cn/en/article/doi/10.11834/jig.230863/
- 지터: 저자 "작은 지터(0.05 ms)는 문제 없음, 옛 폰 같은 저전력 시스템에서나 신경 씀"; IMU 누락이 이미지 누락보다 나쁨. https://github.com/rpng/open_vins/issues/133
- 롤링 셔터: "If the RS read out is small, then likely it should perform similar (up to some order) as a global shutter" (저자, #464). https://github.com/rpng/open_vins/issues/464

### 3.3 드론
- 업스트림 uzhfpv(고속 FPV)·euroc_mav: 위 표. ZUPT 끔, extrinsic 온라인 보정은 uzhfpv 에서 끔, `fast_threshold` 25~50(높음), sigma_px 1.5.
- ModalAI VOXL (OpenVINS 기반 상용 드론 서버): 트래킹 카메라 AR0144 **글로벌 셔터** 30 fps, 롤링 셔터 IMX664 60 fps 프로필도 있음. `imu_init_max_gravity_angle_deg`, 자동 리셋, 진동 점검 권고. https://docs.modalai.com/voxl-open-vins-server/
  - 사용자 공유 설정(VOXL, 모노 트래킹 1대): `try_zupt true`, **`zupt_only_at_beginning true`**, `init_window_time 0.5`, `init_imu_thresh 2`, `max_clone_size 8`, `max_slam 35`, `track_frequency 15`, `fast 15`, IMU σ_w 1.4e-4, σ_a 3.9e-3. 발산 문제 글이므로 "잘 되는 설정"은 아님. https://forum.modalai.com/post/22489
  - ModalAI 직원: 기본은 동적 초기화 꺼짐 → 움직이는 중 리셋되면 재초기화 불가, `init_dyn_use true` 권장. https://forum.modalai.com/post/22505
- 개인 PX4 저장소(별 0~3, 신뢰도 낮음): 둘 다 스테레오 + `try_zupt true` + **`zupt_only_at_beginning true`**.
  one_drone: chi2 0, disp 1.0, noise 10, track 40 Hz, num_pts 300, CLAHE. https://github.com/PingTIKES/one_drone/blob/HEAD/deploy/calibration/uav1/estimator_config.yaml
  dofarifa: chi2 0.001, init_dyn_use true, track 30 Hz. https://github.com/dofarifa/Unmanned-Aerial-Vehicle/blob/HEAD/config/openvins/d435/estimator_config.yaml
- 드론 실패 사례: #427 (BMI270 50 Hz IMU → 저자 "200 Hz 이상 권장", 재투영 > 2 px 보정 불량), #513 (고도 100 m 모노 발산), #488 (PX4 비전 모드: OpenVINS 가 jerk 전에는 출력 안 해 이륙 못 함 = 닭과 달걀).
  https://github.com/rpng/open_vins/issues/427 , https://github.com/rpng/open_vins/issues/513 , https://github.com/rpng/open_vins/issues/488
- 모노 + 등속 이동은 스케일이 약해 누적 후 회전 때 큰 갱신 → 스테레오 권장 (저자, #368, #127). https://github.com/rpng/open_vins/issues/368 , https://github.com/rpng/open_vins/issues/127

## 4. 공통 권고 정리

| 주제 | 관례 | 출처 |
|---|---|---|
| IMU 잡음 | Allan 값 ×10~20 (저자 답변으로는 1~2 자릿수) | docs gs-calibration, #172 |
| 온라인 보정 | timeoffset 은 거의 모두 켬, extrinsic 은 대부분 켬(고속 드론·차는 끔), intrinsics 는 업스트림 대부분 켬 | 3.1 표, docs gs-calibration |
| 특징 수 | 모노·스테레오 모두 `num_pts 150~250`(대부분 200), `max_slam 25~50` | 3.1 표, 폰 예제 |
| fast_threshold | 15~30 일반, 고속 FPV 실내 50 | 3.1 표 |
| ZUPT | 지상 차량은 계속, 손·드론·폰은 꺼짐 또는 처음에만 | 3.1 표, 폰 예제, 드론 저장소 |
| ZUPT 실패 모드 | 등속 오판(속도 검사로 보완), 동적 환경에서 시차 판정 실패, IMU 판정은 R 50~100배 필요 | docs update-zerovelocity |
| IMU 주기 | ≥200 Hz | #427 |
| 롤링 셔터 | 모델 없음, 짧은 판독이면 무시 가능; 시각 오프셋 온라인 추정 | #464, docs gs-calibration |
| 초기화(모노) | 정지 → 들어올림(jerk). 폰 예제 2 px @ f≈478 (≈0.24°) | 폰 예제, #342 |

## 5. 우리용 권장 기준선

"권장"은 출처 있는 관례에 맞춘 출발점이다. 우리 데이터로 같은 녹화·같은 채점(A/B)으로 확인하기 전에는 확정 아님.

| 파라미터 | 우리 값 | 권장 | 이유·출처 | 확실성 |
|---|---|---|---|---|
| `try_zupt` | true | **true 유지** (아래 only_at_beginning 과 함께) | true 면 정지 상태에서 jerk 없이 초기화(코드). 폰 예제는 false 이지만 그러면 초기화 방식도 바뀌므로 별도 A/B | 근거 있음 |
| `zupt_only_at_beginning` | false | **true** | 손·폰·드론 설정 전부 true (tum_vi, rpng_aruco, rpng_plane, open_vins_mobile, 드론 저장소 2개). 계속 켜는 건 지상 차량뿐 | 근거 있음 |
| `zupt_chi2_multipler` | 0 (시차만) | 0 유지 (only_at_beginning true 일 때) / 계속 켤 거면 1 + `zupt_noise_multiplier` 50~100 | 시작 정지 구간만 쓰면 시차만으로 충분. 계속 쓰면 문서가 IMU+시차 병용과 R 50~100배 권고 | 근거 있음 |
| `zupt_max_disparity` | 0.5 px/프레임 @60 fps | **0.2 px** (= 업스트림 0.5 px @20 Hz 와 같은 px/s) 또는 30 Hz 추적 시 0.3 | 업스트림 값은 20~31 Hz 기준. 60 fps 에서 0.5 px 는 30 px/s 미만 동작을 정지로 봄 | 환산은 계산, 효과는 추정 |
| `zupt_noise_multiplier` | 10 | 10~50 | 업스트림 1~50, 폰 예제 50 | 범위만 근거 |
| `zupt_max_velocity` | 0.1 | 0.1 | 대부분 0.1 (chi2 0 이면 안 쓰임) | 근거 있음 |
| `init_window_time` | 1.0 (실행) / 1.5 (strict) | 1.0 | 폰 예제 1.0, tum_vi 1.5, 대부분 2.0 | 근거 있음 |
| `init_max_disparity` | 15 px (실행, f≈430 에서 ≈2.0°) / strict 2.25 px | **2~2.5 px (strict 값)** | 폰 예제 2.0 px @ f≈478 (≈0.24°), uzhfpv 2.0, rpng_aruco 2.0. 15 px 는 tum_vi(512x512 어안)·euroc(752x480) 수준이라 우리 화각엔 큼 | 근거 있음 |
| `init_imu_thresh` | 0.3 (strict) | 0.3~0.5 | 폰 예제 0.4, uzhfpv 0.3. ZUPT 켜면 "정지 중 가속 std 상한"으로 쓰임 | 근거 있음 |
| `init_dyn_use` | false | false | 폰 예제·대부분 false; Android 빌드는 동적 초기화 없음 | 근거 있음 |
| `track_frequency` / 입력 fps | 60 | **30 Hz 로 A/B** (2프레임에 1장) | 업스트림·폰 예제 21~31 Hz. 60 Hz + 11 clone = 0.18 s 창 | 추정 (A/B 필요) |
| `max_clones` | 11 | 60 Hz 유지 시 15~20 으로 A/B, 30 Hz 면 11 | 창 시간(0.35~0.5 s)을 업스트림과 맞추려는 것. 계산량 증가 | 추정 |
| `max_slam` | 50 | 50 | 거의 모든 설정 50 | 근거 있음 |
| `num_pts` | 200 | 200 | 폰 예제·대부분 200 | 근거 있음 |
| `fast_threshold` | 15 | 15~20 | 폰 예제 20, 업스트림 15~30 | 근거 있음 |
| `min_px_dist` | 10 | 10 | 폰 예제 10 (640x480) | 근거 있음 |
| `use_klt` / `histogram_method` | true / HISTOGRAM | 같음 | 전 설정 공통 | 근거 있음 |
| `up_msckf_sigma_px` / `up_slam_sigma_px` | 1 | **1.5~2** | 폰 예제 2, uzhfpv·kaist 1.5. RS·압축·자동노출 잔차 흡수 (이유는 추정) | 값 근거 있음 |
| `up_*_chi2_multipler` | 1 | 1 → A/B 로 5 | 폰 예제 5 (나머지 전부 1). 너무 크면 이상치 통과 | 값 근거 있음, 효과 추정 |
| `calib_cam_timeoffset` | true | true | 문서 권장, 거의 전부 true | 근거 있음 |
| `calib_cam_extrinsics` | true | true | 폰 예제 true. 단 S24 p_CinI 초기값은 가정값 | 근거 있음 |
| `calib_cam_intrinsics` | false | false 유지 (A/B 후보 true) | 폰 예제는 true 지만 우리는 왜곡 편 핀홀 + 초점 보정을 이미 함. 모노 온라인 intrinsic 은 들뜰 수 있음 | 추정 |
| IMU gyro ND | 1.0e-3 | 그대로 (Allan ×15) | 문서 10~20배; 폰 예제 1.7e-4 (Allan ×2.6 수준) | 근거 있음 |
| IMU acc ND | 1.0e-2 | 그대로 또는 6e-3 (Allan ×15) | 현재 ×25 로 문서 상한 약간 위. 0.5배에서 lost 기록 있음(make_ov_config 주석) | 추정 |
| IMU gyro RW | 1.0e-4 | 그대로 또는 3e-5 | Allan ×37. 업스트림 1e-5~2e-5 | 추정 |
| IMU acc RW | 1.0e-3 | 그대로 | 업스트림 1e-4~3e-3, 폰 예제 3e-3 | 근거 있음 |
| `update_rate` | 400 (템플릿) | 실제 주기(≈500) | 문서 주석 "Make sure this is correct" (#436 인용 설정) | 근거 있음 |
| 롤링 셔터 | skew 2.9 ms, 클라이언트가 노출 중앙 보정 | 시각 = 노출+판독 중앙, timeoffset 온라인, sigma_px 1.5~2 | OpenVINS 는 RS 모델 없음 (#464) | 근거 있음 |
| 카메라 제어 | (확인 안 함) | AF 고정(초점 고정), AE 노출 짧게(−EV) | 폰 예제 앱 | 근거 있음 |

## 6. 바로 할 A/B (같은 녹화·같은 채점, 한 번에 하나)
1. `zupt_only_at_beginning true` (나머지 그대로) ← 가장 근거 강함.
2. `zupt_max_disparity 0.2` (only_at_beginning false 일 때만 의미).
3. 30 Hz 입력(2프레임 중 1장) vs 60 Hz + clones 20.
4. `up_msckf_sigma_px/up_slam_sigma_px 1.5, 2`.
5. 폰 예제 설정 통째(try_zupt false, init 1.0/0.4/2.0, sigma 2, chi2 5) — 초기화가 jerk 방식으로 바뀌는 섞인 변수임을 기록.

## 7. 못 찾은 것
- ADVIO 등 폰 데이터셋용 OpenVINS 공식 설정: 없음.
- 잘 알려진 PX4 + OpenVINS 레퍼런스 설정: 없음(개인 저장소만). ModalAI VOXL 실제 기본 YAML 값은 공개 문서에 없음.
