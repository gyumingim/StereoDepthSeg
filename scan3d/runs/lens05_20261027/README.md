# lens05_20261027 — 앱의 모든 메인 렌즈 값을 초점 0.5 dpt 보드 보정값으로 교체

만든 시각: 2026-10-27 08:35–(설치 시각은 아래 4절). 브랜치 `lens05-20261027` (integ2-20261026 4035d59 에서 시작), 워크트리 `~/scanner/StereoDepthSeg-lens05`.
사용자 지시 (10-27 08:35): "모든 렌즈는 0.5 dpt 로 맞춰놔".

## 1. 결론
- 앱이 읽는 메인 렌즈 설정(`files/ov/<cfg>/` 17개 폴더 + `files/ov/remap_1920x1080.bin`, `remap_640x360.bin`, `variants.json`)을
  **calcap k12_notan (초점 0.5 dpt, 세트 A, OpenCV k1 k2, 접선 0)** 하나에서 다시 만들었다. 60 fps(c4f60)와 30 fps(c4) 모두 같은 원본에서 해상도만 다르다.
  - 18번째 폴더 `c4_nc05` (c4 + IMU 잡음 ×0.5, 생성 스크립트 밖에서 만든 opt-in, 폰에는 없었음): 옛 imucam yaml 이 옛 c4 와 같았으므로 새 c4 yaml·remap 을 복사 (독립 검증자가 빠진 것을 찾음).
- 바뀐 것은 렌즈 관련 값뿐이다: 각 폴더 `kalibr_imucam_chain.yaml` 의 `intrinsics` 한 줄과 remap 파일. estimator 설정, R_ci, 레버, 시간차, IMU 잡음은 그대로 (diff 로 확인, 17개 폴더 모두 intrinsics 외 차이 0).
- 앱 0.6.29 (vc 35): 렌즈와 무관한 **작은 검사 하나**만 추가 — remap 출력 크기 ≠ 설정 해상도, 또는 remap 원본 크기 ≠ 카메라 프레임이면 시작 시 오류. 맞는 파일이면 동작 같음.
- 값 선택 근거는 보드 보정뿐이다. VIO 결과로 고르거나 조정하지 않았다. PC 재생 ATE 는 "깨지지 않음" 확인용 참고값.

## 2. 무엇을 어디서 가져왔나 (선택 이유)
- 원본: `~/scanner/StereoDepthSeg-calcap/scan3d/runs/calcap_calib_20261026/lens_candidate_20261026.json` (sha256 af5f27aa…, fit 2026-10-26 21:00,
  세트 `recordings/calcap_20261025_214000`, 정지사진 132장 33자세, 노트북 5×3 ChArUco, RMS 0.191 px).
  → 값 그대로 복사해 `scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json` (sha256 21b5f334…).
- 선택 이유:
  - `runs/lenscal_20261027/README.md` 1절 판정 = "지금 값(cal20261024d 또는 calcap k12, fx 1286)은 임시로 쓸 만하다", 하나를 고르지는 않음.
  - 그래서 코디네이터 규칙(권장이 없으면 0.5 dpt 독립 세트 A, OpenCV k1k2 = README 의 "임시 사용" 값)을 따랐다 = calcap k12_notan.
  - 독립 검증(`scratchpad/team/verify_lenscal_20261027.md` 2.1): 이 렌즈를 독립 세트 B 로 채점하면 0.155 px (B 자기 보정 0.145). c4f60 옛 렌즈는 같은 B 에서 0.217 px.
  - 접선(p1 p2) 0 모델: 접선을 풀면 cx 가 5–6 px 흔들림 (lenscal README), OpenVINS radtan 과 호환.
- 확정 아님: fx 계통 불확실성 −0.96 … +0.36 %, r0.8 왜곡 약 2–3 px (노트북 화면 보드). 인쇄 A1 ChArUco 두 세트(lenscal README 2절) 뒤 다시 바꿀 값이다.
- 초점 배율: 렌즈 파일이 0.5 dpt 보정이라 `focus_scale` = 1.0 (옛 세트는 2.22 dpt 렌즈 × 0.98624).
- 생성: `scan3d/ov_bridge/android/make_phone_ov_main.py OUT --lens-file config/lens/std_1920x1080_f0.50_calcap20261026.json` (코드 변경 없음).
  같은 스크립트로 옛 렌즈를 다시 만들면 c4/c4f60 remap(206cfa26…)·top-level remap·모든 yaml 이 폰/저장소 파일과 sha 까지 같음 → 생성 경로 확인
  (예외: 저장소의 c5/c6/c7 yaml 은 그보다 앞선 렌즈로 만든 낡은 파일이었다 — 이번에 함께 새 값으로 바뀜).

## 3. 옛 값 → 새 값
원본 렌즈 (1920×1080, 정류 전)

| | fx | fy | cx | cy | k1 | k2 | p1 | p2 | k3 |
|---|---|---|---|---|---|---|---|---|---|
| 옛 (hs f2.22 cal20261022 × 0.98624) | 1285.95 | 1284.96 | 968.38 | 552.47 | 0.07395 | −0.11746 | −0.00019 | 0.00100 | 0.05764 |
| 새 (calcap k12_notan 0.5 dpt) | 1286.25 | 1286.44 | 965.19 | 553.22 | 0.09033 | −0.07917 | 0 | 0 | 0 |

앱 설정의 정류 핀홀 (`kalibr_imucam_chain.yaml` intrinsics, getOptimalNewCameraMatrix alpha 0 — 왜곡 모양이 바뀌어 잘라내기가 달라짐, calcap README 결과 4 예측과 같음)

| 크기 | 폴더 | 옛 fx fy cx cy | 새 fx fy cx cy |
|---|---|---|---|
| 960×540 | c4, c4f60, c4c20, c4n400, c4s100, c4n400s100c20 | 652.100 652.079 485.002 275.873 | 659.354 658.670 482.344 276.318 |
| 640×360 | base60, n150, n100, c8, n100c8, fps30, fps30n100c8, c2, c5 | 434.507 434.316 323.166 183.744 | 439.340 438.706 321.395 184.041 |
| 800×450 | c6 | 543.303 543.197 404.084 229.809 | 549.347 548.688 401.870 230.180 |
| 1280×720 | c7 | 869.694 869.842 646.837 368.001 | 879.368 878.634 643.293 368.595 |

sha256 (앞 16자리)

| 파일 | 옛 | 새 |
|---|---|---|
| c4/, c4f60/ (+ c4c20 c4n400 c4s100 c4n400s100c20) remap_1920x1080.bin (960×540) | 206cfa2658e43306 | 483168c9723c0ba1 |
| remap_1920x1080.bin (top-level, 640×360) | 2d5812fa06e32a3f | ada7d553955d6afa |
| remap_640x360.bin | 6489f8d101e765a4 | 824623aa4cde4d59 |
| c6/remap_1920x1080.bin | c5c29400818fa51d | b185b32d8eaaeb32 |
| c7/remap_1920x1080.bin / remap_1280x720.bin | 8e0d6e084f6a511b / 04abf80ebe59ed54 | bb9c6c1a44a83adf / 5f24817d30054708 |
| c4/, c4f60/ kalibr_imucam_chain.yaml | f77576c61963865 | 58b9a1af46461eb3 |
| variants.json | 270f2f490a9a8050 (재생성) / ad129fc5a01372df (폰·main) | 953c9844d72aba42 |

remap 검사 (`chk_remap.py` → `chk_remap_new.txt`): 17개 폴더 모두 remap 출력 크기 = yaml 해상도, 원본 1920×1080 (v2), 격자 점마다
remap 좌표 = K_rect⁻¹q 를 새 렌즈로 투영한 값과 최대 0.016 px (1/32 px 고정소수) 차이, 출력 화면의 99.5 % 이상이 원본 안.

## 4. 폰 (설치)
- 순서 (`phone_install.sh`): 폰 잠금(`scratchpad/team/phone_lock.d`) → 옛 세트를 `files/ov/bak_f222_20261024/` 로 **이동**(지우지 않음) → 새 세트 push → sha 확인 → 0.6.29 설치.
  조건: 화면 꺼짐 또는 런처 앞, 카메라 클라이언트 [], SKIN < 36 °C, ov_bridge/앱 프로세스 0. 카메라·앱은 켜지 않음.
- 결과 (`install.log`): 08:44–09:13 사용자가 폰을 써서 대기 → 09:13 화면 꺼짐(Dozing)·카메라 []·SKIN 34.9·프로세스 0 → 옛 20개 항목을 `bak_f222_20261024/` 로 이동,
  새 세트 67개 파일 push (c4_nc05 포함), 폰 sha = 3절 새 값과 같음, **09:14:35 0.6.29 설치 성공** (versionCode 35, versionName 0.6.29-lens05-20261027 확인). 카메라·앱은 켜지 않음. 폰 실주행은 아직 안 함.
- 바꾸지 않은 것 (목록): `files/ov/` 바로 아래 `estimator_config.yaml` / `kalibr_*.yaml` / `lens0u_remap.bin` (2026-10-18, **렌즈 2 = 다른 카메라** 640×480, 0.5 dpt 보드 보정 없음),
  `run_*` 기록, `live_stream*.bin`. PC 쪽 기본 렌즈 표 `lens_calib.DEFAULT_LENS` (벤치용, 앱 아님)도 그대로.
- 롤백:
  1. 렌즈만: `adb shell "cd /sdcard/Android/data/com.scan3d.arlat/files/ov && mkdir bak_calcap05 && for x in <폴더·파일 20개>; do mv \$x bak_calcap05/; done && mv bak_f222_20261024/* ."`
     (또는 intent `ov_cfg bak_f222_20261024/c4f60` 로 옛 폴더를 직접 지정).
  2. 앱: `adb install -r -d scratchpad\team\lens05\apk\arlat_0.6.28_rollback.apk` (vc 34, sha 97921f52…). 0.6.28 은 크기 검사만 없고 렌즈 파일은 폴더에서 읽으므로, 앱 롤백만으로 렌즈는 안 돌아간다.

## 5. 앱 0.6.29 (vc 35, `0.6.29-lens05-20261027`)
- `OvRunner.checkRemapSize`: 쓰는 remap 의 출력 크기가 run 폴더 imucam yaml (estimator 설정의 `relative_config_imucam`, 기본 `kalibr_imucam_chain.yaml`) resolution 과 다르면, v2 remap 의 원본 크기가 카메라 프레임(W×H)과 다르면, 파일 길이가 형식과 다르면 시작 시 오류.
  - 고친 위험: c4f60 폴더에 remap 이 없으면 `files/ov/remap_1920x1080.bin`(640×360)으로 조용히 대체되고 설정은 960×540 이라 틀린 K 로 VIO 가 돌던 경로 (OvRunner 411–413, ov_bridge 1730 은 크기 대조 없음).
  - 브리지(네이티브)는 고치지 않았다 → libov_bridge.so 가 0.6.28 과 sha 같음 (6f29ce70…). PC 재생에는 이 검사가 없다(남은 일).
  - 부수 효과: 프로필 `rec20261025` 를 `ov_cfg` 없이 메인 카메라로 돌리면 `files/ov` 바로 아래 렌즈 2 설정(640×480) + 메인 remap(640×360)이 섞여 원래도 틀린 조합이었는데, 이제 시작 시 오류가 난다 (`ov_cfg c4` 를 줘야 함; `tools/ov_phone_run.sh` 는 항상 줌).
- PC 시험 (`jt/T.java`, OvRunner 의 메서드 본문을 그대로 붙여 javac): 9/9 통과 — 새·옛 c4f60, 640×360 변형, c7, 렌즈 2 v1 파일은 통과, 640×360 대체 + c4f60 yaml 은 오류, 원본 크기 불일치는 오류.
- APK: aapt vc 35 / 0.6.29-lens05-20261027, 같은 개발 인증서, sha 5feb3e53…, `scratchpad\team\lens05\apk\arlat_0.6.29.apk`.
  libarloop.so 는 같은 소스를 다른 경로에서 다시 빌드해 sha 만 다름 (크기·문자열 같음).

## 6. PC 재생 ("깨지지 않음" 확인, 정확도는 참고)
- 방법 (`phone_cfg_replay.py`, `pc_replay.sh`): 폰에 올리는 폴더 그대로(yaml + remap)를 PC 브리지(integ2 고정본 81e547e9…)에 `--remap` 으로 주고,
  std60 녹화의 원본 1920×1080 회색 프레임 + IMU 를 vio_bench_hs 공급 방식으로 넣음. `--profile rec60 --drop-late 0 --loop 0`. 채점 = vio_bench_hs.evaluate (카메라 GT).
  폰과 다른 점(옛·새 같음): 프레임이 이미 녹화별 dt 로 IMU 시계에 맞춰져 있는데 yaml 의 timeshift 2.17 ms 도 그대로; R_ci 는 폰 평균값.

| 녹화 | 폴더 | 렌즈 | 넣은 프레임 | 추적 | ATE cm | 회전 중앙 ° | RPE 1 s cm |
|---|---|---|---|---|---|---|---|
| hs_20261024_165506_std60 | c4f60 (60 fps) | 옛 | 1690 | 1394 (82.5 %) | 1.35 | 0.46 | 0.76 |
| 〃 | c4f60 | 새 | 1690 | 1394 (82.5 %) | 1.63 | 0.51 | 0.80 |
| 〃 | c4 (30 fps) | 옛 | 845 | 694 (82.1 %) | 1.39 | 0.55 | 0.74 |
| 〃 | c4 | 새 | 845 | 694 (82.1 %) | 1.44 | 0.46 | 0.71 |
| hs_20261024_173432_std60 | c4f60 | 옛 / 새 | 1700 | 0 / 0 (초기화 안 됨, 둘 다) | — | — | — |

- 판정: 새 렌즈로 깨지지 않음 (포즈 수·추적 비율이 옛 렌즈와 같음, remap 960×540 정상 로드). 173432 는 옛 렌즈도 초기화가 안 되는 이 하네스의 문제(렌즈 무관).
- 정확도: 한 녹화 1회씩이라 차이(±0.3 cm)는 판단 근거가 아니다. 렌즈 선택에 쓰지 않았다.

## 7. 검증 / 남은 일 / 가정
- 독립 검증자 (읽기 전용, `scratchpad/team/lens05_verify/`): 주장 7개 중 6 맞음, 1 과장(c4_nc05 누락 → 고침). 지적(imucam 이름 고정 → relative_config_imucam 사용으로 고침).
  c4f60 PC 재생 ATE 1.35 → 1.63 cm 는 1회씩 값이라 판단 근거 아님 (렌즈 선택에 안 씀).
- 폰에 실제로 깔린 옛 파일 = 저장소/재생성과 같음을 sha 로 확인 (c4·c4f60 remap 206cfa26, c4f60 yaml f77576c6, variants ad129fc5 = main 워크트리).
- R_ci·레버·시간차는 옛 렌즈로 맞춘 녹화 정렬값 그대로 (lenscal README: 주점·왜곡이 바뀌면 R_ci 가 약 0.4–0.6° 어긋날 수 있음, 추정). 렌즈 확정 뒤 다시 정렬할 것.
- ov_bridge 쪽 크기 검사(PC 재생용)는 네이티브 재빌드가 필요해 이번에 안 함.
- 인쇄 A1 보드 세트로 렌즈를 확정하면 같은 스크립트로 다시 만든다.

파일: `README.md`, `chk_remap.py` + `chk_remap_new.txt`, `phone_cfg_replay.py`, `pc_replay.sh` + `pc_replay.log`, `phone_install.sh` + `install.log`, `jt/` (T.java, run_t.sh, run_t.log), `build_apk.sh` + `apk_build.txt`, `mklens.py`.
옛 설정 사본: `scan3d/config/phone_ov_main_f222_20261024/` (저장소의 옛 yaml·variants.json, *.bin 은 원래 커밋 안 함 — 옛 bin 은 main 워크트리 `scan3d/config/phone_ov_main/` 와 폰 `bak_f222_20261024/`).
