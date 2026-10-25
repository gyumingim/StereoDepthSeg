# integ 2026-10-25 — 역할 브랜치 통합 (branch `integ-20261025`, 앱 0.6.22)

## 결론 (먼저)
- ovopt(0.6.19 imuprio 포함) 위에 **ovdrone, ovheat, esp32fc, ovacc(결과만), ovrobust** 를 합쳤다. 새 기능은 모두 옵션이고
  **옵션을 안 주면 이전과 같다**: PC 에서 ovopt 기준 빌드와 포즈가 17자리까지 같음 (하네스 845/845, raw80 2327/2327,
  GT 45칸 포즈 해시 45/45). 0.6.20 / 0.6.21 / 0.6.22 빌드마다 다시 확인.
- **추천 묶음 `rec20261025`** (앱 `--es profile rec20261025`, 브리지 `--profile rec20261025`): W3 계산 옵션 + idle_spin 2 + 20 ms 창
  + drop-late 50 + health + (0.6.21, 사용자 19:40) IMU 우선 전송 + 200 Hz 예측(예측 스레드). **기본값은 안 바꿨다.**
  PC 정확도: GT 42칸 ATE 중앙값 0.5914 cm, 발산 8칸 = 기준과 같음 (짝 -0.000 cm, 34쌍 중 21 개선).
  라이브 (사용자, 0.6.21, 30 fps, 각 1회): 처리 p50/p95 10.9/21.2 ms · 11.5/15.9 ms, 버림 0, 예측 포즈 나이 p50 4.0-4.1 / p95 7.2-7.3 ms
  (228.7 Hz), 원점 복귀 29.5 cm/16.0 m (빠른 손), 2.6 cm/11.7 m (느린 손).
- **60 fps (사용자 20:00)**: 설정 `c4f60` + 앱 묶음 `rec60` (0.6.22) 만들고 폰에 설치·설정 올림. PC 에서 std60 녹화 165506 으로
  초기화·추적 확인 (ATE 1.57-1.59 cm vs 30 fps 1.39-1.74, 3칸). **라이브 60 fps 는 아직 안 함** (사용자가 함).
- 드론 링크 127 Hz 문제: 원인 = 옛 경로(메인 루프, "직전 예측 후 5 ms" 규칙 + 프레임 처리 중 멈춤). 예측 스레드가 VIOP/MAVLink 도
  보내게 함: PC 실시간 재생 2회 124/127 Hz → 229/229 Hz (200 Hz 격자 + 프레임 직후 갱신), 간격 p95 20-22 → 6 ms.
- 폰 IMU 송신기 (VIOP 0x06, 앱 별도 스레드, 기본 꺼짐) 구현: Java = C 헤더 바이트 2000/2000. **USB(ESP32) 경로는 장치가 없어 미검증.**

## 합친 것
| 브랜치 | 커밋 | 내용 | 충돌 |
|---|---|---|---|
| ovopt-20261025 | 35cbf60 | 기준 (0.6.18 ovopt + 0.6.19 imuprio) | - |
| ovdrone-20261025 | c1d9f2a | VIOP v2 + HEARTBEAT 0x02, MAVLink, rs_derot/rs_row_ppm/rs_postinit | main.cpp, OvRunner, MainActivity: 양쪽 유지 |
| ovheat-20261025 | 988f796 | `--klt-pyr N` (기본 꺼짐), 속도 결과 | usage 문자열 |
| esp32fc-20261025 | 8dd9b03 | ESP32 C 펌웨어 + PC 점검 389개 | 없음 |
| ovacc-20261025 | 0ab2d82 | 정확도 결과만 | 없음 |
| ovrobust-20261025 | 3986b51 | `--health`, `--init-still`, `--recover`, `--test-blank` | main.cpp, ovopt.h/.cpp, 앱, ov_phone_timing.py: 양쪽 유지 |
통합에서 한 일: viop_fc_ext.h → viop_proto.h (ESP32 사본 동일, 389/389), 예측 스레드→드론 링크, `OV_KLT_PYR` 환경변수 무시,
`--profile`, c4_nc05 / c4f60 설정, 앱 0.6.20→0.6.22, IMU 송신기.

## 옵션 (전부 기본 꺼짐)
| 어디 | 옵션 | 내용 |
|---|---|---|
| 브리지 | `--profile rec20261025` | `--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol --idle-spin 2 --idle-spin-ms 20 --drop-late 50 --health 1 --predict 2 --predict-hz 200 --pred-thread 1` (먼저 적용, 따로 준 옵션이 이김; --tcp 나 --predict 0 이면 pred-thread 빠짐) |
| 브리지 | `--profile rec60` | 위 + `--idle-spin-ms 0` (20 ms 창이 60 fps 프레임 간격 16.7 ms 보다 길어 → 전 구간 spin 을 명시) |
| 앱 | `--es profile rec20261025` | 위 값 + `imu_prio 1` (IMU 우선 + 이미지 64 KB 조각). 따로 준 intent 가 이김. 없으면 0.6.19 동작 |
| 앱 | `--es profile rec60` | rec20261025 + `fps 60` + `ov_cfg c4f60` + `idle_spin_ms 0`; drop_late 50 ms 유지 (시간 기준 지연 한도 같게, 60 fps 에선 3 프레임) |
| 설정 | `c4f60` | c4 와 같고 `track_frequency 60` 만 다름 (생성기에서 fps 는 이것만 바꿈). clones 15 유지 = 0.25 s 창 (60 fps 옛 기준 11 = 0.18 s 보다 김) + 프레임당 계산량 c4 와 같음 (clones +5 마다 +1 ms, 폰 예산 16.7 ms 가 빠듯해 늘리지 않음). init_max_disparity 22.5 px 는 1.0 s 창 전체 기준이라 그대로 |
| 설정 | `c4_nc05` | c4 + IMU 잡음 4값 ×0.5 (ovacc 후보, 라이브 A/B 전) |
| 브리지 | `--pred-thread 1` + `--viop-udp`/`--mav-udp` | 예측 스레드가 드론 링크로도 보냄 (이전엔 이 조합에서 안 보냄) |
| 앱 | `viop_udp HOST:PORT`, `viop_hb_hz` | 브리지 `--viop-udp` (predict 1/2 필요), `<run>/viop_log.csv` |
| 앱 | `viop_imu udp:HOST:PORT` 또는 `usb`, `viop_imu_uncal 1`, `--es mode imu_tx` | ViopImuSender (자이로마다 36 B 프레임), imu_tx = 카메라 없이 송신기만 |
| 브리지 | ovrobust `--health` 등, `--klt-pyr`, rs_*, `--mav-udp` | 각 역할 README. `--recover`, `--init-still`, rs_derot 는 꺼 둘 것 (나빠짐) |

## 근거 (반복, 입력)
| 검사 | 입력 | 결과 | 반복 |
|---|---|---|---|
| PC 동일성 (`ident_pc.log`, `_0621`, `_0622`) | 하네스 h_L0.4nc (845), raw80 (2327), 17자리 | 기본/all_exact/pred-thread+viop/health/OV_KLT_PYR=3 → ovopt 기준과 IDENTICAL; rec20261025·rec60 = ovopt W3+spin 실행과 IDENTICAL (`h`, `late_dropped` 필드 제외). 예측 줄 내용 자체는 비교 안 함 | 빌드 3개 각 1회 (결정적) |
| PC ATE (`score_B0_PR.json`, `ate_vs_ovopt.txt`) | HS GT 42칸, ovopt 러너 | B0 0.5914 / 발산 8 (ovopt B0 와 포즈 해시 45/45); PR 0.5914 / 8, 짝 -0.000, 21/34 (= ovopt W3, 차이는 제외 녹화 213955 o1·o2 만). PR = 0.6.20 묶음 (예측 없음; 예측은 포즈 안 바꿈 = 동일성 검사) | 칸당 1회 |
| VIOP 속도 (`viop_rate_pc.log`) | 하네스 실시간, UDP 루프백, PC 공유 | 옛 124/127 Hz, p95 22/20 ms; 새 229/229 Hz, p95 6/6 ms, 손실 0. 같은 t_pred 쌍 312/303개 = 프레임 직후 새 상태로 다시 낸 것 (FROM_FRAME), 감소 0. 내용은 옛 경로와 비트 동일 (검증자) | 2회 |
| IMU 프레임 (`imu_tx_test/`) | 2000개 | Java=C 2000/2000, 디코드 ½ LSB 이내, 스트림 파서·viop_rx.py OK | 1회 |
| 60 fps PC (`f60_pc.py`, `score_std_F30_F60.json`) | std60 165506·173432 × 오프셋 3, 라이브식 (histogram NONE) | 165506: F60 추적 1394 프레임, ATE 1.58/1.57/1.59 cm (F30 1.74/1.74/1.39); 173432: 30·60 둘 다 초기화 안 됨 (알려진 1.0 s 창 문제). PC 단일 스레드 ms 중앙값 21 (F60) — 폰 시간 아님 | 칸당 1회 |
| 카메라 60 fps | 폰 기존 run_20261024_150327_main60_base60 frames.csv | 1920x1080 에서 프레임 간격 평균 16.667 ms (1699장) = 60 fps 전달 확인 | 기존 1회 |
| 라이브 (사용자) | run_20261025_195227 / 195554_main30_c4 (`runs/phone_ov_timing/`) | 위 결론 참고. imu_prio + pred_thread 의 첫 라이브 근거 | 2회 (움직임 다름) |

## 폰
- 설치: 0.6.20 19:24, 0.6.21 19:41 (코디네이터도 0.6.20 설치), **0.6.22 20:08** (versionCode 28, aapt/apksigner 확인). c4f60 은
  files/ov/c4f60 (yaml 3개 + c4 의 remap 복사, sha 같음).
- 카메라 없는 재생 비교: I_1 (기본 vs 묶음) 은 기본값 첫 실행이 45 초에 AP 50.9 °C 로 중단 (기본값은 무겁고 밀림), I_2 (현재 라이브
  조합 vs 묶음) 는 사용자 라이브 시험 때문에 시작 직후 멈춤 → **폰 재생 비교 결과 없음**. 대신 사용자 라이브 2회가 있음.

## 60 fps 라이브 명령 (사용자가 실행; WSL, `~/scanner/StereoDepthSeg-integ`)
```
LIVE=1 SEG=still0:0:8,hand:8:58,still:58:60 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4f60 62 --es profile rec60
```
(스크립트가 c4f60 의 track_frequency 로 `--ei fps 60` 을 붙임. 열 보호: 시작 SKIN<36, 앱 40 °C 자동 정지 그대로.)
위험 (추정): 30 fps 라이브 p95 16-21 ms → 60 fps 예산 16.7 ms 와 비슷/초과 → 밀리면 drop-late 50 이 프레임을 버림. CPU 일이 약 2배라 더 뜨거움.

## 아직 확인 안 된 것
- 60 fps 라이브 (시간·밀림·정확도·열), c4f60 의 clones 15 선택 (근거 = 계산 예산, 정확도 비교 없음).
- ViopImuSender USB(CDC): 장치 없음. UDP 경로도 폰에서 안 돌림. `stop()` 이 UI 스레드에서 CSV 를 써 긴 실행에서 잠깐 멈출 수 있음 (낮음).
- 폰에서 예측 스레드 + VIOP 실제 속도 (PC 만). heartbeat 는 메인 루프 → PC 최대 간격 99-122 ms (ESP32 제한 250 ms 안).
- viop/mav 로그의 "send - newest IMU receive" 는 예측 스레드 모드에서 메인 스레드 지연을 재므로 링크 지연이 아님.
- `--recover-full` 오프셋이 켜지면 드론 링크 p/v 와 자세가 어긋남 (기본·묶음에선 안 씀, 낮음).
- c4_nc05: 오프라인 이득이 잡음 수준, 라이브 A/B 필요.

## 되돌리기
```
adb -s 100.79.156.20:5555 install -r -d C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\arlat_0.6.19.apk
# 0.6.21 / 0.6.20: ...\scratchpad\integ_stage\arlat_0.6.21_integ.apk / arlat_0.6.20_integ.apk
# 코드: 브랜치 따로 (gyumingim, ovopt 그대로). 묶음 intent 를 안 주면 0.6.19 동작. 폰 설정 지우기: files/ov/c4f60 폴더만 삭제.
```

## 파일
`ident_pc.sh`, `ate_run.py` (조건 PR), `viop_rate_pc.sh`, `imu_tx_test/`, `f60_pc.py`, `build_apk.sh`, `phone/` (러너, 목록 I_1/I_2), `cmp2.py`.
