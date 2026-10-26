# integ2 2026-10-26 — 밤샘 최종 통합 (branch `integ2-20261026`, `calcap-20261025` 661048d 에서 시작)

## 결론
- 합친 것: **droplatefix** (필수 안전 수정), **esp32fix**, **ovupd**, **ovgyro**. **ovpipe·ov60eval 코드는 안 합침** (ovpipe = 멈춤·교착, ov60eval = 결과뿐 → README 만 `ov60eval_README_reference.md` 로 복사).
- 충돌 2곳 해결: `main.cpp` VIOP heartbeat (esp32fix `--viop-hb-src` ↔ droplatefix lost 표시 → 두 경로 모두 `lost_ || late_lost_ || hb_stale()`), `ovpatch/ovopt.cpp` 옵션 표 (ovupd 3개 + gyro_klt 모두 넣음).
- 새로 넣은 것 (통합 담당): `--pipe 1` 거부 (gyro_klt 와 같이 주면 전용 오류, rc 2), PC 빌드(OV 소스 없음)에서 ovupd 옵션도 거부, 프로필 ovopt 변경, 앱 0.6.26.
- **앱 0.6.26-integ2-20261026 (vc 32) 폰에 설치함 (06:14:17)**. calcap 모드 시작·정지 확인 (아래).

## 2026-10-26 아침: `gyro_klt=5` 를 프로필에 넣음 (사용자 결정) — 앱 0.6.27 (vc 33)
- 바꾼 것: `--profile rec20261025` / `rec60` 의 ovopt 와 앱 profile 기본 `ov_opt` = `all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5`
  (`main.cpp`, `MainActivity.java`, 매니페스트 vc 33 / 0.6.27-integ2-20261026). 직접 준 `--ovopt` / intent `ov_opt` 가 이김 (그대로).
  프로필 + `--pipe 1` 은 이제 gyro_klt 전용 오류 (rc 2) — `--pipe 1` 은 원래도 거부. calcap 코드 변경 없음.
- PC 동일성 (`gyro/ident_gyro.{sh,log}`, 17자리): 프로필 없음 harness 845/845·raw80 2327/2327 = integ 와 IDENTICAL; 옛 integ 프로필(ovopt 직접) = integ IDENTICAL;
  gyro 없는 ovopt 직접 (rec20261025·rec60) = 0.6.26 빌드 프로필과 IDENTICAL; 새 프로필 = 직접 `...,gyro_klt=5` (새 빌드·0.6.26 빌드 둘 다) IDENTICAL, 2회 동일.
  새 프로필은 raw80 포즈 323 부터 다름 (gyro 효과, 예상대로). raw80 gyro 통계: 시작점 오차 평균 1.82 px, 상한 걸린 점 131, 이동 거부 프레임 4.
- GT 42칸 (ov60eval 자, `--drop-late 0`, 칸당 1회 × 시작 3; `gyro/score_gyro.json`, `gyro/ate_score_gyro.txt`; N30/NL30 = 0.6.26 빌드 결과 재사용):

  | 조건 | 발산 | ATE_w 중앙 (전체 정상칸) | 같은 칸 중앙 | 짝 차 중앙 | 222747_o0 |
  |---|---|---|---|---|---|
  | P: N30 → **G30** | 8 → **3** (131326_o0/o1, 154320_o0; 새 발산 0) | 0.5914 → 0.5965 | 34칸 0.5914 → 0.6026 | +0.012 cm (14/34 좋음, 녹화 5/12) | 0.98 → **1.68** |
  | L: NL30 → **GL30** | 4 → **3** (131326_o0/o1/o2; 새 발산 0) | 0.5597 → 0.5342 | 38칸 0.5597 → 0.5392 | −0.013 cm (27/38, 녹화 9/13) | 1.13 → 1.58 |

  구제된 칸: P = 214550 o0/o1/o2 (0.35/0.43/0.42 cm), 154320 o1/o2 (2.37/2.83) = ovgyro PG5 와 같은 5칸; L = 214550_o0 (0.50).
  정상 칸 정확도 차이는 잡음 범위 (ovgyro 와 같은 결론). 222747_o0 은 P·L 모두 악화 (ovgyro 에서 본 악화 재현); 222747 o1/o2 는 P 좋아짐 (1.14→1.01, 1.26→0.92), L 나빠짐 (0.86→1.13, 1.31→1.70).
- rec60 + gyro (처음 시험, std60 board 165506 × 시작 3, 라이브식 histogram NONE, 60 fps): **초기화·추적 정상** — 3/3 발산 없음, 초기화 시각 같음
  (4.93/3.93/2.93 s), ATE_w 1.58/1.57/1.59 (gyro 없음 NL60) → 1.50/1.55/1.70 cm (GL60). 173432 은 gyro 유무·옛 ov60eval L60 모두 초기화 안 됨 (기존 문제, gyro 무관).
- 앱 0.6.27: aapt vc 33 / 0.6.27-integ2-20261026, apksigner 같은 개발 인증서, APK 안 .so·classes.dex 에 새 ovopt 문자열 확인 (`gyro/build_apk_gyro.log`).
  **설치함 08:51:53** (`adb install -r` Success, dumpsys: vc 33 / 0.6.27-integ2-20261026; 카메라 `[]`, 브리지·앱 프로세스 없음, 실행 안 함).
  (처음 08:26·08:47 시도는 폰 adb 연결 안 됨 → 코디네이터가 다시 연결해 준 뒤 설치.) calcap 은 코드 변경 없음, 설치 뒤 시험은 안 함.
  APK = `scratchpad\integ2\apk\arlat_0.6.27.apk` (sha d90f7ce2), 되돌리기 APK = `scratchpad\integ2\apk\arlat_0.6.26_rollback.apk` (sha 3330e758, vc 32).
- 확인 안 된 것: 폰 ARM 동일성, 라이브 카메라·빠른 손동작, 폰 시간 비용 (ovgyro: 모드 3 에서 1 ms 미만 차이는 못 봄).
- 되돌리기: intent `ov_opt "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse"` (브리지 `--ovopt` 같은 값) = 0.6.26 프로필과 PC 비트 동일, 또는 0.6.26 APK `adb install -r -d`.

## 2026-10-26 저녁: 루프 닫기 (ovloop BoW v1) 합침 + **rec60 에만** 켬 — 앱 0.6.28 (vc 34)
- 합침: `ovloop-20261026` (0d71a86) → merge 커밋 7c02ad3 (충돌 없음, `--loop` 기본 0). 근거·평가는 `runs/ovloop_20261026/README.md`.
- 사용자 결정 (코디네이터 경유): **60 fps 만** → `--profile rec60` 와 앱 profile `rec60` 에만 `--loop 1`. **`rec20261025` (30 fps) 는 안 바꿈.**
- 브리지 (`main.cpp`): rec60 이 `a.loop = 1`. 어휘 = `--loop-vocab`, 없으면 설정 옆 `voc_k10L4.voc` → 한 단계 위 → (PC 빌드만) 소스 트리
  `ov_bridge/loop/voc_k10L4.voc` (`OVLOOP_SRC_VOC`, CMakeLists); 못 찾으면 rc 2. `--loop 0` = 루프 없는 rec60. 프로필 루프 + `--recover N` → 루프만 끄고 알림
  (직접 준 `--loop 1` + `--recover` 는 예전처럼 rc 2).
- 앱: intent `loop` (기본 0; rec60 은 1, `recover` 가 있으면 0). 어휘는 **APK assets 에 넣음** (`build.sh` 가 `-A`, 528 KB, 압축 376 KB) →
  실행 때 `files/ov/voc_k10L4.voc` 로 복사 (없거나 크기 다를 때만, .tmp 후 rename). 브리지에 `--loop 1 --loop-vocab <그 파일> --loop-log <run>/loop.jsonl`.
  meta.json / perf 에 `loop`. APK 8.70 → 10.65 MB (대부분 루프 코드가 든 libov_bridge.so).
- 동일성 (std60 board 165506·173432 × 시작 3, vio_bench 공급, 60 fps, `--pose-precision 17`, `loop60/cmp_id.txt`): 0.6.27 빌드 rec60 (O60) vs
  0.6.28 rec60 `--loop 0` (N60) **6칸 모든 pose 줄 동일** (실질 증거는 165506 3칸 1690/1630/1570 줄; 173432 3칸은 VIO 초기화 안 됨 = 전부 init 줄); vs 루프 켬 `--loop-sync 1` (R60) 와 라이브식 `--loop-sync 0` (A60) = **"map" 키 말고 모든 줄 동일**
  (실제로 다른 키는 `ms` 하나; pose 줄만 비교 — pred 줄의 map 과 VIOP MAP_ODOM 출력은 이 검사에 없음). raw80 / harness 입력은 WSL 크래시로 사라졌고 폰에서 다시 받기는 ~0.7 MB/s (4.8 GB ≈ 2 시간) + 사용자가 폰 사용 중이라 중단 → **raw80·harness 동일성은 이번에 안 함**
  (ovloop 가 raw80 2327 에서 loop on/off VIO 필드 동일 확인한 것이 있음).
- rec60 루프 결과 (`loop60/score_print.txt`, 각 1회, 결정적 sync): 165506 ×3 ATE VIO 1.50/1.55/1.70 → 지도 1.43/1.36/1.58 cm, 루프 에지 2+8+6 = 16 (GT 확인 16), **틀린 루프 0**
  (최대 3.13 cm). 키프레임 49/47/47, 루프 스레드 KF 당 p50 8.3–9.1 ms (PC). 173432 은 VIO 자체가 초기화 안 됨 (예전과 같음, 루프 0). 지도 쪽이 나빠진 값도 있음: 회전 중앙 o1 0.466→0.512°, o2 0.415→0.506°, o2 RPE1s 0.83→0.88 cm.
- 거부·대체 검사 (`loop60/refusal_checks.txt`): 설정 옆 어휘 사용 / 소스 트리 대체 / `--loop 0` / rec20261025 루프 없음 / rec60+recover → 루프 끔 / 직접 loop+recover rc 2 / 없는 어휘 rc 1.
- ESP32 (코드 읽기): `esp32fc/main/link_task.c` `handle_phone_frame` 은 POSE·HEARTBEAT·IMU 만 처리, 0x07 은 아무 동작 없이 버림 (파서 한도 86 B > 44 B, 생존 판정에도 안 씀).
- 메모리·CPU (추정): KF 약 30 KB (코디네이터 수치, 직접 측정 안 함 = 추정), 60 fps std60 움직일 때 약 2 KF/s → 약 3.6 MB/분, 멈춰 있으면 KF 안 생김 (0.25 s·8 cm·8° 규칙). **KF 상한 없음** → 긴 비행은 계속 늘어남.
  폰 루프 스레드 35 ms/KF p50 (ovloop P1, 30 fps 재생) → 2 KF/s 면 한 코어의 약 7 % (추정, 60 fps 폰 측정 없음).
- 앱 0.6.28: aapt vc 34 / 0.6.28-integ2-20261026, 같은 개발 인증서, assets/voc_k10L4.voc sha 2d6b5cec = 저장소 파일 (`loop60/apk_build.log`).
  APK = `scratchpad\integ2\apk\arlat_0.6.28.apk` (sha 97921f52), 되돌리기 = `scratchpad\integ2\apk\arlat_0.6.27_rollback.apk` (sha d90f7ce2, vc 33) `adb install -r -d`.
  **설치함 18:51:51** (`adb install -r` Success; dumpsys vc 34 / 0.6.28-integ2-20261026; 카메라 `[]`, 브리지·앱 프로세스 0, 실행 안 함). 18:34–18:49 는 사용자가 폰 사용 중이라 기다림 (런처 화면 18:50 확인 뒤 설치, `loop60/install_0628.log`).
- 되돌리기 (설치 안 바꾸고): intent `loop 0` (브리지 `--loop 0`) = 0.6.27 rec60 과 같은 VIO (위 동일성).
- 확인 안 된 것: 폰 ARM 에서 rec60 루프 (실행 금지), 60 fps 폰 비용, 긴 비행 메모리, raw80·harness 동일성, 앱의 어휘 복사 코드 실제 실행.

## 지금 프로필 (`--profile rec20261025` / `rec60`, 앱 intent `profile` 도 같음)
`--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5` (gyro_klt=5 는 0.6.27 부터) `--idle-spin 2` `--idle-spin-ms 20` (rec60: 0) `--drop-late 50` (= **mode 1** 기본: cap 3, 창 3 s, pred-stale 500 ms 자동) `--health 1` `--predict 2 --predict-hz 200 --pred-thread 1`. 앱은 + `imu_prio 1`, rec60 = 60 fps + c4f60.
- **0.6.28 부터 rec60 만 + `--loop 1`** (어휘 기본 찾기, 위 절). rec20261025 는 루프 없음.
- **0.6.27 부터 ovopt 끝에 `,gyro_klt=5` 추가** (위 절). 넣지 않음: `par_cov` (폰에서 더 느림), `init_gemm`.
- 옛 프로필 (integ-20261025) = 같은 프로필 + `--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol` 를 직접 줌 (앱: intent `ov_opt` 같은 값). 옛 drop-late 규칙 = `--drop-late-mode 0`.
- 선택 옵션만 (기본 꺼짐): `--viop-hb-src 1` (앱 intent `viop_hb_src 1`), ESP32 펌웨어 `openloop_cut_mode 1` (기본 0).

## 증거 (입력, 반복)
| 항목 | 입력 / 방법 | 결과 | 반복 |
|---|---|---|---|
| PC 동일성 (새 플래그 끔) | integ2 PC 빌드 a0311486 vs integ 14de3b8 빌드 32cf9bb2, pose-precision 17, `ident/` | harness 845/845 IDENTICAL ×3 (plain, pred-thread+VIOP, +`--viop-hb-src 1`); raw80 2327/2327 IDENTICAL ×4 (프로필 없음, 옛 rec20261025, 옛 rec60, `--drop-late-mode 0`; 모두 `--drop-late 0`) | 1회씩 (결정적), 검증자가 harness 2쌍 재실행 동일 |
| 새 프로필 (sym_inplace+ekf_sparse) | raw80 | 포즈 333 부터 다름, 최대 \|Δp\| 4.1e-6 cm, 상태 차이 0; 같은 설정 2회 = 2327/2327 동일 | 2 |
| 42칸 GT ATE (ov60eval 러너·자 그대로, `--drop-late 0`) | `integ2_ate.py`, `score.json`, `ate_pose_cmp*.txt` | 옛 프로필 O30 = ov60eval P30 **45/45칸 포즈 동일**, OL30 = L30 45/45 동일 (자 재현). 새 vs 옛: P 발산 8→8 (같은 칸), ATE_w 중앙 0.5914→0.5914, 짝 차 중앙 −0.0 (19/34 좋음 = 잡음); L 발산 4→4, 0.5597→0.5597. 정상 칸 포즈 차 최대 9.1e-3 cm (P) / 7.8e-3 cm (L), ATE 차 최대 2e-4 cm. 발산 칸은 둘 다 발산 (값은 혼돈적으로 다름) | 칸당 1회 × 오프셋 3 |
| drop-late, 폰 ARM (I_1, 카메라 없음) | feeder3v 영구 +400 ms 이동 (t=25 s), 45 s, `--profile rec20261025` | **옛 mode 0: 600/600 버림, 25 s 뒤 포즈 0, h>0.5 s 인데 "tracking" 예측 3599줄 (최대 20.0 s)** → 결함 재현. **새 mode 1: 69 버림 + 22 forced, t=28.03 에 re-anchor (+400.0 ms), 25 s 뒤 tracking 포즈 531, 예측 h 최대 0.168 s** = PC (75+22, 3.03 s) 와 같은 동작 | 각 1회 |
| drop-late, 폰 실제 시계 벌어짐 (계획 밖) | I_1 t_* 4회 (보통 재생, drop-late 50) | 폰 절전(suspend) 으로 BOOTTIME 수신 지연 +1.5~11.7 s (MONOTONIC feeder 지연 ≤ 0.4 s) → mode 1 이 평평할 때 re-anchor (+1.5/+2.8/+6.7/+9.3 s), 움직일 때 late-lost 로 출력 lost; **mode 1 실행 전부 h>0.5 s "tracking" 예측 0줄** | 4 |
| 폰 시간 (I_2, `--drop-late 0`, ABBA) | raw80 실시간, c4, 2 threads, AC 연결·충전 안 함 (st4), 화면 Dozing | slam_ekf p50 새 0.64/0.82 vs 옛 1.41/1.55 ms (ovupd 와 같은 방향, 약 절반). 프레임 p50 새 8.55/10.77 vs 옛 10.82/13.26 ms 이지만 cpu9 상한 중앙 새 2304/1824 vs 옛 2112/1728 MHz + 절전 섞임 → **프레임 단위 이득은 확정 아님** | 2+2 |
| ESP32 PC 검사 | `esp32fc/test` make (esp32_test.log), host_syntax | **ALL PASS 573**, syntax fail 0 | 1 |
| 앱 | `build_check.sh` | aapt: vc 32 / 0.6.26-integ2-20261026, apksigner: 같은 개발 인증서, APK 안 libov_bridge.so sha 527b94eb = 폰 재생 바이너리 | — |
| calcap | `app/calcap_smoke_try*.txt` (서버와 같은 시작 순서, CAP 없음) | 1차 (06:14, 내 스크립트가 서버의 WAKEUP+재연결을 안 함) 바로 끊김 → 2차 (서버 순서) 연결 4.0 s, PREV 15/15, 10.0 fps, 노출 2 ms, ISO 3200 (밤, 시작 때 1번 자동), 초점 0.4987, STOP OK, 카메라 `[]`, 앱 프로세스 없음. PC 세트 calcap_20261025_214000 = 8/33 그대로 (안 건드림) | 1 |

## 사용자가 정할 것
1. **60 fps vs 30 fps**: 오프라인 자료로는 못 정함 (verify_ov60eval). 다음 실험 = 라이브 모드(2 ms·ISO 고정·0.5 dpt) 네이티브 60 fps + 보드 GT 녹화 8개 이상 (드론식 빠른 동작, 제자리 복귀, 시작·끝 2 s 정지) → 같은 자로 L60 vs L30 (짝·홀 두 위상) 짝 비교.
2. ~~gyro_klt=5 켤지~~ → **켬 (사용자 2026-10-26 아침, 0.6.27)**. 당시 근거: GT 발산 8→3 (녹화 2개 구제), 222747_o0 +0.7 cm 악화, rec60·라이브 빠른 동작·ARM 동일성 미검증, `--pipe` 와 금지.
3. **ESP32 openloop_cut_mode 1 켤지** (기본 0): 시뮬 결과뿐, 실패 범위는 esp32fc README.
4. **NC05 (IMU 잡음 ×0.5)**: 42칸 발산 4→3, 짝 −0.031 cm (녹화 10/13, p≈0.09) — 프로필에 안 넣음.

## 확인 안 된 것 / 주의
- 프레임 단위 시간 이득 (폰): CPU 상한·절전이 섞여 2+2회로는 확정 못 함. 폰 재생은 화면 Dozing 이면 절전으로 시계가 벌어짐 (2 s adb 폴링으로도 못 막음) → 앞으로 재생 시간 측정은 이 점을 고려 (tl.csv 에 MONOTONIC 시각 추가 권장, 검증자 제안).
- drop-late 50 경로 (mode 0/1) 는 시간에 따라 달라 동일성 검사에 없음 → PC 시험 (droplatefix) + 폰 I_1 로만 확인.
- 라이브 카메라 실행 없음 (지시대로). ESP32 실제 빌드·비행 없음. `--viop-hb-src 1` 폰 시험 없음.
- 첫 calcap 시도에서 카메라가 잠깐 켜졌을 수 있음 (바로 force-stop, `[]` 확인).

## 되돌리기
- 앱: 0.6.25 APK = `C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\integ2\apk\arlat_0.6.25_rollback.apk` (= /tmp/calcap/apk/arlat.apk, vc 31) → `adb install -r -d` (버전 내림 허용, 앱 데이터 유지).
- 브리지/앱 동작만: 옛 프로필 = `ov_opt "all_exact,ekf_gemm,qr_compress,ekf_chol"`, 옛 drop-late = `--drop-late-mode 0`.
- git: `integ-20261025`, `calcap-20261025` 는 그대로. gyumingim 에 안 합침.

## 파일
`ident/` (동일성 스크립트·로그·출력), `integ2_ate.py`·`score.json`·`ate_score.txt`·`ate_pose_cmp*.{json,txt}` (원시 포즈 WSL `/tmp/integ2/ate/`), `phone/` (계획·러너·목록 I_1/I_2, `out_I_1`·`out_I_2`, 분석 `dl_an_I_1.txt`·`predchk_I_1.txt`·`I_2_timing.md`·`lag_I_*.txt`; `I_1_timing.md` 는 시간 비교로 무효), `app/` (calcap smoke), `esp32_*.log`, `build_check.sh`, `verify_integ2.md` (독립 검증자 보고).
