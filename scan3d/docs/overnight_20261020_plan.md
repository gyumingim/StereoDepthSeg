# 밤샘 계획 2026-10-20 → 21 09:00 (결과 전에 적어 둔 판정 기준)

## 데이터
새 녹화 6 개 (폰 → Cloudflare 터널로 전부 받기, sha256 대조): hs_20261020_214138 (v0.2, 원본 IMU 있음, 앱 온도 기록),
221052 · 221903 · 221925 · 222747 (v0.4: + NR/edge 끔 요청, 노출·ISO·AWB 기록), 223815 (v0.5: 화면 정지 카운트다운).
221903 + 221925 는 같은 세션 (뒤로 가기로 끊김, 2.35 s 빈틈 — 이어 붙이지 않음, 바이어스만 공유).
녹화마다 처음 정지가 있는지는 IMU 로 확인 (|ω| 표준편차·|a| 일정 구간 ≥ 2 s).

## 주의 (먼저 확인)
- 보드가 **노트북 화면(1440 px 폭)** 에 띄워졌을 수 있다 → 칸 크기가 63.403 mm 가 아니다. 영상에서 보드 칸 수·크기를 확인하고,
  노트북이면 (a) 가속도계 적분으로 축척 추정 (imu_learn/scale_check.py 방식, 기존 ±2 %) 해서 칸 크기를 정하고 "추정" 표시,
  (b) 아침에 사용자에게 자로 재 달라고 요청 목록에 올린다. 렌즈는 같은 초점(2.22 dpt)·해상도라 기존 lens json 사용.
- 잡음 제거·선명화 끔 요청이 실제로 적용됐는지 results_cam5.csv nr_mode/edge_mode 로 확인 (v0.4+).

## 순서
1. 받기·대조 (업로드 담당). 2. 입력 점검 (메타인지 담당, 읽기 전용): 새 앱 열·단위·시계·온도·카메라 상태·정지 구간·프레임 누락.
3. 정답: imu_gt.py --rs --keep-corners (imu_gt_rs) → spline_gt.py knot 0.0125 (imu_gt_spl). batch_sync 로 dt·R_ci.
4. VIO (벤치 strict 기본, 한 코드 버전, 결과마다 해시): 녹화마다 IMU 보정판(4/1) vs 원본(16/35), 설정 (a) strict 정지 초기화
   (이번엔 처음 정지가 있으니 시험 가능), (d) strict+dynamic, (c) 옛 loose — 시작 시점 0/1/2 s 반복 3 회, 잡음 = 앱 기본(주) / Allan(보조).
5. 보정: 물리 보정을 옛 녹화로 학습한 값 / 새 녹화로 학습한 값 / 바이어스+온도 1차식만 — VIO 로 채점 (녹화 하나 빼고 학습).
6. 비판·검증·재판: 각 단계 결과는 다른 에이전트가 반박 → 재판이 유지/수정/철회. 메타인지 검사 단계마다.
7. 보고서 docs/report_20261021.html (쉬운 말, 결론 먼저, 확정/추정 구분) + 커밋·푸시.

## 판정 기준 (미리 정함)
- 초기화 성공 = 초기화 뒤 10 s 창 Sim3 축척 0.5~2 그리고 그 창 ATE < 10 cm. 주 지표 = 성공한 실행의 초기화 뒤 ATE, 보조 = 성공률.
- C5 (느슨한 초기화가 주원인): strict 정지 초기화 (a) 가 처음 정지가 있는 새 녹화에서 무너짐을 loose 대비 절반 이하로 줄이면 지지,
  아니면 지지 안 함. 녹화 단위 부호 검정으로 판단, 녹화 수가 적으면 "근거 약함" 표시.
- 원본 vs 보정판 IMU: 같은 설정에서 녹화 단위로 ATE 중앙값 비교, 반복 흩어짐보다 작으면 "차이 없음".
- 물리 보정: 보정 없음 대비 녹화 단위 개선 수 / 악화 수, 같은 기준.

## 정답(GT) 만들기 — 노트북 화면 보드 (GT 담당, 2026-10-20 23:30~)
- 보드: `config/charuco/board_laptop_6x6.json` (모니터 보드와 같은 사전·배치·마커 비율, **칸 40.70 mm = IMU 추정**, `square_mm_source` 에 표시).
- 녹화마다 `imu_learn/overnight_gt.sh` (업로드 완료·sha256 확인 뒤 `overnight_gt_loop.sh` 가 차례로 부름):
  imu_gt (원본 IMU 35/16 이 있으면 `--raw-imu`, align.json `imu_raw`) → board_shift (자르기) → batch_sync (dt·R_ci·자이로 바이어스)
  → imu_gt `--rs --keep-corners` (imu_gt_rs) → spline_gt knot 0.0125 카메라만 (imu_gt_spl) → scale_ls (IMU 축척 확인) → gt_status.
- 모든 정답 폴더에 `board.json` (보드 파일·칸 크기·출처·git HEAD·코드 sha256), gt.npz 에 `board_json`·`square_mm`.
- 상태: `runs/overnight_gt_status.json`, 진행: `runs/overnight_gt.log`, 단계별 로그: `runs/overnight_gt/`.
- 축척 확인 방법 치우침: 모니터 녹화 11 개(칸을 앎)에 같은 scale_ls → `runs/overnight_gt/scale_ref_monitor_*.json`.

- 결과 (03:19): 녹화 6 개 IMU 축척 → 칸 **44.5 mm (95 % 43.6–45.4, 추정)**, 녹화끼리 일치 (chi2 6.4/5). 감사 때 40.7 mm 는 221925 앞 7.5 s 만 쓴 값 —
  정답 파일은 밤새 일관되게 40.70 mm 로 만들었음 (위치가 약 8 % 작음 추정). 221903 (자세 OK 7 %), 224100 (보드 없음) 은 정답으로 못 씀.

## 아침 할 일
1. **노트북 화면 가로 길이 재기** (검은 테두리 안쪽, 빛나는 부분, mm) = W. 칸 = W / 7.07.
2. 정답 축척 고치기 — 한 명령, 몇 초 (위치만 비례, 회전·dt·R_ci 는 그대로라 다시 돌릴 필요 없음):
   `cd ~/scanner/StereoDepthSeg/scan3d && ../venv_ffs/bin/python imu_learn/rescale_gt.py --screen-width-mm W --update-board recordings/hs_20261020_2*`
   (`--dry` 로 먼저 확인 가능. imu_gt·imu_gt_rs·imu_gt_spl 은 정확히 비례; imu_gt_spl_imu 는 가속도계가 축척을 정하므로 `spline_gt.py --imu` 다시)
3. 확인: `../venv_ffs/bin/python imu_learn/scale_ls.py recordings/hs_20261020_2* --gt-dir imu_gt_rs` 의 s 가 모니터 녹화 값(ref)과 비슷해야 함,
   그리고 `../venv_ffs/bin/python imu_learn/gt_status.py recordings/hs_20261020_2*` 로 상태 json 갱신.
4. 정답 축척 위에서 돈 VIO 판정(ATE 10 cm, Sim3 축척)은 고친 축척으로 다시 계산 (ATE 는 축척에 비례).
