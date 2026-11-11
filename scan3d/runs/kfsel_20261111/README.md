# kfsel 2026-11-11 — 움직임 기반 키프레임 고르기 (interval vs motion)

브랜치 `kfsel-20261111` (scanner-defaults-20261111 bcc2c75 에서). 코드: `scanA/kf_select.py` (새), `runs/kfonly_20261111/make_reduced.py --kf-select`,
`scanA/kf_ba.py --kf-list`, `scanA/run_scanner.sh --kf-select interval|motion --kf-dist CM --kf-rot DEG --kf-win N --kf-blur lap|gyro|none`.

## 1. 규칙 (폰에서 그대로 돌릴 수 있게: 실시간 VIO 자세 + 자이로 + 화상, 몇 프레임 창만)
- interval (지금 기본값): VIO tracking 프레임 10 개마다 (133820: 342 장, 약 5 장/s, 움직임과 무관).
- motion: 마지막 키프레임 (VIO 자세) 에서 **≥ d 이동 또는 ≥ θ 회전** 하면 "때가 됨". 그 프레임 + 뒤 VIO 프레임 (win−1) 개 (win 4 = 약 70 ms 앞까지)
  중에서 가장 선명한 것 하나를 키프레임으로 (blur lap = 480×270 회색 Laplacian 분산 최대; gyro = 노출 시간 [t_mid ± (노출+RS skew)/2]
  평균 |ω| 최소). 다음 키프레임은 고른 프레임의 자세에서 잰다. 첫 tracking 프레임이 첫 창을 엶. 창은 늘 하나를 내보냄 (멈춤 없음).
- kf_ba 는 make_reduced 의 manifest.json `keyframes` 목록을 그대로 씀 (`--kf-list`); 짝 규칙 (`--seq 6` 키프레임, 공시 10) 은 그대로.

## 2. 사전 등록 (측정 전에 씀, 11-11)
녹화 1 개: 20261022_133820, 노트북 (`tools/remote_laptop.sh job 133820 scan`), VIO = kfonly 와 같은 재생 자세 파일
(`vio/20261022_133820.poses.jsonl.gz`, sha256 51a19faa…, README 4 절에서 재생과 3833 행 같음 확인) 을 `--vio` 로 넘김 (vio 단계 생략, 모든 변형 같음).
변형 (각 1 회):
| 태그 | 규칙 |
|---|---|
| I | interval (이 브랜치에서 다시; ghost 4.1 절 dense 결과와 같아야 함 = 코드 바뀐 것 없음 확인) |
| M3 | motion 3 cm / 4°, win 4, lap |
| M2 | motion 2 cm / 3°, win 4, lap |
| M4 | motion 4 cm / 6°, win 4, lap |
| (M3g, 시간 남으면) | motion 3 cm / 4°, win 4, gyro |

**합격 규칙** (motion 변형마다, I 와 비교): |dZ| p90 (모은 화소, kf 별 p90 중앙 둘 다) 가 I 보다 5 % 넘게 나쁘지 않음, 축척 k 가 I 의 ±0.3 %p 안,
보드 덮임 (`mesh_board_cover_frac`) ≥ I, 키프레임 수 ≤ I. 넷 다 맞아야 합격. |dZ| 는 축척 오차 k 가 섞인 값 (VIO 축척 그대로).
참고 (판정 밖): 키프레임/분, 보낸 JPEG MB/분 (1920 q90), kf_ba 초, 전체 초, FFS 짝 있는 키프레임 비율 (`n_kf_depth / n_kf`),
이중층 (`kfonly mesh_layers.py`). 실행 1 회씩이라 BA 자세 흔들림 (±10 %, kfonly) 보다 작은 차이는 같다고 읽음.

## 3. 결과
(측정 후 씀)
