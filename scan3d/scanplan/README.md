# scanplan 2026-11-05 — 폰 3D 스캐너 첫 판 계획 (메모)

보고서: C:\Users\a3162\scan3d_handoff\changes_0620\scanplan_20261105.html (http://localhost:8796/scanplan_20261105.html)

## 오늘 돌린 것 (녹화 1개, 미검증)
`e2e_mono_ms.py`: hs std60 녹화(렌즈 5, 1920x1080@60, 0.5 dpt) → 960x540 → 키프레임 10 장마다, 기선 4–30 cm 짝 2개 →
`motion_stereo.pair_depth` (FFS, anchor 없음) → `combine_motion` → `tsdf_map.TSDFMap` (1 cm) → mesh.ply.
평가 = 노트북 화면 ChArUco 5x3 (302x181 mm) 평면 z=0.

| 자세 | ATE | 키프레임 깊이 \|dZ\| p50 (p90) | 메시 평면 치우침 / RMS |
|---|---|---|---|
| GT imu_gt_spl | — | 0.23 (0.45) cm | −0.04 / 0.17 cm |
| VIO fps30 o0_C4 (30 fps 960x540) | 1.26 cm | 1.53 (2.13) cm | +0.86 / 0.16 cm |
| VIO fps30 o0_C1 (60 fps 640x360) | 2.44 cm | 4.78 (5.89) cm | −1.5 / 0.72 cm |

결과: `scan3d/runs/scanplan_20261105/*/summary.json`. 재현:
`FFS_ROOT=~/scanner/StereoDepthSeg ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/scanplan/e2e_mono_ms.py <rec> --out <dir> [--vio <poses.jsonl.gz>]`

주의: GT 자세는 같은 보드에서 구함(순환, 상한으로만). 보드 = 무늬 많은 쉬운 면. VIO 는 SE3 로만 맞춤(축척 VIO 그대로).
알려진 작은 버그: mesh_board_cover_frac 칸 반올림으로 1 을 넘을 수 있음.

## 추천 (A안)
VIO 녹화 그대로 → PC VIO 재생(lp_replay, tmp_livepose bridge 다시 빌드) → 키프레임 BA(pycolmap, sfm_recon 조각) → 모션 스테레오 FFS → TSDF.
성공 기준: 보드 평면 |dZ| ≤ 1 cm, 크기 아는 상자 모서리 ≤ 1 cm (≤ 2 %), 덮임 ≥ 90 %.
사용자 녹화: 보드 + 자로 잰 상자, 0.6–1.2 m 천천히 돌기 40–60 s × 3, 흰 물체 1, 방 1 (보고서 6절).
