# 60 fps 촬영 → 15 fps 저장 (앱 1.8-subsample) 실측 기록

목표 (사용자 결정 2026-10-18): 60 fps 로 찍고 1/4 만 저장해 두 렌즈(2·5) 시각차를 최대 8.3 ms 로 줄이고,
녹화 용량·인코더 부담은 15 fps 수준으로 유지. 구현은 SPEC.md 1.2.3, 시험 도구 `scan3d/tools/subsample_test.ps1` +
`scan3d/tools/subsample_ana.py`.

## 1. 실측한 것 (2026-10-18 11:25, 실내, 폰 책상 위 정지, 1440x1080 렌즈 2+5, mode both, 14 Mbps)

| 항목 | 60 촬영 / 15 저장 (15 s) | 15 fps 기존 모드 |
|---|---|---|
| 카메라 결과 (촬영) | 50.4 /s (주기 16.78 ms, 약 5~6 장마다 HAL 이 한 장 건너뜀) | (못 잼 — 아래 2절) |
| 렌즈별 cam_fps (metrics) | 49.6 / 49.6 | |
| 저장 fps | 14.2 (그 뒤 격자 유지로 고침 → 15 기대, 재측정 필요) | 15 |
| 저장 짝 시각차 (노출 가운데) | 중앙 4.8 ms · p95 12.7 ms · 최대 16.6 ms (짝 202, 짝 없음 1) | 위상 10~33 ms (CLAUDE.md 8절, 다시 안 잼) |
| 노출 | 두 렌즈 16.67 ms (= 60 fps 한 프레임 전부, 어두운 실내에서 AE 가 최대) | |
| 폰 CPU (proc, 한 코어 %) | 196 % (이때 YUV 복사가 프레임당 20 ms — 그 뒤 줄 단위 일괄 읽기로 고침, 재측정 필요) | |
| 배터리 온도 | 31.2 → 31.2 °C (15 s) | |
| 녹화 비트레이트 | 14.1 / 13.8 Mbps | |
| 녹화 용량 | 49.8 MB / 14.3 s = 209 MB/분 (렌즈 2개) | 14 Mbps × 2 ≈ 210 MB/분 (같은 설정) |
| 1.7-mid 앱 60 fps (스트림 없음, 비교) | cam_fps 49.5, 같은 건너뜀 → 앱이 아니라 카메라(HAL) 쪽 한계 | |

관찰:
- 60 fps 에서 렌즈 5 가 logical 기준(물리 오프셋 0), 렌즈 2 의 위상이 프레임마다 약 +0.13 ms 씩 계속 미끄러져(약 8 ms/s)
  같은 요청 안 짝은 −15 ~ +7 ms 사이를 돈다 → 시각으로 가장 가까운 짝을 고르면 0~8.3 ms 에 고르게 퍼짐 (중앙 ≈ 4 ms).
- 노출 16.67 ms (60 fps 상한)라 어두운 곳에서는 15 fps 때(노출을 더 길게 쓸 수 있음)보다 영상이 어둡거나 ISO 가 오른다. 15 fps 의
  노출값과 같은 장소에서 비교 필요.
- 녹화 PC 읽기: `recording.Recording` 이 새 frames CSV(`pair_dt_ns` 열)를 그대로 읽음, fps 15, 가장 가까운 짝 시각차 중앙 4.78 ms
  (frames CSV 로만 확인 — mp4 는 안 가져옴, 복원(replay)은 못 돌림).

## 2. 못 한 것 (폰 무선 디버깅이 11:29 에 끊김 — 15 fps 기준 시험 도중)
- 15 fps 기존 모드 같은 시간 비교 (CPU·온도·노출), 60/15 를 1~2 분 (온도), 고친 앱(격자 유지·빠른 복사)으로 다시.
- 다시 하는 법 (서버 idle 일 때, Windows PowerShell):
  ```
  powershell -ExecutionPolicy Bypass -File REPO\scan3d\tools\subsample_test.ps1 -Name base15 -Fps 15 -Seconds 90
  powershell -ExecutionPolicy Bypass -File REPO\scan3d\tools\subsample_test.ps1 -Name s60 -Fps 60 -SaveFps 15 -Seconds 90
  wsl -d Ubuntu-22.04 -e bash -lc "cd ~/scanner/StereoDepthSeg && python3 scan3d/tools/subsample_ana.py scan3d/runs/subsample_base15 scan3d/runs/subsample_s60"
  ```
  (시험 동안 서버 자동 가져오기를 끄고 끝나면 다시 켬, 시험 녹화는 작은 파일만 가져온 뒤 폰에서 지움)
- 결과로 정할 것: `scan3d.py CAPTURE_FPS_DEFAULT` (지금 0 = 옵션), `load_model.PHONE_CAPTURE_PCT_PER_FPS` (지금 0).
