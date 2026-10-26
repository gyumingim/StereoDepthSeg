# dpvofast 2026-10-26 — 미리 정한 계획 (결과 보기 전, 18:20 작성)

목표: DPVO 가 실제로 얼마나 빨라질 수 있나 (PC, RTX 5080, WSL2). 폰 안 씀.

## 고정 평가 세트 (모든 설정 같은 세트, 같은 채점 = dpvo_20261026/dpvo_run.py score_one / loop_eval)
- std60 165506 o0, o1, o2 (60 fps 격자, GT imu_gt_spl) → window ATE (전체 Sim3 축척), final + causal
- 손 루프 142148_o0, 142610_o0 (60 fps) → 시작-끝 / 경로 %, final + causal
- 비교 상대: OpenVINS GL60 (dpvo_20261026 ov_ref 재실행 값) 1.50/1.55/1.70 cm, 루프 3.27 / 5.20 %.
  공정성 (코디네이터 요청): OpenVINS GL60 도 **같은 GT Sim3 축척 정렬**로 다시 채점해 표에 같이 적는다.

## 설정 (각 1회, 시드 0; base 는 시드 0/1/2 로 반복 → 잡음 폭)
base (96 패치, 960x528, window 10, fp16 autocast = yaml 기본), p64, p48, p32, w7 (OPT 7 / REMOVAL 16 / LIFE 11),
fast (= config/fast.yaml: p48 + w7), ba1 (BA 내부 반복 2→1), fp32 (반대 방향 확인), compile (torch.compile),
nosync (호스트 동기 1개 제거), sc067 (640x352), sc05 (480x264), s2/s3/s6 (60 fps 의 매 2/3/6 번째 프레임 = 30/20/10 Hz).
조합은 위 결과를 본 뒤 2–4개만 추가 (추가분은 README 에 "사후" 로 표시).

## 타이밍
- 작업 1개씩 순서대로 (GPU 단독), 시작 전 부하 기록. 프레임당 cuda-synchronize 벽시계 (H2D 포함), 앞 30 프레임 제외, 중앙/p95.
- 시간 분해: dpvofast.py split (초기화 후 100–400 프레임, 홀수 프레임 = 구역 타이머 동기, 짝수 = 깨끗한 벽시계,
  그 다음 300 프레임 = torch.profiler 커널 시간·launch 수). 960x528 / 640x352, std60 165506_o0 + HS 221052_o0.

## 판정
Pareto (ms/frame vs std ATE·루프 %). "OpenVINS GL60 보다 분명히 좋음" = std 3칸 모두 OV 보다 작고 (같은 Sim3 자 기준 포함),
루프 2개 모두 OV 의 절반 이하.
