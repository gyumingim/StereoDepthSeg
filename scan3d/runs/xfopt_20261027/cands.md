# xfopt 조건 기록 (실행 전에 한 줄씩 추가; 섞인 변수 표시)

| 조건 | 바뀐 것 (B60 / X1 대비) | 왜 | 추가 시각 |
|---|---|---|---|
| B60 | BoW + box-BRIEF (기준선), log_cand=1 (결정 안 바뀜) | 기준선 (새 빌드 · 새 렌즈) | 10-27 12:40 |
| B60r | B60 에서 log_cand 없음 | log_cand 가 결정을 안 바꾸는지 확인 | 10-27 12:40 |
| X1 | XFeat NN 1k, 640x360, xf_tri 1° 2 px (= ovxfeat N1kt) + xf_both=1 · log_cand=1 | 출발점 (BRIEF · XFeat 두 경로 후보 자세 모두 기록 → 문 / 하이브리드 오프라인 재현) | 10-27 12:40 |
| X1n | X1 에서 xf_tri 0 (= N1k) | 삼각측량 없이 | 10-27 12:40 |
| X1L | X1 에서 LighterGlue (= L1kt) | LG 비교 | 10-27 12:40 |
