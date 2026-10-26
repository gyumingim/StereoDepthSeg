# lenscal_20261027 — S24 메인 렌즈 보정 결론 (작업 중, 중간 저장)

만든 시각: 2026-10-26 22:2x (중간 저장, STOP NOW). 계산만 함 — 폰·카메라 안 씀. 앱/설정 파일에 새 값 안 넣음.
렌즈 값은 보드 보정 데이터로만 정함 (VIO 오차로 고르지 않음).

## 상태
- 조사 노트: `research_notes.md` (출처 URL·확신 표시, 검색 요약만 본 것 표시). 핵심: 지금 약점 = 사진당 모서리 8개, 노트북 화면(평탄도·덮개 유리 굴절 미측정),
  yaw ±40° 까지·roll 없음·대각 적음, 화면 귀퉁이 r > 0.88 비어 있음. mrcal 합격 판단 = 독립 세트 두 개 차이 ≤ 약 2× 투영 불확실성 (공식 px 기준 없음).
- mrcal 2.1 설치됨 (sudo 없이 jammy .deb 를 `apt-get download` + `dpkg -x` → `~/scanner/tmp_lenscal/root`). 환경: `source mrcal_env.sh` 후 `/usr/bin/python3` (numpy 1.21.5, cv2 4.5.4).
  확인: `import mrcal` OK, LENSMODEL_OPENCV4/8, SPLINED_STEREOGRAPHIC 지원, `mrcal-calibrate-cameras` 등 CLI 있음.
- 분석 (mrcal 휨·모델·불확실성·교차검증): **아직 안 함**. 다음 단계는 아래 RESUME.

## 지금까지 알게 된 것 (분석 전, 기존 결과 기준)
- calcap_calib_20261026: fx 1286.25 ± 4.7 (칸 부트스트랩), 반반 통과 8 %. 화면 0.5 mm 휨 가정 시 fx −7.7 px (휨 미측정).
- 이전 glass.py (lens_calib_f050_20261023/results/glass.json): 덮개 유리 굴절 모델 (n 1.5, t ≤ 3 mm) 은 fx 를 +0.2 px 만 바꾸고 t 가 경계 3 mm 에 붙음 → 굴절로는 기울기별 fx 차이 (25–45° 1287.5 vs ≥45° 1279.1) 를 설명 못 함.
- 코디네이터 입력 (rectdiff_960x540.json, c772f8e): 정류 K 652.10 고정 시 같은 광선 위치 차이 c4f60 원본 렌즈 대 새 보정 = 가운데 1.66 / r0.5 최대 3.12 / r0.8 최대 7.09 / 귀퉁이 최대 8.84 px; cal20261024d 대 새 보정 < 1 px.
  659.35 vs 652.10 = 정류 new K 선택 차이 (원본 fx 셋 다 ~1286), 원인은 c4f60 렌즈 k2 −0.117, k3 +0.058. → c4f60 이 지금 앱에서 쓰이는지 확인 필요 (결론에 반영 예정).

## RESUME (다음 단계)
1. `~/scanner/StereoDepthSeg/venv_ffs/bin/python` 로 calcap_calib.load_views 를 불러 132장 모서리를 `~/scanner/tmp_lenscal/views_calcap.npz` 로 덤프 (id→행/열 = round(obj/sq)−1, 4×2 격자).
   가능하면 cal20261024d 세트 (`~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_f050_20261023/views/*.pkl`, 5x3·6x4) 도 덤프 → 독립 세트 B.
2. corners.vnl 작성 → `mrcal-calibrate-cameras --corners-cache ... --lensmodel LENSMODEL_OPENCV4|OPENCV8|SPLINED... --focal 1286 --object-spacing 0.0604 --object-width-n 4 --object-height-n 2`.
   주의: 2줄 보드라 calobject_warp 의 y 항은 관측 불가 (특이 행렬 가능) → 안 되면 Python API 로 x 휨만 직접 모델.
3. 투영 불확실성 지도 (mrcal.projection_uncertainty), 세트 A/B (또는 세션별·반반) 교차검증 차이 vs 불확실성, 휨 추정값 ± sd.
4. 결론 (README 맨 위 형식: 지금 값 사용 가능 여부·숫자 / 이번 주 절차 / 합격 기준 / 근거 표). c4f60 이 앱에서 쓰이는지 config 확인.
