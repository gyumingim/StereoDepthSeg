# lenskit_20261027 — 인쇄 보드 렌즈 재촬영 준비 (보드 PDF · calcap 설정 · 칸 목록 · 한 번에 계산)

만든 시각: 2026-10-27 (KST). 브랜치 `lenskit-20261027` (calcap-20261025 242a96a 에서 시작), 워크트리 `~/scanner/StereoDepthSeg-lenskit`.
**폰·카메라는 쓰지 않았다.** 모든 시험은 PC 합성 영상이다. 렌즈 값은 보드 데이터로만 정하고 VIO 로 고르지 않는다.
근거: `runs/lenscal_20261027/README.md` (2절 절차·자세, 3절 합격 기준 C1–C8), 검증 `scratchpad/team/verify_lenscal_20261027.md`.

## 1. 결론
- 인쇄 PDF 3장 준비: A1 12×9칸 60 mm (권장), A2 42 mm, A3 30 mm. 벡터 PDF, 실제 크기 100 %, 100 mm 막대 2개(가로·세로), 사전·ID·칸 크기 표기.
  다시 그려서(PyMuPDF 4 px/mm) OpenCV 로 검출: 88/88 모서리, 위치 오차 최대 0.007 mm, 배율 오차 < 0.005 % (3장 모두).
- calcap 이 새 보드·칸 목록을 **플래그로만** 쓴다. 플래그 없으면 예전과 같다 (옛 slots.py 와 400 자세·26 400 fills·13 200 hints·66 외곽선 비교에서 차이 0).
- 칸 목록 55개(거리 ≥ 0.9 m), 계산 한 번에(`lenskit_run.sh`): OpenCV k1k2 + mrcal(휨 2축) + C1–C8 판정 + 통과 시 lens05 생성기로 앱 렌즈 세트 (설치 안 함).
- 합성 시험: 평평한 보드는 참값을 fx −0.009 %, 주점 0.01 px 로 되찾고, 앱 세트 생성까지 끝까지 돈다. 0.5 mm 처지는 보드(기울기 부호마다 반대)는 **FAIL** (C1 모서리 무늬, C6).
- 주의 (이번에 새로 안 것): **C7·C8 (README 규칙 "≤ 2 sd") 은 평평한 합성 보드에서도 FAIL** 이다 (fx 0.23 px = 0.018 %, cy 0.55 px 차이; 잡음이 작아 sd 가 0.02–0.1 px 라서).
  합성 렌더링(흐림·보간)의 작은 검출 치우침이 통계적으로 보이는 것으로 본다(추정). 실제 데이터도 그럴 수 있다.
  그래서 `--accept-small` (차이가 C4 허용치의 절반 = fx 0.075 %, cy 1 px 보다 작으면 통과) 를 **선택 플래그**로 넣었다. 기본은 README 규칙 그대로(엄격). 쓸지는 사람이 정하고, 쓰면 결과·렌즈 파일에 기록된다.

## 2. 파일
| 파일 | 내용 |
|---|---|
| `config/charuco/board_print_{A1,A2,A3}_12x9.json` | 보드 정의. ID 범위 A1 130–183, A2 184–237 (DICT_6X6_250), A3 300–353 (DICT_6X6_1000) — 노트북·모니터 보드(ID 0–121)가 화면에 보여도 섞이지 않음 (시험 확인). `min_corner_frac` 0.3 (가장자리에서 일부 잘려도 됨). **인쇄 후 채울 칸**: `measured_span_x_mm`/`_y_mm` (첫→끝 안쪽 모서리, 600 / 420 mm 공칭), `flatness_mm` |
| `config/charuco/print/*.pdf`, `print_check.json` | 인쇄용 PDF와 다시 그려 검출한 결과. 같은 PDF: `C:\Users\a3162\scan3d_handoff\calib_board\` |
| `config/calcap_slots/print_A1_55.json` | 새 칸 55개 (아래 3절) |
| `tools/calcap/slots.py` | Board: `type` charuco(기본)/chessboard/circles/acircles, `ids_start`, `min_corner_frac`, 자 측정 x·y 따로 배율. 자세에 `roll` 추가. 칸 파일(kind G) 읽기·판정·힌트. 기존 칸 33개 동작은 그대로 |
| `tools/calcap/calcap_server.py` | `--boards`, `--slots` 추가 (기본 = 노트북 5x3,6x4 + 33칸). 인쇄 보드면 노트북 화면의 보드 그림을 숨김 |
| `tools/calcap/calcap_calib.py` | `--boards`, `--charuco-dir`, `--dsplit`, `--focus-dpt` 추가 (기본값 = 예전). 후보 파일에 size·boards 기록 |
| `tools/calcap/make_print_board.py` | PDF 생성 + `--check` (PyMuPDF 로 다시 그려 검출) |
| `tools/calcap/make_slots_print.py` | 칸 목록 생성 + 자기 점검(각 칸의 목표 자세를 투영·재검출해 그 칸을 채우는지) |
| `tools/calcap/lenskit_selftest.py` | 기본 동작 동일성 + 보드 종류별 검출 + PDF 래스터 검출 (RESULT: PASS) |
| `tools/calcap/lenskit_synth.py` | 합성 세트(calcap 형식 그대로, 처짐 옵션) |
| `tools/calcap/lenskit_run.sh` | **한 번에 계산** (아래 4절) |
| `tools/calcap/lenskit_dump.py`, `lenskit_mrcal.py`, `lenskit_make_app_lens.py`, `check_lens_focus.py` | 단계별 도구 |
| `runs/lenskit_20261027/synthetic/` | 합성 시험 결과 (`run2` 평판, `runsag` 처짐). `app_lens_synthetic_NOT_REAL.json` 은 합성 렌즈 — 쓰지 말 것 |

## 3. 칸 목록 (print_A1_55.json, 판은 받침에 고정, 폰은 삼각대)
README 2절 표를 그대로 옮기되 **거리 ≥ 0.9 m** (초점 0.5 dpt 의 심도가 약 0.9 m 부터라는 추정 — C7 이 확인): 0.6–0.7 m → N 0.90–1.15 m, 1.0 m → M 1.15–1.5 m, 1.5 m → F 1.6–2.1 m.
README N6 (0.6 m) 은 N-R (0.9–1.15 m, 판 돌림 많은 자세) 로 바꿨다.

| 묶음 | 거리 | 위치 | 기울기 | 수 |
|---|---|---|---|---|
| NC | 0.90–1.15 | 가운데 | 좌우 ±45, 상하 ±45, 대각 (±35,±35) 4, 돌림 90°+좌우 ±45, 돌림 ±30°+상하 ±40 | 12 |
| NE | 0.90–1.15 | 왼·오·위·아래 가장자리 | 그 축으로 ±35 | 8 |
| NK | 0.90–1.15 | 네 귀퉁이 | 대각 ±(21,21) ≈ 28° 두 방향 | 8 |
| MC | 1.15–1.5 | 가운데 | 정면, 좌우 ±40, 상하 ±40, 대각 2, 돌림 45°+좌우 20 | 8 |
| MEK | 1.15–1.5 | 가장자리 4 + 귀퉁이 4 | 30° (가운데 쪽) | 8 |
| F | 1.6–2.1 | 가운데 | 정면, 좌우 ±35, 상하 ±35 | 5 |
| NR | 0.90–1.15 | 가운데 | 돌림 90°+상하 ±45, 돌림 45°+좌우 ±45, 돌림 −45°+상하 ±45 | 6 |

- 합계 55: 상하+ 18 / 상하− 18 (C8 은 각각 ≥ 10 필요), 대각 18, 돌림 0/30/45/90°.
- 칸 창: 목표 각 ±12°, 거리 띠. 부호: 좌우(turn) + = 판 오른쪽 끝이 카메라에 가까움, 상하(lean) + = 아래쪽 끝이 가까움 (합성 투영으로 확인).
- 판 돌림(roll) 은 같은 법선의 "안 돌린 판" 기준 각도, 180° 접힘(−90…90).
- 자기 점검: 55칸 모두 목표 자세가 자기 칸을 채움 (가장자리·귀퉁이는 판을 화면 밖으로 밀도록 goal_gap 자동 조정).
- 렌즈는 c20261024d 로 칸 안내(노란 틀)를 계산한다. 안내용일 뿐 결과에 안 들어간다.

## 4. 촬영과 계산 명령
촬영 (세트마다, Windows PowerShell):
```
python \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg-lenskit\scan3d\tools\calcap\calcap_server.py --boards board_print_A1_12x9 --slots \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg-lenskit\scan3d\config\calcap_slots\print_A1_55.json --frames 3
```
- `--frames` 는 3–5 (서버가 3 미만을 3 으로 올림; 재확인 조건이 "3장 이상"). README 는 2장이면 충분하다고 했으나 서버 기본 동작은 바꾸지 않았다.
- 세트 A 1일차, 세트 B 2일차(또는 앱 재시작 + 폰을 받침에서 떼었다 다시 달기). 세트 G(선택): 폰을 세로/거꾸로, 10자세.
- 앞 서버는 20 분 최대·5 분 무촬영 자동 정지 (폰 열 보호). 55칸은 한 번에 안 끝날 수 있다 → 식힌 뒤 `--set <세트 폴더>` 로 이어 찍기.

계산 (WSL, 한 번에):
```
cd ~/scanner/StereoDepthSeg-lenskit
bash scan3d/tools/calcap/lenskit_run.sh <세트A> <세트B> [--g <세트G>] [--out <폴더>]
```
- 결과: `<폴더>/criteria_ko.txt` (C1–C8 판정), `criteria.json`, `opencv_*.json`, `mrcal_*/`, 통과 시 `app_lens/std_1920x1080_f0.50_print<날짜>.json` + `app_lens/phone_ov_main/` (17 폴더 + remap; `c4_nc05` 는 생성기 밖에서 만든 것이라 없음 — lens05 README 1절).
- 끝 코드: 0 = 통과 + 앱 세트, 2 = 기준 실패 (앱 세트 안 만듦), 그 밖 = 오류. 소요 약 2 분 (합성 2 세트, 부트스트랩 30/20).
- 앱 렌즈 = OpenCV k12_notan (A+B, p1 p2 k3 = 0, OpenVINS radtan). mrcal OPENCV4+휨은 판정용. 둘 차이는 `info_k12_vs_mrcal_AB` 로 출력.
- 설치는 하지 않는다. 설치·remap 교체·R_ci 재정렬은 사람이 정한다 (lens05 `phone_install.sh` 방식).
- **인쇄 후 꼭:** board json 에 `flatness_mm`(없으면 C5 = 판정 불가 → 전체 불통과)과 `measured_span_x_mm`/`_y_mm` 를 적는다.

## 5. C1–C8 구현 (lenscal README 3절 숫자 그대로)
| # | 구현 |
|---|---|
| C1 | 세트별 mrcal(OPENCV4+휨) RMS (OpenCV 방식 √(dx²+dy²)) ≤ 0.25, 이상치 ≤ 3 %, 모서리별 평균 잔차 벡터 최대 ≤ 0.05 px |
| C2 | 자세당 사진 1장으로 다시 푼 모델의 불확실성(무한 거리, 최악 방향): 가운데 ≤ 0.2, r0.8 대각 4점 최대 ≤ 0.5 px |
| C3 | 자세당 1장 A·B 모델, mrcal 회전 맞춤 차이 ≤ uA + uB (가운데·r0.5·r0.8), r0.8 ≤ 1.0 px |
| C4 | mrcal fx A·B 차 ≤ 0.15 %, 자세 부트스트랩(30회) sd(fx) ≤ 0.15 %, OpenCV k12_notan 주점 A−B ≤ 2 px |
| C5 | board json flatness_mm ≤ 0.1, mrcal 휨 2축 최대가 실측 ±0.1 mm, 휨 켬/끔 fx 차 ≤ 0.1 % |
| C6 | 세트별 반반 6회 (자세당 1장): 차이 ≤ 2√(u1²+u2²) 인 격자점(r ≤ 0.8) 비율 ≥ 70 % — "2 × 불확실성" 을 차이의 2σ 로 해석 (lenscal an2 와 같음) |
| C7 | A+B 가까이(< 1.15 m) 대 멀리 fx 차 ≤ 2√(sd1²+sd2²) (부트스트랩 20회); 세트 G 있으면 fx ≤ 0.1 %, 주점 ≤ 2 px |
| C8 | A+B 상하+ 대 상하− (각 ≥ 10 자세) fx·cy 차 ≤ 2 sd |

합성 시험 결과 (`synthetic/`):
| 시험 | C1 | C2 | C3 | C4 | C5 | C6 | C7 | C8 | 결과 |
|---|---|---|---|---|---|---|---|---|---|
| 평판 A·B (seed 1·2), `--flatness-mm 0.05 --accept-small` | PASS | PASS | PASS | PASS | PASS | PASS | PASS* | PASS* | 앱 세트 생성 (fx 1289.93 대 참 1290.04, cx 967.14 대 967.13, k1 0.0865 대 0.0865) |
| 같은 평판, 엄격(기본) | PASS | PASS | PASS | PASS | 판정 불가 (평면도 없음) | PASS | FAIL (0.23 > 0.13 px) | FAIL (cy 0.55 > 0.30 px) | 불통과 |
| 처짐 0.5 mm (상하 부호마다 반대), `--flatness-mm 0.05` | FAIL (모서리 무늬 0.10 px) | PASS | PASS | PASS | PASS | FAIL | PASS | FAIL | 불통과 |

\* `--accept-small` 때문에 통과. 처짐 시험에서 mrcal 휨은 0.01–0.04 mm 로 거의 0 이다 (부호가 반대인 처짐이 평균되어 사라짐 — lenscal 검증 지적 그대로). 그래서 C5 는 못 잡고 C1·C6 이 잡았다.

## 6. 초점 검사 (`check_lens_focus.py`) 와 앱 쪽 제안
- 렌즈 파일에 `focus_dpt`(사진 메타데이터 측정 중앙값), `focus_dpt_measured_range`, `focus_dpt_app`, `size` 를 적는다.
- 검사: 렌즈 초점 vs 앱 초점(또는 variants.json `run_focus_dpt`) ±0.05 dpt, 사진 측정 초점 범위, variants.json 의 `lens_focus_dpt` ≠ `run_focus_dpt` (예전 c4f60 = 2.22 렌즈 × 0.98624 경우) 를 WARN, 끝 코드 1.
  `python check_lens_focus.py <렌즈.json> --variants <phone_ov_main>/variants.json --frames <세트>/*/frames.jsonl`
- **앱 제안 (구현 안 함):** OvRunner 시작 때 `variants.json` 의 `lens_focus_dpt` 와 실제 요청 초점(인텐트 focus_dpt, 기본 0.5) 이 0.05 dpt 넘게 다르면 화면·로그에 경고하고 run meta.json 에 둘 다 기록. 실행 중 CaptureResult `LENS_FOCUS_DISTANCE` 가 렌즈 초점에서 벗어나면 경고 1회. 0.6.29 의 remap 크기 검사 옆에 두면 된다.

## 7. 검증 · 가정 · 남은 일
- `lenskit_selftest.py` RESULT: PASS (기본 동작 동일, A1/A2/A3·체커보드·원 격자(대칭·비대칭) 검출, 노트북 보드 무시, PDF 래스터 검출).
- 가정: 인쇄소가 100 % 로 찍는다 (막대·자로 확인해야 함). 심도 0.9 m 시작은 추정. 원 격자는 원근 치우침이 있어 비권장 (지원만). 비대칭 원 격자는 mrcal 이 못 받아 OpenCV 만 됨.
- mrcal 은 칸 간격 하나만 받는다: 자로 잰 가로·세로 배율이 0.05 % 넘게 다르면 경고만 하고 OpenCV 쪽에만 반영된다.
- 발견: mrcal corners 캐시에서 빠진 모서리를 `- - -` 로 쓰면 그 줄이 "이 사진에 보드 없음" 으로 읽혀 개수가 틀어진다 → `0 0 -` 로 씀. (lenscal 데이터는 사진마다 모서리 8개가 모두 있어 영향 없음: A 132장 1056점, B 217장 1736점 = ×8.)
- 남은 일: 인쇄·판 부착·평면도/자 측정 → 세트 A·B 촬영 → `lenskit_run.sh`. `--accept-small` 사용 여부는 결과를 보고 사람이 정한다.
