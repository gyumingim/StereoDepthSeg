# 렌즈 자동 캡처 (calcap, 2026-10-25)

폰은 VIO 와 **같은 카메라 흐름**(카메라 0, 1920x1080 YUV, TEMPLATE_RECORD, 초점 0.5 dpt 고정, 노출 2 ms, ISO 는 시작 때 한 번 정해서 고정)으로
**초당 10장만** 찍는다 (VIO 없음 → 덜 뜨거움). PC 가 미리보기로 보드를 찾고, 비어 있는 칸(거리·기울기·화면 위치)에 맞고 0.3~0.4 초 가만히 있으면
폰에 "원본 4장 저장" 을 시킨다 (무손실 Y 사진, PGM). 사진은 자동으로 PC 로 옮겨지고 다시 확인한 뒤 칸이 채워진다.

## 시작
1. 노트북(PC)에서 서버 켜기 (Windows PowerShell):
   ```
   python \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg-calcap\scan3d\tools\calcap\calcap_server.py
   ```
   (이어서 찍기: 끝에 `--set <세트 폴더>` 추가)
2. 노트북 브라우저로 **http://100.92.214.97:8789** 열기 → 오른쪽 아래 「패널」 → **시작**.
   - 폰 앱이 자동으로 켜짐 (폰 화면이 꺼져 있어도 됨). 노트북 화면은 자동 전체화면 → 5×3 보드가 1:1 크기로 보임.
   - 폰이 따뜻하면(SKIN 36 °C 이상) 시작 안 함. 다른 작업이 폰을 쓰는 중(잠금)이어도 시작 안 함.

## 하는 일 (사람)
- 노트북은 보드를 띄운 채 두고, **폰을 손에 들고** 카메라로 노트북 화면을 비춘다 (거리 0.4~1.2 m, 노란 틀은 대부분 0.5 m 근처 — 가까울수록 fx 가 잘 정해짐).
- 폰 화면: 카메라 그림 위에 **노란 틀 = 목표 위치·모양**, 점 = 지금 찾은 보드 (주황 = 아직, **초록 = 맞음**). 아래 글자 = 목표와 힌트.
  노란 틀에 점들을 겹치고 **가만히** 있으면 "찰칵" (노트북에서 삐 소리, 폰 파란 화면). 다음 목표가 나온다.
- 노트북 위·아래 얇은 줄: 목표 / 거리·기울기 / 채운 칸 수. 「패널」(또는 P 키)에서 칸 목록·미리보기·로그.
- 칸은 33개: 정면 9곳(가운데·가장자리·귀퉁이), 가운데 20° 기울임 8개(가까이/멀리), 가운데 40° 4개, 가장자리 40° 8개, 귀퉁이 40° 4개.

## 끝내기
- 패널의 **정지**, 또는 33칸이 다 차면 자동으로 끝. 끝나면 카메라가 꺼졌는지 서버가 `dumpsys media.camera` 로 확인해 패널에 표시.
- 자동 정지: 5 분 동안 찰칵 없음, 20 분 최대, 폰 SKIN ≥ 40 °C 또는 AP ≥ 50 °C 또는 배터리 42 °C, PC 연결이 20 초 끊김, 앱을 나감.
  중간에 멈춰도 찍은 칸은 남는다 → 식힌 뒤 `--set <세트 폴더>` 로 이어서.

## 결과 위치
`~/scanner/StereoDepthSeg/scan3d/recordings/calcap_<시각>/` : `captures.json`(칸·자세·확인 결과), `<폰 세션>/cap_*.pgm`(원본),
`frames.jsonl`(사진마다 노출·ISO·초점·센서 시각·칸·자세), `meta.json`, `temps.csv`, `server_log.txt`.

## 보정 계산 (WSL)
```
cd ~/scanner/StereoDepthSeg-calcap
~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/tools/calcap/calcap_calib.py ~/scanner/StereoDepthSeg/scan3d/recordings/calcap_<시각>
```
→ `<세트>/calib/summary_*.txt` (fx fy cx cy, 한 칸 빼기·반반 나누기 반복성, 거리·기울기 그룹별 fx, 지금 기본값 대비),
`lens_candidate_*.json` (후보일 뿐, config/lens 에 자동으로 안 넣음). 렌즈 값은 보드 측정으로만 정하고 VIO 오차로 고르지 않는다.

## 시험 (폰 없이)
`fake_phone.py` (가짜 폰: 알려진 렌즈로 보드를 그려 줌) + `calcap_server.py --fake 127.0.0.1:8812 --fake-dir <dir> --lock-dir ""`.

## 앱
arlat 0.6.23-calcap-20261025 (vc 29), `--es mode calcap` (기본 동작은 그대로). 인텐트: fps(10), exp_us(2000), focus_dpt(0.5), port(8811),
idle_s(300), max_s(1200), max_skin_c(40, 더 낮게만), max_ap_c(50, 더 낮게만). PC 와는 TCP 8811 (PREV/CAP/STAT/MSG/IMG/STOP).
