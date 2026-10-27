# npu_20261027 — XFeat · LighterGlue 를 Galaxy S24 (Exynos 2400) NPU 로 (2026-10-27 저녁)

## 결론
- **NPU 는 못 썼습니다.** NNAPI 'enn' 장치로 실제 실행된 모든 모델은 ENN Accelerator(2) = GpuUserDriver(**GPU**) 에서 돌았고, NPU 흔적은 0 입니다. adb shell 과 앱 프로세스(untrusted_app)에서 결과가 같습니다.
  - int8/uint8 conv(단일 연산·2~8 개 사슬·RGB 입력, per-tensor/uint8/per-channel): 드라이버가 "mainly run on NPU_DSP" 로 보내려 하지만 `cannot find original layer with the same name as: NpuInput_N/NpuOutput_N` 에서 실패 → Operation Not Supported.
  - 공식 uint8 MobileNet v1: 31/31 위임됐지만 드라이버가 "CPU_GPU" 로 분류 → GPU 에서 13 ms.
  - XFeat int8(TFLite NNAPI 위임): "CPU_GPU" 로 분류된 뒤 `exception lifetime is not supported` 로 거부.
  - vendor 라이브러리의 NPU 경로는 NNC(미리 컴파일, 암호화 가능) 모델용입니다. Exynos AI Studio(계정 필요) 나 ENNDelegate(라이선스 동의 필요) 가 있어야 하므로 **사용자 동의 전이라 진행하지 않았습니다.** ENN public API(`libenn_public_api_cpp.so`) 는 앱과 shell 모두에서 dlopen 됩니다(기호만 확인, 호출 안 함).
- **GPU(TFLite GPU 위임, fp16, 전부 GPU, CPU 섞임 없음)는 됩니다.**

| 경로 | 모델 | ms p50 (1회 / 2회) | ms p95 | 정확도 (vs PC fp32) | 판단 |
|---|---|---|---|---|---|
| GPU fp16 | XFeat 640x352 | 7.7 / 10.0 | 9.6 / 11.3 | top-1024 겹침 98.6~99.6 %, 기술자 cos ≥ 0.9998 (3 프레임) | 사용 가능 |
| GPU fp16 | LighterGlue 512 (모바일 연산판, tanh-GELU) | 29.9 / 29.6 | 34.7 / 34.0 | 상호 짝 일치 96~100 % (3 쌍) | 사용 가능 |
| GPU fp32 | LighterGlue 512 | 41.4 | 43.9 | — | — |
| GPU fp16 | LighterGlue 1024 | 61.8 / 48.4 | 66 / 80 | **변환 오류**(PC TFLite 부터 틀림) | 미완 |
| NPU (enn) | XFeat int8 / int16 / LG | — | — | — | 위임 0 (위 이유) |
| enn → GPU | MobileNet uint8 (판정용) | 13.0 | 13.5 | — | NPU 아님 |
| CPU (참고만, 사용 금지) | XFeat int8 XNNPACK 대체 실행 | 13.4 | 13.6 | — | CPU 대체 실행 |

- 루프 클로저 키프레임 예산 150 ms 대비: XFeat GPU 약 10 ms + LighterGlue 512 GPU 약 30 ms × 매처 4 회 ≈ **130 ms**(추정, 따로따로 잰 값의 합이며 함께 돌리면 경쟁이 생김) → 1 초에 1 장이면 여유.
- 측정 조건: 사용자가 폰을 쓰는 중(Brave/카톡/X), 배터리 83~91 % 충전 안 함, 절전 꺼짐, 화면 켜짐. 시작 기준은 코디네이터 지시로 SKIN<40·AP<45 로 완화 → 시간은 **참고값**. 21:13 에 AP 51.5 라 규칙대로 한 번 멈춤.

## PC 정확도 (int8 양자화, 우리 녹화 프레임 기준)
- 보정용 48 프레임(녹화 8 개), 평가용 13 프레임/쌍(다른 녹화 12 개 + 흰 벽 142610#600, 쌍 = +30 프레임). `make_calib.py`, `prep_feats.py`.
- XFeat int8: top-1024 겹침 74 %, 기술자 cos 중앙값 0.994. 실제 쓰임새(F-RANSAC 인라이어, 13 쌍, 질감 적은 쌍 1 개 포함)로 보면 fp32 326 → int8 289(-11 %). int16 활성(입출력 float 판) 328. 손실은 키포인트 머리 내부에서 생깁니다(softmax 를 그래프에 넣거나 clamp 해도 그대로).
- LighterGlue int8 은 짝 일치 23 % 라 못 씁니다. 가중치만 int8(dyn) 97 %, fp16 100 %.

## 한 일 / 파일
- `lg_clean.py`: kornia LightGlue 를 모바일 연산(3-D, Linear·BMM·Softmax·LayerNorm·GELU·cos/sin·concat)만 쓰도록 다시 짬. rotary 는 채널 순서를 바꿔 rotate_half 를 concat 으로 정확히 대체. 무작위 입력에서 kornia 와 m0/m1 이 100 % 같음. 예전 onnx2tf 변환을 막던 Expand 문제를 해결.
- `conv_tfl.sh`, `conv_lg_int8.sh`: onnx2tf 변환(int8 은 실제 프레임·실제 짝으로 보정).
- `make_probes*.py`: 연산 하나짜리 TFLite probe. `npuprobe/`: 직접 만든 NDK NNAPI probe(shell 실행 파일과 앱 com.scan3d.npuprobe 둘 다, 외부 코드 없음, 앱은 화면 없이 am instrument 로만 실행하고 끝난 뒤 지움).
- `acc_*.py`, `ds_*.py`: 정확도. `phone/`: 폰 실행 목록·결과(`r*.txt` = 항목별 SKIN/AP 전·후, 배터리, 충전, 절전, 화면 상태), ENN/GRAPHGEN 만 걸러낸 logcat(사용자 앱 로그는 빼고 저장).
- `results.json`: 위 숫자 원본.

## 정정 · 주의
- 처음 "per-channel 이 원인" 이라고 본 것은 틀렸습니다(per-tensor·uint8 도 거부). "NNAPI 는 즉석에서 GPU 만" 도 틀렸습니다. 드라이버는 폰에서 NNC 를 만들고 NPU_DSP 로 보내려다 층 이름 매칭에서 실패합니다(감독 21:10 지적).
- probes2 의 u8 pad/resize/sigmoid 는 QUANTIZE 가 끼어 있어 "거부" 판정이 무효입니다. per-channel int8 FC 의 BAD_DATA 는 NNAPI 규격 때문입니다.
- 규칙 위반 기록: ENN 이 거부한 TFLite probe 가 XNNPACK(CPU)로 각 ~1 s 돌았습니다(r1~r3). r4 의 옛 앱은 am start 로 사용자 화면 앞에 4 번 떴습니다. 이후 require_full_delegation 과 NDK probe 만 썼고, 앱은 instrument 전용으로 바꿨습니다.
- LighterGlue 1024 TFLite 는 onnx2tf 변환 결과부터 틀립니다(N=512 는 맞음). 원인은 아직 모릅니다.
- 폰 /data/local/tmp/npu (77 MB: 모델·probe·입출력) 는 지우지 않았습니다(사용자 허락 전).

## 다음 선택지 (사용자 결정 필요)
1. ENNDelegate(GitHub, 클릭 동의 라이선스) 또는 Exynos AI Studio(계정)로 XFeat int8 을 NNC 로 컴파일해 ENN API 로 실행 → NPU 가능성은 있음(미검증).
2. 동의하지 않으면 GPU 로 진행: XFeat GPU fp16 + LighterGlue 512 GPU fp16(정확도 확인됨).
