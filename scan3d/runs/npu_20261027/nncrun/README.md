# nncrun 2026-10-27 — AI Studio .nnc 를 S24(Exynos 2400) NPU 에서 실제 실행

## 결론
- **NPU 에서 돈다.** 앱 프로세스(untrusted_app, `am instrument`, 화면·권한 없음)에서 vendor 공개 라이브러리
  `libenn_public_api_cpp.so` 를 dlopen → `EnnOpenModel` → `EnnExecuteModel`. logcat: `Operator[1] (hw: 4) ENN_NPU`,
  `NpuUserDriver::OpenSubGraph / VS4L STREAM_ON`, `Unload Accelerator(4)` (5회 실행 모두 같음). GPU 부분 그래프 없음.
- **CPU 가 섞이는 곳**: float 입출력 형 변환 `QUANTIZE`/`DEQUANTIZE` 두 연산이 `hw: 1`(CPU, vendor ENN 서비스 안)에서 돈다.
  컴파일 설정이 `quant_type sw / dequant_type sw` 이기 때문. 모델 본체(conv 전부)는 NPU.
- **폰 출력 = AI Studio 골든**: 골든 입력에서 SNR 77.46 dB (AI Studio 보고 77.5 dB).
- **시간** (EnnExecuteModel, 워밍업 50, 200회×2): 기본(perf 7) p50 **2000 / 1993 us**, p95 2062 / 2037 us.
  BOOST(1) p50 1966 / 1933, BALANCED(4) 2162 / 2157, LOW_POWER(0) ~10 ms. 원격 프로파일 1751 us 보다 약 0.25 ms 느림
  (CPU 형 변환 포함 추정). 앱 쪽 memcpy 입력 157 us, 출력 854 us 별도.
- **정확도는 지금 쓸 수 없다.** 실제 13 프레임, PC fp32 ONNX 대비: SNR 6.5 dB, 키포인트 top-1024 겹침 **31 %**(20~37),
  기술자 cos 중앙 0.87. 원인 = **무작위 보정**(`use_randomdb: True`): 입력 양자화 범위가 [-1, 1] 인데 실제 입력은
  [-4.8, 10.6] (화소 23 % 가 잘림). PC 에서 입력만 [-1,1] 로 잘라도 겹침 70 %, cos 0.98 → 나머지 손실은 내부 층 범위.
  → **해결: AI Studio 에서 실제 보정 데이터(`npu_upload/calib`, 3채널 48장)로 다시 컴파일** (가능하면 quant/dequant hw 로).

## 파일
| 파일 | 내용 |
|---|---|
| `nncrun.cpp` | 자체 코드 ENN 실행기 (JNI). 함수 이름 = 폰 라이브러리 C++ 기호(`llvm-nm -D`), 인자 = ExecuTorch(BSD)·LiteRT(Apache) 공개 로더 + mangled 이름. 삼성 헤더·바이너리 복사 없음. CPU 대체 경로 없음 |
| `src/.../Probe.java`, `AndroidManifest.xml`, `build.sh` | APK `com.scan3d.nncprobe` (Activity 없음, 권한 없음, run-as 용 debuggable) |
| `prep_ref.py` | 폰 입력(`golden_in.bin`, `eval3ch.bin`)과 PC fp32 기준 출력, cat1out == 원래 xfeatnn 확인 (heat 129 = logit, sigmoid 후 일치) |
| `acc_nnc.py`, `acc_nnc_a2.json` | 폰 출력 vs PC fp32 (프레임별 SNR, 겹침, cos, 입력 잘림 분리) |
| `run1.sh`, `st.sh` | 한 번 실행 + 온도/배터리 관문 (SKIN<36·AP<46 시작, SKIN≥45·AP≥50·배터리<20 중단) |
| `phone/*.applog.txt`, `*.logcat_enn.txt`, `*.state_*.txt` | 폰 기록. 전체 logcat(필터 없음)은 개인 앱 로그가 섞여 저장소 밖 `~/scanner/tmp_npu/nncrun_data/logcat_full/` |
| `results.json` | 요약 |

.nnc 는 삼성 산출물이라 커밋하지 않음: `C:\Users\a3162\scan3d_handoff\npu_results\a2_xfeat3ch_cat1out_e2400\` (md5 94c857eb…).
폰 출력 bin: `~/scanner/tmp_npu/nncrun_data/phone_out/`.

## 다시 돌리기 (새 .nnc)
1. `bash build.sh` → `adb install -r ~/scanner/tmp_npu/nncprobe/nncprobe.apk`
2. `adb exec-in run-as com.scan3d.nncprobe sh -c 'cat > files/model.nnc' < NEW.nnc` (입력도 같은 방법; /data/local/tmp 안 씀)
3. `bash run1.sh TAG "model=model.nnc in=eval3ch.bin out=out_eval.bin warm=50 iters=200 reps=2"`
4. `adb exec-out run-as com.scan3d.nncprobe cat files/out_eval.bin > out.bin` → `python acc_nnc.py out.bin acc.json`
5. 끝나면 `adb uninstall com.scan3d.nncprobe`, `pm list packages | grep nncprobe` = 0 확인.

## 조건·가정
- 폰: 사용자 수면 중(Dozing), SKIN 29.7~30.1 °C, AP 28.9~38.4 °C, 배터리 73→72 %, 충전 안 함. 카메라 안 씀.
- 시간은 같은 입력 반복(프레임 0). 실제 프레임마다 첫 실행은 2.0~4.1 ms (캐시·클럭 상승 추정).
- NumberOfBuffersInfo / EnnBufferInfo 구조는 공개 로더 기준으로 읽었고, 돌려받은 값(입력 640×352×3, 출력 80×44×130,
  크기 일치)으로 확인함.
- 반복: 시간 2회씩, 정확도 1회(13 프레임). 기준 = PC onnxruntime CPU fp32 (같은 cat1out ONNX).
