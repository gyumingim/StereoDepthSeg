# REVIEW8 — 앱 1.5-sync 렌즈 위상 맞추기 독립 검토 (2026-10-16)

대상: `PhaseSync.java`(새 파일), `MainActivity.java`, `Caps.java`, `AndroidManifest.xml`, `scan3d/web_server.py`, `scan3d/scan3d.py`, `scan3d/web/app.js`.
코드를 읽기만 했습니다. 실행·측정은 하지 않았습니다. 그래서 S24에서 실제로 어떻게 움직이는지는 아직 모릅니다.
아래 버그는 심각한 것부터 적었습니다. 줄 번호는 지금 작업 트리 기준입니다.

## 문제 없음으로 확인한 것
- 부호와 감싸기: `phase = 렌즈5 − 렌즈2`입니다. ids가 [2,5]이면 lens1 − lens0이라서 측정 메모와 부호가 같습니다. 위상이 양수이면 렌즈 5 주기를 줄이는데, 이 방향이 맞습니다. `wrap()`은 값을 [−T/2, T/2]로 맞춥니다. ids가 [5,2]여도 부호는 그대로입니다.
- status 형식: 첫 낱말은 바뀌지 않습니다. `ERROR`·`OPEN` 줄에는 꼬리를 붙이지 않고, `warn=`은 계속 맨 끝에 남습니다. 그래서 `phone_link.parse_status`(k=v 방식), `win_relay`(첫 낱말 방식), `h264Error`(` warn=`부터 잘라 냄)는 깨지지 않습니다.
- timestamp: 센서 시계는 계속 앞으로만 갑니다. 세션을 다시 만들어도 렌즈별 timestamp가 뒤로 가지 않습니다. 그래서 PC의 `reordered`는 늘지 않을 것으로 봅니다. 결과가 안 오는 프레임은 matcher의 `unmatched_dropped`로 셉니다.
- H.264: 인코더는 멈추지 않고 계속 돌고 I-프레임은 1초마다 나옵니다. 그래서 재시작한 뒤 키프레임을 따로 넣을 필요가 없습니다. 녹화는 byte[]로 받기 때문에 세션을 다시 만들어도 Surface가 끊기지 않습니다.
- 교착: 잠금 순서는 언제나 `meta` → `PhaseSync`입니다. 카메라 스레드는 `PhaseSync` 잠금을 잡은 채 `status()`를 부르지만 `meta`는 잡지 않습니다. 그래서 서로 기다리다 멈추는 일은 없습니다.
- 녹화 gate가 렌즈별·프레임별로 열려도 괜찮습니다. 복원은 timestamp로 짝을 짓기 때문입니다(`recording.iter_pairs`).

## H1. 방법 A 요청이 두 렌즈의 초점·OIS·fps 범위를 기본값으로 되돌릴 수 있음 (MainActivity `buildRequest`, 약 832–866줄)
- 맞추는 동안 쓰는 요청은 `createCaptureRequest(TEMPLATE_PREVIEW, {2,5})`로 만듭니다. 이렇게 만들면 렌즈마다 따로 쓰는 설정(physical settings)이 **템플릿 기본값으로 새로 채워집니다**. `request.set(...)`으로 넣은 값(AF 끔, `LENS_FOCUS_DISTANCE`, OIS 끔, `CONTROL_AE_TARGET_FPS_RANGE`)은 논리 카메라에만 들어갑니다.
- 이 키들이 `getAvailablePhysicalCameraRequestKeys()`에 들어 있으면, 맞추는 동안에는 HAL이 렌즈별 기본값을 씁니다. 그러면 렌즈 2·5 둘 다 AF가 연속 초점으로 돌아가서 초점이 흔들리고(focus breathing 때문에 dy가 1–2 px 바뀜), OIS가 켜지고, fps 범위가 가변으로 바뀔 수 있습니다. **기준인 렌즈 2까지 흔들립니다.** fps 범위가 가변이 되면 렌즈 2의 주기도 바뀌어 위상 측정이 틀어집니다.
- 고칠 방법: physical 키 목록에 있는 키는 논리 값과 똑같이 `setPhysicalCameraKey(key, 값, id)`로 **두 렌즈 모두에** 복사합니다. 대상 키는 AF_MODE, LENS_FOCUS_DISTANCE, LENS_OPTICAL_STABILIZATION_MODE, CONTROL_VIDEO_STABILIZATION_MODE, CONTROL_AE_TARGET_FPS_RANGE입니다. 아니면 set에 렌즈 5만 넣고 렌즈 5에 같은 복사를 합니다. caps.json `logical.physical_request_keys`에 이 키들이 있는지 먼저 봐야 합니다.

## H2. 방법 A의 위상 맞추기 제어가 안정 한계 근처라 흔들리기 쉽고, 한 번 시간 초과로 실패하면 A를 영영 안 씀 (PhaseSync `stepA` 223–256줄, `failA` 258–265줄)
- 위상은 요청한 주기 차이를 프레임마다 더해 가는 적분기입니다. 여기에 비례 이득 K = 0.15/프레임을 걸고 있습니다.
- 그런데 되먹임이 몇 프레임 늦게 돌아옵니다. median(5)에서 약 2프레임, 요청이 실제로 적용되기까지(pipeline) 3–6프레임, 3프레임마다 한 번 요청하는 것에서 0–3프레임이 늦어집니다. 합치면 d ≈ 6–10프레임입니다.
- 이 경우 안정 한계는 2·sin(π/(2(2d+1))) ≈ 0.24(d=6) … 0.15(d=10)입니다. 0.15는 그 경계에 있어서, 많이 출렁이거나 계속 진동할 수 있습니다.
- ±2 % 제한까지 겹치면 더 나빠집니다. 15 fps, +26 ms에서 시작하면 1.33 ms/프레임으로 다가가다가 지연 때문에 약 d×1.33 ≈ 8–13 ms를 지나칩니다.
- `good >= 3`(1 ms 안쪽이 3번 이어짐)을 못 채우면 20초 시간 초과로 `failA`에 갑니다. 그러면 `aFailed = true`가 **영구히** 남아서, A가 실제로는 먹히는 폰이어도 그 뒤로는 B만 씁니다.
- 고칠 방법:
  1. 이득을 0.05 안팎으로 낮추거나, 필요한 이동량을 한 번에 계산해서 고정 주기를 N프레임 동안 걸고 되돌린 뒤, pipeline 깊이와 median 창만큼 기다렸다가 다시 잽니다(deadbeat).
  2. 렌즈 5 결과의 `SENSOR_FRAME_DURATION`(`tgtDur`는 지금 모으기만 하고 안 씀)으로 요청이 실제로 적용된 시점을 알 수 있습니다. 적용되기 전 샘플은 제어에 쓰지 않습니다.
  3. 시간 초과는 그 회차만 실패로 둡니다. "요청을 안 따름"일 때만 영구 실패로 둡니다.

## H3. 미끄러짐 보정(feed-forward)이 전제를 확인하지 않고, 렌즈 5의 실제 주기를 재지 않음 (PhaseSync 243–247줄)
- 지금 계산은 `dur = T_ref − K·err − slip/프레임`입니다. 이 식은 "요청한 주기와 실제 주기의 차이 = 평소 미끄러짐"이라고 가정합니다.
- 폰이 요청한 주기를 정확히 지키면 이 보정은 오히려 −slip만큼 틀립니다. 그러면 err ≈ −slip/K 근처에서 멈춥니다. 30 fps(0.133 ms/프레임)에서는 약 0.9 ms로, 1 ms 잠금 기준의 경계라 맞을 때도 있고 못 맞을 때도 있습니다. 실제 차이가 다른 값이면 그만큼 다른 곳에서 멈춥니다.
- 고칠 방법: 렌즈 5 timestamp 간격의 중앙값(또는 결과의 `SENSOR_FRAME_DURATION`)으로 "실제 주기 − 요청 주기"를 그때그때 추정해서 보정합니다. 아니면 적분 항을 조금 더합니다.

## M1. `slip_ms_s`가 1000배 큼 (PhaseSync 313줄)
- `slopeNsPerNs`의 단위는 ns/ns(= ms/ms)입니다. ms/s로 바꾸려면 ×1000만 하면 됩니다. 그런데 지금 코드는 `Math.round(slopeNsPerNs * 1e6 * 1000) / 1000.0`입니다. 4 ms/s를 4000으로 보고하고, 웹 고급 줄에도 "미끄러짐 4000 ms/s"로 나옵니다.
- 고칠 코드: `Math.round(slopeNsPerNs * 1e3 * 1000) / 1000.0`. 제어 계산(`slopeNsPerNs * T`)은 맞습니다. 표시와 metrics/meta만 틀립니다.

## M2. 방법 B는 구조상 성공률이 낮은데 포기하지도 않고, 녹화 중에도 돎 (PhaseSync 158–159, 165–166, 268–293줄)
- 재시작할 때마다 위상이 무작위로 정해진다고 가정합니다. 그러면 ±2.5 ms 안에 들어갈 확률은 한 번에 5/T입니다. 15 fps에서 7.5 %, 30 fps에서 15 %입니다. 10번 다 실패할 확률은 15 fps에서 약 46 %, 30 fps에서 약 20 %입니다.
- 한 회차에 10 × (0.7–1.5 s 기다림 + 10프레임)이 걸리니 14–22초 동안 영상이 자꾸 끊깁니다. 그 뒤 `failed`가 되고, 60초 쉰 다음 다시 10번을 합니다. 이것이 **끝없이** 되풀이되어 1분마다 VIO가 깨집니다.
- 잠긴 뒤에도 4 ms를 넘게 미끄러지면 B가 다시 돕니다. 15 fps(0.05–0.08 ms/s)에서는 약 50–80초마다, 30 fps(4 ms/s)에서는 잠금 뒤 약 1.6초 만에 다시 돌아서 거의 계속 재시작합니다. `mode=record`에서 gate가 열린 뒤나 `mode=both`이면 그때마다 MP4에서 0.7–1.5초가 빠집니다.
- 고칠 방법:
  1. 녹화 중(gate가 열린 뒤, both)에는 B로 다시 맞추지 않고 meta에 기록만 합니다.
  2. 다시 맞추는 횟수에 세션당 상한을 두고, 넘으면 `failed`에 머무르게 합니다.
  3. 미끄러짐이 2 ms/s를 넘으면(30 fps) B를 쓰지 않습니다.

## M3. 30 fps에서 방법 A가 잠금과 풀림을 계속 되풀이하고, 렌즈 5 노출이 거의 내내 고정됨 (PhaseSync 31, 156–159, 226–229줄)
- 1 ms 안쪽이 되어 잠그면 원래 요청으로 돌아갑니다. 그러면 0.133 ms/프레임으로 다시 미끄러져 약 15프레임(0.5 s) 뒤에 2 ms를 넘고, 다시 맞추기 시작합니다.
- 그래서 렌즈 5는 AE 꺼짐 상태로 거의 계속 있습니다. `tgtExp/tgtIso`는 override가 꺼진 동안에만 갱신되는데, 그 사이가 ~15프레임뿐이고 AE 결과도 몇 프레임 늦게 옵니다. 결국 처음에 고정한 노출에 가깝게 계속 머물러서, 빛이 바뀌면 두 렌즈의 밝기가 벌어집니다.
- 고칠 방법: 미끄러짐이 크면 주기만 계속 조절(추적)하는 쪽이 낫습니다. AE 끄기 없이 할 수 있는지는 caps.json으로 확인합니다. 아니면 다시 맞추는 기준을 미끄러짐에 비례해 넓힙니다.

## M4. `sync=a`인데도 B가 돎 (PhaseSync 192, 207, 165–166줄)
- `useA()`는 `aPossible && !aFailed`를 봅니다. 그래서 (1) `sync=a`인데 A가 처음부터 불가능하면 바로 B로 갑니다. (2) `failA`에서 `failed`가 되고 60초 뒤 `measuring` → `decide` → `startAlign`으로 가는데, 이때 `useA()`가 false라 B를 돌립니다.
- 고칠 방법: `mode == "a"`이면 B를 아예 고르지 않습니다. A가 불가능하면 `failed`(또는 `off`)로 둡니다.

## M5. 원래 요청으로 되돌리기가 실패해도 무시함 (PhaseSync `endNudge` 218–221줄)
- `act.applyDuration(0,0,0)`이 false를 돌려줘도 `overrideOn = false`가 됩니다. 이때 HAL은 계속 짧은 주기 + 렌즈 5 AE 꺼짐 상태로 돌 수 있습니다. 그 뒤 `free` 샘플이 override 상태에서 쌓여 미끄러짐 추정도 틀어집니다.
- 고칠 방법: false이면 다음 프레임에서 다시 시도하고, 그동안은 `overrideOn`을 유지합니다.

## L1. status를 다시 쓸 때 경쟁 조건 (MainActivity `syncChanged` 약 375–383줄)
- 카메라 스레드는 `cur = lastStatus`를 읽고 `withSync(cur)`로 다시 씁니다. 그런데 그 사이에 Recorder 스레드가 `RECORD_DONE …`을 쓰거나 인코더 스레드가 `warn=LIVE_ENCODER`를 쓰면, 옛 `RECORDING …` 줄이 그것을 덮어씁니다.
- `RECORD_DONE`이 사라지면 PC가 정지를 기다리다 시간 초과가 날 수 있습니다. 타이밍이 좁게 맞아야 생기는 일이지만, `status()`와 `syncChanged()`를 같은 잠금으로 묶고, sync 꼬리를 뺀 원래 줄(base)을 따로 들고 있다가 다시 붙이는 편이 안전합니다.

## L2. 세션을 다시 만들다 실패하면 회복하지 못함 (MainActivity `createSession` `onConfigureFailed`)
- B가 세션을 다시 만들다 `CONFIGURE_FAILED`가 나면 `session = null`인 채로 남습니다. 영상이 끊기고 PhaseSync는 결과를 영영 받지 못합니다. 첫 낱말이 바뀌므로 PC가 앱을 다시 켜서 회복하는지 확인해야 합니다.
- 고칠 방법: `first == false`에서 실패하면 한 번 더 시도하거나 `ERROR`로 분명히 알립니다.

## L3. PC 쪽에 낡은 sync 값이 남음
- `scan3d.py` 1191줄: 실시간 모드의 `app_status`는 `start_app` 때 한 번만 찍어 둔 값입니다(꼬리가 보통 `sync=measuring`). `ui.sync`가 없으면, 예를 들어 부하 모니터가 꺼져 있으면, `syncOf()`가 이 낡은 값을 씁니다. 그러면 고급 줄은 계속 "재는 중"을 보여 주고, 짝 경고 안내도 "자동으로 맞춥니다"로 고정됩니다. 녹화가 끝나 `rec_status`에서 꼬리가 사라질 때도 똑같이 이 시작 때 값으로 돌아갑니다.
- `web_server.py` 476–485줄: `perf`가 dict인데 `sync`가 없으면(옛 앱 등) `phone_sync`가 지워지지 않고 그대로 남습니다.
- `app.js` 265–266줄: 녹화 중 B 재시작이나 다시 맞추기로 `aligning`이 되면, gate가 이미 열려 녹화가 진행 중인데도 "맞춘 뒤 녹화 영상이 시작됩니다 (최대 8초)"가 뜹니다. 그 사이 녹화 시간 표시도 가려집니다.

## L4. 줄바꿈이 CRLF로 바뀜
- `scan3d/scan3d.py`, `scan3d/web_server.py`, `scan3d/web/app.js`가 CRLF가 되었습니다(HEAD는 LF). 그래서 diff가 2948/1433/1704줄로 커졌지만, 실제로 바뀐 줄은 14/13/44줄입니다.
- `#!/usr/bin/env python3\r` 때문에 `./scan3d.py`로 바로 실행하면 실패합니다. 실행 스크립트는 `"$PY" scan3d.py`로 부르므로 그쪽은 괜찮습니다. 커밋하기 전에 LF로 되돌리는 것을 권합니다.

## 실행 전에 caps.json에서 볼 것
`logical.physical_request_keys`에 SENSOR_FRAME_DURATION, CONTROL_AE_MODE, SENSOR_EXPOSURE_TIME, SENSOR_SENSITIVITY가 있는지 봅니다. H1 때문에 CONTROL_AF_MODE, LENS_FOCUS_DISTANCE, LENS_OPTICAL_STABILIZATION_MODE, CONTROL_AE_TARGET_FPS_RANGE도 함께 봅니다. `request.pipelineMaxDepth`도 확인합니다(H2의 지연 d).

고침: 앱 1.6-sync (versionCode 10, 2026-10-16) — S24 실측(A·B 위상 안 옮김, 장치 close/open 때만 바뀜)에 따라 auto 기본 = 방법 C (CameraDevice 닫고 다시 열기, 3~5 프레임 재기, 2.5 ms 안이면 잠금, 25 번 + 30 s 쉬고 반복, 녹화 중엔 안 열고 drift 경고), A·B 는 sync=a|b 에서만. H1 렌즈별 AF·초점·OIS·영상안정화·fps 범위 물리 키 복사, H2.3 시간 초과는 영구 실패 아님, M1 slip_ms_s ×1e3, M2 녹화 중 B 안 함, M4 sync=a 에서 B 안 감, M5 되돌리기 실패 시 매 프레임 다시, L1 status 한 잠금 + base 줄 + changed() post, L2 세션 재생성 실패 1번 더 → ERROR, L3 scan3d 낡은 app_status 안 씀·web_server 부하 모니터 꺼져도 sync 갱신·sync 없으면 지움·웹 녹화 대기 문구는 rec_wait 일 때만, L4 LF 로 되돌림. H2(이득·지연)·H3(미끄러짐 보정 전제)·M3 은 A 를 auto 에서 안 쓰므로 그대로.

## 검토2: 앱 1.6-sync 방법 C (카메라 닫고 다시 열기) 독립 검토 (2026-10-16)

대상: `PhaseSync.java`, `MainActivity.java`, `Caps.java`, `AndroidManifest.xml`, `phone_link.py`, `scan3d.py`, `web_server.py`, `web/app.js`, `SPEC.md` 15절.
코드는 읽기만 했습니다. 앱 실행·측정은 하지 않았습니다(스캐너 서버가 돌고 있음). 그래서 한 번 다시 여는 데 걸리는 시간과 잠금이 얼마나 자주 되는지는 아직 모릅니다.
줄 번호는 지금 작업 트리 기준입니다.

### 문제 없음으로 확인한 것
- 설치: 폰 `versionName=1.6-sync`, `versionCode=10`이 맞습니다. APK(14:02:15)가 소스(14:01:58)보다 나중에 만들어졌습니다. `py_compile`은 통과했습니다. WSL PATH에 node가 없어서 `node --check`는 다시 돌리지 못했습니다.
- 줄바꿈(REVIEW8 L4): `scan3d.py`, `web_server.py`, `app.js`, `phone_link.py`, `MainActivity.java`, `PhaseSync.java`, `SPEC.md` 모두 CR이 0개입니다. 다 LF입니다.
- 다시 여는 순서(`doReopen` 743–759줄): 세션 close → 장치 close → 매처 비우기 → 같은 id로 `openCamera`. ImageReader는 그대로 다시 씁니다. 이미지는 `receive`에서 바로 닫히므로 붙잡힌 버퍼가 없습니다. `onOpened`는 `!running`이면 장치를 닫습니다. `ERROR_CAMERA_IN_USE`처럼 열기 중에 생기는 `onError`와 `openCamera` 예외는 0.7초 간격으로 3번 다시 시도합니다. 그래서 앱이 죽는 경로(잡히지 않는 예외)는 찾지 못했습니다.
- 녹화 중에는 다시 열지 않습니다. 이 검사가 세 곳에 있습니다: `startAlign`/`beginReopen`의 `mayRestart`, 잠금 뒤 어긋남이면 `hold`, 그리고 `doReopen` 안의 `mayRestartCamera` 재확인. 그래서 post가 늦게 실행되는 사이에 gate가 열려도 막힙니다. `mode=record|both`에서 gate가 닫혀 있는 동안 프레임은 녹화기로 가지 않습니다. 프레임 없이 끝나도 `muxing=false`라 muxer.stop 예외가 나지 않습니다.
- 첫 프레임과 PC: 새 장치의 timestamp도 같은 부팅 시계(elapsedRealtime)라서 렌즈별로 앞으로만 갑니다. 그래서 PC `reordered`는 늘지 않을 것으로 봅니다. 실시간 H.264 인코더는 멈추지 않고 I-프레임이 1초마다 나옵니다. 그래서 다시 연 뒤에 키프레임을 따로 넣을 필요가 없습니다. both의 녹화 인코더는 gate가 열린 뒤 첫 입력부터 새로 시작합니다.
- PhaseSync 초기화: 새 장치의 첫 결과에서 `C_MEASURE`로 바뀌고 `resetSamples`, `periods.clear`, `lastRefTs=-1`, 2개 버림이 일어납니다. 옛 장치 결과는 `C_REQUESTED` 동안 버립니다. 25번 실패 → `failed` 30초 → `measuring` → 새 회차, 이 끝없는 반복에는 매번 쉬는 시간이 들어갑니다. 확률 계산도 확인했습니다: 1−0.925^25 = 86 %, 1−0.85^25 = 98 %.
- REVIEW8 고침: H1(물리 키를 두 렌즈에 복사, override 요청일 때만), M1(×1e3 = ms/s 맞음), M4, M5, L1(잠금 순서 statusLock → PhaseSync, PhaseSync는 changed를 post만 함), L2, L3(web_server는 부하 모델이 꺼져도 sync를 갱신하고 sync가 없으면 지움, scan3d는 녹화 모드에서만 status 꼬리를 씀)이 다 들어갔습니다.

### 검토2-H1. 카메라가 다시 열리지 못하면 회복되지 않고, 웹은 노란 "곧 다시 시도"를 계속 보여 줌 (MainActivity 268–284, 205–207, 422–433줄; PhaseSync 222–224, 307–311줄; scan3d.py 773–786; app.js 379줄)
- `ERROR CAMERA_REOPEN_FAILED`, `ERROR CONFIGURE_FAILED (phase sync)`, 그리고 다시 연 뒤에 생기는 `CAMERA_ERROR`가 모두 같은 문제를 겪습니다. 이 셋은 장치나 세션 없이 끝납니다. 그런데 이 경우를 복구하는 경로가 없습니다.
  1. PhaseSync의 쉬는 시간 확인(`S_FAILED` → `measuring`)은 `onResult` 안에서만 돕니다. 결과가 더는 오지 않으니 영원히 `failed`에 머뭅니다. 앞의 두 오류에서는 `aligning`/`C_MEASURE`에 머뭅니다.
  2. 코드 주석은 "the PC restarts the app"라고 하지만, PC는 소켓이 3번 연달아 바로 끊길 때만 앱을 다시 켭니다(scan3d.py 773줄). 이때 소켓과 IMU는 살아 있으므로 다시 켜지 않습니다.
  3. 웹의 `se.app`은 앱을 시작할 때 한 번 찍어 둔 값입니다. 그래서 실시간 모드에서 status가 `ERROR`로 바뀐 것을 웹은 모릅니다. metrics `retry_in_s`가 0까지 줄어든 뒤에도 "0초 뒤 다시 시도합니다"나 "카메라 다시 여는 중 (N번째)"가 노랑으로 끝없이 나옵니다. 녹화 모드에서도 `rec_wait=1`이 그대로 남아 같은 노란 줄이 나옵니다.
- 상황 예: 다른 앱이 카메라를 잠깐 잡고 있어서 열기를 4번 다 실패하면, 앱은 영상 없이 IMU만 보내는 상태로 계속 남습니다.
- 고칠 방법:
  1. 앱: `reopenFailed` 뒤에 `handler.postDelayed`로 C_REST 뒤 다시 엽니다(결과를 기다리지 않음). 아니면 PhaseSync 상태를 `error`로 두고 metrics에 내보내서, 웹이 빨강 "폰 카메라 오류 — [■ 정지] → [▶ 스캔 시작]"을 보여 주게 합니다.
  2. PC: 실시간이나 both이고 녹화 중이 아닐 때, `link.status_raw()` 첫 낱말이 `ERROR`/`CAMERA_ERROR`/`DISCONNECTED`이면 앱을 다시 켭니다.

### 검토2-M1. 30 fps에서는 잠근 뒤 20초 동안 위상이 다 흘러가도 `locked`(웹 "맞음", 초록)로 보임 (PhaseSync 208–217줄; app.js 396–404줄)
- 추가한 20초 규칙 자체는 찬성합니다. 이 규칙이 없으면 1–2초마다 카메라를 다시 엽니다.
- 그런데 30 fps는 약 4 ms/s로 미끄러집니다. 그래서 잠금 뒤 약 0.4–1.6초 만에 ±4 ms를 넘고, 20초 동안 한 주기(33 ms)를 두 바퀴 넘게 돕니다. 짝 기준 5 ms 안에 있는 시간은 대략 10/33 ≈ 30 %뿐입니다. 그동안 상태는 `locked`이고, 짝 경고 안내는 "앱이 자동으로 맞춥니다"로 나옵니다.
- record·both 모드에서는 잠그는 순간 gate가 열립니다. 그 뒤 1초 안에 `drift`가 되므로, 30 fps 녹화는 사실상 늘 어긋납니다. SPEC의 "30 fps 25번 안에 ~98 %"는 잠금에 성공할 확률일 뿐이고, 맞은 상태가 유지되는 시간은 약 1초입니다.
- 고칠 방법:
  1. `S_LOCKED`에서 |위상| > 4 ms인데 20초가 아직 안 지났으면, `locked`에 머물지 말고 `drift`로 둡니다(hold_reason에 "미끄러짐 X ms/s, N초 뒤 다시 맞춤").
  2. 미끄러짐이 1 ms/s를 넘으면 SPEC과 웹에 "이 fps에서는 맞춰도 금방 어긋남(15·20·24 fps 권장)"이라고 적습니다.

### 검토2-M2. 5초 watchdog과 0.7초 재시도가 겹치면 카메라를 두 번 열 수 있음 → 장치가 새고, ImageReader가 늘고, status가 `DISCONNECTED`로 남음 (MainActivity 196–208, 258, 268–284, 287–293줄)
- `reopenWatchdog`는 `camera==null`이라는 것만 보고 `reopenOpenFailed`를 부릅니다. 그런데 그 순간 열기가 아직 진행 중(5초 넘게 걸림)이거나 0.7초 재시도가 대기 중일 수 있습니다. 그러면 `openCamera`가 하나 더 나갑니다.
- 그 뒤에 이런 일이 생길 수 있습니다:
  1. 같은 앱이 같은 카메라를 두 번 열면 먼저 연 장치가 쫓겨납니다. 그때 `onDisconnected`가 `camera==null` 상태에서 오면 `d!=camera && camera!=null`이 거짓이 되어 `status("DISCONNECTED")`가 됩니다.
  2. 두 번째 `onOpened`가 `reopening=false`일 때 오면 `configure()`가 불립니다. 그러면 ImageReader가 새로 2개 더 `readers`에 붙고(닫히지 않음), 이미 열려 있던 `camera`는 닫히지 않은 채 덮어써집니다.
- 그리고 다시 열기에 성공해도 `createSession(false, …)`은 로그만 남깁니다. 그래서 중간에 `DISCONNECTED`나 `CAMERA_ERROR`가 써졌으면 영상이 다시 와도 status 첫 낱말은 그대로입니다(phone_link `BAD_WORDS`).
- 고칠 방법:
  1. `onOpened`에서 `!reopening && camera!=null`이면 `d.close()`만 하고 끝냅니다(늦게 온 중복 열기).
  2. watchdog은 진행 중인 열기나 대기 중인 재시도가 없을 때만 다시 엽니다(`openInFlight` 플래그).
  3. 다시 여는 중에 오는 `onDisconnected`는 `reopenOpenFailed`로 보냅니다.
  4. `reopened()`의 세션이 `onConfigured`되면 `status(runningState()+…)`를 다시 씁니다.

### 검토2-L1. 옛 세션 결과가 "새 장치 첫 결과"로 잡힐 수 있음 (MainActivity 170–201줄 `timing`, PhaseSync 171–176줄)
- `doReopen`이 실행되기 전에 handler에 이미 들어와 있던 옛 세션의 `onCaptureCompleted`는 `reopenClosed()` 다음에 처리됩니다. 그래서 `C_CLOSED` → `C_MEASURE`로 넘어갑니다.
- 그러면 `c_open_ms`가 ≈0으로 기록됩니다. 또 버림 2개가 옛 결과로 소비되어, 새 장치의 첫 프레임까지 측정에 들어갑니다. 매처에도 옛 결과가 들어가 `unmatched_dropped`가 조금 늡니다.
- 고칠 방법: `timing`에서 `s != session`이면(다시 여는 중 포함) 무시합니다.

### 검토2-L2. `sync=a`에서 "영구 실패"(`aFailed`)가 더는 읽히지 않음 (PhaseSync 266–271, 390–397줄)
- `useA()`를 지우면서 `aFailed`를 확인하는 곳이 없어졌습니다. 그래서 "요청을 안 따름"으로 영구 실패해도 60초 뒤 `measuring` → `startAlign` → `beginNudge`로 A를 끝없이 다시 시도합니다.
- 시험용 모드라서 영향은 작습니다. `startAlign`에서 `aFailed`이면 `failed`(cooldown 없음)로 두면 됩니다.

### 참고 (버그는 아님)
- `rec_wait_sync_s` 기본값 40초와 한 회차(25번 × 한 번 약 1–1.6초 추정 = 25–40초)가 거의 같습니다. 그래서 15 fps에서 회차가 실패하는 약 14 %의 녹화는, 맞추지 못한 채 시작되고 끝까지 다시 맞추지 않습니다. 메인 세션에서 `c_try_ms_avg`를 잰 뒤에 기본값을 다시 정하기를 권합니다.
- 웹 문구는 쉬운 말이고 한국어입니다. 맞추는 중에는 빨간 짝 경고를 가리고 노란 안내를 보여 주는데, 이것도 의도대로입니다. 다만 위 H1의 경우에도 노란 안내가 남는 것은 고쳐야 합니다.
