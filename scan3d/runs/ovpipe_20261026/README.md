# ovpipe-20261026 — 다음 프레임 추적을 이번 프레임 갱신과 겹치기 (`ov_bridge --pipe 1`)

브랜치 `ovpipe-20261026` (integ-20261025 14de3b8 에서). **플래그 기본 꺼짐 = 예전과 같음.**

## 무엇을 바꿨나
| 파일 | 내용 |
|---|---|
| `ov_bridge/ovpatch/ovpipe.h`, `ovopt.cpp` 끝 | 작업 스레드 1개 (Pipe): 새 영상 기록이 들어오면 곧바로 remap + KLT 추적(프레임 N+1)을 계산해 결과(Res)에 담음. 메인 스레드는 그동안 프레임 N 의 갱신 |
| `ovpatch/TrackKLT.cpp` | `--pipe` 일 때만 쓰는 경로: 추적 계산 = 원래 코드와 같은 순서·같은 함수, 결과를 DB/멤버 대신 Res 로. 메인 스레드가 원래 자리(track_image_and_update 맨 앞)에서 Res 를 "적용"(DB update_feature 같은 순서, *_last 멤버) |
| `ovpatch/VioManager.cpp` | `--pipe` 일 때만 새 tracker 포인터를 브리지에 넘김 (읽기만) |
| `main.cpp` | `--pipe 0|1`: 읽기 스레드가 영상 기록을 작업 스레드에 넘김, 버린 프레임(크기 다름·시간 역행·큐 넘침·drop-late·RESET) 알림, idle-spin 2/3 은 작업 스레드가 일하는 동안 메인 스레드만 돈다 (OpenCV 풀을 비움), 끝에 `[ov_bridge] pipe: ...` 통계, ready 줄 `"pipe":1` |
| CMake (PC/폰 klt_bench) | ovopt.cpp 추가 (링크만) |

같음 보장 방법: 결과 Res 는 "적용된 마지막 상태" 에서 계산된 것일 때만 씀 (base id·tracker 세대·num_features 확인). 아니면 메인 스레드가
예전 방식대로 그 자리에서 다시 계산 → 결과는 순차 실행과 같다. 추적 계산은 필터 상태·특징 DB 를 읽지 않음 (코드 확인: TrackKLT 는 자기 멤버 + 영상 + 카메라 내부값만 씀). 카메라 내부값은 고정이어야 하므로 `calib_cam_intrinsics` 가 켜진 설정이면 `--pipe` 를 끔 (검증 에이전트 지적 02:15 반영). num_features 는 초기화 때 한 번 바뀌는데 작업 스레드가 동시에 읽을 수 있음 → 바뀐 경우 그 결과는 버리고 다시 계산 (형식상 경쟁, 결과에는 영향 없음).
조건: `--stdio`, mono KLT, `--predict` 를 쓰면 `--pred-thread 1` 필요, `--init-gate`/`--test-blank`/`--klt-overlap 1` 과 같이 못 씀 (오류로 거부).

## 결과
### PC 동일성 (`ident_pc.sh`, `ident_pc_1.log`, `ident_pc_2.log`; 17자리, 결정적이라 1회)
| 입력 | 비교 | 결과 |
|---|---|---|
| 하네스 h_L0.4nc 845 | integ 빌드 vs 이 빌드 (pipe 0 / pipe 1 / pipe 1 + pred-thread) | 845/845 IDENTICAL |
| raw80 2327 (c4 + remap) | integ vs pipe 0 / pipe 1; rec60 profile integ vs + pipe 1 | 2327/2327 IDENTICAL |
| raw80 + 37장마다 앞 프레임 복사 + RESET 1회 (inject dup) | pipe 0 vs 1 (기본, rec60) | 2389/2389 IDENTICAL (tracker 2개) |
| raw80 + 37장마다 크기 바꾼 복사 (거부 → 버림) + RESET | pipe 0 vs 1 | 2327/2327 IDENTICAL (버림 62, 재연결 56, 작업 스레드 예외 54 → 전부 처리) |

### 폰 재생 시간 (S24, 카메라 없음, raw80 c4 2 스레드, AC 충전 중, 시작 SKIN < 36)
60 fps 대용 = raw80(30 fps 녹화)을 2배 속도로 보냄 → 16.7 ms 마다 1장 (같은 영상, 프레임 사이 움직임은 실제 60 fps 의 2배 → 일이 더 많을 것, **추정**).
e2e = 브리지가 영상 기록을 다 읽은 때 → 그 프레임 처리 끝 (예측 스레드 "frame" 줄 tb). 큐 대기 + remap + 추적 + 갱신 전부. `phone_an.py`.

| 묶음 | 실행 | e2e p50 / p95 / max ms | e2e > 33 ms 프레임 | late drop | 포즈 = A |
|---|---|---|---|---|---|
| A = `--profile rec60` | A60_r1, A60_r2 (P_1), A60_r3 (P_3) | 42.7/56.8/77, 41.6/57.1/67, 33.4/56.5/70 | 1325, 1209, 1004 / 2003 | 0 | — |
| B = A + `--pipe 1` | B60_r1 (P_1), B60L_r1, B60L_r2 (P_3) | 19.2/29.0/51, 19.5/29.4/49, 18.3/27.7/50 | 32, 25, 19 | 0 | 2327/2327 |
| B + `--pipe-cpus 4-9` | B60C_r1, B60C_r2 (P_3) | 18.0/26.9/46, 19.5/30.4/44 | 11, 43 | 0 | 2327/2327 |
| B (이상) | B60_r2 (P_1) | 19.2/44.4/**1029** | 209 | **240** | 다름 (drop 때문) |

- A 는 프레임당 일(remap ~2 + 추적 ~7 + 갱신 ~6 ms)이 16.7 ms 를 자주 넘어서 큐에 1–2장이 늘 밀려 있음 (e2e p50 33–43 ms). B 는 추적을
  앞당겨 계산해서 큐 적체가 없어짐 (p50 18–19.5, p95 27–30 ms). 한 프레임 지연 자체(remap+추적+갱신 ≈ 19 ms)는 그대로라 e2e > 16.7 ms 프레임은 A·B 모두 1200–1480장 — "처리량"이 60 fps 를 따라가게 된 것이지 한 장 지연이 16.7 ms 아래로 간 것은 아님. 폰(ARM)에서도 A 와 B 포즈가 2327/2327 같음 (drop 없는 실행).
- **B60_r2 이상 (6번 중 1번)**: feed_ms > 40 ms 인 프레임 34장 (대부분 160–410 ms; e2e > 200 ms 14장, e2e 최대 1029 ms, claim 대기 최대 343 ms) → drop-late 240장. 시작 때 cpu9 상한 672 MHz (열 제한)
  였음. 원인 못 찾음; `--pipe-log` 을 붙인 4번 (P_3, 바이너리 ovpipe2, 시작 cpu9 상한 ≥ 1.92 GHz) 에서는 다시 안 생김 (claim 대기 최대 13–32 ms). 다만 672 MHz 상한 상태는 다시 만들지 않았고 (열 시험 금지) A 를 그 상태에서 돌린 적도 없어 pipe 탓인지 확인 안 됨. 추정 요인: drop-late 가 연달아 버리면 매번 뒤 프레임 결과를 다시 계산 (rechains 71, wasted 62) → 대기가 커지는 되먹임. **미해결 위험**.
- `--pipe-cpus 4-9` (작업 스레드를 큰/중간 코어에 묶기): 2번 중 1번 조금 좋음 — 차이 없다고 봄.
- 반복: A 3, B 3 (+ cpus 2, 이상 1). 같은 날 같은 충전 상태. 섞인 변수: 실행 순서별 온도(시작 SKIN 35.8–35.9 로 맞춤), cpu9 상한 변동.
