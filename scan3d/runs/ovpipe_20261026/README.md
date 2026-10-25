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
예전 방식대로 그 자리에서 다시 계산 → 결과는 순차 실행과 같다. 추적 계산은 필터 상태·특징 DB 를 읽지 않음 (코드 확인: TrackKLT 는 자기 멤버 + 영상만 씀).
조건: `--stdio`, mono KLT, `--predict` 를 쓰면 `--pred-thread 1` 필요, `--init-gate`/`--test-blank`/`--klt-overlap 1` 과 같이 못 씀 (오류로 거부).

## 결과
### PC 동일성 (`ident_pc.sh`, `ident_pc_1.log`, `ident_pc_2.log`; 17자리, 결정적이라 1회)
| 입력 | 비교 | 결과 |
|---|---|---|
| 하네스 h_L0.4nc 845 | integ 빌드 vs 이 빌드 (pipe 0 / pipe 1 / pipe 1 + pred-thread) | 845/845 IDENTICAL |
| raw80 2327 (c4 + remap) | integ vs pipe 0 / pipe 1; rec60 profile integ vs + pipe 1 | 2327/2327 IDENTICAL |
| raw80 + 37장마다 앞 프레임 복사 + RESET 1회 (inject dup) | pipe 0 vs 1 (기본, rec60) | 2389/2389 IDENTICAL (tracker 2개) |
| raw80 + 37장마다 크기 바꾼 복사 (거부 → 버림) + RESET | pipe 0 vs 1 | 2327/2327 IDENTICAL (버림 62, 재연결 56, 작업 스레드 예외 54 → 전부 처리) |

### 폰 재생 시간 — 아래 (측정 뒤 채움)
