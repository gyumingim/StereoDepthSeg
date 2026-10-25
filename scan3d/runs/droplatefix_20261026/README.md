# droplatefix 2026-10-26 — `--drop-late` permanent-drop bug fix (branch droplatefix-20261026, from integ-20261025 14de3b8)

## 결론
- 버그 (verify_ovgyro §5, 독립 검증자가 확인): `late_drop()` 의 기준값 `lag_min_ms_` 는 실행 전체의 최소값이고, 다시 잡히지 않으며, 연속으로 버리는 장수에 상한이 없음.
  → 시계 차이나 지연이 50 ms 넘게 **계속** 올라가 있으면 그 뒤 프레임을 모두 버림 (IMU 만 씀). 그동안 health 는 OK 이고, pred 스레드는 "tracking" 으로 외삽을 계속함.
- 고침: `--drop-late` 를 켜면 기본값이 mode 1. 추천 프로필 rec20261025 / rec60 에도 그대로 들어감. 최종 코드 = 커밋 2개 (3971b1d + 두 번째 커밋).
  1. **늦은 프레임의 지연이 평평할 때만** (최근 `--drop-late-win 3` s 동안 늘 지연 범위 ≤ 50 ms, 즉 버려도 안 줄어드는 영구 이동)
     연속 버림을 `--drop-late-cap 3` 장으로 제한함. 3장 버리면 다음 늦은 프레임은 그냥 넣음 ("forced").
     지연이 움직이면 (backlog 를 비우는 중이거나 과부하) 옛 규칙처럼 상한 없이 버림.
  2. 다시 잡기 (re-anchor): 3 s 내내 늦었는데 그 3 s 동안 지연이 평평하면, 그 최소값을 새 기준으로 함.
     지연이 아직 움직이면 다시 잡지 않음 (검증자 지적: 과부하 backlog 를 다시 잡으면 지연이 끝없이 커짐).
  3. 지연이 6 s (창 2개) 넘게 늦은 채로 계속 움직이면 "late-lost" 로 봄.
     이때 pose/pred status 는 lost, MAVLink/VIOP 는 lost (TRACKING 비트 꺼짐, quality 0, 하트비트 LOST), health 는 LOST.
     지연이 제자리로 돌아오거나 평평해지면 풂.
  4. `--health`: forced 프레임, 또는 상한보다 긴 늦은 구간에 들어간 프레임은 최소 DEGRADED. h 에 `late_run` 을 넣음.
  5. `--pred-stale-ms` 는 drop-late mode 1 이면 자동 500, 아니면 0 (꺼짐).
     예측 horizon (최신 IMU − 필터 상태 시각) 이 500 ms 를 넘으면 pred 줄에 `"status":"lost"` + `"stale":true` 를 넣고, MAVLink/VIOP 를 lost 로 보냄.
     선(wire) 프로토콜은 바뀌지 않음. ESP32 `link_task.c:54` 의 good = VALID && TRACKING && quality>0 조건으로 그대로 안전 쪽으로 감.
- 옛 규칙은 `--drop-late-mode 0` 으로 그대로 씀 (A/B 용). `--drop-late` 를 안 켜면 코드 경로와 출력이 integ 와 같음.
- 시험 전용: `--test-feed-sleep MS:T0` (기본 꺼짐) = 프레임마다 MS 를 쉬어 과부하를 흉내 냄.

## 증거 (PC, raw80, feeder3v 실시간 재생 = 검증자 재현 방식, 최종 빌드 sha c56bbe52.., 각 1회, PC 를 다른 에이전트와 같이 씀)
| 시나리오 (feeder3v, 25 s 에 400 ms, 45 s 에서 끊음) | integ 14de3b8 | 새 빌드 mode 0 | **새 빌드 mode 1 (기본)** |
|---|---|---|---|
| 영구 이동 +400 ms: 버림 | **600/600** (25 s 뒤 포즈 0) | **600/600** | 75 버림 + 22 forced, 3.03 s 에 re-anchor (+400.3 ms), 25 s 뒤 포즈 526 |
| 같은 경우 pred horizon 최대 / 상태 | 20.0 s, 내내 "tracking" | 20.0 s, "tracking" | 0.18 s, lost 없음 |
| 같은 경우 끝 위치 (멈춤 없는 실행과 비교) | 25 s 에서 멈춤 | 같음 | 약 3 mm 차이 (실행 사이 흔들림 수준) |
| 한 번 멈춤 (catch-up) | 15 버림 (1차) | 13 (1차) | 16 버림, forced 0 (= 옛 규칙). 1 줄 stale (h 0.501 s) |
| 과부하 (`--test-feed-sleep 150:25`, 약 4.5 배) | — (플래그 없음) | 687 버림, feeder 지연 ≤ 158 ms | 687 버림, ≤ 153 ms, re-anchor 0 (지연이 계속 커지지 않음) |
| 과부하 60 ms/프레임 | — | — | 494 버림, ≤ 58 ms |
| 정상 실시간 raw80 전체 82 s | 0, 0 (2회) | — | 0, 0, 0 (3회). h p99 0.078 s |
- 동일성 (drop-late 끔, pose-precision 17, 포즈 줄 비교): `--profile rec20261025 --drop-late 0` 과 프로필 없음 둘 다 2327/2327 IDENTICAL.
  첫 버전과 최종 버전에서 각각 확인함.
- 선(wire) 확인 (`dlf_udp.py` 로 VIOP UDP 를 받음, 1차 빌드, stale 코드는 최종과 같음):
  - integ: 영구 이동 뒤 20 s 동안 하트비트 TRACKING, POSE TRACKING q75 를 보냄 (horizon 6.5 s 에서 포화).
  - mode 0 + `--pred-stale-ms 500`: 25 s 초부터 POSE 3887 개 lost (q0), 하트비트 LOST 385 개.
- s0 (멈춤 없음) 최종 실행에서 4장 버림: 이 실행 때 feeder 자체가 109 ms 늦었음 (PC 부하). 1차 실행은 0장.
- 원자료:
  - `raw/`: 1차 빌드 결과 + 선 확인.
  - `raw2/`: 최종 빌드 결과.
  - 표: `analysis_round2_final.txt`, `analysis_round1_v1.txt`.
  - 스크립트: `scripts/`.

## ovpipe B_r2 (P_1 의 B60_r2) 240장 버림 — 같은 결함인가? (읽기 전용, `ovpipe_B60_r2_check.txt`, 검증자도 맞음 판정)
- **아님. drop-late 가 의도대로 일시 backlog 를 비운 것.**
  - 실행 시간 41.5 s 로 정상 (41.2 s) → 폰 절전이 아님.
  - 버림은 브리지가 멈춘 직후 (feed_ms 188~411 ms, 17회) 4~41 프레임씩 모여 있고, 매번 회복함. 처리 2087 + 버림 240 = 2327.
  - speed 2.0 재생이라 "지연"이 계속 줄어들고, 실행 최소값 기준도 저절로 내려감.
- 최종 mode 1 은 이런 경우 (지연이 움직임) 옛 규칙과 똑같이 버림.

## 남은 것 / 주의
- 폰(ARM) 재생은 하지 않음. Android 빌드는 컴파일만 확인함. 로직이 시계와 상관없어서 PC 로 충분하다고 판단했음 (추정).
- late-lost 경로 (지연이 6 s 넘게 계속 움직이는 경우, 예: 시계가 계속 벌어짐) 는 재현하지 못함. 코드 검토만 했음.
  이 경우에도 프레임을 다 버리면 horizon 이 0.5 s 를 넘어 stale lost 가 먼저 걸림 (P_r2 는 horizon 69 s 였음).
- 400 ms 한 번 멈춤이면 pred 1~몇 줄이 stale 로 lost 가 될 수 있음 (h 0.50 s).
  ESP32 가 good 아닌 포즈 한 장을 어떻게 다루는지 (pose_age 250 ms 판정) 는 확인하지 않음.
- 기본값 cap 3, win 3 s, stale 500 ms 는 판단으로 정한 값 (측정으로 정하지 않음). 정상 horizon p99 0.078 s 보다 약 6 배 여유가 있음.
