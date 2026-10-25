# droplatefix 2026-10-26 — `--drop-late` permanent-drop bug fix (branch droplatefix-20261026, from integ-20261025 14de3b8)

## 결론
- 버그 (verify_ovgyro §5, 독립 검증자가 확인): `late_drop()` 기준값 `lag_min_ms_` 가 실행 전체 최소값이고 다시 잡히지 않으며 연속 버림 상한이 없음
  → 시계 차이/지연이 50 ms 넘게 **계속** 오르면 그 뒤 모든 프레임을 버림 (IMU만), health OK 그대로, pred 스레드는 "tracking" 으로 외삽 계속.
- 고침 (`--drop-late` 를 켜면 기본 = mode 1, 추천 프로필 rec20261025 / rec60 포함):
  1. 연속 버림 상한 `--drop-late-cap 3`: 3장 버리면 다음 늦은 프레임은 그냥 넣음 ("forced").
  2. 기준 다시 잡기 `--drop-late-win 3.0` s: 3 s 동안 들어온 프레임이 전부 늦으면 (버림+forced) 그 구간 최소 지연을 새 기준으로.
     진짜 backlog 는 버림으로 지연이 기준 아래로 돌아오므로 3 s 를 못 채움 → 영구 이동만 다시 잡힘.
  3. `--health`: forced 프레임 / 상한 넘는 늦음 연속 구간은 최소 DEGRADED, h 에 `late_run`.
  4. `--pred-stale-ms` (drop-late mode 1 이면 자동 500, 아니면 0=꺼짐): 예측 horizon(최신 IMU − 필터 상태 시각) > 500 ms 면
     pred 줄 `"status":"lost"` + `"stale":true`, MAVLink quality 0 / CRITICAL, VIOP POSE 의 TRACKING 비트 끔·quality 0, VIOP 하트비트 LOST.
     프로토콜 변경 없음 (ESP32 link_task.c: good = VALID && TRACKING && quality>0 → 그대로 fail-safe 로 감).
- 옛 규칙은 `--drop-late-mode 0` 으로 그대로 (A/B 용). `--drop-late` 를 안 켜면 코드 경로·출력 동일.

## 증거 (PC, raw80, feeder3v 실시간 재생 = 검증자 재현 그대로, PC 공유 중이라 각 1회)
| 시나리오 (25 s 에 400 ms, 45 s 에서 끊음) | integ 14de3b8 | 새 빌드 mode 0 | 새 빌드 mode 1 (기본) |
|---|---|---|---|
| 영구 이동 (feeder mode 2) 버림 | **600/600** (25 s 이후 포즈 0) | **600/600** | 버림 69 + forced 22, 3.03 s 뒤 re-anchor (+400.0 ms), 이후 포즈 531 |
| 위 경우 pred horizon 최대 / 상태 | 20.0 s, 내내 "tracking" | 20.0 s, "tracking" | 0.16 s, 손실 없음 |
| 위 경우 끝 위치 vs 멈춤 없는 실행 | 25 s 에서 멈춤 | 같음 | 2 mm 이내 (0.110,−0.184,0.229 vs 0.112,−0.185,0.227) |
| 한 번 멈춤 (catch-up, mode 1) 버림 | 15 | 13 | 12 + forced 3, DEGRADED 3 프레임 |
| 멈춤 없음 45 s | 0 | 0 | 0 |
| 정상 실시간 raw80 전체 82 s ×2 | 0, 0 | — | 0, 0 (h p99 0.078 s, 0.15 s 넘는 것은 녹화 끝뒤 IMU만 남은 구간뿐) |
- 동일성 (drop-late 끔, pose-precision 17, 포즈 줄 비교): `--profile rec20261025 --drop-late 0` 2327/2327 IDENTICAL, 프로필 없음 2327/2327 IDENTICAL.
- 선(wire) 확인 (VIOP UDP 수신기 `dlf_udp.py`, 영구 이동): integ 는 25 s 뒤 20 s 동안 HB TRACKING + POSE TRACKING q75 (horizon 6.5 s 포화) 를 보냄.
  mode 0 + `--pred-stale-ms 500` → POSE 3887 개 lost(q0), HB LOST 385 개 (25 s 초부터). mode 1 은 프레임이 계속 처리돼 stale 이 안 걸림.
- 원자료: `raw/` (`*.err` 요약, `*.feeder`, `*.jsonl.gz` 포즈/pred, `*.viop.csv.gz`, `*.udp.json`), 표 = `analysis.txt`, 스크립트 = `scripts/`.

## ovpipe B_r2 (P_1 의 B60_r2) 240장 버림 — 같은 결함인가? (읽기 전용, `ovpipe_B60_r2_check.txt`)
- **아님 (다른 원인, drop-late 가 의도대로 동작)**. 실행 시간 41.5 s (정상 41.2 s) → 절전 아님. 버림은 브리지 멈춤(feed_ms 188~411 ms, 17회) 바로 뒤에
  4~41 프레임씩 모여 있고 매번 회복 (끝까지 2087 프레임 처리). 즉 일시 backlog 를 버림이 비운 것. 이 재생은 speed 2.0 이라 "지연" 자체가 계속 줄어들어
  run-min 기준이 저절로 내려감 → 영구 버림 경로가 생길 수 없음. mode 1 이면 이 구간에서 4장마다 1장은 처리됨 (결과 조금 달라짐, 추정).

## 남은 것 / 주의
- 폰(ARM) 재생 미실행 (야간 폰 사용 줄이기, 동작은 시계 무관 로직이라 PC 로 충분하다고 판단 — 추정). Android 빌드 컴파일만 확인.
- 라이브에서 지연이 천천히 (50 ms / 3 s 보다 느리게) 오르는 backlog 는 옛 규칙과 같이 잡힘 (기준은 실행 최소값 유지, 늦은 프레임만 3 s 연속일 때 다시 잡음).
- re-anchor 이후는 지연이 +400 ms 인 채로 추적 (pred horizon 은 IMU 도 같이 늦어 짧음). 실제 카메라만 늦어지면 horizon 이 커져 500 ms 에서 lost 로 표시됨.
- 기본값 cap 3 / win 3 s / stale 500 ms 는 판단값 (측정 기반 아님). 정상 horizon p99 0.078 s 와 6배 여유.
