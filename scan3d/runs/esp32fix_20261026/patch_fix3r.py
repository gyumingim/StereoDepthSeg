p = 'esp32fc/README.md'
s = open(p, encoding='utf-8').read()


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep("처음 0.6 s는 1 m/s²로 하강을 시작하고, 이후는 hover의 95 % 추력으로 내려갑니다.",
    "처음 0.4 s는 1 m/s²로 하강을 시작하고, 이후는 hover의 99 % 추력으로 내려갑니다. 마지막 VIO 높이로 예상 착지 시간을 계산해 그 1.2배 뒤에 모터를 끕니다 (기본 = `openloop_cut_mode 0`).")
rep("     - **모터 끄는 시점 (esp32fix 2026-10-26 고침)**: 마지막 VIO 높이에서,",
    "     - **선택 사항 `openloop_cut_mode 1` (esp32fix 2026-10-26, 기본 꺼짐 — 사용자가 정함; 함께 `openloop_vz_down 0.6`, `openloop_cruise_ratio 0.95`)**: 마지막 VIO 높이에서,")
rep("VIO로 하강을 본 뒤 높이 < 0.10 m · |vz| < 0.2 m/s가 0.15 s 이어질 때 바로 끕니다.",
    "VIO로 하강을 본 뒤 높이 < 0.10 m · |vz| < 0.2 m/s가 0.15 s 이어질 때 바로 끕니다. 타이머 시각에 VIO가 아직 하강을 보이면 최대 2 s 기다립니다. 기본 방식은 시뮬레이션에서 착지 뒤 98 % 추력으로 1.2–1.7 s 돌고, 기록값이 3 % 크면 공중에서 끕니다. 근거와 실패 범위: `scan3d/runs/esp32fix_20261026/README.md`.")
rep("**PC 단위시험: 516개 검사 모두 통과**", "**PC 단위시험: 573개 검사 모두 통과**")
rep("r = 1 − 1.0/g for 0.6 s (descend at ~0.6 m/s), then r = 0.95; from t_fast r ramps to 0.70 in 0.4 s.",
    "r = 1 − 1.0/g for 0.4 s (descend at ~0.4 m/s), then r = 0.99 (mode 1 as tested: 0.6 s / 0.6 m/s, then 0.95; from t_fast r ramps to 0.70 in 0.4 s).")
rep("**mode 1 (default, esp32fix):**", "**mode 1 (opt-in, esp32fix; default off):**")
rep("Impact cut if an accelerometer is still available. **mode 0 (legacy):**",
    "While VIO is usable and still shows a descent (vz ≤ −0.2 m/s) the timer cut waits, at most 2 s. Impact cut if an accelerometer is still available. **mode 0 (default, esp32fc):**")
rep("Result: **516 checks, 0 failed**", "Result: **573 checks, 0 failed**")
rep("- **Phone-IMU loss from 1 m (esp32fix, mode 1):** ground contact after 1.45 s at 1.24 m/s, motors cut at 2.05 s: 0.6 s on the ground, of which ~30 ms above 85 % of hover thrust (rest ramping to 70 %). Legacy (mode 0, esp32fc): contact 1.86 s at 0.80 m/s, cut 3.53 s = **~1.7 s on the ground at ~98 % hover**.",
    "- **Phone-IMU loss from 1 m (default mode 0):** contact 1.86 s at 0.80 m/s, cut 3.53 s = **~1.7 s on the ground at ~98 % hover** (this test's record is ~1 % low). Opt-in mode 1, record exact: contact 1.57 s at 1.23 m/s, cut 2.05 s.")
rep("mode 1 worst: impact 2.19 m/s, never cut in the air, on-ground spin ≤ 1.05 s, of it ≤ 0.18 s above 85 % hover. Legacy worst: cut in the air at 1.62 m (impact 5.64 m/s), on-ground spin 3.7 s at > 85 % hover.",
    "error relative to the actual record. Mode 1 worst: impact 2.35 m/s, no cut in the air **within the same model** (predictor = test plant), ≤ 0.12 s on the ground above 85 % hover. Mode 0 worst: cut in the air 3/9 (up to 3.37 m), on-ground spin 3.5 s at > 85 % hover. Failure ranges with other plant models: run README.")
open(p, 'w', encoding='utf-8', newline='\n').write(s)
print('ok')
