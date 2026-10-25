p='README.md'
s=open(p,encoding='utf-8').read()
def rep(old,new):
    global s
    assert s.count(old)==1,old
    s=s.replace(old,new)
rep("""  3. **LAND_OPENLOOP**: 자세 제어가 없습니다. 이번 비행 이륙 때 기록한 모터별 값에서 출력을 **역산**합니다. 추력 ∝ 명령² 모델을 쓰고 배터리 전압 차이를 보정합니다. 처음 0.4 s는 1 m/s²로 하강을 시작하고, 이후는 hover의 99 % 추력으로 내려갑니다. 마지막 VIO 높이로 예상 착지 시간을 계산해 그 1.2배 뒤에 모터를 끕니다.
""","""  3. **LAND_OPENLOOP**: 자세 제어가 없습니다. 이번 비행 이륙 때 기록한 모터별 값에서 출력을 **역산**합니다. 추력 ∝ 명령² 모델을 쓰고 배터리 전압 차이를 보정합니다. 처음 0.6 s는 1 m/s²로 하강을 시작하고, 이후는 hover의 95 % 추력으로 내려갑니다.
     - **모터 끄는 시점 (esp32fix 2026-10-26 고침)**: 마지막 VIO 높이에서, 기록값이 ±3 % 틀린 두 경우를 미리 계산해 가장 이른 착지 시각(t_fast)과 가장 늦은 착지 시각(t_slow)을 구합니다. t_fast부터 0.4 s 동안 추력을 hover의 70 %로 내리고, t_slow + 0.2 s에 모터를 끕니다. 폰 pose가 아직 오면(IMU 프레임만 끊긴 경우) 높이 < 0.10 m · |vz| < 0.2 m/s가 0.15 s 이어질 때 바로 끕니다.
     - 예전 방식(`openloop_cut_mode = 0`): 착지 후 약 1.7 s 동안 hover의 98 % 추력으로 돌았고, 기록값이 3 % 높으면 하강이 멈춰 공중에서 모터가 꺼졌습니다(시뮬레이션 1.6 m 높이).
""")
rep("""- **PC 단위시험: 389개 검사 모두 통과**""","""- **PC 단위시험: 510개 검사 모두 통과** (esp32fix 2026-10-26; 이전 389개)""")
rep("""r = 1 − 1.0/g for 0.4 s (descend at ~0.4 m/s), then r = 0.99. | cut at (estimated touchdown time from the last VIO height) × 1.2 + 0.3 s, max 6 s; or impact, if an accelerometer is still available (not with phone IMU loss) |""",
"""r = 1 − 1.0/g for 0.6 s (descend at ~0.6 m/s), then r = 0.95; from t_fast r ramps to 0.70 in 0.4 s. | **mode 1 (default, esp32fix):** at the tier start the profile is simulated from the last VIO height with the thrust off by −3 % / +3 % → earliest / latest touchdown t_fast / t_slow; cut at t_slow + 0.2 s (max 6 s). If poses + heartbeat still arrive: cut when height < 0.10 m and \|vz\| < 0.2 m/s for 150 ms. Impact cut if an accelerometer is still available. **mode 0 (legacy):** cut at (touchdown estimate with constant 0.4 m/s) × 1.2 + 0.3 s. |""")
rep("""- The OPENLOOP numbers are **provisional and will be tested in sim / real later**: thrust exponent, 1 m/s², 0.4 m/s, 0.99, margin 1.2.""",
"""- The OPENLOOP numbers are **provisional and will be tested in sim / real later**: thrust exponent, 1 m/s², 0.6 m/s, 0.95, ±3 % record error band, end ratio 0.70, ramp 0.4 s, hold 0.2 s (legacy: 0.4 m/s, 0.99, margin 1.2 with `openloop_cut_mode = 0`).""")
rep("""Result: **389 checks, 0 failed**. Raw log: `scan3d/runs/esp32fc_20261025/test_safety_run.log`.""",
"""Result: **510 checks, 0 failed** (esp32fix 2026-10-26; 389 before). Raw logs: `scan3d/runs/esp32fix_20261026/test_*.log` (earlier: `scan3d/runs/esp32fc_20261025/test_safety_run.log`).""")
rep("""- **Phone-IMU loss from 1 m:** ground contact after 1.86 s at up to 0.80 m/s. Motors are cut at 3.53 s, so they **spin on the ground for ~1.7 s at ~98 % of hover thrust**. That is a tip-over risk; tune `openloop_cut_margin` / `openloop_cruise_ratio` in tests.""",
"""- **Phone-IMU loss from 1 m (esp32fix, mode 1):** ground contact after 1.45 s at 1.24 m/s, motors cut at 2.05 s: 0.6 s on the ground, of which ~30 ms above 85 % of hover thrust (rest ramping to 70 %). Legacy (mode 0, esp32fc): contact 1.86 s at 0.80 m/s, cut 3.53 s = **~1.7 s on the ground at ~98 % hover**.
- **Sweep (`t_openloop_cut`, height 0.3 / 1 / 2 m × thrust error −3 / 0 / +3 %, 1 run each, deterministic sim):** mode 1 worst: impact 2.19 m/s, never cut in the air, on-ground spin ≤ 1.05 s, of it ≤ 0.18 s above 85 % hover. Legacy worst: cut in the air at 1.62 m (impact 5.64 m/s), on-ground spin 3.7 s at > 85 % hover. Python model of the same sweep (0.2–3 m, ±1.5 / ±3 %): `scan3d/runs/esp32fix_20261026/olsim3_out.txt`.
- **VIO still alive, IMU frames lost (mode 1):** cut 150 ms after ground contact (VIO touchdown).
- Trade-off (추정 아님, sim): impact speed at 1 m rises from 0.80 to 1.24 m/s (descent 0.6 m/s + end ramp) in exchange for ~1.1 s less spinning on the ground at near-hover thrust.""")
rep("""| open-loop landing | k = 2, 1 m/s², 0.4 m/s, 0.99, margin 1.2 | to be tested in sim / real (user) |""",
"""| open-loop landing | k = 2, 1 m/s², 0.6 m/s, 0.95, band ±3 %, end 0.70 / 0.4 s, hold 0.2 s, VIO touchdown 0.10 m / 150 ms (legacy `openloop_cut_mode = 0` + 0.4 m/s + 0.99) | to be tested in sim / real (user) |""")
open(p,'w',encoding='utf-8',newline='\n').write(s)
print('ok')
