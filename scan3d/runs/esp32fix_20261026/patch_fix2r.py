def patch(p, pairs):
    s=open(p,encoding='utf-8').read()
    for old,new in pairs:
        assert s.count(old)>=1,(p,old)
        s=s.replace(old,new)
    open(p,'w',encoding='utf-8',newline='\n').write(s)
R='scan3d/runs/esp32fix_20261026/README.md'
patch(R,[
("(1 m → 0.97 m 높이, 2 m → 1.62 m, 낙하 5.6 m/s)","(C 시뮬레이션: 1 m → 0.09 m 높이, 2 m → 1.62 m 높이에서 꺼져 5.6 m/s 낙하)"),
("폰 pose 가 아직 오면 높이 < 0.10 m · |vz| < 0.2 m/s 가 0.15 s 이면 바로 끔.","폰 pose 가 아직 오면, VIO 로 하강(vz < −0.3 m/s)을 본 뒤에 높이 < 0.10 m · |vz| < 0.2 m/s 가 0.15 s 이면 바로 끔."),
("| **510 통과** (ASan/UBSan) | `test_m1b.log`","| **516 통과** (ASan/UBSan) | `test_fix2.log` (스윕 수치는 `test_m1b.log` 와 같음)"),
("| 펌웨어 문법 검사 6 조합 | 통과 | 통과 | `syntax_m1b.log` |","| 펌웨어 문법 검사 6 조합 | 통과 | 통과 | `syntax_fix2.log` |"),
("""## 확인 안 된 것""","""## 독립 검증 (읽기 전용 에이전트, 22:45)
- 고친 것 (확인된 버그): VIO 착지 판정이 하강 확인 없이 걸려, 이륙 지점보다 높은 곳 위 호버 또는 VIO 높이 오차 때 공중(1.03 m)에서 150 ms 만에 끌 수 있었음 → 이 비행 단계에서 VIO 하강을 본 뒤에만 판정 (`openloop_vio_desc_vz` 0.3), 회귀 시험 `openloop_vio_touch_needs_descent` 추가. 숫자 불일치 2개(README 0.97 m, 코드 주석 2.98/0.12 s = Python 값) 고침.
- 남은 위험 (안 고침):
  - IMU 단계 착륙 중에 OPENLOOP 으로 넘어오면 높이가 낡았고 이미 내려가는 중 → t_fast 전에 닿아 0.95 추력으로 땅에서 돌 수 있음 (예전 코드도 같은 낡은 높이 사용).
  - 시험의 기체 모델과 예측 모델이 같은 식(추력 ∝ 명령², 공기저항 없음) → 실제 착지는 더 늦을 수 있음.
  - 띠 계산 비용(ESP32 제어 루프 안 2×1200 반복)은 추정 수백 µs–1 ms, 측정 안 함 (최악 루프 초과 1회).
  - heartbeat src 1: 메인 루프가 멈추면 ESP32 가 알아채는 데 약 450 ms (src 0 은 약 250 ms, 추정). IMU_STALE 기준이 수신 스레드 시각.
  - 종료 때 reader 스레드가 g_mav 를 볼 수 있는 경쟁 (출력 쓰기 실패 뒤 프로세스 종료 시에만, 낮음).

## 확인 안 된 것"""),
])
patch('esp32fc/README.md',[
("**510개 검사 모두 통과** (esp32fix 2026-10-26; 이전 389개)","**516개 검사 모두 통과** (esp32fix 2026-10-26; 이전 389개)"),
("Result: **510 checks, 0 failed**","Result: **516 checks, 0 failed**"),
("폰 pose가 아직 오면(IMU 프레임만 끊긴 경우) 높이 < 0.10 m","폰 pose가 아직 오면(IMU 프레임만 끊긴 경우) VIO로 하강을 본 뒤 높이 < 0.10 m"),
("If poses + heartbeat still arrive: cut when height","If poses + heartbeat still arrive and a VIO descent (vz < −0.3 m/s) was seen: cut when height"),
])
print('ok')
