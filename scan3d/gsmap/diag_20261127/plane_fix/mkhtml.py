import json, time
from pathlib import Path
O = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/diag_plane_fix_20261127"
D1 = json.loads((O / "plane_fix.json").read_text()); D2 = json.loads((O / "plane_fix2.json").read_text())
C = dict(D1["conditions"]); C.update(D2["conditions"])
OUTP = Path("/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo/plane_fix_preview.html")
ROIS = [("cabinet_front_kf774", "캐비닛 앞면"), ("wall_kf538", "벽 538"), ("floor_kf3205", "바닥 3205")]
CONDS = [("b2_before", "보정 전 (기준)"), ("p1", "P1 원안 (점 ≥ 50, a+b·행+c·열)"), ("p1_n200_abc", "P1 점 ≥ 200, a+b+c"), ("p1_n200_a", "P1 점 ≥ 200, 상수 a 만"),
         ("p1_n50_a", "P1 점 ≥ 50, 상수 a 만"), ("rs+", "RS 보정 + 부호 (전량)"), ("rs-", "RS 보정 − 부호 (전량)"), ("rs_k033", "RS + 부호 × 0.33"), ("rs+p1", "RS+ 뒤 P1"), ("rs-p1", "RS− 뒤 P1")]
f = lambda x: "—" if x is None else f"{x:.2f}"


def table(roi):
    h = "<table><tr><th>조건</th><th>a 장 안 RMS</th><th>a 장 사이 SD</th><th>a 강건 SD</th><th>a 같은 kf 짝끼리 SD</th><th>b 장 사이 SD</th><th>b 강건 SD</th><th>c 합친 RMS</th></tr>"
    for k, lab in CONDS:
        a = C[k][roi]["a"]["summary"]; b = C[k][roi]["b"]["summary"]
        cls = " class=c" if k == "b2_before" else ""
        h += (f"<tr{cls}><td>{lab}</td><td>{f(a['within_rms_own_cm_median'])}</td><td>{f(a['offset_sd_cm'])}</td><td>{f(a['offset_robust_sd_cm'])}</td>"
              f"<td>{f(a.get('within_kf_partner_offset_sd_cm_mean'))}</td><td>{f(b['offset_sd_cm'])}</td><td>{f(b['offset_robust_sd_cm'])}</td><td>{f(b['combined_rms_global_cm'])}</td></tr>")
    b0 = C["before"][roi]
    h += (f"<tr><td class=s>참고: plane_diag 원래 b (실제 학습 깊이)</td><td>{f(b0['a']['summary']['within_rms_own_cm_median'])}</td><td>{f(b0['a']['summary']['offset_sd_cm'])}</td>"
          f"<td>{f(b0['a']['summary']['offset_robust_sd_cm'])}</td><td>{f(b0['a']['summary'].get('within_kf_partner_offset_sd_cm_mean'))}</td><td>{f(b0['b']['summary']['offset_sd_cm'])}</td>"
          f"<td>{f(b0['b']['summary']['offset_robust_sd_cm'])}</td><td>{f(b0['b']['summary']['combined_rms_global_cm'])}</td></tr></table>")
    return h


cov = D1["coverage"]; rs = D1["rs_corr"]; mix = D1["mix"]; fl = D2["ba_flatness"]; gy = D1["gyro"]
mx = lambda roi, st, k: C["before"][roi][st]["summary"].get(k)
html = f"""<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>평면 보정 미리보기</title><style>
:root{{--bg:#fff;--fg:#1a202c;--mut:#5a6577;--line:#d9dee6;--hi:#fff8e1}}
@media (prefers-color-scheme:dark){{:root{{--bg:#14171c;--fg:#e6e9ef;--mut:#9aa5b4;--line:#2c333d;--hi:#2a2614}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,'Malgun Gothic',sans-serif;max-width:1100px;margin:0 auto;padding:16px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0;display:block;overflow-x:auto}}td,th{{border:1px solid var(--line);padding:4px 6px;text-align:right}}
td:first-child,th:first-child{{text-align:left}}tr.c td{{background:var(--hi)}}.s{{color:var(--mut);font-size:13px}}.box{{border:2px solid #2b6cb0;border-radius:8px;padding:10px 14px;margin:10px 0}}code{{font-size:12px}}
</style></head><body>
<h1>평면 어긋남 보정 미리보기 — P1(BA 점 시차 앵커) · RS(롤링셔터)</h1>
<p class=s>만든 시각 {time.strftime('%F %T')} · 재학습 없음 · 녹화 121801 1개, 평면 ROI 3개 (n 작음) · 잣대 = <a href=plane_diag.html>plane_diag</a> 의 단계 a·b·c 그대로(같은 스크립트 복사본) ·
원자료 <code>runs/gsmap_20261125_121801_v2/diag_plane_fix_20261127/plane_fix.json, plane_fix2.json</code></p>
<div class=box><b>결론</b><ol>
<li><b>P1 은 장 사이 어긋남을 거의 못 줄인다 → 파이프라인 구현 + 재학습은 보류(no-go).</b> 짝 1장 기준(a) 장 사이 SD: 캐비닛 3.57→3.57, 벽 1.48→1.41, 바닥 0.96→1.05 cm (점 ≥ 200 안전판).
목표였던 0.5 cm 이하는 어디서도 안 나왔다.</li>
<li>이유: <b>캐비닛·벽 ROI 안에는 BA 점이 거의 없다</b> (ROI 안 BA 3D 점 캐비닛 {fl['cabinet_front_kf774']['n_ba_pts']}개, 벽 {fl['wall_kf538']['n_ba_pts']}개, 바닥 {fl['floor_kf3205']['n_ba_pts']}개).
무늬 없는 면이라 BA 도 점을 못 만든다 → 보정이 ROI 바깥 점에서 외삽된다. 점이 적은 짝(60–90개)에서 외삽이 틀려 벽은 오히려 나빠졌다(원안 b SD 0.96→1.98 cm).</li>
<li>바닥(점 많음)만 조금 좋아진다: 융합 b SD 0.79→0.63, 합친 c 0.75→0.63 cm (점 ≥ 200). 하지만 강건 SD 는 0.30→0.33 그대로 = 꼬리 몇 장이 줄어든 것.
BA 점 자체가 바닥 평면에서 MAD {f(fl['floor_kf3205']['own_plane_mad_cm'])} cm, RMS {f(fl['floor_kf3205']['own_plane_rms_cm'])} cm 퍼져 있어, 앵커로서 0.5 cm 보다 정확해지기 어렵다(추정).</li>
<li><b>RS 는 실제로 있다(상관 0.40) 그러나 작고, 보정해도 평면이 평평해지지 않는다 → RS 구현도 지금은 no-go.</b> 자이로 예측 행 기울기와 맞춘 b 의 상관 Pearson {f(rs['all']['pearson'])} (n {rs['all']['n']}),
빠른 회전(|Δω| &gt; 0.3 rad/s)만 {f(rs['fast']['pearson'])}. 관측 크기는 예측의 {f(rs['all']['slope'])} 배. 세 가지 부호·크기 모두 ROI 표에서 일관된 개선 없음.</li>
<li><b>섞인 조건(BA 포즈 두 벌)의 몫은 작다.</b> ba_partners→ba_partners2 사이 짝 상대 회전 변화 중앙값 {mix['mix_rot_deg_p50_95'][0]:.3f}° (p95 {mix['mix_rot_deg_p50_95'][1]:.3f}°) → 예측 시차 이동 중앙값 {mix['mix_pred_a_absmed']:.2f} px, 맞춘 a 와 상관 {f(mix['a_vs_pred']['pearson'])}.
다만 벽에서는 relax 짝을 가진 키프레임 4장이 평균 {f(mx('wall_kf538','b','offset_mean_cm_target'))} cm (나머지 16장 {f(mx('wall_kf538','b','offset_mean_cm_nontarget'))} cm) 떨어져 있어, 이 4장을 빼면 벽 b SD 0.96→{f(mx('wall_kf538','b','offset_sd_cm_nontarget'))} cm. 포즈 차이가 아니라 relax 짝(다른 짝·부분 정렬) 쪽 원인으로 본다(추정).</li>
<li>남은 큰 어긋남은 <b>같은 키프레임의 짝끼리</b> 여전히 크다(캐비닛 2.7 cm, 벽 0.9 cm). BA 점 앵커와 RS 로 설명되지 않으므로, 다음은 짝 자체(FFS 시차 크기/기선, 부분 정렬 구역 경계, 짝 선택) 쪽을 봐야 한다(추정).</li>
</ol></div>

<h2>1. 전·후 표 (cm, plane_diag 와 같은 단위·잣대)</h2>
<p class=s>a = 짝 1개 깊이, b = 키프레임 융합, c = b 를 월드에 모은 것. 단계 b 는 보정된 짝을 다시 합쳐야 하므로 모든 조건을 <b>같은 간이 재융합</b>(화소별 중앙값 문턱 + 역분산 평균, 융합 화소 F==1 위)으로 만들었다.
"보정 전 (기준)" 줄이 그 간이 재융합의 보정 전 값이다(노란 줄). 맨 아래 회색 줄은 실제 학습 깊이(plane_diag b) 값으로, 간이 재융합과 거의 같다(검산).
전역 평면은 각 조건의 b 로 plane_diag 와 같은 방법으로 다시 맞췄다. "plane_diag 원래" 줄은 plane_diag.json 수치를 소수 셋째 자리까지 그대로 재현했다(잣대 검산).</p>
"""
for roi, lab in ROIS:
    html += f"<h3>{lab} <span class=s>({roi})</span></h3>" + table(roi)
html += f"""
<h2>2. P1 — 짝별 시차 앵커</h2>
<ul>
<li>방법: ba_partners2 BA 상태(ba_state.npz: 포즈 R1·t1, 3D 점 X1, 관측 uv)에서 그 키프레임이 관측한 점의 카메라 깊이(지도 축척 {D1['ba_info']['scale_map_over_ba']:.5f} 배) → 짝마다 잔차 r = f·B·sinθ·(1/Z_FFS − 1/Z_BA) [px] 에
a + b·(행−cy)/100 + c·(열−cx)/100 을 강건 맞춤(중앙값 ±2 px 문턱, Huber 0.3 px). 행·열 폭이 150 px 미만이면 그 기울기는 뺐다.</li>
<li><b>순환 방지</b>: 그 키프레임이 보는 모든 ROI 영역(20 px 넓힘) 안의 BA 점은 맞춤에 쓰지 않았다. 그 점들은 검증에만 썼다.</li>
<li>덮음: 전체 짝 {cov['n_pairs']}개(키프레임 {cov['n_kf']}장) 중 점 ≥ 50 = {cov['n_fit_ge50']}, ≥ 100 = {cov['n_fit_ge100']}, ≥ 200 = {cov['n_fit_ge200']} 짝. ROI 짝 {cov['roi_pairs']}개 중 {cov['roi_fitted']}개 맞춤.
짝당 관측 점 중앙값 {cov['n_obs_med']:.0f}, FFS 와 겹쳐 맞춤에 쓴 점 중앙값 {cov['n_fit_med']:.0f}.</li>
<li>맞춘 a: p5/50/95 = {cov['a_px_p5_50_95'][0]:.2f} / {cov['a_px_p5_50_95'][1]:.2f} / {cov['a_px_p5_50_95'][2]:.2f} px (MAD {cov['a_px_mad']:.2f} px) — 자문이 말한 "±0.3 px 넘게 흔들림"은 맞다.
행 기울기 b: p5/50/95 = {cov['b_px100_p5_50_95'][0]:.2f} / {cov['b_px100_p5_50_95'][1]:.2f} / {cov['b_px100_p5_50_95'][2]:.2f} px / 100 행.</li>
<li>빼 둔 ROI 안 BA 점으로 검증(짝 {cov['hold']['n_pairs']}개, 대부분 바닥): |잔차| 중앙값 {cov['hold']['absmed_before_med']:.2f} → {cov['hold']['absmed_after_med']:.2f} px,
짝별 중앙값의 SD {cov['hold']['signed_med_sd_before']:.2f} → {cov['hold']['signed_med_sd_after']:.2f} px. 즉 FFS 는 BA 점에 더 잘 맞게 되지만, 그게 평면 일치로 이어지지 않는다.</li>
<li>ROI 안 BA 점의 평면성(지도 좌표, 전역 평면 대비): 캐비닛 점 {fl['cabinet_front_kf774']['n_ba_pts']}개, 벽 {fl['wall_kf538']['n_ba_pts']}개 (RMS {f(fl['wall_kf538']['rms_cm'])} cm),
바닥 {fl['floor_kf3205']['n_ba_pts']}개: 전역 평면 대비 RMS {f(fl['floor_kf3205']['rms_cm'])}, MAD {f(fl['floor_kf3205']['mad_cm'])}, 중앙값 {f(fl['floor_kf3205']['median_cm'])} cm.</li>
<li>"P1 직접"(융합 지도에 바로 앵커, 역깊이 단위)도 시험: 벽 b SD 0.96→{f(C['p1_direct']['wall_kf538']['b']['summary']['offset_sd_cm'])}, 바닥 0.79→{f(C['p1_direct']['floor_kf3205']['b']['summary']['offset_sd_cm'])}, 캐비닛 1.87→{f(C['p1_direct']['cabinet_front_kf774']['b']['summary']['offset_sd_cm'])} cm. 더 나쁨.</li>
</ul>
<h2>3. RS — 롤링셔터</h2>
<ul>
<li>읽기 시간: <code>frames_cam5.csv</code> 의 rolling_shutter_skew_ns = {gy['skew_ns']/1e6:.3f} ms (폰 HAL 보고값, 매 프레임 같음). 전체 센서 기준인지 1080p 뷰 기준인지는 미확인(rolling_shutter_survey.md: 뷰 기준이면 약 4.9 ms).</li>
<li>자이로: imu.csv type 4, {gy['rate_hz']:.0f} Hz, 카메라←IMU 회전 = config/phone_ov_main/base60 T_imu_cam, 시각 이동 {gy['tshift']*1e3:.3f} ms.
검산: BA2 프레임 사이 회전 {gy['check_vs_ba2']['n']}쌍에서 자이로 적분 vs BA 회전 오차 중앙값 {gy['check_vs_ba2']['err_over_rot_med']*100:.1f} %, 방향 cos {gy['check_vs_ba2']['cos_med']:.4f} → 회전 축·시각은 맞다.</li>
<li>예측: 행 기울기 = f·(t_r/540)·(Δω_y·e_x − Δω_x·e_y), Δω = 짝 − 키프레임 각속도(카메라 좌표), e = 기선의 영상 방향. 예측 |값| 중앙값 {rs['all']['x_absmed']:.2f} px/100행.</li>
<li>상관(맞춘 b 와): 전체 Pearson {f(rs['all']['pearson'])} / Spearman {f(rs['all']['spearman'])} (n {rs['all']['n']}), 빠른 회전 {f(rs['fast']['pearson'])} / {f(rs['fast']['spearman'])} (n {rs['fast']['n']}),
ROI 짝만 {f(rs['roi']['pearson'])} (n {rs['roi']['n']}). 기울기 {f(rs['all']['slope'])}. 롤(z축) 항은 {f(rs['roll_term']['pearson'])} (없음). 상수 a 와는 {f(rs['a_vs_rs']['pearson'])} (없음).
자문 판정 문턱 0.5 에는 못 미침. 크기가 예측의 1/3 인 이유(실제 읽기 시간이 짧음 / 잡음에 의한 희석)는 이 측정으로 못 가림.</li>
<li>보정 미리보기(ROI 짝, 시차에서 행 기울기 빼기): + 부호·− 부호·+0.33 배 모두 표 1 에서 일관된 개선 없음.</li>
<li>"지도별 평면 기울기(행 방향) vs 자이로" 상관은 따로 계산하지 않았다(짝별 b 상관으로 대신).</li>
</ul>
<h2>4. 섞인 조건</h2>
<ul>
<li>정정: gsm_ffs_relax 의 "target" 166 장(넓은 짝 없던 키프레임)은 기존 base/low 짝(ba_partners 포즈) + 새 relax 짝(ba_partners2 포즈)으로 깊이를 만들었고, 나머지 328 장은 전부 ba_partners 포즈 깊이다. 학습 포즈는 모두 ba_partners2.</li>
<li>캐비닛 ROI 키프레임은 전부 target, 바닥은 전부 non-target 이라 그 두 ROI 안에서는 비교가 안 된다. 벽만 target 4 / non-target 16 장.</li>
<li>벽 b: target 평균 {f(mx('wall_kf538','b','offset_mean_cm_target'))} cm (SD {f(mx('wall_kf538','b','offset_sd_cm_target'))}), non-target 평균 {f(mx('wall_kf538','b','offset_mean_cm_nontarget'))} cm (SD {f(mx('wall_kf538','b','offset_sd_cm_nontarget'))}).
벽 a: target 짝 평균 {f(mx('wall_kf538','a','offset_mean_cm_target'))}, non-target {f(mx('wall_kf538','a','offset_mean_cm_nontarget'))} cm.</li>
<li>포즈 차이 자체: ba_partners 포즈 짝의 상대 회전이 ba_partners2 와 다른 정도 중앙값 {mix['mix_rot_deg_p50_95'][0]:.3f}°, p95 {mix['mix_rot_deg_p50_95'][1]:.3f}°; 예측 시차 이동 중앙값 {mix['mix_pred_a_absmed']:.3f} px; 맞춘 a 와 상관 {f(mix['a_vs_pred']['pearson'])}.
맞춘 a 의 MAD: ba_partners 포즈 짝 {mix['a_mad_bap']:.2f} px (n {mix['n_bap']}), ba_partners2 포즈 relax 짝 {mix['a_mad_ba2']:.2f} px (n {mix['n_ba2']}) — 짝 종류가 달라 섞인 비교.</li>
</ul>
<h2>못 한 것 · 가정</h2>
<ul class=s>
<li>파이프라인의 실제 융합(gsm_ffs_fix3 fuse: 키프레임 사이 일치 검사 포함)을 보정된 짝으로 다시 돌리지 않았다. 간이 재융합으로 대신했고, 보정 전 간이 재융합은 실제 학습 깊이와 거의 같다(표의 노란 줄 vs 회색 줄).</li>
<li>BA 점은 RS 를 무시한(rs_skew 0) BA 결과다. RS 를 켠 BA(P2a) 위에서는 P1 결과가 다를 수 있다(미시험).</li>
<li>영상 위→아래가 읽기 순서라는 가정은 미확인(부호는 양쪽 다 시험). 읽기 6.54 ms 가 1080p 뷰 기준인지 미확인.</li>
<li>2DGS·메시 단계(d·e)는 다시 재지 않았다(재학습 없음). ROI 3개·녹화 1개.</li>
</ul>
</body></html>"""
OUTP.write_text(html, encoding="utf-8")
print("wrote", OUTP)
