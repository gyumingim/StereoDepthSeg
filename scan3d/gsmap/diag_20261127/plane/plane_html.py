#!/usr/bin/env python3
import json, time
from pathlib import Path
OUT = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/diag_plane_20261127"
D = Path("/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo")
J = json.loads((OUT / "plane_diag.json").read_text())
NM = dict(a="a 짝 1개 FFS 깊이", b="b 키프레임 융합 FFS (학습 입력)", c="c 모든 b 점을 월드에 합침", d="d 2DGS 렌더 깊이",
          e="e 최종 메시 (e3c, 생성면 제외)", f="d* 2DGS, FFS 있는 화소만", g="e* 메시, FFS 있는 화소만")
KO = dict(cabinet_front_kf774="캐비닛 앞면 (kf 774)", wall_kf538="벽 (kf 538)", floor_kf3205="바닥 (kf 3205, 실제 FFS 부분)")
f2 = lambda v: "–" if v is None else f"{v:.2f}"


def table(name, R):
    rows = []
    for st in "abcdefg":
        s = R["stages"][st]["summary"]; b = s["bands_combined"]
        if st == "c":
            rows.append(f"<tr class=c><td>{NM[st]}</td><td>–</td><td>–</td><td>–</td><td>–</td><td><b>{f2(s['combined_rms_global_cm'])}</b> / {f2(s['combined_mad_global_cm'])}</td>"
                        f"<td>{f2(b['long_gt20_cm'])}</td><td>{f2(b['mid_5_20_cm'])}</td><td>{f2(b['short_lt5_cm'])}</td></tr>")
            continue
        rows.append(f"<tr><td>{NM[st]}</td><td>{s['n_maps']}</td><td>{f2(s['within_rms_own_cm_median'])}</td>"
                    f"<td>{f2(s['offset_sd_cm'])} ({f2(s['offset_robust_sd_cm'])})</td><td>{f2(s['tilt_median_deg_wide'])}</td>"
                    f"<td><b>{f2(s['combined_rms_global_cm'])}</b> / {f2(s['combined_mad_global_cm'])}</td>"
                    f"<td>{f2(b['long_gt20_cm'])}</td><td>{f2(b['mid_5_20_cm'])}</td><td>{f2(b['short_lt5_cm'])}</td></tr>")
    a = R["stages"]["a"]["summary"]
    return (f"<h3>{KO[name]} — 넓이 {R['hull_area_m2']:.2f} m², 거리 약 {R['stages']['b']['summary']['median_depth_m']:.1f} m</h3>"
            "<table><tr><th>단계</th><th>장 수</th><th>장 안 잔차 RMS<br>(자기 평면, cm)</th><th>장 사이 평면 위치<br>표준편차 cm (강건)</th>"
            "<th>장 기울기<br>중앙값 °</th><th>합친 RMS / MAD<br>(전역 평면, cm)</th><th>&gt;20 cm</th><th>5–20 cm</th><th>&lt;5 cm</th></tr>"
            + "".join(rows) + "</table>"
            f"<p class=s>짝(a) 장 사이 차이 중 <b>같은 키프레임의 짝끼리</b> 차이 평균 {a['within_kf_partner_offset_sd_cm_mean']:.2f} cm, "
            f"키프레임 평균끼리 차이 {a['across_kf_mean_offset_sd_cm']:.2f} cm (짝 2개 이상 키프레임 {a['n_kf_multi_pair']}개). "
            f"기울기는 짧은 쪽 폭 15 cm 이상인 장만. d·e 는 한 개의 3D 모델이라 '장 사이' 값은 원래 0 에 가깝다(비교 의미 없음).</p>"
            f"<img src='plane_diag/plot_{name}.png' alt='{name}'>")


R = J["rois"]
html = f"""<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>평면 울퉁불퉁 진단</title><style>
:root{{--bg:#fff;--fg:#1a202c;--mut:#5a6577;--line:#d9dee6;--hi:#fff8e1}}
@media (prefers-color-scheme:dark){{:root{{--bg:#14171c;--fg:#e6e9ef;--mut:#9aa5b4;--line:#2c333d;--hi:#2a2614}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,'Malgun Gothic',sans-serif;max-width:1100px;margin:0 auto;padding:16px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0;display:block;overflow-x:auto}}td,th{{border:1px solid var(--line);padding:4px 6px;text-align:right}}
td:first-child,th:first-child{{text-align:left}}tr.c td{{background:var(--hi)}}img{{max-width:100%;border:1px solid var(--line);margin:6px 0 18px}}
.s{{color:var(--mut);font-size:13px}}.box{{border:2px solid #2b6cb0;border-radius:8px;padding:10px 14px;margin:10px 0}}code{{font-size:12px}}
</style></head><body>
<h1>평면이 어디서 울퉁불퉁해지나 — 단계별 평면 잔차 (ffsonly2, 121801)</h1>
<p class=s>만든 시각 {J['made']} · 재학습 없음 · 녹화 1개, 평면 ROI 3개 (n 작음) · 원자료: <code>runs/gsmap_20261125_121801_v2/diag_plane_20261127/plane_diag.json</code></p>
<div class=box><b>결론</b><ol>
<li><b>FFS 깊이의 화소 잡음은 작다. 문제는 장 사이 어긋남(치우침)이다.</b> 짝 1개 깊이를 자기 평면에 맞추면 잔차 RMS 는
벽 {R['wall_kf538']['stages']['a']['summary']['within_rms_own_cm_median']:.2f}, 바닥 {R['floor_kf3205']['stages']['a']['summary']['within_rms_own_cm_median']:.2f},
캐비닛 {R['cabinet_front_kf774']['stages']['a']['summary']['within_rms_own_cm_median']:.2f} cm 인데,
장마다 평면 위치가 벽 {R['wall_kf538']['stages']['a']['summary']['offset_sd_cm']:.2f}, 바닥 {R['floor_kf3205']['stages']['a']['summary']['offset_sd_cm']:.2f},
캐비닛 {R['cabinet_front_kf774']['stages']['a']['summary']['offset_sd_cm']:.2f} cm (표준편차) 씩 다르다 → 4–10 배. 자문서 판정 규칙(A' &gt; A)으로 <b>포즈/치우침이 주범</b>.</li>
<li>그 어긋남의 대부분은 <b>같은 키프레임의 짝끼리도</b> 난다 (벽 0.92 vs 1.19, 바닥 0.60 vs 0.95, 캐비닛 2.73 vs 2.87 cm). 키프레임 월드 포즈보다
<b>짝의 상대 자세(짝 포즈·롤링셔터·기선)</b> 쪽이 크다는 뜻이다 (추정 — 어느 것인지는 이 측정으로 못 가림). → 자문 P1(짝별 시차 보정)·P2(RS) 방향과 맞다.</li>
<li>키프레임 융합(b)은 장 사이 차이를 35–50 % 만 줄인다 (벽 1.49→0.96, 바닥 0.96→0.79, 캐비닛 3.61→1.87 cm). √N 만큼 줄지 않는다 = 독립 잡음이 아님.</li>
<li><b>학습·메시 단계</b>: 바닥은 오히려 평평해진다 (합친 FFS 0.75 → 메시 0.44 cm). 캐비닛도 줄지만 (3.18 → FFS 화소 메시 2.09 cm) 여전히 2 cm.
벽은 늘어난다 (0.85 → 1.69 cm) — 그런데 대부분이 ROI 한쪽 끝 약 20 cm 띠에서 메시가 FFS 보다 최대 6 cm 떠 있는 것이고 (그림 가운데), MAD 로는 0.70 → 0.78 cm 로 거의 같다. 그 띠의 원인은 미확인.</li>
<li>파장: 합친 FFS 지도(c)에서 가장 큰 성분은 &gt;20 cm (캐비닛 1.65, 바닥 0.58 cm), &lt;5 cm 는 0.18–0.57 cm → 물결은 치우침형. 벽 메시의 &gt;20 cm 1.2 cm 는 위 끝 띠 때문.</li>
</ol><b>순서 제안</b>: P2(a) RS 켠 BA → P1 짝별 시차 보정 → 같은 진단 다시. 벽 끝 띠는 사진으로 따로 확인.</div>

<h2>단계별 표</h2>
<p class=s>모든 단계가 같은 화소(전역 ROI 평면에 닿는 광선, ROI 윤곽 안)를 쓴다. 전역 평면 = b 점 전부에 강건 평면 맞춤. 평면에서 ±10 cm 밖 점은 버림(가림·다른 물체).
'장 안' = 각 장을 자기 평면에 맞춘 잔차 (화소 잡음 + 곡률). '장 사이' = 각 장의 평면이 전역 평면에서 떨어진 거리의 표준편차. 파장 칸 = 1 cm 격자 평균 잔차를 20 cm / 5 cm 상자로 나눈 RMS.</p>
{''.join(table(n, R[n]) for n in ('cabinet_front_kf774', 'wall_kf538', 'floor_kf3205'))}

<h2>코드 확인</h2><ul>
<li><b>ba_partners2 BA 는 rs_skew 0 (전역 셔터 가정) — 확인.</b> <code>ba_partners2/ba/ba.json:19 "rs_skew_ms": 0.0</code>; 설정은 옛 BA 그대로
(<code>gsm_ba_partners.py:117</code> "original settings = OLD_BA/ba.json args", <code>gsm_ffs_relax.py:157-158</code> 이 그 함수를 호출); kf_ba 기본값 0 (<code>StereoDepthSeg-fixes/scan3d/scanA/kf_ba.py:347</code>).</li>
<li><b>E 보정(특징점으로 짝 자세 다시 풀기) 없음.</b> 짝 깊이는 BA 자세를 그대로 씀: <code>gsm_ba_partners.py:286-287</code>, <code>gsm_ffs_4m.py:115</code>, <code>gsm_ffsonly_ba.py:120</code> → <code>motion_stereo.pair_depth</code>.
pair_depth 의 시차 보정(c·d+e)은 두 렌즈 깊이 Z2 를 줄 때만 (<code>motion_stereo.py:387-391</code>) — 위 호출은 안 줌. E 풀이는 실시간 클래스에만 있음 (<code>motion_stereo.py:243, 516-517</code>).</li>
<li><b>기선 규칙은 있음, 목표 시차 규칙은 없음.</b> 기본 짝 B 3–25 cm, 광축-기선 각 ≥ 40° (대체 30°), 회전 ≤ 25°, 에피폴 화면 밖
(<code>gsm_ba_partners.py:38-43</code>, <code>gsm_ffs_pairs2.py:11-13, 162</code>); 넓은 짝 B 15–35 cm (<code>gsm_ffsonly_ba.py</code> A_ wb_min/wb_max, <code>gsm_ffs_4m.py:56</code>);
relax 짝 B 10–50 cm, 각 ≥ 10° (<code>gsm_ffs_relax.py:8, 97, 374</code>). 화소별로 σZ/Z ≤ 3 % 컷 (<code>gsm_ffs_pairs3.py:19, 147</code>, 융합 때 모든 짝에 <code>gsm_ffs_fix3.py:87</code>)
= 화소 시차 f·B·sinθ/Z ≥ 약 33 px 와 같은 뜻(계산). 즉 '짧은 기선 버리기'는 있지만 '시차 60–80 px 를 목표로 고르기'(자문 D2)는 없음.</li>
<li>참고(섞인 조건): 짝 없는 166 kf 외의 키프레임 깊이(ffsonly_ba/s4m)는 <b>ba_partners</b> 자세로 만들었고 학습은 <b>ba_partners2</b> 자세로 놓는다
(<code>gsm_ffs_relax.py:23</code> combine 설명). 두 BA 사이 키프레임 이동이 장 사이 어긋남에 얼마나 들어갔는지는 안 쟀다.</li></ul>

<h2>못 한 것 / 가정</h2><ul class=s>
<li>RS 상관(장별 행 방향 기울기 vs 자이로 각속도 차)과 P1 미리보기(BA 점 시차 vs FFS 시차)는 하지 않았다.</li>
<li>캐비닛 ROI 는 작다(0.10 m², 융합 장 7개, 장 폭 약 9 cm) → 캐비닛 기울기 값은 믿기 어렵다. 캐비닛 짝 점의 37 % 가 ±10 cm 밖(두 겹/가장자리).</li>
<li>벽 끝 6 cm 띠: 실제 물체(문틀·몰딩 등)인지 2DGS 오류인지 사진으로 확인 안 함. 띠를 빼면 벽 메시 잔차는 약 ±1 cm (그림에서 읽은 값, 추정).</li>
<li>2DGS 깊이 = median depth, 불투명도 &gt; 0.5 (gsm_mesh2d 와 같은 규칙). 메시 = e3c.ply 첫 충돌(생성된 바닥/천장 면 제외). 서비스 glb 는 같은 e3c.ply 에 텍스처만 입힘.</li>
<li>잣대 재현성(같은 짝 두 번 돌리기)은 확인 안 함. ROI 3개·녹화 1개.</li></ul>
<p class=s>스크립트: <code>{J['script']}</code> (render: GPU 9 초, measure: CPU 83 초). 입력: {', '.join(f'<code>{v}</code>' for k, v in J['inputs'].items() if isinstance(v, str))}.</p>
</body></html>"""
(D / "plane_diag.html").write_text(html, encoding="utf-8")
print("written", time.strftime("%F %T"))
