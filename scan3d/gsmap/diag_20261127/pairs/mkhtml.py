#!/usr/bin/env python3
"""pair_diag.html generator: reads OUT/pairs_table.json, analysis.json, rules.json + conclusion html snippet (argv[2]) -> argv[1]."""
import json, sys, html, time
from pathlib import Path
import numpy as np
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"; OUT = V2 / "diag_pairs_20261127"
A = json.loads((OUT / "analysis.json").read_text()); R = json.loads((OUT / "rules.json").read_text()); T = json.loads((OUT / "pairs_table.json").read_text())
concl = Path(sys.argv[2]).read_text(encoding="utf-8")
ROIS = ["cabinet_front_kf774", "wall_kf538", "floor_kf3205"]; RN = {"cabinet_front_kf774": "캐비닛 앞면", "wall_kf538": "벽 538", "floor_kf3205": "바닥 3205"}
CLS = ["base", "base_fb", "low", "wide_ba", "wide_vio", "relax"]
CN = {"base": "base (기본, 각≥40°)", "base_fb": "base_fb (대체, 30–40°)", "low": "low (저각, 부분정류)", "wide_ba": "wide (긴 기선, BA 포즈)",
      "wide_vio": "wide (긴 기선, VIO 포즈)", "relax": "relax (완화, 부분정류, BA2)"}
COL = {"base": "#2563eb", "base_fb": "#64748b", "low": "#dc2626", "wide_ba": "#059669", "wide_vio": "#84cc16", "relax": "#f59e0b"}
f = lambda x, d=2: "–" if x is None or (isinstance(x, float) and not np.isfinite(x)) else (f"{x:.{d}f}" if isinstance(x, (float, int)) and not isinstance(x, bool) else html.escape(str(x)))


def scatter(roi, xk, xl, yk="off", yl="면 어긋남 (cm, + = 카메라 쪽)", W=380, H=260):
    P = [p for p in A["pairs"] if p["roi"] == roi and p.get(xk) is not None and p.get(yk) is not None]
    xs = np.array([p[xk] for p in P], float); ys = np.array([p[yk] for p in P], float)
    x0, x1 = 0, max(90, xs.max()) if xk == "perp_deg" else (xs.min(), xs.max())
    if xk != "perp_deg":
        x0, x1 = xs.min(), xs.max()
    ym = max(2, np.percentile(np.abs(ys), 98)); y0, y1 = -ym, ym
    L, Rr, Tp, B = 42, 10, 18, 34
    sx = lambda x: L + (x - x0) / (x1 - x0 + 1e-9) * (W - L - Rr); sy = lambda y: Tp + (y1 - np.clip(y, y0, y1)) / (y1 - y0) * (H - Tp - B)
    s = [f'<svg viewBox="0 0 {W} {H}" class="plot" role="img" aria-label="{RN[roi]} {xl}">',
         f'<text x="{L}" y="12" class="pt">{RN[roi]}</text>', f'<line x1="{L}" x2="{W - Rr}" y1="{sy(0):.1f}" y2="{sy(0):.1f}" class="ax0"/>']
    for yt in np.linspace(y0, y1, 5):
        s.append(f'<text x="{L - 4}" y="{sy(yt) + 3:.1f}" class="tk" text-anchor="end">{yt:.1f}</text>')
    for xt in np.linspace(x0, x1, 5):
        s.append(f'<text x="{sx(xt):.1f}" y="{H - B + 13}" class="tk" text-anchor="middle">{xt:.0f}</text>')
    s.append(f'<text x="{(L + W) / 2}" y="{H - 4}" class="tk" text-anchor="middle">{xl}</text>')
    for p, x, y in zip(P, xs, ys):
        s.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.2" fill="{COL[p["cls"]]}" fill-opacity="0.75"><title>kf {p["kf"]}–{p["partner"]} {p["cls"]} off {y:.2f}</title></circle>')
    s.append("</svg>")
    return "".join(s)


def bars():
    G = A["groups"]["pooled"]; items = [(c, G[c]["off_px"]) for c in CLS if c in G and G[c]["off_px"]["n"] >= 2]
    W, H = 520, 200; L = 190; mx = max(v["robust_sd"] for _, v in items) * 1.15
    s = [f'<svg viewBox="0 0 {W} {H}" class="plot" role="img" aria-label="종류별 시차 오차">']
    for i, (c, v) in enumerate(items):
        y = 10 + i * 30; w = v["robust_sd"] / mx * (W - L - 60)
        s.append(f'<text x="{L - 6}" y="{y + 14}" class="tk" text-anchor="end">{CN[c]}</text><rect x="{L}" y="{y}" width="{w:.1f}" height="20" rx="3" fill="{COL[c]}"/>'
                 f'<text x="{L + w + 6:.1f}" y="{y + 14}" class="tk">{v["robust_sd"]:.2f} px (n {v["n"]})</text>')
    s.append("</svg>")
    return "".join(s)


def grp_table():
    h = ['<table><thead><tr><th>ROI</th><th>짝 종류</th><th>n</th><th>평균 어긋남 cm</th><th>강건 SD cm</th><th>같은 kf 짝끼리 차이 |dev| 중앙 cm</th>'
         '<th>시차로 환산 강건 SD px</th><th>기선 B cm</th><th>광축-기선 각 °</th><th>시차 px</th><th>깊이 m</th><th>장 안 잡음 cm</th></tr></thead><tbody>']
    for r in ROIS:
        G = A["groups"][r]
        for c in CLS:
            if c not in G:
                continue
            g = G[c]
            h.append(f'<tr><td>{RN[r]}</td><td><span class="dot" style="background:{COL[c]}"></span>{CN[c]}</td><td>{g["off"]["n"]}</td><td>{f(g["off"]["mean"])}</td>'
                     f'<td>{f(g["off"]["robust_sd"])}</td><td>{f(g["dev"].get("absmed"))}</td><td>{f(g["off_px"]["robust_sd"])}</td><td>{f(g["B_cm"]["median"], 1)}</td>'
                     f'<td>{f(g["perp"]["median"], 0)}</td><td>{f(g["disp_px"]["median"], 0)}</td><td>{f(g["Z"]["median"])}</td><td>{f(g["rms_own"]["median"])}</td></tr>')
    h.append("</tbody></table>")
    return "".join(h)


def rules_table():
    C = R["conditions"]; base = C["all"]; cov = R["coverage"]["all_kf"]; covr = R["coverage"]["roi_kf"]
    h = ['<table><thead><tr><th rowspan="2">규칙</th>' + "".join(f'<th colspan="3">{RN[r]}</th>' for r in ROIS) +
         '<th rowspan="2">FFS 화소 유지 (전체 kf)</th><th rowspan="2">깊이 전부 잃는 kf</th></tr><tr>' + "<th>a SD</th><th>b SD</th><th>c RMS</th>" * 3 + "</tr></thead><tbody>"]
    for nm, v in C.items():
        if nm.startswith("_"):
            continue
        row = f"<tr><td><code>{html.escape(nm)}</code></td>"
        for r in ROIS:
            for st, k in (("a", "offset_sd_cm"), ("b", "offset_sd_cm"), ("c", "combined_rms_global_cm")):
                x = v[r][st][k]; x0 = base[r][st][k]
                if x is None:
                    row += '<td class="bad">없음</td>'; continue
                cls = "" if nm == "all" else ("good" if x < x0 - 0.05 else "bad" if x > x0 + 0.05 else "")
                row += f'<td class="{cls}">{x:.2f}</td>'
        kept = cov["kept_vs_b2"][nm]; lost = cov["kf_losing_all"][nm]
        row += f'<td class="{"bad" if kept < 0.97 else ""}">{100 * kept:.1f} %</td><td>{lost}</td></tr>'
        h.append(row)
    h.append("</tbody></table>")
    return "".join(h)


def pair_table():
    keys = [("roi", "ROI"), ("kf", "kf"), ("partner", "짝"), ("cls", "종류"), ("rect", "정류"), ("partner_pose_src", "짝 포즈"), ("B_geom_cm", "B cm"),
            ("perp_deg", "각°"), ("rot_deg", "회전°"), ("dt_frames", "Δ프레임"), ("epi_in_kf", "에피폴 화면안"), ("epi_out_kf_px", "에피폴 밖 거리 px"),
            ("disp_px_med", "시차 px"), ("theta_med_deg", "ROI 화소-기선 각°"), ("sector_major", "섹터"), ("near_sector_boundary_frac", "섹터경계 비율"),
            ("rect_f", "정류 f"), ("scale", "FFS 배율"), ("ffs_W", "FFS W"), ("ffs_H", "FFS H"), ("radius_med", "화면 반경(0 중심,1 가장자리)"),
            ("Z_med_m", "깊이 m"), ("off", "어긋남 cm"), ("dev", "같은 kf 짝 대비 cm"), ("off_px", "시차환산 px"), ("rho_med", "깊이비-1"),
            ("tilt_deg", "기울기°"), ("rms_own_cm", "장 안 잡음 cm"), ("ba2_B_ratio", "BA2/사용 기선비"), ("p1_a_px", "P1 a px")]
    h = ['<div class="scroll"><table class="small"><thead><tr>' + "".join(f"<th>{l}</th>" for _, l in keys) + "</tr></thead><tbody>"]
    for p in A["pairs"]:
        h.append("<tr>" + "".join(f"<td>{RN.get(p[k], p[k]) if k == 'roi' else f(p.get(k), 4 if k in ('rho_med', 'ba2_B_ratio') else 2)}</td>" for k, _ in keys) + "</tr>")
    h.append("</tbody></table></div>")
    return "".join(h)


reg = A["regress"]; disp = A["dispersion_by_B"]
page = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>짝별 깊이 어긋남 진단</title><style>
:root{{--bg:#fafaf9;--fg:#1c1917;--mut:#57534e;--card:#fff;--line:#e7e5e4;--good:#dcfce7;--bad:#fee2e2;--acc:#2563eb}}
@media (prefers-color-scheme:dark){{:root:not([data-theme=light]){{--bg:#1c1917;--fg:#f5f5f4;--mut:#a8a29e;--card:#292524;--line:#44403c;--good:#14532d;--bad:#7f1d1d;--acc:#60a5fa}}}}
:root[data-theme=dark]{{--bg:#1c1917;--fg:#f5f5f4;--mut:#a8a29e;--card:#292524;--line:#44403c;--good:#14532d;--bad:#7f1d1d;--acc:#60a5fa}}
body{{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,'Malgun Gothic',sans-serif;margin:0;padding:16px}}main{{max-width:1100px;margin:auto}}
h1{{font-size:22px}}h2{{font-size:18px;margin-top:28px;border-bottom:1px solid var(--line)}}.card{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 16px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0}}th,td{{border:1px solid var(--line);padding:3px 6px;text-align:right}}th{{background:var(--card)}}td:first-child,td:nth-child(2){{text-align:left}}
.small td,.small th{{font-size:11px;white-space:nowrap}}.scroll{{overflow-x:auto;max-height:520px}}.good{{background:var(--good)}}.bad{{background:var(--bad)}}
.dot{{display:inline-block;width:10px;height:10px;border-radius:5px;margin-right:4px}}.plots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:10px}}
.plot{{width:100%;background:var(--card);border:1px solid var(--line);border-radius:8px}}.pt{{font-size:12px;fill:var(--fg)}}.tk{{font-size:10px;fill:var(--mut)}}.ax0{{stroke:var(--mut);stroke-dasharray:3 3}}
.mut{{color:var(--mut);font-size:13px}}code{{font-size:12px}}</style></head><body><main>
<h1>평평한 면이 울퉁불퉁한 원인 — 어떤 스테레오 짝이 어긋나나</h1>
<p class="mut">121801 v2 · 2026-11-27 · 재학습 없음, CPU · 잣대 = plane_diag / plane_fix 와 같은 ROI 3 개·같은 평면 맞춤·같은 "단순 재융합". 만든 시각 {time.strftime('%F %T')}</p>
<div class="card">{concl}</div>
<h2>1. 짝 종류별 어긋남</h2>
<p>어긋남 = 짝 하나로 만든 깊이 지도의 면이 ROI 합의 평면에서 법선 방향으로 떨어진 거리 (+ = 카메라 쪽, 깊이가 짧음). "시차 환산 px" = 그 거리를 이 짝의 시차 오차로 바꾼 값 (Z²/(f·B·sinθ) 로 나눔).</p>
{grp_table()}
<div class="plots">{bars()}</div>
<h2>2. 어긋남 vs 광축-기선 각</h2><p class="mut">점 색 = 짝 종류 (파랑 base, 회색 base_fb, 빨강 low, 초록 wide-BA, 연두 wide-VIO, 주황 relax). 마우스를 올리면 kf–짝.</p>
<div class="plots">{''.join(scatter(r, 'perp_deg', '광축-기선 각 (°)') for r in ROIS)}</div>
<h2>3. 깊이 비례(스케일) 인가, 시차 고정(px) 인가</h2>
<table><thead><tr><th>기선 구간 (ROI별 중앙값 뺀 뒤)</th><th>n</th><th>시차 환산 강건 SD (px)</th><th>깊이비 강건 SD (%)</th></tr></thead><tbody>
{''.join(f"<tr><td>{k}</td><td>{v['n']}</td><td>{v['robust_sd']:.3f}</td><td>{disp['depth_pct'][k]['robust_sd']:.3f}</td></tr>" for k, v in disp['disp_px'].items())}
</tbody></table>
<p>강건 회귀 log|어긋남| = ROI + a·log B + b·log Z + c·log sinθ: a = {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lB'][0]:.2f} ± {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lB'][1]:.2f},
b = {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lZ'][0]:.2f} ± {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lZ'][1]:.2f}, c = {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lS'][0]:.2f} ± {reg['log_absoff_dm~roi+logB+logZ+logsin']['coef']['lS'][1]:.2f}
(n {reg['log_absoff_dm~roi+logB+logZ+logsin']['n']}). 고정 px 오차면 a ≈ −1, 고정 % 오차면 a ≈ 0.</p>
<h2>4. 규칙 시험 (단순 재융합, 같은 잣대)</h2>
<p>a = 짝 하나 지도들의 어긋남 SD, b = kf 별 융합 지도의 어긋남 SD (장 사이 불일치, 핵심), c = 모든 b 점을 모은 RMS. 기준 = <code>all</code> (모든 짝, 단순 재융합). 초록 = 0.05 cm 넘게 좋아짐, 빨강 = 나빠짐. 화소 유지 = 전체 494 kf 의 F==1 화소 중 그 규칙으로도 깊이가 남는 비율 (all 대비).</p>
{rules_table()}
<p class="mut">BA2/BAP 기선비 (BAP 포즈로 만든 짝 1742 개): 중앙 {R['global_ratio']['median']:.4f}, 5–95 % {R['global_ratio']['p5_25_75_95'][0]:.4f}–{R['global_ratio']['p5_25_75_95'][3]:.4f}.</p>
<h2>5. 짝별 표 (ROI 3 개에 닿는 모든 짝 {len(A['pairs'])} 개)</h2>{pair_table()}
<h2>입력·검산</h2><ul class="mut">
<li>스크립트·결과: <code>{OUT}</code> (scripts/pair_diag.py, analyse.py, mkhtml.py · pairs_table.json, analysis.json, rules.json)</li>
<li>'before' 재현: plane_fix 의 before 와 소수 3 자리까지 같음. plane_diag.json 과는 6 개 값이 최대 0.006 cm 다름 (plane_fix 때도 같은 차이; 표본 추출 난수 순서 차이로 보임 — 추정).</li>
<li>짝 포즈: base/low = ba_partners poses_all, wide = ba_partners cams + ffsonly_ba/wide_poses (VIO 고정 294/621), relax = ba_partners2. 측정(월드 배치) = ba_partners2 cams.</li>
<li>n 이 작다: ROI 3 개, 녹화 1 개, 캐비닛 짝 18 개·kf 7 개.</li></ul>
</main></body></html>"""
Path(sys.argv[1]).write_text(page, encoding="utf-8"); print("wrote", sys.argv[1], len(page))
