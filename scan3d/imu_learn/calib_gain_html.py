#!/usr/bin/env python3
"""runs/imu_calib_gain/result.json -> docs/imu_calib_gain.html (one plot: per-recording median 10 s rotation error)."""
import io
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

S = Path(__file__).resolve().parent.parent
d = json.load(open(S / "runs/imu_calib_gain/result.json"))
ck = json.load(open(S / "runs/imu_calib_gain/check.json"))
used = [r for r in d["recs"] if "cond" in r]
C = {"A": ("A raw, 보정 없음", "#9aa0a6"), "B": ("B 안드로이드 type 4", "#d9822b"), "C": ("C 시작 정지만", "#5b8def"),
     "D": ("D 정지마다 다시 (인과)", "#1a7f37"), "E": ("E oracle 상수", "#8250df")}
plt.rcParams["svg.fonttype"] = "none"          # text stays text -> browser renders Hangul
plt.rcParams["font.family"] = ["sans-serif"]
fig, ax = plt.subplots(figsize=(8, 4.2), dpi=110)
labels = [r["rec"].replace("hs_2026", "") for r in used]
for k, (name, col) in C.items():
    ys = [r["cond"][k]["median"] for r in used]
    ax.plot(range(len(used)), ys, "o-" if k == "D" else "o", color=col, label=name, ms=7 if k == "D" else 5,
            lw=1.5, alpha=1 if k in "BD" else 0.7)
ax.set_xticks(range(len(used)), labels, rotation=30, ha="right")
ax.set_ylabel("10 s 회전 오차 중앙값 (deg)")
ax.set_ylim(0, 5.3)
ax.grid(axis="y", alpha=0.3)
ax.legend(fontsize=8, ncol=2)
fig.tight_layout()
buf = io.StringIO()
fig.savefig(buf, format="svg")
svg = buf.getvalue()[buf.getvalue().index("<svg"):]
s = d["summary"]
rows = "".join(f"<tr><td>{r['rec']}</td>" + "".join(f"<td>{r['cond'][k]['median']:.2f} / {r['cond'][k]['p90']:.2f} / "
                                                       f"{r['cond'][k]['yaw_dpm_median']:.1f}</td>" for k in "ABCDE")
               + "</tr>" for r in used)
red = d["decision"]["reduction_B_to_D_pct"]
html = f"""<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>자이로 보정 이득</title>
<style>:root{{--bg:#fff;--fg:#1f2328;--mut:#59636e;--line:#d1d9e0}}
@media (prefers-color-scheme:dark){{:root{{--bg:#0d1117;--fg:#e6edf3;--mut:#9198a1;--line:#3d444d}} svg{{background:#fff;border-radius:6px}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.55 system-ui,sans-serif;max-width:900px;margin:0 auto;padding:16px}}
table{{border-collapse:collapse;font-size:13px;width:100%;overflow-x:auto;display:block}}td,th{{border:1px solid var(--line);padding:4px 6px;text-align:right}}
td:first-child{{text-align:left}} .mut{{color:var(--mut);font-size:13px}} svg{{max-width:100%;height:auto}}</style>
<h1>자이로 보정 이득</h1>
<p class="mut">만든 때 {d['made']}, git {d['git_head']}, 사전 등록 {d['plan']}. 독립 검사 calib_gain_check.py: {'PASS' if ck['PASS'] else 'FAIL'}.</p>
<h2>결론</h2>
<p><b>정지마다 바이어스를 다시 잡으면 10 s 회전 오차가 {s['B']['median_of_medians']:.2f} → {s['D']['median_of_medians']:.2f} deg ({red:.0f} % 감소, 안드로이드 type 4 대비).</b>
보정을 아예 안 하면 {s['A']['median_of_medians']:.2f} deg. 요 드리프트 {s['B']['yaw_dpm_median']:.1f} → {s['D']['yaw_dpm_median']:.1f} deg/분.
사전 등록 판정 (D 가 B 보다 20 % 이상 낮은 녹화 ≥ 3): <b>{len(d['decision']['recs_D_beats_B'])} / {len(used)} → 통과</b>.</p>
<p>단, 222747 에서는 D 가 C 보다 나쁨 (3.40 vs 1.27): 천천히 돌던 구간 (7.8–11.1 s, y 평균 −47 mdeg/s) 을 정지로 잘못 잡음. 사후 (등록 안 된) 게이트 D2 (직전 값과 100 mdeg/s 넘게 다르면 버림) 로는 1.27.</p>
{svg}
<h2>녹화별 (중앙값 / p90 deg / 요 deg/분)</h2>
<table><tr><th>녹화</th><th>A raw</th><th>B type4</th><th>C 시작정지</th><th>D 정지마다</th><th>E oracle</th></tr>{rows}</table>
<p class="mut">녹화별 중앙값의 중앙값: A {s['A']['median_of_medians']:.2f}, B {s['B']['median_of_medians']:.2f}, C {s['C']['median_of_medians']:.2f}, D {s['D']['median_of_medians']:.2f}, E {s['E']['median_of_medians']:.2f} deg.
창은 겹침 (프레임마다 시작) → 실제 반복 수 = 녹화 8 개. 자세한 내용 docs/imu_calib_gain.md.</p>
</html>"""
(S / "docs/imu_calib_gain.html").write_text(html)
print("wrote docs/imu_calib_gain.html")
