#!/usr/bin/env python3
"""livepose -> easy Korean HTML (conclusion first). Reads summary.json + notes.json (hand-written text) of this folder.
  python make_html.py OUT.html
"""
import html, json, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
S = json.load(open(HERE / "summary.json"))
N = json.load(open(HERE / "notes.json"))
E = html.escape


def f2(v, d=2):
    return "—" if v is None else f"{v:.{d}f}"


def tbl(head, rows, cls=""):
    h = "".join(f"<th>{E(x)}</th>" for x in head)
    b = "".join("<tr>" + "".join(f"<td>{x}</td>" for x in r) + "</tr>" for r in rows)
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'


SETN = {"S": "라이브와 같음 (std60, GT)", "L": "라이브와 같음 (손 루프, GT 없음)", "H": "같지 않음 (HS 2.22 dpt, GT)"}
base_rows = []
for r in S["baseline"]:
    st = r["stat"][0]
    ates = " / ".join(f2(a) if a is not None else f'<span class="bad">{E(s or "?")}</span>' for a, s in zip(r["ate"], r["stat"]))
    if r["set"] == "L":
        main = f'{f2(r.get("loop_err_cm"))} cm / {f2(r.get("path_m"), 1)} m ({f2(r.get("loop_pct"), 1)} %)' if st == "ok" else f'<span class="bad">{E(st)}</span>'
        base_rows.append([E(r["set"]), E(r["rec"].replace("hs_", "")), ates if False else "—", main, f2(r.get("loop_err_map_cm")),
                          "—", "—", f2(r.get("init_s"), 1), f2(r.get("dt_final_ms"))])
    else:
        base_rows.append([E(r["set"]), E(r["rec"].replace("hs_", "")), ates,
                          f'{f2(r.get("path_m"), 1)} m · {f2(r.get("ate_pct"))} %' if st == "ok" else "—",
                          f2(r.get("ate_map_cm")), f2(r.get("drift_cm_per_m")), f2(r.get("scale_sim3"), 3), f2(r.get("init_s"), 1),
                          f2(r.get("dt_final_ms"))])

pair_rows = []
for k, p in S["pairs"].items():
    c, s = k.split("/")
    nm = N["cond_names"].get(c, c)
    v = p["verdict"]
    cls = "good" if "좋아" in v or (v == "영향 있음" and (p["med"] or 0) < 0) else "bad" if "나빠" in v or (v == "영향 있음" and (p["med"] or 0) > 0) else "muted"
    pair_rows.append([E(c), E(nm), E(s), str(p["n"]), f2(p["med"], 3), f'{p["better"]}/{p["worse"]}',
                      f'{p["b_only_fail"]}/{p["c_only_fail"]}', f'<span class="{cls}">{E(v)}</span>'])

budget_rows = [[E(b["src"]), E(b["how"]), b["size"], E(b["note"])] for b in N["budget"]]
next_rows = [[str(i + 1), E(x["what"]), E(x["gain"]), E(x["user"])] for i, x in enumerate(N["next"])]
rec_rows = [[E(x["kind"]), E(x["recs"]), E(x["eq"]), E(x["gt"])] for x in N["inventory"]]

css = """
:root{--bg:#f7f6f2;--card:#fff;--ink:#1d1d1b;--sub:#5d5b55;--line:#dedbd2;--accent:#2f6fdb;--good:#1f8a4c;--warn:#b7791f;--bad:#c2410c;--muted:#8a877f;--soft:#eef2fb;--softw:#fbf3e4}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#16171a;--card:#1f2125;--ink:#ecebe7;--sub:#b0ada6;--line:#34363c;--accent:#6c9cf0;--good:#4cc283;--warn:#e0a84a;--bad:#f07b4a;--muted:#8d8a84;--soft:#1f2a3d;--softw:#3a2f1a}}
:root[data-theme="dark"]{--bg:#16171a;--card:#1f2125;--ink:#ecebe7;--sub:#b0ada6;--line:#34363c;--accent:#6c9cf0;--good:#4cc283;--warn:#e0a84a;--bad:#f07b4a;--muted:#8d8a84;--soft:#1f2a3d;--softw:#3a2f1a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:-apple-system,"Apple SD Gothic Neo","Malgun Gothic","Noto Sans KR",system-ui,sans-serif;line-height:1.6}
.wrap{max-width:960px;margin:0 auto;padding:20px 16px 80px}h1{font-size:1.5rem;margin:.2em 0}
section{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin:16px 0}
h2{font-size:1.15rem;margin:0 0 8px}.one{background:var(--soft);border-radius:10px;padding:8px 12px;margin:6px 0}
.warn{background:var(--softw);border-radius:10px;padding:8px 12px;margin:6px 0}
.tw{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:.86rem}th,td{border-bottom:1px solid var(--line);padding:5px 7px;text-align:left;vertical-align:top}
th{color:var(--sub);font-weight:600}td{font-variant-numeric:tabular-nums}.good{color:var(--good);font-weight:600}.bad{color:var(--bad);font-weight:600}.muted{color:var(--muted)}
.sub{color:var(--sub);font-size:.88rem}.tag{display:inline-block;font-size:.75rem;padding:1px 8px;border-radius:999px;background:var(--softw);color:var(--warn);font-weight:600}
code{font-size:.85em}
"""
body = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>라이브 위치 정확도</title><style>{css}</style></head><body><div class="wrap">
<h1>라이브 폰 위치 정확도 — 기준선과 오차 예산 (10-30)</h1>
<p class="sub">PC 재생만 (폰·새 녹화 없음). 앱 0.6.38c 라이브 설정 (rec60, c4f60) 을 코드에서 읽어 그대로 재생. <span class="tag">모든 숫자 미검증 표시는 본문 참조</span>
 · 만든 시각 {time.strftime('%Y-%m-%d %H:%M')} · 브랜치 <code>livepose-20261030</code> · <code>scan3d/runs/livepose_20261030/</code></p>
<section><h2>결론 먼저</h2>{''.join(f'<p class="one">{x}</p>' for x in N['conclusion'])}</section>
<section><h2>1. 라이브 설정 (코드에서 읽은 값)</h2>{''.join(f'<p>{x}</p>' for x in N['live_cfg'])}</section>
<section><h2>2. 쓸 수 있는 녹화</h2>{tbl(['종류', '녹화', '라이브와 같나', 'GT'], rec_rows)}</section>
<section><h2>3. 기준선 (라이브 설정 그대로, PC 재생)</h2>{''.join(f'<p>{x}</p>' for x in N['baseline_text'])}
{tbl(['묶음', '녹화', 'VIO ATE cm (시작 0/1/2 s)', '경로 · ATE %', '지도 ATE cm', '표류 cm/m', 'Sim3 축척', '초기화 s', '시간차 ms (끝)'], base_rows)}
<p class="sub">입력 경로: cam5.mp4 → OpenCV H.264 디코드 → BGR → 회색 1920×1080 → 브리지가 폰 remap (calcap 0.5 dpt, sha 483168c9) 으로 960×540. IMU = 보정 안 된 raw (폰과 같음).
시각 = 노출 가운데, 녹화별 이동 없음 (폰과 같음). 다른 점: 코덱 (폰 = Y 평면), drop-late 0, 루프 동기 (지도 = 낙관적).</p></section>
<section><h2>4. 오차 원천별 영향 (한 가지씩 바꾼 재생, 짝 비교)</h2>{''.join(f'<p>{x}</p>' for x in N['pairs_text'])}
{tbl(['조건', '바꾼 것', '묶음', '짝 수', 'Δ ATE 중앙 cm', '좋아짐/나빠짐', '실패 (B만/조건만)', '판정'], pair_rows)}</section>
<section><h2>5. 오차 예산 (추정)</h2>{''.join(f'<p>{x}</p>' for x in N['budget_text'])}{tbl(['원천', '어떻게 쟀나', '크기', '메모'], budget_rows)}</section>
<section><h2>6. 다음 실험 (기대 이득 순, 추정)</h2>{tbl(['#', '실험', '기대 이득 (추정)', '사용자가 할 일'], next_rows)}</section>
<section><h2>7. 라이브 PC 화면 (:8798) 으로 볼 수 있는 것 / 없는 것</h2>{''.join(f'<p>{x}</p>' for x in N['viewer'])}</section>
<section><h2>8. 확인 안 된 것 · 가정</h2>{''.join(f'<p class="warn">{x}</p>' for x in N['caveats'])}</section>
<section><h2>파일</h2>{''.join(f'<p class="sub">{x}</p>' for x in N['files'])}</section>
</div></body></html>"""
Path(sys.argv[1]).write_text(body, encoding="utf-8")
print("wrote", sys.argv[1], len(body))
