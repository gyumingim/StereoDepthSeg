"""Still-threshold sensitivity table (judge round 2 item 17): w < 0.05 (main) vs w < 0.1 rad/s (views_w10, step 2),
plus w < 0.02 (existing _w02 per-recording fits). Same pairs and models as the main repeatability table.
usage: sens_w10.py  -> results/sens_w10.md, results/sens_w10.json"""
import json, time
from pathlib import Path
from common import D, FOCUS

R = lambda n: json.load(open(D / "results" / n)) if (D / "results" / n).exists() else None
PAIRS = [("2.22", "162218", "163644"), ("2.22", "162218", "171455"), ("2.22", "163644", "171455"),
         ("3.33", "163008", "173834"), ("0.37", "163228", "174512"), ("1.00", "162754", "172351")]
TAGS = [("", "w<0.05 (main)"), ("_w10", "w<0.1"), ("_w02", "w<0.02")]


def d(a, b):
    return [round((b[0] - a[0]) / a[0] * 100, 3), round(b[2] - a[2], 2), round(b[3] - a[3], 2)]


out, L = dict(made=time.strftime("%F %T"), rows=[]), []
L.append("| 정지 문턱 | 초점 | A 대 B | 녹화별 rational dfx % / dcx / dcy | 녹화별 k5 dfx % / dcx / dcy | 공동(k1 녹화별) dfx % / dcx / dcy | 사진 수 A / B (녹화별) |")
L.append("|---|---|---|---|---|---|---|")
for tag, lab in TAGS:
    jr = R(f"joint_rec_k5_k1g{tag}.json")
    for f, a, b in PAIRS:
        ca, cb = R(f"cal_{a}{tag}.json"), R(f"cal_{b}{tag}.json")
        row = dict(tag=tag or "main", focus=f, a=a, b=b)
        cells = []
        for m in ("rational", "k1k2p_k3"):
            if ca and cb:
                fa, fb = ca["fits"][m], cb["fits"][m]
                v = d([fa["fx"], fa["fy"], fa["cx"], fa["cy"]], [fb["fx"], fb["fy"], fb["cx"], fb["cy"]])
                row[m] = v
                cells.append(" / ".join(f"{x:+.2f}" for x in v))
            else:
                cells.append("–")
        if jr and a in jr["K4"] and b in jr["K4"]:
            v = d(jr["K4"][a], jr["K4"][b]); row["joint_rec_k5_k1g"] = v
            cells.append(" / ".join(f"{x:+.2f}" for x in v))
        else:
            cells.append("–")
        nv = f"{ca['fits']['rational']['n_views']} / {cb['fits']['rational']['n_views']}" if ca and cb else "–"
        L.append(f"| {lab} | {f} | {a} 대 {b} | {cells[0]} | {cells[1]} | {cells[2]} | {nv} |")
        out["rows"].append(row)
L.append("")
L.append("| 정지 문턱 | 공동 맞춤 초점별 (k1 초점별) fx: 0.37 / 1.00 / 2.22 / 3.33 | 2.22 대비 % | 사진 수 | rms |")
L.append("|---|---|---|---|---|")
for tag, lab in TAGS[:2]:
    jf = R(f"joint_focus_k5_k1g{tag}.json")
    if not jf:
        continue
    fx = {g: jf["K4"][g][0] for g in ("0.37", "1.00", "2.22", "3.33")}
    rel = " / ".join(f"{(fx[g] / fx['2.22'] - 1) * 100:+.2f}" for g in ("0.37", "1.00", "3.33"))
    L.append(f"| {lab} | {' / '.join(f'{fx[g]:.1f}' for g in fx)} | {rel} | {jf['n_views']} | {jf['rms_px']:.3f} |")
    out[f"joint_focus{tag or '_main'}"] = dict(fx=fx, n_views=jf["n_views"], rms=jf["rms_px"], cxcy={g: jf["K4"][g][2:4] for g in fx})
L.append("")
L.append("| 녹화 | 정지 프레임 (w<0.05 / w<0.1) | 쓸 수 있는 사진 (0.05 / 0.1) |")
L.append("|---|---|---|")
for f in ("0.37", "1.00", "2.22", "3.33"):
    for r in FOCUS[f]:
        c0, c1 = R(f"cal_{r}.json"), R(f"cal_{r}_w10.json")
        if not (c0 and c1):
            continue
        k0, k1 = c0["counts"][r], c1["counts"][r]
        L.append(f"| {r} | {k0['n_still']} / {k1['n_still']} | {k0['n_after_thin']} / {k1['n_after_thin']} |")
txt = "\n".join(L)
open(D / "results" / "sens_w10.md", "w").write(txt + "\n")
json.dump(out, open(D / "results" / "sens_w10.json", "w"), indent=1)
print(txt)
