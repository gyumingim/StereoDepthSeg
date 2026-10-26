#!/usr/bin/env python3
"""ovxfeat: phone benchmark log (phone_bench.sh output) -> table (item, model, backend, ms p50 / avg, delegate coverage,
battery / charging / temperatures).  python3 parse_phone.py LOG [LOG..] -> prints + results.json next to the first log"""
import json
import re
import sys

rows = []
charging = None
for fn in sys.argv[1:]:
    for l in open(fn, encoding="utf8", errors="replace"):
        m = re.search(r"AC powered: (\w+)\s+USB powered: (\w+)", l)
        if m and "top:" in l:
            charging = (m.group(1) == "true") or (m.group(2) == "true")
        m = re.match(r"(\S+ \S+) item (\d+) bat (\d+)% (\S+) before\[([^\]]*)\] after\[([^\]]*)\] (.*)", l)
        if not m:
            continue
        t, i, bat, wake, before, after, rest = m.groups()
        r = dict(time=t, item=int(i), bat=int(bat), charging=charging, wake=wake, before=before.strip(), after=after.strip())
        if rest.startswith("TFL"):
            hd = rest.split("::")[0].split()
            r["tool"] = "tflite"
            r["model"] = hd[1]
            args = " ".join(hd[2:])
            r["backend"] = ("gpu" if "use_gpu=true" in args else "nnapi" + ("_fp16" if "allow_fp16" in args else "")
                            + ("_nocpu" if "disable_nnapi_cpu" in args else "") if "use_nnapi=true" in args else "cpu")
            r["args"] = args
            c = re.findall(r"count=(\d+) first=\d+ curr=\d+ min=(\d+) max=(\d+) avg=([\d.]+) std=\d+ p5=\d+ median=(\d+) p95=(\d+)", rest)
            if c:
                n, mn, mx, avg, med, p95 = c[-1]
                r.update(n=int(n), ms_p50=int(med) / 1000, ms_p95=int(p95) / 1000, ms_avg=float(avg) / 1000, ms_min=int(mn) / 1000)
            cov = re.search(r"Replacing (\d+) out of (\d+) node\(s\) with delegate \((\w+)\)", rest)
            if cov:
                r["delegate"] = f"{cov.group(3)} {cov.group(1)}/{cov.group(2)}"
            if "will not be executed by the delegate" in rest:
                r["delegate_note"] = "NNAPI rejected -> XNNPACK CPU"
            if "ERROR" in rest or "Failed" in rest:
                r["error"] = rest.split("::")[1][:200]
        elif rest.startswith("{"):
            try:
                d = json.loads(rest.strip())
            except Exception:
                d = {}
            r.update(tool="ort", model=d.get("model"), backend=d.get("backend"), ms_p50=d.get("ms_p50"), ms_p95=d.get("ms_p95"),
                     ms_min=d.get("ms_min"), n=d.get("iters"), threads=d.get("threads"))
        else:
            r.update(tool="?", error=rest[:200])
        rows.append(r)
json.dump(rows, open(sys.argv[1].rsplit(".", 1)[0] + "_results.json", "w"), indent=1)
for r in rows:
    print(f"{r['item']:3d} {r.get('tool','?'):6s} {str(r.get('model')):32s} {str(r.get('backend')):14s} p50 {r.get('ms_p50')} p95 {r.get('ms_p95')} "
          f"n {r.get('n')} {r.get('delegate','')} {r.get('delegate_note','')} chg {r['charging']} bat {r['bat']} {r['before']} {r.get('error','')[:80]}")
