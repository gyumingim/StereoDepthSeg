import json
a = json.load(open("/home/system/scanner/StereoDepthSeg-dpvo/scan3d/runs/dpvo_20261026/ov_ref/score_ov_ref.json"))
b = json.load(open("/home/system/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026/gyro/score_gyro.json"))
for st in ("hs", "std"):
    B = {r["key"]: r for r in b[st]["cells"]}
    for c in ("GL30", "GL60"):
        n = same = 0; diff = []
        for r in a[st]["cells"]:
            if c in r and c in B.get(r["key"], {}):
                n += 1
                x, y = r[c], B[r["key"]][c]
                if x["poses_hash"] == y["poses_hash"]: same += 1
                else: diff.append((r["key"], x["ate_w"], y["ate_w"]))
        print(st, c, "compared", n, "poses_hash identical", same, "diff", diff[:5])
print({k: v for k, v in a["loops"].items()})
