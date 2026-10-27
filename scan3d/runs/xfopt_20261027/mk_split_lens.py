#!/usr/bin/env python3
"""xfopt 2026-10-27: make split.json (pre-registered data split) and the 2.22 dpt lens derived from the lens05 app lens."""
import hashlib
import json
import time
from pathlib import Path

S3 = Path(__file__).resolve().parents[2]
RD = Path(__file__).resolve().parent
SRC = "config/lens/std_1920x1080_f0.50_calcap20261026.json"
DST = "config/lens/hs_1920x1080_f2.22_from_calcap05_20261027.json"
K_FOCUS = 0.0080   # image scale per dioptre (docs/focus_scale_20261022.md, measured)

L = json.load(open(S3 / SRC))
s = 1 + K_FOCUS * (2.22 - 0.5)
L["K"][0][0] *= s
L["K"][1][1] *= s
sha = hashlib.sha256(open(S3 / SRC, "rb").read()).hexdigest()
L.update(focus_dpt=2.22, source=SRC,
         status="xfopt-20261027 PC eval only (HS 2.22 dpt recordings): lens05 app lens moved to 2.22 dpt by the measured focus scale",
         status_note=("derived from %s (sha256 %s): fx, fy x %.5f = 1 + 0.0080 x (2.22 - 0.5) (docs/focus_scale_20261022.md); "
                      "principal point and distortion unchanged = ASSUMPTION (not measured). Made %s." % (SRC, sha[:16], s, time.strftime("%F %T"))))
json.dump(L, open(S3 / DST, "w"), indent=1)

RECS = ["hs_20261022_133820", "hs_20261022_134550", "hs_20261022_134204", "hs_20261021_214550", "hs_20261021_213341",
        "hs_20261021_215348", "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747",
        "hs_20261020_223815", "hs_20261020_224323", "hs_20261019_131326", "hs_20261019_154320"]   # phone60_run RAW+OLD - DROP
srt = sorted(RECS, key=lambda r: hashlib.sha256(("xfopt-20261027:" + r).encode()).hexdigest())
json.dump(dict(rule='HS 14 = phone60_run RAW+OLD minus DROP; sorted by sha256("xfopt-20261027:"+rec), first 5 = dev, rest = test; recording units',
               made=time.strftime("%F %T"), dev=sorted(srt[:5]), test=sorted(srt[5:]),
               test_std=["hs_20261024_165506_std60"], test_loop=["hs_20261024_142148_std60", "hs_20261024_142610_std60"],
               ref_only=["hs_20261024_173432_std60", "hs_20261024_151646_std60"], drop="hs_20261021_213955",
               lens=dict(hs=DST, std=SRC)),
          open(RD / "split.json", "w"), indent=1)
print("K", L["K"], "dev", sorted(srt[:5]))
