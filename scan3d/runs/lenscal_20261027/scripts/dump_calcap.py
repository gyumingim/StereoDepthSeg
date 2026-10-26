import sys, os, json, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/tools/calcap"))
import calcap_calib as CC, slots as SL
S="/home/system/scanner/tmp_calcap/set_calcap_20261025_214000"
boards=SL.load_boards(os.path.join(CC.S3D,"config","charuco"))
gK,gd=SL.load_lens(CC.DEFAULT)
views,caps=CC.load_views([S],boards,gK,gd,print)
print(len(views), sum(len(v["ids"]) for v in views), set(v["board"] for v in views))
out=[]
for v in views:
    out.append(dict(file=v["file"],cap=v["cap"],slot=v["slot"],board=v["board"],ids=v["ids"].tolist(),obj=v["obj"].tolist(),img=v["img"].reshape(-1,2).tolist()))
json.dump(dict(set=S,board_sq=[b.sq for b in boards],boards=[b.name for b in boards],views=out),open(os.path.expanduser("~/scanner/tmp_lenscal/views_calcap.json"),"w"))
