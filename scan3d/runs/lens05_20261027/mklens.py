import json, hashlib, time
from pathlib import Path
src = Path.home()/"scanner/StereoDepthSeg-calcap/scan3d/runs/calcap_calib_20261026/lens_candidate_20261026.json"
c = json.load(open(src))
out = Path.home()/"scanner/StereoDepthSeg-lens05/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json"
j = dict(K=c["K"], dist=c["dist"], focus_dpt=0.5, model="k1k2p", fit="k12_notan",
         fit_note="k1 k2 free, p1 p2 k3 fixed 0 (OpenVINS radtan compatible). Values copied unchanged from the calcap board fit.",
         rms_px=c["rms_px"], n_views=c["n_views"], n_poses=c["n_captures"], size=[1920, 1080],
         status="app lens (phone_ov_main) since 2026-10-27 - provisional",
         status_note=("User 2026-10-27 08:35: all app lenses at 0.5 dpt. Chosen from board calibration only (never from VIO error): "
                      "lenscal_20261027 README verdict 'cal20261024d or calcap k12 usable provisionally'; coordinator rule = "
                      "independent 0.5 dpt set A + OpenCV k1k2 -> calcap k12_notan (set A). Not confirmed: fx -0.96..+0.36 %, "
                      "edge distortion ~2-3 px at r0.8 (laptop-screen board); replace after the printed A1 ChArUco sets."),
         uncertainty="fx +-4.7 px (0.36 %, capture bootstrap) + systematic -0.96..+0.36 % (board flatness); pp +-1.7 px",
         source=str(src), source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
         source_run="scan3d/runs/calcap_calib_20261026 (branch calcap-20261025), set ~/scanner/StereoDepthSeg/scan3d/recordings/calcap_20261025_214000 (132 stills, 33 poses)",
         cross_check="lenscal_20261027: set B (independent recordings) scores this lens 0.155 px vs its own fit 0.145 px",
         stream=c["stream"], board="board_laptop_5x3.json", fit_made=c["made"],
         made=time.strftime("%Y-%m-%d %H:%M:%S"))
assert not out.exists()
out.write_text(json.dumps(j, indent=1, ensure_ascii=False) + "\n")
print(out, hashlib.sha256(out.read_bytes()).hexdigest())
