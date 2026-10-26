"""calcap_calib_20261026: board flatness sensitivity scan (laptop screen modelled as a cylindrical bow along x or y,
z = b (1 - ((u - uc)/uc)^2)). Board model check only; not used to pick a lens. -> bowscan.json"""
import json, os, sys, time
import numpy as np, cv2
S3D = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
sys.path.insert(0, os.path.join(S3D, "tools", "calcap"))
import calcap_calib as CC, slots as SL  # noqa: E402
st, out = sys.argv[1], sys.argv[2]
boards = SL.load_boards(os.path.join(S3D, "config", "charuco")); B5 = boards[0]
gK, gd = SL.load_lens(CC.DEFAULT)
views, caps = CC.load_views([st], boards, gK, gd, lambda s: None)
r0 = CC.fit(views, "k12_notan"); K0 = r0[1]
rows = []
for axis in ("x", "y"):
    for b in (-1.0, -0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0):
        vs = []
        for v in views:
            o = v["obj"].astype(np.float64).copy(); c = B5.centre
            u = (o[:, 0] - c[0]) / c[0] if axis == "x" else (o[:, 1] - c[1]) / c[1]
            o[:, 2] = b / 1000 * (1 - u ** 2); vs.append(o.astype(np.float32))
        r = cv2.calibrateCamera(vs, [v["img"] for v in views], (1920, 1080), K0.copy(), None,
                                flags=CC.FLAGS["k12_notan"] | cv2.CALIB_USE_INTRINSIC_GUESS, criteria=CC.TERM)
        rows.append(dict(axis=axis, bow_mm=b, rms=r[0], fx=r[1][0, 0], fy=r[1][1, 1], cx=r[1][0, 2], cy=r[1][1, 2], k1=r[2].ravel()[0], k2=r[2].ravel()[1]))
        print(axis, b, "rms %.4f fx %.2f fy %.2f cx %.2f cy %.2f k1 %.4f k2 %.4f" % (r[0], r[1][0, 0], r[1][1, 1], r[1][0, 2], r[1][1, 2], r[2].ravel()[0], r[2].ravel()[1]))
json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), set=st, note=__doc__, rows=rows), open(os.path.join(out, "bowscan.json"), "w"), indent=1, default=float)
