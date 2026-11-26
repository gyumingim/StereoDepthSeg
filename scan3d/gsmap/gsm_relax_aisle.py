#!/usr/bin/env python3
"""gsmap 2026-11-26: aisle-ROI FFS-support share (audit7 box x -0.3..3.3, z -5.0..-3.2, all heights) for meshes + per-face votes
from gsm_ffs_support.py (support_votes.npz, same face order as the mesh it was run on). CPU, light.
  python gsm_relax_aisle.py OUT.json name=MESH.ply:VOTES.npz [...]
Per entry: ROI face area, share with agree >= 2 (supported, N2), share with agree == 0 and >= 1 disagreeing view ('wrong'),
share never seen with FFS (no agree, no disagree), whole-mesh supported share."""
import json, sys, time
import numpy as np
import open3d as o3d


def main():
    out = sys.argv[1]; res = dict(made=time.strftime("%F %T"), roi=dict(x=[-0.3, 3.3], z=[-5.0, -3.2]), rows={})
    for spec in sys.argv[2:]:
        nm, rest = spec.split("=", 1); mp, vp = rest.split(":")
        m = o3d.io.read_triangle_mesh(mp); V = np.asarray(m.vertices); F = np.asarray(m.triangles)
        vt = np.load(vp); ag, fr, bh = vt["agree"], vt["front"], vt["behind"]; assert len(ag) == len(F), (nm, len(ag), len(F))
        ar = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1); c = V[F].mean(1)
        R = (c[:, 0] > -0.3) & (c[:, 0] < 3.3) & (c[:, 2] > -5.0) & (c[:, 2] < -3.2); a = ar[R].sum()
        pc = lambda s: round(float(ar[R & s].sum() / a * 100), 1)
        res["rows"][nm] = dict(mesh=mp, votes=vp, roi_area_m2=round(float(a), 2), roi_supported_ge2_pct=pc(ag >= 2), roi_unsupported_lt2_pct=pc(ag < 2),
                               roi_unsupported_area_m2=round(float(ar[R & (ag < 2)].sum()), 2), roi_wrong_pct=pc((ag < 2) & ((fr + bh) > 0)),
                               roi_never_pct=pc((ag == 0) & ((fr + bh) == 0)), whole_supported_ge2_pct=round(float(ar[ag >= 2].sum() / ar.sum() * 100), 1))
        print(nm, res["rows"][nm], flush=True)
    open(out, "w").write(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
