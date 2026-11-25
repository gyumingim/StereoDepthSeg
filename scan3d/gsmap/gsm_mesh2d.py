#!/usr/bin/env python3
"""gsmap 2026-11-25: mesh from a 2DGS run (gsm_train --surfel 1): render median depth + colour at every keyframe pose,
scanA tsdf_map.TSDFMap (voxel 5 mm, pixel sigma 1 cm -> trunc 2 cm, depth <= 3 m, alpha > 0.5, min_obs 2; Open3D 0.20 legacy
ScalableTSDFVolume returned an empty mesh even on a synthetic flat depth here), coloured mesh, decimate to <= --max-tri.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_mesh2d.py RUN_DIR OUT.ply
World frame = the splat frame (dump T = board frame), so it lines up with splat.ply.
"""
import json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main():
    import torch, open3d as o3d
    from gsplat import rasterization_2dgs
    run, outp = Path(sys.argv[1]), Path(sys.argv[2])
    max_tri = int(sys.argv[3]) if len(sys.argv) > 3 else 1_500_000
    t0 = time.time()
    ck = torch.load(run / "gaussians.pt", weights_only=False)
    p = {k: v.cuda() for k, v in ck["params"].items()}
    T, Kr, (W, H) = ck["T"], ck["Kr"], ck["size"]
    K = torch.from_numpy(Kr).float().cuda()
    import tsdf_map
    vol = tsdf_map.TSDFMap(voxel_m=0.005, z_max=3.0, sigma_max=0.08, sigma_ok=0.02, min_obs=2, trunc_max=0.02, max_blocks=400000)
    with torch.no_grad():
        for c in range(len(T)):
            vm = torch.from_numpy(np.linalg.inv(T[c])).float().cuda()[None]
            r = rasterization_2dgs(p["means"], torch.nn.functional.normalize(p["quats"], dim=-1), torch.exp(p["scales"]),
                                   torch.sigmoid(p["opacities"]), torch.cat([p["sh0"], p["shN"]], 1), vm, K[None], W, H,
                                   sh_degree=ck["sh_degree"], render_mode="RGB+ED", near_plane=0.02, far_plane=20.0)
            rgb = (r[0][0, ..., :3].clamp(0, 1).cpu().numpy() * 255).astype(np.uint8)
            A = r[1][0, ..., 0].cpu().numpy(); Z = r[5][0, ..., 0].float().cpu().numpy()
            Z = np.where((A > 0.5) & np.isfinite(Z) & (Z > 0.05), Z, 0).astype(np.float32)
            V = Z > 0
            vol.integrate(np.where(V, Z, np.nan).astype(np.float32), V, np.ascontiguousarray(rgb[..., ::-1]), float(Kr[0, 0]),
                          float(Kr[0, 2]), float(Kr[1, 2]), 0.08, T[c], sigma=np.where(V, 0.01, np.inf).astype(np.float32))
    vol.update_mesh(force=True)
    Vm, Cm, Fm = vol.mesh_arrays()
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(Vm, np.float64)), o3d.utility.Vector3iVector(np.asarray(Fm, np.int32)))
    cm = np.asarray(Cm, np.float64); m.vertex_colors = o3d.utility.Vector3dVector(cm / 255.0 if cm.max() > 1.5 else cm)
    n0 = len(m.triangles)
    if n0 > max_tri:
        m = m.simplify_quadric_decimation(max_tri)
    m.remove_unreferenced_vertices()
    o3d.io.write_triangle_mesh(str(outp), m, write_ascii=False, compressed=False, write_vertex_normals=False,
                               write_vertex_colors=True, write_triangle_uvs=False)
    info = dict(made=time.strftime("%F %T"), run=str(run), n_views=len(T), tri_before=n0, tri=len(m.triangles), verts=len(m.vertices),
                runtime_s=round(time.time() - t0, 1), voxel=0.005, trunc=0.02, depth_trunc=3.0, alpha_min=0.5)
    (run / "mesh2d.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap mesh2d " + sys.argv[1], torch_cap=True):
        main()
