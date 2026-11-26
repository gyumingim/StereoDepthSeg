#!/usr/bin/env python3
"""gsmap 2026-11-26: seam fix for textured GLBs (post-process, no re-bake).
Texels whose centre lies in no UV triangle (atlas gutter / background, and gaps between charts) get the colour of the
nearest covered texel (Euclidean nearest = full Voronoi fill of every empty texel, so no mip level sees background).
Covered texels are left as decoded; V/UV/F are copied unchanged; atlas re-encoded as JPEG (same quality as gsm_texture2).
Why: gsm_texture2 dilates the gutter with cv2.dilate (= per-channel MAX of the neighbours -> brightened / white-ish /
magenta-ish rims) for only 8 px at the 10k atlas and leaves black beyond; xatlas padding is 4 px (-> ~3 px after the 8k
downscale), so bilinear + mipmap sampling picks those colours up along every chart border.
usage: gsm_seampad.py in.glb out.glb [--quality 88]"""
import argparse, json, struct, sys, time
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_texture import write_glb  # noqa: E402


def read_glb(p):
    b = open(p, "rb").read(); jl = struct.unpack("<I", b[12:16])[0]; j = json.loads(b[20:20 + jl]); bl = b[20 + jl + 8:]
    bv = j["bufferViews"]; g = lambda i: bl[bv[i]["byteOffset"]:bv[i]["byteOffset"] + bv[i]["byteLength"]]  # noqa: E731
    return (np.frombuffer(g(0), np.float32).reshape(-1, 3), np.frombuffer(g(1), np.float32).reshape(-1, 2),
            np.frombuffer(g(2), np.uint32).reshape(-1, 3), cv2.imdecode(np.frombuffer(g(3), np.uint8), 1))


def cover_mask(UV, F, W, H):
    """texel (x, y) is covered if its centre (x+.5, y+.5) lies inside a UV triangle (UV*[W,H], row = v*H as in gsm_texture2)."""
    T2 = (UV * np.array([W, H], np.float32))[F.astype(np.int64)].astype(np.float64); nf = len(T2)
    cov = np.zeros((H, W), bool)
    x0 = np.floor(T2[:, :, 0].min(1) - 0.5).astype(int); x1 = np.ceil(T2[:, :, 0].max(1)).astype(int)
    y0 = np.floor(T2[:, :, 1].min(1) - 0.5).astype(int); y1 = np.ceil(T2[:, :, 1].max(1)).astype(int)
    bw, bh = x1 - x0 + 1, y1 - y0 + 1
    for s0 in range(0, nf, 20000):
        ids = np.arange(s0, min(nf, s0 + 20000)); cnt = bw[ids] * bh[ids]
        rep = np.repeat(ids, cnt); off = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        px = x0[rep] + off % bw[rep]; py = y0[rep] + off // bw[rep]
        ok = (px >= 0) & (px < W) & (py >= 0) & (py < H); rep, px, py = rep[ok], px[ok], py[ok]
        p = np.stack([px + 0.5, py + 0.5], 1); t_ = T2[rep]
        v0, v1, v2 = t_[:, 1] - t_[:, 0], t_[:, 2] - t_[:, 0], p - t_[:, 0]
        d00 = (v0 * v0).sum(1); d01 = (v0 * v1).sum(1); d11 = (v1 * v1).sum(1); d20 = (v2 * v0).sum(1); d21 = (v2 * v1).sum(1)
        den = d00 * d11 - d01 * d01; den = np.where(np.abs(den) < 1e-12, 1e-12, den)
        b1 = (d11 * d20 - d01 * d21) / den; b2 = (d00 * d21 - d01 * d20) / den; b0 = 1 - b1 - b2
        ins = (b0 >= -1e-6) & (b1 >= -1e-6) & (b2 >= -1e-6)
        cov[py[ins], px[ins]] = True
    return cov


def colour_stats(tex, m):
    c = tex[m].astype(np.int16); b, g, r = c[:, 0], c[:, 1], c[:, 2]
    return dict(n=int(m.sum()), near_white=int((c.min(1) >= 215).sum()),
                magenta=int(((r >= 150) & (b >= 150) & (g <= 0.6 * np.minimum(r, b))).sum()), black=int((c.max(1) <= 20).sum()))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--quality", type=int, default=88)
    a = ap.parse_args(); t0 = time.time()
    assert Path(a.out).resolve() != Path(a.inp).resolve(), "never overwrite the input"
    V, UV, F, tex = read_glb(a.inp); H, W = tex.shape[:2]
    cov = cover_mask(UV, F, W, H); t_cov = time.time() - t0
    ring = cv2.dilate(cov.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & ~cov   # empty texels touching a covered one
    before = dict(ring=colour_stats(tex, ring), empty=colour_stats(tex, ~cov))
    from scipy import ndimage
    idx = ndimage.distance_transform_edt(~cov, return_distances=False, return_indices=True)
    out = tex[idx[0], idx[1]]; del idx
    ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, a.quality]); assert ok
    write_glb(a.out, V, UV, F, jpg.tobytes())
    cv2.imwrite(str(Path(a.out).with_suffix(".cover.png")), cov.astype(np.uint8) * 255)
    dec = cv2.imdecode(np.frombuffer(jpg.tobytes(), np.uint8), 1)
    mse = float(((dec[cov].astype(np.float32) - tex[cov].astype(np.float32)) ** 2).mean())
    after = dict(ring=colour_stats(dec, ring), empty=colour_stats(dec, ~cov))
    info = dict(made=time.strftime("%F %T"), inp=str(Path(a.inp).resolve()), out=str(Path(a.out).resolve()), atlas=[W, H],
                faces=int(len(F)), covered_pct=round(float(cov.mean() * 100), 3), empty_texels=int((~cov).sum()),
                before=before, after=after, covered_reencode_psnr_db=round(10 * np.log10(255 ** 2 / max(mse, 1e-9)), 2),
                in_mb=round(Path(a.inp).stat().st_size / 2 ** 20, 2), out_mb=round(Path(a.out).stat().st_size / 2 ** 20, 2),
                times_s=dict(cover=round(t_cov, 1), total=round(time.time() - t0, 1)))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
