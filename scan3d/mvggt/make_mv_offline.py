"""Generate mv_offline.py from scanA/scan_offline.py by exact string patches (FFS path unchanged, only indented under 'else').
Run once (python3 make_mv_offline.py); the output is committed next to it."""
from pathlib import Path

H = Path(__file__).resolve().parent
src = (H.parent / "scanA" / "scan_offline.py").read_text()
P = [
    ("    ap.add_argument(\"--compress\", action=\"store_true\",",
     "    ap.add_argument(\"--ext-depth\", default=\"\", help=\"mvggt: npz (t_ns, depth (N,H,W) m in pose-file units, mask) on the \"\n"
     "                    \"undistorted keyframe grid; replaces FFS. --free-scale multiplies it by the same oracle scale\")\n"
     "    ap.add_argument(\"--kf-subset\", default=\"\", help=\"mvggt: text file of t_ns; only these keyframes are fused/scored \"\n"
     "                    \"(FFS pair candidates still use every row with a pose)\")\n"
     "    ap.add_argument(\"--sig-a\", type=float, default=0.003, help=\"ext depth sigma = a + b*Z^2 (m)\")\n"
     "    ap.add_argument(\"--sig-b\", type=float, default=0.01)\n"
     "    ap.add_argument(\"--compress\", action=\"store_true\","),
    ("    pose_info = dict(source=a.gt)\n",
     "    pose_info = dict(source=a.gt)\n    zscale = 1.0                                   # mvggt: oracle scale applied to external depth\n"),
    ("            ta = mb - Ra @ ma\n        Tal = np.eye(4)",
     "            ta = mb - Ra @ ma\n            zscale = sc\n        Tal = np.eye(4)"),
    ("    import lib_ffs\n    model = lib_ffs.load(valid_iters=a.iters)\n",
     "    ext = None\n"
     "    if a.ext_depth:\n"
     "        _e = np.load(a.ext_depth)\n"
     "        ext = dict(t=_e[\"t_ns\"].astype(np.int64), D=_e[\"depth\"], M=_e[\"mask\"])\n"
     "        assert ext[\"D\"].shape[1:] == (H0, W0), (ext[\"D\"].shape, H0, W0)\n"
     "        assert np.all(np.diff(ext[\"t\"]) > 0)\n"
     "        pose_info[\"ext_depth\"] = dict(file=a.ext_depth, n=int(len(ext[\"t\"])), zscale=float(zscale))\n"
     "        model = None\n"
     "    else:\n"
     "        import lib_ffs\n"
     "        model = lib_ffs.load(valid_iters=a.iters)\n"
     "    subset = None\n"
     "    if a.kf_subset:\n"
     "        subset = np.sort(np.loadtxt(a.kf_subset, dtype=np.int64).reshape(-1))\n"
     "        pose_info[\"kf_subset\"] = dict(file=a.kf_subset, n=int(len(subset)))\n"
     "\n"
     "    def _near(arr, t, tol=1e6):\n"
     "        k = int(np.searchsorted(arr, t))\n"
     "        best = -1\n"
     "        for c in (k - 1, k):\n"
     "            if 0 <= c < len(arr) and abs(int(arr[c]) - int(t)) < tol and (best < 0 or abs(int(arr[c]) - int(t)) < abs(int(arr[best]) - int(t))):\n"
     "                best = c\n"
     "        return best\n"),
    ("        if not ok[i] or imgs[i] is None:\n            continue\n        cands = []\n",
     "        if not ok[i] or imgs[i] is None:\n            continue\n"
     "        if subset is not None and _near(subset, t_ns[i]) < 0:\n            continue\n"
     "        if ext is not None:\n"
     "            ke = _near(ext[\"t\"], t_ns[i])\n"
     "            if ke < 0:\n"
     "                rows.append(dict(i=i, pairs=0, no_ext=True))\n"
     "                continue\n"
     "            Z = ext[\"D\"][ke].astype(np.float32) * np.float32(zscale)\n"
     "            V = ext[\"M\"][ke].astype(bool) & np.isfinite(Z) & (Z > 0)\n"
     "            S = (a.sig_a + a.sig_b * Z * Z).astype(np.float32)\n"
     "            res = [dict(B=0.0, ms=dict(ffs=0.0))]\n"
     "        else:\n"
     "        cands = []\n"),
]
for a_, b_ in P:
    assert src.count(a_) == 1, a_[:70]
    src = src.replace(a_, b_)
lines = src.split("\n")
s0 = next(k for k, l in enumerate(lines) if l == "        else:" and lines[k + 1] == "        cands = []")
e0 = next(k for k, l in enumerate(lines) if "Z, V, S, agree = ms.combine_motion(res, cfg)" in l)
for k in range(s0 + 1, e0 + 1):
    if lines[k].strip():
        lines[k] = "    " + lines[k]
src = "\n".join(lines)
src = src.replace('"""scanA 2026-11-05 step 3',
                  '"""mvggt 2026-11-06: generated from scanA/scan_offline.py by make_mv_offline.py (adds --ext-depth, --kf-subset).\n'
                  'scanA 2026-11-05 step 3', 1)
old = "    sys.path.insert(0, str(Path(__file__).resolve().parent))\n    import gpu_guard"
assert src.count(old) == 1
src = src.replace(old, "    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / \"scanA\"))\n    import gpu_guard")
(H / "mv_offline.py").write_text(src)
print("ok")
