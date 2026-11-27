import re,sys
p=sys.argv[1]; s=open(p).read()
s=s.replace('HERE = Path(__file__).resolve().parent\n','HERE = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"   # night 11-27 copy: run from runs/night_20261127/ptie\n',1)
s=s.replace('    a = ap.parse_args()\n','''    ap.add_argument("--ptie", default="", help="night 11-27 item 3: dir of KF.npz with key L (int16 planar segment ids, 0 = none) = FFS-supported planar segments")
    ap.add_argument("--ptie-w", type=float, default=0.2); ap.add_argument("--ptie-start", type=int, default=5000)
    a = ap.parse_args()
''',1)
s=s.replace('''    wt = [torch.from_numpy(w_.astype(np.float16)).to(dev) for w_ in wnp]
''','''    wt = [torch.from_numpy(w_.astype(np.float16)).to(dev) for w_ in wnp]
    plab = None
    if a.ptie:   # night 11-27: planar segment labels per view (CPU, moved per step)
        plab, npx = [], 0
        for c_, i_ in enumerate(kf):
            q_ = Path(a.ptie) / f"{i_:05d}.npz"
            L_ = np.load(q_)["L"].astype(np.int64) if q_.exists() else None
            if L_ is not None:
                L_ = np.where(vmask, L_, 0); npx += int((L_ > 0).sum())
            plab.append(torch.from_numpy(L_) if L_ is not None and (L_ > 0).any() else None)
        print(f"ptie: {sum(x is not None for x in plab)} views with planar segments, {npx} px, w {a.ptie_w} from step {a.ptie_start}", flush=True)
        vv_, uu_ = np.mgrid[0:H, 0:W]
        rays_t = torch.from_numpy(np.dstack([(uu_ - Kr[0, 2]) / Kr[0, 0], (vv_ - Kr[1, 2]) / Kr[1, 1], np.ones((H, W))]).astype(np.float32)).to(dev)
''',1)
s=s.replace('''        if a.surfel:   # gsplat 2DGS example defaults''','''        ptl = torch.zeros((), device=dev)
        if plab is not None and step >= a.ptie_start and plab[c] is not None:
            # own-plane flatness: per segment, plane fitted to the RENDERED points (detached), penalise |distance| / depth.
            # The plane offset / tilt is free (no FFS offset is imposed); only bumps inside FFS-supported planar segments are penalised.
            L_ = plab[c].to(dev, non_blocking=True); m_ = (L_ > 0) & (ra[0, ..., 0].detach() > 0.5) & (ed.detach() > 0.05)
            if m_.sum() > 100:
                ids = L_[m_]; X_ = rays_t[m_] * ed[m_][:, None]; K_ = int(ids.max()) + 1
                with torch.no_grad():
                    Xd = X_.detach().double(); cnt = torch.zeros(K_, device=dev, dtype=torch.float64).index_add_(0, ids, torch.ones_like(Xd[:, 0]))
                    mu = torch.zeros(K_, 3, device=dev, dtype=torch.float64).index_add_(0, ids, Xd) / cnt.clamp(min=1)[:, None]
                    D_ = Xd - mu[ids]; cov = torch.zeros(K_, 9, device=dev, dtype=torch.float64).index_add_(0, ids, (D_[:, :, None] * D_[:, None, :]).reshape(-1, 9))
                    cov = cov.reshape(K_, 3, 3) / cnt.clamp(min=1)[:, None, None] + torch.eye(3, device=dev, dtype=torch.float64) * 1e-12
                    nrm = torch.linalg.eigh(cov)[1][:, :, 0].float(); okk = (cnt >= 50).float()
                r_ = ((X_ - mu[ids].float()) * nrm[ids]).sum(1).abs() / ed[m_].detach().clamp(min=0.1) * okk[ids]
                ptl = r_.sum() / okk[ids].sum().clamp(min=1)
                loss = loss + a.ptie_w * ptl
        if a.surfel:   # gsplat 2DGS example defaults''',1)
s=s.replace("depth=round(float(dpl), 4),","depth=round(float(dpl), 4), ptie=round(float(ptl), 5),",1)
open(p,'w').write(s)
print(s.count("ptie"))
