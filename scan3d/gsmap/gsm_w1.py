#!/usr/bin/env python3
"""gsmap 2026-11-26 overnight W1: every stereo partner frame on ONE BA (121801).

Why: ffsonly_ba/plan_4m.json pose_src = 327 'ba' + 294 'vio_anchor' wide pairs (partner pose = VIO motion anchored on the nearest BA
frame); the relaxed tier (ffs_relax) used BA2 poses while pairs2/pairs3/wide used ba_partners poses -> depth sets on mixed poses.

  inventory (CPU) OUT/inventory.json: every pair of every depth set (ffs_pairs2/3 on VIO, ba_partners pairs2/3, ffs_4m wide on VIO,
                  ffsonly_ba wide, ffs_relax relax) with the pose source of keyframe and partner
  frames    (CPU) BA3/manifest.json = ba_partners2 frames + wide partner frames; BA3/extra_pairs.txt = ba_partners2 pairs + wide kf-partner pairs
  ba        (GPU) gsm_ba_partners.cmd_ba with OUT = BA3 (identical kf_ba settings)
  post      (CPU) gsm_ba_partners.cmd_post with OUT = BA3 (Sim3 onto the same old map frame)
  change    (CPU) OUT/ba3_change.json: registration, keyframe shift vs ba_partners / ba_partners2, per-pair baseline ratio + relative rotation
                  vs the poses each pair used before; OUT/pair_change.jsonl.gz; seeds BA3/ffs_pairs2, BA3/ffs_pairs3, OUT/wide, OUT/relax with
                  symlinks to the OLD per-keyframe files whose pairs ALL stayed within --b-tol (1 %) and --rot-tol (0.1 deg); the rest is recomputed
  ffs2 / ffs3     (GPU) gsm_ba_partners ffs2 / ffs3 with OUT = BA3 (todo = keyframes without a seeded file)
  fuse23    (CPU) gsm_ba_partners fuse (s12, s12_L) with OUT = BA3
  prep      (CPU) gsm_ffsonly_ba prep with OUT = W1, pairs2 = BA3 (plan_4m.json geometry + base_c on BA3 poses)
  wide      (GPU) gsm_ffsonly_ba ffs with OUT = W1 (BA3 poses)
  relax     (GPU) gsm_ffs_relax ffs with OUT = W1, BA2 := BA3 (plan = ffs_relax/plan_relax.json)
  fuse      (CPU) gsm_ffs_fix3 fuse, judge BA3: s4m (wide only, = ffsonly_ba rule), r15 / r0 (wide + relax, z_split 1.5 / 0)
  combine   (CPU) s4m_r = gsm_ffs_relax combine rule (non-target = s4m, target = r15 else r0 below the split)
  gate      (GPU) per target keyframe new-pixel fail (new = not in W1 s4m), revert > 30 % (>= 200 px) -> OUT/s4m_rg + gate_perkf.json
  measure   (GPU) multi-view fail % (gsm_ffs_relax.mvfail = gsm_ffs_4m measure rule) of s4m and s4m_rg, judge BA3 -> measure_w1.json
  fused     (CPU) training dir OUT/fused from --name (default s4m_rg)
"""
import argparse, gzip, json, os, shutil, sys, time
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import gsm_ba_partners as BP

R1, V2 = BP.R1, BP.V2
BAP = V2 / "ba_partners"; BA2 = V2 / "ba_partners2"; FOB = V2 / "ffsonly_ba"; REL = V2 / "ffs_relax"
OUT = V2 / "w1"; BA3 = OUT / "ba3"
AISLE = list(range(314, 332))


def poses(d):
    P = np.load(d / "poses_all.npz")
    return {int(f): T for f, T in zip(P["frames"], P["T"])}, {int(f): str(s) for f, s in zip(P["frames"], P["src"])}


def wide_old():
    w = np.load(FOB / "wide_poses.npz")
    return {int(f): (T, str(s)) for f, T, s in zip(w["frames"], w["T"], w["src"])}


def all_pairs():
    """(tier, kf, partner) of every pair the current depth sets use: p2 (ba_partners/ffs_pairs2), low (ba_partners/ffs_pairs3),
    wide (ffsonly_ba/plan_4m.json), relax (ffs_relax/relax files)."""
    rows = []
    for f in sorted((BAP / "ffs_pairs2" / "pairs").glob("*.npz")):
        d = np.load(f); rows += [("p2", int(f.stem), int(j), str(s)) for j, s in zip(d["partner"], d["src"])]
    for f in sorted((BAP / "ffs_pairs3" / "extra").glob("*.npz")):
        d = np.load(f); rows += [("low", int(f.stem), int(j), str(t)) for j, t in zip(d["partner"], d["tag"]) if str(t) == "low"]
    pl = json.loads((FOB / "plan_4m.json").read_text())["plan"]
    rows += [("wide", int(k), int(q[0]), str(q[6])) for k, v in pl.items() for q in v]
    for f in sorted((REL / "relax").glob("*.npz")):
        d = np.load(f); rows += [("relax", int(f.stem), int(j), "relax") for j in d["partner"]]
    return rows


def old_pose(tier, k, j, PB, PB2, WO):
    """poses the existing depth file of this pair was computed with: (T_kf, T_partner, partner source)."""
    TmB, srcB = PB; Tm2, src2 = PB2
    if tier in ("p2", "low"):
        return TmB[k], TmB[j], "bap:" + srcB[j]
    if tier == "wide":
        T, s = WO[j]
        return TmB[k], T, "bap:" + ("ba" if s == "ba" else "vio_anchor")
    return Tm2[k], Tm2[j], "ba2:" + src2[j]


def cmd_inventory(a):
    OUT.mkdir(parents=True, exist_ok=True)
    PB, PB2, WO = poses(BAP), poses(BA2), wide_old()
    rows = all_pairs(); inv = []
    for tier, k, j, s in rows:
        _, _, ps = old_pose(tier, k, j, PB, PB2, WO)
        inv.append(dict(tier=tier, kf=k, j=j, sub=s, kf_pose={"p2": "bap", "low": "bap", "wide": "bap", "relax": "ba2"}[tier], partner_pose=ps,
                        depth_file={"p2": f"ba_partners/ffs_pairs2/pairs/{k:05d}.npz", "low": f"ba_partners/ffs_pairs3/extra/{k:05d}.npz",
                                    "wide": f"ffsonly_ba/wide/{k:05d}.npz", "relax": f"ffs_relax/relax/{k:05d}.npz"}[tier]))
    # historical sets (not used by nC / ffsonly2 training, listed for completeness): ffs_pairs2/3 and ffs_4m wide on VIO poses
    hist = {}
    for nm, fp, key in (("ffs_pairs2_vio", V2 / "ffs_pairs2" / "plan.json", "p2"), ("ffs_pairs3_vio", V2 / "ffs_pairs3" / "plan3.json", "p3"),
                        ("ffs_4m_wide_vio", V2 / "ffs_4m" / "plan_4m.json", "w")):
        d = json.loads(fp.read_text()); pl = d["chosen"]["plan"] if key == "p2" else d["plan"]
        hist[nm] = dict(plan=str(fp), n_pairs=sum(len(v) for v in pl.values()), n_kf=len(pl), pose="VIO relative motion anchored on old keyframe BA (11-12 BA)")
    cnt = Counter((r["tier"], r["kf_pose"], r["partner_pose"]) for r in inv)
    used_by = dict(nC_ffsonly_np="ffsonly_ba/s4m = p2 + low + wide (fused, judge ba_partners)",
                   ffsonly2="ffs_relax/s4m_r = ffsonly_ba/s4m for non-target kf + r15/r0 (p2 + low (ba_partners poses) + relax (BA2 poses)) for target kf, judge BA2")
    res = dict(made=time.strftime("%F %T"), inputs=dict(p2=str(BAP / "ffs_pairs2/pairs"), low=str(BAP / "ffs_pairs3/extra"), wide_plan=str(FOB / "plan_4m.json"),
                                                         wide_poses=str(FOB / "wide_poses.npz"), relax=str(REL / "relax"), bap_poses=str(BAP / "poses_all.npz"),
                                                         ba2_poses=str(BA2 / "poses_all.npz")),
               used_by=used_by, n_pairs=len(inv), n_partner_frames=len({r["j"] for r in inv}),
               counts=[dict(tier=t, kf_pose=kp, partner_pose=pp, n=n) for (t, kp, pp), n in sorted(cnt.items())],
               not_on_one_ba=sum(n for (t, kp, pp), n in cnt.items() if not pp.endswith(":ba")) , historical=hist, pairs=inv)
    (OUT / "inventory.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "pairs"}, indent=1))


def cmd_frames(a):
    BA3.mkdir(parents=True, exist_ok=True)
    m = json.loads((BA2 / "manifest.json").read_text())
    pl = json.loads((FOB / "plan_4m.json").read_text())["plan"]
    wp = {(int(k), int(q[0])) for k, v in pl.items() for q in v}; wf = {j for _, j in wp}
    allf = sorted(set(m["keyframes"]) | wf)
    old = [l.split() for l in (BA2 / "extra_pairs.txt").read_text().splitlines() if l.strip()]
    oldp = {(int(x[:-4]), int(y[:-4])) for x, y in old}
    ex = sorted(oldp | {(min(k, j), max(k, j)) for k, j in wp})
    (BA3 / "extra_pairs.txt").write_text("".join(f"{x:05d}.jpg {y:05d}.jpg\n" for x, y in ex))
    info = dict(made=time.strftime("%F %T"), inputs=dict(manifest=str(BA2 / "manifest.json"), extra_pairs=str(BA2 / "extra_pairs.txt"), wide_plan=str(FOB / "plan_4m.json")),
                n_kf=m["n_kf"], n_frames_ba2=len(m["keyframes"]), n_wide_partner_frames=len(wf), n_new_frames=len(wf - set(m["keyframes"])), n_frames=len(allf),
                n_extra_pairs_ba2=len(oldp), n_wide_pairs=len(wp), n_extra_pairs=len(ex), keyframes=allf, old_kf=m["old_kf"])
    (BA3 / "manifest.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k not in ("keyframes", "old_kf")}, indent=1))


def cmd_ba(a):
    BP.OUT = BA3; BP.cmd_ba(a)


def cmd_post(a):
    BP.OUT = BA3; BP.cmd_post(a)


def cmd_change(a):
    import motion_stereo as ms
    PB, PB2, WO = poses(BAP), poses(BA2), wide_old(); Tm3, src3 = poses(BA3)
    post = json.loads((BA3 / "post.json").read_text())
    kf = np.load(BAP / "run/dump_ba/cams.npz")["kf"]; c3 = np.load(BA3 / "run/dump_ba/cams.npz")
    shift = {}
    for nm, d in (("vs_ba_partners", BAP), ("vs_ba_partners2", BA2)):
        o = np.load(d / "run/dump_ba/cams.npz"); assert (o["kf"] == c3["kf"]).all()
        dc = np.linalg.norm(c3["T"][:, :3, 3] - o["T"][:, :3, 3], axis=1) * 100
        r = BP.rot_deg(np.einsum("nji,njk->nik", c3["T"][:, :3, :3], o["T"][:, :3, :3]))
        shift[nm] = dict(shift_cm_med=round(float(np.median(dc)), 3), shift_cm_p95=round(float(np.percentile(dc, 95)), 3), shift_cm_max=round(float(dc.max()), 3),
                         argmax_kf_idx=int(np.argmax(dc)), aisle_med_cm=round(float(np.median(dc[AISLE])), 3),
                         rot_deg_med=round(float(np.median(r)), 4), rot_deg_max=round(float(r.max()), 4))
    rows = []
    for tier, k, j, s in all_pairs():
        Tk0, Tj0, ps = old_pose(tier, k, j, PB, PB2, WO)
        Ro, to = ms.rel_pose(Tk0, Tj0); Rn, tn = ms.rel_pose(Tm3[k], Tm3[j]); co, cn = -Ro.T @ to, -Rn.T @ tn
        Bo, Bn = float(np.linalg.norm(co)), float(np.linalg.norm(cn))
        rows.append(dict(tier=tier, kf=k, j=j, old_partner_pose=ps, new_partner_src=src3.get(j, "missing"), new_kf_src=src3.get(k, "missing"),
                         B_old_cm=round(Bo * 100, 2), B_new_cm=round(Bn * 100, 2), B_ratio=round(Bn / Bo, 5), rel_rot_deg=round(float(BP.rot_deg(Rn @ Ro.T)), 4),
                         dir_deg=round(float(np.degrees(np.arccos(np.clip(co @ cn / Bo / Bn, -1, 1)))), 3)))
    for r in rows:
        r["changed"] = bool(abs(r["B_ratio"] - 1) > a.b_tol or r["rel_rot_deg"] > a.rot_tol)

    def summ(rr):
        if not rr:
            return {}
        x = np.array([r["B_ratio"] for r in rr]); y = np.array([r["rel_rot_deg"] for r in rr]); pc = lambda z: [round(float(v), 4) for v in np.percentile(z, [5, 50, 95])]
        return dict(n=len(rr), B_ratio_p5_50_95=pc(x), B_ratio_absdev_med=round(float(np.median(np.abs(x - 1))), 4), B_ratio_absdev_max=round(float(np.abs(x - 1).max()), 4),
                    frac_B_off_gt_1pct=round(float(np.mean(np.abs(x - 1) > 0.01)), 4), frac_B_off_gt_5pct=round(float(np.mean(np.abs(x - 1) > 0.05)), 4),
                    rel_rot_deg_p5_50_95=pc(y), n_changed=int(sum(r["changed"] for r in rr)))
    groups = {"all": rows}
    for t in ("p2", "low", "wide", "relax"):
        groups[t] = [r for r in rows if r["tier"] == t]
    groups["wide_old_vio_anchor"] = [r for r in rows if r["tier"] == "wide" and r["old_partner_pose"].endswith("vio_anchor")]
    groups["wide_old_ba"] = [r for r in rows if r["tier"] == "wide" and r["old_partner_pose"].endswith(":ba")]
    # seed the unchanged keyframe files (all pairs of the file within tolerance) as symlinks to the old files
    seeds = {}
    for tier, src, dst in (("p2", BAP / "ffs_pairs2" / "pairs", BA3 / "ffs_pairs2" / "pairs"), ("low", BAP / "ffs_pairs3" / "extra", BA3 / "ffs_pairs3" / "extra"),
                           ("wide", FOB / "wide", OUT / "wide"), ("relax", REL / "relax", OUT / "relax")):
        dst.mkdir(parents=True, exist_ok=True); bykf = {}
        for r in rows:
            if r["tier"] == tier or (tier == "low" and r["tier"] == "low"):
                bykf.setdefault(r["kf"], []).append(r["changed"])
        files = sorted(src.glob("*.npz")); keep = [f for f in files if not any(bykf.get(int(f.stem), [True]))]
        if tier == "low":     # pairs3 extra files may hold non-low tags (none in the adopted set); keep only if every pair listed
            keep = [f for f in keep if all(str(t) == "low" for t in np.load(f)["tag"])]
        for f in keep:
            if not (dst / f.name).exists():
                os.symlink(f, dst / f.name)
        if tier == "p2":
            (BA3 / "ffs_pairs2" / "s1").mkdir(parents=True, exist_ok=True)
            for f in keep:
                g = BAP / "ffs_pairs2" / "s1" / f.name
                if not (BA3 / "ffs_pairs2" / "s1" / f.name).exists():
                    os.symlink(g, BA3 / "ffs_pairs2" / "s1" / f.name)
        seeds[tier] = dict(kf_files=len(files), reused=len(keep), recompute=len(files) - len(keep))
    with gzip.open(OUT / "pair_change.jsonl.gz", "wt") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    res = dict(made=time.strftime("%F %T"), inputs=dict(ba3=str(BA3 / "poses_all.npz"), bap=str(BAP / "poses_all.npz"), ba2=str(BA2 / "poses_all.npz"),
                                                         wide_old=str(FOB / "wide_poses.npz")),
               registered=post["registered"], ba=dict(runtime_s=post["ba"]["runtime_s"], n_frames=post["ba"]["n_frames"], final_med_px=post["ba"]["final_med_px"],
                                                      final_rmse_px=post["ba"]["final_rmse_px"]), reproj=post["reproj"], align_to_old_map=post["align_to_old_map"],
               partner_src_ba3=dict(Counter(r["new_partner_src"] for r in rows)), kf_src_ba3=dict(Counter(r["new_kf_src"] for r in rows)),
               kf_shift=shift, tol=dict(B_ratio=a.b_tol, rot_deg=a.rot_tol), pair_change={k: summ(v) for k, v in groups.items()}, seeds=seeds,
               note="B_ratio = BA3 keyframe-partner baseline / baseline of the poses the existing depth file used (p2/low/wide: ba_partners keyframe + "
                    "ba_partners or VIO-anchored partner; relax: BA2). A keyframe file is recomputed if ANY of its pairs changed.")
    (OUT / "ba3_change.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1))


def cmd_ffs2(a):
    BP.OUT = BA3; shutil.copy(V2 / "ffs_pairs2" / "plan.json", BA3 / "ffs_pairs2" / "plan.json"); BP.cmd_ffs2(a)


def cmd_ffs3(a):
    BP.OUT = BA3; BP.cmd_ffs3(a)


def cmd_fuse23(a):
    BP.OUT = BA3; BP.cmd_fuse(a)


def _fo():
    import gsm_ffsonly_ba as FO
    BP.OUT = BA3; FO.OUT = OUT; FO.BAP = BA3; FO.RUNN = BA3 / "run"; FO.P2N = BA3 / "ffs_pairs2"; FO.P3N = BA3 / "ffs_pairs3"
    return FO


def cmd_prep(a):
    FO = _fo(); FO.cmd_prep(a)
    d = json.loads((OUT / "plan_4m.json").read_text()); assert set(d["pose_src"]) == {"ba"}, d["pose_src"]


def cmd_wide(a):
    FO = _fo(); FO.cmd_ffs(a)


def cmd_relax(a):
    import gsm_ffs_relax as FR
    FR.OUT = OUT; FR.BA2 = BA3; shutil.copy(REL / "plan_relax.json", OUT / "plan_relax.json")
    a.theta_min, a.cap = 25.0, 0.03
    FR.cmd_ffs(a)


def cmd_fuse(a):
    import gsm_ffs_fix3 as F3
    FO = _fo(); s = OUT / "src"; (s / "wide").mkdir(parents=True, exist_ok=True); shutil.copy(OUT / "plan_4m.json", s / "plan_4m.json")
    for d in (OUT / "wide", OUT / "relax"):
        for f in d.glob("*.npz"):
            assert d.name == "wide" or not (OUT / "wide" / f.name).exists(), f
            if not (s / "wide" / f.name).exists():
                os.symlink(f.resolve(), s / "wide" / f.name)
    for nm, src, zs in (("s4m", OUT, 1.5), ("r15", s, 1.5), ("r0", s, 0.0)):
        if a.only and nm not in a.only.split(","):
            continue
        F3.cmd_fuse(FO.A_(run=str(BA3 / "run"), out=str(OUT), src=str(src), p2=str(BA3 / "ffs_pairs2"), p3=str(BA3 / "ffs_pairs3"), name=nm, z_split=zs))


def cmd_combine(a):
    import gsm_ffs_relax as FR
    FR.OUT = OUT; FR.OLD = OUT; FR.BA2 = BA3      # OLD/s4m = W1 s4m, r15/r0 = W1 runs
    FR.cmd_combine(a)


def _targets(kf):
    return [ci for ci, f in enumerate(kf) if not (FOB / "wide" / f"{int(f):05d}.npz").exists()]


def cmd_gate(a):
    import gsm_ffs_relax as FR
    run = BA3 / "run"; kf, Z, vm, outl = FR.mvfail(run, OUT / "s4m_r"); n = len(kf); tg = _targets(kf)
    per, rev = [], []
    for ci in tg:
        if outl[ci] is None:
            continue
        v, u, z, fl = outl[ci]; f = OUT / "s4m" / f"{int(kf[ci]):05d}.npz"
        old = np.isfinite(np.load(f)["Z"])[v * 2, u * 2] if f.exists() else np.zeros(len(v), bool)
        m = ~old
        if m.sum() < 200:
            continue
        fp = 100 * float(fl[m].mean()); per.append([ci, int(m.sum()), round(fp, 1)])
        if fp > a.gate:
            rev.append(ci)
    od = OUT / ("s4m_rg" + a.tag); od.mkdir(parents=True, exist_ok=True)
    for ci, i in enumerate(kf):
        nm = f"{int(i):05d}.npz"; src = (OUT / "s4m" / nm) if ci in rev else (OUT / "s4m_r" / nm)
        if (od / nm).exists() or (od / nm).is_symlink():
            os.remove(od / nm)
        if src.exists():
            os.symlink(src.resolve(), od / nm)
    fps = np.array([p[2] for p in per])
    res = dict(made=time.strftime("%F %T"), inputs=dict(set=str(OUT / "s4m_r"), judge=str(run / "dump_ba/cams.npz"), old=str(OUT / "s4m")),
               rule=f"gsm_ffs_4m measure rule on NEW pixels only (not in W1 s4m), per target keyframe with >= 200 new 2x-grid px; revert > {a.gate} %",
               n_kf=len(per), new_fail_pct_per_kf_p50_p90_max=[round(float(x), 2) for x in np.percentile(fps, [50, 90, 100])] if len(fps) else None,
               kf_idx_gt20pct=[p for p in per if p[2] > 20], reverted_kf_idx=rev, out=str(od), per_kf=per)
    (OUT / f"gate_perkf{a.tag}.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: v for k, v in res.items() if k != "per_kf"}, indent=1))


def cmd_measure(a):
    import gsm_ffs_relax as FR
    run = BA3 / "run"; t0 = time.time()
    res = dict(made=time.strftime("%F %T"), note="gsm_ffs_relax.mvfail = gsm_ffs_4m measure rule (2x grid, undistortion mask, each set its own reference, "
                                                  "fail = violated in >= 2 views and >= 30 % of checking views), judge = BA3; new pixels = not in W1 s4m; n = 1", sets={})
    old = None
    for nm in a.name.split(","):
        kf, Z, vm, outl = FR.mvfail(run, OUT / nm); n = len(kf); tg = set(_targets(kf))
        if old is None:
            old = np.zeros(Z.shape, bool)
            for ci, i in enumerate(kf):
                f = OUT / "s4m" / f"{int(i):05d}.npz"
                if f.exists():
                    old[ci] = np.isfinite(np.load(f)["Z"])

        def pct(sel, newonly=False):
            fs = ts = 0
            for ci in range(n):
                if outl[ci] is None or not sel(ci):
                    continue
                v, u, z, fl = outl[ci]
                if newonly:
                    fl = fl[~old[ci][v * 2, u * 2]]
                fs += int(fl.sum()); ts += len(fl)
            return dict(fail_pct=round(100 * fs / max(1, ts), 2), px_2x=ts)
        vf = np.isfinite(Z).reshape(n, -1).mean(1)
        e = dict(run=str(run), set=str(OUT / nm), all=pct(lambda ci: True), nontarget=pct(lambda ci: ci not in tg), target=pct(lambda ci: ci in tg),
                 aisle_314_331=pct(lambda ci: ci in AISLE), new_pixels_only=pct(lambda ci: True, True), new_pixels_target=pct(lambda ci: ci in tg, True),
                 new_pixels_aisle=pct(lambda ci: ci in AISLE, True),
                 usable_pct=dict(all=round(100 * float(vf.mean()), 2), target=round(100 * float(vf[sorted(tg)].mean()), 2), aisle_314_331=round(100 * float(vf[AISLE].mean()), 2),
                                 all_lt2pct=int((vf < 0.02).sum())))
        res["sets"][nm] = e; print(nm, json.dumps(e), f"{time.time() - t0:.0f}s", flush=True)
        del Z
    (OUT / f"measure_w1{a.tag}.json").write_text(json.dumps(res, indent=1)); print("wrote", OUT / f"measure_w1{a.tag}.json")


def cmd_fused(a):
    src = OUT / a.name; od = OUT / "fused"; od.mkdir(parents=True, exist_ok=True)
    kf = np.load(BA3 / "run" / "dump_ba" / "cams.npz")["kf"]; vf = []
    for i in kf:
        f = src / f"{int(i):05d}.npz"
        if f.exists():
            d = np.load(f); Z = d["Z"].astype(np.float32); V = np.isfinite(Z) & (Z > 0); S = np.where(V, d["S"], np.inf).astype(np.float32)
        else:
            Z = np.full((540, 960), np.nan, np.float32); V = np.zeros_like(Z, bool); S = np.full_like(Z, np.inf)
        np.savez_compressed(od / f"{int(i):05d}.npz", Z=np.where(V, Z, 0).astype(np.float32), F=V.astype(np.uint8), S=S); vf.append(float(V.mean()))
    (od / "info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), src=str(src), poses=str(BA3 / "run/dump_ba/cams.npz"),
                                                  note="FFS only (F=1), no AI depth; all pairs on BA3 poses", n=len(kf), valid_mean=float(np.mean(vf)),
                                                  kf_with_depth=int(sum(v > 0 for v in vf))), indent=1))
    print("fused", od, np.mean(vf))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["inventory", "frames", "ba", "post", "change", "ffs2", "ffs3", "fuse23", "prep", "wide", "relax", "fuse", "combine",
                                    "gate", "measure", "fused"])
    ap.add_argument("--b-tol", type=float, default=0.01); ap.add_argument("--rot-tol", type=float, default=0.1)
    ap.add_argument("--gate", type=float, default=30.0); ap.add_argument("--tag", default=""); ap.add_argument("--only", default="")
    ap.add_argument("--name", default="s4m_rg")
    a = ap.parse_args()
    if a.cmd in ("ffs2", "ffs3", "wide", "relax", "gate", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap w1 " + a.cmd, torch_cap=True):
            globals()["cmd_" + a.cmd](a)
    else:
        globals()["cmd_" + a.cmd](a)
