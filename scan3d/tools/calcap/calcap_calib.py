"""Lens calibration from calcap stills (2026-10-25). Same model and checks as the cal20261024d fit
(runs/lens_calib_f050_20261023/calib.py): k12_notan = k1 k2 free, p1 p2 k3 fixed 0 (OpenVINS radtan compatible),
one cluster per captured slot (its 3-5 stills), leave-one-capture-out, random + stratified disjoint halves.
Lens values come ONLY from this board fit - never pick or tune them by VIO error (user rule 2026-10-25).

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/tools/calcap/calcap_calib.py <set dir> [<set dir> ...]
      [--models k12_notan k1k2p] [--tag name] [--true-lens fake/true_lens.json]
-> <first set>/calib/calib_<tag>.json (everything), lens_candidate_<tag>.json (NOT installed anywhere),
   summary_<tag>.txt (Korean, short). Exit code 0 even when the repeatability checks fail (read the summary).
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slots as SL  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
W, H = 1920, 1080
TERM = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-10)
FLAGS = {
    "k12_notan": cv2.CALIB_FIX_TANGENT_DIST | cv2.CALIB_FIX_K3,
    "k1k2p": cv2.CALIB_FIX_K3,
    "k123_notan": cv2.CALIB_FIX_TANGENT_DIST,
}
DEFAULT = os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json")
FX_TOL_PCT, PP_TOL_PX = 0.3, 3.0


def load_views(sets, boards, guide_K, guide_d, log):
    views, caps = [], []
    for sd in sets:
        cj = json.load(open(os.path.join(sd, "captures.json"), encoding="utf-8"))
        for c in cj["captures"]:
            if not c.get("ok"):
                continue
            cid = f"{os.path.basename(sd)}#{c['cap']}#{c['dir']}"
            nv = 0
            for rel in c["files"]:
                g = cv2.imread(os.path.join(sd, rel), cv2.IMREAD_UNCHANGED)
                if g is None or g.shape != (H, W):
                    log(f"  skip {rel}: unreadable / size")
                    continue
                for b in boards:
                    cc, ids = b.detect(g)
                    if len(ids) >= b.min_corners:
                        obj = b.obj[ids]
                        if min(np.ptp(obj[:, 0]), np.ptp(obj[:, 1])) < 0.5 * b.sq:
                            continue
                        p = SL.pose_of(b, cc, ids, guide_K, guide_d)
                        views.append(dict(cap=cid, slot=c["slot"], file=rel, board=b.name, ids=ids,
                                          obj=obj.astype(np.float32), img=cc.astype(np.float32).reshape(-1, 1, 2),
                                          tilt=None if p is None else p["tilt"], dist=None if p is None else p["dist"]))
                        nv += 1
                        break
            if nv:
                caps.append(dict(id=cid, slot=c["slot"], n=nv, motion_px=c.get("motion_px")))
    return views, caps


def fit(views, model):
    rms, K, d, rv, tv = cv2.calibrateCamera([v["obj"] for v in views], [v["img"] for v in views], (W, H), None, None,
                                            flags=FLAGS[model], criteria=TERM)
    return float(rms), K, d.ravel(), rv, tv


def view_err(v, K, d, rv=None, tv=None):
    if rv is None:
        ok, rv, tv = cv2.solvePnP(v["obj"], v["img"], K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    p, _ = cv2.projectPoints(v["obj"], rv, tv, K, d)
    return float(np.sqrt(np.mean(np.sum((p - v["img"]).reshape(-1, 2) ** 2, -1))))


def summ(K, d):
    return dict(fx=round(float(K[0, 0]), 2), fy=round(float(K[1, 1]), 2), cx=round(float(K[0, 2]), 2),
                cy=round(float(K[1, 2]), 2), dist=[round(float(x), 6) for x in d[:5]])


def coverage(views):
    pts = np.concatenate([v["img"].reshape(-1, 2) for v in views])
    fine = np.zeros((18, 32), bool)
    for x, y in pts:
        fine[min(int(y / H * 18), 17), min(int(x / W * 32), 31)] = True
    r = np.hypot(pts[:, 0] - W / 2, pts[:, 1] - H / 2) / np.hypot(W / 2, H / 2)
    return dict(fine_frac=round(float(fine.mean()), 3), r_max=round(float(r.max()), 3), r_p90=round(float(np.quantile(r, 0.9)), 3),
                x_range=[round(float(pts[:, 0].min())), round(float(pts[:, 0].max()))],
                y_range=[round(float(pts[:, 1].min())), round(float(pts[:, 1].max()))])


def run_model(views, caps, model, rng, n_split=30):
    t0 = time.time()
    rms, K, d, rv, tv = fit(views, model)
    per = np.array([view_err(v, K, d, a, b) for v, a, b in zip(views, rv, tv)])
    keep = per < 2.5 * np.median(per)
    dropped = [v["file"] for v, k in zip(views, keep) if not k]
    vs = [v for v, k in zip(views, keep) if k]
    if dropped:
        rms, K, d, rv, tv = fit(vs, model)
    ids = sorted({v["cap"] for v in vs})
    # leave one capture (slot) out
    lopo = {}
    for c in ids:
        tr = [v for v in vs if v["cap"] != c]; te = [v for v in vs if v["cap"] == c]
        r = fit(tr, model)
        lopo[c] = dict(summ(r[1], r[2]), test_err=float(np.median([view_err(v, r[1], r[2]) for v in te])))
    # disjoint halves: random + stratified (tilted > 25 deg vs the rest)
    tilt_of = {c: np.mean([v["tilt"] for v in vs if v["cap"] == c and v["tilt"] is not None] or [0]) for c in ids}
    tilted = [c for c in ids if tilt_of[c] > 25]; flat = [c for c in ids if tilt_of[c] <= 25]
    splits = []
    for k in range(n_split):
        splits.append(("rand", set(rng.permutation(ids)[:len(ids) // 2].tolist())))
    for k in range(n_split):
        S = set(rng.permutation(tilted)[:len(tilted) // 2].tolist()) | set(rng.permutation(flat)[:len(flat) // 2].tolist())
        splits.append(("strat", S))
    halves = []
    for nm, S in splits:
        a = [v for v in vs if v["cap"] in S]; b = [v for v in vs if v["cap"] not in S]
        if len(a) < 6 or len(b) < 6:
            continue
        ra, rb = fit(a, model), fit(b, model)
        Ka, Kb = ra[1], rb[1]
        halves.append(dict(kind=nm, dfx_pct=float((Kb[0, 0] - Ka[0, 0]) / Ka[0, 0] * 100),
                           dcx=float(Kb[0, 2] - Ka[0, 2]), dcy=float(Kb[1, 2] - Ka[1, 2])))
    hv = np.array([[abs(h["dfx_pct"]), abs(h["dcx"]), abs(h["dcy"])] for h in halves]) if halves else np.zeros((0, 3))
    pass_fx = hv[:, 0] <= FX_TOL_PCT if len(hv) else np.zeros(0, bool)
    pass_pp = (np.maximum(hv[:, 1], hv[:, 2]) <= PP_TOL_PX) if len(hv) else np.zeros(0, bool)
    # groups: fx when only that group is used (small groups -> less certain)
    groups = {}
    for gname, sel in (("near<0.95m", lambda v: v["dist"] is not None and v["dist"] < SL.DSPLIT),
                       ("far>=0.95m", lambda v: v["dist"] is not None and v["dist"] >= SL.DSPLIT),
                       ("tilt<=25", lambda v: v["tilt"] is not None and v["tilt"] <= 25),
                       ("tilt>25", lambda v: v["tilt"] is not None and v["tilt"] > 25)):
        g = [v for v in vs if sel(v)]
        if len({v["cap"] for v in g}) >= 6:
            r = fit(g, model)
            groups[gname] = dict(summ(r[1], r[2]), n_caps=len({v["cap"] for v in g}), rms=round(r[0], 4))
        else:
            groups[gname] = dict(n_caps=len({v["cap"] for v in g}), note="too few captures for a separate fit")
    try:
        ext = cv2.calibrateCameraExtended([v["obj"] for v in vs], [v["img"] for v in vs], (W, H), None, None,
                                          flags=FLAGS[model], criteria=TERM)
        sd = [round(float(x), 3) for x in ext[5].ravel()[:4]]
    except cv2.error:
        sd = None
    L = [x for x in lopo.values()]
    return dict(model=model, lens=summ(K, d), K=K.tolist(), dist=d.tolist(), rms_px=round(rms, 4), n_views=len(vs),
                n_caps=len(ids), dropped=dropped, sd_fx_fy_cx_cy=sd,
                lopo_test_median_px=round(float(np.median([x["test_err"] for x in L])), 4) if L else None,
                lopo_fx_range=[min(x["fx"] for x in L), max(x["fx"] for x in L)] if L else None,
                lopo_cx_range=[min(x["cx"] for x in L), max(x["cx"] for x in L)] if L else None,
                lopo_cy_range=[min(x["cy"] for x in L), max(x["cy"] for x in L)] if L else None,
                lopo=lopo, halves_n=len(halves),
                halves_abs_p50=dict(zip(("dfx_pct", "dcx", "dcy"), [round(float(x), 3) for x in np.median(hv, 0)])) if len(hv) else None,
                halves_abs_max=dict(zip(("dfx_pct", "dcx", "dcy"), [round(float(x), 3) for x in hv.max(0)])) if len(hv) else None,
                halves_pass=dict(fx=round(float(pass_fx.mean()), 2) if len(hv) else None,
                                 pp=round(float(pass_pp.mean()), 2) if len(hv) else None,
                                 both=round(float((pass_fx & pass_pp).mean()), 2) if len(hv) else None,
                                 rule=f"|dfx| <= {FX_TOL_PCT} %, |dcx|,|dcy| <= {PP_TOL_PX} px"),
                groups=groups, coverage=coverage(vs), sec=round(time.time() - t0, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sets", nargs="+")
    ap.add_argument("--models", nargs="*", default=["k12_notan", "k1k2p"])
    ap.add_argument("--tag", default=time.strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--true-lens", default="", help="dry test: compare with the lens the fake phone rendered with")
    ap.add_argument("--splits", type=int, default=30)
    a = ap.parse_args()
    out = os.path.join(a.sets[0], "calib")
    os.makedirs(out, exist_ok=True)
    lines = []

    def log(s):
        print(s, flush=True); lines.append(s)
    boards = SL.load_boards(os.path.join(S3D, "config", "charuco"))
    gK, gd = SL.load_lens(DEFAULT)
    views, caps = load_views(a.sets, boards, gK, gd, log)
    log(f"입력: 세트 {len(a.sets)}개, 캡처 {len(caps)}개, 사진 {len(views)}장 (모서리 {sum(len(v['ids']) for v in views)}개)")
    if len(caps) < 8:
        log("캡처가 8개보다 적어서 보정 안 함 (칸을 더 채우세요)")
        json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), sets=a.sets, error="too few captures", n_caps=len(caps)),
                  open(os.path.join(out, f"calib_{a.tag}.json"), "w"), indent=1)
        return
    rng = np.random.default_rng(0)
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), sets=[os.path.abspath(s) for s in a.sets], captures=caps,
               default_lens=DEFAULT, models={})
    dK, dd = SL.load_lens(DEFAULT)
    for m in a.models:
        r = run_model(views, caps, m, rng, a.splits)
        res["models"][m] = r
        Lz = r["lens"]
        r["vs_default"] = dict(dfx_pct=round((Lz["fx"] - dK[0, 0]) / dK[0, 0] * 100, 3), dcx=round(Lz["cx"] - dK[0, 2], 2),
                               dcy=round(Lz["cy"] - dK[1, 2], 2))
        if a.true_lens:
            tK = np.array(json.load(open(a.true_lens))["K"])
            r["vs_true"] = dict(dfx_pct=round((Lz["fx"] - tK[0, 0]) / tK[0, 0] * 100, 3), dfy_pct=round((Lz["fy"] - tK[1, 1]) / tK[1, 1] * 100, 3),
                                dcx=round(Lz["cx"] - tK[0, 2], 2), dcy=round(Lz["cy"] - tK[1, 2], 2))
        hp = r["halves_pass"]
        log(f"[{m}] fx {Lz['fx']} fy {Lz['fy']} cx {Lz['cx']} cy {Lz['cy']} k {Lz['dist'][:2]} · rms {r['rms_px']} px · "
            f"사진 {r['n_views']} (버림 {len(r['dropped'])})")
        log(f"   한 칸 빼기(LOPO): fx {r['lopo_fx_range']} cx {r['lopo_cx_range']} cy {r['lopo_cy_range']} · 시험오차 {r['lopo_test_median_px']} px")
        log(f"   반반 나누기 {r['halves_n']}회: 중앙값 |dfx| {r['halves_abs_p50']['dfx_pct']} % |dcx| {r['halves_abs_p50']['dcx']} "
            f"|dcy| {r['halves_abs_p50']['dcy']} px · 통과 fx {hp['fx']} pp {hp['pp']} 둘다 {hp['both']} ({hp['rule']})")
        log(f"   표준편차 fx fy cx cy {r['sd_fx_fy_cx_cy']} · 지금 기본값(cal20261024d) 대비 {r['vs_default']}"
            + (f" · 참값 대비 {r['vs_true']}" if a.true_lens else ""))
        log(f"   그룹별 fx: " + ", ".join(f"{k} {v.get('fx', '-')} ({v['n_caps']}칸)" for k, v in r["groups"].items()))
        log(f"   덮은 정도: {r['coverage']}")
    main_m = a.models[0]
    r = res["models"][main_m]
    verdict = ("반복성 통과" if (r["halves_pass"]["both"] or 0) >= 0.8 else "반복성 부족 (반반 나누기 통과율 낮음) - 후보로만 보관")
    log(f"판정({main_m}): {verdict}. 이 값은 config/lens 에 자동으로 넣지 않음 (사람이 확인 후 결정).")
    res["verdict"] = verdict
    json.dump(res, open(os.path.join(out, f"calib_{a.tag}.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    cand = dict(K=r["K"], dist=(r["dist"] + [0.0] * 5)[:5], focus_dpt=0.5, model="k1k2p", fit=main_m, rms_px=r["rms_px"],
                n_views=r["n_views"], n_captures=r["n_caps"], status="candidate (calcap, not installed)", verdict=verdict,
                source=res["sets"], made=res["made"],
                stream="std60/VIO 1920x1080 YUV, TEMPLATE_RECORD, focus 0.5 dpt, 2 ms, ISO auto once (calcap stills at 10 fps)")
    json.dump(cand, open(os.path.join(out, f"lens_candidate_{a.tag}.json"), "w"), indent=1)
    open(os.path.join(out, f"summary_{a.tag}.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    log(f"결과: {out}")


if __name__ == "__main__":
    main()
