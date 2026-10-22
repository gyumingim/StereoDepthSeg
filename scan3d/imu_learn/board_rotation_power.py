#!/usr/bin/env python3
"""Power test for the board-rotation metric (runs/board_rotation_review/judge.md, next step 1). No new recordings.

Inject a fake slow GT bias b(t) along board y into the matched GT positions of the valid runs of 134204 (rotated) and
134550 (control), refit Sim3 and recompute the same L30 metric as imu_learn/board_rotation.py (same match / umeyama /
lowpass from ate_floor, same u_g). Report board-y L30 RMS (cm), vertical L30 RMS and r = f_y / f_v.

Bias shapes (A = RMS of the injected bias over an infinite window, cm):
  sine  : A*sqrt2*sin(2 pi t / T + phi), T in {40, 60, 120} s, phi in {0, 90, 180, 270} deg  (t = GT time from GT start)
  ramp  : A*sqrt3*(2 t/T_rec - 1)   and its negative (linear drift over the whole recording)
raw: A is the injected RMS. cal: amplitude scaled per shape and run so that the bias itself shows as A cm of board-y L30
RMS after Sim3 (= the scale the old >30 s floor 0.2-0.4 cm was measured on).
"transfer" = L30 RMS along board y of (error_injected - error_base): how much of the bias survives Sim3 + L30.
Run from scan3d/:  ../venv_ffs/bin/python imu_learn/board_rotation_power.py   -> runs/board_rotation/power.json
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ate_floor import lowpass, match, umeyama  # noqa: E402
from board_rotation import OUT, fractions, gravity_dir, name_of  # noqa: E402

RECS = ["hs_20261022_134204", "hs_20261022_134550"]
AMPS_CM = [0.0, 0.2, 0.3, 0.4]
PERIODS = [40.0, 60.0, 120.0]
PHASES = [0, 90, 180, 270]


def metric(P, Pg, t, ug):
    Rs, c, ts = umeyama(P, Pg, scale=True)
    es = c * P @ Rs.T + ts - Pg
    L = lowpass(t, es, 1 / 30)
    fv, fy = fractions(L, ug), fractions(L, np.eye(3)[1])
    ax = [float(np.sqrt(np.mean(L[:, k] ** 2))) * 100 for k in range(3)]
    return dict(y_rms=ax[1], x_rms=ax[0], z_rms=ax[2], v_rms=float(np.sqrt(np.mean((L @ ug) ** 2))) * 100,
                rms=float(np.sqrt((L ** 2).sum(1).mean())) * 100, f_v=fv, f_y=fy, r=fy / fv), L


def shapes(tg, trec):
    out = []
    for T in PERIODS:
        for ph in PHASES:
            out.append((f"sin T{int(T)} ph{ph}", np.sqrt(2) * np.sin(2 * np.pi * tg / T + np.radians(ph))))
    out.append(("ramp +", np.sqrt(3) * (2 * tg / trec - 1)))
    out.append(("ramp -", -np.sqrt(3) * (2 * tg / trec - 1)))
    return out


def main():
    res = dict(made=time.strftime("%F %T"), script="imu_learn/board_rotation_power.py", gt="imu_gt_spl",
               inputs="runs/board_rotation/<rec>_base_o?.json + poses (ok runs only)", amps_cm=AMPS_CM,
               periods_s=PERIODS, phases_deg=PHASES, recs={})
    for rec in RECS:
        ug = gravity_dir(rec)[0]
        g = np.load(Path("recordings") / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
        gt0 = int(g["t_ns"][0])
        rr = dict(u_g_board=[round(float(v), 4) for v in ug], runs={})
        commons = {}                                            # (mode, shape, A) -> list of (tg, L30 board y)
        for k in range(3):
            jf = OUT / f"{name_of(rec, k)}.json"
            j = json.load(open(jf))
            if j["ate_cm"] > 10 or not (0.5 <= j["scale"] <= 2):
                continue
            poses = [json.loads(x) for x in gzip.open(jf.parent / (jf.stem + ".poses.jsonl.gz"), "rt") if x.strip()]
            M = match(Path("recordings") / rec, j, poses, "imu_gt_spl")
            P, Pg, t = M["P"], M["Pg"], M["t"]
            tg = (M["vt"] - gt0) / 1e9                         # GT time -> same bias for every start offset
            trec = (int(g["t_ns"][-1]) - gt0) / 1e9
            base, Lb = metric(P, Pg, t, ug)
            commons.setdefault(("base", "-", 0.0), []).append((tg, Lb[:, 1]))
            rows = []
            for name, sh in shapes(tg, trec):
                # unit transfer: L30 board-y RMS left of a 1 cm-RMS bias after Sim3 + L30
                _, L1 = metric(P, Pg + np.outer(sh / 100, [0, 1, 0]), t, ug)
                unit = float(np.sqrt(np.mean((L1[:, 1] - Lb[:, 1]) ** 2))) * 100
                for mode in ("raw", "cal"):
                    for A in AMPS_CM[1:]:
                        amp = A if mode == "raw" else A / unit      # cal: bias that shows as A cm in the metric
                        Pg2 = Pg.copy()
                        Pg2[:, 1] += amp / 100 * sh
                        m, L = metric(P, Pg2, t, ug)
                        tr = float(np.sqrt(np.mean((L[:, 1] - Lb[:, 1]) ** 2))) * 100
                        commons.setdefault((mode, name, A), []).append((tg, L[:, 1]))
                        rows.append(dict(mode=mode, shape=name, A=A, injected_rms_cm=round(amp, 3), transfer_cm=round(tr, 3),
                                         **{kk: round(v, 4) for kk, v in m.items()}))
            summ = {}
            for mode in ("raw", "cal"):
              for A in AMPS_CM[1:]:
                for fam in ("sin", "ramp", "all"):
                    sel = [x for x in rows if x["mode"] == mode and x["A"] == A and (fam == "all" or x["shape"].startswith(fam))]
                    summ[f"{mode}_A{A}_{fam}"] = {q: [round(min(x[q] for x in sel), 3), round(float(np.median([x[q] for x in sel])), 3),
                                               round(max(x[q] for x in sel), 3)] for q in ("y_rms", "transfer_cm", "r", "v_rms")}
            rr["runs"][f"o{k}"] = dict(base={kk: round(v, 4) for kk, v in base.items()}, summary_min_med_max=summ, rows=rows)
            print(rec, f"o{k}", "base y", round(base["y_rms"], 3), "r", round(base["r"], 3), flush=True)
            for A in [(m, a) for m in ("raw", "cal") for a in AMPS_CM[1:]]:
                s = summ[f"{A[0]}_A{A[1]}_all"]
                print(f"   {A}: y_rms {s['y_rms']}  transfer {s['transfer_cm']}  r {s['r']}", flush=True)
        # common component (mean over start offsets on a shared GT-time grid) of board-y L30 -- the part a GT bias lives in
        cm = {}
        for key, lst in commons.items():
            lo, hi = max(x[0][0] for x in lst), min(x[0][-1] for x in lst)
            grid = np.arange(lo, hi, 1 / 60)
            C = np.mean([np.interp(grid, a, b) for a, b in lst], 0)
            cm[key] = float(np.sqrt(np.mean(C ** 2))) * 100
        rr["common_y_rms_cm_base"] = round(cm[("base", "-", 0.0)], 3)
        rr["common_y_rms_cm"] = {f"{m}_A{A}": [round(min(v for k, v in cm.items() if k[0] == m and k[2] == A), 3),
                                              round(float(np.median([v for k, v in cm.items() if k[0] == m and k[2] == A])), 3),
                                              round(max(v for k, v in cm.items() if k[0] == m and k[2] == A), 3)]
                                 for m in ("raw", "cal") for A in AMPS_CM[1:]}
        print(rec, "common board-y L30 RMS base", rr["common_y_rms_cm_base"], rr["common_y_rms_cm"], flush=True)
        res["recs"][rec] = rr
    json.dump(res, open(OUT / "power.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
