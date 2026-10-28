#!/usr/bin/env python3
"""lgpost 2026-10-28: LighterGlue L6 (lgnpu, native LN, tanh GELU, 6 layers) with the match decision moved INTO the model.
CPU post today (nncrun lgpost 1/2): P = A * BT^T, sc = log P + logsig(z0)_i + logsig(z1)_j, m0 = argmax_j, m1 = argmax_i,
match i <-> m0[i] if m1[m0[i]] == i and exp(max_j sc_ij) > 0.1.
In model: S = m0 m1^T;  sc = LogSoftmax(S, axis 2) + LogSoftmax(S, axis 1) + E,  E = [ls0, 1] @ [1, ls1]^T  (MatMul K=2, no broadcast)
  (log A_ij = LSr_ij, log BT_ji = LSc_ij  -> identical to log P up to float rounding)
  out [1, N, 3] = (argmax_j sc_ij, argmax_i sc_ij (column j stored at row j), max_j sc_ij)
  CPU finish O(N): ok_i = (m1[m0[i]] == i) and rmax_i > ln 0.1  (+ i, m0[i] < Nvalid in the masked model)
Masked variant (M): input x [1, N, 513], col 512 = v (1 real point, 0 padding). pb = BIG*(v-1) (Linear).
  every attention X->Y uses [qX, vX] @ [kY, pbY]^T  (= qk + pb_j for real queries) -> padding keys get exp(-BIG) weight;
  final S uses [m0, v0, pb0] @ [m1, pb1, v1]^T, E uses ls0+pb0 / ls1+pb1 -> padding rows/cols score <= -BIG.
python lgpost.py check            -> PC fp32 match-set agreement vs current CPU post (torch + ORT), json
python lgpost.py export NAME N M  -> onnx + random golden/calib raw (synthetic random, no user data)"""
import os, sys, json, time, hashlib
os.environ.setdefault("LGLN", "native")
import numpy as np
import torch
import torch.nn as nn
sys.path.insert(0, "/mnt/c/Users/a3162/scan3d_handoff/npu_upload/lgnpu")
import lgnpu  # noqa: E402
from lgnpu import LGNpu, L, pack, post, load_net, D  # noqa: E402

torch.set_grad_enabled(False)
OUT = "/mnt/c/Users/a3162/scan3d_handoff/npu_upload/lgpost"
BIG = float(os.environ.get("LGBIG", "30"))
LN01 = float(np.log(0.1))


def att(q, k, v=None, pb=None):
    if v is not None:
        q, k = torch.cat([q, v], -1), torch.cat([k, pb], -1)
    return torch.softmax(torch.bmm(q, k.transpose(1, 2)), -1)


class LGPost(nn.Module):
    def __init__(self, net, nl=6, kind="tanh", masked=False):
        super().__init__()
        self.b = LGNpu(net, kind, nl).eval()
        self.masked = masked
        if masked:
            self.pbl = L(torch.tensor([[BIG]]), torch.tensor([-BIG]))   # pb = BIG*(v-1)
        self.one = L(torch.zeros(1, 1), torch.ones(1))                  # constant 1 column (0*x + 1)

    def forward(self, x):
        b = self.b
        d0, d1 = b.inp(x[..., 0:64]), b.inp(x[..., 64:128])
        c0, s0, c1, s1 = x[..., 128:224], x[..., 224:320], x[..., 320:416], x[..., 416:512]
        if self.masked:
            v = x[..., 512:513]
            v0 = v1 = v
            pb0 = pb1 = self.pbl(v)
        else:
            v0 = v1 = pb0 = pb1 = None
        for sa, ca in zip(b.selfs, b.cross):
            def self_(xx, cs, sn, vv, pp):
                q = sa.q(xx) * cs + sa.qr(xx) * sn
                k = sa.k(xx) * cs + sa.kr(xx) * sn
                a = att(q, k, vv, pp)
                return xx + sa.ffn(xx, sa.o(torch.bmm(a, sa.v(xx))))
            d0, d1 = self_(d0, c0, s0, v0, pb0), self_(d1, c1, s1, v1, pb1)
            q0, q1 = ca.qk(d0), ca.qk(d1)
            w0, w1 = ca.v(d0), ca.v(d1)
            n0 = torch.bmm(att(q0, q1, v0, pb1), w1)
            n1 = torch.bmm(att(q1, q0, v1, pb0), w0)
            d0, d1 = d0 + ca.ffn(d0, ca.o(n0)), d1 + ca.ffn(d1, ca.o(n1))
        m0, m1 = b.fp(d0), b.fp(d1)
        ls0, ls1 = nn.functional.logsigmoid(b.mt(d0)), nn.functional.logsigmoid(b.mt(d1))
        if self.masked:
            m0, m1 = torch.cat([m0, v0, pb0], -1), torch.cat([m1, pb1, v1], -1)
            ls0, ls1 = ls0 + pb0, ls1 + pb1
        S = torch.bmm(m0, m1.transpose(1, 2))
        one0 = self.one(x[..., 0:1])
        E = torch.bmm(torch.cat([ls0, one0], -1), torch.cat([one0, ls1], -1).transpose(1, 2))
        sc = torch.log_softmax(S, 2) + torch.log_softmax(S, 1) + E
        r0 = torch.argmax(sc, 2, keepdim=True).float()            # [1,N,1]
        r1 = torch.argmax(sc, 1, keepdim=True).float().transpose(1, 2)  # [1,N,1]
        rm = torch.amax(sc, 2, keepdim=True)
        return torch.cat([r0, r1, rm], 2)


def data(which, n):
    B = "/home/system/scanner/tmp_npu/data"
    return {k: torch.from_numpy(np.load(f"{B}/lg{which}{n}_{k}.npy")).contiguous() for k in ("kpts0", "kpts1", "desc0", "desc1")}


def finish(o, nvalid=None):
    """CPU O(N) finish on out [1,N,3]."""
    o = o[0]
    N = o.shape[0]
    nv = N if nvalid is None else nvalid
    m0 = np.rint(o[:, 0]).astype(np.int64)
    m1 = np.rint(o[:, 1]).astype(np.int64)
    s = set()
    for i in range(nv):
        j = m0[i]
        if 0 <= j < nv and m1[j] == i and o[i, 2] > LN01:
            s.add((i, int(j)))
    return s


def finish_nomaskguard(o):
    o = o[0]
    m0 = np.rint(o[:, 0]).astype(np.int64)
    m1 = np.rint(o[:, 1]).astype(np.int64)
    return {(i, int(m0[i])) for i in range(len(m0)) if 0 <= m0[i] < len(m0) and m1[m0[i]] == i and o[i, 2] > LN01}


def mset_md5(s):
    a = np.array(sorted(s), np.int32).reshape(-1)
    return hashlib.md5(a.tobytes()).hexdigest()[:12]


def padded(net, a, nv, N, seed=0, padmode="zero"):
    """real first nv points + (N - nv) padding rows; v column appended."""
    x = pack(net, *[t[:, :nv] for t in a])
    g = torch.Generator().manual_seed(seed)
    if padmode == "zero":
        pad = torch.zeros(1, N - nv, 512)
        pad[..., 128:224] = 1.0
        pad[..., 320:416] = 1.0
    else:  # random descriptors / positions (worst case: looks like real points)
        k = torch.rand(1, N - nv, 2, generator=g) * 2 - 1
        dd = torch.nn.functional.normalize(torch.randn(1, N - nv, 64, generator=g), dim=-1)
        dd1 = torch.nn.functional.normalize(torch.randn(1, N - nv, 64, generator=g), dim=-1)
        pad = pack(net, k, k.flip(-1), dd, dd1)
    x = torch.cat([x, pad], 1)
    v = torch.zeros(1, N, 1)
    v[:, :nv] = 1
    return torch.cat([x, v], -1)


def check(net):
    import onnxruntime as ort
    res = dict(made=time.strftime("%F %T"), BIG=BIG, ref="lgnpu L6 (LGLN native, tanh, 6 layers) fp32 out A|BT|z0|z1 + current CPU post (lgnpu.post = nncrun lgpost 1/2 math), lgev 13 pairs")
    d512, d1024 = data("ev", 512), data("ev", 1024)
    ks = ("kpts0", "kpts1", "desc0", "desc1")
    base = LGNpu(net, "tanh", 6).eval()
    p512 = LGPost(net).eval()
    pm = LGPost(net, masked=True).eval()
    os.makedirs(OUT, exist_ok=True)
    # ORT sessions on exported onnx
    sess = {}
    for name, m, N, msk in (("L6p", p512, 512, False), ("L6pm768", pm, 768, True)):
        f = f"{OUT}/{name}.onnx"
        if not os.path.exists(f):
            xx = torch.zeros(1, N, 513 if msk else 512)
            torch.onnx.export(m, (xx,), f, input_names=["x"], output_names=["out"], opset_version=13, do_constant_folding=True, dynamo=False)
        sess[name] = ort.InferenceSession(f)
    rows = []
    for i in range(13):
        a5 = [d512[k][i:i + 1] for k in ks]
        a10 = [d1024[k][i:i + 1] for k in ks]
        x5 = pack(net, *a5)
        ref = post(base(x5).numpy(), 512)
        r = dict(pair=i, n_ref=len(ref), ref_md5=mset_md5(ref))
        o = p512(x5).numpy()
        t = finish(o)
        r.update(L6p_torch=len(ref & t) / max(1, len(ref | t)), L6p_torch_md5=mset_md5(t))
        oo = sess["L6p"].run(None, {"x": x5.numpy()})[0]
        t = finish(oo)
        r.update(L6p_ort=len(ref & t) / max(1, len(ref | t)), L6p_ort_md5=mset_md5(t))
        # masked: 512 real + 256 pad (zero / random) vs ref512 ; 674 real + 94 pad vs base on 674 ; 768 real vs base 768
        for nv, pm_ in ((512, "zero"), (512, "rand"), (674, "zero"), (674, "rand"), (768, "zero")):
            xm = padded(net, a10 if nv > 512 else a5, nv, 768, seed=i, padmode=pm_)
            if nv == 512:
                rf = ref
            else:
                rf = post(base(pack(net, *[t_[:, :nv] for t_ in a10])).numpy(), nv)
            om = sess["L6pm768"].run(None, {"x": xm.numpy()})[0]
            tg = finish(om, nv)
            tn = finish_nomaskguard(om)
            padhit = sum(1 for (p, q) in tn if p >= nv or q >= nv)
            r[f"M{nv}{pm_}"] = dict(agree=len(rf & tg) / max(1, len(rf | tg)), same_md5=mset_md5(rf) == mset_md5(tg),
                                   n_ref=len(rf), n=len(tg), pad_matches_without_cpu_guard=padhit,
                                   pad_rows_max_score=float(om[0, nv:, 2].max()) if nv < 768 else None)
        rows.append(r)
        print(json.dumps(r), flush=True)
    res["rows"] = rows
    res["summary"] = dict(
        L6p_torch_all_identical=all(r["L6p_torch_md5"] == r["ref_md5"] for r in rows),
        L6p_ort_all_identical=all(r["L6p_ort_md5"] == r["ref_md5"] for r in rows),
        masked_all_identical={k: all(r[k]["same_md5"] for r in rows) for k in rows[0] if k.startswith("M")},
        masked_pad_matches_total={k: sum(r[k]["pad_matches_without_cpu_guard"] for r in rows) for k in rows[0] if k.startswith("M")})
    print(json.dumps(res["summary"]))
    json.dump(res, open(f"{OUT}/pc_check.json", "w"), indent=1)


def export(net, name, N, masked, ncal=40):
    """onnx + synthetic random calib/golden inputs (no user data): kpts U[-1,1]^2, desc = unit gaussian 64-d;
    masked: random valid count in [N/2, N], padding rows zero desc, cos 1 sin 0."""
    m = LGPost(net, masked=masked).eval()
    od = f"{OUT}/{name}"
    os.makedirs(od + "/calraw", exist_ok=True)
    g = torch.Generator().manual_seed(1234)
    xs = []
    for i in range(ncal):
        k0, k1 = torch.rand(1, N, 2, generator=g) * 2 - 1, torch.rand(1, N, 2, generator=g) * 2 - 1
        dd0 = torch.nn.functional.normalize(torch.randn(1, N, 64, generator=g), dim=-1)
        dd1 = torch.nn.functional.normalize(torch.randn(1, N, 64, generator=g), dim=-1)
        if masked:
            nv = int(torch.randint(N // 2, N + 1, (1,), generator=g))
            xs.append(padded(net, [k0, k1, dd0, dd1], nv, N, seed=i, padmode="zero"))
        else:
            xs.append(pack(net, k0, k1, dd0, dd1))
    for i, x in enumerate(xs):
        x.numpy().astype(np.float32).tofile(f"{od}/calraw/c{i:03d}.raw")
    f = f"{od}/{name}.onnx"
    torch.onnx.export(m, (xs[0],), f, input_names=["x"], output_names=["out"], opset_version=13, do_constant_folding=True, dynamo=False)
    info = dict(made=time.strftime("%F %T"), input=f"x [1,{N},{xs[0].shape[-1]}]", output=f"out [1,{N},3] = argmax_j, argmax_i, max_j sc",
                masked=masked, BIG=BIG, calib=f"{ncal} synthetic random inputs (seed 1234), no user data",
                onnx_md5=hashlib.md5(open(f, "rb").read()).hexdigest())
    json.dump(info, open(f"{od}/info.json", "w"), indent=1)
    print(info)


if __name__ == "__main__":
    net = load_net()
    if sys.argv[1] == "check":
        check(net)
    elif sys.argv[1] == "stats":  # attention-logit spread to size BIG
        pass
    else:
        export(net, sys.argv[2], int(sys.argv[3]), sys.argv[4] == "1")
