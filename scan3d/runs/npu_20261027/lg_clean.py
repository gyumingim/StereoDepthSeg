#!/usr/bin/env python3
"""npu 2026-10-27: LighterGlue (kornia LightGlue, xfeat conf: dim 96, 1 head, 6 layers, no pruning / early stop) rewritten
with only mobile-friendly ops (3-D tensors [1,N,C], Linear, BMM, Softmax, LayerNorm, GELU(tanh), cos/sin, concat) so that
it converts to TFLite and can run on the GPU / NNAPI delegate.
 - rotary: q/k channels permuted (pairs (2i,2i+1) -> (i,i+48)); same permutation on q and k leaves q.k unchanged;
   rotate_half -> concat(-x2, x1), repeat_interleave(cos) -> concat(cos, cos)  (exact)
 - final log assignment: graph outputs P = softmax_row(sim) * softmax_col(sim) and z0, z1 (matchability logits);
   CPU: score = log P + logsigmoid(z0) + logsigmoid(z1)^T  (== kornia scores0 + scores1 + certainties), then max/argmax
 - GELU: tanh approximation if GELU_TANH=1 (else exact erf)
Usage as module: build(net) -> LGClean;  python3 lg_clean.py OUTDIR  -> exports lgc_{N}.onnx + checks vs kornia."""
import os, sys, time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
torch.set_grad_enabled(False)
D = 96
PERM = list(range(0, D, 2)) + list(range(1, D, 2))


def lin(l, rows=None):
    w, b = l.weight.data, (l.bias.data if l.bias is not None else None)
    if rows is not None:
        w = w[rows]
        b = b[rows] if b is not None else None
    n = nn.Linear(w.shape[1], w.shape[0], bias=b is not None)
    n.weight.data.copy_(w)
    if b is not None:
        n.bias.data.copy_(b)
    return n


class FFN(nn.Module):
    def __init__(self, seq, tanh):
        super().__init__()
        self.l1, self.ln, self.l2 = seq[0], seq[1], seq[3]
        self.tanh = tanh

    def forward(self, x):
        return self.l2(F.gelu(self.ln(self.l1(x)), approximate="tanh" if self.tanh else "none"))


class Self_(nn.Module):
    def __init__(self, b, tanh):
        super().__init__()
        q = [d * 3 + 0 for d in range(D)]
        k = [d * 3 + 1 for d in range(D)]
        v = [d * 3 + 2 for d in range(D)]
        self.q = lin(b.Wqkv, [q[i] for i in PERM])
        self.k = lin(b.Wqkv, [k[i] for i in PERM])
        self.v = lin(b.Wqkv, v)
        self.o = b.out_proj
        self.ffn = FFN(b.ffn, tanh)

    def forward(self, x, cs, sn):
        def rot(t):
            return t * cs + torch.cat([-t[..., D // 2:], t[..., :D // 2]], -1) * sn
        q = rot(self.q(x)) * (D ** -0.5)
        k = rot(self.k(x))
        a = torch.softmax(torch.bmm(q, k.transpose(1, 2)), -1)
        m = self.o(torch.bmm(a, self.v(x)))
        return x + self.ffn(torch.cat([x, m], -1))


class Cross_(nn.Module):
    def __init__(self, b, tanh):
        super().__init__()
        self.qk, self.v, self.o = b.to_qk, b.to_v, b.to_out
        self.s = b.scale ** 0.5
        self.ffn = FFN(b.ffn, tanh)

    def forward(self, x0, x1):
        q0, q1 = self.qk(x0) * self.s, self.qk(x1) * self.s
        v0, v1 = self.v(x0), self.v(x1)
        sim = torch.bmm(q0, q1.transpose(1, 2))
        m0 = torch.bmm(torch.softmax(sim, -1), v1)
        m1 = torch.bmm(torch.softmax(sim.transpose(1, 2), -1), v0)
        m0, m1 = self.o(m0), self.o(m1)
        return x0 + self.ffn(torch.cat([x0, m0], -1)), x1 + self.ffn(torch.cat([x1, m1], -1))


class LGClean(nn.Module):
    def __init__(self, net, tanh=False):
        super().__init__()
        self.inp = net.input_proj
        self.wr = net.posenc.Wr
        self.selfs = nn.ModuleList([Self_(t.self_attn, tanh) for t in net.transformers])
        self.cross = nn.ModuleList([Cross_(t.cross_attn, tanh) for t in net.transformers])
        la = net.log_assignment[net.conf.n_layers - 1]
        self.fp, self.mt = la.final_proj, la.matchability

    def forward(self, k0, k1, d0, d1):
        p0, p1 = self.wr(k0), self.wr(k1)
        c0, s0 = torch.cos(p0), torch.sin(p0)
        c1, s1 = torch.cos(p1), torch.sin(p1)
        c0, s0, c1, s1 = (torch.cat([t, t], -1) for t in (c0, s0, c1, s1))
        d0, d1 = self.inp(d0), self.inp(d1)
        for sa, ca in zip(self.selfs, self.cross):
            d0, d1 = sa(d0, c0, s0), sa(d1, c1, s1)
            d0, d1 = ca(d0, d1)
        md0, md1 = self.fp(d0) * (D ** -0.25), self.fp(d1) * (D ** -0.25)
        sim = torch.bmm(md0, md1.transpose(1, 2))
        P = torch.softmax(sim, 2) * torch.softmax(sim, 1)
        return P, self.mt(d0), self.mt(d1)


def post(P, z0, z1):
    """numpy CPU post: same outputs as ovxfeat LGFixed (m0, m1, s0)."""
    ls = lambda z: -np.logaddexp(0, -z)
    sc = np.log(np.maximum(P[0].astype(np.float64), 1e-30)) + ls(z0[0].astype(np.float64)) + ls(z1[0, :, 0].astype(np.float64))[None, :]
    return sc.argmax(1), sc.argmax(0), np.exp(sc.max(1))


def load_net():
    from modules.lighterglue import LighterGlue
    return LighterGlue(weights=os.path.join(XF_SRC, "weights", "xfeat-lighterglue.pt")).net.cpu().eval()


def kornia_ref(net, k0, k1, d0, d1):
    for att in net.modules():
        if hasattr(att, "enable_flash"):
            att.enable_flash = True
    d0, d1 = net.input_proj(d0), net.input_proj(d1)
    e0, e1 = net.posenc(k0), net.posenc(k1)
    for i in range(net.conf.n_layers):
        d0, d1 = net.transformers[i](d0, d1, e0, e1)
    scores, _ = net.log_assignment[net.conf.n_layers - 1](d0, d1)
    s = scores[:, :-1, :-1]
    return s.max(2).indices[0].numpy(), s.argmax(1)[0].numpy(), s.max(2).values.exp()[0].numpy()


if __name__ == "__main__":
    OUT = sys.argv[1]
    os.makedirs(OUT, exist_ok=True)
    net = load_net()
    info = []
    for tanh in (False, True):
        m = LGClean(net, tanh).eval()
        for N in [512, 1024]:
            g = torch.Generator().manual_seed(N)
            k0, k1 = torch.rand(1, N, 2, generator=g) * 2 - 1, torch.rand(1, N, 2, generator=g) * 2 - 1
            d0 = F.normalize(torch.randn(1, N, 64, generator=g), dim=-1)
            d1 = F.normalize(torch.randn(1, N, 64, generator=g), dim=-1)
            r = kornia_ref(net, k0, k1, d0, d1)
            c = post(*[t.numpy() for t in m(k0, k1, d0, d1)])
            info.append(f"tanh={tanh} N={N} random-input m0 agree {np.mean(r[0]==c[0]):.4f} m1 {np.mean(r[1]==c[1]):.4f} s0 maxerr {np.abs(r[2]-c[2]).max():.2e}")
            print(info[-1], flush=True)
            fn = os.path.join(OUT, f"lgc{'t' if tanh else ''}_{N}.onnx")
            torch.onnx.export(m, (k0, k1, d0, d1), fn, input_names=["kpts0", "kpts1", "desc0", "desc1"],
                              output_names=["P", "z0", "z1"], opset_version=17, dynamo=False)
    open(os.path.join(OUT, "lgc_info.txt"), "w").write(time.strftime("%F %T") + "\n" + "\n".join(info) + "\n")
