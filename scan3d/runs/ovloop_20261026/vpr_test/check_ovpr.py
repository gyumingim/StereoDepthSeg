import struct, sys, numpy as np
def load(p):
    b = open(p, "rb").read(); assert b[:4] == b"OVPR"; dim = struct.unpack("<I", b[4:8])[0]
    rec = 8 + 4 * dim; n = (len(b) - 8) // rec; assert 8 + n * rec == len(b)
    ts = []; V = []
    for i in range(n):
        o = 8 + i * rec; ts.append(struct.unpack("<q", b[o:o+8])[0]); V.append(np.frombuffer(b[o+8:o+rec], "<f4"))
    return dim, ts, np.array(V)
for p in sys.argv[1:]:
    dim, ts, V = load(p)
    print(p, "dim", dim, "n", len(ts), "sorted", ts == sorted(ts), "norms", np.round(np.linalg.norm(V, axis=1), 4), "finite", np.isfinite(V).all())
    np.set_printoptions(precision=3, suppress=True); print(V @ V.T)
if len(sys.argv) == 3:
    _, _, A = load(sys.argv[1]); _, _, B = load(sys.argv[2]); print("cos(A_i,B_i)", np.round((A*B).sum(1), 4))
