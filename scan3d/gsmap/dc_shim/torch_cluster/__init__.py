"""gsmap 11-27 dc: minimal stand-in for torch_cluster.knn (no wheel for torch 2.11 / cu128). CPU scipy cKDTree, same output layout:
[2, M*k] with row 0 = index into y (query), row 1 = index into x (reference). Batches handled one by one."""
import numpy as np
import torch
from scipy.spatial import cKDTree


def knn(x, y, k, batch_x=None, batch_y=None, cosine=False, num_workers=1):
    dev = x.device
    xn, yn = x.detach().float().cpu().numpy(), y.detach().float().cpu().numpy()
    bx = np.zeros(len(xn), np.int64) if batch_x is None else batch_x.detach().cpu().numpy().astype(np.int64)
    by = np.zeros(len(yn), np.int64) if batch_y is None else batch_y.detach().cpu().numpy().astype(np.int64)
    rows, cols = [], []
    for b in np.unique(by):
        iy = np.nonzero(by == b)[0]; ix = np.nonzero(bx == b)[0]
        kk = min(k, len(ix))
        _, j = cKDTree(xn[ix]).query(yn[iy], k=kk, workers=8)
        j = j.reshape(len(iy), kk)
        rows.append(np.repeat(iy, kk)); cols.append(ix[j].reshape(-1))
    return torch.from_numpy(np.stack([np.concatenate(rows), np.concatenate(cols)])).to(dev)
