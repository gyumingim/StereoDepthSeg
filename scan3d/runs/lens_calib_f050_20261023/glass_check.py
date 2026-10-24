"""Unit check of glass.refract_points: Snell ratio and coplanarity on random geometry (2026-10-24)."""
import numpy as np
import glass
rng = np.random.default_rng(1)
C = np.c_[rng.uniform(-0.5, 0.5, (1000, 2)), rng.uniform(-1.5, -0.3, 1000)]
P = np.c_[rng.uniform(-0.3, 0.3, (1000, 2)), np.zeros(1000)]
Q = glass.refract_points(P, C, 0.003)
a, b = Q - C, P - Q
s1 = np.hypot(a[:, 0], a[:, 1]) / np.linalg.norm(a, axis=1)
s2 = np.hypot(b[:, 0], b[:, 1]) / np.linalg.norm(b, axis=1)
print("sin1/sin2 min max", (s1 / s2).min(), (s1 / s2).max(), "coplanar max |cross|", np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).max(),
      "Q z", np.unique(Q[:, 2]), "lateral shift mm p50/max", 1000 * np.median(np.linalg.norm((Q - P)[:, :2], axis=1)), 1000 * np.linalg.norm((Q - P)[:, :2], axis=1).max())
