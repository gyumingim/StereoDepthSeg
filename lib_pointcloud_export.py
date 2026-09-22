"""Export a single colored point cloud; no object tracking or accumulation."""
from pathlib import Path
import numpy as np


def write_ply(path, points, colors):
    with Path(path).open('w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex '+str(len(points))+'\n')
        f.write('property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p, c in zip(points, colors):
            f.write(' '.join([*(f'{v:.5f}' for v in p), *(str(int(v)) for v in np.clip(c, 0, 255))])+'\n')
