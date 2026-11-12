"""TSDF block eviction (fixes 11-12 unit 12): blocks not touched for a while leave GPU memory for CPU RAM (or a disk memmap)
and come back when a keyframe sees them again, so a room / merged scan is not cut at the GPU block cap.

  EvictTSDFMap(gpu_blocks=N, cpu_dtype="f32"|"f16", disk_dir=None, **tsdf_map kwargs)
    * cfg max_blocks = limit on ALL blocks (GPU + CPU); gpu_blocks = how many may sit on the GPU at once.
    * eviction: when a keyframe needs more GPU rows than gpu_blocks, the least recently touched blocks (block version bver =
      last keyframe that changed it) that this keyframe does not use go to the CPU store, plus evict_frac x gpu_blocks extra
      so it does not happen every keyframe. A keyframe that alone needs more than gpu_blocks -> RuntimeError (no silent cut).
    * reload: a block that is on the CPU and is touched again is copied back (same values) and fused as usual -> no duplicate.
    * final mesh (update_mesh(force=True)): all blocks go to the CPU store, then marching cubes runs in groups of 64 x 512
      blocks in allocation order on a small temporary GPU map that holds the group plus its +{0,1}^3 neighbour blocks
      -> same cubes, same 512-block parts and same chunk order as TSDFMap.update_mesh(force=True) on one big map.
      With cpu_dtype f32 the mesh is meant to be bit-identical to TSDFMap (tools/tsdf_evict_check.py checks it).
    * cpu_dtype f16: sdf and w stored as float16 on the CPU (w clamped to 65000) -> 4.0 instead of 6.0 KiB per block, values change
      slightly (lossy; sdf is in +-10 cm, f16 step <= 0.06 mm there).
  Only the offline path is supported (integrate + update_mesh(force=True) + mesh_arrays + stats). Incremental meshing, merge,
  transform, save_npz, voxel_layers, cell_layers raise NotImplementedError.
"""
import mmap
import time
from pathlib import Path

import numpy as np

import tsdf_map
from tsdf_map import BLK, CHUNK, TSDFMap, _pack, _unpack, _unit_offsets

MC_GROUP = 64 * 512          # final mesh: blocks per temporary GPU map (multiple of the 512-block part in TSDFMap._mc)
F16_WMAX = 65000.0


class _CpuStore:
    """Blocks on the CPU: slot arrays + sorted packed keys for lookup. disk_dir -> numpy memmap files (grow = new file + copy)."""

    def __init__(self, dtype="f32", disk_dir=None):
        self.fdt = np.float16 if dtype == "f16" else np.float32
        self.disk = Path(disk_dir) if disk_dir else None
        if self.disk:
            self.disk.mkdir(parents=True, exist_ok=True)
        self.cap = 0
        self.top = 0                                  # slots [0, top) were used once
        self.free = np.zeros(0, np.int64)
        self.spk = np.zeros(0, np.int64)              # sorted packed keys
        self.sslot = np.zeros(0, np.int64)
        self.gen = 0
        self.io_w_s = self.io_r_s = 0.0                # seconds spent writing / reading the store (disk I/O when disk_dir)
        self.a = {}
        self._grow(4096)

    @property
    def n(self):
        return len(self.spk)

    def _mk(self, name, shape, dt):
        if self.disk is None:
            return np.zeros(shape, dt)
        self.gen += 1
        return np.lib.format.open_memmap(self.disk / f"{name}_{self.gen}.npy", mode="w+", dtype=dt, shape=shape)

    def _grow(self, need):
        if need <= self.cap:
            return
        new = max(need, self.cap * 2)
        spec = dict(pk=((), np.int64), birth=((), np.int64), bver=((), np.int64), sdf=((BLK ** 3,), self.fdt),
                    w=((BLK ** 3,), self.fdt), rgb=((BLK ** 3, 3), np.uint8), nobs=((BLK ** 3,), np.uint8))
        for k, (sh, dt) in spec.items():
            b = self._mk(k, (new,) + sh, dt)
            if k in self.a:
                b[:self.cap] = self.a[k][:self.cap]
                old = self.a[k]
                if self.disk is not None:
                    fn = getattr(old, "filename", None)
                    del old
                    self.a[k] = None
                    if fn:
                        Path(fn).unlink(missing_ok=True)
            self.a[k] = b
        self.cap = new

    def _unmap(self):
        """disk store: drop the mapped pages from this process (MADV_DONTNEED on a shared file map keeps the data in the file /
        page cache, which the kernel can write back and free) -> spilled blocks do not count as process RAM."""
        if self.disk is None:
            return
        for v in self.a.values():
            mm = getattr(v, "_mmap", None)
            if mm is not None:
                mm.madvise(mmap.MADV_DONTNEED)

    def nbytes(self):
        return int(sum(v.nbytes for v in self.a.values()))

    def find(self, pk):
        """packed keys (N,) np int64 -> slot or -1."""
        if self.n == 0 or len(pk) == 0:
            return np.full(len(pk), -1, np.int64)
        pos = np.searchsorted(self.spk, pk).clip(max=self.n - 1)
        return np.where(self.spk[pos] == pk, self.sslot[pos], -1)

    def add(self, pk, birth, bver, sdf, w, rgb, nobs):
        m = len(pk)
        if m == 0:
            return
        t0 = time.perf_counter()
        nf = min(m, len(self.free))
        slots = np.concatenate([self.free[len(self.free) - nf:], np.arange(self.top, self.top + m - nf, dtype=np.int64)])
        self.free = self.free[:len(self.free) - nf]
        self._grow(self.top + m - nf)
        self.top += m - nf
        o = np.argsort(slots)                          # write in slot order (memmap friendly)
        s = slots[o]
        A = self.a
        A["pk"][s], A["birth"][s], A["bver"][s] = pk[o], birth[o], bver[o]
        if self.fdt == np.float16:
            A["sdf"][s] = sdf[o].astype(np.float16)
            A["w"][s] = np.minimum(w[o], F16_WMAX).astype(np.float16)
        else:
            A["sdf"][s], A["w"][s] = sdf[o], w[o]
        A["rgb"][s], A["nobs"][s] = rgb[o], nobs[o]
        ins = np.searchsorted(self.spk, pk)
        self.spk = np.insert(self.spk, ins, pk)
        self.sslot = np.insert(self.sslot, ins, slots)
        oo = np.argsort(self.spk, kind="stable")       # np.insert with unsorted pk keeps order only per insert point
        if not np.all(oo == np.arange(len(oo))):
            self.spk, self.sslot = self.spk[oo], self.sslot[oo]
        self._unmap()
        self.io_w_s += time.perf_counter() - t0

    def take(self, slots):
        """read slots and drop them from the store -> dict of numpy arrays (sdf/w as float32)."""
        t0 = time.perf_counter()
        s = np.asarray(slots, np.int64)
        A = self.a
        out = dict(pk=A["pk"][s].copy(), birth=A["birth"][s].copy(), bver=A["bver"][s].copy(),
                   sdf=np.asarray(A["sdf"][s], np.float32), w=np.asarray(A["w"][s], np.float32),
                   rgb=A["rgb"][s].copy(), nobs=A["nobs"][s].copy())
        pos = np.searchsorted(self.spk, out["pk"])
        keep = np.ones(self.n, bool); keep[pos] = False
        self.spk, self.sslot = self.spk[keep], self.sslot[keep]
        self.free = np.concatenate([self.free, s])
        self._unmap()
        self.io_r_s += time.perf_counter() - t0
        return out

    def read(self, slots):
        t0 = time.perf_counter()
        s = np.asarray(slots, np.int64)
        A = self.a
        out = dict(pk=A["pk"][s], sdf=np.asarray(A["sdf"][s], np.float32), w=np.asarray(A["w"][s], np.float32),
                   rgb=A["rgb"][s].copy(), nobs=A["nobs"][s].copy())
        self._unmap()
        self.io_r_s += time.perf_counter() - t0
        return out

    def disk_bytes(self):
        """spill bytes really on disk (allocated blocks; the .npy files are sized for the whole capacity, unused tail is sparse)."""
        return int(sum(f.stat().st_blocks * 512 for f in self.disk.glob("*.npy"))) if self.disk else 0

    def close(self, delete=True):
        """drop the arrays; delete=True removes the spill files (and the folder if it is then empty)."""
        files = [getattr(v, "filename", None) for v in self.a.values()]
        self.a = {}
        if self.disk is not None and delete:
            for fn in files:
                if fn:
                    Path(fn).unlink(missing_ok=True)
            try:
                self.disk.rmdir()
            except OSError:
                pass

    def all_slots_by_birth(self):
        s = self.sslot
        return s[np.argsort(self.a["birth"][s], kind="stable")]


class EvictTSDFMap(TSDFMap):
    def __init__(self, gpu_blocks, cpu_dtype="f32", disk_dir=None, evict_frac=0.25, **kw):
        if cpu_dtype not in ("f32", "f16"):
            raise ValueError(f"cpu_dtype {cpu_dtype!r} (f32|f16)")
        self.gpu_budget = int(gpu_blocks)
        if self.gpu_budget < 1024:
            raise ValueError("gpu_blocks must be >= 1024")
        self.evict_frac = float(evict_frac)
        self._birth_next = 0
        super().__init__(**kw)
        self.cpu = _CpuStore(cpu_dtype, disk_dir)
        self.cpu_dtype = cpu_dtype
        self.ev = dict(evict_events=0, evicted=0, reloaded=0, evict_s=0.0, reload_s=0.0, gpu_peak_blocks=0, cpu_peak_blocks=0,
                       cpu_peak_bytes=0, mesh_groups=0, mesh_s=0.0)

    @property
    def n_total(self):
        return int(self.nb + self.cpu.n)

    # ---------------- GPU store (base + birth, growth capped at the budget) ----------------
    def _alloc(self, cap):
        import torch
        super()._alloc(min(cap, self.gpu_budget))
        self.birth = torch.zeros(self.cap, dtype=torch.int64, device=self.dev)

    def _grow(self, need):
        import torch
        if need <= self.cap:
            return
        if need > self.gpu_budget:
            raise RuntimeError(f"[tsdf-evict] internal: {need} GPU rows > budget {self.gpu_budget}")
        new = min(max(need, self.cap * 2), self.gpu_budget)
        for name in ("keys", "sdf", "w", "rgb", "nobs", "bver", "birth"):
            a = getattr(self, name)
            b = torch.zeros((new,) + tuple(a.shape[1:]), dtype=a.dtype, device=a.device)
            b[:self.cap] = a
            setattr(self, name, b)
        self.cap = new

    def _resort(self):
        import torch
        sk, order = torch.sort(_pack(self.keys[:self.nb]))
        self._sk, self._si = sk, order

    def _evict_rows(self, ev):
        """GPU rows ev (1-D tensor, unique) -> CPU store; tail rows move into the holes."""
        import torch
        t0 = time.perf_counter()
        ev = torch.sort(ev).values
        m = len(ev)
        if m == 0:
            return
        self.cpu.add(_pack(self.keys[ev]).cpu().numpy(), self.birth[ev].cpu().numpy(), self.bver[ev].cpu().numpy(),
                     self.sdf[ev].cpu().numpy(), self.w[ev].cpu().numpy(), self.rgb[ev].cpu().numpy(), self.nobs[ev].cpu().numpy())
        newnb = self.nb - m
        holes = ev[ev < newnb]
        gone = torch.zeros(self.nb - newnb, dtype=torch.bool, device=self.dev)
        gone[ev[ev >= newnb] - newnb] = True
        tail = torch.nonzero(~gone).squeeze(1) + newnb
        assert len(holes) == len(tail)
        if len(holes):
            for name in ("keys", "sdf", "w", "rgb", "nobs", "bver", "birth"):
                a = getattr(self, name)
                a[holes] = a[tail]
        self.nb = newnb
        self._resort()
        self.ev["evict_events"] += 1
        self.ev["evicted"] += m
        self.ev["cpu_peak_blocks"] = max(self.ev["cpu_peak_blocks"], self.cpu.n)
        self.ev["cpu_peak_bytes"] = max(self.ev["cpu_peak_bytes"], self.cpu.nbytes())
        self.ev["evict_s"] += time.perf_counter() - t0

    def _activate(self, bkeys):
        """as TSDFMap._activate; blocks on the CPU are reloaded (not new); max_blocks counts GPU + CPU blocks."""
        import torch
        pk = _pack(bkeys)
        u, inv = torch.unique(pk, return_inverse=True)
        rows = self._find(u)
        miss = rows < 0
        if not bool(miss.any()):
            return rows[inv]
        um = u[miss]
        slots = self.cpu.find(um.cpu().numpy())
        incpu = torch.as_tensor(slots >= 0, device=self.dev)
        new = ~incpu
        nnew = int(new.sum())
        room = int(self.cfg["max_blocks"]) - self.n_total
        if nnew > room:
            if not self.full:
                self.log(f"[tsdf] 블록 한도 {self.cfg['max_blocks']} — 새 영역은 더 안 넣음")
            self.full = True
            idx = torch.nonzero(new).squeeze(1)[:max(room, 0)]
            new = torch.zeros_like(new); new[idx] = True
            nnew = len(idx)
        rl_slots = slots[slots >= 0]
        nrl = len(rl_slots)
        need = self.nb + nrl + nnew
        if need > self.gpu_budget:
            hit = rows[~miss]
            prot = torch.zeros(self.nb, dtype=torch.bool, device=self.dev)
            prot[hit] = True
            cand = torch.nonzero(~prot).squeeze(1)
            short = need - self.gpu_budget
            if len(cand) < short:
                raise RuntimeError(f"[tsdf-evict] one keyframe needs {len(hit) + nrl + nnew} blocks > GPU budget "
                                   f"{self.gpu_budget} (raise --tsdf-evict-gpu-blocks)")
            k = min(len(cand), short + int(self.evict_frac * self.gpu_budget))
            order = torch.argsort(self.bver[cand], stable=True)       # least recently changed first
            self._evict_rows(cand[order[:k]])
        # reload
        if nrl:
            t0 = time.perf_counter()
            d = self.cpu.take(rl_slots)
            r = torch.arange(self.nb, self.nb + nrl, device=self.dev)
            self._grow(self.nb + nrl)
            self.keys[r] = _unpack(torch.as_tensor(d["pk"], device=self.dev))
            self.birth[r] = torch.as_tensor(d["birth"], device=self.dev)
            self.bver[r] = torch.as_tensor(d["bver"], device=self.dev)
            self.sdf[r] = torch.as_tensor(d["sdf"], device=self.dev)
            self.w[r] = torch.as_tensor(d["w"], device=self.dev)
            self.rgb[r] = torch.as_tensor(d["rgb"], device=self.dev)
            self.nobs[r] = torch.as_tensor(d["nobs"], device=self.dev)
            self.nb += nrl
            self.ev["reloaded"] += nrl
            self.ev["reload_s"] += time.perf_counter() - t0
        if nnew:
            self._grow(self.nb + nnew)
            r = torch.arange(self.nb, self.nb + nnew, device=self.dev)
            self.keys[r] = _unpack(um[new])
            self.birth[r] = torch.arange(self._birth_next, self._birth_next + nnew, device=self.dev)
            self._birth_next += nnew
            for name in ("sdf", "w", "rgb", "nobs", "bver"):     # rows freed by eviction hold old data
                getattr(self, name)[r] = 0
            self.nb += nnew
        if nrl or nnew:
            self._resort()
        self.ev["gpu_peak_blocks"] = max(self.ev["gpu_peak_blocks"], self.nb)
        return self._find(u)[inv]

    # ---------------- final mesh over GPU + CPU blocks ----------------
    def update_mesh(self, force=False, min_interval=0.0):
        import torch
        with self.lock:
            if not force:
                if self.version == self._mesh_at:
                    return 0
                raise NotImplementedError("EvictTSDFMap: only the final full mesh (update_mesh(force=True))")
            t0 = time.perf_counter()
            if self.nb:
                self._evict_rows(torch.arange(self.nb, device=self.dev))
            self._alloc(1024)                                        # free the GPU store
            if self.dev.type == "cuda":
                torch.cuda.empty_cache()
            order = self.cpu.all_slots_by_birth()
            out = {}
            unit = _unit_offsets(self.dev)
            ng = 0
            for a in range(0, len(order), MC_GROUP):
                gs = order[a:a + MC_GROUP]
                gk = _unpack(torch.as_tensor(self.cpu.a["pk"][gs], device=self.dev))
                nk = torch.unique(_pack((gk[:, None, :] + unit[None]).reshape(-1, 3)))
                ns = self.cpu.find(nk.cpu().numpy())
                ns = ns[ns >= 0]
                d = self.cpu.read(np.sort(ns))
                tmp = TSDFMap(device=self.dev, **dict(self.cfg, max_blocks=len(ns) + 1))
                tmp._alloc(max(1024, len(ns)))
                n = len(ns)
                tmp.keys[:n] = _unpack(torch.as_tensor(d["pk"], device=self.dev))
                tmp.sdf[:n] = torch.as_tensor(d["sdf"], device=self.dev)
                tmp.w[:n] = torch.as_tensor(d["w"], device=self.dev)
                tmp.rgb[:n] = torch.as_tensor(d["rgb"], device=self.dev)
                tmp.nobs[:n] = torch.as_tensor(d["nobs"], device=self.dev)
                tmp.nb = n
                sk, oo = torch.sort(_pack(tmp.keys[:n]))
                tmp._sk, tmp._si = sk, oo
                rows = tmp._find(_pack(gk))                         # group rows in allocation (birth) order
                assert bool((rows >= 0).all())
                res = tmp._mc(rows)
                for k, (vv, cc, ff) in res.items():                 # same join as inside TSDFMap._mc across 512-block parts
                    prev = out.get(k)
                    if prev is not None:
                        ff = ff + len(prev[0])
                        vv, cc, ff = np.concatenate([prev[0], vv]), np.concatenate([prev[1], cc]), np.concatenate([prev[2], ff])
                    out[k] = (vv, cc, ff)
                del tmp
                ng += 1
            self._mesh_ver += 1
            self._mesh = {}
            for k in sorted(out):                                   # TSDFMap: dirty chunks in sorted packed-key order
                v = out[k]
                self._mesh[k] = dict(v=v[0], c=v[1], f=v[2], ver=self._mesh_ver)
            self._mesh_at = self.version
            self._mesh_t = time.monotonic()
            self._mesh_ms = 1000 * (time.perf_counter() - t0)
            self.ev["mesh_groups"] = ng
            self.ev["mesh_s"] = round(time.perf_counter() - t0, 2)
            return len(self._mesh)

    def evict_stats(self):
        e = dict(self.ev)
        e.update(store_write_s=round(self.cpu.io_w_s, 2), store_read_s=round(self.cpu.io_r_s, 2),
                 spill_bytes=self.cpu.disk_bytes(), spill_gb=round(self.cpu.disk_bytes() / 2 ** 30, 3))
        e.update(gpu_budget_blocks=self.gpu_budget, cpu_dtype=self.cpu_dtype, disk=str(self.cpu.disk) if self.cpu.disk else None,
                 blocks_total=self.n_total, blocks_gpu=int(self.nb), blocks_cpu=int(self.cpu.n),
                 evict_s=round(e["evict_s"], 2), reload_s=round(e["reload_s"], 2),
                 cpu_peak_gb=round(e["cpu_peak_bytes"] / 2 ** 30, 3),
                 gpu_store_gb=round(self.gpu_budget * BLK ** 3 * (4 + 4 + 3 + 1) / 2 ** 30, 3))
        return e

    def stats(self):
        s = super().stats()
        s.update(blocks=self.n_total, evict=self.evict_stats())
        return s

    def _no(self, *a, **k):
        raise NotImplementedError("EvictTSDFMap: offline integrate + final mesh only")

    mesh_bytes = save_npz = voxel_layers = cell_layers = merge = transform = _no
