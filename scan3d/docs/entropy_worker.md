# entropy = second VIO worker PC (2026-10-23)

entropy (Ryzen 7 8845HS, Zen 4, same AVX-512 flags as this PC's 7800X3D; WSL Ubuntu-22.04 sees 16 threads / ~7.5 GB RAM)
runs the VIO bench (imu_learn/vio_bench_hs.py through the runners) with the SAME absolute paths as here
(`/home/system/scanner/...`, user `system`, uid 1000, passwordless sudo, default WSL user).
Access: Tailscale `a3162@100.91.12.2` (Windows OpenSSH, key auth, cmd.exe shell) -> `wsl -d Ubuntu-22.04 -u system`.

## What is there
| part | how | same as here? |
|---|---|---|
| apt build deps | libopencv-dev 4.5.4+dfsg-9ubuntu4, libeigen3-dev 3.4.0-2ubuntu2, Boost 1.74, liblapack/libblas 3.10.0 (reference), gcc 11.4, cmake 3.22.1 | same versions |
| Ceres | rebuilt from `~/scanner/src/ceres-solver` -> `~/scanner/local` (static, miniglog, EIGENSPARSE, LAPACK, no SuiteSparse/gflags) | same options |
| OpenVINS | rebuilt `~/scanner/src/ovbuild` from `~/scanner/src/open_vins` (Release, ROS off, ARUCO on) | same source/options |
| bridge | rebuilt `scan3d/ov_bridge/build/ov_bridge` (stock = base variant) | yes |
| other bridges | sources copied (`open_vins_*`, `tools/ov_bridge_*`) but NOT built (zbase, zuptA..., rs, diag) | build when needed |
| python | uv CPython 3.12.15 copied; `venv_ffs` SLIM: only numpy 2.5.2, opencv(-contrib)-python 5.0.0.93 (cv2 files copied as is), scipy 1.18.1 | bench needs only cv2+numpy |
| code | repo working tree (incl. untracked) minus venv, recordings, runs, ov_bridge/android, phone_stereo, *.pt | push before each run |
| recordings | 15 test recordings (phone60 RAW+OLD): csv, meta, imu_gt, imu_gt_spl only (no png/pkl) | 0.65 GB |
| videos / frame caches | NOT copied (C: space). Only `hs_20261022_134550` has its current-lens o0 cache (`frames_640x360_eefb9e615b1c3348`, 0.88 GB) | |

`cam5.mp4` of 134550 on entropy is a PLACEHOLDER (true size + true first/last 8 MiB, zero hole; marker
`cam5.mp4.PLACEHOLDER`): the frame-cache key only reads size + head/tail, so the cache hits, but the video cannot
be decoded. Its cache dir is read-only, so a cache MISS fails loudly instead of writing garbage frames.
A cell needs, per recording: the cache dir for the current lens key (or the real mp4). To add one: copy
`recordings/<rec>/cache/frames_640x360_<key>/` (key = `FrameCache(...).key`, see frame_cache.py) + make the
placeholder as above, or copy the real `cam5.mp4` (1-10 GB each, 60 GB for all 15).

## Bit-identity check (2026-10-23)
Cell `lens_switch_ab.py job 134550_o0_new` (config/mono + try_zupt=false, stock bridge, --lever, new 2.22 lens,
frame cache) on entropy vs `runs/lens_switch/134550_o0_new` here: **3809/3809 poses identical** (all fields except
the timing fields ms/wall_ms), ate_cm 1.34 / sim3 1.27 / scale 0.9681 equal. JSON differs only in git_head,
bench_dirty, imu_gt_blob (this PC's tree has moved on), timing. Job time 23.8 s there vs 56.4 s here (loaded).
Only one cell checked; re-check one cell after any rebuild/apt upgrade on either PC.

## Run cells there (from this PC's WSL, cwd scan3d)
```bash
cat > /tmp/cmds.txt <<'EOF'
../venv_ffs/bin/python imu_learn/lens_switch_ab.py job 134550_o0_new
EOF
tools/remote_entropy.sh run mytag /tmp/cmds.txt -j 8     # pushes code first, blocks until done
tools/remote_entropy.sh status mytag                      # cmds / done / failed (rc != 0) + last lines
tools/remote_entropy.sh pull mytag runs/lens_switch       # files written since start -> same paths here (OVERWRITES)
tools/remote_entropy.sh sh 'ls runs/lens_switch'          # any one-off command
```
- one shell command per line, run in scan3d with `nice -n 5`; logs `~/scanner/entropy_jobs/<tag>/NNN.log`, `.rc`.
- `run` keeps the ssh session open until the jobs end; for long batches start it with `nohup ... &` locally.
  If the session drops, the remote jobs may stop with it (not tested) - check status and rerun missing cells.
- RAM: 7.5 GB in WSL. One bench cell is light, but building C++ with `-j6` hung WSL (OOM); use `make -j3`.
  If WSL hangs (`Wsl/Service/WSAETIMEDOUT`): `ssh a3162@100.91.12.2 "wsl --shutdown"` and retry.
- pull overwrites a same-named local result; use a different runner OUT dir/tag if the local one must stay.

## Space (entropy C:, check with `fsutil volume diskfree c:` - WSL df is not real free space)
C: free 34.4 GB before this setup -> 31.3 GB after (WSL disk ~7.4 GB used inside, incl. Ubuntu base + apt ~4 GB,
sources+builds 1.7 GB, repo+slim venv 0.8 GB, recordings 0.65 GB + one cache 0.88 GB).
The ext4.vhdx does not shrink by itself after deletes (compacting needs admin: `wsl --shutdown` then
`Optimize-VHD` or `wsl --manage Ubuntu-22.04 --set-sparse true`).

## Setup scripts used (re-runnable)
build: ceres `cmake -DBUILD_SHARED_LIBS=OFF -DGFLAGS=OFF -DMINIGLOG=ON -DSUITESPARSE=OFF -DEIGENSPARSE=ON -DLAPACK=ON
-DUSE_CUDA=OFF -DCMAKE_INSTALL_PREFIX=~/scanner/local`, ovbuild `cmake -S open_vins/ov_msckf -DENABLE_ROS=OFF
-DENABLE_ARUCO_TAGS=ON -DCMAKE_PREFIX_PATH=~/scanner/local; make ov_msckf_lib`, bridge `cmake -S scan3d/ov_bridge`.
Compiler `/usr/lib/ccache/g++` (= g++-11; this PC uses /usr/lib/ccache/c++ -> same g++-11).
