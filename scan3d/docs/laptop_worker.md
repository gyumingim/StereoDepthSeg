# laptop = offline scanner worker PC (2026-11-11)

HP Victus 15 (Ryzen 7 8845HS 16 threads, RTX 4060 Laptop 8 GB, driver 592.82, 15 GB RAM -> WSL sees 7.6 GB), Windows 11.
WSL 2.6.3 (same version as this PC), Ubuntu-22.04, user `system` (uid 1000, passwordless sudo, default user), systemd on,
`%USERPROFILE%\.wslconfig` = `instanceIdleTimeout=-1`, `vmIdleTimeout=-1`. Same absolute paths as this PC (`/home/system/scanner/...`).
Access from this PC's WSL: `ssh a3162@169.254.78.233` (LAN cable, ~100 MB/s) or `a3162@192.168.1.76` (Wi-Fi) -> `wsl -d Ubuntu-22.04 -u system`.
No GUI, nothing opens windows on the laptop.

## What is there (copied from this PC, not rebuilt)
| part | how | same as here? |
|---|---|---|
| apt | runtime libs only: libopencv 4.5.4+dfsg-9ubuntu4, boost-filesystem 1.74, lapack/blas 3.10.0 reference, tbb2, libgl1, build-essential (triton) | same versions |
| python | uv CPython 3.12.15 + `venv_ffs` copied as is (torch 2.11.0+cu128, pycolmap-cuda12 4.2.1, open3d 0.20, cv2 5.0.0, numpy 2.5.2) | same files |
| FFS | `~/scanner/StereoDepthSeg/tools/Fast-FoundationStereo` incl. weights `c/` | same files |
| VIO | `~/scanner/build_livepose/bridge/ov_bridge` (sha 55e5d35b) + `cfg_gen/c4f60` (imucam 58b9a1af, remap 483168c9) | same sha |
| code | `~/scanner/StereoDepthSeg` (main tree minus venv/recordings/runs/android/phone_stereo/.git/*.pt) + worktrees pushed by the script | push before each job |
| recordings | only while a job runs (the script deletes them after) | |
Disk: WSL ext4.vhdx 13.3 GB (venv 8.8 GB). Windows C: free 190.9 -> 163.4 GB (the rest = WSL package, feature install, reboots; not split).
Setup = WSL + VirtualMachinePlatform features + WSL MSI + 2 reboots (user rebooted), apt as root, `tar | ssh` of the dirs above.

## Use (from this PC's WSL, any worktree that has the script)
```bash
cd ~/scanner/StereoDepthSeg-laptop/scan3d
tools/remote_laptop.sh job 165506 all                 # vio -> kf_ba (SIFT, seed 0) -> FFS+TSDF mesh; results -> runs/laptop/20261024_165506_std60/
tools/remote_laptop.sh job 165506 kfba --vio runs/x/B_o0.poses.jsonl.gz --tag mytry     # one step on an input from here (sent automatically)
tools/remote_laptop.sh job 165506 mesh --poses runs/x/poses_ba.jsonl.gz --args '--free-scale'
WT=~/scanner/StereoDepthSeg-boardscale tools/remote_laptop.sh job 165506 board --poses <abs path>/poses_ba.jsonl.gz
tools/remote_laptop.sh job 165506 'cmd:$P scan3d/scanA/kf_ba.py $R --out $O/x ...'     # any command; $P $R $O $G $CACHE set
tools/remote_laptop.sh status | pull RID[/TAG] | sh 'cmd' | push [WT] | unlock
```
- One job at a time: mkdir lock `~/scanner/laptop_gpu.lock.d` on the laptop (holder = host:pid:time:job). A second job waits (polls 60 s, up to
  `LOCK_WAIT_S`, default 6 h). A lock whose holder pid on this PC is gone is removed automatically. Tested with two jobs at once (B waited for A).
- A job: lock -> push code -> send recording (no `cache`/`scanA_cache`, 542 MB for 165506, ~6 s) -> run under `setsid` (survives ssh drop;
  plain `nohup &` was killed when the wsl.exe session ended) -> poll rc -> pull `runs/laptop/<RID>[/<TAG>]` -> delete recording + kf_ba cache
  -> unlock. `--keep` keeps the recording. Logs there: `~/scanner/laptop_jobs/<job id>/job.log`, copy pulled as `job_<id>.log`.
- Pull overwrites same-named files here. Binary streams must not go through `tr -d '\0'` (it corrupted the first pull).
- `wsl.exe` expands `$VAR` in command lines given to it: send scripts on stdin (`bash -s`) as the script does, not `bash -lc '...$x...'`.
- RAM in WSL 7.6 GB: mesh peaks 3.5 GB, kf_ba 1.7 GB. Keep one job at a time.

## Same results as this PC? (165506 std60, 11-11, code 7384db5 = featloop-20261105; reference = rerun here the same day, same commands)
| step | laptop vs this PC | verdict |
|---|---|---|
| VIO c4f60 (lp_replay) | 1690/1690 pose rows identical in every non-timing field; also identical to scanA step1 B_o0 (11-05). ATE 1.70 cm | **bit-identical** |
| kf_ba CPU part (`--db-from` the same DB: verify seed 0 -> tracks -> BA -> Sim3) | counts identical, poses max abs diff 3.6e-12 (here: two reruns bit-identical) | identical to ~1e-12 |
| kf_ba full (GPU SIFT) | **this PC is not repeatable**: 3 runs scale k 1.01082 / 1.01027 / 1.00902, points 8328 / 8244 / 8421, obs 38073 / 39017 / 41771 (keypoints differ in 45/139 images, images bit-identical). Laptop: 3 runs bit-identical, k 1.01092, points 8617, obs 43188 | k inside the spread (0.01 % from run 1, spread 0.18 %); points/obs 2-3 % above this PC's 3-run max — spread-level, n = 3 |
| FFS + TSDF mesh, same BA poses | not bitwise: |dZ| p90 median 1.8989 vs 1.8998 cm, mesh RMS 0.1364 vs 0.1373 cm, cover 0.904 vs 0.899, vertices 39882 vs 39923 (rel diff <= 0.6 %). This PC rerun = identical | GPU arch difference (5080 vs 4060 kernels), ~1e-3 relative |
| board_scale, same poses | all numbers identical (c 0.97880) | identical |
| board_scale on laptop's own BA | c 0.97834 vs 0.97880 (boot sd 0.0043) | within noise |
Cause guesses (not tested): the 5080 (sm_120) nondeterminism in pycolmap CUDA SIFT may come from a PTX-JIT / atomic-order path not hit on sm_89.
Mesh numbers from the laptop's own BA (pooled |dZ| p90 3.10 cm vs 5.54 here) differ more because the BA poses differ — the pooled p90 is
sensitive to a few keyframes; compare depth on the same poses.

Wall time (s) | laptop | this PC (idle, heavy.lock)
---|---|---
VIO (lp_replay, CPU) | 19.9 / 23.0 | 23.9
kf_ba (GPU SIFT + CPU BA) | 40.5 / 71.9 / 62.9 (BA phase 18 -> 53 s, CPU throttle/background, not checked) | 40.1
mesh (FFS 960x540 + TSDF) | 85.5 / 92.6 | 68.1 / 64.1
board_scale | 7.9 | 8.7
job overhead | push 1 s, send rec 6 s, pull 0-2 s, poll ≤ 20 s | -
Laptop on AC (battery 100 %). One recording only; re-check one job after any apt/venv change on either PC.
