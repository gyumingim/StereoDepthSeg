# vioscale 2026-11-11: why is the VIO scale +4.9 % on hs_20261111_202906_std60, and can a VIO setting fix it?

Branch `vioscale-20261111` from redscreen-20261111 4bff5b8, worktree `~/scanner/StereoDepthSeg-vioscale`. All compute on the laptop worker
(`tools/remote_laptop.sh`). One recording (202906) + 165506 only as the overfitting guard (board GT). Numbers 미검증 until an independent check.

## 0. What is known before this run (not hidden from the pre-registration)
- redscreen (runs/redscreen_20261111/README.md): red full-screen rectangle, reference 302.00 x 188.75 mm (EDID pitch, 가정),
  estimator B on the BA keyframe poses -> s = 1.0489 (boot sd 0.0003, threshold spread ±0.3 %); BA vs VIO Sim3 0.9985 -> the scale is in VIO.
- scalehunt-20261105 (runs/scalehunt_20261105/README.md, s165506_conds.txt): 40 one-factor VIO conditions x 13 GT recordings, same bridge
  (55e5d35b) + c4f60. On ONE recording almost every small change moves k by ±1-2 %p (165506: B k_fwd +3.1 / k_rev +1.4; gravity 9.7973 -> +0.6 %p).
  Only VIO fx (+0.56 %p per +1 %) and accelerometer scale (+1.25 %p per +1 %, positive control) moved k consistently over 13 recordings.
  Static |a|/g on earlier recordings 0.9992 ± 0.1 %.
- Lens intrinsics are never changed here (user rule: lens only from board calibration).

## 1. Pre-registration (written before any variant was run, 2026-11-11 night)
Ruler for 202906: **s_VIO** = red-rectangle size measured with the VIO poses directly (measure_red.py estimator B, excl 0.15, thr 0.5,
same 324 keyframe JPEGs and frame numbers as redscreen, lens std_1920x1080_f0.50_calcap20261026), s = mean(W/302.00, H/188.75).
Step 1 (reproduce): s_VIO of the original VIO run B_o0 must be 1.049 ± 0.005 (BA/VIO 0.9985 predicts ~1.050).
Ruler for 165506: board GT `imu_gt_spl`, k_fwd and k_rev as scalehunt `sh_score.py` (k = VIO length / GT length − 1, %).
Second, independent ruler on 202906 (no lens, no red screen): **IMU ruler** = imu_learn/scale_ls.py s_fixedlever on the VIO trajectory
(accelerometer double integral vs VIO positions, 1 s windows). s_imu < 1 means the accelerometer says the VIO path is too long.
Noise floor: the same config with start offsets that keep the start inside the still phase (0.05 / 0.1 / 0.2 / 0.5 s) — physically the same
estimate. Their spread = how much s moves by chance on this one recording.
Variants (one factor each vs B; no lens intrinsics): init window/threshold/disparity, start offset after the still phase, dynamic init,
IMU noise densities and random walks, online accelerometer/gyro intrinsics (calib_imu_intrinsics) and g-sensitivity, gravity magnitude
(local 9.7973 estimate and the value measured from the still phase), camera-IMU time offset (fixed vs online, ±3 ms), extrinsics online off,
feature tracker (num_pts, fast threshold, max_slam, sigma px, clones), loop closure off, IMU input (gyro Android-calibrated type 4,
200 Hz decimation, low-pass), accelerometer scale x0.99/x1.01 (positive control), rolling-shutter derotation.
Counts as a **fix** only if ALL of:
  (a) |s_VIO − 1| < 0.5 % on 202906,
  (b) 165506 not worse: |k_rev| ≤ |k_rev(B)| + 0.3 %p and |k_fwd| ≤ |k_fwd(B)| + 0.3 %p,
  (c) the setting is physically justified independently of this number (a measured value or a model that is known to be missing),
      not a tuning knob (noise densities, feature counts, sigma are tuning knobs: reported, never adopted from one recording),
  (d) the shift is larger than the noise floor (|Δs| > 2 x the null-offset spread).
Otherwise the verdict is "no single setting explains it" and the +4.9 % is reported as within the chaotic per-recording spread or as an
input problem (provenance section).
