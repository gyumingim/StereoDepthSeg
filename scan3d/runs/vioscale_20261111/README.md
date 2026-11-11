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

## 2. Runs (2026-11-12 06:42-07:11, laptop, 3 jobs; 미검증)
- 202906: 57 conditions (vs_batch.py, conds.json), 165506: the same 57. Replays deterministic; 165506 B = scalehunt B exactly
  (k_fwd 3.111, k_rev 1.416, ATE 1.702 cm) -> same bridge/config/ruler.
- Mishaps (fixed, nothing rerun with changed numbers): (1) vs_batch out path was relative while lp_replay runs with cwd = livepose dir ->
  first pass wrote replays under scan3d/runs/livepose_20261030/scan3d/... on the laptop, no scores; files moved, scored in pass 2.
  (2) vs_prov onset searched from the first IMU sample (IMU starts 3.1 s before frame 0, phone handled) -> onset/still empty in pass 1;
  fixed to gyro-only from frame 0 − 0.8 s (a desk tap at +2.0..3.0 s gives acc std 0.7 without rotation). S_*, A_meas, GR_meas, I_*_post
  were added in pass 2. I_stat_post (static init after the motion started) does not initialise (202906 diverged, 165506 no init): excluded.
- Identical to B (option has no effect on the tracking poses): I_dyn (static init succeeds first), C_gsens, L_off (loop closure only
  corrects the map pose, not T_world_cam0), I_th0.15/0.6 on 165506; I_audit = I_w15 on 202906 (init_max_disparity irrelevant there).

## 3. Results (미검증)
Rulers: s = relative Sim3 (variant / B at the 324 keyframes) x 1.0489 (= B after BA on the red screen) — vs_rel.py; the red ruler on raw
VIO poses agrees (B 1.0377, rms 22 px; BA poses reproduce 1.04892 exactly). s_imu = IMU ruler (accelerometer double integral, 1 s windows,
fixed lever) on the variant's own trajectory. Full table: results_both.md (cols: cond | s | red on VIO | s_imu | s x s_imu |
165506 k_fwd | k_rev | s_imu(165506) | k_fwd x s_imu | k_rev x s_imu), results_202906/table.txt, results_165506/table.txt.

### 3.1 Provenance (both recordings)
- IMU: types 1/4 (Android calibrated) and 35/16 (raw) all 499 Hz, dt 2.0 ms, 0 gaps > 5 ms, monotonic. VIO uses raw 35/16 (= phone).
  Type 35 extra columns (OS acc bias) = 0 -> acc is unbiased-by-OS raw; type 16 OS gyro bias 0.0024/0.0025/0.0012 rad/s = still-phase gyro mean.
- Still phase (202906 frame 0 − 0.8 s .. +5.45 s): |mean acc| 9.7878 m/s^2 = 0.9990 x local g 9.7973 (165506: 0.9969) -> accelerometer
  scale error ≤ 0.1-0.3 %, can not make +4.9 % (sensitivity ~1 %p per 1 %).
- Frames: 3324 @ 60.02 fps, max dt 16.8 ms, frames_unmatched 0, t_mid − SENSOR_TIMESTAMP = 4.270 ms = (exposure 2 ms + RS skew 6.541 ms)/2
  for every frame (mid exposure, as the VIO assumes). clock_offset_ns is the wall-clock offset (not used: frames and IMU share elapsedRealtime).
- Motion: high-pass linear acceleration rms 0.61 m/s^2 on 202906 vs 1.47 on 165506 (2.4x less), gyro 0.62 vs 0.56 rad/s -> the half-circle
  around the laptop is rotation-heavy and gentle; metric scale is weakly observable from the accelerometer.

### 3.2 Scale on 202906
| set | n | s min / median / max | sd | s x s_imu min / median / max | sd |
|---|---|---|---|---|---|
| null floor (B + start 0.05/0.1/0.2/0.5/1/2 s, all inside the still phase) | 7 | 0.951 / 1.034 / 1.092 | 4.5 % | 0.972 / 0.994 / 1.006 | 1.2 % |
| all 57 conditions | 57 | 0.923 / 1.058 / 1.213 | 5.6 % | 0.936 / 0.994 / 1.027 | 2.0 % |
- B: s 1.049, s_imu 0.959 [95 % 0.926-0.987] -> the accelerometer itself says the B trajectory is ~4 % too long (independent of the lens and
  of the red-screen reference; agrees with the red screen within its CI).
- One-factor shifts (s; B 1.049): accel x0.95 1.004 (positive control, but the measured accel scale is 0.999 -> not physical),
  accel x1.01 1.097, x0.99 1.040; gravity 9.7973 1.057, gravity = still |a| 9.7878 1.050, accel rescaled to local g 1.060;
  online IMU intrinsics 0.923 (+g-sens 1.097); time offset fixed 1.121, ±3 ms 1.055/1.171, ±10 ms 1.059/1.004; extrinsics fixed 0.975;
  noise all x0.5 1.019, x2 1.161, acc noise x2 1.138; num_pts 300 1.030, fast 10 1.213, clones 11 1.004, sigma 0.5 px 1.135;
  init window 0.5/1.5/2 s 1.093/1.143/1.083; dynamic init after the still phase 1.058; gyro Android-calibrated 1.146; IMU 250 Hz 1.102;
  rolling-shutter derotation 1.122; ZUPT on 1.051; loop off = B.
- Every shift is inside the null floor (start offset 0.05 s alone: 0.951; 0.5 s: 1.092) -> pre-registered rule (d) fails for all.

### 3.3 Verdict (pre-registered rule)
- **No VIO setting is a fix.** Settings that land within 0.5 % of 1 (accel x0.95, time offset +10 ms, clones 11) are either contradicted by a
  measurement (accel 0.999, OpenVINS online dt) or tuning knobs, and all are smaller than the null-floor spread (±4.5 %).
- **Cause (추정, strong evidence): the +4.9 % is not a bias of one input but one draw of a chaotic, weakly observed VIO scale on this
  recording.** Physically identical runs (start 0.05-2 s earlier/later inside the still phase) give −4.9 … +9.2 %. Inputs are clean
  (accel scale 0.999, gyro bias small, timestamps consistent, no gaps). The filter's scale is not pinned because this motion has 2.4x less
  linear acceleration than 165506 (where the same null floor spans only k_rev +0.1 … +1.4 %).
- The accelerometer data does contain the right scale: s x s_imu (rescale each VIO trajectory by the accelerometer's own LS fit) gives
  0.994 median, null spread 0.972-1.006 (sd 1.2 % vs 4.5 %), B 1.006.

### 3.4 Exploratory (NOT pre-registered, needs more recordings): post-VIO accelerometer rescale
- 202906: B 1.049 -> 1.006; all 57 conditions sd 5.6 % -> 2.0 %; median 0.994.
- 165506 guard (board GT): B k_fwd 3.11 / k_rev 1.42 -> 1.28 / −0.39; null floor k_rev +0.13…+1.42 -> −1.32…+0.04; mean |k_rev| over
  57 conditions 1.20 -> 0.70, mean |k_fwd| 2.57 -> 1.00 (not worse). The ±2-3 % CI of a single s_imu limits it; 165506 with 4.5 s of motion
  windows has CI ~±2 %.
- Interpretation: OpenVINS gets metric scale from the accelerometer only through the filter (init + MSCKF updates); a batch LS of the whole
  trajectory against the accelerometer recovers it better when the excitation is weak. Candidate for the scanner (scale step after VIO/BA,
  IMU-only, no lens change); to be tested on more GT recordings before adoption (user rule: one recording per experiment -> separate test).
