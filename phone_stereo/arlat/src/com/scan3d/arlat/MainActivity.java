package com.scan3d.arlat;

import android.app.Activity;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.os.Bundle;
import android.os.SystemClock;
import android.view.WindowManager;
import android.widget.TextView;

import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Locale;

/**
 * VIO latency/pose bench (PC page: scan3d/tools/vio_bench_server.py).
 * Intent extras: mode = arcore (default) | ov (live OpenVINS, OvRunner) | cam2 (camera2 delivery only, Cam2Probe);
 * secs = run length (0 = until the activity is stopped).
 * ov (0.6.7): ov_cam lens2 (default) | main (std60 camera, baseline timing), ov_size, fps, exp_us, iso, focus_dpt,
 * ov_cfg (config subdir of files/ov), ov_preset bench | live, imu raw | cal, ov_timing 0/1, max_temp_dc, pool,
 * perf 0/1 (0.6.13, default 1: thread priorities + PerformanceHintManager + sustained mode, OvRunner doc);
 * results in files/ov/run_<time>_.../ (OvRunner).
 * 0.6.20: intent profile rec20261025 (mode ov): recommended live settings as defaults (OvRunner doc; explicit intents win).
 * mode imu_tx (no camera, no VIO): only ViopImuSender for secs (intent viop_imu udp:HOST:PORT | usb, viop_imu_uncal) ->
 * files/ov/imu_tx_<time>/viop_imu.json + .csv (PC check of the phone IMU stream).
 * 0.6.23 mode calcap: lens-calibration stills from the live-VIO stream at 10 fps, driven by the PC over TCP (CalCapRunner).
 * 0.6.26 (integ2-20261026): profiles rec20261025/rec60 ov_opt + sym_inplace,ekf_sparse; bridge --drop-late mode 1 (droplatefix),
 *   intent viop_hb_src (esp32fix), ov_opt gyro_klt=N opt-in (ovgyro). calcap unchanged.
 * 0.6.27 (user 2026-10-26 morning): profiles rec20261025/rec60 ov_opt + gyro_klt=5 (ovgyro guarded mode, changes poses).
 *   Without gyro = intent ov_opt "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse" (explicit intent wins).
 * 0.6.28 (user 2026-10-26, 60 fps only): profile rec60 + loop 1 (rec20261025 unchanged) (ovloop BoW loop closure, own thread; vocabulary shipped in
 *   the APK assets, copied to files/ov/voc_k10L4.voc). Without loop = intent loop 0. Profile + recover N -> loop off.
 * 0.6.30 (appnpu-20261028): intent npu_loop 1 (default 0 = 0.6.29 behaviour) -> XFeat + LighterGlue on the NPU in the loop
 *   verification (NpuLoop, needs loop on); options npu_xf, npu_lg, npu_lg_in, npu_lg_pad, npu_topk, npu_cpus, npu_nice,
 *   npu_perf, npu_cache_mb, npu_cfg, npu_timeout_ms.
 * arcore: a GL thread loops Session.update() and per new frame logs/streams
 *   {"src":"arcore","t_sensor":android camera ts,"t_frame":ARCore ts,"t_call":..,"t_out":update() return,"status":..,"p":[..],"q":[..]}
 * also written to files/arlat_<ms>.csv. Latency = t_out - t_sensor (phone clock).
 */
public class MainActivity extends Activity {
    private Session session;
    private volatile boolean running;
    private Thread worker;
    private TextView status;
    private Cam2Probe cam2;
    private OvRunner ov;
    private boolean sustainedOn;                   // 0.6.13 ov: Window sustained performance mode set
    private HsRecorder hs;
    private PeekRunner peek;                       // mode peek: 1 fps framing stills (0.6.9)
    private CalCapRunner calcap;                   // mode calcap: auto lens-calibration stills at 10 fps (0.6.23)
    private HsPreview hsPrev;
    private Streamer streamer;
    private ViopImuSender imuTx;                    // 0.6.20 mode imu_tx

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setShowWhenLocked(true);      // started from the PC while the phone is locked: run over the lock screen
        setTurnScreenOn(true);
        status = new TextView(this);
        status.setTextSize(18);
        status.setPadding(40, 120, 40, 40);
        setContentView(status);
        streamer = new Streamer();
    }

    @Override
    protected void onDestroy() {
        if (streamer != null) streamer.close();
        super.onDestroy();
    }

    @Override
    protected void onResume() {
        super.onResume();
        final int secs = getIntent().getIntExtra("secs", 0);
        String mode = getIntent().getStringExtra("mode");
        if ("cam2".equals(mode)) {          // camera2 delivery latency of the VIO lens instead of ARCore
            if (cam2 == null) {
                cam2 = new Cam2Probe(this, this::show);
                String phys = getIntent().getStringExtra("phys");
                try { cam2.start(secs > 0 ? secs : 30, phys == null ? "2" : phys); }
                catch (Exception e) { status.setText("cam2 error: " + e); }
            }
            return;
        }
        if ("peek".equals(mode)) {          // 0.6.9: 1 fps JPEG stills -> files/peek.jpg for the board page's framing view
            if (peek == null) {
                peek = new PeekRunner(this, this::show);
                peek.focusDpt = getIntent().getFloatExtra("focus_dpt", 0.5f);
                peek.expUs = getIntent().getIntExtra("exp_us", 2000);
                peek.iso = getIntent().getIntExtra("iso", 0);                 // 0.6.11: > 0 = fixed ISO (old); else auto ISO once at start
                peek.autoIso = getIntent().getIntExtra("auto_iso", 1) != 0;
                peek.isoScale = getIntent().getFloatExtra("iso_target_scale", 1f);
                peek.periodMs = getIntent().getIntExtra("period_ms", 1000);
                peek.secs = secs > 0 ? secs : 300;
                peek.okView = t -> runOnUiThread(() -> {      // full-screen green + text when the PC says the pose fits
                    if (t.isEmpty()) { status.setBackgroundColor(0xFF000000); status.setTextColor(0xFFFFFFFF); status.setTextSize(18); status.setText("위치 맞추기 (1 fps)"); }
                    else { status.setBackgroundColor(0xFF16A34A); status.setTextColor(0xFFFFFFFF); status.setTextSize(34); status.setText("✔ " + t + "\n\n노트북에서 촬영"); }
                });
                try { peek.start(); status.setText("위치 맞추기 (1 fps)"); }
                catch (Exception e) { status.setText("peek error: " + e); peek = null; }
            }
            return;
        }
        if ("calcap".equals(mode)) {        // 0.6.23: auto lens-calibration stills (CalCapRunner doc); default app behaviour unchanged
            if (calcap == null) {
                calcap = new CalCapRunner(this, this::show);
                calcap.fps = Math.max(1, Math.min(30, getIntent().getIntExtra("fps", 10)));
                calcap.aeFps = Math.max(1, Math.min(60, getIntent().getIntExtra("ae_fps", 30)));
                calcap.expUs = getIntent().getIntExtra("exp_us", 2000);
                calcap.iso = getIntent().getIntExtra("iso", 0);
                calcap.autoIso = getIntent().getIntExtra("auto_iso", 1) != 0;
                calcap.isoScale = getIntent().getFloatExtra("iso_target_scale", 1f);
                calcap.focusDpt = getIntent().getFloatExtra("focus_dpt", 0.5f);
                calcap.port = getIntent().getIntExtra("port", 8811);
                calcap.prevDiv = Math.max(1, Math.min(4, getIntent().getIntExtra("prev_div", 2)));
                calcap.idleS = Math.max(30, getIntent().getIntExtra("idle_s", 300));
                calcap.maxS = Math.max(30, Math.min(3600, getIntent().getIntExtra("max_s", 1200)));
                calcap.lostS = Math.max(5, getIntent().getIntExtra("lost_s", 20));
                calcap.connectGraceS = Math.max(10, getIntent().getIntExtra("connect_grace_s", 90));
                // overheat guards: may only be made stricter than the defaults (SKIN 40 / AP 50 / battery 42.0 C)
                float ms = getIntent().getFloatExtra("max_skin_c", 40f), ma = getIntent().getFloatExtra("max_ap_c", 50f);
                calcap.maxSkinC = Float.isNaN(ms) ? 40f : Math.min(40f, ms);     // NaN would switch the check off
                calcap.maxApC = Float.isNaN(ma) ? 50f : Math.min(50f, ma);
                calcap.maxBatteryTenthC = Math.min(420, getIntent().getIntExtra("max_temp_dc", 420));
                calcap.localDiv = Math.max(0, Math.min(8, getIntent().getIntExtra("local_div", 3)));   // 0.6.25 own picture; 0 = off
                final CalCapView iv = new CalCapView(this);      // 0.6.25: own live picture + PC overlay (was a PC-rendered JPEG)
                android.widget.LinearLayout ll = new android.widget.LinearLayout(this);
                ll.setOrientation(android.widget.LinearLayout.VERTICAL);
                if (status.getParent() != null) ((android.view.ViewGroup) status.getParent()).removeView(status);
                ll.addView(iv, new android.widget.LinearLayout.LayoutParams(-1, 0, 3f));
                ll.addView(status, new android.widget.LinearLayout.LayoutParams(-1, 0, 2f));
                status.setPadding(30, 20, 30, 20);
                setContentView(ll);
                calcap.screen = new CalCapRunner.Screen() {
                    @Override public void show(int bg, String t) { runOnUiThread(() -> {
                        status.setBackgroundColor(bg); status.setTextColor(0xFFFFFFFF); status.setTextSize(26); status.setText(t); }); }
                    @Override public void image(android.graphics.Bitmap b) { }     // old PC picture: ignored (own picture shown)
                    @Override public void frame(byte[] y, int w, int h) { iv.frame(y, w, h); }
                    @Override public void overlay(org.json.JSONObject j) { iv.overlay(j); }
                    @Override public void sensorOrientation(int so) { iv.setSensorOrientation(so); }
                };
                calcap.onAutoStop = () -> runOnUiThread(this::finish);      // onPause stops the runner (camera off, meta.json)
                try { calcap.start(); }
                catch (Exception e) { status.setText("calcap error: " + e); android.util.Log.e("arlat", "calcap", e); calcap.stop(); calcap = null; finish(); }
            }
            return;
        }
        if ("hs".equals(mode)) {            // 120 fps high-speed recording (main camera + IMU) for board ground truth
            if (hs == null) {
                hs = new HsRecorder(this, this::show);
                String sess = getIntent().getStringExtra("session");       // "hs" (default) | "std" (regular session, 1080p60)
                hs.session = "std".equals(sess) ? "std" : "hs";
                hs.wantFps = getIntent().getIntExtra("fps", hs.isStd() ? 60 : 120);
                // 0.6.11 (user 2026-10-24): std = exposure fixed (exp_us, default 2000) + ISO chosen ONCE at start from AE, then
                // locked (auto_iso 1, default). iso > 0 = that ISO (no auto); auto_iso 0 = old behaviour (AE unless exp_us/iso given).
                hs.autoIso = getIntent().getIntExtra("auto_iso", 1) != 0;
                hs.isoScale = getIntent().getFloatExtra("iso_target_scale", 1f);
                hs.expUs = getIntent().getIntExtra("exp_us", hs.isStd() && hs.autoIso ? 2000 : 0);   // std: manual exposure (AE off) in microseconds
                hs.iso = getIntent().getIntExtra("iso", 0);                    // std: manual ISO (AE off)
                hs.evComp = getIntent().getIntExtra("ev", 0);
                hs.wantKbps = getIntent().getIntExtra("kbps", 160_000);
                hs.focusDpt = getIntent().getFloatExtra("focus_dpt", 0.5f);
                hs.afRegion = getIntent().getStringExtra("af_region");      // "full" | "center:0.3" (with af_mode)
                hs.afMode = getIntent().getIntExtra("af_mode", -1);          // CONTROL_AF_MODE value instead of fixed focus
                hs.focusSweep = getIntent().getStringExtra("focus_sweep");  // "0.37,3.33,1.0" dpt, each held 2 s (cycling)
                hs.maxBatteryTenthC = getIntent().getIntExtra("max_temp_dc", 460);
                hs.maxThermal = getIntent().getIntExtra("max_thermal", hs.maxThermal);
                try {
                    if (getIntent().getIntExtra("preview", 1) != 0) {     // on-screen view + board detected / not detected
                        hs.choose();
                        if (hsPrev == null) { hsPrev = new HsPreview(this, status, hs.size.getWidth(), hs.size.getHeight()); setContentView(hsPrev.root); }
                        hsPrev.start();
                        hs.previewSurface = hsPrev.surface;
                    }
                    hs.start();
                    if (hsPrev != null) hsPrev.setGuide(getIntent().getStringExtra("guide"), getIntent().getIntExtra("guide_secs", 0));
                    if (hsPrev != null) hsPrev.startGuide(this, () -> {
                        if (hs != null) hs.stopReason = "user (guide end " + hsPrev.guideEnd + " s)";
                        finish();                                     // onPause stops the recorder and closes the files
                    });
                }
                catch (Exception e) { status.setText("hs error: " + e); android.util.Log.e("arlat", "hs", e); hs = null; }
            }
            return;
        }
        if ("imu_tx".equals(mode)) {        // 0.6.20: phone IMU sender only (no camera)
            if (imuTx == null) {
                try {
                    String tg = getIntent().getStringExtra("viop_imu");
                    java.io.File d = new java.io.File(getExternalFilesDir(null), "ov/imu_tx_" + new java.text.SimpleDateFormat(
                            "yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date()));
                    d.mkdirs();
                    imuTx = new ViopImuSender(this, tg == null ? "" : tg.trim(), getIntent().getIntExtra("viop_imu_uncal", 0) != 0, d);
                    imuTx.start();
                    status.setText("imu_tx " + tg + " -> " + d);
                    final ViopImuSender mine = imuTx;
                    status.postDelayed(() -> { if (imuTx == mine) finish(); }, Math.max(1, secs > 0 ? secs : 30) * 1000L);
                } catch (Exception e) { status.setText("imu_tx error: " + e); imuTx = null; }
            }
            return;
        }
        if ("ov".equals(mode)) {
            if (ov == null) {
                ov = new OvRunner(this, this::show, streamer);
                ov.dumpSecs = getIntent().getIntExtra("dump", 0);
                ov.expCapNs = getIntent().getIntExtra("exp_cap_us", 0) * 1000L;   // lens2: short exposure (less motion blur)
                // 2026-10-23 (0.6.7): ov_cam main = the std60 recording camera with the baseline config (OvRunner doc)
                String oc = getIntent().getStringExtra("ov_cam");
                boolean main = "main".equals(oc);
                ov.cam = main ? "main" : "lens2";
                String sz = getIntent().getStringExtra("ov_size");
                if (sz == null) sz = main ? "1920x1080" : "640x480";
                ov.W = Integer.parseInt(sz.split("x")[0]);
                ov.H = Integer.parseInt(sz.split("x")[1]);
                ov.fps = getIntent().getIntExtra("fps", main ? 60 : 30);
                ov.expUs = getIntent().getIntExtra("exp_us", main ? 2000 : 0);
                ov.autoIso = getIntent().getIntExtra("auto_iso", 1) != 0;     // 0.6.11 main: ISO from AE once at start, then locked
                ov.isoScale = getIntent().getFloatExtra("iso_target_scale", 1f);
                ov.iso = getIntent().getIntExtra("iso", main && !ov.autoIso ? 500 : 0);   // > 0 = that ISO (no auto)
                ov.focusDpt = getIntent().getFloatExtra("focus_dpt", main ? 0.5f : -1f);
                String cfg = getIntent().getStringExtra("ov_cfg");
                ov.cfgName = cfg == null ? "" : cfg;
                String pre = getIntent().getStringExtra("ov_preset");
                ov.preset = pre != null ? pre : (main ? "bench" : "live");
                String imu = getIntent().getStringExtra("imu");
                ov.imuRaw = imu != null ? "raw".equals(imu) : main;
                ov.ovTiming = getIntent().getIntExtra("ov_timing", 1) != 0;
                ov.maxBatteryTenthC = getIntent().getIntExtra("max_temp_dc", 460);
                ov.maxThermal = getIntent().getIntExtra("max_thermal", ov.maxThermal);
                ov.poolSize = Math.max(2, getIntent().getIntExtra("pool", 4));
                ov.perfOn = getIntent().getIntExtra("perf", 1) != 0;      // 0.6.13: priorities + hint session + sustained mode
                ov.hintMs = Math.max(0, getIntent().getIntExtra("hint_ms", 0));   // 0.6.14: hint target ms (0 = 1000/fps)
                String cpus = getIntent().getStringExtra("cpus");               // 0.6.14: ov_bridge --cpus (e.g. 7,8,9)
                ov.cpus = cpus == null ? "" : cpus.trim();
                ov.ovThreads = Math.max(0, getIntent().getIntExtra("ov_threads", 0));   // 0.6.15: ov_bridge --threads (0 = config)
                ov.kltOverlap = getIntent().getIntExtra("klt_overlap", 0) != 0;        // 0.6.15: ov_bridge --klt-overlap 1
                ov.idleSpin = Math.max(0, Math.min(2, getIntent().getIntExtra("idle_spin", 0)));   // 0.6.16: ov_bridge --idle-spin 0|1|2
                ov.predict = Math.max(0, Math.min(2, getIntent().getIntExtra("predict", 0)));       // 0.6.17: ov_bridge --predict 0|1|2
                ov.predictHz = Math.max(1, Math.min(500, getIntent().getIntExtra("predict_hz", 100)));   // 0.6.17: --predict-hz (mode 2)
                ov.predictStream = getIntent().getIntExtra("predict_stream", 0) != 0;   // 0.6.17: pred lines also to the PC stream
                String oo = getIntent().getStringExtra("ov_opt");                       // 0.6.18: ov_bridge --ovopt LIST
                ov.ovOpt = oo == null ? "" : oo.trim();
                ov.ovProf = getIntent().getIntExtra("ov_prof", 0) != 0;                 // 0.6.18: --prof-log <run>/ov_prof.csv
                ov.idleSpinMs = Math.max(0, Math.min(1000, getIntent().getIntExtra("idle_spin_ms", 0)));   // 0.6.18: --idle-spin-ms
                ov.spinWfe = getIntent().getIntExtra("spin_wfe", 0) != 0;               // 0.6.18: idle_spin 2 -> --idle-spin 3
                ov.dropLateMs = Math.max(0, Math.min(10000, getIntent().getIntExtra("drop_late_ms", 0)));   // 0.6.18: --drop-late
                ov.imuPrio = getIntent().getIntExtra("imu_prio", 0) != 0 ? 1 : 0;        // 0.6.19: IMU never waits behind an image write
                ov.predThread = getIntent().getIntExtra("pred_thread", 1) != 0 ? 1 : 0;  // 0.6.19: predict 2 -> --pred-thread 1 (0 = 0.6.18 mode 2)
                ov.imuLog = getIntent().getIntExtra("imu_log", 0) != 0;                 // 0.6.19: IMU delivery logs
                String mu = getIntent().getStringExtra("mav_udp");                      // ovdrone: ov_bridge --mav-udp HOST:PORT
                ov.mavUdp = mu == null ? "" : mu.trim();
                String mm = getIntent().getStringExtra("mav_mount");                    // ovdrone: --mav-mount imu|cam_fwd|cam_down|9 numbers
                if (mm != null) ov.mavMount = mm.trim();
                String mg = getIntent().getStringExtra("mav_msg");                      // ovdrone: --mav-msg odom|vpe|both
                if (mg != null) ov.mavMsg = mg.trim();
                ov.ovHealth = getIntent().getIntExtra("ov_health", 0) != 0;                 // ovrobust: --health 1 + health.csv
                ov.initStill = getIntent().getIntExtra("init_still", 0) != 0;               // ovrobust: --init-still 1
                ov.recoverFrames = Math.max(0, Math.min(1000, getIntent().getIntExtra("recover", 0)));   // ovrobust: --recover N
                ov.loop = getIntent().getIntExtra("loop", 0) != 0 ? 1 : 0;                  // 0.6.28 (ovloop): --loop 1 (BoW loop closure)
                if (getIntent().getIntExtra("npu_loop", 0) != 0) {                           // 0.6.30 (appnpu): NPU XFeat + LighterGlue, default off
                    NpuLoop n = new NpuLoop(this);
                    String s0;
                    if ((s0 = getIntent().getStringExtra("npu_xf")) != null) n.xfName = s0;
                    if ((s0 = getIntent().getStringExtra("npu_lg")) != null) n.lgName = s0;
                    if ((s0 = getIntent().getStringExtra("npu_lg_in")) != null) n.lgIn = s0;
                    if ((s0 = getIntent().getStringExtra("npu_cpus")) != null) n.cpus = s0;
                    if ((s0 = getIntent().getStringExtra("npu_cfg")) != null) n.cfgExtra = s0;
                    n.lgPad = Math.max(0, Math.min(2, getIntent().getIntExtra("npu_lg_pad", n.lgPad)));
                    n.topk = Math.max(64, Math.min(2048, getIntent().getIntExtra("npu_topk", n.topk)));
                    n.nice = Math.max(-20, Math.min(19, getIntent().getIntExtra("npu_nice", n.nice)));
                    n.perf = getIntent().getIntExtra("npu_perf", n.perf);
                    n.cacheMb = Math.max(16, Math.min(2000, getIntent().getIntExtra("npu_cache_mb", (int) n.cacheMb)));
                    n.timeoutMs = Math.max(200, Math.min(60000, getIntent().getIntExtra("npu_timeout_ms", n.timeoutMs)));
                    ov.npu = n;
                }
                // 0.6.20 (integ): named profile = defaults for intents not given explicitly; no profile = 0.6.19 defaults
                String prof = getIntent().getStringExtra("profile");
                if ("rec20261025".equals(prof) || "rec60".equals(prof)) {
                    ov.profile = prof;
                    // 0.6.26 (integ2-20261026): + ovupd sym_inplace (exact) and ekf_sparse (near-exact); old 0.6.25 profile = ov_opt
                    // "all_exact,ekf_gemm,qr_compress,ekf_chol" given explicitly. 0.6.27 (user 2026-10-26): + gyro_klt=5 (ovgyro guarded mode);
                    // profile without gyro = ov_opt "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse" given explicitly.
                    if (!getIntent().hasExtra("ov_opt")) ov.ovOpt = "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5";
                    if (!getIntent().hasExtra("idle_spin")) ov.idleSpin = 2;
                    if (!getIntent().hasExtra("idle_spin_ms")) ov.idleSpinMs = 20;
                    if (!getIntent().hasExtra("drop_late_ms")) ov.dropLateMs = 50;
                    if (!getIntent().hasExtra("ov_health")) ov.ovHealth = true;
                    // 0.6.21 (user 19:40): IMU priority send (incl. chunked images) + 200 Hz prediction with the prediction thread
                    if (!getIntent().hasExtra("imu_prio")) ov.imuPrio = 1;
                    if (!getIntent().hasExtra("predict")) ov.predict = 2;
                    if (!getIntent().hasExtra("predict_hz")) ov.predictHz = 200;
                    if (!getIntent().hasExtra("pred_thread")) ov.predThread = 1;
                    if ("rec60".equals(prof)) {
                        // 0.6.28 (user 2026-10-26, 60 fps only): + loop closure (VIO/control pose unchanged; adds map fields /
                        // MAP_ODOM). rec20261025 unchanged. Not with recover N.
                        if (!getIntent().hasExtra("loop")) ov.loop = ov.recoverFrames > 0 ? 0 : 1;   // 0.6.22 (user 20:00): same + 60 fps C4 (config c4f60)
                        if (!getIntent().hasExtra("fps")) ov.fps = 60;
                        if (!getIntent().hasExtra("ov_cfg")) ov.cfgName = "c4f60";
                        // 20 ms window > 16.7 ms frame period: spin the whole gap (0 = no window), set explicitly
                        if (!getIntent().hasExtra("idle_spin_ms")) ov.idleSpinMs = 0;
                        // drop_late 50 ms kept = same latency bound in time (3 frames at 60 fps)
                    }
                } else if (prof != null && !prof.isEmpty() && !"none".equals(prof)) throw new IllegalArgumentException("unknown profile " + prof);
                String vu = getIntent().getStringExtra("viop_udp");                     // 0.6.20: ov_bridge --viop-udp HOST:PORT
                ov.viopUdp = vu == null ? "" : vu.trim();
                ov.viopHbHz = Math.max(0, Math.min(200, getIntent().getIntExtra("viop_hb_hz", 20)));
                ov.viopHbSrc = getIntent().getIntExtra("viop_hb_src", 0) != 0 ? 1 : 0;  // esp32fix: 1 = steady heartbeat thread
                String vi = getIntent().getStringExtra("viop_imu");                     // 0.6.20: ViopImuSender
                ov.viopImu = vi == null ? "" : vi.trim();
                ov.viopImuUncal = getIntent().getIntExtra("viop_imu_uncal", 0) != 0;
                sustainedOn = false;
                try {
                    boolean sup = ((android.os.PowerManager) getSystemService(POWER_SERVICE)).isSustainedPerformanceModeSupported();
                    ov.notePerf("sustained_mode_supported", sup);
                    if (sup && ov.perfOn) { getWindow().setSustainedPerformanceMode(true); sustainedOn = true; }
                    ov.notePerf("sustained_mode_set", sustainedOn);
                } catch (Throwable e) { ov.notePerf("sustained_mode", e.toString()); }
                ov.onAutoStop = () -> runOnUiThread(this::finish);      // onPause stops the runner and writes meta.json
                // 0.6.12: step guide like hs/std (extras guide "label:secs;...", guide_secs): big countdown from the first fed
                // frame (after auto ISO); at the end finish() -> onPause stops the runner and writes meta.json. With a guide the
                // secs timer is not used (the guide end stops the run).
                final OvGuide g = OvGuide.make(this, getIntent().getStringExtra("guide"), getIntent().getIntExtra("guide_secs", 0));
                if (g != null) {
                    if (status.getParent() != null) ((android.view.ViewGroup) status.getParent()).removeView(status);
                    g.root.addView(status);
                    status.setTextSize(14); status.setTextColor(0xFFBBBBBB); status.setPadding(0, 60, 0, 0);
                    setContentView(g.root);
                    g.waiting(ov.autoIsoActive());
                    ov.noteGuide(g.spec, g.totalS);
                    final OvRunner mineG = ov;
                    ov.onFeedStart = () -> runOnUiThread(() -> {
                        if (ov != mineG) return;
                        g.start(this, () -> {
                            if (ov == mineG) ov.stopReason = "user (guide end " + g.totalS + " s)";
                            finish();
                        });
                    });
                }
                try {
                    ov.start(); status.setText("OpenVINS starting...");
                    final OvRunner mine = ov;
                    if (secs > 0 && g == null) status.postDelayed(() -> {
                        if (ov != mine) return;       // an older run's timer must not stop a newer run
                        ov.stopReason = "secs " + secs;
                        finish();
                    }, secs * 1000L);
                }
                catch (Exception e) { status.setText("ov error: " + e); android.util.Log.e("arlat", "ov", e); }
            }
            return;
        }
        try {
            if (session == null) {
                session = new Session(this);
                // camera configs this phone offers ARCore (logcat "arlat cfg"); extra fps=60 picks a 60 fps one if any
                int wantFps = getIntent().getIntExtra("fps", 30);
                com.google.ar.core.CameraConfigFilter ff = new com.google.ar.core.CameraConfigFilter(session);
                java.util.List<com.google.ar.core.CameraConfig> cfgs = session.getSupportedCameraConfigs(ff);
                com.google.ar.core.CameraConfig pick = null;
                for (com.google.ar.core.CameraConfig cc : cfgs) {
                    android.util.Log.i("arlat", "cfg cam " + cc.getCameraId() + " fps " + cc.getFpsRange() + " cpu " + cc.getImageSize()
                            + " tex " + cc.getTextureSize() + " depth " + cc.getDepthSensorUsage() + " facing " + cc.getFacingDirection());
                    if (pick == null && cc.getFpsRange().getUpper() >= wantFps
                            && cc.getFacingDirection() == com.google.ar.core.CameraConfig.FacingDirection.BACK) pick = cc;
                }
                if (pick != null && wantFps > 30) {
                    session.setCameraConfig(pick);
                    android.util.Log.i("arlat", "cfg picked fps " + pick.getFpsRange() + " " + pick.getImageSize());
                }
                Config c = new Config(session);
                c.setUpdateMode(Config.UpdateMode.BLOCKING);
                c.setFocusMode(Config.FocusMode.FIXED);
                // ARCore depth gives the 3D points for feature loop closure (ArLoop "feature" / "both")
                // (only when feature loops are on: depth costs ARCore time — measure with extra depth=0/1)
                String lm = getIntent().getStringExtra("loop");
                // default off: ARCore depth raised pose latency 75 -> 176 ms (median) on the S24; arloop triangulates instead
                boolean wantDepth = getIntent().getIntExtra("depth", 0) != 0 && (lm == null || lm.equals("both") || lm.equals("feature"));
                if (wantDepth && session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) c.setDepthMode(Config.DepthMode.AUTOMATIC);
                session.configure(c);
            }
            session.resume();
        } catch (Exception e) {
            status.setText("ARCore error: " + e);
            android.util.Log.e("arlat", "start", e);
            return;
        }
        running = true;
        worker = new Thread(() -> loop(secs), "arlat");
        worker.start();
    }

    @Override
    protected void onPause() {
        running = false;
        if (cam2 != null) { cam2.stop(); cam2 = null; }
        if (ov != null) { ov.stop(); ov = null; }
        if (imuTx != null) { imuTx.stop(); imuTx = null; }   // 0.6.20 mode imu_tx
        if (sustainedOn) { try { getWindow().setSustainedPerformanceMode(false); } catch (Throwable ignored) { } sustainedOn = false; }
        if (hs != null) { hs.stop(); hs = null; }
        if (peek != null) { peek.stop(); peek = null; }
        if (calcap != null) { calcap.stop(); calcap = null; }   // 0.6.23: camera off whenever the activity leaves the front
        if (hsPrev != null) { hsPrev.stop(); hsPrev = null; }
        if (worker != null) {
            try { worker.join(2000); } catch (InterruptedException ignored) { }
        }
        if (session != null) session.pause();
        super.onPause();
    }

    private void show(final String s) {
        android.util.Log.i("arlat", "status: " + s.replace('\n', ' '));
        runOnUiThread(() -> status.setText(s));
    }

    private void loop(int secs) {
        EGLDisplay dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
        int[] ver = new int[2];
        EGL14.eglInitialize(dpy, ver, 0, ver, 1);
        int[] attr = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_NONE};
        EGLConfig[] cfg = new EGLConfig[1];
        int[] n = new int[1];
        EGL14.eglChooseConfig(dpy, attr, 0, cfg, 0, 1, n, 0);
        EGLContext ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT,
                new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
        EGLSurface surf = EGL14.eglCreatePbufferSurface(dpy, cfg[0],
                new int[]{EGL14.EGL_WIDTH, 16, EGL14.EGL_HEIGHT, 16, EGL14.EGL_NONE}, 0);
        EGL14.eglMakeCurrent(dpy, surf, surf, ctx);
        int[] tex = new int[1];
        GLES20.glGenTextures(1, tex, 0);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex[0]);
        session.setCameraTextureName(tex[0]);

        File out = new File(getExternalFilesDir(null), "arlat_" + System.currentTimeMillis() + ".csv");
        long tEnd = secs > 0 ? SystemClock.elapsedRealtime() + secs * 1000L : Long.MAX_VALUE;
        int frames = 0, tracking = 0;
        long lastTs = 0;
        // loop closure: extras loop = both | marker | feature | off, mdict (ArUco dictionary id, 0 = 4X4_50), mlen (m),
        // dump > 0 = also save keyframes to files/arloop_kf.bin for PC replay
        String loopMode = getIntent().getStringExtra("loop");
        if (loopMode == null) loopMode = "both";
        ArLoop loop = null;
        try {
            loop = new ArLoop(loopMode, getIntent().getIntExtra("mdict", 0), getIntent().getFloatExtra("mlen", 0.15f),
                    getIntent().getIntExtra("dump", 0) > 0 ? new File(getExternalFilesDir(null), "arloop_kf.bin") : null);
        } catch (Throwable e) {
            android.util.Log.e("arlat", "arloop", e);
            show("arloop error: " + e);
        }
        try (PrintWriter w = new PrintWriter(new FileWriter(out))) {
            w.println("t_frame_ns,t_cam_android_ns,t_call_ns,t_return_ns,state,tx,ty,tz,qx,qy,qz,qw");
            while (running && SystemClock.elapsedRealtime() < tEnd) {
                long t0 = SystemClock.elapsedRealtimeNanos();
                Frame f = session.update();
                long t1 = SystemClock.elapsedRealtimeNanos();
                long ts = f.getTimestamp();
                if (ts == 0 || ts == lastTs) continue;
                lastTs = ts;
                TrackingState st = f.getCamera().getTrackingState();
                // why ARCore is not tracking (PAUSED/STOPPED): INSUFFICIENT_FEATURES, EXCESSIVE_MOTION, INSUFFICIENT_LIGHT, ...
                String stName = st == TrackingState.TRACKING ? "TRACKING" : st + "/" + f.getCamera().getTrackingFailureReason();
                Pose p = f.getCamera().getPose();
                long tCam = f.getAndroidCameraTimestamp();
                frames++;
                if (st == TrackingState.TRACKING) tracking++;
                w.println(String.format(Locale.US, "%d,%d,%d,%d,%s,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f", ts,
                        tCam, t0, t1, st, p.tx(), p.ty(), p.tz(), p.qx(), p.qy(), p.qz(), p.qw()));
                String extra = "";
                if (loop != null && st == TrackingState.TRACKING) {
                    float[] p7 = ArLoop.pose7(p);
                    loop.onFrame(f, ts, p7);
                    if (loop.active()) {
                        float[] c = loop.correct(ts, p7);
                        long tc = SystemClock.elapsedRealtimeNanos();
                        extra = String.format(Locale.US, ",\"pc\":[%.5f,%.5f,%.5f],\"qc\":[%.6f,%.6f,%.6f,%.6f],\"t_corr\":%d",
                                c[0], c[1], c[2], c[3], c[4], c[5], c[6], tc);
                    }
                    if (frames % 15 == 0 && loop.active()) extra += ",\"loop_mode\":\"" + loop.mode + "\",\"loop\":" + loop.stats();
                }
                streamer.send(String.format(Locale.US,
                        "{\"src\":\"arcore\",\"t_sensor\":%d,\"t_frame\":%d,\"t_call\":%d,\"t_out\":%d,\"status\":\"%s\","
                                + "\"p\":[%.5f,%.5f,%.5f],\"q\":[%.6f,%.6f,%.6f,%.6f]%s}",
                        tCam, ts, t0, t1, stName, p.tx(), p.ty(), p.tz(), p.qx(), p.qy(), p.qz(), p.qw(), extra));
                if (frames % 15 == 0) {
                    show(String.format(Locale.US, "ARCore %s\nframes %d tracking %d\nlatency %.1f ms\nPC %s", st, frames,
                            tracking, (t1 - tCam) / 1e6, streamer.connected ? "connected" : "-"));
                }
            }
        } catch (Exception e) {
            show("error: " + e);
            android.util.Log.e("arlat", "loop", e);
            if (loop != null) loop.close();
            return;
        }
        if (loop != null) loop.close();
        show("DONE " + frames + " frames -> " + out.getName());
        android.util.Log.i("arlat", "DONE " + out);
    }
}
