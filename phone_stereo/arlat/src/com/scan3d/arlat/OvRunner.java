package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Range;
import android.util.Size;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Live OpenVINS on the phone -> ov_bridge child process (lib/arm64-v8a/libov_bridge.so, protocol
 * scan3d/ov_bridge/PROTOCOL.md) -> one JSON line per pose to the Streamer:
 *   {"src":"ov","t_sensor":..,"t_arrive":..,"t_out":..,"status":..,"p":[..],"q":[..],"ms":..,"n_slam":..,"n_msckf":..}
 * t_sensor = SENSOR_TIMESTAMP (start of exposure), t_arrive = image callback, t_out = pose line read (all elapsedRealtimeNanos).
 *
 * Two cameras (intent extra ov_cam):
 *  - "lens2" (default, unchanged since 2026-10-18): physical lens 2 of logical "0", YUV 640x480 @ 30 fps, calibrated
 *    gyro/accel, config + lens0u_remap.bin from files/ov/ (vio_bench_server.py / make_phone_ov.py), "live" bridge args.
 *  - "main" (2026-10-23, on-phone timing of the baseline): logical camera "0" = the std60 recording camera (HsRecorder
 *    session std), YUV ov_size (default 1920x1080) @ fps (60), fixed focus focus_dpt (0.5), manual exposure exp_us/iso
 *    (2000/500), OIS/EIS/NR/edge/distortion correction off like the recorder, uncalibrated gyro/accel (types 16/35,
 *    values 0..2 = the bench's --imu-raw), "bench" bridge args (vio_bench_hs.py), config from files/ov/<ov_cfg>/ and
 *    remap_<WxH>.bin (make_phone_ov_main.py: raw frame -> undistorted 640x360 pinhole, same maps as the bench).
 * Every run writes files/ov/run_<time>_<cam><fps>_<cfg>/: estimator_config.yaml (+ kalibr chains, timing paths set to
 * the run dir), ov_timing.txt (OpenVINS record_timing_information), bridge_timing.csv (ov_bridge --timing-log),
 * frames.csv (every image: fate sent / dropped), poses.csv (every pose line), temps.csv (battery + thermal status, 2 s),
 * events.jsonl (bridge resets / errors), bridge_stderr.log, meta.json (settings, stream offer, counts, stop reason).
 */
class OvRunner implements SensorEventListener {
    interface Status { void show(String s); }

    static final String VERSION = "ov-20261023-timing";
    private static final int MAGIC = 0x4F564252;
    // settings (intent extras, MainActivity)
    String cam = "lens2";
    int W = 640, H = 480;
    int fps = 30;
    int expUs = 0, iso = 0;                     // main: either > 0 -> AE off, manual exposure
    float focusDpt = -1f;                       // main: >= 0 fixed focus (AF off)
    String cfgName = "";                        // config subdir of files/ov ("" = files/ov itself)
    String preset = "live";                     // bridge args: "live" (scan3d live defaults) | "bench" (vio_bench_hs.py)
    boolean imuRaw = false;                     // uncalibrated gyro/accel (types 16/35, values 0..2)
    boolean ovTiming = true;                    // OpenVINS record_timing_information -> run dir/ov_timing.txt
    int maxBatteryTenthC = 460;                 // stop at this battery temperature (backstop; the PC loop watches SKIN); 0.6.8: 42 -> 46 C
    int maxThermal = android.os.PowerManager.THERMAL_STATUS_CRITICAL;   // 0.6.8: was SEVERE (3)
    int poolSize = 4;                           // image buffers in flight (more frames waiting -> dropped, counted)
    Runnable onAutoStop;                        // called (UI side finishes the activity) when a stop rule fires
    volatile String stopReason = "user";

    private final Context ctx;
    private final Status status;
    private final Streamer out;
    private HandlerThread camThread, imuThread;
    private CameraDevice dev;
    private ImageReader reader;
    private Process proc;
    private Thread writerT, readerT;
    private volatile boolean running;
    private final LinkedBlockingQueue<byte[]> toBridge = new LinkedBlockingQueue<>(4096);
    private ArrayBlockingQueue<byte[]> pool;
    /** t_ns sent to the bridge (mid exposure) -> {t_sensor, t_arrive} */
    private final ConcurrentHashMap<Long, long[]> sent = new ConcurrentHashMap<>();
    private volatile long midOffsetNs;          // (exposure + rolling shutter skew) / 2 from the latest capture result
    private volatile long expNs;                // exposure of the latest capture result (motion blur check)
    /** lens2: exposure cap in ns (0 = AE): after 15 AE frames switch to a fixed short exposure with ISO raised to keep the
     *  brightness (ISO capped at the lens max -> shortest exposure that still keeps it). Less blur = fewer lost features. */
    long expCapNs;
    private double meterEv, meterExp; private int meterN, meterFrames;
    private volatile boolean manualSet;
    private CameraCaptureSession sess;
    private CaptureRequest.Builder reqB;
    private volatile int nCam, nPose, nImu, nAcc, camDropped, poolDropped, nResults;
    private volatile float lastFocus = -1;
    private String lastStatus = "-";
    private File dir, runDir;
    private BufferedWriter framesW, posesW, tempsW, eventsW;
    private final Object framesLock = new Object(), posesLock = new Object();
    private final JSONObject meta = new JSONObject();
    private long t0Elapsed;
    private Handler camH;
    private String ldcHow = "not requested";
    private CaptureRequest.Key<Integer> ldcKey;
    private int accType;

    OvRunner(Context ctx, Status status, Streamer out) { this.ctx = ctx; this.status = status; this.out = out; }

    boolean isMain() { return "main".equals(cam); }

    void start() throws Exception {
        running = true;
        t0Elapsed = SystemClock.elapsedRealtimeNanos();
        dir = new File(ctx.getExternalFilesDir(null), "ov");
        File cfgDir = cfgName.isEmpty() ? dir : new File(dir, cfgName);
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
        runDir = new File(dir, "run_" + stamp + "_" + cam + fps + (cfgName.isEmpty() ? "" : "_" + cfgName));
        if (!runDir.mkdirs()) throw new IllegalStateException("cannot create " + runDir);
        File cfg = prepareConfig(cfgDir);
        String remapName = isMain() ? "remap_" + W + "x" + H + ".bin" : "lens0u_remap.bin";
        File remap = new File(cfgDir, remapName);
        if (!remap.exists()) remap = new File(dir, remapName);
        if (!remap.exists()) throw new IllegalStateException("no " + remapName + " in " + cfgDir + " or " + dir);

        String lib = ctx.getApplicationInfo().nativeLibraryDir;
        List<String> cmd = new ArrayList<>();
        cmd.add(lib + "/libov_bridge.so");
        cmd.add(cfg.getAbsolutePath());
        cmd.add("--stdio");
        if ("bench".equals(preset)) {           // = vio_bench_hs.py (baseline runs): no auto reset, never "lost"
            Collections.addAll(cmd, "--max-speed", "8", "--lost-frames", "1000000", "--startup-lost-frames", "1000000");
        } else {
            Collections.addAll(cmd, "--auto-reset", "--reinit-state", "--init-gate", "0.006,0.15,0.8", "--max-cam-queue", "2");
        }
        Collections.addAll(cmd, "--remap", remap.getAbsolutePath(), "--timing-log", new File(runDir, "bridge_timing.csv").getAbsolutePath());
        ProcessBuilder pb = new ProcessBuilder(cmd);
        pb.environment().put("LD_LIBRARY_PATH", lib);
        pb.redirectError(new File(runDir, "bridge_stderr.log"));

        framesW = new BufferedWriter(new FileWriter(new File(runDir, "frames.csv")), 1 << 16);
        framesW.write("idx,t_sensor_ns,t_arrive_ns,t_send_ns,copy_us,fate,exp_ns,focus_dpt\n");   // fate 0 sent, 1 pool empty, 2 queue full
        posesW = new BufferedWriter(new FileWriter(new File(runDir, "poses.csv")), 1 << 16);
        posesW.write("t_ns,t_sensor_ns,t_arrive_ns,t_out_ns,status,ms,remap_ms,n_slam,n_msckf,cam_dropped_bridge,pool_dropped,queue_dropped,n_cam,n_imu,dt_cam_imu\n");
        tempsW = new BufferedWriter(new FileWriter(new File(runDir, "temps.csv")));
        tempsW.write("elapsed_ns,t_rel_s,battery_c,thermal_status,headroom_10s\n");
        eventsW = new BufferedWriter(new FileWriter(new File(runDir, "events.jsonl")));
        meta.put("app_version", VERSION).put("cam", cam).put("size", W + "x" + H).put("fps", fps).put("exp_us", expUs).put("iso", iso)
                .put("focus_dpt", focusDpt).put("cfg", cfgName).put("cfg_dir", cfgDir.getAbsolutePath()).put("preset", preset)
                .put("remap", remap.getAbsolutePath()).put("bridge_cmd", String.join(" ", cmd)).put("ov_timing", ovTiming)
                .put("pool", poolSize).put("start_elapsed_ns", t0Elapsed).put("start_wall", stamp)
                .put("battery_c_start", batteryTenthC() / 10.0).put("thermal_status_start", thermalStatus())
                .put("device", android.os.Build.MODEL);

        proc = pb.start();
        writerT = new Thread(this::writer, "ov-writer");
        readerT = new Thread(this::readerLoop, "ov-reader");
        writerT.start();
        readerT.start();

        imuThread = new HandlerThread("imu");
        imuThread.start();
        SensorManager sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        Handler ih = new Handler(imuThread.getLooper());
        Sensor acc = null, gyr = null;
        if (imuRaw) {
            acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
            gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
            if (acc == null || gyr == null) { acc = null; gyr = null; status.show("no uncalibrated IMU -> calibrated"); }
        }
        boolean raw = acc != null;
        if (!raw) { acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE); }
        accType = acc.getType();
        meta.put("imu", raw ? "uncalibrated types 35/16 (values 0..2)" : "calibrated types 1/4");
        sm.registerListener(this, acc, SensorManager.SENSOR_DELAY_FASTEST, ih);
        sm.registerListener(this, gyr, SensorManager.SENSOR_DELAY_FASTEST, ih);

        camThread = new HandlerThread("cam");
        camThread.start();
        final Handler h = new Handler(camThread.getLooper());
        camH = h;
        pool = new ArrayBlockingQueue<>(poolSize);
        for (int i = 0; i < poolSize; i++) pool.add(new byte[12 + 20 + W * H]);
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        final CameraCharacteristics ch = cm.getCameraCharacteristics(isMain() ? "0" : "2");
        streamOffer(ch);
        reader = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4);
        reader.setOnImageAvailableListener(this::onImage, h);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                if (!running) { d.close(); return; }      // stopped before the camera opened
                dev = d;
                try {
                    OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
                    if (!isMain()) oc.setPhysicalCameraId("2");
                    final CaptureRequest.Builder rb = d.createCaptureRequest(isMain() ? CameraDevice.TEMPLATE_RECORD : CameraDevice.TEMPLATE_PREVIEW);
                    rb.addTarget(reader.getSurface());
                    rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(fps, fps));
                    rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                    // tracking-friendly image processing (same as the recorder 1.15-isp): no noise reduction / sharpening
                    try {
                        if (has(ch.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES), CaptureRequest.NOISE_REDUCTION_MODE_OFF))
                            rb.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
                        if (has(ch.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CaptureRequest.EDGE_MODE_OFF))
                            rb.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
                        if (isMain()) mainRequest(rb, ch);
                        else {
                            if (has(ch.get(CameraCharacteristics.TONEMAP_AVAILABLE_TONE_MAP_MODES), CaptureRequest.TONEMAP_MODE_GAMMA_VALUE)) {
                                rb.set(CaptureRequest.TONEMAP_MODE, CaptureRequest.TONEMAP_MODE_GAMMA_VALUE);
                                rb.set(CaptureRequest.TONEMAP_GAMMA, 2.2f);
                            }
                            Integer dc = rb.get(CaptureRequest.DISTORTION_CORRECTION_MODE);
                            if (dc != null) rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, dc);
                        }
                    } catch (Exception ex) { android.util.Log.w("arlat", "isp", ex); }
                    final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                        @Override public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest q, TotalCaptureResult res) {
                            Long exp = res.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                            Long skew = res.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                            Float fd = res.get(CaptureResult.LENS_FOCUS_DISTANCE);
                            midOffsetNs = ((exp == null ? 0 : exp) + (skew == null ? 0 : skew)) / 2;
                            expNs = exp == null ? 0 : exp;
                            if (fd != null) lastFocus = fd;
                            if (nResults++ == 0) try {
                                meta.put("first_result", "exp_ns " + exp + " skew_ns " + skew + " iso " + res.get(CaptureResult.SENSOR_SENSITIVITY)
                                        + " focus " + fd + " frame_duration_ns " + res.get(CaptureResult.SENSOR_FRAME_DURATION)
                                        + " crop " + HsRecorder.cropRegion(res) + " dc " + res.get(CaptureResult.DISTORTION_CORRECTION_MODE)
                                        + " ois " + res.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE));
                            } catch (Exception ignored) { }
                            if (!isMain() && expCapNs > 0 && !manualSet) meter(res, h);
                        }
                    };
                    d.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,
                            Collections.singletonList(oc), ctx.getMainExecutor(), new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    sess = s; reqB = rb; capCb = cb;
                                    try { s.setRepeatingRequest(rb.build(), cb, h); }
                                    catch (Exception e) {
                                        if (ldcKey == null) { status.show("repeat: " + e); return; }
                                        try {                     // never lose the run to the Samsung vendor tag: retry without it
                                            rb.set(ldcKey, null); ldcKey = null; ldcHow += " -> request failed, removed";
                                            s.setRepeatingRequest(rb.build(), cb, h);
                                        } catch (Exception e2) { status.show("repeat: " + e2); }
                                    }
                                    try { meta.put("samsung_ldc_request", ldcHow); } catch (Exception ignored) { }
                                    h.postDelayed(heatTick, 1000);
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) { status.show("configure failed"); }
                            }));
                } catch (Exception e) { status.show("session: " + e); }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.show("camera error " + e); d.close(); }
        }, h);
    }

    /** Copy files/ov/<cfg>/estimator_config.yaml (+ its kalibr chains) into the run dir with the OpenVINS output paths
     *  pointing there and record_timing_information = ovTiming. */
    private File prepareConfig(File cfgDir) throws Exception {
        File src = new File(cfgDir, "estimator_config.yaml");
        String txt = new String(java.nio.file.Files.readAllBytes(src.toPath()), StandardCharsets.UTF_8);
        StringBuilder sb = new StringBuilder();
        List<String> rel = new ArrayList<>();
        String rd = runDir.getAbsolutePath();
        for (String line : txt.split("\n", -1)) {
            String t = line.trim();
            if (t.startsWith("record_timing_information:")) line = "record_timing_information: " + ovTiming;
            else if (t.startsWith("record_timing_filepath:")) line = "record_timing_filepath: \"" + rd + "/ov_timing.txt\"";
            else if (t.startsWith("filepath_est:")) line = "filepath_est: \"" + rd + "/ov_estimate.txt\"";
            else if (t.startsWith("filepath_std:")) line = "filepath_std: \"" + rd + "/ov_estimate_std.txt\"";
            else if (t.startsWith("filepath_gt:")) line = "filepath_gt: \"" + rd + "/ov_groundtruth.txt\"";
            else if (t.startsWith("relative_config_imu:") || t.startsWith("relative_config_imucam:")) {
                rel.add(t.split(":", 2)[1].split("#")[0].trim().replace("\"", ""));
            }
            sb.append(line).append('\n');
        }
        File dst = new File(runDir, "estimator_config.yaml");
        java.nio.file.Files.write(dst.toPath(), sb.toString().getBytes(StandardCharsets.UTF_8));
        for (String r : rel) java.nio.file.Files.copy(new File(cfgDir, r).toPath(), new File(runDir, r).toPath());
        return dst;
    }

    /** main camera request = HsRecorder session std: OIS off, distortion correction off (AOSP + Samsung tag), fixed focus,
     *  manual exposure. */
    private void mainRequest(CaptureRequest.Builder rb, CameraCharacteristics ch) throws Exception {
        rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
        int[] dm = ch.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
        if (has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF))
            rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
        meta.put("aosp_distortion_request", has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF) ? "OFF" : "not offered");
        try {
            CaptureRequest.Key<Integer> k = new CaptureRequest.Key<>(HsRecorder.LDC_TAG, Integer.class);
            rb.set(k, 0); ldcKey = k; ldcHow = "set 0 (int32)";
        } catch (Throwable e) { ldcHow = "not set: " + e; }
        if (focusDpt >= 0) {
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
        }
        if (expUs > 0 || iso > 0) {
            long frameDur = 1_000_000_000L / fps;
            long e = expUs > 0 ? Math.min(expUs * 1000L, frameDur) : frameDur / 2;
            int s = iso > 0 ? iso : 400;
            Range<Long> er = ch.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
            Range<Integer> sr = ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            if (er != null) e = er.clamp(e);
            if (sr != null) s = sr.clamp(s);
            rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            rb.set(CaptureRequest.SENSOR_EXPOSURE_TIME, e);
            rb.set(CaptureRequest.SENSOR_SENSITIVITY, s);
            rb.set(CaptureRequest.SENSOR_FRAME_DURATION, frameDur);
            meta.put("exposure_ns_set", e).put("iso_set", s).put("frame_duration_ns_set", frameDur);
        } else meta.put("exposure_ns_set", "AE");
    }

    /** What the camera offers for this YUV size (meta.json): size listed, min frame duration (<= 1/fps for real time). */
    private void streamOffer(CameraCharacteristics ch) {
        try {
            StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size want = new Size(W, H);
            boolean ok = false;
            StringBuilder sz = new StringBuilder();
            Size[] ss = map.getOutputSizes(ImageFormat.YUV_420_888);
            if (ss != null) for (Size s : ss) { sz.append(s).append(' '); ok |= s.equals(want); }
            meta.put("yuv_size_offered", ok).put("yuv_sizes", sz.toString().trim());
            if (ok) {
                long md = map.getOutputMinFrameDuration(ImageFormat.YUV_420_888, want);
                meta.put("yuv_min_frame_duration_ns", md);
                if (md > 1_000_000_000L / fps + 100_000L) status.show("WARNING: YUV " + want + " min frame " + md / 1e6 + " ms > 1/" + fps);
            } else status.show("WARNING: YUV " + want + " not offered by the camera");
        } catch (Exception e) { try { meta.put("stream_offer_error", e.toString()); } catch (Exception ignored) { } }
    }

    private CameraCaptureSession.CaptureCallback capCb;

    private static boolean has(int[] a, int v) { if (a == null) return false; for (int x : a) if (x == v) return true; return false; }

    private void meter(TotalCaptureResult res, Handler h) {
        Long e = res.get(CaptureResult.SENSOR_EXPOSURE_TIME);
        Integer is = res.get(CaptureResult.SENSOR_SENSITIVITY);
        if (e == null || is == null || e <= 0 || is <= 0) return;
        if (++meterFrames < 5) return;
        meterEv += e * (double) is; meterExp += e; meterN++;
        if (meterFrames < 15 || sess == null || reqB == null) return;
        try {
            android.util.Range<Integer> sr = ((CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE))
                    .getCameraCharacteristics("2").get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            int isoMax = sr != null ? sr.getUpper() : 3200, isoMin = sr != null ? sr.getLower() : 50;
            double ev = meterEv / meterN, aeExp = meterExp / meterN;
            long exp = (long) Math.min(expCapNs, aeExp);
            if (ev / exp > isoMax) exp = (long) (ev / isoMax);
            int is2 = (int) Math.max(isoMin, Math.min(isoMax, Math.round(ev / Math.max(1, exp))));
            reqB.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            reqB.set(CaptureRequest.SENSOR_EXPOSURE_TIME, exp);
            reqB.set(CaptureRequest.SENSOR_SENSITIVITY, is2);
            reqB.set(CaptureRequest.SENSOR_FRAME_DURATION, 33_333_333L);
            reqB.set(CaptureRequest.CONTROL_AWB_LOCK, true);
            sess.setRepeatingRequest(reqB.build(), capCb, h);
            fixExp = exp; fixIso = is2; isoLo = isoMin; isoHi = isoMax;
            manualSet = true;
            android.util.Log.i("arlat", "short exposure " + exp / 1000 + " us iso " + is2 + " (AE " + (long) aeExp / 1000 + " us)");
        } catch (Exception ex) {
            android.util.Log.w("arlat", "short exposure failed", ex);
            manualSet = true;
        }
    }

    // lens2: adaptive brightness on the short exposure: ISO follows the 95th percentile of Y toward 215 (no clipped highlights)
    private long fixExp; private int fixIso, isoLo = 50, isoHi = 3200, abFrames;
    private final int[] abHist = new int[256];

    private void autoBrightness(byte[] buf, int off) {
        for (int y = H / 16; y < H; y += H / 24) for (int x = W / 32; x < W; x += W / 32) abHist[buf[off + y * W + x] & 0xff]++;
        if (++abFrames < 15) return;
        int total = 0; for (int v : abHist) total += v;
        int acc = 0, p95 = 255; for (int v = 0; v < 256; v++) { acc += abHist[v]; if (acc >= 0.95 * total) { p95 = v; break; } }
        java.util.Arrays.fill(abHist, 0); abFrames = 0;
        double ratio = p95 >= 254 ? 0.75 : 215.0 / Math.max(8, p95);
        if (Math.abs(1 - ratio) < 0.06) return;
        ratio = Math.max(0.8, Math.min(1.25, ratio));
        double want = fixIso * ratio; long exp = fixExp; int is2;
        if (want > isoHi) { is2 = isoHi; exp = Math.min(Math.max(expCapNs, fixExp), (long) (fixExp * want / isoHi)); }
        else if (want < isoLo) { is2 = isoLo; exp = Math.max(100_000L, (long) (fixExp * want / isoLo)); }
        else is2 = (int) Math.round(want);
        if (is2 == fixIso && exp == fixExp) return;
        fixIso = is2; fixExp = exp;
        try {
            reqB.set(CaptureRequest.SENSOR_EXPOSURE_TIME, exp);
            reqB.set(CaptureRequest.SENSOR_SENSITIVITY, is2);
            sess.setRepeatingRequest(reqB.build(), capCb, camH);
        } catch (Exception ex) { android.util.Log.w("arlat", "auto brightness failed", ex); }
    }

    /** temps.csv every 2 s; battery >= maxBatteryTenthC or thermal status >= SEVERE -> stop (onAutoStop). */
    private final Runnable heatTick = new Runnable() {
        @Override public void run() {
            if (!running) return;
            int b = batteryTenthC();
            int ts = thermalStatus();
            float hr = Float.NaN;
            try { hr = ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getThermalHeadroom(10); } catch (Throwable ignored) { }
            long now = SystemClock.elapsedRealtimeNanos();
            try {
                tempsW.write(now + "," + String.format(Locale.US, "%.1f", (now - t0Elapsed) / 1e9) + "," + b / 10.0 + "," + ts + "," + hr + "\n");
                tempsW.flush();
            } catch (Exception ignored) { }
            String why = null;
            if (b >= maxBatteryTenthC) why = "battery " + b / 10.0 + " C";
            else if (ts >= maxThermal) why = "thermal status " + ts;
            if (why != null && onAutoStop != null) {
                stopReason = why;
                status.show("STOP: " + why);
                onAutoStop.run();
                return;
            }
            if (camH != null) camH.postDelayed(this, 2000);
        }
    };

    int batteryTenthC() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi == null ? 0 : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
    }

    int thermalStatus() {
        try { return ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus(); }
        catch (Throwable e) { return -1; }
    }

    void stop() {
        if (!running && proc == null) return;
        running = false;
        try { ((SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE)).unregisterListener(this); } catch (Exception ignored) { }
        if (dev != null) { dev.close(); dev = null; }
        if (camThread != null) camThread.quitSafely();
        if (imuThread != null) imuThread.quitSafely();
        try { if (writerT != null) writerT.join(500); } catch (InterruptedException ignored) { }
        if (proc != null && writerT != null && writerT.isAlive()) proc.destroy();   // writer stuck in a 2 MB write: closing stdin would block the UI thread
        if (proc != null) {
            // EOF on stdin -> the bridge returns from main (closes bridge_timing.csv / ov_timing.txt), then kill if needed
            try { proc.getOutputStream().close(); } catch (Exception ignored) { }
            try { if (!proc.waitFor(1500, java.util.concurrent.TimeUnit.MILLISECONDS)) proc.destroy(); } catch (Exception ignored) { proc.destroy(); }
            try { meta.put("bridge_exit", proc.isAlive() ? "killed" : String.valueOf(proc.exitValue())); } catch (Exception ignored) { }
            proc = null;
        }
        try { if (readerT != null) readerT.join(500); } catch (InterruptedException ignored) { }
        synchronized (framesLock) { closeQuiet(framesW); framesW = null; }
        synchronized (posesLock) { closeQuiet(posesW); posesW = null; closeQuiet(eventsW); eventsW = null; }
        closeQuiet(tempsW);
        if (reader != null) { try { reader.close(); } catch (Exception ignored) { } reader = null; }
        try {
            meta.put("end_elapsed_ns", SystemClock.elapsedRealtimeNanos()).put("secs", (SystemClock.elapsedRealtimeNanos() - t0Elapsed) / 1e9)
                    .put("frames", nCam).put("poses", nPose).put("imu_records", nImu).put("acc_samples", nAcc)
                    .put("capture_results", nResults).put("pool_dropped", poolDropped).put("queue_dropped", camDropped)
                    .put("battery_c_end", batteryTenthC() / 10.0).put("thermal_status_end", thermalStatus())
                    .put("stop_reason", stopReason).put("streamer_dropped", out.dropped);
            if (runDir != null) java.nio.file.Files.write(new File(runDir, "meta.json").toPath(), meta.toString(1).getBytes(StandardCharsets.UTF_8));
        } catch (Exception e) { android.util.Log.e("arlat", "ov meta", e); }   // meta.json last = all files complete
        android.util.Log.i("arlat", "ov stopped (" + stopReason + ") -> " + runDir);
    }

    private static void closeQuiet(java.io.Closeable c) { try { if (c != null) c.close(); } catch (Exception ignored) { } }

    private void onImage(ImageReader r) {
        long now = SystemClock.elapsedRealtimeNanos();
        Image im = r.acquireLatestImage();
        if (im == null) return;
        long ts = im.getTimestamp();
        long tSend = ts + midOffsetNs;
        int fate = 0;
        long copyUs = 0;
        byte[] buf = pool == null ? null : pool.poll();
        if (buf == null) {
            fate = 1; poolDropped++;
        } else {
            long c0 = System.nanoTime();
            Image.Plane y = im.getPlanes()[0];
            ByteBuffer b = y.getBuffer();
            int rs = y.getRowStride();
            ByteBuffer rec = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN);
            rec.putInt(MAGIC).putInt(2).putInt(20 + W * H);
            rec.putLong(tSend).putInt(W).putInt(H).put((byte) 1).put((byte) 0).put((byte) 0).put((byte) 0);
            if (rs == W) { b.position(0); b.get(buf, 32, W * H); }
            else for (int i = 0; i < H; i++) { b.position(i * rs); b.get(buf, 32 + i * W, W); }
            copyUs = (System.nanoTime() - c0) / 1000;
            if (!isMain() && manualSet && fixIso > 0) autoBrightness(buf, 32);
            if (nCam % 150 == 0 && (!isMain() || nCam <= 600)) {      // raw Y snapshot (main: 4 only, a 2 MB write stalls the cam thread) for checking field of view / blur on the PC (<WxH> 8-bit)
                File snapDir = isMain() ? runDir : dir;
                try (FileOutputStream fo = new FileOutputStream(new File(snapDir, "frame_" + (nCam / 150 % 4) + ".y"))) {
                    fo.write(buf, 32, W * H);
                } catch (Exception ignored) { }
            }
            sent.put(tSend, new long[]{ts, now});
            if (sent.size() > 300) { final long old = tSend - 2_000_000_000L; sent.keySet().removeIf(k -> k < old); }
            if (!toBridge.offer(buf)) { fate = 2; camDropped++; pool.offer(buf); }
        }
        im.close();
        synchronized (framesLock) {
            try {
                if (framesW != null) framesW.write(nCam + "," + ts + "," + now + "," + tSend + "," + copyUs + "," + fate + "," + expNs + "," + lastFocus + "\n");
            } catch (Exception ignored) { }
        }
        nCam++;
    }

    /**
     * One IMU record per gyro sample with the accelerometer interpolated at the gyro time.
     * S24 delivers the accelerometer in bursts (~every 12 ms, several samples at once) while the gyro streams, so a gyro
     * sample is held until an accel sample at or after its time has arrived, then interpolated between the bracketing
     * accel samples. (2026-10-18: keeping only the last two accel samples froze accel for ~6 gyro samples at a time —
     * 16 % unique values — and OpenVINS drifted off by tens of metres; the same stream replayed on the PC did the same.)
     * Uncalibrated sensors: values[0..2] only (raw; Android's bias estimate in 3..5 is not applied) = bench --imu-raw.
     */
    private final java.util.ArrayDeque<long[]> gyroQ = new java.util.ArrayDeque<>();      // {t, bits(x), bits(y), bits(z)}
    private final java.util.ArrayList<float[]> accV = new java.util.ArrayList<>();
    private final java.util.ArrayList<Long> accT = new java.util.ArrayList<>();

    @Override public void onSensorChanged(SensorEvent e) {
        if (e.sensor.getType() == accType) {
            if (!accT.isEmpty() && e.timestamp <= accT.get(accT.size() - 1)) return;
            accT.add(e.timestamp);
            accV.add(new float[]{e.values[0], e.values[1], e.values[2]});
            nAcc++;
        } else {
            gyroQ.add(new long[]{e.timestamp, Float.floatToIntBits(e.values[0]), Float.floatToIntBits(e.values[1]),
                    Float.floatToIntBits(e.values[2])});
        }
        flushImu();
    }

    private void flushImu() {
        if (accT.isEmpty()) { gyroQ.clear(); return; }
        long accNewest = accT.get(accT.size() - 1);
        while (!gyroQ.isEmpty()) {
            long[] g = gyroQ.peekFirst();
            long tg = g[0];
            // wait for the accel that brackets this gyro sample; give up waiting after 50 ms (accel stalled)
            if (tg > accNewest && gyroQ.peekLast()[0] - tg < 50_000_000L) break;
            gyroQ.pollFirst();
            float[] a;
            int k = 0;
            while (k + 1 < accT.size() && accT.get(k + 1) <= tg) k++;    // accT[k] <= tg < accT[k+1] when possible
            if (tg <= accT.get(0)) a = accV.get(0);
            else if (k + 1 >= accT.size()) a = accV.get(accT.size() - 1);
            else {
                double f = (double) (tg - accT.get(k)) / (accT.get(k + 1) - accT.get(k));
                float[] a0 = accV.get(k), a1 = accV.get(k + 1);
                a = new float[]{(float) (a0[0] + f * (a1[0] - a0[0])), (float) (a0[1] + f * (a1[1] - a0[1])),
                        (float) (a0[2] + f * (a1[2] - a0[2]))};
            }
            // drop accel samples no later gyro can need (keep the one at or before tg)
            if (k > 0) { accT.subList(0, k).clear(); accV.subList(0, k).clear(); }
            ByteBuffer rec = ByteBuffer.allocate(12 + 56).order(ByteOrder.LITTLE_ENDIAN);
            rec.putInt(MAGIC).putInt(1).putInt(56).putLong(tg);
            rec.putDouble(Float.intBitsToFloat((int) g[1])).putDouble(Float.intBitsToFloat((int) g[2]))
                    .putDouble(Float.intBitsToFloat((int) g[3]));
            rec.putDouble(a[0]).putDouble(a[1]).putDouble(a[2]);
            toBridge.offer(rec.array());
            nImu++;
        }
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    /** seconds of bridge input to also save as files/ov/live_stream.bin (0 = off) — for replaying the live run on the PC */
    int dumpSecs;

    private void writer() {
        OutputStream dump = null;
        long dumpEnd = 0;
        try {
            if (dumpSecs > 0) {
                dump = new java.io.BufferedOutputStream(new FileOutputStream(new File(dir, "live_stream.bin")), 1 << 20);
                dumpEnd = SystemClock.elapsedRealtime() + dumpSecs * 1000L;
            }
            OutputStream o = proc.getOutputStream();
            while (running) {
                byte[] r = toBridge.poll(100, java.util.concurrent.TimeUnit.MILLISECONDS);
                if (r == null) continue;
                o.write(r);
                if (dump != null) {
                    dump.write(r);
                    if (SystemClock.elapsedRealtime() > dumpEnd) { dump.close(); dump = null; status.show("dump done"); }
                }
                if (r.length > 68 && pool != null) pool.offer(r);    // image buffer back to the pool
                if (toBridge.isEmpty()) o.flush();
            }
            if (dump != null) dump.close();
        } catch (Exception e) {
            if (running) status.show("writer: " + e);
        }
    }

    private void readerLoop() {
        try {
            InputStream in = proc.getInputStream();
            BufferedReader br = new BufferedReader(new InputStreamReader(in));
            String l;
            while ((l = br.readLine()) != null) {
                long tOut = SystemClock.elapsedRealtimeNanos();
                if (!l.startsWith("{")) continue;
                JSONObject j = new JSONObject(l);
                String type = j.optString("type");
                if (!"pose".equals(type)) {
                    JSONObject ev = new JSONObject().put("src", "ov").put("event", j);
                    out.send(ev.toString());
                    synchronized (posesLock) { if (eventsW != null) { eventsW.write(tOut + " " + l + "\n"); eventsW.flush(); } }
                    continue;
                }
                long tNs = j.optLong("t_ns");
                long[] s = sent.remove(tNs);
                JSONObject o = new JSONObject();
                o.put("src", "ov").put("t_out", tOut).put("status", j.optString("status"));
                if (s != null) o.put("t_sensor", s[0]).put("t_arrive", s[1]);
                if (j.has("p")) o.put("p", j.getJSONArray("p"));
                if (j.has("q")) o.put("q", j.getJSONArray("q"));
                o.put("ms", j.optDouble("ms", 0)).put("n_slam", j.optInt("n_slam")).put("n_msckf", j.optInt("n_msckf"));
                o.put("dt_cam_imu", j.optDouble("dt_cam_imu", 0)).put("cam_q_drop", camDropped).put("pool_drop", poolDropped);
                o.put("exp_ms", expNs / 1e6).put("mid_ms", midOffsetNs / 1e6).put("n_imu", nImu).put("n_acc", nAcc).put("n_cam", nCam);
                out.send(o.toString());
                synchronized (posesLock) {
                    if (posesW != null) posesW.write(tNs + "," + (s == null ? -1 : s[0]) + "," + (s == null ? -1 : s[1]) + "," + tOut + ","
                            + j.optString("status") + "," + j.optDouble("ms", 0) + "," + j.optDouble("remap_ms", 0) + ","
                            + j.optInt("n_slam") + "," + j.optInt("n_msckf") + "," + j.optLong("cam_dropped") + "," + poolDropped + ","
                            + camDropped + "," + nCam + "," + nImu + "," + j.optDouble("dt_cam_imu", 0) + "\n");
                }
                lastStatus = j.optString("status");
                if (++nPose % 15 == 0) {
                    status.show(String.format(Locale.US, "OpenVINS %s (%s %dx%d@%d %s)\ncam %d imu %d pose %d drop %d/%d\nlatency %.0f ms  ov %.1f ms\nbattery %.1f C\n%s",
                            lastStatus, cam, W, H, fps, cfgName, nCam, nImu, nPose, poolDropped, camDropped,
                            s == null ? -1.0 : (tOut - s[0]) / 1e6, j.optDouble("ms", 0), batteryTenthC() / 10.0,
                            runDir == null ? "" : runDir.getName()));
                }
            }
            if (running) status.show("bridge exited (see " + (runDir == null ? "files/ov" : runDir.getName()) + "/bridge_stderr.log)");
        } catch (Exception e) {
            if (running) status.show("reader: " + e);
        }
    }
}
