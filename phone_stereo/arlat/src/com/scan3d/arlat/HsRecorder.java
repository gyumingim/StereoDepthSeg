package com.scan3d.arlat;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Range;
import android.util.Size;
import android.view.Surface;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ConcurrentHashMap;

/**
 * 120 fps (or the fastest high-speed mode) recording of the main camera for ChArUco ground truth (scan3d IMU_PLAN.md):
 * Android constrained high-speed session -> H.264 encoder surface -> files/hs_<time>/cam5.mp4, plus frames_cam5.csv
 * (frame_idx, t_mid_ns, physical_ts_ns, logical_ts_ns, exposure_ns, rolling_shutter_skew_ns, pts_us — same columns as
 * the scanner recorder), imu.csv (type, t_ns, v0..v2 for accelerometer 1 / gyroscope 4 at the fastest rate) and
 * meta.json, so scan3d/imu_learn/imu_gt.py reads it unchanged. High-speed sessions only feed encoder/preview surfaces
 * (no CPU frames), hence record-only. Exposure is the camera's own AE (<= 1/fps).
 * Since VERSION hs-20261020: imu.csv also has raw types 16/35 (8 columns), sensors run on their own thread,
 * temps.csv (battery + thermal status/headroom + whatever thermal HAL / sysfs source this app may read; every 10 s and
 * when a value changes), results_cam5.csv has the actual OIS/EIS/NR/edge/distortion state per frame (columns at the end),
 * and Samsung lens distortion correction is requested off. meta.json: app_version, temp_sources, capture.
 */
class HsRecorder implements SensorEventListener {
    interface Status { void show(String s); }
    static final String VERSION = "hs-20261020-nroff-ae";       // written to meta.json app_version + status line (nroff: NR/edge OFF requested; ae: AE/ISO/AWB/colour logged)

    static boolean has(int[] modes, int m) {
        if (modes == null) return false;
        for (int x : modes) if (x == m) return true;
        return false;
    }
    static final String LDC_TAG = "samsung.android.control.lensDistortionCorrectionMode";

    private final Context ctx;
    private final Status status;
    private HandlerThread th;
    private Handler h;
    private CameraDevice dev;
    private CameraConstrainedHighSpeedCaptureSession sess;
    private MediaCodec enc;
    private MediaMuxer mux;
    private int track = -1;
    private Surface encSurface;
    Surface previewSurface;                      // optional 2nd output (HsPreview: on-screen view + board check)
    volatile String stopReason = "user";
    int maxBatteryTenthC = 460;                  // stop when the battery reaches 46.0 C or Android reports SEVERE throttling
    private File dir;
    private BufferedWriter imuW, frW, resW;     // resW: every capture result (offline check of the pts -> sensor ts match)
    private volatile boolean running;
    private final ConcurrentHashMap<Long, long[]> meta = new ConcurrentHashMap<>();   // sensor ts -> {exp, skew}
    private int nFrames, nImu, nRes, nUnmatched;
    private long t0Elapsed, t0Wall;
    private long clockOffset;                    // BOOTTIME - MONOTONIC (ns)
    private long lastDelta, lastExp, lastSkew;
    int wantFps = 120;
    int evComp = 0;
    float focusDpt = -2;                         // >= 0: fixed focus at this many diopters (1/m); -1: autofocus all along;
                                                 // -2: autofocus for 1.5 s then lock there (aim at the board when starting)
    private volatile float lastFocus, focusLocked = -1;
    private long focusLockNs;
    int wantKbps = 160_000;                     // S24 1080p240: 160 Mbps no dropped frames, 240 Mbps (encoder max) dropped 7 %; 0 = max
    int bitrate;                              // AE exposure compensation steps (high-speed sessions allow only AE controls)
    Size size;
    Range<Integer> range;
    private CameraCharacteristics chars;
    // IMU on its own thread (the hs thread also serves the encoder + 240 capture results/s); writes under imuLock
    private HandlerThread imuTh;
    private final Object imuLock = new Object();
    // temperatures (temps.csv): own thread, tick every 2 s, a row every 10 s or when a value changes
    private HandlerThread tempTh;
    private Handler tempH;
    private BufferedWriter tempW;
    private final Object tempLock = new Object();
    private final JSONObject tempSources = new JSONObject();
    private Object thermalSvc;                     // IThermalService (hidden; reflection), null if not reachable
    private java.lang.reflect.Method thermalGet;   // getCurrentTemperatures()
    private final List<String[]> zones = new ArrayList<>();   // readable /sys/class/thermal zones {type, temp path}
    private Object hpm;                            // HardwarePropertiesManager if getDeviceTemperatures is allowed
    private String lastTempKey = "";
    private long lastTempRowNs;
    private int nTemps;
    private Object thermalListener;                // PowerManager.OnThermalStatusChangedListener (API 29)
    static final long TEMP_TICK_MS = 2000, TEMP_ROW_NS = 10_000_000_000L;
    static final String[] HAL_COLS = {"AP", "BAT", "CP", "PA", "SKIN", "USB"};
    // capture state: requested vendor/AOSP keys and how the per-frame vendor result is read
    private final JSONObject capInfo = new JSONObject();
    private CaptureRequest.Key<?> ldcReqKey;       // set in the request (null if the framework does not know the tag)
    private CaptureResult.Key<?> ldcResKey;
    private boolean ldcResTried;
    private final ConcurrentHashMap<Long, Long> readoutTs = new ConcurrentHashMap<>();   // frame number -> readout start ns

    HsRecorder(Context ctx, Status status) { this.ctx = ctx; this.status = status; }

    /** Pick the high-speed size/fps: the largest size offering wantFps (fixed range [f,f]). */
    String choose() throws Exception {
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        CameraCharacteristics ch = cm.getCameraCharacteristics("0");
        chars = ch;
        StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        StringBuilder all = new StringBuilder();
        Size best = null; Range<Integer> bestR = null;
        for (Size s : map.getHighSpeedVideoSizes()) {
            for (Range<Integer> r : map.getHighSpeedVideoFpsRangesFor(s)) {
                all.append(s).append('@').append(r).append(' ');
                if (r.getLower() == wantFps && r.getUpper() == wantFps
                        && (best == null || s.getWidth() * s.getHeight() > best.getWidth() * best.getHeight())) { best = s; bestR = r; }
            }
        }
        size = best; range = bestR;
        android.util.Log.i("arlat", "high-speed configs: " + all);
        return all.toString();
    }

    void start() throws Exception {
        String cfgs = choose();
        if (size == null) throw new IllegalStateException("no " + wantFps + " fps high-speed mode: " + cfgs);
        running = true;
        th = new HandlerThread("hs"); th.start(); h = new Handler(th.getLooper());
        dir = new File(ctx.getExternalFilesDir(null), "hs_" + new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date()));
        dir.mkdirs();
        imuW = new BufferedWriter(new FileWriter(new File(dir, "imu.csv")), 1 << 16);
        imuW.write("type,t_ns,v0,v1,v2,v3,v4,v5\n");
        frW = new BufferedWriter(new FileWriter(new File(dir, "frames_cam5.csv")), 1 << 16);
        resW = new BufferedWriter(new FileWriter(new File(dir, "results_cam5.csv")), 1 << 16);
        // new columns only at the end (old readers use the first 8 by name/position)
        resW.write("sensor_ts_ns,exposure_ns,rolling_shutter_skew_ns,frame_number,arrive_elapsed_ns,focus_dpt,af_mode,lens_moving"
                + ",ois_mode,eis_mode,nr_mode,edge_mode,readout_ts_ns,distortion_mode,samsung_ldc_mode"
                + ",ae_mode,ae_state,ae_comp,iso,post_raw_boost,frame_duration_ns,awb_mode,awb_state,cc_mode,awb_gains_rggb,ccm_rowmajor,tonemap_mode,scene_mode,effect_mode\n");
        frW.write("frame_idx,t_mid_ns,physical_ts_ns,logical_ts_ns,exposure_ns,rolling_shutter_skew_ns,pts_us\n");
        t0Elapsed = SystemClock.elapsedRealtimeNanos(); t0Wall = System.currentTimeMillis();
        clockOffset = SystemClock.elapsedRealtimeNanos() - System.nanoTime();
        startTemps();
        imuTh = new HandlerThread("hs-imu", android.os.Process.THREAD_PRIORITY_DISPLAY); imuTh.start();
        Handler imuH = new Handler(imuTh.getLooper());
        SensorManager sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST, imuH);
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST, imuH);
        // raw streams too (type 16 / 35: values 0-2 = uncalibrated, 3-5 = Android's current bias estimate) — we calibrate ourselves
        Sensor gu = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), au = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        if (gu != null) sm.registerListener(this, gu, SensorManager.SENSOR_DELAY_FASTEST, imuH);
        if (au != null) sm.registerListener(this, au, SensorManager.SENSOR_DELAY_FASTEST, imuH);

        MediaFormat f = MediaFormat.createVideoFormat("video/avc", size.getWidth(), size.getHeight());
        f.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface);
        // bitrate: the encoder's maximum for this size/rate unless 'kbps' asks for less. 40 Mbps at 1080p240 was
        // ~0.08 bit/pixel (8x the compression of the 60 fps recorder) and blurs marker corners.
        String encName = new android.media.MediaCodecList(android.media.MediaCodecList.REGULAR_CODECS).findEncoderForFormat(f);
        int maxBps = 40_000_000;
        try {
            MediaCodecInfo.VideoCapabilities vc = null;      // read caps from the list (creating a codec here starved the real one)
            for (MediaCodecInfo ci : new android.media.MediaCodecList(android.media.MediaCodecList.REGULAR_CODECS).getCodecInfos())
                if (ci.getName().equals(encName)) vc = ci.getCapabilitiesForType("video/avc").getVideoCapabilities();
            maxBps = vc.getBitrateRange().getUpper();
            android.util.Log.i("arlat", "hs encoder " + encName + " bitrate max " + maxBps + " size/rate ok "
                    + vc.areSizeAndRateSupported(size.getWidth(), size.getHeight(), wantFps));
        } catch (Exception e) { android.util.Log.w("arlat", "encoder caps", e); }
        bitrate = wantKbps > 0 ? Math.min(maxBps, wantKbps * 1000) : maxBps;
        f.setInteger(MediaFormat.KEY_BIT_RATE, bitrate);
        f.setInteger(MediaFormat.KEY_FRAME_RATE, wantFps);
        f.setInteger(MediaFormat.KEY_OPERATING_RATE, wantFps);
        f.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1);
        enc = encName != null ? MediaCodec.createByCodecName(encName) : MediaCodec.createEncoderByType("video/avc");
        enc.setCallback(new MediaCodec.Callback() {
            @Override public void onInputBufferAvailable(MediaCodec c, int i) { }
            @Override public void onOutputBufferAvailable(MediaCodec c, int i, MediaCodec.BufferInfo info) {
                try {
                    ByteBuffer b = c.getOutputBuffer(i);
                    if ((info.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) == 0 && info.size > 0 && track >= 0) {
                        mux.writeSampleData(track, b, info);
                        // encoder pts is CLOCK_MONOTONIC (no deep sleep) while SENSOR_TIMESTAMP and the IMU are
                        // BOOTTIME (elapsedRealtimeNanos): measured 4189 s apart on the S24 (2026-10-19). Shift pts by
                        // (boottime - monotonic) and take the exact sensor timestamp of the matching capture result.
                        long guess = info.presentationTimeUs * 1000L + clockOffset;
                        long[] m = nearestEntry(guess);
                        long ts;
                        if (m != null) { ts = m[2]; lastDelta = m[2] - guess; lastExp = m[0]; lastSkew = m[1]; }
                        else { ts = guess + lastDelta; nUnmatched++; }   // result not here yet: same pts->sensor shift as the last match
                        long exp = m == null ? lastExp : m[0], skew = m == null ? lastSkew : m[1];
                        frW.write(nFrames + "," + (ts + exp / 2 + skew / 2) + "," + ts + "," + ts + "," + exp + "," + skew + "," + info.presentationTimeUs + "\n");
                        nFrames++;
                        if (nFrames % 120 == 0) status.show(String.format(Locale.US, "HS %s@%d recording\nframes %d imu %d\n%s",
                                size, wantFps, nFrames, nImu, dir.getName()));
                    }
                    c.releaseOutputBuffer(i, false);
                    if ((info.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) finish();
                } catch (Exception e) { android.util.Log.e("arlat", "hs enc", e); }
            }
            @Override public void onError(MediaCodec c, MediaCodec.CodecException e) { status.show("encoder error " + e); }
            @Override public void onOutputFormatChanged(MediaCodec c, MediaFormat fmt) {
                track = mux.addTrack(fmt); mux.start();
            }
        }, h);
        enc.configure(f, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
        encSurface = enc.createInputSurface();
        mux = new MediaMuxer(new File(dir, "cam5.mp4").getAbsolutePath(), MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
        enc.start();

        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                dev = d;
                try {
                    d.createConstrainedHighSpeedCaptureSession(previewSurface == null ? Arrays.asList(encSurface) : Arrays.asList(encSurface, previewSurface), new CameraCaptureSession.StateCallback() {
                        @Override public void onConfigured(CameraCaptureSession s) {
                            sess = (CameraConstrainedHighSpeedCaptureSession) s;
                            try {
                                CaptureRequest.Builder rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
                                rb.addTarget(encSurface);
                                if (previewSurface != null) rb.addTarget(previewSurface);
                                rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, range);
                                rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                                rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                                requestDistortionOff(rb);
                                // noise reduction / edge enhancement OFF (TEMPLATE_RECORD defaults to FAST) — only if the camera offers OFF
                                if (has(chars.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES), CaptureRequest.NOISE_REDUCTION_MODE_OFF))
                                    rb.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
                                if (has(chars.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CaptureRequest.EDGE_MODE_OFF))
                                    rb.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
                                if (evComp != 0) rb.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, evComp);   // darker = shorter exposure, if AE obeys
                                if (focusDpt >= 0) {        // fixed focus: autofocus moves the lens and changes the intrinsics mid-recording
                                    rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                                    rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
                                }
                                final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                                    // API 34+: sensor readout start (only called if CameraCharacteristics.SENSOR_READOUT_TIMESTAMP = HARDWARE)
                                    @Override public void onReadoutStarted(CameraCaptureSession ss, CaptureRequest q, long ts, long frameNumber) {
                                        readoutTs.put(frameNumber, ts);
                                    }
                                    @Override public void onCaptureCompleted(CameraCaptureSession ss, CaptureRequest q, TotalCaptureResult r) {
                                        Long ts = r.get(CaptureResult.SENSOR_TIMESTAMP), e = r.get(CaptureResult.SENSOR_EXPOSURE_TIME),
                                                sk = r.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                                        Float fd = r.get(CaptureResult.LENS_FOCUS_DISTANCE);
                                        Integer afm = r.get(CaptureResult.CONTROL_AF_MODE), lst = r.get(CaptureResult.LENS_STATE);
                                        try { resW.write(ts + "," + e + "," + sk + "," + r.getFrameNumber() + "," + SystemClock.elapsedRealtimeNanos()
                                                + "," + fd + "," + afm + "," + lst + "," + captureState(r) + "\n"); }
                                        catch (Exception ignored) { }
                                        if (ts != null) meta.put(ts, new long[]{e == null ? 0 : e, sk == null ? 0 : sk, ts});
                                        if (++nRes <= 3) android.util.Log.i("arlat", "hs result ts " + ts + " exp " + e + " skew " + sk);
                                        // drop only stale entries (> 1 s old); clearing the whole map lost pending matches (525 frames per 134 s)
                                        if (ts != null && meta.size() > 2000) meta.keySet().removeIf(k -> k < ts - 1_000_000_000L);
                                        if (fd != null) lastFocus = fd;
                                    }
                                };
                                try { sess.setRepeatingBurst(sess.createHighSpeedRequestList(rb.build()), cb, h); }
                                catch (Exception be) {      // never lose the recording to the vendor tag: retry without it
                                    if (ldcReqKey == null) throw be;
                                    android.util.Log.w("arlat", "hs burst with " + LDC_TAG + " failed, retry without", be);
                                    rb.set((CaptureRequest.Key<Object>) ldcReqKey, null);
                                    ldcReqKey = null;
                                    try { capInfo.put("samsung_ldc_request", "set but burst failed (" + be + ") -> removed"); } catch (Exception ignored) { }
                                    sess.setRepeatingBurst(sess.createHighSpeedRequestList(rb.build()), cb, h);
                                }
                                if (focusDpt == -2) h.postDelayed(() -> lockFocus(rb, cb), 1500);   // autofocus 1.5 s, then hold it
                                h.postDelayed(heatCheck, 2000);
                                status.show("HS " + size + "@" + wantFps + " recording -> " + dir.getName() + "\n" + VERSION);
                            } catch (Exception e) { status.show("hs request: " + e); }
                        }
                        @Override public void onConfigureFailed(CameraCaptureSession s) { status.show("hs configure failed"); }
                    }, h);
                } catch (Exception e) { status.show("hs session: " + e); }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.show("camera error " + e); d.close(); }
        }, h);
    }

    /** Ask for no distortion correction: AOSP DISTORTION_CORRECTION_MODE OFF (if offered) and the Samsung vendor tag = 0.
     *  Every step is guarded (unknown tag -> IllegalArgumentException from the framework) and the outcome goes to meta.json. */
    @SuppressWarnings("unchecked")
    private void requestDistortionOff(CaptureRequest.Builder rb) {
        try {
            JSONArray rq = new JSONArray(), rs = new JSONArray();
            for (CaptureRequest.Key<?> k : chars.getAvailableCaptureRequestKeys())
                if (k.getName().toLowerCase(Locale.US).contains("distortion")) rq.put(k.getName());
            for (CaptureResult.Key<?> k : chars.getAvailableCaptureResultKeys())
                if (k.getName().toLowerCase(Locale.US).contains("distortion")) rs.put(k.getName());
            capInfo.put("request_keys_distortion", rq).put("result_keys_distortion", rs);
        } catch (Exception e) { try { capInfo.put("keys_error", e.toString()); } catch (Exception ignored) { } }
        try {
            capInfo.put("readout_timestamp", android.os.Build.VERSION.SDK_INT >= 34
                    ? String.valueOf(chars.get(CameraCharacteristics.SENSOR_READOUT_TIMESTAMP)) + " (1 = HARDWARE -> readout_ts_ns column)" : "API < 34");
        } catch (Throwable e) { try { capInfo.put("readout_timestamp", "error " + e); } catch (Exception ignored) { } }
        try {
            int[] dm = chars.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
            boolean off = false;
            if (dm != null) for (int m : dm) off |= m == CaptureRequest.DISTORTION_CORRECTION_MODE_OFF;
            if (off) rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
            capInfo.put("aosp_distortion_modes", dm == null ? "none" : Arrays.toString(dm)).put("aosp_distortion_request", off ? "OFF" : "not offered");
        } catch (Exception e) { try { capInfo.put("aosp_distortion_request", "error " + e); } catch (Exception ignored) { } }
        String how = "unknown tag";
        try {
            CaptureRequest.Key<Integer> k = new CaptureRequest.Key<>(LDC_TAG, Integer.class);
            rb.set(k, 0); ldcReqKey = k; how = "set 0 (int32)";
        } catch (Throwable e1) {
            try {
                CaptureRequest.Key<Byte> k = new CaptureRequest.Key<>(LDC_TAG, Byte.class);
                rb.set(k, (byte) 0); ldcReqKey = k; how = "set 0 (byte)";
            } catch (Throwable e2) { how = "not set: " + e1; }
        }
        try { capInfo.put("samsung_ldc_request", how); } catch (Exception ignored) { }
        android.util.Log.i("arlat", "hs distortion request: " + capInfo);
    }

    /** Actual per-frame state (results_cam5.csv end columns): OIS, EIS, NR, edge, readout ts (API 34+), AOSP / Samsung distortion. */
    private String captureState(CaptureResult r) {
        Long readout = readoutTs.remove(r.getFrameNumber());      // from onReadoutStarted (API 34+), null otherwise
        if (readoutTs.size() > 1000) readoutTs.clear();
        Object ldc = null;
        if (!ldcResTried) {                     // find a type the framework accepts for the vendor tag, once
            ldcResTried = true;
            for (Class<?> c : new Class<?>[]{Integer.class, Byte.class}) {
                try {
                    CaptureResult.Key<?> k = new CaptureResult.Key<>(LDC_TAG, c);
                    ldc = r.get(k); ldcResKey = k;
                    try { capInfo.put("samsung_ldc_result", "readable as " + c.getSimpleName() + ", first value " + ldc); } catch (Exception ignored) { }
                    break;
                } catch (Throwable e) {
                    try { capInfo.put("samsung_ldc_result", "not readable: " + e); } catch (Exception ignored) { }
                }
            }
        } else if (ldcResKey != null) {
            try { ldc = r.get(ldcResKey); } catch (Throwable e) { ldcResKey = null; }
        }
        Object dc = null;
        try { dc = r.get(CaptureResult.DISTORTION_CORRECTION_MODE); } catch (Throwable ignored) { }
        return r.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE) + "," + r.get(CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE)
                + "," + r.get(CaptureResult.NOISE_REDUCTION_MODE) + "," + r.get(CaptureResult.EDGE_MODE)
                + "," + readout + "," + dc + "," + ldc + "," + exposureState(r);
    }

    // AE / ISO / AWB / colour processing actually applied (columns appended after samsung_ldc_mode)
    static String exposureState(CaptureResult r) {
        Object gains = null, ccm = null;
        try {
            android.hardware.camera2.params.RggbChannelVector g = r.get(CaptureResult.COLOR_CORRECTION_GAINS);
            if (g != null) gains = g.getRed() + " " + g.getGreenEven() + " " + g.getGreenOdd() + " " + g.getBlue();
        } catch (Throwable ignored) { }
        try {
            android.hardware.camera2.params.ColorSpaceTransform t = r.get(CaptureResult.COLOR_CORRECTION_TRANSFORM);
            if (t != null) {
                StringBuilder s = new StringBuilder();
                for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) s.append(i + j > 0 ? " " : "").append(t.getElement(j, i).floatValue());
                ccm = s;
            }
        } catch (Throwable ignored) { }
        return r.get(CaptureResult.CONTROL_AE_MODE) + "," + r.get(CaptureResult.CONTROL_AE_STATE) + ","
                + r.get(CaptureResult.CONTROL_AE_EXPOSURE_COMPENSATION) + "," + r.get(CaptureResult.SENSOR_SENSITIVITY) + ","
                + r.get(CaptureResult.CONTROL_POST_RAW_SENSITIVITY_BOOST) + "," + r.get(CaptureResult.SENSOR_FRAME_DURATION) + ","
                + r.get(CaptureResult.CONTROL_AWB_MODE) + "," + r.get(CaptureResult.CONTROL_AWB_STATE) + ","
                + r.get(CaptureResult.COLOR_CORRECTION_MODE) + "," + gains + "," + ccm + ","
                + r.get(CaptureResult.TONEMAP_MODE) + "," + r.get(CaptureResult.CONTROL_SCENE_MODE) + "," + r.get(CaptureResult.CONTROL_EFFECT_MODE);
    }

    // ---------------- temperatures -> temps.csv ----------------
    // Columns phone_uptime_s, pc_unix, battery_dC, thermal_status, cam_on, AP..USB match scan3d/tools/imu_shell temps_all.csv
    // so analyze_static.read_temps() reads it (pc_unix here = the PHONE's wall clock). 'other' = name=value;... for
    // every further readable value (other HAL names, sysfs zones, battery intent extras with "temp" in the key).

    private void startTemps() {
        try {
            tempW = new BufferedWriter(new FileWriter(new File(dir, "temps.csv")));
            tempW.write("elapsed_ns,wall_ms,phone_uptime_s,pc_unix,battery_dC,thermal_status,headroom_10s,cam_on,"
                    + String.join(",", HAL_COLS) + ",reason,other\n");
            tempSources.put("interval_s", TEMP_ROW_NS / 1e9).put("poll_s", TEMP_TICK_MS / 1e3)
                    .put("battery_intent", "yes").put("note", "pc_unix = phone wall clock s; rows also on change (reason=change/status)");
        } catch (Exception e) { android.util.Log.e("arlat", "temps.csv", e); return; }
        android.os.PowerManager pm = (android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE);
        put("thermal_status", "yes (PowerManager.getCurrentThermalStatus)");
        if (android.os.Build.VERSION.SDK_INT >= 30) {
            float hr = pm.getThermalHeadroom(10);
            put("thermal_headroom", Float.isNaN(hr) ? "NaN at start (unsupported or rate limited)" : "yes");
        } else put("thermal_headroom", "no (API < 30)");
        // thermal HAL temperatures (what 'dumpsys thermalservice' prints) through the hidden IThermalService
        try {
            Class<?> smc = Class.forName("android.os.ServiceManager");
            android.os.IBinder b = (android.os.IBinder) smc.getMethod("getService", String.class).invoke(null, "thermalservice");
            if (b == null) throw new IllegalStateException("getService(thermalservice) = null");
            Object svc = Class.forName("android.os.IThermalService$Stub").getMethod("asInterface", android.os.IBinder.class).invoke(null, b);
            java.lang.reflect.Method m = svc.getClass().getMethod("getCurrentTemperatures");
            Object arr = m.invoke(svc);
            thermalSvc = svc; thermalGet = m;
            put("thermal_hal", "yes (" + java.lang.reflect.Array.getLength(arr) + " sensors: " + halRead().keySet() + ")");
        } catch (Throwable e) {
            Throwable c = e instanceof java.lang.reflect.InvocationTargetException && e.getCause() != null ? e.getCause() : e;
            put("thermal_hal", "denied/unreachable: " + c);
        }
        // sysfs thermal zones (usually blocked by SELinux for apps)
        for (int i = 0; i < 100; i++) {
            File d = new File("/sys/class/thermal/thermal_zone" + i);
            try {
                String type = readLine(new File(d, "type"));
                readLine(new File(d, "temp"));
                zones.add(new String[]{type + "#" + i, new File(d, "temp").getPath()});
            } catch (Throwable e) {
                if (i == 0) put("sysfs_zone0", "not readable: " + e);
                if (!d.exists()) break;
            }
        }
        put("sysfs_zones", zones.size() + " readable");
        // HardwarePropertiesManager.getDeviceTemperatures (needs device owner / system: normally SecurityException)
        try {
            android.os.HardwarePropertiesManager hp = (android.os.HardwarePropertiesManager) ctx.getSystemService(Context.HARDWARE_PROPERTIES_SERVICE);
            hp.getDeviceTemperatures(android.os.HardwarePropertiesManager.DEVICE_TEMPERATURE_CPU, android.os.HardwarePropertiesManager.TEMPERATURE_CURRENT);
            hpm = hp; put("hardware_properties", "yes");
        } catch (Throwable e) { put("hardware_properties", "denied: " + e); }
        android.util.Log.i("arlat", "hs temp sources: " + tempSources);
        tempTh = new HandlerThread("hs-temps"); tempTh.start(); tempH = new Handler(tempTh.getLooper());
        if (android.os.Build.VERSION.SDK_INT >= 29) {
            try {
                android.os.PowerManager.OnThermalStatusChangedListener l = s -> tempRow("status");
                pm.addThermalStatusListener(tempH::post, l);
                thermalListener = l;
            } catch (Throwable e) { android.util.Log.w("arlat", "thermal listener", e); }
        }
        tempH.post(tempTick);
    }

    private void put(String k, String v) { try { tempSources.put(k, v); } catch (Exception ignored) { } }

    private static String readLine(File f) throws java.io.IOException {
        try (java.io.BufferedReader r = new java.io.BufferedReader(new java.io.FileReader(f))) { return r.readLine().trim(); }
    }

    private java.util.LinkedHashMap<String, Float> halRead() {
        java.util.LinkedHashMap<String, Float> out = new java.util.LinkedHashMap<>();
        if (thermalGet == null) return out;
        try {
            Object arr = thermalGet.invoke(thermalSvc);
            for (int i = 0; i < java.lang.reflect.Array.getLength(arr); i++) {
                Object t = java.lang.reflect.Array.get(arr, i);
                out.put((String) t.getClass().getMethod("getName").invoke(t), (Float) t.getClass().getMethod("getValue").invoke(t));
            }
        } catch (Throwable e) { thermalGet = null; put("thermal_hal_error", e.toString()); }
        return out;
    }

    private final Runnable tempTick = new Runnable() {
        @Override public void run() {
            if (!running) return;
            tempRow(null);
            tempH.postDelayed(this, TEMP_TICK_MS);
        }
    };

    /** reason null = poll: write only if 10 s since the last row ("periodic") or a value changed ("change"). */
    private void tempRow(String reason) {
        if (!running || tempW == null) return;
        try {
            long now = SystemClock.elapsedRealtimeNanos();
            android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
            int bat = bi == null ? 0 : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
            android.os.PowerManager pm = (android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE);
            int st = pm.getCurrentThermalStatus();
            java.util.LinkedHashMap<String, Float> hal = halRead();
            StringBuilder other = new StringBuilder();
            for (java.util.Map.Entry<String, Float> e : hal.entrySet())
                if (!Arrays.asList(HAL_COLS).contains(e.getKey())) other.append(e.getKey().replaceAll("[,;=\\s]", "_")).append('=').append(e.getValue()).append(';');
            for (String[] z : zones) {
                try { other.append(z[0].replaceAll("[,;=\\s]", "_")).append('=').append(Long.parseLong(readLine(new File(z[1]))) / 1000.0).append(';'); }
                catch (Throwable ignored) { }
            }
            if (hpm != null) {
                try {
                    float[] cpu = ((android.os.HardwarePropertiesManager) hpm).getDeviceTemperatures(
                            android.os.HardwarePropertiesManager.DEVICE_TEMPERATURE_CPU, android.os.HardwarePropertiesManager.TEMPERATURE_CURRENT);
                    for (int i = 0; i < cpu.length; i++) other.append("hpm_cpu").append(i).append('=').append(cpu[i]).append(';');
                } catch (Throwable ignored) { }
            }
            if (bi != null && bi.getExtras() != null)
                for (String k : bi.getExtras().keySet())
                    if (k.toLowerCase(Locale.US).contains("temp") && !k.equals(android.os.BatteryManager.EXTRA_TEMPERATURE))
                        other.append("bat_").append(k.replaceAll("[,;=\\s]", "_")).append('=').append(String.valueOf(bi.getExtras().get(k)).replaceAll("[,;\\s]", "_")).append(';');
            StringBuilder halCols = new StringBuilder();
            for (String c : HAL_COLS) { Float v = hal.get(c); halCols.append(v == null ? "" : String.valueOf(v)).append(','); }
            String key = bat + "|" + st + "|" + halCols + "|" + other;
            if (reason == null) {
                if (now - lastTempRowNs >= TEMP_ROW_NS - TEMP_TICK_MS * 500_000L) reason = "periodic";   // half-tick slack
                else if (!key.equals(lastTempKey)) reason = "change";
                else return;
            }
            float hr = Float.NaN;
            if (android.os.Build.VERSION.SDK_INT >= 30) { try { hr = pm.getThermalHeadroom(10); } catch (Throwable ignored) { } }
            String row = now + "," + System.currentTimeMillis() + "," + String.format(Locale.US, "%.3f,%.3f", now / 1e9, System.currentTimeMillis() / 1e3)
                    + "," + bat + "," + st + "," + (Float.isNaN(hr) ? "" : String.format(Locale.US, "%.4f", hr)) + ",1," + halCols + reason + "," + other + "\n";
            synchronized (tempLock) {
                if (tempW == null) return;
                tempW.write(row);
                tempW.flush();                  // few rows; keep them even if the app is killed
            }
            lastTempKey = key;
            if ("periodic".equals(reason)) lastTempRowNs = now;
            nTemps++;
        } catch (Throwable e) { android.util.Log.w("arlat", "temp row", e); }
    }

    private void stopTemps() {
        if (thermalListener != null && android.os.Build.VERSION.SDK_INT >= 29) {
            try { ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE))
                    .removeThermalStatusListener((android.os.PowerManager.OnThermalStatusChangedListener) thermalListener); }
            catch (Throwable ignored) { }
        }
        if (tempH != null) tempH.removeCallbacks(tempTick);
    }

    /** Freeze the lens where autofocus put it (AF off + that focus distance) for the rest of the recording. */
    private void lockFocus(CaptureRequest.Builder rb, CameraCaptureSession.CaptureCallback cb) {
        if (!running) return;
        try {
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, lastFocus);
            sess.setRepeatingBurst(sess.createHighSpeedRequestList(rb.build()), cb, h);
            focusLocked = lastFocus; focusLockNs = SystemClock.elapsedRealtimeNanos();
            android.util.Log.i("arlat", "HS focus locked at " + lastFocus + " dpt");
        } catch (Exception e) { android.util.Log.e("arlat", "focus lock", e); }
    }

    /** Overheating: stop the recording (the files up to now are closed normally and are usable data). */
    private final Runnable heatCheck = new Runnable() {
        @Override public void run() {
            if (!running) return;
            int temp = batteryTenthC();
            int th = ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus();
            if (temp >= maxBatteryTenthC || th >= android.os.PowerManager.THERMAL_STATUS_SEVERE) {
                stopReason = String.format(Locale.US, "overheat (battery %.1f C, thermal status %d)", temp / 10.0, th);
                android.util.Log.w("arlat", "HS stop: " + stopReason);
                stop();
                return;
            }
            h.postDelayed(this, 2000);
        }
    };

    int batteryTenthC() {
        android.content.Intent b = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return b == null ? 0 : b.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
    }

    private long[] nearestEntry(long ts) {
        long best = Long.MAX_VALUE; Long key = null;
        for (Long k : meta.keySet()) {
            long d = Math.abs(k - ts);
            if (d < best) { best = d; key = k; }
        }
        if (key == null || best > 2_000_000L) return null;     // within 2 ms (frames are 8.3 ms apart at 120 fps)
        return meta.remove(key);
    }

    @Override public void onSensorChanged(SensorEvent e) {
        if (!running) return;
        int t = e.sensor.getType();                                   // Android ids: 1 acc, 4 gyro, 16 gyro raw, 35 acc raw
        String row = (t == Sensor.TYPE_GYROSCOPE_UNCALIBRATED || t == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
                ? t + "," + e.timestamp + "," + e.values[0] + "," + e.values[1] + "," + e.values[2]
                        + "," + e.values[3] + "," + e.values[4] + "," + e.values[5] + "\n"
                : t + "," + e.timestamp + "," + e.values[0] + "," + e.values[1] + "," + e.values[2] + "\n";
        synchronized (imuLock) {              // finish() closes imuW from the hs thread
            if (imuW == null) return;
            try { imuW.write(row); nImu++; } catch (Exception ignored) { }
        }
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    void stop() {
        if (!running) return;
        running = false;
        try { ((SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE)).unregisterListener(this); } catch (Exception ignored) { }
        stopTemps();
        try { if (sess != null) sess.stopRepeating(); } catch (Exception ignored) { }
        try { enc.signalEndOfInputStream(); } catch (Exception e) { finish(); }
    }

    private boolean finished;
    private synchronized void finish() {
        if (finished) return;
        finished = true;
        try { if (dev != null) dev.close(); } catch (Exception ignored) { }
        try { enc.stop(); enc.release(); } catch (Exception ignored) { }
        try { if (track >= 0) mux.stop(); mux.release(); } catch (Exception ignored) { }
        synchronized (imuLock) { try { imuW.close(); } catch (Exception ignored) { } imuW = null; }
        synchronized (tempLock) { try { if (tempW != null) tempW.close(); } catch (Exception ignored) { } tempW = null; }
        try { frW.close(); resW.close(); } catch (Exception ignored) { }
        try {
            JSONObject m = new JSONObject();
            m.put("format", "scan3d-phone-rec/1").put("mode", "hs").put("folder", dir.getName()).put("logical", "0")
                    .put("physical", new JSONArray(Arrays.asList("5"))).put("width", size.getWidth()).put("height", size.getHeight())
                    .put("fps", wantFps).put("save_fps", wantFps).put("frames", nFrames).put("imu_samples", nImu)
                    .put("timestamp_ref", "mid_exposure").put("start_elapsed_ns", t0Elapsed).put("end_elapsed_ns", SystemClock.elapsedRealtimeNanos())
                    .put("start_wall_ms", t0Wall).put("complete", true).put("app", "arlat hs")
                    .put("clock_offset_ns", clockOffset).put("frames_unmatched", nUnmatched).put("ev_comp", evComp)
                    .put("bitrate_bps", bitrate).put("stop_reason", stopReason)
                    .put("battery_c_end", batteryTenthC() / 10.0).put("focus_dpt_request", focusDpt)
                    .put("focus_locked_dpt", focusLocked).put("focus_lock_elapsed_ns", focusLockNs)
                    .put("app_version", VERSION).put("temp_rows", nTemps).put("temp_sources", tempSources)
                    .put("capture", capInfo).put("imu_thread", "dedicated");
            try (FileWriter w = new FileWriter(new File(dir, "meta.json"))) { w.write(m.toString(1)); }
        } catch (Exception ignored) { }
        status.show((stopReason.startsWith("overheat") ? "과열로 녹화 멈춤 — 지금까지 저장됨\n" : "") + "HS done: " + nFrames + " frames, imu " + nImu + " -> " + dir.getName());
        android.util.Log.i("arlat", "HS DONE " + dir);
        if (th != null) th.quitSafely();
        if (imuTh != null) imuTh.quitSafely();
        if (tempTh != null) tempTh.quitSafely();
    }
}
