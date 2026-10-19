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
 */
class HsRecorder implements SensorEventListener {
    interface Status { void show(String s); }

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

    HsRecorder(Context ctx, Status status) { this.ctx = ctx; this.status = status; }

    /** Pick the high-speed size/fps: the largest size offering wantFps (fixed range [f,f]). */
    String choose() throws Exception {
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        CameraCharacteristics ch = cm.getCameraCharacteristics("0");
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
        resW.write("sensor_ts_ns,exposure_ns,rolling_shutter_skew_ns,frame_number,arrive_elapsed_ns,focus_dpt,af_mode,lens_moving\n");
        frW.write("frame_idx,t_mid_ns,physical_ts_ns,logical_ts_ns,exposure_ns,rolling_shutter_skew_ns,pts_us\n");
        t0Elapsed = SystemClock.elapsedRealtimeNanos(); t0Wall = System.currentTimeMillis();
        clockOffset = SystemClock.elapsedRealtimeNanos() - System.nanoTime();
        SensorManager sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST, h);
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST, h);

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
                                if (evComp != 0) rb.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, evComp);   // darker = shorter exposure, if AE obeys
                                if (focusDpt >= 0) {        // fixed focus: autofocus moves the lens and changes the intrinsics mid-recording
                                    rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                                    rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
                                }
                                List<CaptureRequest> burst = sess.createHighSpeedRequestList(rb.build());
                                final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onCaptureCompleted(CameraCaptureSession ss, CaptureRequest q, TotalCaptureResult r) {
                                        Long ts = r.get(CaptureResult.SENSOR_TIMESTAMP), e = r.get(CaptureResult.SENSOR_EXPOSURE_TIME),
                                                sk = r.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                                        Float fd = r.get(CaptureResult.LENS_FOCUS_DISTANCE);
                                        Integer afm = r.get(CaptureResult.CONTROL_AF_MODE), lst = r.get(CaptureResult.LENS_STATE);
                                        try { resW.write(ts + "," + e + "," + sk + "," + r.getFrameNumber() + "," + SystemClock.elapsedRealtimeNanos()
                                                + "," + fd + "," + afm + "," + lst + "\n"); }
                                        catch (Exception ignored) { }
                                        if (ts != null) meta.put(ts, new long[]{e == null ? 0 : e, sk == null ? 0 : sk, ts});
                                        if (++nRes <= 3) android.util.Log.i("arlat", "hs result ts " + ts + " exp " + e + " skew " + sk);
                                        // drop only stale entries (> 1 s old); clearing the whole map lost pending matches (525 frames per 134 s)
                                        if (ts != null && meta.size() > 2000) meta.keySet().removeIf(k -> k < ts - 1_000_000_000L);
                                        if (fd != null) lastFocus = fd;
                                    }
                                };
                                sess.setRepeatingBurst(burst, cb, h);
                                if (focusDpt == -2) h.postDelayed(() -> lockFocus(rb, cb), 1500);   // autofocus 1.5 s, then hold it
                                h.postDelayed(heatCheck, 2000);
                                status.show("HS " + size + "@" + wantFps + " recording -> " + dir.getName());
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
        int t = e.sensor.getType() == Sensor.TYPE_ACCELEROMETER ? 1 : 4;
        try {
            imuW.write(t + "," + e.timestamp + "," + e.values[0] + "," + e.values[1] + "," + e.values[2] + "\n");
            nImu++;
        } catch (Exception ignored) { }
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    void stop() {
        if (!running) return;
        running = false;
        try { ((SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE)).unregisterListener(this); } catch (Exception ignored) { }
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
        try { imuW.close(); frW.close(); resW.close(); } catch (Exception ignored) { }
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
                    .put("focus_locked_dpt", focusLocked).put("focus_lock_elapsed_ns", focusLockNs);
            try (FileWriter w = new FileWriter(new File(dir, "meta.json"))) { w.write(m.toString(1)); }
        } catch (Exception ignored) { }
        status.show((stopReason.startsWith("overheat") ? "과열로 녹화 멈춤 — 지금까지 저장됨\n" : "") + "HS done: " + nFrames + " frames, imu " + nImu + " -> " + dir.getName());
        android.util.Log.i("arlat", "HS DONE " + dir);
        if (th != null) th.quitSafely();
    }
}
