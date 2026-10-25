package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
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

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * mode calcap (0.6.23, 2026-10-25, user: "폰 과열 안 되게 10fps, 일정 거리·각도면 자동 캡처"): lens-calibration stills from the
 * SAME camera stream as the live VIO (OvRunner main / HsRecorder std): camera "0", regular session, TEMPLATE_RECORD,
 * YUV_420_888 1920x1080, OIS off, AOSP + Samsung distortion correction off, noise reduction / edge off, AF off at focus_dpt
 * (0.5), AE off with exposure exp_us (2000) and ISO chosen ONCE at start by AutoIso (metered at ae_fps, default 30 like the
 * live VIO) and then locked. Only difference to the VIO: SENSOR_FRAME_DURATION = 1/fps (default 10 fps) -> low heat.
 * No VIO, no encoder. Per frame the phone only counts it; a preview is made ONLY when the PC asks (PREV) and a capture
 * copies the raw Y plane (lossless) of the next N frames to files/calcap/<session>/cap_<id>_<k>.pgm (+ frames.jsonl with
 * the capture result of each frame: sensor timestamp, exposure, ISO, focus, frame duration, crop, ...).
 * PC link: plain TCP server on port (default 8811, Tailscale / adb forward), one line per command:
 *   PREV                  -> "JPG <len> <sensor_ts>\n" + gray JPEG (Y decimated by prev_div, default 2 -> 960x540)
 *   CAP <n> <tag> [json]  -> waits for the next n frames, saves them -> "DONE {json}" (or "BUSY" / "FAIL {json}")
 *   STAT                  -> "STAT {json}" (frames, measured fps, last capture result, ISO, temps, ...)
 *   MSG <rrggbb> <text>   -> big text on the phone screen ("\n" in text = new line) -> "OK"
 *   STOP                  -> "OK", then the runner stops (camera off) and the activity finishes
 * Auto stop (camera off, meta.json written): PC never connected within connect_grace_s (90) or silent for lost_s (20);
 * no capture for idle_s (300); max_s (1200); SKIN >= max_skin_c (40) or AP >= max_ap_c (50) from the thermal HAL dump
 * (needs adb 'pm grant com.scan3d.arlat android.permission.DUMP', else only battery / thermal status are used);
 * battery >= max_temp_dc (420 = 42.0 C); thermal status >= SEVERE. onPause (app left / screen off) also stops.
 */
class CalCapRunner {
    interface Screen { void show(int bg, String text); void image(android.graphics.Bitmap b); }

    final Context ctx;
    final Consumer<String> status;
    Screen screen;
    Runnable onAutoStop;
    // parameters (MainActivity sets them from the intent)
    int fps = 10, aeFps = 30, expUs = 2000, iso = 0, port = 8811, prevDiv = 2, jpegQ = 80;
    int idleS = 300, maxS = 1200, connectGraceS = 90, lostS = 20, maxBatteryTenthC = 420, maxCapFrames = 8;
    float focusDpt = 0.5f, maxSkinC = 40f, maxApC = 50f;
    boolean autoIso = true;
    double isoScale = 1.0;
    volatile String stopReason = "";

    static final int W = 1920, H = 1080;
    private final JSONObject meta = new JSONObject();
    private File dir;
    private HandlerThread camTh, tickTh;
    private Handler camH, tickH;
    private CameraDevice dev;
    private final Object devLock = new Object();
    private CameraCaptureSession sess;
    private ImageReader reader;
    private AutoIso autoRes;
    private volatile boolean running;
    private long t0Ms;
    private int sensorOrientation = 90;
    private Thread srvT, wrT;
    private ServerSocket srv;
    private volatile Socket cli;

    // frame stats
    private volatile long frames, lastTs, firstTs;
    private final long[] dts = new long[64];
    private int dtN;
    private volatile String firstResult = "", lastResult = "";
    private final LinkedHashMap<Long, JSONObject> results = new LinkedHashMap<Long, JSONObject>() {
        @Override protected boolean removeEldestEntry(java.util.Map.Entry<Long, JSONObject> e) { return size() > 64; }
    };

    // preview (made only on request)
    private final Object prevLock = new Object();
    private volatile boolean prevWanted;
    private byte[] prevY, prevRow;
    private long prevTs;
    private boolean prevReady;

    // capture
    private final Object capLock = new Object();
    private int capId, capLeft, capDoneN, capWant, capSkipped;
    private long capAfterTs;
    private String capTag = "", capInfo = "{}";
    private final JSONArray capFiles = new JSONArray();
    private final BlockingQueue<byte[]> pool = new ArrayBlockingQueue<>(8);
    private final BlockingQueue<Object[]> writeQ = new ArrayBlockingQueue<>(8);
    private int nCaptures, nSaved;
    private volatile long lastCmdMs, lastCapMs, firstCmdMs;

    // temperatures
    private android.os.IBinder thermalBinder;
    private volatile String temps = "{}";
    private java.io.Writer tempsW;

    CalCapRunner(Context ctx, Consumer<String> status) { this.ctx = ctx; this.status = status; }

    void start() throws Exception {
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
        dir = new File(ctx.getExternalFilesDir(null), "calcap/" + stamp);
        if (!dir.mkdirs() && !dir.isDirectory()) throw new IllegalStateException("mkdir " + dir);
        meta.put("app", "arlat mode calcap").put("session", stamp).put("dir", dir.getAbsolutePath())
                .put("params", new JSONObject().put("fps", fps).put("ae_fps", aeFps).put("exp_us", expUs).put("iso", iso)
                        .put("auto_iso", autoIso).put("iso_target_scale", isoScale).put("focus_dpt", focusDpt).put("port", port)
                        .put("prev_div", prevDiv).put("idle_s", idleS).put("max_s", maxS).put("connect_grace_s", connectGraceS)
                        .put("lost_s", lostS).put("max_skin_c", maxSkinC).put("max_ap_c", maxApC).put("max_temp_dc", maxBatteryTenthC))
                .put("stream", "camera 0, SESSION_REGULAR, TEMPLATE_RECORD, YUV_420_888 1920x1080 (= OvRunner main / std60)")
                .put("battery_c_start", batteryTenthC() / 10.0).put("charging_start", charging());
        tempsW = new FileWriter(new File(dir, "temps.csv"));
        tempsW.write("t_rel_s,skin_c,ap_c,battery_c,thermal_status,frames\n");
        try {
            Class<?> smc = Class.forName("android.os.ServiceManager");
            thermalBinder = (android.os.IBinder) smc.getMethod("getService", String.class).invoke(null, "thermalservice");
        } catch (Throwable e) { meta.put("thermal_binder", e.toString()); }
        java.util.LinkedHashMap<String, Float> t = halTemps();
        meta.put("thermal_hal_start", new JSONObject(mapJson(t)));
        Float skin0 = t.get("SKIN");
        if (skin0 != null && skin0 >= maxSkinC) throw new IllegalStateException("SKIN " + skin0 + " >= " + maxSkinC + " C: not starting");
        for (int i = 0; i < 6; i++) pool.add(new byte[W * H]);
        running = true;
        t0Ms = SystemClock.elapsedRealtime();
        lastCapMs = t0Ms;
        camTh = new HandlerThread("calcap-cam"); camTh.start(); camH = new Handler(camTh.getLooper());
        tickTh = new HandlerThread("calcap-tick"); tickTh.start(); tickH = new Handler(tickTh.getLooper());
        wrT = new Thread(this::writerLoop, "calcap-wr"); wrT.start();
        srvT = new Thread(this::serverLoop, "calcap-srv"); srvT.start();
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        final CameraCharacteristics ch = cm.getCameraCharacteristics("0");
        offer(ch);
        Integer so = ch.get(CameraCharacteristics.SENSOR_ORIENTATION);
        if (so != null) sensorOrientation = so;
        meta.put("sensor_orientation", sensorOrientation);
        reader = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4);
        reader.setOnImageAvailableListener(this::onImage, camH);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                synchronized (devLock) {          // stop() on the UI thread must see dev (verifier: close race)
                    if (!running) { d.close(); return; }
                    dev = d;
                }
                if (autoIso && iso <= 0 && expUs > 0) {
                    show(0xFF333333, "카메라 켜는 중 (밝기 측정)…");
                    AutoIso a = new AutoIso();
                    a.size = new Size(W, H); a.fps = new Range<>(aeFps, aeFps); a.focusDpt = focusDpt; a.expNs = expUs * 1000L; a.scale = isoScale;
                    autoRes = a;
                    a.run(d, ch, camH, x -> {
                        try { meta.put("auto_iso", x.json()); } catch (Exception ignored) { }
                        if (running) openSession(d, ch);
                    });
                } else {
                    try { meta.put("auto_iso", new JSONObject().put("auto_iso", false).put("iso", iso)); } catch (Exception ignored) { }
                    openSession(d, ch);
                }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); stopAsync("camera disconnected"); }
            @Override public void onError(CameraDevice d, int e) { d.close(); stopAsync("camera error " + e); }
        }, camH);
        tickH.postDelayed(tick, 1000);
        show(0xFF333333, "렌즈 보정 캡처 시작 중…");
    }

    private void offer(CameraCharacteristics ch) {
        try {
            StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            meta.put("yuv_min_frame_duration_ns", map.getOutputMinFrameDuration(ImageFormat.YUV_420_888, new Size(W, H)));
            Range<Integer>[] rr = ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
            JSONArray a = new JSONArray();
            if (rr != null) for (Range<Integer> r : rr) a.put(r.toString());
            meta.put("ae_available_target_fps_ranges", a);
            meta.put("active_array", String.valueOf(ch.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)));
        } catch (Exception e) { try { meta.put("offer_error", e.toString()); } catch (Exception ignored) { } }
    }

    /** AE target range for the request: [fps,fps] if offered, else the offered range containing fps with the lowest upper. */
    @SuppressWarnings("unchecked")
    private Range<Integer> aeRange(CameraCharacteristics ch) {
        Range<Integer>[] rr = ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        Range<Integer> best = null;
        if (rr != null) for (Range<Integer> r : rr) {
            if (r.getLower() == fps && r.getUpper() == fps) return r;
            if (r.contains(fps) && (best == null || r.getUpper() < best.getUpper())) best = r;
        }
        return best;
    }

    private static boolean has(int[] a, int v) { if (a == null) return false; for (int x : a) if (x == v) return true; return false; }

    private void openSession(final CameraDevice d, final CameraCharacteristics ch) {
        try {
            final CaptureRequest.Builder rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            rb.addTarget(reader.getSurface());
            Range<Integer> ae = aeRange(ch);
            if (ae != null) rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, ae);
            meta.put("ae_target_fps_range_set", String.valueOf(ae));
            rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
            if (has(ch.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES), CaptureRequest.NOISE_REDUCTION_MODE_OFF))
                rb.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
            if (has(ch.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CaptureRequest.EDGE_MODE_OFF))
                rb.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
            // = OvRunner.mainRequest (live VIO main camera)
            rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
            int[] dm = ch.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
            if (has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF))
                rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
            meta.put("aosp_distortion_request", has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF) ? "OFF" : "not offered");
            CaptureRequest.Key<Integer> ldc = null;
            try { ldc = new CaptureRequest.Key<>(HsRecorder.LDC_TAG, Integer.class); rb.set(ldc, 0); meta.put("samsung_ldc_request", "set 0 (int32)"); }
            catch (Throwable e) { ldc = null; meta.put("samsung_ldc_request", "not set: " + e); }
            if (focusDpt >= 0) {
                rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
            }
            long frameDur = 1_000_000_000L / fps;
            long e = expUs * 1000L;
            Range<Long> er = ch.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
            if (er != null) e = er.clamp(e);
            int s = iso > 0 ? iso : (autoRes != null ? autoRes.iso : 400);
            Range<Integer> sr = ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            if (sr != null) s = sr.clamp(s);
            iso = s;
            rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            rb.set(CaptureRequest.SENSOR_EXPOSURE_TIME, e);
            rb.set(CaptureRequest.SENSOR_SENSITIVITY, s);
            rb.set(CaptureRequest.SENSOR_FRAME_DURATION, frameDur);
            meta.put("exposure_ns_set", e).put("iso_set", s).put("frame_duration_ns_set", frameDur);
            final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                @Override public void onCaptureCompleted(CameraCaptureSession ss, CaptureRequest q, TotalCaptureResult r) { onResult(r); }
            };
            final CaptureRequest.Key<Integer> ldcF = ldc;
            OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
            d.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR, Collections.singletonList(oc),
                    ctx.getMainExecutor(), new CameraCaptureSession.StateCallback() {
                @Override public void onConfigured(CameraCaptureSession s2) {
                    sess = s2;
                    try { s2.setRepeatingRequest(rb.build(), cb, camH); }
                    catch (Exception ex) {
                        if (ldcF == null) { stopAsync("repeat: " + ex); return; }
                        try { rb.set(ldcF, null); s2.setRepeatingRequest(rb.build(), cb, camH); meta.put("samsung_ldc_request", "set 0 -> request failed, removed"); }
                        catch (Exception ex2) { stopAsync("repeat: " + ex2); return; }
                    }
                    show(0xFF333333, "준비됨 (" + fps + " fps, ISO " + iso + ")\nPC 연결 기다리는 중…");
                }
                @Override public void onConfigureFailed(CameraCaptureSession s2) { stopAsync("configure failed"); }
            }));
        } catch (Exception e) { stopAsync("session: " + e); }
    }

    private void onResult(TotalCaptureResult r) {
        try {
            Long ts = r.get(CaptureResult.SENSOR_TIMESTAMP);
            if (ts == null) return;
            JSONObject j = new JSONObject();
            j.put("sensor_ts_ns", ts).put("exposure_ns", r.get(CaptureResult.SENSOR_EXPOSURE_TIME))
                    .put("iso", r.get(CaptureResult.SENSOR_SENSITIVITY)).put("focus_dpt", r.get(CaptureResult.LENS_FOCUS_DISTANCE))
                    .put("frame_duration_ns", r.get(CaptureResult.SENSOR_FRAME_DURATION))
                    .put("rolling_shutter_skew_ns", r.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW))
                    .put("crop", HsRecorder.cropRegion(r)).put("lens_state", r.get(CaptureResult.LENS_STATE))
                    .put("af_mode", r.get(CaptureResult.CONTROL_AF_MODE)).put("ae_mode", r.get(CaptureResult.CONTROL_AE_MODE))
                    .put("ois", r.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE))
                    .put("distortion_correction", r.get(CaptureResult.DISTORTION_CORRECTION_MODE))
                    .put("noise_reduction", r.get(CaptureResult.NOISE_REDUCTION_MODE)).put("edge", r.get(CaptureResult.EDGE_MODE));
            synchronized (results) { results.put(ts, j); results.notifyAll(); }
            String s = j.toString();
            if (firstResult.isEmpty()) firstResult = s;
            lastResult = s;
        } catch (Exception ignored) { }
    }

    private void onImage(ImageReader r) {
        Image im;
        try { im = r.acquireNextImage(); } catch (Exception e) { return; }
        if (im == null) return;
        try {
            long ts = im.getTimestamp();
            if (frames == 0) firstTs = ts;
            else { dts[dtN % dts.length] = ts - lastTs; dtN++; }
            frames++; lastTs = ts;
            Image.Plane p = im.getPlanes()[0];
            ByteBuffer b = p.getBuffer();
            int rs = p.getRowStride();
            byte[] cap = null; int id = 0, k = 0; String tag = null, info = null;
            synchronized (capLock) {
                if (capLeft > 0 && ts > capAfterTs) {
                    cap = pool.poll();
                    if (cap == null) capSkipped++;
                    else { id = capId; k = capWant - capLeft; tag = capTag; info = capInfo; capLeft--; }
                }
            }
            if (cap != null) {
                if (rs == W) { b.position(0); b.get(cap, 0, W * H); }
                else for (int y = 0; y < H; y++) { b.position(y * rs); b.get(cap, y * W, W); }
                if (!writeQ.offer(new Object[]{cap, ts, id, k, tag, info})) { pool.offer(cap); synchronized (capLock) { capSkipped++; } }
            }
            if (prevWanted) {
                int d = Math.max(1, prevDiv), pw = W / d, ph = H / d;
                if (prevY == null || prevY.length != pw * ph) { prevY = new byte[pw * ph]; prevRow = new byte[W]; }
                for (int y = 0; y < ph; y++) {
                    b.position(y * d * rs); b.get(prevRow, 0, W);
                    int o = y * pw;
                    for (int x = 0; x < pw; x++) prevY[o + x] = prevRow[x * d];
                }
                synchronized (prevLock) { prevTs = ts; prevReady = true; prevWanted = false; prevLock.notifyAll(); }
            }
        } catch (Exception e) {
            android.util.Log.w("arlat", "calcap image", e);
        } finally { im.close(); }
    }

    /** Saves captured Y planes as PGM + one frames.jsonl line each (with the capture result of that frame). */
    private void writerLoop() {
        while (running || !writeQ.isEmpty()) {
            Object[] it;
            try { it = writeQ.poll(200, TimeUnit.MILLISECONDS); } catch (InterruptedException e) { break; }
            if (it == null) continue;
            byte[] y = (byte[]) it[0]; long ts = (Long) it[1]; int id = (Integer) it[2], k = (Integer) it[3];
            String tag = (String) it[4], info = (String) it[5];
            String name = String.format(Locale.US, "cap_%03d_%d.pgm", id, k);
            JSONObject res = null;
            long until = SystemClock.elapsedRealtime() + 500;
            synchronized (results) {
                while ((res = results.get(ts)) == null && SystemClock.elapsedRealtime() < until) {
                    try { results.wait(50); } catch (InterruptedException ignored) { }
                }
            }
            boolean ok = false;
            try (FileOutputStream f = new FileOutputStream(new File(dir, name))) {
                f.write(("P5\n" + W + " " + H + "\n255\n").getBytes(StandardCharsets.US_ASCII));
                f.write(y);
                ok = true;
            } catch (Exception e) { android.util.Log.w("arlat", "calcap write", e); }
            pool.offer(y);
            try {
                JSONObject j = new JSONObject();
                j.put("file", name).put("cap", id).put("k", k).put("tag", tag).put("sensor_ts_ns", ts).put("written", ok)
                        .put("width", W).put("height", H).put("format", "PGM P5 raw Y plane (YUV_420_888 plane 0, lossless)")
                        .put("result", res == null ? JSONObject.NULL : res);
                try { j.put("pc", new JSONObject(info)); } catch (Exception e) { j.put("pc_raw", info); }
                synchronized (this) {
                    try (FileWriter w = new FileWriter(new File(dir, "frames.jsonl"), true)) { w.write(j.toString() + "\n"); }
                }
            } catch (Exception e) { android.util.Log.w("arlat", "calcap jsonl", e); }
            synchronized (capLock) {
                if (id == capId) { capDoneN++; if (ok) capFiles.put(name); }
                if (ok) nSaved++;
                capLock.notifyAll();
            }
        }
    }

    private void serverLoop() {
        try {
            srv = new ServerSocket();
            srv.setReuseAddress(true);          // a second onResume (lock screen) must be able to bind right after the first stops
            srv.bind(new java.net.InetSocketAddress(port));
            srv.setSoTimeout(1000);
        } catch (Exception e) { stopAsync("server socket " + port + ": " + e); return; }
        while (running) {
            Socket s;
            try { s = srv.accept(); } catch (SocketTimeoutException e) { continue; } catch (Exception e) { break; }
            cli = s;
            try {
                s.setSoTimeout(1000); s.setTcpNoDelay(true);
                BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
                OutputStream out = s.getOutputStream();
                while (running) {
                    String line;
                    try { line = in.readLine(); } catch (SocketTimeoutException e) { continue; }
                    if (line == null) break;
                    lastCmdMs = SystemClock.elapsedRealtime();
                    if (firstCmdMs == 0) firstCmdMs = lastCmdMs;
                    handle(line.trim(), out);
                }
            } catch (Exception e) { android.util.Log.w("arlat", "calcap client", e); }
            try { s.close(); } catch (Exception ignored) { }
            cli = null;
        }
    }

    private void reply(OutputStream out, String line) throws Exception {
        out.write((line.replace('\n', ' ') + "\n").getBytes(StandardCharsets.UTF_8)); out.flush();
    }

    private void handle(String line, OutputStream out) throws Exception {
        String[] a = line.split(" ", 4);
        String c = a[0].toUpperCase(Locale.US);
        if (c.equals("PREV")) {
            byte[] y; long ts; int pw, ph;
            synchronized (prevLock) {
                prevReady = false; prevWanted = true;
                long until = SystemClock.elapsedRealtime() + Math.max(600, 3000 / Math.max(1, fps));
                while (!prevReady && running && SystemClock.elapsedRealtime() < until) prevLock.wait(50);
                if (!prevReady) { prevWanted = false; reply(out, "NOJPG"); return; }
                int d = Math.max(1, prevDiv); pw = W / d; ph = H / d;
                y = prevY.clone(); ts = prevTs;
            }
            byte[] nv = new byte[pw * ph * 3 / 2];
            System.arraycopy(y, 0, nv, 0, pw * ph);
            java.util.Arrays.fill(nv, pw * ph, nv.length, (byte) 128);
            ByteArrayOutputStream bo = new ByteArrayOutputStream(120_000);
            new YuvImage(nv, ImageFormat.NV21, pw, ph, null).compressToJpeg(new Rect(0, 0, pw, ph), jpegQ, bo);
            byte[] j = bo.toByteArray();
            out.write(("JPG " + j.length + " " + ts + "\n").getBytes(StandardCharsets.US_ASCII));
            out.write(j); out.flush();
        } else if (c.equals("CAP")) {
            int n = Math.max(1, Math.min(maxCapFrames, a.length > 1 ? Integer.parseInt(a[1]) : 4));
            String tag = a.length > 2 ? a[2] : "x", info = a.length > 3 ? a[3] : "{}";
            int myId; JSONArray files;
            synchronized (capLock) {
                if (capLeft > 0 || (capWant > 0 && capDoneN < capWant)) { reply(out, "BUSY"); return; }
                myId = ++capId; capWant = n; capLeft = n; capDoneN = 0; capSkipped = 0; capTag = tag; capInfo = info;
                capAfterTs = lastTs;
                while (capFiles.length() > 0) capFiles.remove(0);
                long until = SystemClock.elapsedRealtime() + 3000 + n * 3000L / Math.max(1, fps);
                while (capDoneN < n && running && SystemClock.elapsedRealtime() < until) capLock.wait(50);
                files = new JSONArray(capFiles.toString());
                boolean ok = capDoneN >= n && files.length() == n;
                if (!ok) { capLeft = 0; capWant = 0; }
                nCaptures++;
                lastCapMs = SystemClock.elapsedRealtime();
                JSONObject r = new JSONObject().put("cap", myId).put("tag", tag).put("files", files).put("n", n)
                        .put("skipped", capSkipped).put("dir", dir.getAbsolutePath());
                reply(out, (ok ? "DONE " : "FAIL ") + r);
            }
        } else if (c.equals("STAT")) {
            reply(out, "STAT " + stat());
        } else if (c.equals("MSG")) {
            int bg = 0xFF333333;
            try { bg = 0xFF000000 | Integer.parseInt(a[1], 16); } catch (Exception ignored) { }
            String t = (a.length > 2 ? a[2] : "") + (a.length > 3 ? " " + a[3] : "");
            show(bg, t.replace("\\n", "\n"));
            reply(out, "OK");
        } else if (c.equals("IMG")) {
            try {
                byte[] j = android.util.Base64.decode(a.length > 1 ? a[1] : "", android.util.Base64.DEFAULT);
                android.graphics.Bitmap bm = android.graphics.BitmapFactory.decodeByteArray(j, 0, j.length);
                if (bm != null && screen != null) {
                    android.graphics.Matrix m = new android.graphics.Matrix();
                    m.postRotate(sensorOrientation);
                    screen.image(android.graphics.Bitmap.createBitmap(bm, 0, 0, bm.getWidth(), bm.getHeight(), m, true));
                }
                reply(out, "OK");
            } catch (Exception e) { reply(out, "ERR img " + e); }
        } else if (c.equals("STOP")) {
            reply(out, "OK");
            stopAsync("pc stop");
        } else reply(out, "ERR unknown " + c);
    }

    JSONObject stat() {
        JSONObject j = new JSONObject();
        try {
            int n = Math.min(dtN, dts.length);
            double mean = 0; long mx = 0, mn = Long.MAX_VALUE;
            for (int i = 0; i < n; i++) { mean += dts[i]; mx = Math.max(mx, dts[i]); mn = Math.min(mn, dts[i]); }
            mean = n > 0 ? mean / n : 0;
            j.put("frames", frames).put("fps_measured", mean > 0 ? 1e9 / mean : 0).put("dt_ms_mean", mean / 1e6)
                    .put("dt_ms_min", n > 0 ? mn / 1e6 : 0).put("dt_ms_max", mx / 1e6).put("dt_n", n)
                    .put("span_s", frames > 1 ? (lastTs - firstTs) / 1e9 : 0).put("fps_set", fps).put("iso", iso)
                    .put("exp_us", expUs).put("focus_dpt", focusDpt).put("captures", nCaptures).put("saved_frames", nSaved)
                    .put("uptime_s", (SystemClock.elapsedRealtime() - t0Ms) / 1000.0).put("idle_s_left", idleS - (SystemClock.elapsedRealtime() - lastCapMs) / 1000)
                    .put("temps", new JSONObject(temps)).put("dir", dir.getAbsolutePath()).put("stop_reason", stopReason)
                    .put("last_result", lastResult.isEmpty() ? JSONObject.NULL : new JSONObject(lastResult));
        } catch (Exception ignored) { }
        return j;
    }

    private final Runnable tick = new Runnable() {
        private int k;
        @Override public void run() {
            if (!running) return;
            long now = SystemClock.elapsedRealtime();
            String why = null;
            if (firstCmdMs == 0 && now - t0Ms > connectGraceS * 1000L) why = "PC not connected in " + connectGraceS + " s";
            else if (firstCmdMs != 0 && now - lastCmdMs > lostS * 1000L) why = "PC silent " + lostS + " s";
            else if (now - lastCapMs > idleS * 1000L) why = "idle (no capture " + idleS + " s)";
            else if (now - t0Ms > maxS * 1000L) why = "max " + maxS + " s";
            if (k++ % 5 == 0 || why != null) {          // temperatures every 5 s
                java.util.LinkedHashMap<String, Float> t = halTemps();
                int b = batteryTenthC(), ts = thermalStatus();
                Float skin = t.get("SKIN"), ap = t.get("AP");
                temps = String.format(Locale.US, "{\"skin_c\":%s,\"ap_c\":%s,\"battery_c\":%.1f,\"thermal_status\":%d,\"charging\":%b}",
                        skin == null ? "null" : skin.toString(), ap == null ? "null" : ap.toString(), b / 10.0, ts, charging());
                try {
                    tempsW.write(String.format(Locale.US, "%.1f,%s,%s,%.1f,%d,%d\n", (now - t0Ms) / 1000.0, skin, ap, b / 10.0, ts, frames));
                    tempsW.flush();
                } catch (Exception ignored) { }
                if (why == null) {
                    if (skin != null && skin >= maxSkinC) why = "SKIN " + skin + " C >= " + maxSkinC;
                    else if (ap != null && ap >= maxApC) why = "AP " + ap + " C >= " + maxApC;
                    else if (b >= maxBatteryTenthC) why = "battery " + b / 10.0 + " C";
                    else if (ts >= android.os.PowerManager.THERMAL_STATUS_SEVERE) why = "thermal status " + ts;
                }
            }
            if (why != null) { stopAsync(why); return; }
            tickH.postDelayed(this, 1000);
        }
    };

    private void stopAsync(String why) {
        if (stopReason.isEmpty()) stopReason = why;
        status.accept("calcap stop: " + why);
        if (onAutoStop != null) onAutoStop.run(); else stop();
    }

    private void show(int bg, String text) { if (screen != null) screen.show(bg, text); }

    void stop() {
        if (!running) { try { if (tempsW != null) tempsW.close(); } catch (Exception ignored) { } return; }
        running = false;
        if (stopReason.isEmpty()) stopReason = "activity paused/finished";
        try { if (sess != null) sess.close(); } catch (Exception ignored) { }
        synchronized (devLock) { try { if (dev != null) dev.close(); } catch (Exception ignored) { } dev = null; }
        sess = null;
        try { if (srv != null) srv.close(); } catch (Exception ignored) { }
        try { Socket c = cli; if (c != null) c.close(); } catch (Exception ignored) { }
        try { if (srvT != null) srvT.join(1500); } catch (InterruptedException ignored) { }
        try { if (wrT != null) wrT.join(3000); } catch (InterruptedException ignored) { }
        if (camTh != null) camTh.quitSafely();
        if (tickTh != null) tickTh.quitSafely();
        try { if (reader != null) reader.close(); } catch (Exception ignored) { }
        reader = null;
        try {
            meta.put("stop_reason", stopReason).put("stat_end", stat()).put("first_result", firstResult.isEmpty() ? JSONObject.NULL : new JSONObject(firstResult))
                    .put("battery_c_end", batteryTenthC() / 10.0).put("camera_closed", true);
            try (FileWriter w = new FileWriter(new File(dir, "meta.json"))) { w.write(meta.toString(1)); }
        } catch (Exception ignored) { }
        try { if (tempsW != null) tempsW.close(); } catch (Exception ignored) { }
        android.util.Log.i("arlat", "calcap stopped: " + stopReason);
    }

    // ---- temperatures (same source as HsRecorder: thermalservice dump, "Current temperatures from HAL")
    private static final java.util.regex.Pattern HAL_T = java.util.regex.Pattern.compile("mValue=([-0-9.eE]+).*?mName=([^,}]+)");

    private java.util.LinkedHashMap<String, Float> halTemps() {
        java.util.LinkedHashMap<String, Float> out = new java.util.LinkedHashMap<>();
        if (thermalBinder == null) return out;
        android.os.ParcelFileDescriptor[] p = null;
        try {
            p = android.os.ParcelFileDescriptor.createPipe();
            thermalBinder.dumpAsync(p[1].getFileDescriptor(), new String[0]);
            p[1].close(); p[1] = null;
            boolean in = false;
            try (BufferedReader r = new BufferedReader(new InputStreamReader(new android.os.ParcelFileDescriptor.AutoCloseInputStream(p[0])))) {
                p[0] = null;
                for (String line; (line = r.readLine()) != null; ) {
                    if (line.contains("Current temperatures from HAL")) { in = true; continue; }
                    if (!in) continue;
                    if (!line.isEmpty() && !Character.isWhitespace(line.charAt(0))) break;
                    java.util.regex.Matcher m = HAL_T.matcher(line);
                    if (m.find() && !out.containsKey(m.group(2).trim())) out.put(m.group(2).trim(), Float.parseFloat(m.group(1)));
                }
            }
        } catch (Throwable e) { android.util.Log.w("arlat", "calcap thermal dump", e); }
        finally {
            if (p != null) for (android.os.ParcelFileDescriptor f : p) try { if (f != null) f.close(); } catch (Exception ignored) { }
        }
        return out;
    }

    private static String mapJson(java.util.Map<String, Float> m) {
        StringBuilder sb = new StringBuilder("{");
        for (java.util.Map.Entry<String, Float> e : m.entrySet()) {
            if (sb.length() > 1) sb.append(',');
            sb.append('"').append(e.getKey()).append("\":").append(e.getValue());
        }
        return sb.append('}').toString();
    }

    int batteryTenthC() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi == null ? 0 : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
    }

    boolean charging() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi != null && bi.getIntExtra(android.os.BatteryManager.EXTRA_PLUGGED, 0) != 0;
    }

    int thermalStatus() {
        try { return ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus(); }
        catch (Throwable e) { return -1; }
    }
}
