package com.scan3d.ispprobe;

import android.app.Activity;
import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import android.view.WindowManager;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Camera HAL/ISP delivery latency probe for the S24 main camera (logical "0", same as arlat ov_cam main).
 * For each case: open camera -> one YUV (or RAW) ImageReader -> repeating request for `secs` s -> close camera.
 * Per frame (files/isp/run_<wall>/<case>.csv): frame number, SENSOR_TIMESTAMP, elapsedRealtimeNanos at
 * onCaptureStarted / onReadoutStarted (API 34) / onCaptureCompleted / onImageAvailable, REQUEST_PIPELINE_DEPTH,
 * SENSOR_ROLLING_SHUTTER_SKEW, SENSOR_FRAME_DURATION, SENSOR_EXPOSURE_TIME, SENSOR_SENSITIVITY.
 * Also caps.json (characteristics: timestamp source, pipeline max depth, RAW streams, ISP modes) and plan.json.
 * The camera is closed in a finally for every case, on onPause/onDestroy, and after the last case; done.txt last.
 * Extras: cases (comma list or "all"), secs 5, gap_ms 1500, iso 1600, exp_us 2000, focus 0.5, max_images 4,
 *         dry (true: characteristics + plan only, the camera is never opened).
 * Baseline "base" = arlat 0.6.16 ov main request: TEMPLATE_RECORD, 1920x1080 YUV 30 fps, NR OFF, EDGE OFF, OIS OFF,
 * video stab OFF, AOSP distortion OFF if offered, Samsung LDC tag 0, AF off focus 0.5 dpt, AE off 2 ms, fixed ISO.
 */
public class ProbeActivity extends Activity {
    static final String TAG = "ispprobe";
    static final String CAM = "0";
    static final String LDC_TAG = "samsung.android.control.lensDistortionCorrectionMode";
    static final int ISP_ARLAT = 0, ISP_DEFAULT = 1, ISP_MIN = 2;
    static final String[] ISP_NAME = {"arlat(NR/EDGE off)", "template default", "min(NR/EDGE/hotpix off, tonemap/cc fast)"};

    private TextView tv;
    private volatile boolean abort;
    private volatile CameraDevice curDev;
    private File outDir;
    private int secs = 5, gapMs = 1500, iso = 1600, expUs = 2000, maxImages = 4;
    private float focus = 0.5f;
    private boolean dry;
    private String casesArg = "all";

    static class Case {
        String name; int template; int w, h, fmt, fps, isp; String skip, note = "";
        Case(String n, int t, int w, int h, int f, int fps, int isp) { name = n; template = t; this.w = w; this.h = h; fmt = f; this.fps = fps; this.isp = isp; }
    }

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        tv = new TextView(this);
        tv.setTextSize(16);
        tv.setPadding(24, 48, 24, 24);
        setContentView(tv);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        android.content.Intent it = getIntent();
        secs = it.getIntExtra("secs", secs);
        gapMs = it.getIntExtra("gap_ms", gapMs);
        iso = it.getIntExtra("iso", iso);
        expUs = it.getIntExtra("exp_us", expUs);
        maxImages = it.getIntExtra("max_images", maxImages);
        focus = it.getFloatExtra("focus", focus);
        dry = it.getBooleanExtra("dry", false);
        String c = it.getStringExtra("cases");
        if (c != null && !c.isEmpty()) casesArg = c;
        String wall = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        outDir = new File(getExternalFilesDir(null), "isp/run_" + wall + (dry ? "_dry" : ""));
        outDir.mkdirs();
        show("ispprobe " + (dry ? "DRY (no camera)" : "run") + "\n" + outDir);
        new Thread(this::runAll, "probe").start();
    }

    @Override protected void onPause() { super.onPause(); stopNow("onPause"); }
    @Override protected void onDestroy() { stopNow("onDestroy"); super.onDestroy(); }

    private void stopNow(String why) {
        if (!abort) Log.i(TAG, "abort: " + why);
        abort = true;
        CameraDevice d = curDev;
        if (d != null) try { d.close(); } catch (Throwable ignored) { }
    }

    private void show(final String s) {
        Log.i(TAG, s);
        runOnUiThread(() -> tv.setText(s));
    }

    // ------------------------------------------------------------------ run all cases
    private void runAll() {
        String status = "ok";
        PrintWriter sum = null;
        try {
            sum = new PrintWriter(new FileWriter(new File(outDir, "summary.csv")));
            sum.println("case,status,n_img,n_result,fps_got,lat_img_p50_ms,lat_img_max_ms,depth_med,skew_ms,note");
            CameraManager cm = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
            CameraCharacteristics ch = cm.getCameraCharacteristics(CAM);
            writeCaps(cm, ch);
            List<Case> cases = buildCases(ch);
            writePlan(cases);
            if (!dry) {
                int i = 0;
                for (Case c : cases) {
                    if (abort) { status = "aborted"; break; }
                    i++;
                    if (c.skip != null) { sum.println(c.name + ",skipped,0,0,,,,,," + csvq(c.skip)); sum.flush(); continue; }
                    show("case " + i + "/" + cases.size() + " " + c.name + "\n" + c.w + "x" + c.h + " " + c.fps + " fps");
                    String line = runCase(cm, ch, c);
                    sum.println(line); sum.flush();
                    if (i < cases.size()) sleepAbortable(gapMs);
                }
                if (abort) status = "aborted";
            } else status = "dry";
        } catch (Throwable t) {
            status = "error " + t;
            Log.e(TAG, "runAll", t);
        } finally {
            if (sum != null) sum.close();
            CameraDevice d = curDev;
            if (d != null) try { d.close(); } catch (Throwable ignored) { }
            curDev = null;
            try (PrintWriter w = new PrintWriter(new FileWriter(new File(outDir, "done.txt")))) {
                w.println(status);
                w.println("elapsed_ns " + SystemClock.elapsedRealtimeNanos());
            } catch (Exception ignored) { }
            show("DONE " + status + "\n" + outDir);
            Log.i(TAG, "DONE " + status + " " + outDir);
            runOnUiThread(this::finish);
        }
    }

    private void sleepAbortable(long ms) {
        long end = SystemClock.elapsedRealtime() + ms;
        while (!abort && SystemClock.elapsedRealtime() < end) SystemClock.sleep(50);
    }

    // ------------------------------------------------------------------ cases
    private List<Case> buildCases(CameraCharacteristics ch) {
        StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        final int R = CameraDevice.TEMPLATE_RECORD, P = CameraDevice.TEMPLATE_PREVIEW, Z = CameraDevice.TEMPLATE_ZERO_SHUTTER_LAG;
        final int Y = ImageFormat.YUV_420_888;
        List<Case> all = new ArrayList<>();
        all.add(new Case("base", R, 1920, 1080, Y, 30, ISP_ARLAT));          // = arlat ov main now
        all.add(new Case("isp_default", R, 1920, 1080, Y, 30, ISP_DEFAULT)); // NR/EDGE left at template default
        all.add(new Case("isp_min", R, 1920, 1080, Y, 30, ISP_MIN));
        all.add(new Case("tmpl_preview", P, 1920, 1080, Y, 30, ISP_ARLAT));
        all.add(new Case("tmpl_zsl", Z, 1920, 1080, Y, 30, ISP_ARLAT));
        all.add(new Case("fps60", R, 1920, 1080, Y, 60, ISP_ARLAT));
        all.add(new Case("s1280x720", R, 1280, 720, Y, 30, ISP_ARLAT));
        all.add(new Case("s640x360", R, 640, 360, Y, 30, ISP_ARLAT));
        Case combo = new Case("combo_min_60_720", R, 1280, 720, Y, 60, ISP_MIN);
        combo.note = "combined: isp_min + 60 fps + 1280x720 (mixed variables)";
        all.add(combo);
        all.add(rawCase(map));
        all.add(new Case("base_end", R, 1920, 1080, Y, 30, ISP_ARLAT));      // repeat of base (drift / warm-up check)
        for (Case c : all) {
            if (c.skip != null) continue;
            Size s = new Size(c.w, c.h);
            Size[] ss = map.getOutputSizes(c.fmt);
            if (ss == null || !Arrays.asList(ss).contains(s)) { c.skip = "size " + s + " not offered for format " + c.fmt; continue; }
            long md = map.getOutputMinFrameDuration(c.fmt, s);
            if (md > 1_000_000_000L / c.fps + 100_000) c.skip = "min frame duration " + md + " ns > 1/" + c.fps + " s";
        }
        if (casesArg.equals("all")) return all;
        List<String> want = Arrays.asList(casesArg.split(","));
        List<Case> out = new ArrayList<>();
        for (Case c : all) if (want.contains(c.name)) out.add(c);
        return out;
    }

    /** RAW10 if the logical camera offers it, else RAW_SENSOR; largest size; fps 30 if its min frame duration allows. */
    private Case rawCase(StreamConfigurationMap map) {
        int fmt = -1;
        for (int f : new int[]{ImageFormat.RAW10, ImageFormat.RAW_SENSOR}) {
            Size[] ss = map.getOutputSizes(f);
            if (ss != null && ss.length > 0) { fmt = f; break; }
        }
        if (fmt < 0) {
            Case c = new Case("raw", CameraDevice.TEMPLATE_RECORD, 0, 0, ImageFormat.RAW_SENSOR, 30, ISP_ARLAT);
            c.skip = "no RAW10 / RAW_SENSOR output on logical camera " + CAM;
            return c;
        }
        Size best = null;
        for (Size s : map.getOutputSizes(fmt)) if (best == null || (long) s.getWidth() * s.getHeight() > (long) best.getWidth() * best.getHeight()) best = s;
        long md = map.getOutputMinFrameDuration(fmt, best);
        int fps = 30;
        String note = (fmt == ImageFormat.RAW10 ? "RAW10 " : "RAW_SENSOR ") + best + " min_frame_dur " + md + " ns";
        if (md > 33_433_333L) {
            fps = (int) Math.floor(1e9 / md);
            note += "; 30 fps not possible -> " + fps + " fps";
            if (fps < 5) { Case c = new Case("raw", CameraDevice.TEMPLATE_RECORD, best.getWidth(), best.getHeight(), fmt, 30, ISP_ARLAT); c.skip = note; return c; }
        }
        Case c = new Case("raw", CameraDevice.TEMPLATE_RECORD, best.getWidth(), best.getHeight(), fmt, fps, ISP_ARLAT);
        c.note = note;
        return c;
    }

    private void writePlan(List<Case> cases) throws Exception {
        JSONArray a = new JSONArray();
        for (Case c : cases) {
            a.put(new JSONObject().put("name", c.name).put("template", c.template).put("w", c.w).put("h", c.h).put("fmt", c.fmt)
                    .put("fps", c.fps).put("isp", ISP_NAME[c.isp]).put("skip", c.skip == null ? JSONObject.NULL : c.skip).put("note", c.note));
        }
        JSONObject o = new JSONObject().put("cases", a).put("secs", secs).put("gap_ms", gapMs).put("iso", iso).put("exp_us", expUs)
                .put("focus_dpt", focus).put("max_images", maxImages).put("dry", dry).put("camera", CAM)
                .put("created_wall", new Date().toString()).put("device", android.os.Build.MODEL).put("sdk", android.os.Build.VERSION.SDK_INT);
        writeText(new File(outDir, "plan.json"), o.toString(1));
    }

    // ------------------------------------------------------------------ one case
    private String runCase(CameraManager cm, CameraCharacteristics ch, final Case c) {
        final int cap = secs * c.fps * 2 + 200;
        // capture-callback thread arrays (written only on capT)
        final long[] sTs = new long[cap], sCb = new long[cap], sFn = new long[cap];
        final long[] dTs = new long[cap], dCb = new long[cap], dFn = new long[cap];       // onReadoutStarted (API 34)
        final long[] rTs = new long[cap], rCb = new long[cap], rFn = new long[cap], rSkew = new long[cap], rFd = new long[cap], rExp = new long[cap];
        final int[] rDepth = new int[cap], rIso = new int[cap];
        final int[] n = new int[8];   // 0 started, 1 results, 2 images, 3 failed, 4 lost, 5 readout
        // image thread arrays (written only on imgT)
        final long[] iTs = new long[cap], iCb = new long[cap];
        final JSONObject meta = new JSONObject();
        final AtomicBoolean fin = new AtomicBoolean(false);
        final String[] err = {null};
        final CameraDevice[] devH = {null};
        final CountDownLatch opened = new CountDownLatch(1), closed = new CountDownLatch(1), cfgd = new CountDownLatch(1);
        final CameraCaptureSession[] sessH = {null};
        HandlerThread capT = new HandlerThread("cap"), imgT = new HandlerThread("img");
        capT.start(); imgT.start();
        final Handler capH = new Handler(capT.getLooper()), imgH = new Handler(imgT.getLooper());
        ImageReader reader = null;
        String status = "ok";
        long tOpen = SystemClock.elapsedRealtimeNanos();
        try {
            meta.put("case", c.name).put("template", c.template).put("w", c.w).put("h", c.h).put("fmt", c.fmt).put("fps", c.fps)
                    .put("isp", ISP_NAME[c.isp]).put("note", c.note).put("secs", secs).put("max_images", maxImages)
                    .put("t_open_ns", tOpen);
            reader = ImageReader.newInstance(c.w, c.h, c.fmt, maxImages);
            reader.setOnImageAvailableListener(r -> {
                long now = SystemClock.elapsedRealtimeNanos();
                Image im;
                while (true) {
                    try { im = r.acquireNextImage(); } catch (IllegalStateException e) { break; }
                    if (im == null) break;
                    if (n[2] < cap) { iTs[n[2]] = im.getTimestamp(); iCb[n[2]] = now; n[2]++; }
                    im.close();
                }
            }, imgH);
            cm.openCamera(CAM, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice d) {
                    if (fin.get() || abort) { d.close(); return; }      // opened after timeout / abort: close at once
                    devH[0] = d; curDev = d; opened.countDown();
                }
                @Override public void onDisconnected(CameraDevice d) { err[0] = "disconnected"; d.close(); opened.countDown(); }
                @Override public void onError(CameraDevice d, int e) { err[0] = "camera error " + e; d.close(); opened.countDown(); }
                @Override public void onClosed(CameraDevice d) { closed.countDown(); }
            }, capH);
            if (!opened.await(5, TimeUnit.SECONDS)) throw new Exception("open timeout");
            if (devH[0] == null) throw new Exception("open failed: " + err[0]);
            if (abort) throw new Exception("aborted");
            meta.put("t_opened_ns", SystemClock.elapsedRealtimeNanos());
            final CameraDevice dev = devH[0];
            final CaptureRequest.Builder rb = dev.createCaptureRequest(c.template);
            rb.addTarget(reader.getSurface());
            final CaptureRequest.Key<Integer> ldc = applyRequest(rb, ch, c, meta);
            OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
            dev.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR, Collections.singletonList(oc), capH::post,
                    new CameraCaptureSession.StateCallback() {
                        @Override public void onConfigured(CameraCaptureSession s) { sessH[0] = s; cfgd.countDown(); }
                        @Override public void onConfigureFailed(CameraCaptureSession s) { err[0] = "configure failed"; cfgd.countDown(); }
                    }));
            if (!cfgd.await(5, TimeUnit.SECONDS)) throw new Exception("configure timeout");
            if (sessH[0] == null) throw new Exception(String.valueOf(err[0]));
            meta.put("t_configured_ns", SystemClock.elapsedRealtimeNanos());
            CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                @Override public void onCaptureStarted(CameraCaptureSession s, CaptureRequest q, long ts, long fn) {
                    long now = SystemClock.elapsedRealtimeNanos();
                    if (n[0] < cap) { sTs[n[0]] = ts; sCb[n[0]] = now; sFn[n[0]] = fn; n[0]++; }
                }
                @Override public void onReadoutStarted(CameraCaptureSession s, CaptureRequest q, long ts, long fn) {
                    long now = SystemClock.elapsedRealtimeNanos();
                    if (n[5] < cap) { dTs[n[5]] = ts; dCb[n[5]] = now; dFn[n[5]] = fn; n[5]++; }
                }
                @Override public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest q, TotalCaptureResult res) {
                    long now = SystemClock.elapsedRealtimeNanos();
                    int k = n[1];
                    if (k >= cap) return;
                    rTs[k] = lv(res.get(CaptureResult.SENSOR_TIMESTAMP));
                    rCb[k] = now;
                    rFn[k] = res.getFrameNumber();
                    Byte pd = res.get(CaptureResult.REQUEST_PIPELINE_DEPTH);
                    rDepth[k] = pd == null ? -1 : pd;
                    rSkew[k] = lv(res.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW));
                    rFd[k] = lv(res.get(CaptureResult.SENSOR_FRAME_DURATION));
                    rExp[k] = lv(res.get(CaptureResult.SENSOR_EXPOSURE_TIME));
                    Integer is = res.get(CaptureResult.SENSOR_SENSITIVITY);
                    rIso[k] = is == null ? -1 : is;
                    n[1] = k + 1;
                    if (k == 0) try { meta.put("first_result", firstResult(res)); } catch (Exception ignored) { }
                }
                @Override public void onCaptureFailed(CameraCaptureSession s, CaptureRequest q, CaptureFailure f) { n[3]++; }
                @Override public void onCaptureBufferLost(CameraCaptureSession s, CaptureRequest q, Surface t, long fn) { n[4]++; }
            };
            try { sessH[0].setRepeatingRequest(rb.build(), cb, capH); }
            catch (Exception e) {
                if (ldc == null) throw e;
                rb.set(ldc, null);        // never lose the case to the Samsung vendor tag (same as arlat)
                meta.put("samsung_ldc_request", "set 0 -> request failed (" + e + "), removed");
                sessH[0].setRepeatingRequest(rb.build(), cb, capH);
            }
            meta.put("t_repeating_ns", SystemClock.elapsedRealtimeNanos());
            sleepAbortable(secs * 1000L);
            if (abort) status = "aborted";
            try { sessH[0].stopRepeating(); } catch (Exception ignored) { }
        } catch (Throwable t) {
            status = "error " + t;
            Log.e(TAG, "case " + c.name, t);
        } finally {
            fin.set(true);
            if (sessH[0] != null) try { sessH[0].close(); } catch (Throwable ignored) { }
            CameraDevice d = devH[0];
            if (d != null) {
                try { d.close(); } catch (Throwable ignored) { }
                try { closed.await(3, TimeUnit.SECONDS); } catch (InterruptedException ignored) { }
            }
            curDev = null;
            if (reader != null) try { reader.close(); } catch (Throwable ignored) { }
            capT.quitSafely(); imgT.quitSafely();
            try { capT.join(2000); imgT.join(2000); } catch (InterruptedException ignored) { }
        }
        // ---- write per-frame CSV (threads joined: arrays are safe to read)
        TreeMap<Long, long[]> rows = new TreeMap<>();
        // cols: 0 fn,1 t_sensor,2 t_shutter_cb,3 t_readout_ts,4 t_readout_cb,5 t_result_cb,6 t_image_cb,7 depth,8 skew,9 frame_dur,10 exp,11 iso
        for (int i = 0; i < n[1]; i++) {
            long[] r = row(rows, rTs[i]);
            r[0] = rFn[i]; r[5] = rCb[i]; r[7] = rDepth[i]; r[8] = rSkew[i]; r[9] = rFd[i]; r[10] = rExp[i]; r[11] = rIso[i];
        }
        for (int i = 0; i < n[0]; i++) row(rows, sTs[i])[2] = sCb[i];
        for (int i = 0; i < n[2]; i++) row(rows, iTs[i])[6] = iCb[i];
        // readout timestamp differs from SENSOR_TIMESTAMP: pair with onCaptureStarted by frame number
        java.util.HashMap<Long, Long> fnTs = new java.util.HashMap<>();
        for (int i = 0; i < n[0]; i++) fnTs.put(sFn[i], sTs[i]);
        for (int i = 0; i < n[5]; i++) { Long t = fnTs.get(dFn[i]); if (t == null) continue; long[] r = row(rows, t); r[3] = dTs[i]; r[4] = dCb[i]; }
        List<Double> lat = new ArrayList<>();
        List<Integer> depth = new ArrayList<>();
        List<Long> skew = new ArrayList<>();
        try (PrintWriter w = new PrintWriter(new FileWriter(new File(outDir, c.name + ".csv")))) {
            w.println("frame_number,t_sensor_ns,t_shutter_cb_ns,t_readout_ns,t_readout_cb_ns,t_result_cb_ns,t_image_cb_ns,pipeline_depth,skew_ns,frame_dur_ns,exp_ns,iso");
            for (Map.Entry<Long, long[]> e : rows.entrySet()) {
                long[] r = e.getValue();
                w.println(r[0] + "," + e.getKey() + "," + r[2] + "," + r[3] + "," + r[4] + "," + r[5] + "," + r[6] + "," + r[7] + "," + r[8] + "," + r[9] + "," + r[10] + "," + r[11]);
                if (r[6] > 0) lat.add((r[6] - e.getKey()) / 1e6);
                if (r[7] >= 0) depth.add((int) r[7]);
                if (r[8] > 0) skew.add(r[8]);
            }
        } catch (Exception ex) { status += "; csv " + ex; }
        Collections.sort(lat); Collections.sort(depth); Collections.sort(skew);
        double fpsGot = 0;
        if (n[2] > 1) {
            long a = Long.MAX_VALUE, b = Long.MIN_VALUE;
            for (int i = 0; i < n[2]; i++) { a = Math.min(a, iTs[i]); b = Math.max(b, iTs[i]); }
            if (b > a) fpsGot = (n[2] - 1) / ((b - a) / 1e9);
        }
        try {
            meta.put("status", status).put("err", err[0] == null ? JSONObject.NULL : err[0]).put("n_started", n[0]).put("n_readout", n[5])
                    .put("n_results", n[1]).put("n_images", n[2]).put("n_failed", n[3]).put("n_buffer_lost", n[4])
                    .put("t_closed_ns", SystemClock.elapsedRealtimeNanos())
                    .put("first_image_ns", n[2] > 0 ? iCb[0] : JSONObject.NULL);
            writeText(new File(outDir, c.name + ".json"), meta.toString(1));
        } catch (Exception ignored) { }
        String l = String.format(Locale.US, "%s,%s,%d,%d,%.2f,%s,%s,%s,%s,%s", c.name, csvq(status), n[2], n[1], fpsGot,
                lat.isEmpty() ? "" : String.format(Locale.US, "%.2f", lat.get(lat.size() / 2)),
                lat.isEmpty() ? "" : String.format(Locale.US, "%.2f", lat.get(lat.size() - 1)),
                depth.isEmpty() ? "" : String.valueOf(depth.get(depth.size() / 2)),
                skew.isEmpty() ? "" : String.format(Locale.US, "%.3f", skew.get(skew.size() / 2) / 1e6), csvq(c.note));
        show("case " + c.name + ": " + status + "\nimages " + n[2] + " lat p50 " + (lat.isEmpty() ? "-" : lat.get(lat.size() / 2)) + " ms");
        return l;
    }

    private static long[] row(TreeMap<Long, long[]> rows, long ts) {
        long[] r = rows.get(ts);
        if (r == null) { r = new long[12]; Arrays.fill(r, -1); rows.put(ts, r); }
        return r;
    }

    private static long lv(Long v) { return v == null ? -1 : v; }

    private static String csvq(String s) { return s == null ? "" : "\"" + s.replace("\"", "'") + "\""; }

    /** Request settings. Returns the Samsung LDC key when set (for the retry without it). */
    private CaptureRequest.Key<Integer> applyRequest(CaptureRequest.Builder rb, CameraCharacteristics ch, Case c, JSONObject meta) throws Exception {
        long frameDur = 1_000_000_000L / c.fps;
        rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(c.fps, c.fps));
        rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
        rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
        int[] dm = ch.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
        if (has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF))
            rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
        meta.put("aosp_distortion_request", has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF) ? "OFF" : "not offered");
        CaptureRequest.Key<Integer> ldc = null;
        try { ldc = new CaptureRequest.Key<>(LDC_TAG, Integer.class); rb.set(ldc, 0); meta.put("samsung_ldc_request", "set 0 (int32)"); }
        catch (Throwable e) { ldc = null; meta.put("samsung_ldc_request", "not set: " + e); }
        if (focus >= 0) {
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focus);
        }
        long e = Math.min(expUs * 1000L, frameDur);
        Range<Long> er = ch.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
        if (er != null) e = er.clamp(e);
        int s = iso;
        Range<Integer> sr = ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
        if (sr != null) s = sr.clamp(s);
        rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
        rb.set(CaptureRequest.SENSOR_EXPOSURE_TIME, e);
        rb.set(CaptureRequest.SENSOR_SENSITIVITY, s);
        rb.set(CaptureRequest.SENSOR_FRAME_DURATION, frameDur);
        meta.put("exposure_ns_set", e).put("iso_set", s).put("frame_duration_ns_set", frameDur);
        if (c.isp == ISP_ARLAT || c.isp == ISP_MIN) {
            if (has(ch.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES), CaptureRequest.NOISE_REDUCTION_MODE_OFF))
                rb.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
            if (has(ch.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CaptureRequest.EDGE_MODE_OFF))
                rb.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
        }
        if (c.isp == ISP_MIN) {
            if (has(ch.get(CameraCharacteristics.HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES), CaptureRequest.HOT_PIXEL_MODE_OFF))
                rb.set(CaptureRequest.HOT_PIXEL_MODE, CaptureRequest.HOT_PIXEL_MODE_OFF);
            if (has(ch.get(CameraCharacteristics.TONEMAP_AVAILABLE_TONE_MAP_MODES), CaptureRequest.TONEMAP_MODE_FAST))
                rb.set(CaptureRequest.TONEMAP_MODE, CaptureRequest.TONEMAP_MODE_FAST);
            rb.set(CaptureRequest.COLOR_CORRECTION_MODE, CaptureRequest.COLOR_CORRECTION_MODE_FAST);
            int[] ab = ch.get(CameraCharacteristics.COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES);
            if (has(ab, CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE_OFF))
                rb.set(CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE, CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE_OFF);
            else if (has(ab, CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE_FAST))
                rb.set(CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE, CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE_FAST);
            rb.set(CaptureRequest.SHADING_MODE, CaptureRequest.SHADING_MODE_FAST);
            rb.set(CaptureRequest.STATISTICS_FACE_DETECT_MODE, CaptureRequest.STATISTICS_FACE_DETECT_MODE_OFF);
            rb.set(CaptureRequest.STATISTICS_LENS_SHADING_MAP_MODE, CaptureRequest.STATISTICS_LENS_SHADING_MAP_MODE_OFF);
            rb.set(CaptureRequest.STATISTICS_HOT_PIXEL_MAP_MODE, false);
        }
        CaptureRequest q = rb.build();
        meta.put("request", new JSONObject()
                .put("nr", str(q.get(CaptureRequest.NOISE_REDUCTION_MODE))).put("edge", str(q.get(CaptureRequest.EDGE_MODE)))
                .put("hot_pixel", str(q.get(CaptureRequest.HOT_PIXEL_MODE))).put("tonemap", str(q.get(CaptureRequest.TONEMAP_MODE)))
                .put("cc", str(q.get(CaptureRequest.COLOR_CORRECTION_MODE))).put("aberration", str(q.get(CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE)))
                .put("shading", str(q.get(CaptureRequest.SHADING_MODE))).put("vstab", str(q.get(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE)))
                .put("ois", str(q.get(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE))).put("capture_intent", str(q.get(CaptureRequest.CONTROL_CAPTURE_INTENT))));
        return ldc;
    }

    private static String str(Object o) { return String.valueOf(o); }

    private static JSONObject firstResult(CaptureResult r) throws Exception {
        return new JSONObject()
                .put("nr", str(r.get(CaptureResult.NOISE_REDUCTION_MODE))).put("edge", str(r.get(CaptureResult.EDGE_MODE)))
                .put("hot_pixel", str(r.get(CaptureResult.HOT_PIXEL_MODE))).put("tonemap", str(r.get(CaptureResult.TONEMAP_MODE)))
                .put("cc", str(r.get(CaptureResult.COLOR_CORRECTION_MODE))).put("aberration", str(r.get(CaptureResult.COLOR_CORRECTION_ABERRATION_MODE)))
                .put("shading", str(r.get(CaptureResult.SHADING_MODE))).put("vstab", str(r.get(CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE)))
                .put("ois", str(r.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE))).put("dc", str(r.get(CaptureResult.DISTORTION_CORRECTION_MODE)))
                .put("exp_ns", str(r.get(CaptureResult.SENSOR_EXPOSURE_TIME))).put("iso", str(r.get(CaptureResult.SENSOR_SENSITIVITY)))
                .put("frame_dur_ns", str(r.get(CaptureResult.SENSOR_FRAME_DURATION))).put("skew_ns", str(r.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)))
                .put("pipeline_depth", str(r.get(CaptureResult.REQUEST_PIPELINE_DEPTH))).put("focus", str(r.get(CaptureResult.LENS_FOCUS_DISTANCE)))
                .put("ae_mode", str(r.get(CaptureResult.CONTROL_AE_MODE))).put("capture_intent", str(r.get(CaptureResult.CONTROL_CAPTURE_INTENT)));
    }

    // ------------------------------------------------------------------ characteristics
    private void writeCaps(CameraManager cm, CameraCharacteristics ch) throws Exception {
        JSONObject o = camCaps(ch);
        JSONObject phys = new JSONObject();
        for (String p : ch.getPhysicalCameraIds()) {
            try {
                CameraCharacteristics pc = cm.getCameraCharacteristics(p);
                phys.put(p, new JSONObject().put("raw", streams(pc, new int[]{ImageFormat.RAW10, ImageFormat.RAW_SENSOR, ImageFormat.RAW12}, null))
                        .put("focal", arr(pc.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)))
                        .put("timestamp_source", str(pc.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE))));
            } catch (Exception e) { phys.put(p, "error " + e); }
        }
        o.put("physical", phys);
        JSONArray ids = new JSONArray();
        for (String id : cm.getCameraIdList()) {
            Integer f = cm.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING);
            ids.put(id + ":facing" + f);
        }
        o.put("camera_ids", ids);
        writeText(new File(outDir, "caps.json"), o.toString(1));
    }

    private JSONObject camCaps(CameraCharacteristics ch) throws Exception {
        JSONObject o = new JSONObject();
        o.put("camera", CAM);
        o.put("hardware_level", str(ch.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)));
        Integer tsrc = ch.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
        o.put("timestamp_source", tsrc == null ? "null" : tsrc == 1 ? "REALTIME" : tsrc == 0 ? "UNKNOWN" : str(tsrc));
        o.put("request_pipeline_max_depth", str(ch.get(CameraCharacteristics.REQUEST_PIPELINE_MAX_DEPTH)));
        o.put("request_partial_result_count", str(ch.get(CameraCharacteristics.REQUEST_PARTIAL_RESULT_COUNT)));
        o.put("sync_max_latency", str(ch.get(CameraCharacteristics.SYNC_MAX_LATENCY)));
        try { o.put("sensor_readout_timestamp", str(ch.get(CameraCharacteristics.SENSOR_READOUT_TIMESTAMP))); } catch (Throwable t) { o.put("sensor_readout_timestamp", "n/a"); }
        int[] capsArr = ch.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
        o.put("capabilities", arr(capsArr));
        o.put("capability_raw", has(capsArr, CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_RAW));
        o.put("capability_logical_multi_camera", has(capsArr, CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA));
        o.put("nr_modes", arr(ch.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES)));
        o.put("edge_modes", arr(ch.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES)));
        o.put("hot_pixel_modes", arr(ch.get(CameraCharacteristics.HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES)));
        o.put("tonemap_modes", arr(ch.get(CameraCharacteristics.TONEMAP_AVAILABLE_TONE_MAP_MODES)));
        o.put("aberration_modes", arr(ch.get(CameraCharacteristics.COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES)));
        o.put("video_stab_modes", arr(ch.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES)));
        o.put("ois_modes", arr(ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION)));
        o.put("distortion_modes", arr(ch.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES)));
        Range<Integer>[] fr = ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        o.put("ae_fps_ranges", fr == null ? "null" : Arrays.toString(fr));
        o.put("exposure_range", str(ch.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE)));
        o.put("sensitivity_range", str(ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE)));
        o.put("max_analog_sensitivity", str(ch.get(CameraCharacteristics.SENSOR_MAX_ANALOG_SENSITIVITY)));
        o.put("pixel_array", str(ch.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)));
        o.put("yuv", streams(ch, new int[]{ImageFormat.YUV_420_888},
                new Size[]{new Size(1920, 1080), new Size(1280, 720), new Size(640, 360), new Size(640, 480)}));
        o.put("raw", streams(ch, new int[]{ImageFormat.RAW10, ImageFormat.RAW_SENSOR, ImageFormat.RAW12, ImageFormat.RAW_PRIVATE}, null));
        JSONArray vk = new JSONArray();
        for (CaptureRequest.Key<?> k : ch.getAvailableCaptureRequestKeys()) if (!k.getName().startsWith("android.")) vk.put(k.getName());
        o.put("vendor_request_keys", vk);
        return o;
    }

    /** For each format: sizes (all, or only `only`) with min frame duration and stall duration. */
    private static JSONObject streams(CameraCharacteristics ch, int[] fmts, Size[] only) throws Exception {
        JSONObject o = new JSONObject();
        StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        if (map == null) return o.put("error", "no stream map");
        for (int f : fmts) {
            JSONArray a = new JSONArray();
            Size[] ss;
            try { ss = map.getOutputSizes(f); } catch (Exception e) { ss = null; }
            if (ss != null) for (Size s : ss) {
                if (only != null && !Arrays.asList(only).contains(s)) continue;
                a.put(s + " min_dur " + map.getOutputMinFrameDuration(f, s) + " stall " + map.getOutputStallDuration(f, s));
            }
            o.put(fmtName(f), a);
        }
        return o;
    }

    private static String fmtName(int f) {
        switch (f) {
            case ImageFormat.YUV_420_888: return "YUV_420_888";
            case ImageFormat.RAW10: return "RAW10";
            case ImageFormat.RAW12: return "RAW12";
            case ImageFormat.RAW_SENSOR: return "RAW_SENSOR";
            case ImageFormat.RAW_PRIVATE: return "RAW_PRIVATE";
            default: return "fmt" + f;
        }
    }

    private static JSONArray arr(int[] a) { JSONArray j = new JSONArray(); if (a != null) for (int v : a) j.put(v); return j; }
    private static JSONArray arr(float[] a) throws Exception { JSONArray j = new JSONArray(); if (a != null) for (float v : a) j.put((double) v); return j; }
    private static boolean has(int[] a, int v) { if (a != null) for (int x : a) if (x == v) return true; return false; }

    private static void writeText(File f, String s) throws Exception {
        try (PrintWriter w = new PrintWriter(new FileWriter(f))) { w.print(s); }
    }
}
