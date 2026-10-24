package com.scan3d.arlat;

import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Range;
import android.util.Size;

import org.json.JSONObject;

import java.util.Collections;

/**
 * 0.6.11 (user decision 2026-10-24): exposure FIXED (default 2 ms), ISO chosen ONCE at start, then locked for the run.
 * Before the real session (recording / OpenVINS / peek) is created, a throw-away session on its own YUV ImageReader
 * (same size as the real stream, frames discarded) runs the camera's AE (CONTROL_AE_MODE_ON, target fps [fps,fps],
 * fixed focus) until CONTROL_AE_STATE is CONVERGED (or FLASH_REQUIRED / LOCKED) for 3 frames in a row, or timeoutMs.
 * From the last capture result: brightness = e_ae * iso_ae * post_raw_boost_ae / 100, and
 *   iso = clamp(round(brightness / expNs * scale), SENSOR_INFO_SENSITIVITY_RANGE).
 * The AE session is closed before done() is called, so no AE-phase frame reaches the encoder / bridge / peek file.
 * No AE result at all -> iso = FALLBACK_ISO (error logged). Everything goes to json() for meta.json.
 */
class AutoIso {
    interface Done { void done(AutoIso a); }
    static final int FALLBACK_ISO = 500;

    // inputs
    Size size = new Size(1920, 1080);
    Range<Integer> fps = new Range<>(60, 60);
    float focusDpt = 0.5f;             // >= 0: AF off at this distance (diopters)
    int evComp = 0;
    long expNs = 2_000_000L;           // the fixed exposure the run will use
    double scale = 1.0;                // intent iso_target_scale (brightness bias)
    long timeoutMs = 1400;

    // outputs
    long eAeNs; int isoAe, boostAe = 100, stateAtLock = -1, frames, iso, isoRaw;
    long convergeMs = -1, phaseMs;
    boolean converged;
    String error;
    private Range<Integer> sensRange;
    private Integer maxAnalog;

    private ImageReader reader;
    private CameraCaptureSession ses;
    private long t0, tFirst;
    private int inRow;
    private boolean finished;
    private Handler h;
    private Done done;
    private final Runnable timeout = () -> finish(false);

    void run(CameraDevice d, CameraCharacteristics ch, Handler h, Done done) {
        this.h = h; this.done = done;
        t0 = SystemClock.elapsedRealtime();
        sensRange = ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
        maxAnalog = ch.get(CameraCharacteristics.SENSOR_MAX_ANALOG_SENSITIVITY);
        try {
            reader = ImageReader.newInstance(size.getWidth(), size.getHeight(), ImageFormat.YUV_420_888, 3);
            reader.setOnImageAvailableListener(r -> { Image im = r.acquireLatestImage(); if (im != null) im.close(); }, h);
            d.createCaptureSession(Collections.singletonList(reader.getSurface()), new CameraCaptureSession.StateCallback() {
                @Override public void onConfigured(CameraCaptureSession s) {
                    ses = s;
                    try {
                        CaptureRequest.Builder rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
                        rb.addTarget(reader.getSurface());
                        rb.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
                        rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
                        rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps);
                        rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                        rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                        if (evComp != 0) rb.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, evComp);
                        if (focusDpt >= 0) {
                            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
                        }
                        s.setRepeatingRequest(rb.build(), new CameraCaptureSession.CaptureCallback() {
                            @Override public void onCaptureCompleted(CameraCaptureSession ss, CaptureRequest q, TotalCaptureResult r) { onResult(r); }
                        }, h);
                        h.postDelayed(timeout, timeoutMs);
                    } catch (Exception e) { error = "request: " + e; finish(false); }
                }
                @Override public void onConfigureFailed(CameraCaptureSession s) { error = "configure failed"; finish(false); }
            }, h);
        } catch (Exception e) { error = "session: " + e; finish(false); }
    }

    private void onResult(TotalCaptureResult r) {
        if (finished) return;
        if (frames++ == 0) tFirst = SystemClock.elapsedRealtime();
        Long e = r.get(CaptureResult.SENSOR_EXPOSURE_TIME);
        Integer s = r.get(CaptureResult.SENSOR_SENSITIVITY), st = r.get(CaptureResult.CONTROL_AE_STATE);
        Integer b = null;
        try { b = r.get(CaptureResult.CONTROL_POST_RAW_SENSITIVITY_BOOST); } catch (Throwable ignored) { }
        if (e != null && s != null && e > 0 && s > 0) { eAeNs = e; isoAe = s; boostAe = b == null ? 100 : b; }
        stateAtLock = st == null ? -1 : st;
        boolean ok = st != null && (st == CaptureResult.CONTROL_AE_STATE_CONVERGED || st == CaptureResult.CONTROL_AE_STATE_FLASH_REQUIRED
                || st == CaptureResult.CONTROL_AE_STATE_LOCKED);
        inRow = ok ? inRow + 1 : 0;
        if (frames >= 4 && inRow >= 3 && isoAe > 0) finish(true);
    }

    private void finish(boolean conv) {
        if (finished) return;
        finished = true;
        h.removeCallbacks(timeout);
        converged = conv;
        convergeMs = SystemClock.elapsedRealtime() - t0;
        if (isoAe > 0 && eAeNs > 0 && expNs > 0) {
            double bright = eAeNs * (double) isoAe * boostAe / 100.0;
            isoRaw = (int) Math.round(bright / expNs * scale);
            iso = sensRange != null ? sensRange.clamp(isoRaw) : isoRaw;
        } else {
            iso = FALLBACK_ISO;
            if (error == null) error = "no AE result";
        }
        try { if (ses != null) { ses.stopRepeating(); ses.close(); } } catch (Exception ignored) { }
        final ImageReader rd = reader;
        // let the closing session drain before the reader goes away; the caller creates its own session right away
        h.postDelayed(() -> { try { if (rd != null) rd.close(); } catch (Exception ignored) { } }, 500);
        phaseMs = SystemClock.elapsedRealtime() - t0;
        android.util.Log.i("arlat", "auto iso: " + json());
        h.post(() -> done.done(this));
    }

    JSONObject json() {
        JSONObject j = new JSONObject();
        try {
            j.put("auto_iso", true).put("e_ae_ns", eAeNs).put("iso_ae", isoAe).put("post_raw_boost_ae", boostAe)
                    .put("exposure_ns", expNs).put("iso_target_scale", scale).put("iso_raw", isoRaw).put("iso", iso)
                    .put("ae_converged", converged).put("ae_state_at_lock", stateAtLock)
                    .put("ae_converge_ms", convergeMs).put("ae_first_result_ms", tFirst > 0 ? tFirst - t0 : -1)
                    .put("ae_frames", frames).put("ae_timeout_ms", timeoutMs)
                    .put("sensitivity_range", String.valueOf(sensRange)).put("max_analog_sensitivity", String.valueOf(maxAnalog))
                    .put("iso_above_analog_max", maxAnalog != null && iso > maxAnalog)
                    .put("ae_size", String.valueOf(size)).put("ae_fps_range", String.valueOf(fps))
                    .put("ae_phase", "separate session, own YUV reader, frames discarded (not recorded / not fed)")
                    .put("error", error == null ? "none" : error);
        } catch (Exception ignored) { }
        return j;
    }
}
