package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Size;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.function.Consumer;

/**
 * mode peek (0.6.9, 2026-10-24): low-load framing view for the lens-calibration board page.
 * One JPEG still about every periodMs (default 1000 = 1 fps) from the main camera "0" with the recording settings
 * (manual focus focusDpt, manual exposure expUs / iso) -> files/peek.jpg (written as peek.tmp then renamed).
 * No repeating request: the sensor only streams when a still is requested. Stops after secs (default 300) or on onPause.
 */
class PeekRunner {
    final Context ctx;
    final Consumer<String> status;
    Consumer<String> okView;      // 0.6.10: called once per still with the text of files/peek_ok.txt ("" = none); the PC writes it
    float focusDpt = 0.5f;
    int expUs = 2000, iso = 500, periodMs = 1000, secs = 300;
    private HandlerThread th;
    private Handler h;
    private CameraDevice dev;
    private CameraCaptureSession ses;
    private ImageReader reader;
    private volatile boolean on;
    private long t0;
    private int n;

    PeekRunner(Context ctx, Consumer<String> status) { this.ctx = ctx; this.status = status; }

    void start() throws Exception {
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        StreamConfigurationMap map = cm.getCameraCharacteristics("0").get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        Size best = null;
        for (Size s : map.getOutputSizes(ImageFormat.JPEG)) {     // smallest 16:9 size >= 960 wide (keeps the transfer small)
            if (s.getWidth() * 9 == s.getHeight() * 16 && s.getWidth() >= 960 && (best == null || s.getWidth() < best.getWidth())) best = s;
        }
        if (best == null) best = new Size(1920, 1080);
        th = new HandlerThread("peek"); th.start(); h = new Handler(th.getLooper());
        reader = ImageReader.newInstance(best.getWidth(), best.getHeight(), ImageFormat.JPEG, 2);
        final File dir = ctx.getExternalFilesDir(null);
        reader.setOnImageAvailableListener(r -> {
            try (Image im = r.acquireLatestImage()) {
                if (im == null) return;
                ByteBuffer b = im.getPlanes()[0].getBuffer(); byte[] a = new byte[b.remaining()]; b.get(a);
                File tmp = new File(dir, "peek.tmp"), out = new File(dir, "peek.jpg");
                try (FileOutputStream f = new FileOutputStream(tmp)) { f.write(a); }
                tmp.renameTo(out);
                n++;
                status.accept("peek " + n);
            } catch (Exception e) { status.accept("peek write error " + e); }
        }, h);
        on = true; t0 = System.currentTimeMillis();
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                dev = d;
                try {
                    d.createCaptureSession(Collections.singletonList(reader.getSurface()), new CameraCaptureSession.StateCallback() {
                        @Override public void onConfigured(CameraCaptureSession s) { ses = s; h.post(PeekRunner.this::shot); }
                        @Override public void onConfigureFailed(CameraCaptureSession s) { status.accept("peek: configure failed"); }
                    }, h);
                } catch (Exception e) { status.accept("peek session error " + e); }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.accept("peek camera error " + e); d.close(); }
        }, h);
    }

    private void shot() {
        if (!on || ses == null) return;
        if (okView != null) {          // the PC writes peek_ok.txt when the framing matches a still-missing pose
            String t = "";
            try { File f = new File(ctx.getExternalFilesDir(null), "peek_ok.txt");
                  if (f.exists()) t = new String(java.nio.file.Files.readAllBytes(f.toPath()), "UTF-8").trim(); }
            catch (Exception ignored) { }
            okView.accept(t);
        }
        if (System.currentTimeMillis() - t0 > secs * 1000L) { status.accept("peek: time limit"); stop(); return; }
        try {
            CaptureRequest.Builder rb = dev.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
            rb.addTarget(reader.getSurface());
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
            rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            rb.set(CaptureRequest.SENSOR_EXPOSURE_TIME, expUs * 1000L);
            rb.set(CaptureRequest.SENSOR_SENSITIVITY, iso);
            rb.set(CaptureRequest.JPEG_QUALITY, (byte) 70);
            ses.capture(rb.build(), null, h);
        } catch (Exception e) { status.accept("peek capture error " + e); }
        h.postDelayed(this::shot, periodMs);
    }

    void stop() {
        on = false;
        try { if (ses != null) ses.close(); } catch (Exception ignored) { }
        try { if (dev != null) dev.close(); } catch (Exception ignored) { }
        try { if (reader != null) reader.close(); } catch (Exception ignored) { }
        ses = null; dev = null; reader = null;
        if (th != null) th.quitSafely();
    }
}
