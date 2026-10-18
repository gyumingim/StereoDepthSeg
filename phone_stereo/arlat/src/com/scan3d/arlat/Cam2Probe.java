package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Range;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Collections;
import java.util.Locale;

/**
 * Camera2 delivery latency for the OpenVINS path: logical camera "0", physical lens "2" (ultrawide, the VIO lens),
 * YUV 640x480 @ 30 fps. Logs per image SENSOR_TIMESTAMP and elapsedRealtimeNanos when onImageAvailable ran
 * -> files/cam2lat_<ms>.csv. Latency = t_arrive - t_sensor (same clock when the timestamp source is REALTIME).
 */
class Cam2Probe {
    interface Status { void show(String s); }

    private final Context ctx;
    private final Status status;
    private HandlerThread thread;
    private CameraDevice dev;
    private ImageReader reader;
    private PrintWriter w;
    private int n;

    Cam2Probe(Context ctx, Status status) { this.ctx = ctx; this.status = status; }

    void start(final int secs, String physId) throws Exception {
        thread = new HandlerThread("cam2");
        thread.start();
        final Handler h = new Handler(thread.getLooper());
        File out = new File(ctx.getExternalFilesDir(null), "cam2lat_" + System.currentTimeMillis() + ".csv");
        w = new PrintWriter(new FileWriter(out));
        w.println("t_sensor_ns,t_arrive_ns");
        reader = ImageReader.newInstance(640, 480, ImageFormat.YUV_420_888, 4);
        reader.setOnImageAvailableListener(r -> {
            long now = SystemClock.elapsedRealtimeNanos();
            Image im = r.acquireLatestImage();
            if (im == null) return;
            long ts = im.getTimestamp();
            im.close();
            w.println(ts + "," + now);
            if (++n % 30 == 0) status.show(String.format(Locale.US, "cam2 frames %d\nlatency %.1f ms", n, (now - ts) / 1e6));
        }, h);
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                dev = d;
                try {
                    OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
                    oc.setPhysicalCameraId(physId);
                    final CaptureRequest.Builder rb = d.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                    rb.addTarget(reader.getSurface());
                    rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(30, 30));
                    d.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,
                            Collections.singletonList(oc), ctx.getMainExecutor(), new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    try { s.setRepeatingRequest(rb.build(), null, h); }
                                    catch (Exception e) { status.show("repeat: " + e); }
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) { status.show("configure failed"); }
                            }));
                } catch (Exception e) { status.show("session: " + e); }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.show("camera error " + e); d.close(); }
        }, h);
        h.postDelayed(() -> {
            stop();
            status.show("DONE cam2 " + n + " frames -> " + out.getName());
            android.util.Log.i("arlat", "DONE " + out);
        }, secs * 1000L);
    }

    void stop() {
        if (dev != null) { dev.close(); dev = null; }
        if (w != null) { w.flush(); w.close(); w = null; }
    }
}
