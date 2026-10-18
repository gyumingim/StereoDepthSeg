package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Range;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Collections;
import java.util.Locale;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Live OpenVINS on the phone: lens 2 (physical "2" of logical "0") YUV 640x480 @ 30 fps + gyro/accel at the fastest
 * rate -> ov_bridge child process (lib/arm64-v8a/libov_bridge.so, protocol scan3d/ov_bridge/PROTOCOL.md, --remap
 * undistorts to lens0u) -> one JSON line per pose to the Streamer:
 *   {"src":"ov","t_sensor":..,"t_arrive":..,"t_out":..,"status":..,"p":[..],"q":[..],"ms":..,"n_slam":..,"n_msckf":..}
 * t_sensor = SENSOR_TIMESTAMP (start of exposure), t_arrive = image callback, t_out = pose line read (all elapsedRealtimeNanos).
 * Config files come from files/ov/ (pushed by vio_bench_server.py, made by make_phone_ov.py).
 */
class OvRunner implements SensorEventListener {
    interface Status { void show(String s); }

    private static final int W = 640, H = 480, MAGIC = 0x4F564252;
    private final Context ctx;
    private final Status status;
    private final Streamer out;
    private HandlerThread camThread, imuThread;
    private CameraDevice dev;
    private ImageReader reader;
    private Process proc;
    private volatile boolean running;
    private final LinkedBlockingQueue<byte[]> toBridge = new LinkedBlockingQueue<>(64);
    /** t_ns sent to the bridge (mid exposure) -> {t_sensor, t_arrive} */
    private final ConcurrentHashMap<Long, long[]> sent = new ConcurrentHashMap<>();
    private volatile long midOffsetNs;          // (exposure + rolling shutter skew) / 2 from the latest capture result
    private float[] accPrev, accLast;
    private long accPrevT, accLastT;
    private int nCam, nPose, nImu, camDropped;
    private String lastStatus = "-";

    OvRunner(Context ctx, Status status, Streamer out) { this.ctx = ctx; this.status = status; this.out = out; }

    void start() throws Exception {
        running = true;
        File dir = new File(ctx.getExternalFilesDir(null), "ov");
        String lib = ctx.getApplicationInfo().nativeLibraryDir;
        ProcessBuilder pb = new ProcessBuilder(lib + "/libov_bridge.so", new File(dir, "estimator_config.yaml").getAbsolutePath(),
                "--stdio", "--auto-reset", "--reinit-state", "--init-gate", "0.006,0.15,0.8", "--max-cam-queue", "2",
                "--remap", new File(dir, "lens0u_remap.bin").getAbsolutePath());
        pb.environment().put("LD_LIBRARY_PATH", lib);
        pb.redirectError(new File(dir, "bridge_stderr.log"));
        proc = pb.start();
        Thread w = new Thread(this::writer, "ov-writer");
        Thread r = new Thread(this::readerLoop, "ov-reader");
        w.start();
        r.start();

        imuThread = new HandlerThread("imu");
        imuThread.start();
        SensorManager sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        Handler ih = new Handler(imuThread.getLooper());
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST, ih);
        sm.registerListener(this, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST, ih);

        camThread = new HandlerThread("cam");
        camThread.start();
        final Handler h = new Handler(camThread.getLooper());
        reader = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4);
        reader.setOnImageAvailableListener(this::onImage, h);
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                dev = d;
                try {
                    OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
                    oc.setPhysicalCameraId("2");
                    final CaptureRequest.Builder rb = d.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                    rb.addTarget(reader.getSurface());
                    rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(30, 30));
                    rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                    final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                        @Override public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest q, TotalCaptureResult res) {
                            Long exp = res.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                            Long skew = res.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                            midOffsetNs = ((exp == null ? 0 : exp) + (skew == null ? 0 : skew)) / 2;
                        }
                    };
                    d.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,
                            Collections.singletonList(oc), ctx.getMainExecutor(), new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    try { s.setRepeatingRequest(rb.build(), cb, h); }
                                    catch (Exception e) { status.show("repeat: " + e); }
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) { status.show("configure failed"); }
                            }));
                } catch (Exception e) { status.show("session: " + e); }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.show("camera error " + e); d.close(); }
        }, h);
    }

    void stop() {
        running = false;
        try { ((SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE)).unregisterListener(this); } catch (Exception ignored) { }
        if (dev != null) { dev.close(); dev = null; }
        if (proc != null) { proc.destroy(); proc = null; }
        if (camThread != null) camThread.quitSafely();
        if (imuThread != null) imuThread.quitSafely();
    }

    private void onImage(ImageReader r) {
        long now = SystemClock.elapsedRealtimeNanos();
        Image im = r.acquireLatestImage();
        if (im == null) return;
        long ts = im.getTimestamp();
        Image.Plane y = im.getPlanes()[0];
        ByteBuffer b = y.getBuffer();
        int rs = y.getRowStride();
        ByteBuffer rec = ByteBuffer.allocate(12 + 20 + W * H).order(ByteOrder.LITTLE_ENDIAN);
        long tSend = ts + midOffsetNs;
        rec.putInt(MAGIC).putInt(2).putInt(20 + W * H);
        rec.putLong(tSend).putInt(W).putInt(H).put((byte) 1).put((byte) 0).put((byte) 0).put((byte) 0);
        byte[] row = new byte[W];
        for (int i = 0; i < H; i++) {
            b.position(i * rs);
            b.get(row, 0, W);
            rec.put(row);
        }
        im.close();
        sent.put(tSend, new long[]{ts, now});
        if (sent.size() > 300) sent.clear();
        if (!toBridge.offer(rec.array())) camDropped++;
        nCam++;
    }

    @Override public void onSensorChanged(SensorEvent e) {
        if (e.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            accPrev = accLast; accPrevT = accLastT;
            accLast = e.values.clone(); accLastT = e.timestamp;
            return;
        }
        if (accLast == null) return;
        float[] a = accLast;
        if (accPrev != null && accLastT > accPrevT && e.timestamp <= accLastT) {   // interpolate accel at gyro time
            double f = (double) (e.timestamp - accPrevT) / (accLastT - accPrevT);
            f = Math.max(0, Math.min(1, f));
            a = new float[3];
            for (int i = 0; i < 3; i++) a[i] = (float) (accPrev[i] + f * (accLast[i] - accPrev[i]));
        }
        ByteBuffer rec = ByteBuffer.allocate(12 + 56).order(ByteOrder.LITTLE_ENDIAN);
        rec.putInt(MAGIC).putInt(1).putInt(56).putLong(e.timestamp);
        rec.putDouble(e.values[0]).putDouble(e.values[1]).putDouble(e.values[2]);
        rec.putDouble(a[0]).putDouble(a[1]).putDouble(a[2]);
        toBridge.offer(rec.array());
        nImu++;
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    private void writer() {
        try {
            OutputStream o = proc.getOutputStream();
            while (running) {
                byte[] r = toBridge.poll(100, java.util.concurrent.TimeUnit.MILLISECONDS);
                if (r == null) continue;
                o.write(r);
                if (toBridge.isEmpty()) o.flush();
            }
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
                o.put("dt_cam_imu", j.optDouble("dt_cam_imu", 0)).put("cam_q_drop", camDropped);
                out.send(o.toString());
                lastStatus = j.optString("status");
                if (++nPose % 15 == 0) {
                    status.show(String.format(Locale.US, "OpenVINS %s\ncam %d imu %d pose %d\nlatency %.0f ms\nPC %s",
                            lastStatus, nCam, nImu, nPose, s == null ? -1.0 : (tOut - s[0]) / 1e6, out.connected ? "connected" : "-"));
                }
            }
            if (running) status.show("bridge exited (see files/ov/bridge_stderr.log)");
        } catch (Exception e) {
            if (running) status.show("reader: " + e);
        }
    }
}
