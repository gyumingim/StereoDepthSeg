package com.scan3d.arlat;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;

import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.Arrays;

/**
 * VIOP_T_IMU (0x06) phone IMU sender for the ESP32 flight controller (integ-20261025, app 0.6.20; default OFF).
 * Spec: scan3d/runs/ovdrone_20261025/README.md "Phone IMU sender" + scan3d/ov_bridge/viop_proto.h (VIOP_T_IMU).
 * Started only by intent viop_imu ("udp:HOST:PORT" or "usb"): with mode ov next to the VIO run, or alone with mode imu_tx
 * (no camera). Own SensorEventListener on its own HandlerThread (never the VIO feeder thread), ACCELEROMETER + GYROSCOPE
 * (or *_UNCALIBRATED with intent viop_imu_uncal 1 -> flag VIOP_IMUF_UNCAL), SENSOR_DELAY_FASTEST, maxReportLatencyUs 0.
 * On every gyro event: paired with the newest accel sample if |dt| <= 5 ms, else held and sent at the next accel event
 * (then dropped if still > 5 ms apart: counted). One frame per sample (n = 1, 36 bytes): u64 t0_us = gyro
 * event.timestamp / 1000 (CLOCK_BOOTTIME, same timebase as POSE), u8 n 1, u8 flags, u16 sample_seq0, u16 dt_us 0,
 * i16 gyro[3] (1e-3 rad/s), i16 acc[3] (5e-3 m/s^2), saturated; Android sensor axes unchanged; frame seq +1 per frame.
 * Sinks: UDP (one frame per datagram, bench / PC loopback test) or USB CDC (bulk OUT of the first CDC data interface of
 * an attached device, permission asked once; bulkTransfer timeout 2 ms, failures counted, never blocks longer).
 * USB PATH IS UNTESTED (no ESP32 attached during integration). Log: <dir>/viop_imu.json (counts, write time, sample ->
 * write-done latency p50/p95/max, sent-sample gaps) + viop_imu.csv (t_sample_ns, t_written_ns, ok) ring 2^19.
 * usbrelay 10-29 (0.6.38): target usb goes through a ViopRelayCore (UsbLink), the single owner of the USB OUT writes,
 * shared with the ov_bridge POSE / HEARTBEAT relay (OvRunner viop_udp usb). Fast path: if no transfer is in flight the
 * frame is written right here as before; else it is queued (IMU before any POSE) and send() returns at once. So
 * "t_written" / lat_* = written (fast path) or QUEUED (ok = queued); the link summary (usb_link / viop_usb.json) has the
 * per-frame queued -> written latency (imu_q_ms_*) and imu_direct. UDP target unchanged.
 */
public class ViopImuSender implements SensorEventListener {
    static final int T_IMU = 0x06, SRC_PHONE = 1, VERSION = 2, IMUF_UNCAL = 0x02;
    static final double GYRO_LSB = 1e-3, ACC_LSB = 5e-3;
    static final long PAIR_NS = 5_000_000L;

    final String target;
    final boolean uncal;
    final File dir;
    private final Context ctx;
    private HandlerThread th;
    private SensorManager sm;
    private DatagramSocket sock;
    private DatagramPacket pkt;
    private UsbLink link;            // usbrelay: target usb -> shared link (OvRunner) or own link (mode imu_tx)
    private boolean ownLink;
    private final byte[] frame = new byte[36];
    private int frameSeq, sampleSeq;
    // newest accel + held gyro (sensor thread only)
    private long accT = Long.MIN_VALUE;
    private final float[] acc = new float[3];
    private long heldGyroT = Long.MIN_VALUE;
    private final float[] heldGyro = new float[3];
    // stats (sensor thread; read at stop() after the thread ended)
    long nGyro, nAcc, nSent, nFail, nDropPair, maxWriteNs;
    static final int RING = 1 << 19;
    private final long[] ring = new long[3 * RING];
    private long nRing;
    String error = "";

    ViopImuSender(Context ctx, String target, boolean uncal, File dir) { this(ctx, target, uncal, dir, null); }

    /** shared = the USB link owned by the caller (OvRunner with viop_udp usb); null = open an own link for target usb. */
    ViopImuSender(Context ctx, String target, boolean uncal, File dir, UsbLink shared) {
        this.ctx = ctx;
        this.target = target;
        this.uncal = uncal;
        this.dir = dir;
        this.link = shared;
    }

    /** throws on a bad target; the USB permission dialog may appear (the sink then stays closed until granted). */
    void start() throws Exception {
        if (target.startsWith("udp:")) {
            String hp = target.substring(4);
            int c = hp.lastIndexOf(':');
            if (c <= 0) throw new IllegalArgumentException("viop_imu udp:HOST:PORT");
            InetAddress a = InetAddress.getByName(hp.substring(0, c));   // numeric address expected (no DNS on the hot path)
            int port = Integer.parseInt(hp.substring(c + 1));
            sock = new DatagramSocket();
            pkt = new DatagramPacket(frame, frame.length, a, port);
        } else if (target.equals("usb")) {
            if (link == null) { link = UsbLink.open(ctx, false); ownLink = true; }
        } else throw new IllegalArgumentException("viop_imu must be udp:HOST:PORT or usb, got " + target);
        th = new HandlerThread("viop_imu", android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY);
        th.start();
        sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        Sensor g = sm.getDefaultSensor(uncal ? Sensor.TYPE_GYROSCOPE_UNCALIBRATED : Sensor.TYPE_GYROSCOPE);
        Sensor ac = sm.getDefaultSensor(uncal ? Sensor.TYPE_ACCELEROMETER_UNCALIBRATED : Sensor.TYPE_ACCELEROMETER);
        if (g == null || ac == null) throw new IllegalStateException("no gyro / accel sensor");
        Handler h = new Handler(th.getLooper());
        sm.registerListener(this, ac, SensorManager.SENSOR_DELAY_FASTEST, 0, h);
        sm.registerListener(this, g, SensorManager.SENSOR_DELAY_FASTEST, 0, h);
    }

    @Override
    public void onSensorChanged(SensorEvent e) {
        int t = e.sensor.getType();
        if (t == Sensor.TYPE_ACCELEROMETER || t == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) {
            nAcc++;
            accT = e.timestamp;
            acc[0] = e.values[0]; acc[1] = e.values[1]; acc[2] = e.values[2];
            if (heldGyroT != Long.MIN_VALUE) {
                if (Math.abs(heldGyroT - accT) <= PAIR_NS) send(heldGyroT, heldGyro, acc);
                else nDropPair++;
                heldGyroT = Long.MIN_VALUE;
            }
        } else {
            nGyro++;
            float[] w = {e.values[0], e.values[1], e.values[2]};   // uncalibrated: values[0..2] = raw rate (bias not removed)
            if (heldGyroT != Long.MIN_VALUE) nDropPair++;            // an older held gyro never got an accel partner
            heldGyroT = Long.MIN_VALUE;
            if (accT != Long.MIN_VALUE && Math.abs(e.timestamp - accT) <= PAIR_NS) send(e.timestamp, w, acc);
            else { heldGyroT = e.timestamp; System.arraycopy(w, 0, heldGyro, 0, 3); }
        }
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    private void send(long tNs, float[] w, float[] a) {
        build(frame, frameSeq, tNs / 1000L, uncal ? IMUF_UNCAL : 0, sampleSeq, w, a);
        frameSeq = (frameSeq + 1) & 0xffff;
        sampleSeq = (sampleSeq + 1) & 0xffff;
        long t0 = SystemClock.elapsedRealtimeNanos();
        boolean ok;
        try {
            if (sock != null) { sock.send(pkt); ok = true; }
            else ok = link.core.offerImu(frame, frame.length);   // usbrelay: direct if the port is free, else queued
        } catch (Exception ex) { ok = false; if (error.isEmpty()) error = ex.toString(); }
        long t1 = SystemClock.elapsedRealtimeNanos();
        if (ok) nSent++; else nFail++;
        maxWriteNs = Math.max(maxWriteNs, t1 - t0);
        int k = (int) (nRing++ & (RING - 1)) * 3;
        ring[k] = tNs; ring[k + 1] = t1; ring[k + 2] = ok ? 1 : 0;
    }

    static short sat16(double v) { return (short) Math.max(-32768, Math.min(32767, Math.round(v))); }

    /** one VIOP_T_IMU frame (n = 1, 36 bytes) into f; same bytes as viop_build() in viop_proto.h. */
    static void build(byte[] f, int seq, long t0Us, int flags, int sampleSeq0, float[] gyro, float[] acc) {
        f[0] = (byte) 0xA5; f[1] = (byte) 0x5A; f[2] = VERSION; f[3] = T_IMU; f[4] = 26; f[5] = SRC_PHONE;
        f[6] = (byte) seq; f[7] = (byte) (seq >> 8);
        for (int i = 0; i < 8; i++) f[8 + i] = (byte) (t0Us >>> (8 * i));
        f[16] = 1; f[17] = (byte) flags; f[18] = (byte) sampleSeq0; f[19] = (byte) (sampleSeq0 >> 8);
        f[20] = 0; f[21] = 0;   // dt_us of sample 0
        for (int i = 0; i < 3; i++) {
            short g = sat16(gyro[i] / GYRO_LSB), a = sat16(acc[i] / ACC_LSB);
            f[22 + 2 * i] = (byte) g; f[23 + 2 * i] = (byte) (g >> 8);
            f[28 + 2 * i] = (byte) a; f[29 + 2 * i] = (byte) (a >> 8);
        }
        int c = crc16(f, 2, 8 - 2 + 26);
        f[34] = (byte) c; f[35] = (byte) (c >> 8);
    }

    static int crc16(byte[] d, int off, int n) {   // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
        int crc = 0xFFFF;
        for (int i = 0; i < n; i++) {
            crc ^= (d[off + i] & 0xff) << 8;
            for (int b = 0; b < 8; b++) crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
        }
        return crc;
    }

    /** stop, write viop_imu.json / viop_imu.csv into dir; returns the summary. */
    JSONObject stop() {
        if (sm != null) sm.unregisterListener(this);
        if (th != null) { th.quitSafely(); try { th.join(500); } catch (InterruptedException ignored) { } }
        if (sock != null) sock.close();
        JSONObject linkRes = null;
        if (link != null && ownLink) linkRes = link.stop();
        JSONObject j = new JSONObject();
        try {
            int n = (int) Math.min(nRing, RING);
            long first = nRing > RING ? nRing - RING : 0;
            long[] lat = new long[n], gap = new long[Math.max(0, n - 1)];
            long prevT = 0;
            BufferedWriter w = new BufferedWriter(new FileWriter(new File(dir, "viop_imu.csv")), 1 << 16);
            w.write("t_sample_ns,t_written_ns,ok\n");
            for (int i = 0; i < n; i++) {
                int k = (int) ((first + i) & (RING - 1)) * 3;
                lat[i] = ring[k + 1] - ring[k];
                if (i > 0) gap[i - 1] = ring[k] - prevT;
                prevT = ring[k];
                w.write(ring[k] + "," + ring[k + 1] + "," + ring[k + 2] + "\n");
            }
            w.close();
            Arrays.sort(lat);
            Arrays.sort(gap);
            long g10 = 0;
            for (long g : gap) if (g > 10_000_000L) g10++;
            j.put("target", target).put("uncal", uncal ? 1 : 0).put("gyro_events", nGyro).put("acc_events", nAcc)
                    .put("frames_sent", nSent).put("write_fail", nFail).put("pair_dropped", nDropPair)
                    .put("max_write_ms", maxWriteNs / 1e6).put("lat_ms_p50", q(lat, 0.5) / 1e6).put("lat_ms_p95", q(lat, 0.95) / 1e6)
                    .put("lat_ms_max", n > 0 ? lat[n - 1] / 1e6 : -1).put("gap_ms_p50", q(gap, 0.5) / 1e6)
                    .put("gap_ms_p95", q(gap, 0.95) / 1e6).put("gap_ms_max", gap.length > 0 ? gap[gap.length - 1] / 1e6 : -1)
                    .put("gaps_over_10ms", g10).put("error", error).put("usb", link != null ? link.usb.info : "")
                    .put("usb_queued", target.equals("usb") ? 1 : 0);
            if (linkRes != null) j.put("usb_link", linkRes);
            FileWriter fw = new FileWriter(new File(dir, "viop_imu.json"));
            fw.write(j.toString(1));
            fw.close();
        } catch (Exception ex) { android.util.Log.e("arlat", "viop_imu log", ex); }
        return j;
    }

    static double q(long[] s, double f) { return s.length == 0 ? -1 : s[Math.min(s.length - 1, (int) (f * s.length))]; }

    /** usbrelay 10-29: the USB endpoint pair + its ViopRelayCore (the single owner of the USB OUT writes and IN reads). */
    static class UsbLink {
        final UsbSink usb;
        final ViopRelayCore core;
        private UsbLink(UsbSink u, ViopRelayCore c) { usb = u; core = c; }
        /** opens the first CDC device (asks permission if needed: then no port this run, as before) and starts the link.
         *  udpRelay = also open the ov_bridge relay socket (127.0.0.1:core.udpPort()). Never throws for "no device". */
        static UsbLink open(Context ctx, boolean udpRelay) throws Exception {
            UsbSink u = new UsbSink(ctx);
            u.open();
            ViopRelayCore c = new ViopRelayCore(u.conn != null ? u : null,
                    () -> android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY));
            c.start(udpRelay);
            return new UsbLink(u, c);
        }
        JSONObject stop() {
            JSONObject j = core.stop();
            usb.close();
            try { j.put("usb", usb.info); } catch (Exception ignored) { }
            return j;
        }
    }

    /** USB CDC endpoint pair (UNTESTED on hardware). First attached device with a CDC data interface (class 0x0A) and a
     *  bulk OUT endpoint (+ its bulk IN endpoint, usbrelay); DTR set on the control interface (ESP32 TinyUSB CDC needs it to
     *  accept data); on close DTR is cleared first (the FC link_task.c reads DTR 1 -> 0 as "phone app closed the port"). */
    static class UsbSink implements ViopRelayCore.Port {
        final Context ctx;
        android.hardware.usb.UsbDeviceConnection conn;
        android.hardware.usb.UsbEndpoint out, in;
        android.hardware.usb.UsbInterface dataIf;
        int ctrlIf;
        String info = "";
        UsbSink(Context c) { ctx = c; }
        void open() {
            android.hardware.usb.UsbManager um = (android.hardware.usb.UsbManager) ctx.getSystemService(Context.USB_SERVICE);
            for (android.hardware.usb.UsbDevice d : um.getDeviceList().values()) {
                for (int i = 0; i < d.getInterfaceCount(); i++) {
                    android.hardware.usb.UsbInterface itf = d.getInterface(i);
                    if (itf.getInterfaceClass() != 0x0A) continue;
                    android.hardware.usb.UsbEndpoint o = null, n = null;
                    for (int e = 0; e < itf.getEndpointCount(); e++) {
                        android.hardware.usb.UsbEndpoint ep = itf.getEndpoint(e);
                        if (ep.getType() != android.hardware.usb.UsbConstants.USB_ENDPOINT_XFER_BULK) continue;
                        if (ep.getDirection() == android.hardware.usb.UsbConstants.USB_DIR_OUT) { if (o == null) o = ep; }
                        else if (n == null) n = ep;
                    }
                    if (o == null) continue;
                    info = String.format(java.util.Locale.US, "%04x:%04x %s%s", d.getVendorId(), d.getProductId(), d.getDeviceName(),
                            n == null ? " (no bulk IN)" : "");
                    if (!um.hasPermission(d)) {
                        info += " (no permission: asked, start again after allowing)";
                        um.requestPermission(d, android.app.PendingIntent.getBroadcast(ctx, 0,
                                new android.content.Intent("com.scan3d.arlat.USB_PERMISSION"),
                                android.app.PendingIntent.FLAG_IMMUTABLE));
                        return;
                    }
                    conn = um.openDevice(d);
                    if (conn == null) { info += " (open failed)"; return; }
                    conn.claimInterface(itf, true);
                    dataIf = itf;
                    out = o;
                    in = n;
                    // SET_CONTROL_LINE_STATE DTR|RTS on interface i - 1 (the CDC control interface precedes data)
                    ctrlIf = Math.max(0, i - 1);
                    conn.controlTransfer(0x21, 0x22, 0x03, ctrlIf, null, 0, 100);
                    return;
                }
            }
            info = "no CDC device";
        }
        @Override public int write(byte[] b, int n, int timeoutMs) {
            android.hardware.usb.UsbDeviceConnection c = conn;
            return c == null ? -1 : c.bulkTransfer(out, b, n, timeoutMs);
        }
        @Override public int read(byte[] b, int timeoutMs) {
            android.hardware.usb.UsbDeviceConnection c = conn;
            if (c == null || in == null) { try { Thread.sleep(timeoutMs); } catch (InterruptedException ignored) { } return -1; }
            return c.bulkTransfer(in, b, b.length, timeoutMs);
        }
        void close() {
            if (conn == null) return;
            try { conn.controlTransfer(0x21, 0x22, 0x00, ctrlIf, null, 0, 100); } catch (Exception ignored) { }   // DTR 0
            conn.releaseInterface(dataIf);
            conn.close();
            conn = null;
        }
    }
}
