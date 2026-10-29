package com.scan3d.arlat;

import org.json.JSONObject;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.util.ArrayDeque;
import java.util.Arrays;

/**
 * usbrelay 2026-10-29 (app 0.6.38): the ONE owner of the phone <-> FC USB CDC byte streams. Plain Java (no android.*)
 * so it runs in the PC unit test (scan3d/runs/usbrelay_20261029/test) with a fake endpoint; the Android endpoint is
 * ViopImuSender.UsbSink (implements Port). USB PATH IS UNTESTED ON HARDWARE (PC fake endpoint only).
 *
 * OUT (phone -> FC): every bulk transfer happens under one lock (wl), so only one write is in flight at a time.
 *   - IMU fast path: offerImu() (ViopImuSender sensor thread) writes the frame itself when no write is in flight and no
 *     IMU frame is queued (= the 0.6.37c latency: one transfer on the sensor thread). Otherwise it is queued and
 *     offerImu returns at once (never waits for a POSE transfer). Reason: on the PC test the queue hand-off alone added
 *     p95 0.6-1.4 ms vs the direct write (scan3d/runs/usbrelay_20261029 README).
 *   writer thread "viop_usb_tx":
 *   - queue HI = phone IMU frames that could not take the fast path
 *   - queue LO = every valid VIOP frame ov_bridge sends to the relay UDP socket (POSE 0x01, HEARTBEAT 0x02, MAP_ODOM 0x07,
 *     ...): ov_bridge runs with --viop-udp 127.0.0.1:<udpPort()>, one frame per datagram (mav_out.h send_raw).
 *   - one Port.write (bulk transfer) per batch: ALL queued HI frames first, then LO frames only if HI is empty, up to
 *     BATCH bytes. Frames are never split / interleaved; an IMU frame never waits behind a LO frame that was queued
 *     later. The FC parses a byte stream (link_task.c usb_rx_cb -> viop_stream_*), so concatenated frames are fine.
 *     Bytes are forwarded unchanged.
 *   - full queue: the OLDEST frame is dropped (counted). HI cap 64 (~0.13 s at 500 Hz), LO cap 32.
 * IN (FC -> phone), reader thread "viop_usb_rx": Port.read -> VIOP stream parser (viop_stream_next rules: hunt A5 5A,
 *   version 2, len <= 86, CRC-16/CCITT-FALSE; a bad frame drops one byte). VIOP_T_VIO_RESET_REQ (0x23) frames are sent
 *   unchanged to the address the last bridge datagram came from (ov_bridge's own socket; its rx thread checks
 *   confirm and does full_reset() = kReset). Every other type is only counted + logged (the FC sends only 0x23 over USB
 *   today; FC_STATUS / FC_EVENT go to the PC over ESP-NOW).
 * No port (no USB device, no permission): the relay socket still drains the bridge datagrams (counted lo_no_port,
 *   dropped); offerImu() returns false (ViopImuSender counts it as write_fail, as before).
 */
public class ViopRelayCore {
    /** USB endpoint pair. write: bytes written or < 0; read: bytes read, <= 0 on timeout / error. Each blocks <= timeoutMs. */
    public interface Port {
        int write(byte[] b, int n, int timeoutMs);
        int read(byte[] b, int timeoutMs);
    }

    static final int SYNC0 = 0xA5, SYNC1 = 0x5A, VERSION = 2, HDR = 8, MAX_PAYLOAD = 86, MAX_FRAME = HDR + MAX_PAYLOAD + 2;
    static final int T_POSE = 0x01, T_HB = 0x02, T_IMU = 0x06, T_VIO_RESET_REQ = 0x23;
    static final int HI_CAP = 64, LO_CAP = 32, BATCH = 256, WRITE_TIMEOUT_MS = 2, READ_TIMEOUT_MS = 100;
    static final int RING = 1 << 16;

    private final Port port;                       // null = no device
    private final Runnable threadPrio;             // Android: Process.setThreadPriority in the writer / reader thread (may be null)
    private final Object lk = new Object();                                    // queues
    private final java.util.concurrent.locks.ReentrantLock wl = new java.util.concurrent.locks.ReentrantLock();   // USB OUT writes
    boolean imuDirect = true;   // IMU fast path (see offerImu); false = always queued (PC test compares both)
    long imuDirectN;            // IMU frames written by the fast path
    private final ArrayDeque<byte[]> hi = new ArrayDeque<>(), lo = new ArrayDeque<>();
    private final ArrayDeque<Long> hiT = new ArrayDeque<>(), loT = new ArrayDeque<>();   // enqueue time (ns) per frame
    private volatile boolean running;
    private Thread tx, rx, udp;
    private DatagramSocket sock;
    private volatile SocketAddress bridgeAddr;

    // stats: each counter has one writer thread; read in stop() after the threads ended
    volatile long imuIn, imuDropFull, imuNoPort;
    long imuWritten, imuFail;
    long udpIn, udpBad, loNoPort;
    volatile long loDropFull;
    long loWritten, loFail, writes, writeFailCalls, maxWriteNs;
    final long[] udpByType = new long[256], inByType = new long[256];
    long inBytes, inBad, inDroppedBytes, resetFwd, resetNoBridge, readErr;
    final long[] imuLat = new long[RING], loLat = new long[RING];   // enqueue -> write returned (ns), newest RING
    long nImuLat, nLoLat;
    final StringBuilder inLog = new StringBuilder();   // first 200 IN frames: t_ns,type,seq,len
    int inLogN;
    volatile String error = "";

    ViopRelayCore(Port port, Runnable threadPrio) {
        this.port = port;
        this.threadPrio = threadPrio;
    }

    boolean hasPort() { return port != null; }

    /** start writer + reader (if a port); udpRelay = also open the bridge relay socket on 127.0.0.1 (see udpPort()). */
    void start(boolean udpRelay) throws Exception {
        running = true;
        if (udpRelay) {
            sock = new DatagramSocket(new InetSocketAddress(InetAddress.getByName("127.0.0.1"), 0));
            sock.setSoTimeout(200);
            udpPort();
            udp =new Thread(this::udpLoop, "viop_usb_udp");
            udp.start();
        }
        if (port != null) {
            tx = new Thread(this::txLoop, "viop_usb_tx");
            rx = new Thread(this::rxLoop, "viop_usb_rx");
            tx.start();
            rx.start();
        }
    }

    private int udpPortNum = -1;
    int udpPort() { if (udpPortNum < 0 && sock != null) udpPortNum = sock.getLocalPort(); return udpPortNum; }

    /** sensor thread: send one IMU frame. Fast path (imuDirect, default): if no write is in flight and no IMU frame is
     *  queued, write it right here (= the 0.6.37c path: one transfer, <= 2 ms timeout); else queue it (copied) and return
     *  at once, the writer thread sends it before any LO frame. false = no port / stopped / direct write failed. */
    boolean offerImu(byte[] f, int n) {
        imuIn++;
        if (port == null || !running) { imuNoPort++; return false; }
        long t = System.nanoTime();
        if (imuDirect && wl.tryLock()) {
            try {
                boolean empty;
                synchronized (lk) { empty = hi.isEmpty(); }
                if (empty) {   // order kept: the writer dequeues only while holding wl, and no older IMU frame is waiting
                    imuDirectN++;
                    return doWrite(f, n, null, 1, null, 0, t);
                }
            } finally { wl.unlock(); }
        }
        byte[] c = Arrays.copyOf(f, n);
        synchronized (lk) {
            if (hi.size() >= HI_CAP) { hi.poll(); hiT.poll(); imuDropFull++; }
            hi.add(c);
            hiT.add(t);
            lk.notify();
        }
        return true;
    }

    private void offerLo(byte[] c) {
        if (port == null) { loNoPort++; return; }
        Long t = System.nanoTime();
        synchronized (lk) {
            if (lo.size() >= LO_CAP) { lo.poll(); loT.poll(); loDropFull++; }
            lo.add(c);
            loT.add(t);
            lk.notify();
        }
    }

    private void udpLoop() {
        byte[] b = new byte[512];
        DatagramPacket p = new DatagramPacket(b, b.length);
        while (running) {
            try {
                p.setLength(b.length);
                sock.receive(p);
            } catch (SocketTimeoutException e) { continue; }
            catch (Exception e) { if (running && error.isEmpty()) error = "udp: " + e; break; }
            int n = p.getLength();
            udpIn++;
            if (check(b, 0, n) != 0) { udpBad++; continue; }
            bridgeAddr = p.getSocketAddress();
            udpByType[b[3] & 0xff]++;
            offerLo(Arrays.copyOf(b, n));
        }
    }

    private void txLoop() {
        if (threadPrio != null) try { threadPrio.run(); } catch (Exception ignored) { }
        byte[] batch = new byte[BATCH];
        long[] tImu = new long[BATCH / HDR], tLo = new long[BATCH / HDR];
        while (true) {
            int n = 0, ni = 0, nl = 0;
            synchronized (lk) {
                while (running && hi.isEmpty() && lo.isEmpty()) { try { lk.wait(); } catch (InterruptedException e) { return; } }
                if (hi.isEmpty() && lo.isEmpty()) return;   // stopped and drained
            }
            wl.lock();   // dequeue + write under wl: the sensor-thread fast path cannot overtake a dequeued IMU frame
            try {
                synchronized (lk) {
                    while (!hi.isEmpty() && n + hi.peek().length <= BATCH) {
                        byte[] f = hi.poll();
                        System.arraycopy(f, 0, batch, n, f.length);
                        n += f.length;
                        tImu[ni++] = hiT.poll();
                    }
                    while (hi.isEmpty() && !lo.isEmpty() && n + lo.peek().length <= BATCH) {   // LO only after all HI
                        byte[] f = lo.poll();
                        System.arraycopy(f, 0, batch, n, f.length);
                        n += f.length;
                        tLo[nl++] = loT.poll();
                    }
                }
                if (n > 0) doWrite(batch, n, tImu, ni, tLo, nl, 0);
            } finally { wl.unlock(); }
        }
    }

    /** one bulk transfer + stats; caller holds wl (so the stats have one writer at a time). tImu == null: one direct IMU
     *  frame offered at t0Direct. */
    private boolean doWrite(byte[] b, int n, long[] tImu, int ni, long[] tLo, int nl, long t0Direct) {
        long t0 = System.nanoTime();
        int r;
        try { r = port.write(b, n, WRITE_TIMEOUT_MS); } catch (Exception e) { r = -1; if (error.isEmpty()) error = "write: " + e; }
        long t1 = System.nanoTime();
        for (int i = 0; i < nl; i++) loLat[(int) (nLoLat++ & (RING - 1))] = t1 - tLo[i];
        writes++;
        maxWriteNs = Math.max(maxWriteNs, t1 - t0);
        boolean ok = r == n;
        if (!ok) writeFailCalls++;
        if (ok) { imuWritten += ni; loWritten += nl; } else { imuFail += ni; loFail += nl; }
        if (tImu == null) imuLat[(int) (nImuLat++ & (RING - 1))] = t1 - t0Direct;
        else for (int i = 0; i < ni; i++) imuLat[(int) (nImuLat++ & (RING - 1))] = t1 - tImu[i];
        return ok;
    }

    // ---- IN: stream parser (viop_stream_next rules) ----
    private final byte[] sbuf = new byte[4 * MAX_FRAME];
    private int sn;

    private void rxLoop() {
        if (threadPrio != null) try { threadPrio.run(); } catch (Exception ignored) { }
        byte[] b = new byte[512];
        while (running) {
            long t0 = System.nanoTime();
            int r;
            try { r = port.read(b, READ_TIMEOUT_MS); } catch (Exception e) { r = -1; readErr++; if (error.isEmpty()) error = "read: " + e; }
            if (r <= 0) {   // timeout, or an immediate error (detached device): never spin
                if (System.nanoTime() - t0 < 20_000_000L) try { Thread.sleep(20); } catch (InterruptedException e) { return; }
                continue;
            }
            inBytes += r;
            push(b, r);
        }
    }

    /** feed received bytes (reader thread; package-private for the test). */
    void push(byte[] d, int len) {
        int k = 0;
        while (k < len) {
            int room = sbuf.length - sn;
            if (room == 0) { inDroppedBytes++; System.arraycopy(sbuf, 1, sbuf, 0, sn - 1); sn--; room = 1; }
            int c = Math.min(len - k, room);
            System.arraycopy(d, k, sbuf, sn, c);
            sn += c;
            k += c;
            int fs;
            while ((fs = next()) > 0) {
                onInFrame(sbuf, fs);
                System.arraycopy(sbuf, fs, sbuf, 0, sn - fs);
                sn -= fs;
            }
        }
    }

    private int next() {
        for (;;) {
            int i = 0;
            while (i + 1 < sn && !((sbuf[i] & 0xff) == SYNC0 && (sbuf[i + 1] & 0xff) == SYNC1)) i++;
            if (i > 0) { inDroppedBytes += i; System.arraycopy(sbuf, i, sbuf, 0, sn - i); sn -= i; }
            if (sn < HDR + 2) return 0;
            int len = sbuf[4] & 0xff;
            if ((sbuf[2] & 0xff) != VERSION || len > MAX_PAYLOAD) { inBad++; inDroppedBytes++; System.arraycopy(sbuf, 1, sbuf, 0, sn - 1); sn--; continue; }
            int fs = HDR + len + 2;
            if (sn < fs) return 0;
            if (check(sbuf, 0, fs) == 0) return fs;
            inBad++; inDroppedBytes++; System.arraycopy(sbuf, 1, sbuf, 0, sn - 1); sn--;
        }
    }

    private void onInFrame(byte[] f, int n) {
        int type = f[3] & 0xff;
        inByType[type]++;
        if (inLogN < 200) {
            inLogN++;
            inLog.append(System.nanoTime()).append(',').append(type).append(',').append((f[6] & 0xff) | ((f[7] & 0xff) << 8))
                    .append(',').append(f[4] & 0xff).append('\n');
        }
        if (type == T_VIO_RESET_REQ) {
            SocketAddress a = bridgeAddr;
            if (a == null || sock == null) { resetNoBridge++; return; }
            try { sock.send(new DatagramPacket(Arrays.copyOf(f, n), n, a)); resetFwd++; }
            catch (Exception e) { resetNoBridge++; if (error.isEmpty()) error = "reset fwd: " + e; }
        }
    }

    /** 0 ok, < 0 bad (viop_check in viop_proto.h). */
    static int check(byte[] f, int off, int n) {
        if (n < HDR + 2 || (f[off] & 0xff) != SYNC0 || (f[off + 1] & 0xff) != SYNC1) return -1;
        int len = f[off + 4] & 0xff;
        if ((f[off + 2] & 0xff) != VERSION || len > MAX_PAYLOAD || n != HDR + len + 2) return -2;
        int c = (f[off + HDR + len] & 0xff) | ((f[off + HDR + len + 1] & 0xff) << 8);
        return crc16(f, off + 2, HDR - 2 + len) == c ? 0 : -3;
    }

    static int crc16(byte[] d, int off, int n) {   // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
        int crc = 0xFFFF;
        for (int i = 0; i < n; i++) {
            crc ^= (d[off + i] & 0xff) << 8;
            for (int b = 0; b < 8; b++) crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
        }
        return crc;
    }

    /** one VIOP frame (= viop_build in viop_proto.h). */
    static byte[] build(int type, int src, int seq, byte[] payload) {
        byte[] f = new byte[HDR + payload.length + 2];
        f[0] = (byte) SYNC0; f[1] = (byte) SYNC1; f[2] = VERSION; f[3] = (byte) type; f[4] = (byte) payload.length; f[5] = (byte) src;
        f[6] = (byte) seq; f[7] = (byte) (seq >> 8);
        System.arraycopy(payload, 0, f, HDR, payload.length);
        int c = crc16(f, 2, HDR - 2 + payload.length);
        f[HDR + payload.length] = (byte) c; f[HDR + payload.length + 1] = (byte) (c >> 8);
        return f;
    }

    /** stop (the writer first drains what is queued, each write bounded by the 2 ms timeout); returns the summary. */
    JSONObject stop() {
        running = false;
        synchronized (lk) { lk.notifyAll(); }
        try { if (tx != null) tx.join(1000); } catch (InterruptedException ignored) { }
        try { if (rx != null) rx.join(READ_TIMEOUT_MS + 500); } catch (InterruptedException ignored) { }
        if (sock != null) sock.close();
        try { if (udp != null) udp.join(500); } catch (InterruptedException ignored) { }
        JSONObject j = new JSONObject();
        try {
            j.put("port", port != null ? 1 : 0).put("udp_port", udpPort())
                    .put("imu_in", imuIn).put("imu_written", imuWritten).put("imu_fail", imuFail).put("imu_drop_full", imuDropFull)
                    .put("imu_no_port", imuNoPort).put("imu_direct", imuDirectN).put("imu_direct_on", imuDirect ? 1 : 0)
                    .put("udp_in", udpIn).put("udp_bad", udpBad).put("udp_pose", udpByType[T_POSE]).put("udp_hb", udpByType[T_HB])
                    .put("udp_other", udpIn - udpBad - udpByType[T_POSE] - udpByType[T_HB])
                    .put("lo_written", loWritten).put("lo_fail", loFail).put("lo_drop_full", loDropFull).put("lo_no_port", loNoPort)
                    .put("writes", writes).put("write_fail_calls", writeFailCalls).put("max_write_ms", maxWriteNs / 1e6)
                    .put("imu_q_ms_p50", pct(imuLat, nImuLat, 0.5)).put("imu_q_ms_p95", pct(imuLat, nImuLat, 0.95))
                    .put("imu_q_ms_p99", pct(imuLat, nImuLat, 0.99)).put("imu_q_ms_max", pct(imuLat, nImuLat, 1.0))
                    .put("lo_q_ms_p50", pct(loLat, nLoLat, 0.5)).put("lo_q_ms_p95", pct(loLat, nLoLat, 0.95))
                    .put("lo_q_ms_max", pct(loLat, nLoLat, 1.0))
                    .put("in_bytes", inBytes).put("in_bad", inBad).put("in_dropped_bytes", inDroppedBytes)
                    .put("in_reset_req", inByType[T_VIO_RESET_REQ]).put("reset_fwd", resetFwd).put("reset_no_bridge", resetNoBridge)
                    .put("in_other", sum(inByType) - inByType[T_VIO_RESET_REQ]).put("read_err", readErr)
                    .put("in_log", inLog.toString()).put("error", error);
        } catch (Exception ignored) { }
        return j;
    }

    static long sum(long[] a) { long s = 0; for (long v : a) s += v; return s; }

    /** percentile of the newest min(n, RING) latencies, ms (-1 if none). */
    static double pct(long[] ring, long n, double q) {
        int m = (int) Math.min(n, ring.length);
        if (m == 0) return -1;
        long[] s = Arrays.copyOf(ring, m);
        Arrays.sort(s);
        return s[Math.min(m - 1, (int) (q * m))] / 1e6;
    }
}
