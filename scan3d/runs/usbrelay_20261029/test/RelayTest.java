// usbrelay 2026-10-29: PC test of ViopRelayCore with a fake USB endpoint (no Android, no device). Run: run_test.sh
// Writes into argv[0]: usb_out_*.bin (raw OUT byte stream, checked again by stream_check.c with the FC's viop_stream
// parser), reset_fwd.hex (0x23 frames the fake bridge socket received), results.txt (all numbers).
package com.scan3d.arlat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.LockSupport;

public class RelayTest {
    static int fails, checks;
    static PrintWriter out;

    static void check(boolean c, String what) {
        checks++;
        if (!c) { fails++; out.println("FAIL " + what); System.out.println("FAIL " + what); }
    }

    static void spin(long ns) { long e = System.nanoTime() + ns; while (System.nanoTime() < e) { } }

    /** fake bulk endpoints: write = busy delay (fixed + per 64-byte packet), record bytes; read = scripted chunks. */
    static class FakePort implements ViopRelayCore.Port {
        final long fixedNs, perPktNs;
        final boolean failAll;
        final java.io.ByteArrayOutputStream sink = new java.io.ByteArrayOutputStream();
        final ConcurrentLinkedQueue<byte[]> rxq = new ConcurrentLinkedQueue<>();
        volatile int concurrentWriters, maxConcurrent;
        FakePort(long fixedNs, long perPktNs, boolean failAll) { this.fixedNs = fixedNs; this.perPktNs = perPktNs; this.failAll = failAll; }
        @Override public int write(byte[] b, int n, int timeoutMs) {
            synchronized (this) { concurrentWriters++; maxConcurrent = Math.max(maxConcurrent, concurrentWriters); }
            try {
                if (failAll) { spin(timeoutMs * 1_000_000L); return -1; }
                spin(fixedNs + perPktNs * ((n + 63) / 64));
                synchronized (sink) { sink.write(b, 0, n); }
                return n;
            } finally { synchronized (this) { concurrentWriters--; } }
        }
        @Override public int read(byte[] b, int timeoutMs) {
            byte[] c = rxq.poll();
            if (c == null) { LockSupport.parkNanos(2_000_000L); return 0; }
            System.arraycopy(c, 0, b, 0, c.length);
            return c.length;
        }
    }

    static byte[] posePayload(int i) { byte[] p = new byte[82]; for (int k = 0; k < 82; k++) p[k] = (byte) (i * 31 + k); return p; }
    static byte[] hbPayload(int i) { byte[] p = new byte[18]; for (int k = 0; k < 18; k++) p[k] = (byte) (i + k); return p; }

    static byte[] imuFrame(int seq) {
        byte[] f = new byte[36];
        ViopImuSender.build(f, seq, 1_000_000_000L + seq * 2000L, 0, seq, new float[]{0.01f * (seq % 7), 0.02f, -0.03f}, new float[]{0.1f, 0.2f, 9.81f});
        return f;
    }

    /** parse a byte stream, return frames (viop_stream rules); counts bad. */
    static List<byte[]> parse(byte[] s, int[] bad) {
        List<byte[]> r = new ArrayList<>();
        int i = 0;
        while (i + ViopRelayCore.HDR + 2 <= s.length) {
            if ((s[i] & 0xff) != 0xA5 || (s[i + 1] & 0xff) != 0x5A) { bad[0]++; i++; continue; }
            int fs = ViopRelayCore.HDR + (s[i + 4] & 0xff) + 2;
            if (i + fs > s.length || ViopRelayCore.check(s, i, fs) != 0) { bad[0]++; i++; continue; }
            r.add(Arrays.copyOfRange(s, i, i + fs));
            i += fs;
        }
        if (i != s.length) bad[0] += s.length - i;
        return r;
    }

    /** one traffic run: IMU 500 Hz from a "sensor" thread, optional bridge POSE 200 Hz + HB 20 Hz over UDP, secs. */
    static boolean directMode = true;
    static double[] lastOffer = new double[3];   // offerImu time p50 / p99 / max, ms

    static double q(long[] v, int n, double f) { long[] s = Arrays.copyOf(v, n); Arrays.sort(s); return n == 0 ? -1 : s[Math.min(n - 1, (int) (f * n))] / 1e6; }

    /** old path (0.6.37c UsbSink.write on the sensor thread): latency = time the sensor thread is blocked in the write. */
    static double[] direct(FakePort port, double secs) {
        long t0 = System.nanoTime(), dur = (long) (secs * 1e9), next = t0;
        long[] lat = new long[(int) (secs * 600) + 10];
        int n = 0;
        while (System.nanoTime() - t0 < dur && n < lat.length) {
            next += 2_000_000L;
            long w = next - System.nanoTime();
            if (w > 0) LockSupport.parkNanos(w);
            byte[] f = imuFrame(n & 0xffff);
            long a = System.nanoTime();
            port.write(f, f.length, 2);
            lat[n++] = System.nanoTime() - a;
        }
        return new double[]{q(lat, n, 0.5), q(lat, n, 0.95), q(lat, n, 0.99), q(lat, n, 1.0)};
    }

    static org.json.JSONObject traffic(String name, File dir, FakePort port, boolean bridge, double secs, long[] offerMaxNs,
                                       int[] sentCounts) throws Exception {
        ViopRelayCore core = new ViopRelayCore(port, null);
        core.imuDirect = directMode;
        core.start(true);
        int rport = core.udpPort();
        DatagramSocket br = new DatagramSocket(new InetSocketAddress("127.0.0.1", 0));
        Thread bt = null;
        final long t0 = System.nanoTime(), dur = (long) (secs * 1e9);
        final int[] nPose = {0}, nHb = {0};
        if (bridge) {
            bt = new Thread(() -> {
                try {
                    long next = t0;
                    int k = 0;
                    while (System.nanoTime() - t0 < dur) {
                        next += 5_000_000L;   // 200 Hz pose
                        long w = next - System.nanoTime();
                        if (w > 0) LockSupport.parkNanos(w);
                        byte[] f = ViopRelayCore.build(ViopRelayCore.T_POSE, 1, nPose[0]++, posePayload(k));
                        br.send(new DatagramPacket(f, f.length, new InetSocketAddress("127.0.0.1", rport)));
                        if (k++ % 10 == 0) {
                            byte[] h = ViopRelayCore.build(ViopRelayCore.T_HB, 1, nHb[0]++, hbPayload(k));
                            br.send(new DatagramPacket(h, h.length, new InetSocketAddress("127.0.0.1", rport)));
                        }
                    }
                } catch (Exception e) { e.printStackTrace(); }
            }, "fake_bridge");
            bt.start();
        }
        long next = t0, maxOffer = 0;
        long[] offT = new long[(int) (secs * 600) + 10];
        int nImu = 0;
        while (System.nanoTime() - t0 < dur) {   // the "sensor thread"
            next += 2_000_000L;   // 500 Hz
            long w = next - System.nanoTime();
            if (w > 0) LockSupport.parkNanos(w);
            byte[] f = imuFrame(nImu & 0xffff);
            long a = System.nanoTime();
            core.offerImu(f, f.length);
            long d = System.nanoTime() - a;
            maxOffer = Math.max(maxOffer, d);
            if (nImu < offT.length) offT[nImu] = d;
            nImu++;
        }
        if (bt != null) bt.join();
        Thread.sleep(50);   // let the last datagrams arrive
        org.json.JSONObject j = core.stop();
        br.close();
        offerMaxNs[0] = maxOffer;
        int no = Math.min(nImu, offT.length);
        lastOffer = new double[]{q(offT, no, 0.5), q(offT, no, 0.99), q(offT, no, 1.0)};
        sentCounts[0] = nImu; sentCounts[1] = nPose[0]; sentCounts[2] = nHb[0];
        byte[] s = port.sink.toByteArray();
        try (FileOutputStream fo = new FileOutputStream(new File(dir, "usb_out_" + name + ".bin"))) { fo.write(s); }
        out.println(name + ": " + j);
        out.println(name + ": imu sent " + nImu + " pose sent " + nPose[0] + " hb sent " + nHb[0] + " offer max us " + maxOffer / 1000
                + " out bytes " + s.length + " max concurrent writers " + port.maxConcurrent);
        return j;
    }

    static void checkStream(String name, byte[] s, int nImu, int nPose, int nHb) {
        int[] bad = {0};
        List<byte[]> fr = parse(s, bad);
        int ci = 0, cp = 0, ch = 0, lastImu = -1, lastPose = -1;
        boolean imuOrder = true, poseOrder = true, poseSame = true;
        for (byte[] f : fr) {
            int t = f[3] & 0xff, seq = (f[6] & 0xff) | ((f[7] & 0xff) << 8);
            if (t == ViopRelayCore.T_IMU) {
                if (seq != lastImu + 1) imuOrder = false;
                lastImu = seq;
                if (!Arrays.equals(f, imuFrame(seq))) imuOrder = false;
                ci++;
            } else if (t == ViopRelayCore.T_POSE) {
                if (seq != lastPose + 1) poseOrder = false;
                lastPose = seq;
                if (!Arrays.equals(f, ViopRelayCore.build(ViopRelayCore.T_POSE, 1, seq, posePayload(seq)))) poseSame = false;
                cp++;
            } else if (t == ViopRelayCore.T_HB) ch++;
        }
        check(bad[0] == 0, name + ": OUT stream has 0 bad bytes (got " + bad[0] + ")");
        check(ci == nImu && cp == nPose && ch == nHb, name + ": all frames out once (imu " + ci + "/" + nImu + " pose " + cp + "/" + nPose
                + " hb " + ch + "/" + nHb + ")");
        check(imuOrder, name + ": IMU frames in order, bytes unchanged");
        check(poseOrder && poseSame, name + ": POSE frames in order, bytes unchanged");
    }

    public static void main(String[] a) throws Exception {
        File dir = new File(a[0]);
        dir.mkdirs();
        out = new PrintWriter(new File(dir, "results.txt"));
        out.println("RelayTest " + new java.util.Date() + " java " + System.getProperty("java.version") + " cpus "
                + Runtime.getRuntime().availableProcessors());
        double secs = a.length > 1 ? Double.parseDouble(a[1]) : 10;
        long[] om = {0};
        int[] sc = new int[3];

        // 1-4: traffic with two fake USB speeds (ASSUMED transfer times, not measured on the S24: 0.125 ms fixed + 0.02 ms
        // per 64-byte packet ~ one full-speed microframe-ish; and a slow 1 ms fixed + 0.1 ms / packet)
        // name, fixed ns, per-packet ns, bridge traffic, IMU fast path (d = direct when free, q = always queued)
        String[][] cases = {{"fast_imu_d", "125000", "20000", "0", "d"}, {"fast_imu_q", "125000", "20000", "0", "q"},
                {"fast_both_d", "125000", "20000", "1", "d"}, {"fast_both_q", "125000", "20000", "1", "q"},
                {"slow_imu_d", "1000000", "100000", "0", "d"}, {"slow_imu_q", "1000000", "100000", "0", "q"},
                {"slow_both_d", "1000000", "100000", "1", "d"}, {"slow_both_q", "1000000", "100000", "1", "q"}};
        StringBuilder table = new StringBuilder("case | imu queued->written p50 / p95 / p99 / max ms | pose udp->written p50 / p95 / max ms | writes | offerImu p50 / p99 / max ms\n");
        for (String[] c : cases) {
            FakePort p = new FakePort(Long.parseLong(c[1]), Long.parseLong(c[2]), false);
            directMode = c[4].equals("d");
            org.json.JSONObject j = traffic(c[0], dir, p, c[3].equals("1"), secs, om, sc);
            checkStream(c[0], p.sink.toByteArray(), sc[0], sc[1], sc[2]);
            check(j.optLong("imu_drop_full") == 0 && j.optLong("lo_drop_full") == 0, c[0] + ": no queue drops");
            check(p.maxConcurrent == 1, c[0] + ": one writer at a time (max " + p.maxConcurrent + ")");
            // q mode (not the app default) is a comparison only: its offerImu times are reported, not checked
            if (directMode) check(j.optLong("imu_direct") > 0, c[0] + ": fast path used (" + j.optLong("imu_direct") + " of " + sc[0] + ")");
            table.append(String.format(java.util.Locale.US, "%s | %.3f / %.3f / %.3f / %.3f | %.3f / %.3f / %.3f | %d | %.3f / %.3f / %.3f\n", c[0],
                    j.optDouble("imu_q_ms_p50"), j.optDouble("imu_q_ms_p95"), j.optDouble("imu_q_ms_p99"), j.optDouble("imu_q_ms_max"),
                    j.optDouble("lo_q_ms_p50"), j.optDouble("lo_q_ms_p95"), j.optDouble("lo_q_ms_max"), j.optLong("writes"),
                    lastOffer[0], lastOffer[1], lastOffer[2]));
            if (c[3].equals("0") && directMode) {   // same fake speed, old direct path (0.6.37c), right after = same machine load
                double[] d = direct(new FakePort(Long.parseLong(c[1]), Long.parseLong(c[2]), false), secs);
                table.append(String.format(java.util.Locale.US, "old_direct_%s | %.3f / %.3f / %.3f / %.3f | - | - | sensor thread blocked = these\n",
                        c[0].substring(0, c[0].indexOf('_')), d[0], d[1], d[2], d[3]));
            }
        }
        out.println("LATENCY TABLE (enqueue -> fake write returned)\n" + table);
        System.out.print(table);

        // 5: USB IN -> 0x23 to the bridge socket; garbage, split frame, bad CRC, other type
        {
            FakePort p = new FakePort(100000, 0, false);
            ViopRelayCore core = new ViopRelayCore(p, null);
            core.start(true);
            DatagramSocket br = new DatagramSocket(new InetSocketAddress("127.0.0.1", 0));
            br.setSoTimeout(500);
            byte[] cmd = {6, 0, (byte) (0xC0DE ^ 6), (byte) ((0xC0DE ^ 6) >> 8)};
            byte[] r1 = ViopRelayCore.build(ViopRelayCore.T_VIO_RESET_REQ, 2, 0, cmd);
            // 0x23 before the bridge sent anything: no address -> counted, not sent
            p.rxq.add(r1.clone());
            Thread.sleep(100);
            check(core.resetNoBridge == 1 && core.resetFwd == 0, "IN: 0x23 before any bridge datagram -> reset_no_bridge");
            byte[] hb = ViopRelayCore.build(ViopRelayCore.T_HB, 1, 0, hbPayload(0));
            br.send(new DatagramPacket(hb, hb.length, new InetSocketAddress("127.0.0.1", core.udpPort())));
            Thread.sleep(50);
            byte[] r2 = ViopRelayCore.build(ViopRelayCore.T_VIO_RESET_REQ, 2, 1, cmd);
            byte[] badc = r2.clone(); badc[badc.length - 1] ^= 0x55;   // bad CRC
            byte[] st = ViopRelayCore.build(0x05, 2, 7, new byte[40]);   // FC_STATUS-like other type
            byte[] r3 = ViopRelayCore.build(ViopRelayCore.T_VIO_RESET_REQ, 2, 2, cmd);
            java.io.ByteArrayOutputStream bs = new java.io.ByteArrayOutputStream();
            bs.write(new byte[]{0x00, (byte) 0xA5, 0x11, (byte) 0xA5, 0x5A, 0x07});   // garbage incl. a false sync
            bs.write(badc); bs.write(r2); bs.write(st); bs.write(r3);
            byte[] all = bs.toByteArray();
            for (int i = 0; i < all.length; i += 5) p.rxq.add(Arrays.copyOfRange(all, i, Math.min(all.length, i + 5)));   // 5-byte chunks
            List<byte[]> got = new ArrayList<>();
            try (PrintWriter hx = new PrintWriter(new File(dir, "reset_fwd.hex"))) {
                for (int k = 0; k < 3; k++) {
                    byte[] b = new byte[200];
                    DatagramPacket dp = new DatagramPacket(b, b.length);
                    try { br.receive(dp); } catch (java.net.SocketTimeoutException e) { break; }
                    byte[] g = Arrays.copyOf(b, dp.getLength());
                    got.add(g);
                    StringBuilder sb = new StringBuilder();
                    for (byte x : g) sb.append(String.format("%02x", x & 0xff));
                    hx.println(sb);
                }
            }
            org.json.JSONObject j = core.stop();
            br.close();
            out.println("in_test: " + j);
            check(got.size() == 2 && Arrays.equals(got.get(0), r2) && Arrays.equals(got.get(1), r3),
                    "IN: exactly the 2 valid 0x23 frames reach the bridge socket unchanged (got " + got.size() + ")");
            check(j.optLong("in_reset_req") == 3 && j.optLong("reset_fwd") == 2 && j.optLong("in_other") == 1 && j.optLong("in_bad") >= 1,
                    "IN: counts reset_req 3 / fwd 2 / other 1 / bad >= 1");
        }

        // 6: no port (no USB device): bridge datagrams drained + counted, IMU refused, nothing blocks
        {
            ViopRelayCore core = new ViopRelayCore(null, null);
            core.start(true);
            DatagramSocket br = new DatagramSocket(new InetSocketAddress("127.0.0.1", 0));
            for (int i = 0; i < 100; i++) {
                byte[] f = ViopRelayCore.build(ViopRelayCore.T_POSE, 1, i, posePayload(i));
                br.send(new DatagramPacket(f, f.length, new InetSocketAddress("127.0.0.1", core.udpPort())));
            }
            byte[] junk = {1, 2, 3};
            br.send(new DatagramPacket(junk, junk.length, new InetSocketAddress("127.0.0.1", core.udpPort())));
            boolean imuOk = core.offerImu(imuFrame(0), 36);
            Thread.sleep(100);
            org.json.JSONObject j = core.stop();
            br.close();
            out.println("no_port: " + j);
            check(!imuOk && j.optLong("imu_no_port") == 1, "no port: offerImu false, counted");
            check(j.optLong("udp_in") == 101 && j.optLong("udp_bad") == 1 && j.optLong("lo_no_port") == 100 && j.optLong("writes") == 0,
                    "no port: 100 poses drained + dropped, 1 junk datagram bad, 0 writes");
        }

        // 7: stuck FC (every write times out): queues bounded, oldest dropped, sensor thread never blocks
        {
            // 7a fast path on: the sensor thread blocks for at most one 2 ms timeout per frame (= 0.6.37c), never longer
            {
                FakePort p = new FakePort(0, 0, true);
                ViopRelayCore core = new ViopRelayCore(p, null);
                core.start(false);
                long mx = 0;
                for (int i = 0; i < 200; i++) {
                    long t = System.nanoTime();
                    core.offerImu(imuFrame(i), 36);
                    mx = Math.max(mx, System.nanoTime() - t);
                }
                org.json.JSONObject j = core.stop();
                out.println("stuck_direct: " + j + " offer max us " + mx / 1000);
                check(j.optLong("imu_drop_full") + j.optLong("imu_fail") == 200, "stuck direct: every IMU frame accounted");
                out.println("stuck direct: offerImu max " + mx / 1000 + " us (one 2 ms timeout + preemption; old path the same)");
            }
            FakePort p = new FakePort(0, 0, true);
            ViopRelayCore core = new ViopRelayCore(p, null);
            core.imuDirect = false;   // 7b queue bound
            core.start(false);
            long mx = 0;
            for (int i = 0; i < 500; i++) {
                long t = System.nanoTime();
                core.offerImu(imuFrame(i), 36);   // burst: faster than the stuck writer drains (7 frames / 2 ms)
                mx = Math.max(mx, System.nanoTime() - t);
            }
            org.json.JSONObject j = core.stop();
            out.println("stuck: " + j + " offer max us " + mx / 1000);
            check(j.optLong("imu_drop_full") > 0 && j.optLong("imu_written") == 0 && j.optLong("imu_fail") > 0, "stuck: drops + fails counted");
            check(j.optLong("imu_drop_full") + j.optLong("imu_fail") == 500, "stuck: every IMU frame accounted (drop + fail = 500)");
            check(mx < 2_000_000L, "stuck: offerImu max " + mx / 1000 + " us < 2 ms");
        }

        // 8: stop drains the queue (the bridge's 3 STOPPING heartbeats sent just before stop still go out)
        {
            FakePort p = new FakePort(300000, 0, false);
            ViopRelayCore core = new ViopRelayCore(p, null);
            core.start(true);
            DatagramSocket br = new DatagramSocket(new InetSocketAddress("127.0.0.1", 0));
            for (int i = 0; i < 3; i++) {
                byte[] h = ViopRelayCore.build(ViopRelayCore.T_HB, 1, i, hbPayload(i));
                br.send(new DatagramPacket(h, h.length, new InetSocketAddress("127.0.0.1", core.udpPort())));
            }
            Thread.sleep(30);
            org.json.JSONObject j = core.stop();
            br.close();
            int[] bad = {0};
            check(parse(p.sink.toByteArray(), bad).size() == 3 && j.optLong("lo_written") == 3, "stop: 3 STOPPING heartbeats written before the link closes");
        }

        String res = (fails == 0 ? "ALL PASS: " : "FAILED: ") + checks + " checks, " + fails + " failed";
        out.println(res);
        out.close();
        System.out.println(res);
        System.exit(fails == 0 ? 0 : 1);
    }
}
