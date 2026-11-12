package com.scan3d.arlat;

import android.content.Context;
import android.media.Image;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.os.SystemClock;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.zip.GZIPOutputStream;

/**
 * phonevio 2026-11-12 (0.6.40, mode replay): OFFLINE VIO of a finished recording ON THE PHONE, no camera, no sensors.
 *   adb shell am start -n com.scan3d.arlat/.MainActivity --es mode replay --es rec hs_20261112_121801_std60 [--es cfg c4f60]
 * = the PC offline replay scan3d/runs/livepose_20261030/lp_replay.py (run_scanner.sh step vio, defaults --imu raw --time live
 * --loop 1 --fps 60), ported line by line:
 *   - config files/ov/CFG (default c4f60) copied to OUT/cfg/, the "/storage/.../files/ov/NAME/" paths replaced by OUT/cfg/
 *     (lp_replay regex; record_timing_information kept as in the yaml)
 *   - bridge args = lp_replay: --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --remap CFG/remap_1920x1080.bin
 *     --profile rec60 --drop-late 0 --idle-spin 0 --predict 0 --loop 1 --loop-sync 1 --loop-vocab files/ov/voc_k10L4.voc --loop-log OUT/loop.jsonl
 *   - frames: frames_cam5.csv t_mid_ns, selection = vio_bench_hs.fed_selection / pick_by_time (60 fps grid, trim.json if present)
 *   - IMU: imu.csv uncalibrated types 35 (acc) / 16 (gyro) values v0..v2, acc linearly interpolated to the gyro times (np.interp),
 *     feed from t_start - 2 s, before each frame every IMU sample <= t_frame + 5 ms, after the last frame 200 more, end marker
 *   - image: cam5.mp4 decoded by MediaCodec (H.264 is bit exact), Y plane -> LUT round((Y - 16) * 255 / 219) clipped = the
 *     PC's OpenCV BGR->GRAY of a limited-range stream up to chroma rounding/clipping (PC check on 121801: mean |diff| 1.4 levels)
 *   - every frame waits for the pipe: nothing is dropped (blocking writes, drop-late 0)
 * Writes REC/phonevio_<stamp>/: poses.jsonl.gz (every bridge "pose" line as printed), loop.jsonl, bridge.err, temps.csv
 * (2 s: battery, thermal status, frames fed, poses), run.json (timing, temperatures, counts, input notes, sha of cfg files).
 * Stops (bridge killed, run.json status "aborted: ...") when the battery reaches maxBatteryTenthC (default 42.0 C).
 */
class ReplayVio {
    interface Status { void show(String s); }
    interface Done { void done(String result); }

    static final String VERSION = "phonevio-20261112-0.6.40";
    private static final int MAGIC = 0x4F564252;
    private final Context ctx;
    private final Status status;
    private final Done done;
    final String recName, cfgName;
    int maxBatteryTenthC = 420;
    double fps = 60.0;

    private volatile boolean stop;
    private volatile String abortWhy;
    private volatile int nFed, nSel, nDecoded, nPose, nTracking;
    private volatile long tFirstTrackNs = -1;
    private int nTotal;
    private Process proc;
    private Thread mainT;

    ReplayVio(Context ctx, String rec, String cfg, Status status, Done done) {
        this.ctx = ctx; this.recName = rec; this.cfgName = cfg; this.status = status; this.done = done;
    }

    void start() {
        mainT = new Thread(() -> {
            String res;
            try { res = run(); }
            catch (Throwable e) { android.util.Log.e("arlat", "replay", e); res = "오류: " + e; }
            if (done != null) done.done(res);
        }, "phonevio");
        mainT.start();
    }

    void stop() {
        stop = true;
        if (abortWhy == null) abortWhy = "stopped (activity destroyed)";
        Process p = proc;
        if (p != null) p.destroy();
    }

    boolean running() { return mainT != null && mainT.isAlive(); }

    private static String sha(File f) {
        try (FileInputStream in = new FileInputStream(f)) {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            byte[] b = new byte[1 << 16]; int n;
            while ((n = in.read(b)) > 0) md.update(b, 0, n);
            StringBuilder sb = new StringBuilder();
            for (byte x : md.digest()) sb.append(String.format("%02x", x));
            return sb.substring(0, 16);
        } catch (Exception e) { return "err " + e; }
    }

    int batteryTenthC() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi == null ? 0 : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
    }

    int thermalStatus() {
        try { return ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus(); }
        catch (Throwable e) { return -1; }
    }

    // ---- inputs (lp_replay / vio_bench_hs / imu_gt.read_imu)
    private static long[] readFrames(File csv) throws Exception {
        List<String> lines = java.nio.file.Files.readAllLines(csv.toPath(), StandardCharsets.UTF_8);
        String[] hdr = lines.get(0).split(",");
        int c = Arrays.asList(hdr).indexOf("t_mid_ns");
        if (c < 0) throw new IllegalStateException("frames_cam5.csv: no t_mid_ns");
        long[] t = new long[lines.size() - 1];
        int n = 0;
        for (int i = 1; i < lines.size(); i++) {
            String l = lines.get(i);
            if (l.isEmpty()) continue;
            t[n++] = Long.parseLong(l.split(",")[c]);
        }
        return Arrays.copyOf(t, n);
    }

    /** pick_by_time (vio_bench_hs.py): nearest frame per grid step, gap rule 0.75 step. */
    static int[] pickByTime(long[] tf, long tStart, long tEnd, double fps, double periodNs) {
        double step = 1e9 / fps;
        int ng = (int) ((tEnd - tStart) / step) + 1;
        double tol = Math.min(step / 2, 1.01 * periodNs);
        int n = tf.length;
        java.util.TreeSet<Integer> js = new java.util.TreeSet<>();
        for (int k = 0; k < ng; k++) {
            double g = tStart + k * step;
            int j = 0;                                              // searchsorted left: first t >= g
            int lo = 0, hi = n;
            while (lo < hi) { int m = (lo + hi) >>> 1; if (tf[m] < g) lo = m + 1; else hi = m; }
            j = Math.max(1, Math.min(n - 1, lo));
            if (Math.abs(tf[j - 1] - g) <= Math.abs(tf[j] - g)) j = j - 1;
            if (Math.abs(tf[j] - g) <= tol && tf[j] >= tStart && tf[j] <= tEnd) js.add(j);
        }
        List<Integer> out = new ArrayList<>();
        Long last = null;
        for (int i : js) {
            if (last == null || tf[i] - last >= 0.75 * step) { out.add(i); last = tf[i]; }
        }
        int[] r = new int[out.size()];
        for (int i = 0; i < r.length; i++) r[i] = out.get(i);
        return r;
    }

    /** np.interp(x, xp, fp) for one x (xp ascending): clamp outside, last j with xp[j] <= x. */
    static double interp(double x, double[] xp, double[] fp) {
        int n = xp.length;
        if (x <= xp[0]) return fp[0];
        if (x >= xp[n - 1]) return fp[n - 1];
        int lo = 0, hi = n;                                         // upper bound: first xp > x
        while (lo < hi) { int m = (lo + hi) >>> 1; if (xp[m] <= x) lo = m + 1; else hi = m; }
        int j = lo - 1;
        if (x == xp[j]) return fp[j];
        double slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
        return slope * (x - xp[j]) + fp[j];
    }

    private String run() throws Exception {
        long w0 = SystemClock.elapsedRealtimeNanos();
        File files = ctx.getExternalFilesDir(null);
        File rec = new File(files, recName);
        File ovDir = new File(files, "ov");
        File cfgSrc = new File(ovDir, cfgName);
        for (String f : new String[]{"meta.json", "frames_cam5.csv", "imu.csv", "cam5.mp4"})
            if (!new File(rec, f).isFile()) throw new IllegalStateException("no " + f + " in " + rec);
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
        File out = new File(rec, "phonevio_" + stamp);
        File cfg = new File(out, "cfg");
        if (!cfg.mkdirs()) throw new IllegalStateException("cannot create " + cfg);
        int batStart = batteryTenthC(), thStart = thermalStatus();
        JSONObject meta = new JSONObject(new String(java.nio.file.Files.readAllBytes(new File(rec, "meta.json").toPath()), StandardCharsets.UTF_8));
        int W = meta.getInt("width"), H = meta.getInt("height");
        double saveFps = meta.optDouble("save_fps", 240);
        if (saveFps <= 0 || Double.isNaN(saveFps)) saveFps = 240;
        status.show("준비 중: " + recName);

        // ---- config (lp_replay: copy *.yaml, replace the phone paths of the config folder by the copy)
        JSONObject yamlSha = new JSONObject();
        File[] ys = cfgSrc.listFiles();
        if (ys == null) throw new IllegalStateException("no config " + cfgSrc);
        String prefix = "\"/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/";
        for (File f : ys) {
            if (!f.getName().endsWith(".yaml")) continue;
            String t = new String(java.nio.file.Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8);
            if (f.getName().equals("estimator_config.yaml"))
                t = t.replaceAll(java.util.regex.Pattern.quote(prefix) + "[A-Za-z0-9_]+/", java.util.regex.Matcher.quoteReplacement("\"" + cfg.getAbsolutePath() + "/"));
            java.nio.file.Files.write(new File(cfg, f.getName()).toPath(), t.getBytes(StandardCharsets.UTF_8));
        }
        File[] cs = cfg.listFiles(); Arrays.sort(cs);
        for (File f : cs) yamlSha.put(f.getName(), sha(f));
        File remap = new File(cfgSrc, "remap_" + W + "x" + H + ".bin");
        if (!remap.isFile()) throw new IllegalStateException("no " + remap);
        File voc = new File(ovDir, "voc_k10L4.voc");
        if (!voc.isFile()) throw new IllegalStateException("no " + voc);
        String lib = ctx.getApplicationInfo().nativeLibraryDir;
        File bridge = new File(lib, "libov_bridge.so");
        List<String> cmd = new ArrayList<>();
        Collections.addAll(cmd, bridge.getAbsolutePath(), new File(cfg, "estimator_config.yaml").getAbsolutePath(), "--stdio",
                "--max-speed", "8", "--lost-frames", "1000000", "--startup-lost-frames", "1000000", "--remap", remap.getAbsolutePath(),
                "--profile", "rec60", "--drop-late", "0", "--idle-spin", "0", "--predict", "0",
                "--loop", "1", "--loop-sync", "1", "--loop-vocab", voc.getAbsolutePath(), "--loop-log", new File(out, "loop.jsonl").getAbsolutePath());

        // ---- inputs
        long[] tf = readFrames(new File(rec, "frames_cam5.csv"));
        nTotal = tf.length;
        long tStart = tf[0];
        File trim = new File(rec, "imu_gt/trim.json");
        if (trim.isFile()) {
            JSONObject tj = new JSONObject(new String(java.nio.file.Files.readAllBytes(trim.toPath()), StandardCharsets.UTF_8));
            tStart = Math.max(tStart, tj.getLong("start_ns"));
        }
        long tEnd = tf[tf.length - 1];
        int[] sel = pickByTime(tf, tStart, tEnd, fps, 1e9 / saveFps);
        boolean[] selected = new boolean[tf.length];
        for (int i : sel) selected[i] = true;
        status.show("IMU 읽는 중 ...");
        ArrayList<double[]> acc = new ArrayList<>(), gyr = new ArrayList<>();
        int bad = 0;
        try (BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(new File(rec, "imu.csv")), StandardCharsets.UTF_8), 1 << 16)) {
            String l;
            while ((l = br.readLine()) != null) {
                if (l.isEmpty() || l.startsWith("type")) continue;
                String[] r = l.split(",", -1);
                if (r.length < 5) { bad++; continue; }
                try {
                    int ty = Integer.parseInt(r[0]);
                    if (ty != 35 && ty != 16) continue;
                    double[] v = {Long.parseLong(r[1]), Double.parseDouble(r[2]), Double.parseDouble(r[3]), Double.parseDouble(r[4])};
                    (ty == 35 ? acc : gyr).add(v);
                } catch (RuntimeException e) { bad++; }
            }
        }
        if (acc.isEmpty() || gyr.isEmpty()) throw new IllegalStateException("no uncalibrated IMU rows (35: " + acc.size() + ", 16: " + gyr.size() + ")");
        int na = acc.size(), ng = gyr.size();
        double[] at = new double[na]; double[][] av = new double[3][na];
        for (int i = 0; i < na; i++) { double[] v = acc.get(i); at[i] = v[0]; for (int j = 0; j < 3; j++) av[j][i] = v[1 + j]; }
        long[] tImu = new long[ng]; double[][] gI = new double[ng][3], aI = new double[ng][3];
        for (int i = 0; i < ng; i++) {
            double[] v = gyr.get(i);
            tImu[i] = (long) v[0];
            for (int j = 0; j < 3; j++) { gI[i][j] = v[1 + j]; aI[i][j] = interp(v[0], at, av[j]); }
        }
        acc = null; gyr = null;

        // ---- bridge
        ProcessBuilder pb = new ProcessBuilder(cmd);
        pb.environment().put("LD_LIBRARY_PATH", lib);
        pb.redirectError(new File(out, "bridge.err"));
        final GZIPOutputStream gz = new GZIPOutputStream(new FileOutputStream(new File(out, "poses.jsonl.gz")), 1 << 16);
        final BufferedWriter tempsW = new BufferedWriter(new FileWriter(new File(out, "temps.csv")));
        tempsW.write("t_rel_s,battery_c,thermal_status,frames_decoded,frames_fed,poses\n");
        long tw0 = SystemClock.elapsedRealtimeNanos();
        proc = pb.start();
        final Process p = proc;
        final int[] statusCounts = new int[3];   // init, tracking, other
        Thread reader = new Thread(() -> {
            try (BufferedReader br = new BufferedReader(new InputStreamReader(p.getInputStream(), StandardCharsets.UTF_8), 1 << 16)) {
                String l;
                while ((l = br.readLine()) != null) {
                    if (!l.contains("\"pose\"")) continue;
                    JSONObject m;
                    try { m = new JSONObject(l); } catch (Exception e) { continue; }
                    if (!"pose".equals(m.optString("type"))) continue;
                    gz.write((l + "\n").getBytes(StandardCharsets.UTF_8));
                    nPose++;
                    String s = m.optString("status");
                    if ("tracking".equals(s)) { statusCounts[1]++; nTracking++; if (tFirstTrackNs < 0) tFirstTrackNs = m.optLong("t_ns", -1); }
                    else if ("init".equals(s)) statusCounts[0]++;
                    else statusCounts[2]++;
                }
            } catch (Exception e) { android.util.Log.w("arlat", "replay reader", e); }
        }, "phonevio-reader");
        reader.start();
        final long[] maxBat = {batStart}; final int[] maxTh = {thStart};
        Thread temps = new Thread(() -> {
            while (!stop) {
                int b = batteryTenthC(), ts = thermalStatus();
                if (b > maxBat[0]) maxBat[0] = b;
                if (ts > maxTh[0]) maxTh[0] = ts;
                double tr = (SystemClock.elapsedRealtimeNanos() - tw0) / 1e9;
                try { tempsW.write(String.format(Locale.US, "%.1f,%.1f,%d,%d,%d,%d\n", tr, b / 10.0, ts, nDecoded, nFed, nPose)); tempsW.flush(); }
                catch (Exception ignored) { }
                status.show(String.format(Locale.US, "폰에서 VIO 처리 중 (카메라 안 씀)\n%s\n\n프레임 %d / %d (%.0f %%)\n자세 %d (추적 %d)\n경과 %.0f 초\n배터리 %.1f °C, 발열 상태 %d",
                        recName, nFed, sel.length, 100.0 * nFed / Math.max(1, sel.length), nPose, nTracking, tr, b / 10.0, ts));
                if (b >= maxBatteryTenthC && abortWhy == null) {
                    abortWhy = "battery " + b / 10.0 + " C >= " + maxBatteryTenthC / 10.0;
                    stop = true; p.destroy();
                }
                try { Thread.sleep(2000); } catch (InterruptedException e) { break; }
            }
        }, "phonevio-temps");
        temps.start();

        // ---- decode + feed (one thread, blocking pipe writes = no frame dropped)
        OutputStream os = p.getOutputStream();
        ByteBuffer imuRec = ByteBuffer.allocate(12 + 56).order(ByteOrder.LITTLE_ENDIAN);
        byte[] img = new byte[12 + 20 + W * H];
        ByteBuffer ib = ByteBuffer.wrap(img).order(ByteOrder.LITTLE_ENDIAN);
        byte[] lut = new byte[256];
        for (int v = 0; v < 256; v++) lut[v] = (byte) Math.max(0, Math.min(255, Math.round((v - 16) * 255.0 / 219.0)));
        byte[] row = new byte[8192];
        int ii = 0;
        { long t0 = tStart - 2_000_000_000L; int lo = 0, hi = ng; while (lo < hi) { int m = (lo + hi) >>> 1; if (tImu[m] < t0) lo = m + 1; else hi = m; } ii = lo; }
        String crashed = null;
        long decodeNs = 0, convNs = 0, writeNs = 0;
        MediaExtractor ex = new MediaExtractor();
        MediaCodec dec = null;
        String decName = "";
        int nImu = 0;
        try {
            ex.setDataSource(new File(rec, "cam5.mp4").getAbsolutePath());
            int track = -1; MediaFormat fmt = null;
            for (int i = 0; i < ex.getTrackCount(); i++) {
                MediaFormat f = ex.getTrackFormat(i);
                if (f.getString(MediaFormat.KEY_MIME).startsWith("video/")) { track = i; fmt = f; break; }
            }
            if (track < 0) throw new IllegalStateException("no video track");
            ex.selectTrack(track);
            fmt.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible);
            dec = MediaCodec.createDecoderByType(fmt.getString(MediaFormat.KEY_MIME));
            decName = dec.getName();
            dec.configure(fmt, null, null, 0);
            dec.start();
            MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();
            boolean inEos = false, outEos = false;
            int fi = 0;
            long td = System.nanoTime();
            feed:
            while (!outEos && !stop) {
                if (!inEos) {
                    int ix = dec.dequeueInputBuffer(2000);
                    if (ix >= 0) {
                        ByteBuffer bb = dec.getInputBuffer(ix);
                        int sz = ex.readSampleData(bb, 0);
                        if (sz < 0) { dec.queueInputBuffer(ix, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM); inEos = true; }
                        else { dec.queueInputBuffer(ix, 0, sz, ex.getSampleTime(), 0); ex.advance(); }
                    }
                }
                int ox = dec.dequeueOutputBuffer(info, 2000);
                if (ox < 0) continue;
                if ((info.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) outEos = true;
                if (info.size == 0 && outEos) { dec.releaseOutputBuffer(ox, false); break; }
                decodeNs += System.nanoTime() - td;
                int cur = fi++;
                nDecoded = fi;
                if (cur >= tf.length) { dec.releaseOutputBuffer(ox, false); break; }
                long tc = tf[cur];
                if (tc > tEnd) { dec.releaseOutputBuffer(ox, false); break; }
                if (!selected[cur]) { dec.releaseOutputBuffer(ox, false); td = System.nanoTime(); continue; }
                nSel++;
                long tw = System.nanoTime();
                while (ii < ng && tImu[ii] <= tc + 5_000_000L) { writeImu(os, imuRec, tImu[ii], gI[ii], aI[ii]); ii++; nImu++; }
                long tcv = System.nanoTime();
                Image im = dec.getOutputImage(ox);
                if (im == null) throw new IllegalStateException("decoder gave no image (frame " + cur + ")");
                android.graphics.Rect cr = im.getCropRect();
                Image.Plane y = im.getPlanes()[0];
                ByteBuffer yb = y.getBuffer();
                int rs = y.getRowStride(), ps = y.getPixelStride();
                if (cr.width() != W || cr.height() != H) throw new IllegalStateException("decoded " + cr + " != " + W + "x" + H);
                if (ps != 1) throw new IllegalStateException("Y pixel stride " + ps);
                ib.clear();
                ib.putInt(MAGIC).putInt(2).putInt(20 + W * H).putLong(tc).putInt(W).putInt(H).put((byte) 1).put((byte) 0).put((byte) 0).put((byte) 0);
                if (row.length < W) row = new byte[W];
                for (int r = 0; r < H; r++) {
                    yb.position((r + cr.top) * rs + cr.left);
                    yb.get(row, 0, W);
                    int o = 32 + r * W;
                    for (int c = 0; c < W; c++) img[o + c] = lut[row[c] & 0xFF];
                }
                im.close();
                dec.releaseOutputBuffer(ox, false);
                long tww = System.nanoTime();
                convNs += tww - tcv;
                try { os.write(img); }
                catch (java.io.IOException e) { crashed = "bridge pipe closed after " + nFed + " frames: " + e.getMessage(); break feed; }
                writeNs += System.nanoTime() - tww + (tcv - tw);
                nFed++;
                td = System.nanoTime();
            }
            if (crashed == null && !stop) {
                for (int k = 0; k < 200 && ii < ng; k++, ii++) writeImu(os, imuRec, tImu[ii], gI[ii], aI[ii]);
                ByteBuffer e = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN);
                e.putInt(MAGIC).putInt(4).putInt(0);
                os.write(e.array());
            }
            os.flush();
            os.close();
        } catch (java.io.IOException e) {
            if (crashed == null) crashed = "bridge pipe: " + e.getMessage() + " after " + nFed + " frames";
        } finally {
            try { if (dec != null) { dec.stop(); dec.release(); } } catch (Exception ignored) { }
            ex.release();
        }
        long tFedNs = SystemClock.elapsedRealtimeNanos();
        status.show("마지막 처리 기다리는 중 ...");
        int rc;
        if (!p.waitFor(300, java.util.concurrent.TimeUnit.SECONDS)) { p.destroy(); rc = -9; }
        else rc = p.exitValue();
        reader.join(10000);
        long tEndNs = SystemClock.elapsedRealtimeNanos();
        stop = true;
        temps.interrupt(); temps.join(3000);
        gz.close(); tempsW.close();
        if (crashed == null && rc != 0 && abortWhy == null) crashed = "bridge exited rc " + rc + " after all " + nFed + " frames were sent";
        int batEnd = batteryTenthC(), thEnd = thermalStatus();
        double recS = (tf[tf.length - 1] - tf[0]) / 1e9;
        double wall = (tEndNs - tw0) / 1e9;
        String st = abortWhy != null ? "aborted: " + abortWhy : (crashed != null ? "failed: " + crashed : "ok");
        JSONObject res = new JSONObject();
        res.put("rec", recName).put("made", new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new java.util.Date()))
                .put("version", VERSION).put("status", st).put("crashed", crashed == null ? JSONObject.NULL : crashed).put("bridge_rc", rc)
                .put("bridge_cmd", String.join(" ", cmd)).put("bridge_sha", sha(bridge)).put("remap_sha", sha(remap)).put("vocab_sha", sha(voc))
                .put("cfg_src", cfgSrc.getAbsolutePath()).put("yaml_sha", yamlSha)
                .put("decoder", decName).put("frames_csv", tf.length).put("frames_decoded", nDecoded).put("frames_selected", sel.length)
                .put("frames_fed", nFed).put("imu_rows_gyro", ng).put("imu_rows_acc", na).put("imu_bad_rows", bad).put("imu_fed", ii)
                .put("poses", nPose).put("status_counts", new JSONObject().put("init", statusCounts[0]).put("tracking", statusCounts[1]).put("other", statusCounts[2]))
                .put("t_first_fed", tf[sel[0]]).put("t_last_fed", tf[sel[sel.length - 1]])
                .put("t_first_tracking_ns", tFirstTrackNs).put("init_s_from_first_frame", tFirstTrackNs < 0 ? JSONObject.NULL : (tFirstTrackNs - tf[sel[0]]) / 1e9)
                .put("rec_len_s", recS).put("setup_s", (tw0 - w0) / 1e9).put("feed_s", (tFedNs - tw0) / 1e9).put("wall_s", wall)
                .put("wall_over_rec", wall / recS)
                .put("decode_ms_per_frame", decodeNs / 1e6 / Math.max(1, nDecoded)).put("convert_ms_per_frame", convNs / 1e6 / Math.max(1, nFed))
                .put("write_ms_per_frame", writeNs / 1e6 / Math.max(1, nFed))
                .put("battery_c_start", batStart / 10.0).put("battery_c_end", batEnd / 10.0).put("battery_c_max", maxBat[0] / 10.0)
                .put("thermal_status_start", thStart).put("thermal_status_end", thEnd).put("thermal_status_max", maxTh[0])
                .put("input_path", new JSONObject()
                        .put("decode", "cam5.mp4 MediaCodec " + decName + " Y plane -> LUT round((Y-16)*255/219) (PC: OpenCV-FFmpeg BGR -> GRAY)")
                        .put("imu", "uncalibrated 35/16 v0..2, acc interp to gyro times (= lp_replay --imu raw)")
                        .put("time", "t_mid_ns, no shift (= lp_replay --time live)")
                        .put("offline", "drop-late 0, loop-sync 1, idle-spin 0, predict 0; blocking pipe writes")
                        .put("selection", fps + " fps grid from save_fps " + saveFps));
        java.nio.file.Files.write(new File(out, "run.json").toPath(), res.toString(1).getBytes(StandardCharsets.UTF_8));
        String msg = String.format(Locale.US, "%s\n%s\n프레임 %d / %d, 자세 %d (추적 %d)\n처리 %.0f 초 (녹화 %.0f 초의 %.2f 배)\n배터리 %.1f → %.1f °C\n결과: %s",
                st.equals("ok") ? "끝 (정상)" : "끝: " + st, recName, nFed, sel.length, nPose, nTracking, wall, recS, wall / recS, batStart / 10.0, batEnd / 10.0, out.getName());
        android.util.Log.i("arlat", "phonevio done: " + res.toString());
        return msg;
    }

    private static void writeImu(OutputStream os, ByteBuffer b, long t, double[] g, double[] a) throws java.io.IOException {
        b.clear();
        b.putInt(MAGIC).putInt(1).putInt(56).putLong(t).putDouble(g[0]).putDouble(g[1]).putDouble(g[2]).putDouble(a[0]).putDouble(a[1]).putDouble(a[2]);
        os.write(b.array(), 0, 68);
    }
}
