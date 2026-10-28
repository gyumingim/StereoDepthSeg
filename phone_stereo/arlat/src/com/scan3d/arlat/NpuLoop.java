package com.scan3d.arlat;

import android.content.Context;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

/**
 * appnpu 2026-10-28 (0.6.30): XFeat + LighterGlue on the S24 NPU for the loop-closure verification of ov_bridge (intent
 * npu_loop 1, default 0 = this class is never touched and libxfnpu.so is never loaded).
 *
 * The NPU is reached through the vendor library libenn_public_api_cpp.so, which only this app process may dlopen (manifest
 * uses-native-library, required=false); the bridge is a separate executable, so the NPU work runs here in a native server
 * thread (npu/xfnpu_core.h) that the bridge's loop thread connects to: ov_bridge --loop-xf unix:@SOCK --loop-cfg CFG.
 * start() opens ENN + both models + one warm-up run each BEFORE the bridge starts; any failure -> returns the reason and
 * OvRunner REFUSES the run (0.6.31 user rule: no fallback of any kind). No CPU / GPU fallback for the models.
 * Model files: name -> files/npu/NAME.nnc (internal storage; ENN gets a file path). Copied from the APK assets npu/NAME.nnc
 * when that asset exists (rewritten if the size differs), else from external files/npu/NAME.nnc (adb push target), else an
 * absolute path is used as is. Same for the LighterGlue positional-encoding weights npu/lg_wr_LGNAME.npy.
 * Logs: run dir npu.jsonl (per keyframe / match: times in us, points, SKIN / AP / battery / thermal status, cache) and
 * npu_start.txt; meta.json "npu_*".
 */
final class NpuLoop {
    // settings (intents, MainActivity)
    String xfName = "x960p1mc";        // npu_xf: XFeat 960x544 1ch, softmax in the model (xf960soft); x960f1mc = post_off option
    String lgName = "L6_a16mc";        // npu_lg: LighterGlue 6 layers N 512 (lgnpu)
    String lgIn = "x512";              // npu_lg_in: x512 (XFeat points only) | trk768 (tracks + XFeat, masked model needed)
    int lgPad = 2;                     // npu_lg_pad: 0 zero-centre, 1 repeat, 2 repeat + copy sum (PC check pad_check.json)
    int topk = 512;                    // npu_topk
    String cpus = "";                  // npu_cpus: "" = auto (complement of the bridge cpus, else 0-3), "none" = no pinning
    int nice = 10;                     // npu_nice
    int perf = -1;                     // npu_perf: EnnSetPreferencePerfMode (-1 = not called)
    double cacheMb = 300;              // npu_cache_mb
    String cfgExtra = "";              // npu_cfg: appended to the bridge --loop-cfg (K=V,...)
    String opts = "";                 // 0.6.35 npu_opts: nn_fast=0|1,lg_post=2|3 (server CPU post speed options, default = 0.6.34)
    int timeoutMs = 3000;              // npu_timeout_ms: bridge socket timeout -> BRIEF fallback for the rest of the run

    static final String BRIDGE_CFG = "xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1";   // = lgopt FLGc gates

    private static native String nStart(String sock, String xf, String lg, String wr, String log, int topk, String lgIn, int lgPad,
                                        String cpus, int nice, int perf, double cacheMb, String opts);
    private static native String nStartLog();
    private static native String nStop();   // 0.6.31: "" or the server failure (exec errors / stuck worker)
    private static native void nTemps(float skin, float ap, float bat, int status);
    private static native boolean nFailed();   // 0.6.33: server recorded a failure (cheap: mutex + empty check)

    boolean failedNow() { try { return running && nFailed(); } catch (Throwable e) { return false; } }

    private final Context ctx;
    private String sock;
    private volatile boolean running;
    private Thread tempT;
    private android.os.IBinder thermalBinder;
    String usedCpus = "";
    String xfPath = "", lgPath = "", wrPath = "";

    NpuLoop(Context ctx) { this.ctx = ctx; }

    String sock() { return sock; }

    String bridgeCfg() {
        String c = BRIDGE_CFG + ",xf_timeout_ms=" + timeoutMs;
        return cfgExtra.isEmpty() ? c : c + "," + cfgExtra;
    }

    /** @return null when the server runs, else why the feature is off (the caller logs it and runs without it). */
    String start(File runDir, String bridgeCpus) {
        try {
            if (!lgIn.matches("x512|trk768")) return "bad npu_lg_in " + lgIn;
            if (!opts.isEmpty() && !opts.matches("(nn_fast=[01]|lg_post=[23])(,(nn_fast=[01]|lg_post=[23]))*")) return "bad npu_opts " + opts;
            if (!cfgExtra.isEmpty() && !cfgExtra.matches("[a-z_]+=[-0-9.eE]+(,[a-z_]+=[-0-9.eE]+)*")) return "bad npu_cfg " + cfgExtra;
            usedCpus = pickCpus(bridgeCpus);
            File dir = new File(ctx.getFilesDir(), "npu");
            if (!dir.isDirectory() && !dir.mkdirs()) return "cannot create " + dir;
            xfPath = model(dir, xfName, ".nnc");
            lgPath = model(dir, lgName, ".nnc");
            wrPath = model(dir, "lg_wr_" + lgName, ".npy");
            if (xfPath == null) return "no XFeat model " + xfName + " (assets npu/, files/npu/ or external files/npu/)";
            if (lgPath == null) return "no LighterGlue model " + lgName;
            if (wrPath == null) return "no lg_wr_" + lgName + ".npy";
            try { System.loadLibrary("xfnpu"); } catch (Throwable e) { return "loadLibrary xfnpu: " + e; }
            sock = "scan3d_xfnpu_" + android.os.Process.myPid();
            String err = nStart(sock, xfPath, lgPath, wrPath, new File(runDir, "npu.jsonl").getAbsolutePath(), topk, lgIn, lgPad,
                    "none".equals(usedCpus) ? "" : usedCpus, nice, perf, cacheMb, opts);
            writeText(new File(runDir, "npu_start.txt"), "models: " + xfPath + " | " + lgPath + " | " + wrPath + "\ncpus: " + usedCpus
                    + "\n" + nStartLog() + (err.isEmpty() ? "started\n" : "FAILED: " + err + "\n"));
            if (!err.isEmpty()) return err;
            running = true;
            startTemps();
            return null;
        } catch (Throwable e) {
            return "npu start: " + e;
        }
    }

    /** @return "" or why the NPU failed during the run (0.6.31: the caller marks the run INVALID). */
    String stop() {
        if (!running) return "";
        running = false;
        if (tempT != null) { tempT.interrupt(); try { tempT.join(1000); } catch (InterruptedException ignored) { } }
        try { String r = nStop(); return r == null ? "" : r; } catch (Throwable e) { android.util.Log.e("arlat", "npu stop", e); return "nStop: " + e; }
    }

    /** "" = auto: complement of the bridge --cpus inside 0-9 if the bridge is pinned, else the little cores 0-3 (추정: VIO
     *  threads are scheduled on the mid / big cores when not pinned; not measured). */
    String pickCpus(String bridgeCpus) {
        if (!cpus.isEmpty()) {
            if (!cpus.equals("none") && !cpus.matches("[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*")) throw new IllegalArgumentException("bad npu_cpus " + cpus);
            return expand(cpus);
        }
        if (bridgeCpus != null && !bridgeCpus.isEmpty()) {
            java.util.Set<Integer> used = new java.util.HashSet<>();
            for (String t : expand(bridgeCpus).split(",")) used.add(Integer.parseInt(t));
            StringBuilder b = new StringBuilder();
            for (int c = 0; c <= 9; c++) if (!used.contains(c)) b.append(b.length() > 0 ? "," : "").append(c);
            if (b.length() > 0) return b.toString();
        }
        return "0,1,2,3";
    }

    private static String expand(String s) {
        if (s.equals("none")) return s;
        StringBuilder b = new StringBuilder();
        for (String t : s.split(",")) {
            String[] r = t.split("-");
            int a = Integer.parseInt(r[0]), z = r.length > 1 ? Integer.parseInt(r[1]) : a;
            for (int c = a; c <= z; c++) b.append(b.length() > 0 ? "," : "").append(c);
        }
        return b.toString();
    }

    /** NAME -> internal files/npu/NAME+ext (copied from assets npu/ or external files/npu/), or an absolute path. */
    private String model(File dir, String name, String ext) throws Exception {
        if (name.startsWith("/")) return new File(name).isFile() ? name : null;
        if (!name.matches("[A-Za-z0-9_.-]+")) throw new IllegalArgumentException("bad model name " + name);
        File dst = new File(dir, name + ext);
        String asset = "npu/" + name + ext;
        long alen = assetLen(asset);
        if (alen > 0) {
            if (dst.isFile() && dst.length() == alen) return dst.getAbsolutePath();
            try (InputStream in = ctx.getAssets().open(asset)) { copy(in, dst, alen); }
            return dst.getAbsolutePath();
        }
        File ext0 = ctx.getExternalFilesDir(null);
        File pushed = ext0 == null ? null : new File(new File(ext0, "npu"), name + ext);
        if (pushed != null && pushed.isFile()) {
            if (dst.isFile() && dst.length() == pushed.length()) return dst.getAbsolutePath();
            try (InputStream in = new java.io.FileInputStream(pushed)) { copy(in, dst, pushed.length()); }
            return dst.getAbsolutePath();
        }
        return dst.isFile() ? dst.getAbsolutePath() : null;
    }

    private long assetLen(String a) {
        try (android.content.res.AssetFileDescriptor fd = ctx.getAssets().openFd(a)) { return fd.getLength(); }
        catch (Exception e) { /* compressed asset or missing */ }
        try (InputStream in = ctx.getAssets().open(a)) {
            long n = 0; byte[] b = new byte[1 << 16]; int k;
            while ((k = in.read(b)) > 0) n += k;
            return n;
        } catch (Exception e) { return -1; }
    }

    private static void copy(InputStream in, File dst, long want) throws Exception {
        File tmp = new File(dst.getPath() + ".tmp");
        try (OutputStream o = new FileOutputStream(tmp)) { byte[] b = new byte[1 << 16]; int n; while ((n = in.read(b)) > 0) o.write(b, 0, n); }
        if (tmp.length() != want || !tmp.renameTo(dst)) throw new IllegalStateException("cannot write " + dst);
    }

    private static void writeText(File f, String s) {
        try { java.nio.file.Files.write(f.toPath(), s.getBytes(StandardCharsets.UTF_8)); } catch (Exception ignored) { }
    }

    // ---- SKIN / AP every 2 s for npu.jsonl (thermalservice binder dump = 'dumpsys thermalservice'; needs the DUMP
    // permission granted over adb, as HsRecorder). Without it: NaN (battery temperature + thermal status still logged).
    private void startTemps() {
        try {
            Class<?> smc = Class.forName("android.os.ServiceManager");
            thermalBinder = (android.os.IBinder) smc.getMethod("getService", String.class).invoke(null, "thermalservice");
        } catch (Throwable e) { thermalBinder = null; }
        tempT = new Thread(() -> {
            while (running) {
                java.util.Map<String, Float> t = dump();
                float bat = batteryC();
                int st = thermalStatus();
                try { nTemps(get(t, "SKIN"), get(t, "AP"), bat, st); } catch (Throwable ignored) { }
                try { Thread.sleep(2000); } catch (InterruptedException e) { return; }
            }
        }, "npu-temps");
        tempT.setDaemon(true);
        tempT.start();
    }

    private static float get(java.util.Map<String, Float> m, String k) { Float v = m.get(k); return v == null ? Float.NaN : v; }

    private float batteryC() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi == null ? Float.NaN : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0) / 10f;
    }

    private int thermalStatus() {
        try { return ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus(); }
        catch (Throwable e) { return -1; }
    }

    private static final java.util.regex.Pattern HAL_T = java.util.regex.Pattern.compile("mValue=([-0-9.eE]+).*?mName=([^,}]+)");

    private java.util.Map<String, Float> dump() {   // same parsing as HsRecorder.dumpRead
        java.util.LinkedHashMap<String, Float> out = new java.util.LinkedHashMap<>();
        if (thermalBinder == null) return out;
        android.os.ParcelFileDescriptor[] p = null;
        try {
            p = android.os.ParcelFileDescriptor.createPipe();
            thermalBinder.dumpAsync(p[1].getFileDescriptor(), new String[0]);
            p[1].close(); p[1] = null;
            boolean in = false;
            try (java.io.BufferedReader r = new java.io.BufferedReader(new java.io.InputStreamReader(
                    new android.os.ParcelFileDescriptor.AutoCloseInputStream(p[0])))) {
                p[0] = null;
                for (String line; (line = r.readLine()) != null; ) {
                    if (line.contains("Current temperatures from HAL")) { in = true; continue; }
                    if (!in) continue;
                    if (!line.isEmpty() && !Character.isWhitespace(line.charAt(0))) break;
                    java.util.regex.Matcher m = HAL_T.matcher(line);
                    if (m.find() && !out.containsKey(m.group(2).trim())) out.put(m.group(2).trim(), Float.parseFloat(m.group(1)));
                }
            }
        } catch (Throwable ignored) { }
        finally {
            if (p != null) for (android.os.ParcelFileDescriptor f : p) if (f != null) try { f.close(); } catch (Exception ignored) { }
        }
        return out;
    }
}
