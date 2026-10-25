package com.scan3d.arlat;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Range;
import android.util.Size;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Live OpenVINS on the phone -> ov_bridge child process (lib/arm64-v8a/libov_bridge.so, protocol
 * scan3d/ov_bridge/PROTOCOL.md) -> one JSON line per pose to the Streamer:
 *   {"src":"ov","t_sensor":..,"t_arrive":..,"t_out":..,"status":..,"p":[..],"q":[..],"ms":..,"n_slam":..,"n_msckf":..}
 * t_sensor = SENSOR_TIMESTAMP (start of exposure), t_arrive = image callback, t_out = pose line read (all elapsedRealtimeNanos).
 *
 * Two cameras (intent extra ov_cam):
 *  - "lens2" (default, unchanged since 2026-10-18): physical lens 2 of logical "0", YUV 640x480 @ 30 fps, calibrated
 *    gyro/accel, config + lens0u_remap.bin from files/ov/ (vio_bench_server.py / make_phone_ov.py), "live" bridge args.
 *  - "main" (2026-10-23, on-phone timing of the baseline): logical camera "0" = the std60 recording camera (HsRecorder
 *    session std), YUV ov_size (default 1920x1080) @ fps (60), fixed focus focus_dpt (0.5), manual exposure exp_us/iso
 *    (2000/500), OIS/EIS/NR/edge/distortion correction off like the recorder, uncalibrated gyro/accel (types 16/35,
 *    values 0..2 = the bench's --imu-raw), "bench" bridge args (vio_bench_hs.py), config from files/ov/<ov_cfg>/ and
 *    remap_<WxH>.bin (make_phone_ov_main.py: raw frame -> undistorted 640x360 pinhole, same maps as the bench).
 * Every run writes files/ov/run_<time>_<cam><fps>_<cfg>/: estimator_config.yaml (+ kalibr chains, timing paths set to
 * the run dir), ov_timing.txt (OpenVINS record_timing_information), bridge_timing.csv (ov_bridge --timing-log),
 * frames.csv (every image: fate sent / dropped), poses.csv (every pose line), temps.csv (battery + thermal status, 2 s),
 * events.jsonl (bridge resets / errors), bridge_stderr.log, meta.json (settings, stream offer, counts, stop reason).
 *
 * 0.6.13 (intent perf, default 1; perf 0 = off for A/B timing): app threads cam / imu / ov-writer / ov-reader at
 * THREAD_PRIORITY_URGENT_DISPLAY; at the 30th pose the ov_bridge child (same uid) threads get the same priority
 * (setpriority on another process's tids: works only if the kernel allows it -> read back and logged) and a
 * PerformanceHintManager session (API 31+) is created for app + bridge tids (Android 14+ accepts tids of other
 * processes of the same uid; on failure app tids only) with target = 1/fps; every pose reports the bridge "ms"
 * (OpenVINS time of that frame) as the actual duration. At the 300th pose new bridge threads are added (setThreads,
 * API 34+). MainActivity sets Window.setSustainedPerformanceMode(true) when supported. All -> meta.json "perf".
 *
 * 0.6.14 A/B knobs (both default off = 0.6.13 behaviour): intent hint_ms N (> 0) = hint session target N ms instead of
 * 1000/fps (a target below the reported "ms" asks the governor for more speed); intent cpus "7,8,9" / "7-9" = ov_bridge
 * --cpus (sched_setaffinity of all bridge threads, logged in bridge_stderr.log "[ov_bridge] affinity ..."). At the 30th
 * pose perf "bridge_affinity" / "app_affinity" = Cpus_allowed_list of every bridge / app perf thread (/proc/<pid>/task).
 *
 * 0.6.15 knobs (default = 0.6.14 behaviour): intent ov_threads N (> 0) = ov_bridge --threads N = OpenCV thread pool size
 * (overrides the config num_opencv_threads, 2 in the phone configs; remap / KLT pyramid / LK / FAST grid run on it);
 * intent klt_overlap 1 = ov_bridge --klt-overlap 1 (patched TrackKLT: detection overlapped with the pyramid; same output).
 * meta.json "ov_threads" / "klt_overlap" (requested) and perf "bridge_cv_threads" / "bridge_cv_parallel" /
 * "bridge_klt_overlap" (what the bridge reports in its ready line: cv::getNumThreads and the OpenCV parallel framework).
 *
 * 0.6.16 (default = 0.6.15 behaviour): intent idle_spin 0|1|2 = ov_bridge --idle-spin (0 = block in read, 1 = bridge main
 * thread busy-polls its stdin between records, 2 = 1 + the whole OpenCV/TBB pool spins; keeps the cores clocked up between
 * frames, see scan3d/ov_bridge/main.cpp). meta.json "ov_idle_spin", perf "ov_idle_spin" (requested) / "bridge_idle_spin" /
 * "bridge_idle_spin_stripes" (ready line). Latency logging (timing only, same clock elapsedRealtimeNanos = CLOCK_BOOTTIME):
 * frames.csv + copy_done_ns (Y plane copied into the record) / enq_ns (record queued for the writer, 0 = dropped);
 * writes.csv (ov-writer, image records) t_send_ns,deq_ns,write_done_ns (taken from the queue / written into the pipe);
 * bridge_timing.csv rx_hdr_boot_ns / rx_boot_ns (bridge read the record). NOTE frames.csv / writes.csv "t_send_ns" is the
 * frame time sent to the bridge = SENSOR_TIMESTAMP + midOffsetNs (mid exposure, sensor clock), NOT a send time (name kept
 * for the tools).
 *
 * 0.6.17 (default = 0.6.16 behaviour): intent predict 0|1|2 = ov_bridge --predict (IMU forward-predicted "now" pose:
 * after every frame update the bridge integrates the newest IMU samples from the filter state to the newest IMU time;
 * 2 = also between frames at intent predict_hz, default 100; the filter and the normal pose lines are unchanged).
 * The bridge "pred" lines go to poses_pred.csv (NOT events.jsonl): t_pred_ns = predicted instant on the IMU / sensor
 * clock, t_cam_ns / t_sensor_ns = the frame the prediction starts from (bridge t_ns / SENSOR_TIMESTAMP), t_out_ns = line
 * read (elapsedRealtimeNanos) -> latency of the predicted pose = t_out_ns - t_pred_ns, horizon = h_ms. Intent
 * predict_stream 1 also sends them to the Streamer ({"src":"ov_pred",...}; default 0 = stream unchanged).
 * meta.json "ov_predict" / "ov_predict_hz" / "predict_lines", perf "ov_predict" / "bridge_predict" (ready line).
 *
 * 0.6.18 (branch ovopt-20261025; every default = 0.6.17 behaviour): intent ov_opt "LIST" = ov_bridge --ovopt LIST (speed
 * options of the patched OpenVINS, e.g. "all_exact" = skip_retri,par_tri,par_jac,par_ma,par_s,cov_move,frame_nocopy,givens_rm:
 * poses identical on the PC checks, phone check pending; see scan3d/ov_bridge/main.cpp); intent ov_prof 1 = ov_bridge
 * --prof-log <run>/ov_prof.csv (per-frame
 * OpenVINS stage times); intent idle_spin_ms W = ov_bridge --idle-spin-ms W (with idle_spin > 0: spin only W ms before
 * the expected next frame); intent spin_wfe 1 = with idle_spin 2 pass --idle-spin 3 (pool waits with WFE between input
 * polls instead of a yield loop); intent drop_late_ms N = ov_bridge --drop-late N (after init, skip a frame fed more than
 * N ms later than the best lag so far: no backlog spiral; changes results only when it drops). meta.json + perf "ov_opt" /
 * "ov_prof" / "idle_spin_ms" / "spin_wfe" / "drop_late_ms".
 *
 * 0.6.19 (branch ovopt-20261025; every default = 0.6.18 behaviour): intent imu_prio 1 = IMU records never wait behind an
 * image write: IMU go to their own queue, the writer sends all waiting IMU first, then the image as 64 KB chunk records
 * (types 5/6, scan3d/ov_bridge/main.cpp header) with the IMU that arrived meanwhile written between the chunks, flushing
 * after every IMU batch. An IMU sample is sent ahead of an image already queued in the app only if t_imu <= t_image +
 * 200 ms (the bridge holds IMU <= 250 ms waiting for a camera, so the bridge feeds OpenVINS the identical sequence; PC
 * check with reordered streams). Not covered (same as 0.6.18): an image that reaches onImage > ~250 ms after exposure.
 * Benefit needs a bridge that reads while it processes a frame (pred_thread with predict 2); the bridge reader stops
 * reading (IMU too) while 2 images are unprocessed (backlog). intent pred_thread (default 1, used with predict 2) = ov_bridge --pred-thread 1 (input reader + prediction
 * thread: predicted poses also while a frame is processed); pred_thread 0 = the 0.6.18 predict 2. intent imu_log 1
 * (timing only): imu_samples.csv (every sensor event: type, event.timestamp, elapsedRealtimeNanos in onSensorChanged; ring
 * of the last 2^19), imu_enq.csv (IMU record t_ns, queued), imu_writes.csv (t_ns, writer dequeued, write/flush call that
 * pushed it into the pipe returned), ov_bridge --imu-log bridge_imu.csv (t_ns, header / payload read, CLOCK_BOOTTIME).
 * meta.json + perf "imu_prio" / "pred_thread" / "imu_log", meta "chunked_images".
 */
class OvRunner implements SensorEventListener {
    interface Status { void show(String s); }

    static final String VERSION = "ov-20261025-imuprio";
    private static final int MAGIC = 0x4F564252;
    // settings (intent extras, MainActivity)
    String cam = "lens2";
    int W = 640, H = 480;
    int fps = 30;
    int expUs = 0, iso = 0;                     // main: either > 0 -> AE off, manual exposure
    boolean autoIso = true;                     // 0.6.11 intent auto_iso (main): ISO from AE once at start, then locked (AutoIso)
    double isoScale = 1.0;                      // intent iso_target_scale: brightness bias for the auto ISO
    private AutoIso autoRes;                    // AE metering result (null = not used)
    float focusDpt = -1f;                       // main: >= 0 fixed focus (AF off)
    String cfgName = "";                        // config subdir of files/ov ("" = files/ov itself)
    String preset = "live";                     // bridge args: "live" (scan3d live defaults) | "bench" (vio_bench_hs.py)
    boolean imuRaw = false;                     // uncalibrated gyro/accel (types 16/35, values 0..2)
    boolean ovTiming = true;                    // OpenVINS record_timing_information -> run dir/ov_timing.txt
    int maxBatteryTenthC = 460;                 // stop at this battery temperature (backstop; the PC loop watches SKIN); 0.6.8: 42 -> 46 C
    int maxThermal = android.os.PowerManager.THERMAL_STATUS_CRITICAL;   // 0.6.8: was SEVERE (3)
    int poolSize = 4;                           // image buffers in flight (more frames waiting -> dropped, counted)
    Runnable onAutoStop;                        // called (UI side finishes the activity) when a stop rule fires
    Runnable onFeedStart;                       // 0.6.12: called once (cam thread) when the first frame goes to the bridge (after auto ISO)
    private volatile boolean fed;
    volatile String stopReason = "user";

    private final Context ctx;
    private final Status status;
    private final Streamer out;
    private HandlerThread camThread, imuThread;
    private CameraDevice dev;
    private ImageReader reader;
    private Process proc;
    private Thread writerT, readerT;
    private volatile boolean running;
    private final LinkedBlockingQueue<byte[]> toBridge = new LinkedBlockingQueue<>(4096);
    private ArrayBlockingQueue<byte[]> pool;
    /** t_ns sent to the bridge (mid exposure) -> {t_sensor, t_arrive} */
    private final ConcurrentHashMap<Long, long[]> sent = new ConcurrentHashMap<>();
    private volatile long midOffsetNs;          // (exposure + rolling shutter skew) / 2 from the latest capture result
    private volatile long expNs;                // exposure of the latest capture result (motion blur check)
    /** lens2: exposure cap in ns (0 = AE): after 15 AE frames switch to a fixed short exposure with ISO raised to keep the
     *  brightness (ISO capped at the lens max -> shortest exposure that still keeps it). Less blur = fewer lost features. */
    long expCapNs;
    private double meterEv, meterExp; private int meterN, meterFrames;
    private volatile boolean manualSet;
    private CameraCaptureSession sess;
    private CaptureRequest.Builder reqB;
    private volatile int nCam, nPose, nImu, nAcc, camDropped, poolDropped, nResults;
    private volatile float lastFocus = -1;
    private String lastStatus = "-";
    private File dir, runDir;
    private BufferedWriter framesW, posesW, tempsW, eventsW;
    private BufferedWriter writesW;             // 0.6.16 writes.csv: written and closed by the ov-writer thread only
    private final Object framesLock = new Object(), posesLock = new Object();
    private final JSONObject meta = new JSONObject();
    private long t0Elapsed;
    private Handler camH;
    private String ldcHow = "not requested";
    private CaptureRequest.Key<Integer> ldcKey;
    private int accType;

    // ---- 0.6.13 performance hints (class doc) -> meta.json "perf"
    boolean perfOn = true;                      // intent perf (default 1)
    int hintMs = 0;                             // 0.6.14 intent hint_ms: hint target in ms (0 = 1000/fps)
    String cpus = "";                           // 0.6.14 intent cpus: ov_bridge --cpus list ("" = no pinning)
    int ovThreads = 0;                          // 0.6.15 intent ov_threads: ov_bridge --threads (0 = config num_opencv_threads)
    boolean kltOverlap = false;                 // 0.6.15 intent klt_overlap: ov_bridge --klt-overlap 1
    int idleSpin = 0;                           // 0.6.16 intent idle_spin: ov_bridge --idle-spin 0|1|2 (0 = off)
    int predict = 0;                            // 0.6.17 intent predict: ov_bridge --predict 0|1|2 (0 = off)
    int predictHz = 100;                        // 0.6.17 intent predict_hz: ov_bridge --predict-hz (mode 2)
    boolean predictStream = false;              // 0.6.17 intent predict_stream: also send "pred" lines to the Streamer
    String ovOpt = "";                          // 0.6.18 intent ov_opt: ov_bridge --ovopt LIST ("" = off)
    boolean ovProf = false;                     // 0.6.18 intent ov_prof 1: ov_bridge --prof-log <run>/ov_prof.csv
    int idleSpinMs = 0;                         // 0.6.18 intent idle_spin_ms: ov_bridge --idle-spin-ms W (0 = whole gap)
    boolean spinWfe = false;                    // 0.6.18 intent spin_wfe 1: idle_spin 2 -> ov_bridge --idle-spin 3
    int dropLateMs = 0;                         // 0.6.18 intent drop_late_ms: ov_bridge --drop-late MS (0 = off)
    int imuPrio = 0;                            // 0.6.19 intent imu_prio: 1 = IMU never waits behind an image write (class doc)
    int predThread = 1;                         // 0.6.19 intent pred_thread: with predict 2 -> ov_bridge --pred-thread 1
    boolean imuLog = false;                     // 0.6.19 intent imu_log: IMU delivery logs (class doc)
    private final LinkedBlockingQueue<byte[]> imuQ = new LinkedBlockingQueue<>(65536);   // imu_prio: IMU records
    private final java.util.concurrent.Semaphore wake = new java.util.concurrent.Semaphore(0);   // imu_prio: writer wake-up
    static final int CHUNK = 65536;             // imu_prio: image chunk payload bytes
    static final long IMU_LEAD_NS = 200_000_000L;   // imu_prio: IMU may go ahead of a pending image up to t_image + this
    private long nChunkImg, nChunks;
    static final int RING = 1 << 19;            // imu_log ring sizes (entries)
    private long[] sRing, eRing, wRing;         // samples {type, t_event, t_arrive}, enq {t, enq}, writes {t, deq, done}
    private long sN, eN, wN;
    private long[] wPend = new long[4096];      // imu_log: ring slots of IMU written but not yet pushed by a write/flush
    private int wPendN;
    // ovdrone (2026-10-25, branch ovdrone-20261025; default off = unchanged): intent mav_udp "HOST:PORT" = ov_bridge
    // --mav-udp (MAVLink 2 ODOMETRY of every predicted pose over UDP + 1 Hz HEARTBEAT + TIMESYNC replies; needs intent
    // predict 1|2, see scan3d/ov_bridge/mav_out.h), intent mav_mount (default "imu") / mav_msg (default "odom"); the bridge
    // also writes <run>/mav_log.csv. meta.json "mav_udp" / "mav_mount" / "mav_msg".
    String mavUdp = "";
    String mavMount = "imu";
    String mavMsg = "odom";
    private BufferedWriter predW;               // poses_pred.csv (predict > 0)
    private long nPred;
    private long lastPoseTns = -1, lastPoseSensor = -1, lastPoseArrive = -1;   // frame of the latest pose line (reader)
    static final int PRIO = android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY;   // -8
    private final JSONObject perf = new JSONObject();
    private int bridgePid = -1;
    private volatile Object hint;               // android.os.PerformanceHintManager.Session (API 31+)
    private boolean hintHasBridge;
    private int perfPoses, hintReports, hintErrors;
    private final java.util.Set<Integer> appTids = java.util.Collections.newSetFromMap(new ConcurrentHashMap<Integer, Boolean>());
    private final java.util.Set<Integer> bridgeTids = new java.util.HashSet<>();

    void notePerf(String k, Object v) { synchronized (perf) { try { perf.put(k, v); } catch (Exception ignored) { } } }

    /** Raise the calling thread's priority (perfOn) and log the value read back. */
    private void boostSelf(String name) {
        int tid = android.os.Process.myTid();
        if (!perfOn) { notePerf("prio_" + name, "tid " + tid + " off -> " + android.os.Process.getThreadPriority(tid)); return; }
        String r;
        try { android.os.Process.setThreadPriority(PRIO); r = "ok"; } catch (Throwable e) { r = e.toString(); }
        notePerf("prio_" + name, "tid " + tid + " set " + PRIO + " " + r + " -> " + android.os.Process.getThreadPriority(tid));
        appTids.add(tid);
    }

    private static int pidOf(Process p) {
        java.util.regex.Matcher m = java.util.regex.Pattern.compile("pid=(\\d+)").matcher(String.valueOf(p));   // Android: Process[pid=N, ...]
        if (m.find()) return Integer.parseInt(m.group(1));
        try { java.lang.reflect.Field f = p.getClass().getDeclaredField("pid"); f.setAccessible(true); return f.getInt(p); }
        catch (Throwable e) { return -1; }
    }

    private static List<Integer> tidsOf(int pid) {
        List<Integer> l = new ArrayList<>();
        String[] ts = pid > 0 ? new File("/proc/" + pid + "/task").list() : null;
        if (ts != null) for (String t : ts) try { l.add(Integer.parseInt(t)); } catch (Exception ignored) { }
        return l;
    }

    /** "tid:comm=cpus ..." from /proc/<pid>/task/<tid>/status (Name, Cpus_allowed_list). */
    private static String affinityOf(int pid, java.util.Collection<Integer> tids) {
        StringBuilder sb = new StringBuilder();
        for (int t : tids) {
            String name = "?", cpu = "?";
            try (BufferedReader r = new BufferedReader(new java.io.FileReader("/proc/" + pid + "/task/" + t + "/status"))) {
                for (String l; (l = r.readLine()) != null; ) {
                    if (l.startsWith("Name:")) name = l.substring(5).trim();
                    else if (l.startsWith("Cpus_allowed_list:")) { cpu = l.substring(18).trim(); break; }
                }
            } catch (Exception e) { cpu = "err"; }
            sb.append(t).append(':').append(name).append('=').append(cpu).append(' ');
        }
        return sb.toString().trim();
    }

    private static int[] toArray(java.util.Collection<Integer> a, java.util.Collection<Integer> b) {
        List<Integer> l = new ArrayList<>(a);
        if (b != null) l.addAll(b);
        int[] r = new int[l.size()];
        for (int i = 0; i < r.length; i++) r[i] = l.get(i);
        return r;
    }

    /** reader thread, every pose: hint setup at pose 30 (refresh at 300), report the bridge's per-frame time. */
    private void perfOnPose(double ms) {
        if (!perfOn) return;
        perfPoses++;
        if (perfPoses == 30 || perfPoses == 300) setupBridgePerf(perfPoses == 300);
        Object h = hint;
        if (h != null && ms > 0 && android.os.Build.VERSION.SDK_INT >= 31) {
            try { ((android.os.PerformanceHintManager.Session) h).reportActualWorkDuration((long) (ms * 1e6)); hintReports++; }
            catch (Throwable e) { if (hintErrors++ == 0) notePerf("hint_report_error", e.toString()); }
        }
    }

    private void setupBridgePerf(boolean refresh) {
        List<Integer> bt = tidsOf(bridgePid);
        int ok = 0, fail = 0, added = 0;
        String err = null;
        StringBuilder pr = new StringBuilder();
        for (int t : bt) {
            if (bridgeTids.contains(t)) continue;
            added++;
            try { android.os.Process.setThreadPriority(t, PRIO); ok++; } catch (Throwable e) { fail++; if (err == null) err = e.toString(); }
            try { pr.append(t).append(':').append(android.os.Process.getThreadPriority(t)).append(' '); } catch (Throwable ignored) { }
            bridgeTids.add(t);
        }
        notePerf(refresh ? "bridge_prio_refresh" : "bridge_prio", "pid " + bridgePid + " threads " + bt.size() + " new " + added
                + " set ok " + ok + " fail " + fail + (err == null ? "" : " (" + err + ")") + " read back " + pr.toString().trim());
        if (!refresh) {   // 0.6.14: allowed cpus per thread (bridge --cpus check; app threads are not pinned)
            notePerf("bridge_affinity", affinityOf(bridgePid, bt));
            notePerf("app_affinity", affinityOf(android.os.Process.myPid(), appTids));
        }
        if (android.os.Build.VERSION.SDK_INT < 31) { notePerf("hint", "API " + android.os.Build.VERSION.SDK_INT + " < 31"); return; }
        try {
            if (!refresh) {
                android.os.PerformanceHintManager pm = ctx.getSystemService(android.os.PerformanceHintManager.class);
                if (pm == null) { notePerf("hint", "no PerformanceHintManager"); return; }
                notePerf("hint_preferred_rate_ns", pm.getPreferredUpdateRateNanos());
                long target = hintMs > 0 ? hintMs * 1_000_000L : 1_000_000_000L / fps;
                int[] all = toArray(appTids, bridgeTids);
                Object s = null;
                String how;
                try {
                    s = pm.createHintSession(all, target);
                    how = s == null ? "app+bridge: null (not supported)" : "app+bridge " + all.length + " tids";
                    hintHasBridge = s != null;
                } catch (Throwable e) { how = "app+bridge failed: " + e; }
                if (s == null) {
                    int[] app = toArray(appTids, null);
                    try { s = pm.createHintSession(app, target); how += s == null ? "; app only: null" : "; app only " + app.length + " tids"; }
                    catch (Throwable e) { how += "; app only failed: " + e; }
                }
                hint = s;
                notePerf("hint", how);
                notePerf("hint_target_ns", target);
                notePerf("hint_tids", java.util.Arrays.toString(all) + " (app " + appTids + ")");
            } else if (hint != null && hintHasBridge && added > 0) {
                if (android.os.Build.VERSION.SDK_INT >= 34) {
                    int[] all = toArray(appTids, bridgeTids);
                    ((android.os.PerformanceHintManager.Session) hint).setThreads(all);
                    notePerf("hint_refresh", "setThreads " + all.length + " tids");
                } else notePerf("hint_refresh", added + " new bridge threads not added (setThreads needs API 34)");
            }
        } catch (Throwable e) { notePerf(refresh ? "hint_refresh" : "hint", "error " + e); }
    }

    OvRunner(Context ctx, Status status, Streamer out) { this.ctx = ctx; this.status = status; this.out = out; }

    boolean isMain() { return "main".equals(cam); }

    void start() throws Exception {
        running = true;
        t0Elapsed = SystemClock.elapsedRealtimeNanos();
        dir = new File(ctx.getExternalFilesDir(null), "ov");
        File cfgDir = cfgName.isEmpty() ? dir : new File(dir, cfgName);
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
        runDir = new File(dir, "run_" + stamp + "_" + cam + fps + (cfgName.isEmpty() ? "" : "_" + cfgName));
        if (!runDir.mkdirs()) throw new IllegalStateException("cannot create " + runDir);
        File cfg = prepareConfig(cfgDir);
        String remapName = isMain() ? "remap_" + W + "x" + H + ".bin" : "lens0u_remap.bin";
        File remap = new File(cfgDir, remapName);
        if (!remap.exists()) remap = new File(dir, remapName);
        if (!remap.exists()) throw new IllegalStateException("no " + remapName + " in " + cfgDir + " or " + dir);

        String lib = ctx.getApplicationInfo().nativeLibraryDir;
        List<String> cmd = new ArrayList<>();
        cmd.add(lib + "/libov_bridge.so");
        cmd.add(cfg.getAbsolutePath());
        cmd.add("--stdio");
        if ("bench".equals(preset)) {           // = vio_bench_hs.py (baseline runs): no auto reset, never "lost"
            Collections.addAll(cmd, "--max-speed", "8", "--lost-frames", "1000000", "--startup-lost-frames", "1000000");
        } else {
            Collections.addAll(cmd, "--auto-reset", "--reinit-state", "--init-gate", "0.006,0.15,0.8", "--max-cam-queue", "2");
        }
        Collections.addAll(cmd, "--remap", remap.getAbsolutePath(), "--timing-log", new File(runDir, "bridge_timing.csv").getAbsolutePath());
        if (!cpus.isEmpty()) {
            if (!cpus.matches("[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*")) throw new IllegalArgumentException("bad cpus " + cpus);
            Collections.addAll(cmd, "--cpus", cpus);
        }
        if (ovThreads > 0) Collections.addAll(cmd, "--threads", Integer.toString(ovThreads));
        if (kltOverlap) Collections.addAll(cmd, "--klt-overlap", "1");
        if (idleSpin > 0) Collections.addAll(cmd, "--idle-spin", Integer.toString(idleSpin == 2 && spinWfe ? 3 : idleSpin));
        if (idleSpin > 0 && idleSpinMs > 0) Collections.addAll(cmd, "--idle-spin-ms", Integer.toString(idleSpinMs));   // 0.6.18
        if (predict > 0) Collections.addAll(cmd, "--predict", Integer.toString(predict), "--predict-hz", Integer.toString(predictHz));
        if (!ovOpt.isEmpty()) {                                                                                        // 0.6.18
            if (!ovOpt.matches("[a-z_]+(=[0-9]+)?(,[a-z_]+(=[0-9]+)?)*")) throw new IllegalArgumentException("bad ov_opt " + ovOpt);
            Collections.addAll(cmd, "--ovopt", ovOpt);
        }
        if (ovProf) Collections.addAll(cmd, "--prof-log", new File(runDir, "ov_prof.csv").getAbsolutePath());          // 0.6.18
        if (dropLateMs > 0) Collections.addAll(cmd, "--drop-late", Integer.toString(dropLateMs));                       // 0.6.18
        if (predict == 2 && predThread > 0) Collections.addAll(cmd, "--pred-thread", "1");                            // 0.6.19
        if (imuLog) {                                                                                                   // 0.6.19
            Collections.addAll(cmd, "--imu-log", new File(runDir, "bridge_imu.csv").getAbsolutePath());
            sRing = new long[3 * RING]; eRing = new long[2 * RING]; wRing = new long[3 * RING];
        }
        if (!mavUdp.isEmpty()) {                                                                                       // ovdrone
            if (!mavUdp.matches("[0-9.]+:[0-9]+")) throw new IllegalArgumentException("bad mav_udp " + mavUdp);
            if (!mavMount.matches("[a-z_]+|[-0-9.eE]+(,[-0-9.eE]+){8}")) throw new IllegalArgumentException("bad mav_mount " + mavMount);
            if (!mavMsg.matches("odom|vpe|both")) throw new IllegalArgumentException("bad mav_msg " + mavMsg);
            if (predict < 1) throw new IllegalArgumentException("mav_udp needs predict 1 or 2");
            Collections.addAll(cmd, "--mav-udp", mavUdp, "--mav-mount", mavMount, "--mav-msg", mavMsg,
                    "--mav-log", new File(runDir, "mav_log.csv").getAbsolutePath());
        }
        ProcessBuilder pb = new ProcessBuilder(cmd);
        pb.environment().put("LD_LIBRARY_PATH", lib);
        pb.redirectError(new File(runDir, "bridge_stderr.log"));

        framesW = new BufferedWriter(new FileWriter(new File(runDir, "frames.csv")), 1 << 16);
        // fate 0 sent, 1 pool empty, 2 queue full. t_send_ns = frame time sent to the bridge (SENSOR_TIMESTAMP + midOffsetNs =
        // mid exposure on the sensor clock), not a send time (misnamed, kept for the tools). 0.6.16: + copy_done_ns, enq_ns.
        framesW.write("idx,t_sensor_ns,t_arrive_ns,t_send_ns,copy_us,fate,exp_ns,focus_dpt,copy_done_ns,enq_ns\n");
        writesW = new BufferedWriter(new FileWriter(new File(runDir, "writes.csv")), 1 << 16);   // 0.6.16, ov-writer only
        writesW.write("t_send_ns,deq_ns,write_done_ns\n");
        posesW = new BufferedWriter(new FileWriter(new File(runDir, "poses.csv")), 1 << 16);
        posesW.write("t_ns,t_sensor_ns,t_arrive_ns,t_out_ns,status,ms,remap_ms,n_slam,n_msckf,cam_dropped_bridge,pool_dropped,queue_dropped,n_cam,n_imu,dt_cam_imu\n");
        if (predict > 0) {   // 0.6.17: bridge "pred" lines (class doc)
            predW = new BufferedWriter(new FileWriter(new File(runDir, "poses_pred.csv")), 1 << 16);
            predW.write("t_pred_ns,t_cam_ns,t_sensor_ns,t_arrive_ns,t_out_ns,src,status,h_ms,n_imu,px,py,pz,qx,qy,qz,qw,vx,vy,vz\n");
        }
        tempsW = new BufferedWriter(new FileWriter(new File(runDir, "temps.csv")));
        tempsW.write("elapsed_ns,t_rel_s,battery_c,thermal_status,headroom_10s\n");
        eventsW = new BufferedWriter(new FileWriter(new File(runDir, "events.jsonl")));
        meta.put("app_version", VERSION).put("cam", cam).put("size", W + "x" + H).put("fps", fps).put("exp_us", expUs).put("iso", iso)
                .put("focus_dpt", focusDpt).put("cfg", cfgName).put("cfg_dir", cfgDir.getAbsolutePath()).put("preset", preset)
                .put("remap", remap.getAbsolutePath()).put("bridge_cmd", String.join(" ", cmd)).put("ov_timing", ovTiming)
                .put("pool", poolSize).put("start_elapsed_ns", t0Elapsed).put("start_wall", stamp)
                .put("battery_c_start", batteryTenthC() / 10.0).put("thermal_status_start", thermalStatus())
                .put("device", android.os.Build.MODEL).put("ov_threads", ovThreads).put("klt_overlap", kltOverlap ? 1 : 0)
                .put("ov_idle_spin", idleSpin).put("ov_predict", predict).put("ov_predict_hz", predictHz)
                .put("ov_predict_stream", predictStream ? 1 : 0)
                .put("ov_opt", ovOpt).put("ov_prof", ovProf ? 1 : 0).put("idle_spin_ms", idleSpinMs).put("spin_wfe", spinWfe ? 1 : 0)
                .put("drop_late_ms", dropLateMs).put("mav_udp", mavUdp).put("mav_mount", mavMount).put("mav_msg", mavMsg)
                .put("imu_prio", imuPrio).put("pred_thread", predict == 2 && predThread > 0 ? 1 : 0).put("imu_log", imuLog ? 1 : 0);

        proc = pb.start();
        bridgePid = pidOf(proc);
        notePerf("perf_on", perfOn);
        notePerf("hint_ms", hintMs);
        notePerf("cpus", cpus.isEmpty() ? "off" : cpus);
        notePerf("ov_threads", ovThreads > 0 ? Integer.toString(ovThreads) : "config");
        notePerf("klt_overlap", kltOverlap ? 1 : 0);
        notePerf("ov_idle_spin", idleSpin);
        notePerf("ov_predict", predict);
        notePerf("ov_predict_hz", predictHz);
        notePerf("ov_opt", ovOpt);                 // 0.6.18
        notePerf("ov_prof", ovProf ? 1 : 0);
        notePerf("idle_spin_ms", idleSpinMs);
        notePerf("spin_wfe", spinWfe ? 1 : 0);
        notePerf("drop_late_ms", dropLateMs);
        notePerf("imu_prio", imuPrio);             // 0.6.19
        notePerf("pred_thread", predict == 2 && predThread > 0 ? 1 : 0);
        notePerf("imu_log", imuLog ? 1 : 0);
        notePerf("bridge_pid", bridgePid);
        notePerf("sdk", android.os.Build.VERSION.SDK_INT);
        writerT = new Thread(this::writer, "ov-writer");
        readerT = new Thread(this::readerLoop, "ov-reader");
        writerT.start();
        readerT.start();

        imuThread = new HandlerThread("imu");
        imuThread.start();
        SensorManager sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        Handler ih = new Handler(imuThread.getLooper());
        ih.post(() -> boostSelf("imu"));
        Sensor acc = null, gyr = null;
        if (imuRaw) {
            acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
            gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
            if (acc == null || gyr == null) { acc = null; gyr = null; status.show("no uncalibrated IMU -> calibrated"); }
        }
        boolean raw = acc != null;
        if (!raw) { acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE); }
        accType = acc.getType();
        meta.put("imu", raw ? "uncalibrated types 35/16 (values 0..2)" : "calibrated types 1/4");
        sm.registerListener(this, acc, SensorManager.SENSOR_DELAY_FASTEST, ih);
        sm.registerListener(this, gyr, SensorManager.SENSOR_DELAY_FASTEST, ih);

        camThread = new HandlerThread("cam");
        camThread.start();
        final Handler h = new Handler(camThread.getLooper());
        camH = h;
        h.post(() -> boostSelf("cam"));
        pool = new ArrayBlockingQueue<>(poolSize);
        for (int i = 0; i < poolSize; i++) pool.add(new byte[12 + 20 + W * H]);
        CameraManager cm = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        final CameraCharacteristics ch = cm.getCameraCharacteristics(isMain() ? "0" : "2");
        streamOffer(ch);
        reader = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4);
        reader.setOnImageAvailableListener(this::onImage, h);
        cm.openCamera("0", new CameraDevice.StateCallback() {
            @Override public void onOpened(CameraDevice d) {
                if (!running) { d.close(); return; }      // stopped before the camera opened
                dev = d;
                if (autoIsoActive()) {      // 0.6.11: AE metering in a throw-away session; no AE-phase frame reaches the bridge
                    status.show("OV main auto ISO: AE metering...");
                    AutoIso a = new AutoIso();
                    a.size = new Size(W, H); a.fps = new Range<>(fps, fps); a.focusDpt = focusDpt; a.expNs = mainExpNs(ch); a.scale = isoScale;
                    autoRes = a;
                    a.run(d, ch, h, x -> {
                        try { meta.put("auto_iso", x.json()); } catch (Exception ignored) { }
                        if (running) openSession(d, ch, h);
                    });
                } else {
                    try { meta.put("auto_iso", new JSONObject().put("auto_iso", false).put("why", !isMain() ? "lens2" : !autoIso ? "auto_iso=0"
                            : iso > 0 ? "explicit iso" : "exp_us=0 (AE)")); } catch (Exception ignored) { }
                    openSession(d, ch, h);
                }
            }
            @Override public void onDisconnected(CameraDevice d) { d.close(); }
            @Override public void onError(CameraDevice d, int e) { status.show("camera error " + e); d.close(); }
        }, h);
    }

    /** main + auto_iso (default on) + exp_us > 0 + no explicit iso: ISO from the camera AE once at start (AutoIso). */
    /** 0.6.12: on-screen guide (MainActivity / OvGuide) -> meta.json; its countdown starts at first_fed_elapsed_ns. */
    void noteGuide(String spec, int secs) {
        try { meta.put("guide", spec).put("guide_secs", secs).put("guide_t0", "first_fed_elapsed_ns"); } catch (Exception ignored) { }
    }

    boolean autoIsoActive() { return isMain() && autoIso && iso <= 0 && expUs > 0; }

    /** main manual exposure in ns: exp_us capped at the frame duration, clamped to the sensor range. */
    private long mainExpNs(CameraCharacteristics ch) {
        long frameDur = 1_000_000_000L / fps;
        long e = expUs > 0 ? Math.min(expUs * 1000L, frameDur) : frameDur / 2;
        Range<Long> er = ch.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
        return er != null ? er.clamp(e) : e;
    }

    /** The real session (YUV reader -> bridge) and its repeating request; after AutoIso when that is active. */
    private void openSession(final CameraDevice d, final CameraCharacteristics ch, final Handler h) {
                try {
                    OutputConfiguration oc = new OutputConfiguration(reader.getSurface());
                    if (!isMain()) oc.setPhysicalCameraId("2");
                    final CaptureRequest.Builder rb = d.createCaptureRequest(isMain() ? CameraDevice.TEMPLATE_RECORD : CameraDevice.TEMPLATE_PREVIEW);
                    rb.addTarget(reader.getSurface());
                    rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(fps, fps));
                    rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                    // tracking-friendly image processing (same as the recorder 1.15-isp): no noise reduction / sharpening
                    try {
                        if (has(ch.get(CameraCharacteristics.NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES), CaptureRequest.NOISE_REDUCTION_MODE_OFF))
                            rb.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
                        if (has(ch.get(CameraCharacteristics.EDGE_AVAILABLE_EDGE_MODES), CaptureRequest.EDGE_MODE_OFF))
                            rb.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
                        if (isMain()) mainRequest(rb, ch);
                        else {
                            if (has(ch.get(CameraCharacteristics.TONEMAP_AVAILABLE_TONE_MAP_MODES), CaptureRequest.TONEMAP_MODE_GAMMA_VALUE)) {
                                rb.set(CaptureRequest.TONEMAP_MODE, CaptureRequest.TONEMAP_MODE_GAMMA_VALUE);
                                rb.set(CaptureRequest.TONEMAP_GAMMA, 2.2f);
                            }
                            Integer dc = rb.get(CaptureRequest.DISTORTION_CORRECTION_MODE);
                            if (dc != null) rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, dc);
                        }
                    } catch (Exception ex) { android.util.Log.w("arlat", "isp", ex); }
                    final CameraCaptureSession.CaptureCallback cb = new CameraCaptureSession.CaptureCallback() {
                        @Override public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest q, TotalCaptureResult res) {
                            Long exp = res.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                            Long skew = res.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                            Float fd = res.get(CaptureResult.LENS_FOCUS_DISTANCE);
                            midOffsetNs = ((exp == null ? 0 : exp) + (skew == null ? 0 : skew)) / 2;
                            expNs = exp == null ? 0 : exp;
                            if (fd != null) lastFocus = fd;
                            if (nResults++ == 0) try {
                                meta.put("first_result", "exp_ns " + exp + " skew_ns " + skew + " iso " + res.get(CaptureResult.SENSOR_SENSITIVITY)
                                        + " focus " + fd + " frame_duration_ns " + res.get(CaptureResult.SENSOR_FRAME_DURATION)
                                        + " crop " + HsRecorder.cropRegion(res) + " dc " + res.get(CaptureResult.DISTORTION_CORRECTION_MODE)
                                        + " ois " + res.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE));
                            } catch (Exception ignored) { }
                            if (!isMain() && expCapNs > 0 && !manualSet) meter(res, h);
                        }
                    };
                    d.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,
                            Collections.singletonList(oc), ctx.getMainExecutor(), new CameraCaptureSession.StateCallback() {
                                @Override public void onConfigured(CameraCaptureSession s) {
                                    sess = s; reqB = rb; capCb = cb;
                                    try { s.setRepeatingRequest(rb.build(), cb, h); }
                                    catch (Exception e) {
                                        if (ldcKey == null) { status.show("repeat: " + e); return; }
                                        try {                     // never lose the run to the Samsung vendor tag: retry without it
                                            rb.set(ldcKey, null); ldcKey = null; ldcHow += " -> request failed, removed";
                                            s.setRepeatingRequest(rb.build(), cb, h);
                                        } catch (Exception e2) { status.show("repeat: " + e2); }
                                    }
                                    try { meta.put("samsung_ldc_request", ldcHow); } catch (Exception ignored) { }
                                    h.postDelayed(heatTick, 1000);
                                }
                                @Override public void onConfigureFailed(CameraCaptureSession s) { status.show("configure failed"); }
                            }));
                } catch (Exception e) { status.show("session: " + e); }
    }

    /** Copy files/ov/<cfg>/estimator_config.yaml (+ its kalibr chains) into the run dir with the OpenVINS output paths
     *  pointing there and record_timing_information = ovTiming. */
    private File prepareConfig(File cfgDir) throws Exception {
        File src = new File(cfgDir, "estimator_config.yaml");
        String txt = new String(java.nio.file.Files.readAllBytes(src.toPath()), StandardCharsets.UTF_8);
        StringBuilder sb = new StringBuilder();
        List<String> rel = new ArrayList<>();
        String rd = runDir.getAbsolutePath();
        for (String line : txt.split("\n", -1)) {
            String t = line.trim();
            if (t.startsWith("record_timing_information:")) line = "record_timing_information: " + ovTiming;
            else if (t.startsWith("record_timing_filepath:")) line = "record_timing_filepath: \"" + rd + "/ov_timing.txt\"";
            else if (t.startsWith("filepath_est:")) line = "filepath_est: \"" + rd + "/ov_estimate.txt\"";
            else if (t.startsWith("filepath_std:")) line = "filepath_std: \"" + rd + "/ov_estimate_std.txt\"";
            else if (t.startsWith("filepath_gt:")) line = "filepath_gt: \"" + rd + "/ov_groundtruth.txt\"";
            else if (t.startsWith("relative_config_imu:") || t.startsWith("relative_config_imucam:")) {
                rel.add(t.split(":", 2)[1].split("#")[0].trim().replace("\"", ""));
            }
            sb.append(line).append('\n');
        }
        File dst = new File(runDir, "estimator_config.yaml");
        java.nio.file.Files.write(dst.toPath(), sb.toString().getBytes(StandardCharsets.UTF_8));
        for (String r : rel) java.nio.file.Files.copy(new File(cfgDir, r).toPath(), new File(runDir, r).toPath());
        return dst;
    }

    /** main camera request = HsRecorder session std: OIS off, distortion correction off (AOSP + Samsung tag), fixed focus,
     *  manual exposure. */
    private void mainRequest(CaptureRequest.Builder rb, CameraCharacteristics ch) throws Exception {
        rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
        int[] dm = ch.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
        if (has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF))
            rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
        meta.put("aosp_distortion_request", has(dm, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF) ? "OFF" : "not offered");
        try {
            CaptureRequest.Key<Integer> k = new CaptureRequest.Key<>(HsRecorder.LDC_TAG, Integer.class);
            rb.set(k, 0); ldcKey = k; ldcHow = "set 0 (int32)";
        } catch (Throwable e) { ldcHow = "not set: " + e; }
        if (focusDpt >= 0) {
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDpt);
        }
        if (expUs > 0 || iso > 0) {
            long frameDur = 1_000_000_000L / fps;
            long e = mainExpNs(ch);
            int s = iso > 0 ? iso : (autoRes != null ? autoRes.iso : 400);   // 0.6.11: auto ISO (chosen once, locked)
            Range<Integer> sr = ch.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            if (sr != null) s = sr.clamp(s);
            rb.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            rb.set(CaptureRequest.SENSOR_EXPOSURE_TIME, e);
            rb.set(CaptureRequest.SENSOR_SENSITIVITY, s);
            rb.set(CaptureRequest.SENSOR_FRAME_DURATION, frameDur);
            meta.put("exposure_ns_set", e).put("iso_set", s).put("frame_duration_ns_set", frameDur);
        } else meta.put("exposure_ns_set", "AE");
    }

    /** What the camera offers for this YUV size (meta.json): size listed, min frame duration (<= 1/fps for real time). */
    private void streamOffer(CameraCharacteristics ch) {
        try {
            StreamConfigurationMap map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size want = new Size(W, H);
            boolean ok = false;
            StringBuilder sz = new StringBuilder();
            Size[] ss = map.getOutputSizes(ImageFormat.YUV_420_888);
            if (ss != null) for (Size s : ss) { sz.append(s).append(' '); ok |= s.equals(want); }
            meta.put("yuv_size_offered", ok).put("yuv_sizes", sz.toString().trim());
            if (ok) {
                long md = map.getOutputMinFrameDuration(ImageFormat.YUV_420_888, want);
                meta.put("yuv_min_frame_duration_ns", md);
                if (md > 1_000_000_000L / fps + 100_000L) status.show("WARNING: YUV " + want + " min frame " + md / 1e6 + " ms > 1/" + fps);
            } else status.show("WARNING: YUV " + want + " not offered by the camera");
        } catch (Exception e) { try { meta.put("stream_offer_error", e.toString()); } catch (Exception ignored) { } }
    }

    private CameraCaptureSession.CaptureCallback capCb;

    private static boolean has(int[] a, int v) { if (a == null) return false; for (int x : a) if (x == v) return true; return false; }

    private void meter(TotalCaptureResult res, Handler h) {
        Long e = res.get(CaptureResult.SENSOR_EXPOSURE_TIME);
        Integer is = res.get(CaptureResult.SENSOR_SENSITIVITY);
        if (e == null || is == null || e <= 0 || is <= 0) return;
        if (++meterFrames < 5) return;
        meterEv += e * (double) is; meterExp += e; meterN++;
        if (meterFrames < 15 || sess == null || reqB == null) return;
        try {
            android.util.Range<Integer> sr = ((CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE))
                    .getCameraCharacteristics("2").get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            int isoMax = sr != null ? sr.getUpper() : 3200, isoMin = sr != null ? sr.getLower() : 50;
            double ev = meterEv / meterN, aeExp = meterExp / meterN;
            long exp = (long) Math.min(expCapNs, aeExp);
            if (ev / exp > isoMax) exp = (long) (ev / isoMax);
            int is2 = (int) Math.max(isoMin, Math.min(isoMax, Math.round(ev / Math.max(1, exp))));
            reqB.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            reqB.set(CaptureRequest.SENSOR_EXPOSURE_TIME, exp);
            reqB.set(CaptureRequest.SENSOR_SENSITIVITY, is2);
            reqB.set(CaptureRequest.SENSOR_FRAME_DURATION, 33_333_333L);
            reqB.set(CaptureRequest.CONTROL_AWB_LOCK, true);
            sess.setRepeatingRequest(reqB.build(), capCb, h);
            fixExp = exp; fixIso = is2; isoLo = isoMin; isoHi = isoMax;
            manualSet = true;
            android.util.Log.i("arlat", "short exposure " + exp / 1000 + " us iso " + is2 + " (AE " + (long) aeExp / 1000 + " us)");
        } catch (Exception ex) {
            android.util.Log.w("arlat", "short exposure failed", ex);
            manualSet = true;
        }
    }

    // lens2: adaptive brightness on the short exposure: ISO follows the 95th percentile of Y toward 215 (no clipped highlights)
    private long fixExp; private int fixIso, isoLo = 50, isoHi = 3200, abFrames;
    private final int[] abHist = new int[256];

    private void autoBrightness(byte[] buf, int off) {
        for (int y = H / 16; y < H; y += H / 24) for (int x = W / 32; x < W; x += W / 32) abHist[buf[off + y * W + x] & 0xff]++;
        if (++abFrames < 15) return;
        int total = 0; for (int v : abHist) total += v;
        int acc = 0, p95 = 255; for (int v = 0; v < 256; v++) { acc += abHist[v]; if (acc >= 0.95 * total) { p95 = v; break; } }
        java.util.Arrays.fill(abHist, 0); abFrames = 0;
        double ratio = p95 >= 254 ? 0.75 : 215.0 / Math.max(8, p95);
        if (Math.abs(1 - ratio) < 0.06) return;
        ratio = Math.max(0.8, Math.min(1.25, ratio));
        double want = fixIso * ratio; long exp = fixExp; int is2;
        if (want > isoHi) { is2 = isoHi; exp = Math.min(Math.max(expCapNs, fixExp), (long) (fixExp * want / isoHi)); }
        else if (want < isoLo) { is2 = isoLo; exp = Math.max(100_000L, (long) (fixExp * want / isoLo)); }
        else is2 = (int) Math.round(want);
        if (is2 == fixIso && exp == fixExp) return;
        fixIso = is2; fixExp = exp;
        try {
            reqB.set(CaptureRequest.SENSOR_EXPOSURE_TIME, exp);
            reqB.set(CaptureRequest.SENSOR_SENSITIVITY, is2);
            sess.setRepeatingRequest(reqB.build(), capCb, camH);
        } catch (Exception ex) { android.util.Log.w("arlat", "auto brightness failed", ex); }
    }

    /** temps.csv every 2 s; battery >= maxBatteryTenthC or thermal status >= SEVERE -> stop (onAutoStop). */
    private final Runnable heatTick = new Runnable() {
        @Override public void run() {
            if (!running) return;
            int b = batteryTenthC();
            int ts = thermalStatus();
            float hr = Float.NaN;
            try { hr = ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getThermalHeadroom(10); } catch (Throwable ignored) { }
            long now = SystemClock.elapsedRealtimeNanos();
            try {
                tempsW.write(now + "," + String.format(Locale.US, "%.1f", (now - t0Elapsed) / 1e9) + "," + b / 10.0 + "," + ts + "," + hr + "\n");
                tempsW.flush();
            } catch (Exception ignored) { }
            String why = null;
            if (b >= maxBatteryTenthC) why = "battery " + b / 10.0 + " C";
            else if (ts >= maxThermal) why = "thermal status " + ts;
            if (why != null && onAutoStop != null) {
                stopReason = why;
                status.show("STOP: " + why);
                onAutoStop.run();
                return;
            }
            if (camH != null) camH.postDelayed(this, 2000);
        }
    };

    int batteryTenthC() {
        android.content.Intent bi = ctx.registerReceiver(null, new android.content.IntentFilter(android.content.Intent.ACTION_BATTERY_CHANGED));
        return bi == null ? 0 : bi.getIntExtra(android.os.BatteryManager.EXTRA_TEMPERATURE, 0);
    }

    int thermalStatus() {
        try { return ((android.os.PowerManager) ctx.getSystemService(Context.POWER_SERVICE)).getCurrentThermalStatus(); }
        catch (Throwable e) { return -1; }
    }

    void stop() {
        if (!running && proc == null) return;
        running = false;
        try { ((SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE)).unregisterListener(this); } catch (Exception ignored) { }
        if (dev != null) { dev.close(); dev = null; }
        if (camThread != null) camThread.quitSafely();
        if (imuThread != null) imuThread.quitSafely();
        try { if (writerT != null) writerT.join(500); } catch (InterruptedException ignored) { }
        if (proc != null && writerT != null && writerT.isAlive()) proc.destroy();   // writer stuck in a 2 MB write: closing stdin would block the UI thread
        if (proc != null) {
            // EOF on stdin -> the bridge returns from main (closes bridge_timing.csv / ov_timing.txt), then kill if needed
            try { proc.getOutputStream().close(); } catch (Exception ignored) { }
            try { if (!proc.waitFor(1500, java.util.concurrent.TimeUnit.MILLISECONDS)) proc.destroy(); } catch (Exception ignored) { proc.destroy(); }
            try { meta.put("bridge_exit", proc.isAlive() ? "killed" : String.valueOf(proc.exitValue())); } catch (Exception ignored) { }
            proc = null;
        }
        Object hs = hint; hint = null;
        if (hs != null && android.os.Build.VERSION.SDK_INT >= 31) try { ((android.os.PerformanceHintManager.Session) hs).close(); } catch (Throwable ignored) { }
        notePerf("hint_reports", hintReports);
        notePerf("hint_report_errors", hintErrors);
        synchronized (perf) { try { meta.put("perf", new JSONObject(perf.toString())); } catch (Exception ignored) { } }
        try { if (readerT != null) readerT.join(500); } catch (InterruptedException ignored) { }
        synchronized (framesLock) { closeQuiet(framesW); framesW = null; }
        synchronized (posesLock) { closeQuiet(posesW); posesW = null; closeQuiet(eventsW); eventsW = null; closeQuiet(predW); predW = null; }
        closeQuiet(tempsW);
        if (imuLog && runDir != null) {    // 0.6.19 imu_log rings (imu thread has been quit above; a late event may be torn)
            try { if (imuThread != null) imuThread.join(300); } catch (InterruptedException ignored) { }
            writeRing(new File(runDir, "imu_samples.csv"), "type,t_event_ns,t_arrive_ns", sRing, sN, 3);
            writeRing(new File(runDir, "imu_enq.csv"), "t_ns,enq_ns", eRing, eN, 2);
        }
        if (reader != null) { try { reader.close(); } catch (Exception ignored) { } reader = null; }
        try {
            meta.put("end_elapsed_ns", SystemClock.elapsedRealtimeNanos()).put("secs", (SystemClock.elapsedRealtimeNanos() - t0Elapsed) / 1e9)
                    .put("frames", nCam).put("poses", nPose).put("imu_records", nImu).put("acc_samples", nAcc)
                    .put("capture_results", nResults).put("predict_lines", nPred).put("pool_dropped", poolDropped).put("queue_dropped", camDropped)
                    .put("battery_c_end", batteryTenthC() / 10.0).put("thermal_status_end", thermalStatus())
                    .put("stop_reason", stopReason).put("streamer_dropped", out.dropped)
                    .put("chunked_images", nChunkImg).put("chunks", nChunks).put("imu_log_samples", sN).put("imu_log_writes", wN);
            if (runDir != null) java.nio.file.Files.write(new File(runDir, "meta.json").toPath(), meta.toString(1).getBytes(StandardCharsets.UTF_8));
        } catch (Exception e) { android.util.Log.e("arlat", "ov meta", e); }   // meta.json last = all files complete
        android.util.Log.i("arlat", "ov stopped (" + stopReason + ") -> " + runDir);
    }

    private static void closeQuiet(java.io.Closeable c) { try { if (c != null) c.close(); } catch (Exception ignored) { } }

    /** 0.6.19 imu_log: the last min(n, RING) entries of a ring (width longs each), oldest first. */
    private static void writeRing(File f, String header, long[] ring, long n, int width) {
        if (ring == null) return;
        try (BufferedWriter w = new BufferedWriter(new FileWriter(f), 1 << 16)) {
            w.write(header); w.write('\n');
            long first = Math.max(0, n - RING);
            StringBuilder sb = new StringBuilder(64);
            for (long k = first; k < n; k++) {
                int i = (int) (k & (RING - 1)) * width;
                sb.setLength(0);
                for (int c = 0; c < width; c++) { if (c > 0) sb.append(','); sb.append(ring[i + c]); }
                sb.append('\n');
                w.write(sb.toString());
            }
        } catch (Exception e) { android.util.Log.w("arlat", "imu log " + f, e); }
    }

    private void onImage(ImageReader r) {
        long now = SystemClock.elapsedRealtimeNanos();
        Image im = r.acquireLatestImage();
        if (im == null) return;
        long ts = im.getTimestamp();
        long tSend = ts + midOffsetNs;
        int fate = 0;
        long copyUs = 0, copyDone = 0, enq = 0;
        byte[] buf = pool == null ? null : pool.poll();
        if (buf == null) {
            fate = 1; poolDropped++;
        } else {
            long c0 = System.nanoTime();
            Image.Plane y = im.getPlanes()[0];
            ByteBuffer b = y.getBuffer();
            int rs = y.getRowStride();
            ByteBuffer rec = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN);
            rec.putInt(MAGIC).putInt(2).putInt(20 + W * H);
            rec.putLong(tSend).putInt(W).putInt(H).put((byte) 1).put((byte) 0).put((byte) 0).put((byte) 0);
            if (rs == W) { b.position(0); b.get(buf, 32, W * H); }
            else for (int i = 0; i < H; i++) { b.position(i * rs); b.get(buf, 32 + i * W, W); }
            copyUs = (System.nanoTime() - c0) / 1000;
            copyDone = SystemClock.elapsedRealtimeNanos();
            if (!isMain() && manualSet && fixIso > 0) autoBrightness(buf, 32);
            if (nCam % 150 == 0 && (!isMain() || nCam <= 600)) {      // raw Y snapshot (main: 4 only, a 2 MB write stalls the cam thread) for checking field of view / blur on the PC (<WxH> 8-bit)
                File snapDir = isMain() ? runDir : dir;
                try (FileOutputStream fo = new FileOutputStream(new File(snapDir, "frame_" + (nCam / 150 % 4) + ".y"))) {
                    fo.write(buf, 32, W * H);
                } catch (Exception ignored) { }
            }
            sent.put(tSend, new long[]{ts, now});
            if (sent.size() > 300) { final long old = tSend - 2_000_000_000L; sent.keySet().removeIf(k -> k < old); }
            if (!toBridge.offer(buf)) { fate = 2; camDropped++; pool.offer(buf); }
            else { enq = SystemClock.elapsedRealtimeNanos(); if (imuPrio > 0) wake.release(); }   // 0.6.19: wake the imu_prio writer
            if (fate == 0 && !fed) {
                fed = true;
                try { meta.put("first_fed_elapsed_ns", now).put("first_fed_t_sensor_ns", ts); } catch (Exception ignored) { }
                if (onFeedStart != null) onFeedStart.run();
            }
        }
        im.close();
        synchronized (framesLock) {
            try {
                if (framesW != null) framesW.write(nCam + "," + ts + "," + now + "," + tSend + "," + copyUs + "," + fate + "," + expNs + "," + lastFocus + "," + copyDone + "," + enq + "\n");
            } catch (Exception ignored) { }
        }
        nCam++;
    }

    /**
     * One IMU record per gyro sample with the accelerometer interpolated at the gyro time.
     * S24 delivers the accelerometer in bursts (~every 12 ms, several samples at once) while the gyro streams, so a gyro
     * sample is held until an accel sample at or after its time has arrived, then interpolated between the bracketing
     * accel samples. (2026-10-18: keeping only the last two accel samples froze accel for ~6 gyro samples at a time —
     * 16 % unique values — and OpenVINS drifted off by tens of metres; the same stream replayed on the PC did the same.)
     * Uncalibrated sensors: values[0..2] only (raw; Android's bias estimate in 3..5 is not applied) = bench --imu-raw.
     */
    private final java.util.ArrayDeque<long[]> gyroQ = new java.util.ArrayDeque<>();      // {t, bits(x), bits(y), bits(z)}
    private final java.util.ArrayList<float[]> accV = new java.util.ArrayList<>();
    private final java.util.ArrayList<Long> accT = new java.util.ArrayList<>();

    @Override public void onSensorChanged(SensorEvent e) {
        if (sRing != null) {   // 0.6.19 imu_log: every sensor event (type, event time, delivery time)
            int i = (int) (sN & (RING - 1)) * 3;
            sRing[i] = e.sensor.getType(); sRing[i + 1] = e.timestamp; sRing[i + 2] = SystemClock.elapsedRealtimeNanos();
            sN++;
        }
        if (e.sensor.getType() == accType) {
            if (!accT.isEmpty() && e.timestamp <= accT.get(accT.size() - 1)) return;
            accT.add(e.timestamp);
            accV.add(new float[]{e.values[0], e.values[1], e.values[2]});
            nAcc++;
        } else {
            gyroQ.add(new long[]{e.timestamp, Float.floatToIntBits(e.values[0]), Float.floatToIntBits(e.values[1]),
                    Float.floatToIntBits(e.values[2])});
        }
        flushImu();
    }

    private void flushImu() {
        if (accT.isEmpty()) { gyroQ.clear(); return; }
        long accNewest = accT.get(accT.size() - 1);
        while (!gyroQ.isEmpty()) {
            long[] g = gyroQ.peekFirst();
            long tg = g[0];
            // wait for the accel that brackets this gyro sample; give up waiting after 50 ms (accel stalled)
            if (tg > accNewest && gyroQ.peekLast()[0] - tg < 50_000_000L) break;
            gyroQ.pollFirst();
            float[] a;
            int k = 0;
            while (k + 1 < accT.size() && accT.get(k + 1) <= tg) k++;    // accT[k] <= tg < accT[k+1] when possible
            if (tg <= accT.get(0)) a = accV.get(0);
            else if (k + 1 >= accT.size()) a = accV.get(accT.size() - 1);
            else {
                double f = (double) (tg - accT.get(k)) / (accT.get(k + 1) - accT.get(k));
                float[] a0 = accV.get(k), a1 = accV.get(k + 1);
                a = new float[]{(float) (a0[0] + f * (a1[0] - a0[0])), (float) (a0[1] + f * (a1[1] - a0[1])),
                        (float) (a0[2] + f * (a1[2] - a0[2]))};
            }
            // drop accel samples no later gyro can need (keep the one at or before tg)
            if (k > 0) { accT.subList(0, k).clear(); accV.subList(0, k).clear(); }
            ByteBuffer rec = ByteBuffer.allocate(12 + 56).order(ByteOrder.LITTLE_ENDIAN);
            rec.putInt(MAGIC).putInt(1).putInt(56).putLong(tg);
            rec.putDouble(Float.intBitsToFloat((int) g[1])).putDouble(Float.intBitsToFloat((int) g[2]))
                    .putDouble(Float.intBitsToFloat((int) g[3]));
            rec.putDouble(a[0]).putDouble(a[1]).putDouble(a[2]);
            if (imuPrio > 0) { imuQ.offer(rec.array()); wake.release(); }   // 0.6.19 imu_prio: own queue
            else toBridge.offer(rec.array());
            if (eRing != null) { int i = (int) (eN & (RING - 1)) * 2; eRing[i] = tg; eRing[i + 1] = SystemClock.elapsedRealtimeNanos(); eN++; }
            nImu++;
        }
    }

    @Override public void onAccuracyChanged(Sensor s, int a) { }

    /** seconds of bridge input to also save as files/ov/live_stream.bin (0 = off) — for replaying the live run on the PC */
    int dumpSecs;

    private void writer() {
        OutputStream dump = null;
        long dumpEnd = 0;
        boostSelf("writer");
        try {
            if (dumpSecs > 0) {
                dump = new java.io.BufferedOutputStream(new FileOutputStream(new File(dir, "live_stream.bin")), 1 << 20);
                dumpEnd = SystemClock.elapsedRealtime() + dumpSecs * 1000L;
            }
            OutputStream o = proc.getOutputStream();
            if (imuPrio > 0) { dump = writerPrio(o, dump, dumpEnd); return; }   // 0.6.19
            while (running) {
                byte[] r = toBridge.poll(100, java.util.concurrent.TimeUnit.MILLISECONDS);
                if (r == null) continue;
                long deq = SystemClock.elapsedRealtimeNanos();
                o.write(r);
                if (wRing != null) {                      // 0.6.19 imu_log: IMU deq; done = when a write/flush pushed it
                    if (r.length == 68) imuWritten(r, deq);
                    else imuPushed(SystemClock.elapsedRealtimeNanos());   // the image write flushed the buffered IMU first
                }
                if (r.length > 68 && writesW != null) {   // image record: t_send_ns (bytes 12..19, LE) -> deq / write done
                    long tw = SystemClock.elapsedRealtimeNanos();
                    long ts = ByteBuffer.wrap(r, 12, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
                    writesW.write(ts + "," + deq + "," + tw + "\n");
                }
                if (dump != null) {
                    dump.write(r);
                    if (SystemClock.elapsedRealtime() > dumpEnd) { dump.close(); dump = null; status.show("dump done"); }
                }
                if (r.length > 68 && pool != null) pool.offer(r);    // image buffer back to the pool
                if (toBridge.isEmpty()) { o.flush(); if (wRing != null) imuPushed(SystemClock.elapsedRealtimeNanos()); }
            }
            if (dump != null) dump.close();
        } catch (Exception e) {
            if (running) status.show("writer: " + e);
        } finally {
            if (dump != null) closeQuiet(dump);
            closeQuiet(writesW);
            writesW = null;
            if (wRing != null && runDir != null) writeRing(new File(runDir, "imu_writes.csv"), "t_ns,deq_ns,write_done_ns", wRing, wN, 3);
        }
    }

    /** 0.6.19 imu_log: an IMU record went into the (buffered) output stream at deq. */
    private void imuWritten(byte[] r, long deq) {
        int i = (int) (wN & (RING - 1)) * 3;
        wRing[i] = ByteBuffer.wrap(r, 12, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
        wRing[i + 1] = deq;
        wRing[i + 2] = 0;
        if (wPendN == wPend.length) wPend = java.util.Arrays.copyOf(wPend, wPendN * 2);
        wPend[wPendN++] = wN;
        wN++;
    }

    /** 0.6.19 imu_log: a write / flush call returned at t: every IMU written before it is in the pipe. */
    private void imuPushed(long t) {
        for (int k = 0; k < wPendN; k++) if (wN - wPend[k] < RING) wRing[(int) (wPend[k] & (RING - 1)) * 3 + 2] = t;
        wPendN = 0;
    }

    /** 0.6.19 imu_prio writer: waiting IMU first, then the image in CHUNK pieces (types 5 / 6) with the IMU that arrived
     *  meanwhile between the pieces; flush after every IMU batch and after the image. IMU newer than the pending image
     *  + IMU_LEAD_NS wait until that image is out (keeps the bridge's feed order identical, class doc). Returns the dump
     *  stream if still open (dumpSecs: logical records = whole images, in send order). */
    private OutputStream writerPrio(OutputStream o, OutputStream dump, long dumpEnd) throws Exception {
        byte[] ch = new byte[12];
        ByteBuffer chb = ByteBuffer.wrap(ch).order(ByteOrder.LITTLE_ENDIAN);
        OutputStream[] dumpRef = {dump};
        while (running) {
            wake.tryAcquire(100, java.util.concurrent.TimeUnit.MILLISECONDS);
            wake.drainPermits();                  // everything queued before this point is handled below
            while (running) {
                byte[] img = toBridge.peek();
                if (img == null) { if (drainImu(o, Long.MAX_VALUE, dumpRef, dumpEnd) == 0) break; continue; }
                long ti = ByteBuffer.wrap(img, 12, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
                long limit = ti + IMU_LEAD_NS;
                drainImu(o, limit, dumpRef, dumpEnd);
                toBridge.poll();
                long deq = SystemClock.elapsedRealtimeNanos();
                for (int off = 12; off < img.length; ) {
                    int n = Math.min(CHUNK, img.length - off);
                    boolean last = off + n >= img.length;
                    chb.clear();
                    chb.putInt(MAGIC).putInt(last ? 6 : 5).putInt(n);
                    o.write(ch);
                    o.write(img, off, n);
                    nChunks++;
                    off += n;
                    if (!last) drainImu(o, limit, dumpRef, dumpEnd);   // IMU that came in meanwhile goes between the chunks
                }
                o.flush();
                long tw = SystemClock.elapsedRealtimeNanos();
                if (wRing != null) imuPushed(tw);
                nChunkImg++;
                if (writesW != null) writesW.write(ti + "," + deq + "," + tw + "\n");
                if (dumpRef[0] != null) dumpWrite(dumpRef, img, dumpEnd);
                if (pool != null) pool.offer(img);
            }
        }
        return dumpRef[0];
    }

    /** imu_prio: write every queued IMU record with t <= limit, then flush; returns the count. */
    private int drainImu(OutputStream o, long limit, OutputStream[] dumpRef, long dumpEnd) throws Exception {
        int k = 0;
        byte[] r;
        while ((r = imuQ.peek()) != null) {
            if (ByteBuffer.wrap(r, 12, 8).order(ByteOrder.LITTLE_ENDIAN).getLong() > limit) break;
            imuQ.poll();
            long deq = SystemClock.elapsedRealtimeNanos();
            o.write(r);
            if (wRing != null) imuWritten(r, deq);
            if (dumpRef[0] != null) dumpWrite(dumpRef, r, dumpEnd);
            k++;
        }
        if (k > 0) { o.flush(); if (wRing != null) imuPushed(SystemClock.elapsedRealtimeNanos()); }
        return k;
    }

    private void dumpWrite(OutputStream[] d, byte[] r, long dumpEnd) throws Exception {
        d[0].write(r);
        if (SystemClock.elapsedRealtime() > dumpEnd) { d[0].close(); d[0] = null; status.show("dump done"); }
    }

    private void readerLoop() {
        boostSelf("reader");
        try {
            InputStream in = proc.getInputStream();
            BufferedReader br = new BufferedReader(new InputStreamReader(in));
            String l;
            while ((l = br.readLine()) != null) {
                long tOut = SystemClock.elapsedRealtimeNanos();
                if (!l.startsWith("{")) continue;
                JSONObject j = new JSONObject(l);
                String type = j.optString("type");
                if ("ready".equals(type)) {   // 0.6.15: what the bridge actually uses (cv::getNumThreads, OpenCV parallel framework)
                    notePerf("bridge_cv_threads", j.optInt("cv_threads", -1));
                    notePerf("bridge_cv_parallel", j.optString("cv_parallel", "?"));
                    notePerf("bridge_klt_overlap", j.optInt("klt_overlap", -1));
                    notePerf("bridge_idle_spin", j.optInt("idle_spin", -1));            // 0.6.16
                    notePerf("bridge_idle_spin_stripes", j.optInt("idle_spin_stripes", -1));
                    notePerf("bridge_predict", j.optInt("predict", -1));                // 0.6.17
                    notePerf("bridge_predict_hz", j.optDouble("predict_hz", -1));
                }
                if ("pred".equals(type)) {    // 0.6.17: IMU forward-predicted pose (class doc) -> poses_pred.csv
                    long tc = j.optLong("t_cam_ns");
                    boolean same = tc == lastPoseTns;
                    org.json.JSONArray p = j.optJSONArray("p"), q = j.optJSONArray("q"), v = j.optJSONArray("v");
                    StringBuilder b = new StringBuilder();
                    b.append(j.optLong("t_ns")).append(',').append(tc).append(',').append(same ? lastPoseSensor : -1).append(',')
                            .append(same ? lastPoseArrive : -1).append(',').append(tOut).append(',').append(j.optString("src"))
                            .append(',').append(j.optString("status")).append(',').append(j.optDouble("h", 0) * 1e3).append(',')
                            .append(j.optInt("n_imu"));
                    for (org.json.JSONArray a : new org.json.JSONArray[]{p, q, v})
                        for (int k = 0; k < (a == null ? 0 : a.length()); k++) b.append(',').append(a.optDouble(k));
                    b.append('\n');
                    synchronized (posesLock) { if (predW != null) predW.write(b.toString()); }
                    nPred++;
                    if (predictStream) {
                        JSONObject o = new JSONObject().put("src", "ov_pred").put("t_out", tOut).put("t_pred", j.optLong("t_ns"))
                                .put("t_cam", tc).put("h_ms", j.optDouble("h", 0) * 1e3).put("status", j.optString("status"));
                        if (same && lastPoseSensor > 0) o.put("t_sensor", lastPoseSensor);
                        if (p != null) o.put("p", p);
                        if (q != null) o.put("q", q);
                        out.send(o.toString());
                    }
                    continue;
                }
                if (!"pose".equals(type)) {
                    JSONObject ev = new JSONObject().put("src", "ov").put("event", j);
                    out.send(ev.toString());
                    synchronized (posesLock) { if (eventsW != null) { eventsW.write(tOut + " " + l + "\n"); eventsW.flush(); } }
                    continue;
                }
                long tNs = j.optLong("t_ns");
                long[] s = sent.remove(tNs);
                lastPoseTns = tNs;                                  // 0.6.17: the "pred" line of this frame follows
                lastPoseSensor = s == null ? -1 : s[0];
                lastPoseArrive = s == null ? -1 : s[1];
                JSONObject o = new JSONObject();
                o.put("src", "ov").put("t_out", tOut).put("status", j.optString("status"));
                if (s != null) o.put("t_sensor", s[0]).put("t_arrive", s[1]);
                if (j.has("p")) o.put("p", j.getJSONArray("p"));
                if (j.has("q")) o.put("q", j.getJSONArray("q"));
                o.put("ms", j.optDouble("ms", 0)).put("n_slam", j.optInt("n_slam")).put("n_msckf", j.optInt("n_msckf"));
                o.put("dt_cam_imu", j.optDouble("dt_cam_imu", 0)).put("cam_q_drop", camDropped).put("pool_drop", poolDropped);
                o.put("exp_ms", expNs / 1e6).put("mid_ms", midOffsetNs / 1e6).put("n_imu", nImu).put("n_acc", nAcc).put("n_cam", nCam);
                out.send(o.toString());
                synchronized (posesLock) {
                    if (posesW != null) posesW.write(tNs + "," + (s == null ? -1 : s[0]) + "," + (s == null ? -1 : s[1]) + "," + tOut + ","
                            + j.optString("status") + "," + j.optDouble("ms", 0) + "," + j.optDouble("remap_ms", 0) + ","
                            + j.optInt("n_slam") + "," + j.optInt("n_msckf") + "," + j.optLong("cam_dropped") + "," + poolDropped + ","
                            + camDropped + "," + nCam + "," + nImu + "," + j.optDouble("dt_cam_imu", 0) + "\n");
                }
                lastStatus = j.optString("status");
                perfOnPose(j.optDouble("ms", 0));
                if (++nPose % 15 == 0) {
                    status.show(String.format(Locale.US, "OpenVINS %s (%s %dx%d@%d %s)\ncam %d imu %d pose %d drop %d/%d\nlatency %.0f ms  ov %.1f ms\nbattery %.1f C\n%s",
                            lastStatus, cam, W, H, fps, cfgName, nCam, nImu, nPose, poolDropped, camDropped,
                            s == null ? -1.0 : (tOut - s[0]) / 1e6, j.optDouble("ms", 0), batteryTenthC() / 10.0,
                            runDir == null ? "" : runDir.getName()));
                }
            }
            if (running) status.show("bridge exited (see " + (runDir == null ? "files/ov" : runDir.getName()) + "/bridge_stderr.log)");
        } catch (Exception e) {
            if (running) status.show("reader: " + e);
        }
    }
}
