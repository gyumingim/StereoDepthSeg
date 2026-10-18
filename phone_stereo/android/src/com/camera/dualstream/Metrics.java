package com.camera.dualstream;

import org.json.*;
import java.util.ArrayDeque;
import java.util.concurrent.atomic.AtomicLong;

/** Live load counters for metrics.json (format scan3d/SPEC.md 14.3). Cheap synchronized adds from the camera,
 *  JPEG and USB threads; snapshot() once a second from the metrics thread. Window = last 2 s. Nothing is measured
 *  actively: these are only timings of work the app does anyway. */
final class Metrics {
    static final long WINDOW_NS = 2_000_000_000L;
    final String[] ids;
    private final Lens[] lens;
    final AtomicLong usbBytes = new AtomicLong(), queueEvicted = new AtomicLong(), captureFailed = new AtomicLong();

    private static final class Lens {
        final ArrayDeque<long[]> cam = new ArrayDeque<>();    // {sensor ts ns, copy ns}
        final ArrayDeque<long[]> jpeg = new ArrayDeque<>();   // {end ns (System.nanoTime), encode ns, bytes}
        final ArrayDeque<long[]> h264 = new ArrayDeque<>();   // {output ns (System.nanoTime), queue->output ns, bytes}
        long camFrames, gapDrops, bufferLost, jpegFrames, jpegDropped, lastTs = -1;
        long h264Frames, h264Keys, h264Dropped, h264WaitKey, h264Lost;
        final ArrayDeque<long[]> kept = new ArrayDeque<>();   // subsample: {System.nanoTime, copy ns} of kept frames
        long keptFrames;
    }

    Metrics(String[] ids) {
        this.ids = ids;
        lens = new Lens[ids.length];
        for (int i = 0; i < ids.length; i++) lens[i] = new Lens();
    }

    private static void trim(ArrayDeque<long[]> q, long now) {
        while (!q.isEmpty() && now - q.peekFirst()[0] > WINDOW_NS) q.pollFirst();
    }

    /** Camera thread, every acquired image. A sensor-timestamp gap > 1.5 frame periods counts the missing frames
     *  (HAL skip or acquireLatestImage discarding a late image). */
    synchronized void camFrame(int i, long sensorTs, long copyNs, int fps) {
        Lens l = lens[i];
        l.camFrames++;
        if (l.lastTs > 0 && fps > 0) {
            double period = 1e9 / fps, gap = sensorTs - l.lastTs;
            if (gap > 1.5 * period) l.gapDrops += Math.max(0, Math.round(gap / period) - 1);
        }
        l.lastTs = sensorTs;
        l.cam.addLast(new long[]{sensorTs, copyNs});
        trim(l.cam, sensorTs);
    }

    /** Subsample mode: a kept frame was copied (camera thread). camFrame still counts every captured frame. */
    synchronized void kept(int i, long copyNs) {
        Lens l = lens[i];
        long now = System.nanoTime();
        l.keptFrames++;
        l.kept.addLast(new long[]{now, copyNs});
        trim(l.kept, now);
    }

    synchronized void bufferLost(int i) { if (i >= 0 && i < lens.length) lens[i].bufferLost++; }

    synchronized void jpeg(int i, long encodeNs, int bytes, long nowNs) {
        Lens l = lens[i];
        l.jpegFrames++;
        l.jpeg.addLast(new long[]{nowNs, encodeNs, bytes});
        trim(l.jpeg, nowNs);
    }

    synchronized void jpegDrop(int i) { lens[i].jpegDropped++; }

    /** Encoder thread, every H.264 output frame. encodeNs = queueInputBuffer -> dequeueOutputBuffer. */
    synchronized void h264(int i, long encodeNs, int bytes, boolean key, long nowNs) {
        Lens l = lens[i];
        l.h264Frames++; if (key) l.h264Keys++;
        l.h264.addLast(new long[]{nowNs, encodeNs, bytes});
        trim(l.h264, nowNs);
    }
    /** Camera frame not encoded (encoder queue full / no input buffer). */
    synchronized void h264Drop(int i) { if (i >= 0 && i < lens.length) lens[i].h264Dropped++; }
    /** Encoded P-frame not sent because the lens waits for a keyframe (after a queue eviction or a new client). */
    synchronized void h264WaitKey(int i) { if (i >= 0 && i < lens.length) lens[i].h264WaitKey++; }
    /** Encoder output thrown away (no sensor timestamp for its pts; seq still advanced). */
    synchronized void h264Lost(int i) { if (i >= 0 && i < lens.length) lens[i].h264Lost++; }

    /** Puts "lenses" and "jpeg_core_pct_total" into out. rec* arrays may be null (not recording). */
    synchronized void fill(JSONObject out, long nowNs, long[] recFrames, double[] recFps, long[] recDropQ, long[] recBusy) throws JSONException {
        JSONObject all = new JSONObject();
        double jpegCoreTotal = 0;
        for (int i = 0; i < lens.length; i++) {
            Lens l = lens[i];
            JSONObject o = new JSONObject();
            int n = l.cam.size();
            double camFps = n >= 2 ? (n - 1) * 1e9 / (l.cam.peekLast()[0] - l.cam.peekFirst()[0]) : 0;
            long copySum = 0, copyMax = 0;
            for (long[] c : l.cam) { copySum += c[1]; copyMax = Math.max(copyMax, c[1]); }
            o.put("cam_fps", Caps.num(Caps.round(camFps)));
            o.put("cam_frames", l.camFrames);
            o.put("cam_gap_drops", l.gapDrops);
            o.put("cam_buffer_lost", l.bufferLost);
            trim(l.kept, nowNs);
            if (l.keptFrames > 0) {                            // subsample: copy time of kept frames only
                int kn = l.kept.size();
                copySum = 0; copyMax = 0;
                for (long[] c : l.kept) { copySum += c[1]; copyMax = Math.max(copyMax, c[1]); }
                n = kn;
                o.put("kept_fps", Caps.round(kn * 1e9 / WINDOW_NS));
                o.put("kept_frames", l.keptFrames);
            }
            o.put("copy_ms_avg", n == 0 ? 0 : Caps.round(copySum / 1e6 / n));
            o.put("copy_ms_max", Caps.round(copyMax / 1e6));
            trim(l.jpeg, nowNs);
            int m = l.jpeg.size();
            long encSum = 0, encMax = 0, bytes = 0;
            for (long[] e : l.jpeg) { encSum += e[1]; encMax = Math.max(encMax, e[1]); bytes += e[2]; }
            double core = encSum * 100.0 / WINDOW_NS;          // % of one core spent in compressToJpeg (wall time on its thread)
            jpegCoreTotal += core;
            o.put("jpeg_fps", Caps.round(m * 1e9 / WINDOW_NS));
            o.put("jpeg_ms_avg", m == 0 ? 0 : Caps.round(encSum / 1e6 / m));
            o.put("jpeg_ms_max", Caps.round(encMax / 1e6));
            o.put("jpeg_core_pct", Caps.round(core));
            o.put("jpeg_kb_avg", m == 0 ? 0 : Caps.round(bytes / 1024.0 / m));
            o.put("jpeg_frames", l.jpegFrames);
            o.put("jpeg_dropped", l.jpegDropped);
            trim(l.h264, nowNs);
            int k = l.h264.size();
            long hSum = 0, hMax = 0, hBytes = 0;
            for (long[] e : l.h264) { hSum += e[1]; hMax = Math.max(hMax, e[1]); hBytes += e[2]; }
            o.put("h264_fps", Caps.round(k * 1e9 / WINDOW_NS));
            o.put("h264_kbps", Caps.round(hBytes * 8.0 / 1000 / (WINDOW_NS / 1e9)));     // encoder output, last 2 s
            o.put("h264_enc_ms_avg", k == 0 ? 0 : Caps.round(hSum / 1e6 / k));
            o.put("h264_enc_ms_max", Caps.round(hMax / 1e6));
            o.put("h264_frames", l.h264Frames);
            o.put("h264_keyframes", l.h264Keys);
            o.put("h264_dropped", l.h264Dropped);
            o.put("h264_dropped_wait_key", l.h264WaitKey);
            o.put("h264_lost_output", l.h264Lost);
            if (recFrames != null) {
                o.put("rec_frames", recFrames[i]);
                o.put("rec_fps", Caps.num(Caps.round(recFps[i])));
                o.put("rec_dropped_queue", recDropQ[i]);
                o.put("rec_encoder_busy", recBusy[i]);
            }
            all.put(ids[i], o);
        }
        out.put("lenses", all);
        out.put("jpeg_core_pct_total", Caps.round(jpegCoreTotal));
    }
}
