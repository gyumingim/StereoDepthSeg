package com.camera.dualstream;

import android.media.Image;
import android.media.MediaCodec;
import android.util.Log;
import org.json.JSONObject;
import java.nio.ByteBuffer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

/** Live H.264 stream (stream_fmt=h264, scan3d/SPEC.md 1.2.2): one encoder + thread per lens, output -> H264.Sink.
 *
 *  Input path = the existing ImageReader YUV -> NV21 copy -> encoder input Image (COLOR_FormatYUV420Flexible), not an
 *  encoder input Surface added to the camera session. Reasons:
 *   1. The PC needs the PHYSICAL lens SENSOR_TIMESTAMP on every frame. Those come from the logical capture result
 *      (CaptureTimestampMatcher pairs ImageReader images with getPhysicalCameraResults). A Surface input would carry the
 *      buffer timestamp chosen by the camera HAL / BufferQueue into presentationTimeUs; whether that is the physical or
 *      the logical timestamp for a physical stream of a logical multi-camera is not documented, so it is not "exactly
 *      the sensor time" we can promise.
 *   2. Two more camera outputs (encoder Surface per lens, next to the YUV readers the recorder / matcher still need)
 *      may exceed the guaranteed stream combinations of a logical multi-camera session.
 *   3. The copy is already done for the recorder and the matcher (one NV21 array shared read-only).
 *  So presentationTimeUs here is our own (sensor ts - first ts)/1000 and the sensor timestamp is looked up by PTS on
 *  output (B-frames off -> output order = input order; the map also covers frames the encoder skips). */
final class LiveEncoder {
    private static final String TAG = "PhoneStereoLive";
    /** Small queue = low latency: a frame waiting behind 2 others is already ~130 ms old at 15 fps. Full -> drop new. */
    private static final int QUEUE = 2;
    final int width, height, fps, kbps;
    final String encoderName;
    private final H264.Sink sink;
    private final Lens[] lenses;
    private volatile boolean stopping;
    volatile JSONObject info;          // settings applied (lens 0), for metrics.json

    private static final class Frame {
        final long ts; final byte[] nv21;
        Frame(long t, byte[] d) { ts = t; nv21 = d; }
    }
    private static final Frame EOS = new Frame(0, null);

    LiveEncoder(String[] ids, int width, int height, int fps, int kbps, String encoderName, H264.Sink sink) {
        this.width = width; this.height = height; this.fps = fps; this.kbps = kbps; this.encoderName = encoderName; this.sink = sink;
        lenses = new Lens[ids.length];
        for (int i = 0; i < ids.length; i++) { lenses[i] = new Lens(i, ids[i]); lenses[i].start(); }
    }

    /** Camera thread. Never blocks. nv21 is shared read-only with the recorder. */
    void frame(int index, long sensorTs, byte[] nv21) {
        if (stopping || index < 0 || index >= lenses.length) return;
        if (!lenses[index].queue.offer(new Frame(sensorTs, nv21))) sink.h264Drop(index);
    }

    /** Keyframe as soon as possible (PARAMETER_KEY_REQUEST_SYNC_FRAME). lens < 0 = all lenses. */
    void requestKey(int lens) {
        for (int i = 0; i < lenses.length; i++) if (lens < 0 || lens == i) lenses[i].keyReq = true;
    }

    String error() {
        for (Lens l : lenses) if (l.error != null) return l.id + ": " + l.error;
        return null;
    }

    void stop(long timeoutMs) {
        if (stopping) return;
        stopping = true;
        for (Lens l : lenses) { l.queue.clear(); l.queue.offer(EOS); }
        long end = System.currentTimeMillis() + timeoutMs;
        try { for (Lens l : lenses) l.join(Math.max(1, end - System.currentTimeMillis())); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    private final class Lens extends Thread {
        final int index; final String id;
        final BlockingQueue<Frame> queue = new ArrayBlockingQueue<>(QUEUE);
        volatile boolean keyReq;
        volatile String error;
        private MediaCodec codec;
        private byte[] csd;
        private int seq;
        private long t0 = Long.MIN_VALUE, lastPts = -1;
        private final MediaCodec.BufferInfo bi = new MediaCodec.BufferInfo();
        /** pts us -> {sensor ts ns, System.nanoTime at queueInputBuffer}. Bounded: frames the encoder never outputs age out. */
        private final LinkedHashMap<Long, long[]> pending = new LinkedHashMap<Long, long[]>(32, 0.75f, false) {
            @Override protected boolean removeEldestEntry(Map.Entry<Long, long[]> e) { return size() > 32; }
        };

        Lens(int index, String id) { super("live-h264-" + id); this.index = index; this.id = id; setPriority(Thread.MAX_PRIORITY - 1); }

        @Override public void run() {
            try {
                codec = MediaCodec.createByCodecName(encoderName);
                JSONObject j = H264.configure(codec, width, height, fps, kbps, true);
                if (index == 0) info = j;
                Log.i(TAG, "lens " + id + " " + j);
                codec.start();
                while (true) {
                    Frame fr = queue.poll(10, TimeUnit.MILLISECONDS);
                    if (fr == EOS || stopping) break;
                    if (keyReq) { keyReq = false; try { H264.requestSync(codec); } catch (Exception e) { Log.w(TAG, "sync", e); } }
                    if (fr != null) feed(fr);
                    drain();
                }
            } catch (Throwable e) {
                error = e.toString(); Log.e(TAG, "lens " + id, e);
                if (!stopping) try { sink.h264Error(index, id + ": " + error); } catch (Throwable ignored) {}
            } finally {
                try { if (codec != null) codec.stop(); } catch (Exception ignored) {}
                try { if (codec != null) codec.release(); } catch (Exception ignored) {}
            }
        }

        private void feed(Frame fr) {
            drain();
            int in = -1;
            for (int tries = 0; tries < 2 && in < 0; tries++) { in = codec.dequeueInputBuffer(5_000); if (in < 0) drain(); }
            if (in < 0) { sink.h264Drop(index); return; }
            Image img = codec.getInputImage(in);
            if (img == null) {
                // Caps.findAvcEncoder requires YUV420Flexible, which guarantees getInputImage; give the buffer back empty.
                codec.queueInputBuffer(in, 0, 0, Math.max(0, lastPts), 0);
                if (error == null) {
                    error = "getInputImage null"; Log.e(TAG, "lens " + id + " getInputImage null");
                    try { sink.h264Error(index, id + ": " + error); } catch (Throwable ignored) {}
                }
                sink.h264Drop(index); return;
            }
            if (t0 == Long.MIN_VALUE) t0 = fr.ts;
            long pts = (fr.ts - t0) / 1000;
            if (pts <= lastPts) pts = lastPts + 1;
            lastPts = pts;
            H264.fillImage(img, fr.nv21, width, height);
            pending.put(pts, new long[]{fr.ts, System.nanoTime()});
            codec.queueInputBuffer(in, 0, width * height * 3 / 2, pts, 0);
        }

        private void drain() {
            while (true) {
                int o = codec.dequeueOutputBuffer(bi, 0);
                if (o == MediaCodec.INFO_TRY_AGAIN_LATER) return;
                if (o < 0) continue;                       // format changed / buffers changed
                ByteBuffer ob = codec.getOutputBuffer(o);
                if (ob != null && bi.size > 0) {
                    if ((bi.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) csd = H264.copy(ob, bi);
                    else {
                        boolean key = (bi.flags & MediaCodec.BUFFER_FLAG_KEY_FRAME) != 0;
                        long[] m = pending.remove(bi.presentationTimeUs);
                        if (m != null) {
                            byte[] data = H264.frameBytes(ob, bi, csd, key);
                            int flags = (key ? H264.FLAG_KEY : 0) | (key && H264.hasSps(data) ? H264.FLAG_CONFIG : 0);
                            sink.h264(index, m[0], data, flags, seq++, System.nanoTime() - m[1]);
                        } else {
                            // Dropped output: still advance seq (PC sees the gap) and make the lens wait for a keyframe (REVIEW7 H1)
                            seq++;
                            sink.h264Lost(index);
                            Log.w(TAG, "lens " + id + " output pts " + bi.presentationTimeUs + " without input");
                        }
                    }
                }
                codec.releaseOutputBuffer(o, false);
            }
        }
    }
}
