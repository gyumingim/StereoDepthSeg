package com.camera.dualstream;

import android.media.Image;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.os.Build;
import android.os.Bundle;
import org.json.JSONObject;
import java.io.IOException;
import java.nio.ByteBuffer;

/** Shared H.264 encoder settings for the live stream (LiveEncoder) and the recorder (Recorder).
 *  User decision 2026-10-16 (scan3d/SPEC.md 1.2.2): I+P only (no B-frames), keyframe every 1 s, real-time priority,
 *  bitrate proportional to pixels x fps (4 Mbps per lens at 640x480@15, Caps.autoKbps), hardware encoder first.
 *  Sources (developer.android.com/reference/android/media/):
 *   - MediaFormat#KEY_MAX_B_FRAMES (API 29), #KEY_I_FRAME_INTERVAL, #KEY_PRIORITY (0 = realtime), #KEY_LATENCY (API 30),
 *     #KEY_LOW_LATENCY (API 30, only when the codec reports FEATURE_LowLatency), #KEY_PROFILE
 *   - MediaCodecInfo.EncoderCapabilities#isBitrateModeSupported, BITRATE_MODE_VBR / BITRATE_MODE_CBR
 *   - MediaCodec#PARAMETER_KEY_REQUEST_SYNC_FRAME (keyframe on request), BUFFER_FLAG_KEY_FRAME, BUFFER_FLAG_CODEC_CONFIG
 *  MediaCodec AVC encoders emit Annex-B (00 00 00 01 start codes) for both the codec-config buffer (SPS+PPS) and frames. */
final class H264 {
    static final String AVC = MediaFormat.MIMETYPE_VIDEO_AVC;
    /** STEH payload flags (SPEC 1.2.2). */
    static final int FLAG_KEY = 1, FLAG_CONFIG = 2;

    /** Encoded-frame consumer (MainActivity). Called on the encoder thread; must not block. */
    interface Sink {
        void h264(int lens, long sensorTs, byte[] annexB, int flags, int seq, long encodeNs);
        /** A camera frame was not encoded (encoder queue full / no input buffer). */
        void h264Drop(int lens);
        /** An encoder OUTPUT was thrown away (no sensor timestamp for its pts). seq was already advanced so the PC sees
         *  the gap; later P-frames of this lens lost their reference -> skip to the next keyframe (REVIEW7 H1). */
        void h264Lost(int lens);
        /** The encoder of this lens stopped with an error (REVIEW7 H2). */
        void h264Error(int lens, String error);
    }

    static boolean high(MediaCodecInfo.CodecCapabilities cc) { return hasProfile(cc, MediaCodecInfo.CodecProfileLevel.AVCProfileHigh); }
    static boolean baseline(MediaCodecInfo.CodecCapabilities cc) { return hasProfile(cc, MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline); }
    static boolean main(MediaCodecInfo.CodecCapabilities cc) { return hasProfile(cc, MediaCodecInfo.CodecProfileLevel.AVCProfileMain); }
    private static boolean hasProfile(MediaCodecInfo.CodecCapabilities cc, int p) {
        if (cc.profileLevels != null) for (MediaCodecInfo.CodecProfileLevel pl : cc.profileLevels) if (pl.profile == p) return true;
        return false;
    }
    static boolean lowLatencyFeature(MediaCodecInfo.CodecCapabilities cc) {
        return Build.VERSION.SDK_INT >= 30 && cc.isFeatureSupported(MediaCodecInfo.CodecCapabilities.FEATURE_LowLatency);
    }
    static boolean modeSupported(MediaCodecInfo.CodecCapabilities cc, int mode) {
        MediaCodecInfo.EncoderCapabilities e = cc.getEncoderCapabilities();
        return e != null && e.isBitrateModeSupported(mode);
    }

    /** Bitrate mode: VBR when supported, else CBR.
     *  Why VBR: CBR holds the bits per second constant, so frames with more detail or motion (a hand-held scan moving
     *  fast) get coarser quantisation; VBR moves bits to those frames and keeps the quality closer to constant around the
     *  same average. Stereo matching / VIO care about steady per-frame quality, and USB has ample headroom over
     *  4-8 Mbps per lens, so the short VBR peaks are harmless. (Reasoning, not measured on this phone.) */
    static int bitrateMode(MediaCodecInfo.CodecCapabilities cc) {
        if (modeSupported(cc, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)) return MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR;
        if (modeSupported(cc, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR)) return MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR;
        return -1;
    }

    static String modeName(int m) {
        return m == MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR ? "vbr"
             : m == MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR ? "cbr"
             : m == MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CQ ? "cq" : "default";
    }

    private static MediaFormat format(int w, int h, int fps, int kbps, int profile, int mode, boolean latencyKeys, boolean lowLat) {
        MediaFormat f = MediaFormat.createVideoFormat(AVC, w, h);
        f.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible);
        f.setInteger(MediaFormat.KEY_BIT_RATE, kbps * 1000);
        f.setInteger(MediaFormat.KEY_FRAME_RATE, fps);
        f.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1);                    // keyframe every 1 s
        if (Build.VERSION.SDK_INT >= 29) f.setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0);   // I+P only
        if (profile > 0) f.setInteger(MediaFormat.KEY_PROFILE, profile);
        if (mode >= 0) f.setInteger(MediaFormat.KEY_BITRATE_MODE, mode);
        f.setInteger(MediaFormat.KEY_PRIORITY, 0);                            // 0 = realtime
        if (latencyKeys && Build.VERSION.SDK_INT >= 30) {
            f.setInteger(MediaFormat.KEY_LATENCY, 1);                         // output each frame as soon as it is encoded
            if (lowLat) f.setInteger(MediaFormat.KEY_LOW_LATENCY, 1);
        }
        return f;
    }

    /** Configure a created (uninitialized) encoder. Tries High (B=0) -> Baseline -> Baseline without latency keys ->
     *  encoder defaults; codec.reset() between attempts. Returns what was applied (goes into metrics.json / meta.json). */
    static JSONObject configure(MediaCodec codec, int w, int h, int fps, int kbps, boolean live) throws IOException {
        MediaCodecInfo ci = codec.getCodecInfo();
        MediaCodecInfo.CodecCapabilities cc = ci.getCapabilitiesForType(AVC);
        int mode = bitrateMode(cc);
        boolean lowLat = lowLatencyFeature(cc);
        int hi = MediaCodecInfo.CodecProfileLevel.AVCProfileHigh, base = MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline;
        // {profile, latency keys}; High only when listed (with B=0 it is still I+P, better compression than Baseline).
        int[][] tries = high(cc) ? new int[][]{{hi, 1}, {base, 1}, {base, 0}, {0, 0}} : new int[][]{{base, 1}, {base, 0}, {0, 0}};
        Exception last = null;
        for (int[] t : tries) {
            boolean lk = live && t[1] == 1;
            try {
                codec.configure(format(w, h, fps, kbps, t[0], mode, lk, lk && lowLat), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
                JSONObject j = new JSONObject();
                j.put("encoder", ci.getName());
                j.put("hardware", Build.VERSION.SDK_INT >= 29 && ci.isHardwareAccelerated());
                j.put("profile", t[0] == hi ? "high" : t[0] == base ? "baseline" : "default");
                j.put("max_b_frames", Build.VERSION.SDK_INT >= 29 ? 0 : -1);   // -1 = key unavailable (< API 29)
                j.put("i_frame_interval_s", 1);
                j.put("kbps", kbps); j.put("bitrate_mode", modeName(mode));
                j.put("priority", 0);
                j.put("latency_1", lk && Build.VERSION.SDK_INT >= 30);
                j.put("low_latency", lk && lowLat);
                j.put("low_latency_feature", lowLat);
                j.put("width", w); j.put("height", h); j.put("fps", fps);
                return j;
            } catch (Exception e) {
                last = e;
                try { codec.reset(); } catch (Exception ignored) {}
            }
        }
        throw new IOException("H.264 configure failed: " + last);
    }

    static void requestSync(MediaCodec codec) {
        Bundle b = new Bundle();
        b.putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0);
        codec.setParameters(b);
    }

    /** NV21 (Y, then VU interleaved) -> encoder Image with arbitrary row/pixel strides. */
    static void fillImage(Image img, byte[] nv21, int width, int height) {
        Image.Plane[] p = img.getPlanes();
        ByteBuffer y = p[0].getBuffer(); int yr = p[0].getRowStride(), yp = p[0].getPixelStride();
        for (int r = 0; r < height; r++) {
            if (yp == 1) { y.position(r * yr); y.put(nv21, r * width, width); }
            else for (int c = 0; c < width; c++) y.put(r * yr + c * yp, nv21[r * width + c]);
        }
        ByteBuffer u = p[1].getBuffer(), v = p[2].getBuffer();
        int ur = p[1].getRowStride(), up = p[1].getPixelStride(), vr = p[2].getRowStride(), vp = p[2].getPixelStride();
        int base = width * height, cw = width / 2, ch = height / 2;
        for (int r = 0; r < ch; r++) for (int c = 0; c < cw; c++) {
            int s = base + r * width + 2 * c;           // nv21[s]=V, nv21[s+1]=U
            v.put(r * vr + c * vp, nv21[s]); u.put(r * ur + c * up, nv21[s + 1]);
        }
    }

    /** True when an SPS NAL (type 7) comes before the first slice NAL (type 1 or 5) in an Annex-B buffer. */
    static boolean hasSps(byte[] b) {
        for (int i = 0; i + 3 < b.length; i++) {
            if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1) {
                int t = b[i + 3] & 0x1f;
                if (t == 7) return true;
                if (t == 1 || t == 5) return false;
                i += 2;
            }
        }
        return false;
    }

    /** True when the first slice NAL (type 1 or 5) in the Annex-B bytes b[pos, lim) is a B slice (slice_type % 5 == 1).
     *  Reads first_mb_in_slice ue(v) and slice_type ue(v) right after the NAL header (emulation-prevention 03 skipped).
     *  Absolute reads only: the buffer's position/limit are not changed (REVIEW7 H5: B=0 is only a request). */
    static boolean isBSlice(ByteBuffer b, int pos, int lim) {
        for (int i = pos; i + 3 < lim; i++) {
            if (b.get(i) == 0 && b.get(i + 1) == 0 && b.get(i + 2) == 1) {
                int t = b.get(i + 3) & 0x1f;
                if (t != 1 && t != 5) { i += 2; continue; }
                // collect up to 12 RBSP bytes after the NAL header
                byte[] r = new byte[12]; int n = 0, zeros = 0;
                for (int k = i + 4; k < lim && n < r.length; k++) {
                    int v = b.get(k) & 0xff;
                    if (zeros >= 2 && v == 3) { zeros = 0; continue; }
                    zeros = v == 0 ? zeros + 1 : 0;
                    r[n++] = (byte) v;
                }
                int[] bit = {0};
                if (ue(r, n, bit) < 0) return false;          // first_mb_in_slice
                int st = ue(r, n, bit);
                return st >= 0 && st % 5 == 1;
            }
        }
        return false;
    }

    /** Exp-Golomb ue(v) from r[0..n) at bit[0]; -1 when it runs past the end. */
    private static int ue(byte[] r, int n, int[] bit) {
        int z = 0;
        while (true) {
            if (bit[0] >= n * 8) return -1;
            int v = (r[bit[0] >> 3] >> (7 - (bit[0] & 7))) & 1; bit[0]++;
            if (v == 1) break;
            if (++z > 30) return -1;
        }
        int x = 0;
        for (int k = 0; k < z; k++) {
            if (bit[0] >= n * 8) return -1;
            x = (x << 1) | ((r[bit[0] >> 3] >> (7 - (bit[0] & 7))) & 1); bit[0]++;
        }
        return (1 << z) - 1 + x;
    }

    /** Output buffer -> STEH payload bytes. Keyframes always carry SPS+PPS first (prepended from the cached codec-config
     *  buffer when the encoder did not repeat them), so a PC decoder can start at any keyframe. */
    static byte[] frameBytes(ByteBuffer ob, MediaCodec.BufferInfo info, byte[] csd, boolean key) {
        byte[] f = new byte[info.size];
        ByteBuffer d = ob.duplicate(); d.position(info.offset); d.limit(info.offset + info.size); d.get(f);
        if (!key || csd == null || hasSps(f)) return f;
        byte[] out = new byte[csd.length + f.length];
        System.arraycopy(csd, 0, out, 0, csd.length); System.arraycopy(f, 0, out, csd.length, f.length);
        return out;
    }

    static byte[] copy(ByteBuffer ob, MediaCodec.BufferInfo info) {
        byte[] f = new byte[info.size];
        ByteBuffer d = ob.duplicate(); d.position(info.offset); d.limit(info.offset + info.size); d.get(f);
        return f;
    }
}
