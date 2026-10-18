package com.scan3d.arlat;

import android.media.Image;

import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.ShortBuffer;

/**
 * Loop closure on top of ARCore (native libarloop.so, phone_stereo/arlat/jni/arloop.h).
 * mode: "both" | "marker" (ArUco only) | "feature" (no marker: ORB + ARCore depth) | "off".
 * Picks keyframes (>= 0.25 s apart and moved >= 0.10 m or turned >= 8 deg, or 1 s), copies the 640x480 CPU image
 * (+ ARCore depth when available) to the native worker, and corrects every ARCore pose with the current drift fix.
 * dumpFile != null: every keyframe is also written there ("ARKF" records) for PC replay (arloop_replay).
 */
class ArLoop {
    static { System.loadLibrary("arloop"); }

    private static native long nCreate(boolean markers, boolean features, int dict, float markerLen);
    private static native void nDestroy(long h);
    private static native void nAddKeyframe(long h, long tNs, float[] pose, byte[] gray, int w, int hgt, float[] k,
                                            short[] depth, int dw, int dh);
    private static native void nCorrect(long h, long tNs, float[] in, float[] out);
    private static native String nStats(long h);

    final String mode;
    private long h;
    private long lastKfNs;
    private float[] lastKfPose;
    private OutputStream dump;
    int nKf, nFail;

    ArLoop(String mode, int dict, float markerLen, File dumpFile) {
        this.mode = mode;
        boolean markers = mode.equals("both") || mode.equals("marker");
        boolean features = mode.equals("both") || mode.equals("feature");
        if (markers || features) h = nCreate(markers, features, dict, markerLen);
        try {
            if (dumpFile != null) dump = new BufferedOutputStream(new FileOutputStream(dumpFile), 1 << 20);
        } catch (Exception ignored) { }
    }

    boolean active() { return h != 0; }

    static float[] pose7(Pose p) {
        return new float[]{p.tx(), p.ty(), p.tz(), p.qx(), p.qy(), p.qz(), p.qw()};
    }

    /** Call once per new ARCore frame while TRACKING. */
    void onFrame(Frame f, long tNs, float[] pose) {
        if (h == 0 && dump == null) return;
        if (!isKeyframe(tNs, pose)) return;
        Image img = null, dimg = null;
        try {
            img = f.acquireCameraImage();
            int w = img.getWidth(), hh = img.getHeight();
            Image.Plane y = img.getPlanes()[0];
            ByteBuffer b = y.getBuffer();
            int rs = y.getRowStride();
            byte[] gray = new byte[w * hh];
            for (int r = 0; r < hh; r++) {
                b.position(r * rs);
                b.get(gray, r * w, w);
            }
            img.close();
            img = null;
            CameraIntrinsics ci = f.getCamera().getImageIntrinsics();
            float[] fl = ci.getFocalLength(), pp = ci.getPrincipalPoint();
            int[] dim = ci.getImageDimensions();
            // intrinsics are for dim; scale to the CPU image if they differ
            float sx = (float) w / dim[0], sy = (float) hh / dim[1];
            float[] k = {fl[0] * sx, fl[1] * sy, pp[0] * sx, pp[1] * sy};
            short[] depth = null;
            int dw = 0, dh = 0;
            try {
                dimg = f.acquireDepthImage16Bits();
                dw = dimg.getWidth(); dh = dimg.getHeight();
                Image.Plane dp = dimg.getPlanes()[0];
                ShortBuffer sb = dp.getBuffer().order(ByteOrder.nativeOrder()).asShortBuffer();
                int drs = dp.getRowStride() / 2;
                depth = new short[dw * dh];
                for (int r = 0; r < dh; r++) {
                    sb.position(r * drs);
                    sb.get(depth, r * dw, dw);
                }
            } catch (Exception noDepthYet) {
                depth = null; dw = 0; dh = 0;
            } finally {
                if (dimg != null) dimg.close();
                dimg = null;
            }
            if (h != 0) nAddKeyframe(h, tNs, pose, gray, w, hh, k, depth, dw, dh);
            if (dump != null) writeDump(tNs, pose, gray, w, hh, k, depth, dw, dh);
            nKf++;
        } catch (Exception e) {
            // NotYetAvailableException etc.: skip this keyframe
            nFail++;
            if (nFail <= 5 || nFail % 50 == 0) android.util.Log.w("arlat", "keyframe skipped (" + nFail + "): " + e);
        } finally {
            if (img != null) img.close();
        }
    }

    private boolean isKeyframe(long tNs, float[] p) {
        if (lastKfPose == null) { lastKfNs = tNs; lastKfPose = p; return true; }
        double dt = (tNs - lastKfNs) * 1e-9;
        if (dt < 0.25) return false;
        double dx = p[0] - lastKfPose[0], dy = p[1] - lastKfPose[1], dz = p[2] - lastKfPose[2];
        double dot = Math.abs(p[3] * lastKfPose[3] + p[4] * lastKfPose[4] + p[5] * lastKfPose[5] + p[6] * lastKfPose[6]);
        double ang = 2 * Math.acos(Math.min(1.0, dot)) * 180 / Math.PI;
        if (Math.sqrt(dx * dx + dy * dy + dz * dz) >= 0.10 || ang >= 8 || dt >= 1.0) {
            lastKfNs = tNs; lastKfPose = p;
            return true;
        }
        return false;
    }

    float[] correct(long tNs, float[] pose) {
        if (h == 0) return pose;
        float[] out = new float[7];
        nCorrect(h, tNs, pose, out);
        return out;
    }

    String stats() { return h == 0 ? null : nStats(h); }

    void close() {
        if (h != 0) { nDestroy(h); h = 0; }
        try { if (dump != null) dump.close(); } catch (Exception ignored) { }
        dump = null;
    }

    private void writeDump(long tNs, float[] p, byte[] gray, int w, int hh, float[] k, short[] depth, int dw, int dh)
            throws Exception {
        ByteBuffer b = ByteBuffer.allocate(4 + 8 + 56 + 8 + 32 + 8).order(ByteOrder.LITTLE_ENDIAN);
        b.putInt(0x464B5241).putLong(tNs);
        for (int i = 0; i < 7; i++) b.putDouble(p[i]);
        b.putInt(w).putInt(hh);
        for (int i = 0; i < 4; i++) b.putDouble(k[i]);
        dump.write(b.array(), 0, 4 + 8 + 56 + 8 + 32);
        dump.write(gray);
        ByteBuffer d = ByteBuffer.allocate(8 + 2 * dw * dh).order(ByteOrder.LITTLE_ENDIAN);
        d.putInt(dw).putInt(dh);
        if (depth != null) for (short s : depth) d.putShort(s);
        dump.write(d.array());
    }
}
