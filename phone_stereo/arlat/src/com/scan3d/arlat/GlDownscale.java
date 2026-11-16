package com.scan3d.arlat;

import android.graphics.SurfaceTexture;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLExt;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.view.Surface;

import org.json.JSONObject;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Locale;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * 0.6.41 (rec960, 2026-11-16): session "std960" = the SAME calibrated 1920x1080 camera stream, halved on the GPU before the
 * encoder, so the file is 4x smaller (wireless pull) while the lens geometry stays exactly the board-calibrated 1080 one x 0.5.
 *
 * Why not a native 960x540 camera stream: the S24 camera "0" does not offer 960x540 for MediaCodec at all (meta stream_config
 * mediacodec_sizes: ... 1280x720 960x720 720x480 640x480 640x360 ...), and ISP-scaled streams (e.g. 1280x720) have their own
 * crop/scaler geometry that would need a new board calibration (memory 10-24: c7 720 stream with assumed K*2/3 drifted).
 *
 * Path: camera -> SurfaceTexture (1920x1080, GL_OES external) -> one full-screen quad, GL_LINEAR, highp texture coordinates,
 * viewport 960x540 -> MediaCodec input surface (EGL_RECORDABLE_ANDROID). Every output pixel centre lands exactly between four
 * input texel centres, so bilinear = the 2x2 box average (= cv2.INTER_AREA for an exact 2x downscale); the coordinate map is
 * x960 = (x1920 + 0.5) / 2 - 0.5 (pixel centres at integers), same for y.
 * Orientation (review 11-16): the SurfaceTexture transform matrix of a Camera2 output usually carries the SENSOR_ORIENTATION
 * rotation (90 deg), which the direct MediaCodec path does not apply. So the matrix is NOT used for sampling: a fixed flip-only
 * matrix (s = x, t = 1 - y) = what an untransformed buffer gives = the same raw sensor orientation as the direct encoder path.
 * The real matrix is logged; if it is not a pure rotation/flip (any crop or scale), downscale.matrix_ok = false and
 * lp_replay.py refuses the recording.
 * Timestamps: eglPresentationTimeANDROID = the buffer timestamp moved into the clock the direct encoder path used
 * (CLOCK_MONOTONIC), so HsRecorder's pts -> SENSOR_TIMESTAMP matching is unchanged; the clock decision goes to meta.json.
 * Colour: camera YUV -> RGB in the GPU sampler -> RGB -> YUV in the encoder (not bit-identical to the direct YUV path; luma
 * range/matrix may differ slightly).  UNTESTED ON THE DEVICE when written (built only).
 */
class GlDownscale {
    static final int EGL_RECORDABLE_ANDROID = 0x3142;
    final int inW, inH, outW, outH;
    private HandlerThread th;
    private Handler h;
    private EGLDisplay dpy = EGL14.EGL_NO_DISPLAY;
    private EGLContext ctx = EGL14.EGL_NO_CONTEXT;
    private EGLSurface surf = EGL14.EGL_NO_SURFACE;
    private int tex, prog, aPos, uTexM;
    private FloatBuffer quad;
    private SurfaceTexture st;
    Surface camSurface;                      // camera output target (instead of the encoder surface)
    private final float[] m = new float[16];
    private volatile boolean stopped;
    private volatile int nDrawn, nSkipped;
    private String firstMatrix = "none", clock = "unknown", err = null;
    private boolean matrixOk = true;
    // column-major flip-only matrix: s = x, t = 1 - y (sampling never uses the SurfaceTexture matrix, see doc)
    private static final float[] FLIP = {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1};
    private volatile int nSwapFail;
    private long tsShiftNs;                  // pts_ns = buffer ts - tsShiftNs
    private boolean clockKnown;
    private final long clockOffsetNs;        // BOOTTIME - MONOTONIC at the recording start (HsRecorder.clockOffset)

    GlDownscale(int inW, int inH, int outW, int outH, long clockOffsetNs) {
        this.inW = inW; this.inH = inH; this.outW = outW; this.outH = outH; this.clockOffsetNs = clockOffsetNs;
    }

    /** EGL + GL setup on its own thread, bound to the encoder input surface; returns the camera output surface. */
    Surface start(final Surface encSurface) throws Exception {
        th = new HandlerThread("hs-gl", android.os.Process.THREAD_PRIORITY_DISPLAY); th.start();
        h = new Handler(th.getLooper());
        final CountDownLatch done = new CountDownLatch(1);
        final Exception[] fail = new Exception[1];
        h.post(() -> {
            try { setup(encSurface); } catch (Exception e) { fail[0] = e; }
            done.countDown();
        });
        if (!done.await(3, TimeUnit.SECONDS)) throw new IllegalStateException("GL setup timeout");
        if (fail[0] != null) throw fail[0];
        return camSurface;
    }

    private void setup(Surface encSurface) {
        dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
        int[] v = new int[2];
        if (!EGL14.eglInitialize(dpy, v, 0, v, 1)) throw new IllegalStateException("eglInitialize");
        int[] attr = {EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL_RECORDABLE_ANDROID, 1, EGL14.EGL_NONE};
        EGLConfig[] cfg = new EGLConfig[1];
        int[] n = new int[1];
        if (!EGL14.eglChooseConfig(dpy, attr, 0, cfg, 0, 1, n, 0) || n[0] < 1) throw new IllegalStateException("eglChooseConfig recordable");
        ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT, new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
        if (ctx == EGL14.EGL_NO_CONTEXT) throw new IllegalStateException("eglCreateContext");
        surf = EGL14.eglCreateWindowSurface(dpy, cfg[0], encSurface, new int[]{EGL14.EGL_NONE}, 0);
        if (surf == EGL14.EGL_NO_SURFACE) throw new IllegalStateException("eglCreateWindowSurface (encoder)");
        if (!EGL14.eglMakeCurrent(dpy, surf, surf, ctx)) throw new IllegalStateException("eglMakeCurrent");
        // highp everywhere: mediump (fp16) texture coordinates are ~1 texel off at 1920 and would break the exact 2x2 box
        String vs = "attribute vec2 aPos; uniform mat4 uTexM; varying highp vec2 vTc;\n"
                + "void main() { gl_Position = vec4(aPos, 0.0, 1.0); vTc = (uTexM * vec4(aPos * 0.5 + 0.5, 0.0, 1.0)).xy; }\n";
        String fs = "#extension GL_OES_EGL_image_external : require\nprecision highp float;\n"
                + "varying highp vec2 vTc; uniform samplerExternalOES uTex;\n"
                + "void main() { gl_FragColor = texture2D(uTex, vTc); }\n";
        prog = GLES20.glCreateProgram();
        GLES20.glAttachShader(prog, shader(GLES20.GL_VERTEX_SHADER, vs));
        GLES20.glAttachShader(prog, shader(GLES20.GL_FRAGMENT_SHADER, fs));
        GLES20.glLinkProgram(prog);
        int[] ok = new int[1];
        GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, ok, 0);
        if (ok[0] == 0) throw new IllegalStateException("link: " + GLES20.glGetProgramInfoLog(prog));
        int[] hp = new int[2], rg = new int[2];
        GLES20.glGetShaderPrecisionFormat(GLES20.GL_FRAGMENT_SHADER, GLES20.GL_HIGH_FLOAT, rg, 0, hp, 0);
        if (hp[0] < 23) throw new IllegalStateException("fragment highp float precision " + hp[0] + " bits < 23");
        aPos = GLES20.glGetAttribLocation(prog, "aPos");
        uTexM = GLES20.glGetUniformLocation(prog, "uTexM");
        float[] q = {-1, -1, 1, -1, -1, 1, 1, 1};
        quad = ByteBuffer.allocateDirect(q.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        quad.put(q).position(0);
        int[] t = new int[1];
        GLES20.glGenTextures(1, t, 0);
        tex = t[0];
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
        st = new SurfaceTexture(tex);
        st.setDefaultBufferSize(inW, inH);
        st.setOnFrameAvailableListener(s -> draw(), h);
        camSurface = new Surface(st);
    }

    private static int shader(int type, String src) {
        int s = GLES20.glCreateShader(type);
        GLES20.glShaderSource(s, src);
        GLES20.glCompileShader(s);
        int[] ok = new int[1];
        GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0);
        if (ok[0] == 0) throw new IllegalStateException("shader: " + GLES20.glGetShaderInfoLog(s));
        return s;
    }

    /** GL thread: one camera frame -> one encoder frame. */
    private void draw() {
        try {
            st.updateTexImage();
            if (stopped) { nSkipped++; return; }          // after stop(): latch (frees the camera buffer) but feed nothing
            st.getTransformMatrix(m);
            long ts = st.getTimestamp();
            if (!clockKnown) {                             // which clock is the SurfaceTexture timestamp in?
                long boot = SystemClock.elapsedRealtimeNanos(), mono = System.nanoTime();
                // the frame was exposed shortly BEFORE now: pick the clock where now - ts is in [0, 0.5 s] (review R3)
                boolean bootOk = boot - ts >= 0 && boot - ts < 500_000_000L, monoOk = mono - ts >= 0 && mono - ts < 500_000_000L;
                boolean isBoot = bootOk && !monoOk ? true : monoOk && !bootOk ? false : Math.abs(boot - ts) < Math.abs(mono - ts);
                tsShiftNs = isBoot ? clockOffsetNs : 0;     // pts must be MONOTONIC like the direct camera -> encoder path
                clock = (isBoot ? "BOOTTIME" : "MONOTONIC") + String.format(Locale.US, " (ts-boot %.1f ms, ts-mono %.1f ms)",
                        (ts - boot) / 1e6, (ts - mono) / 1e6);
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < 16; i++) sb.append(i > 0 ? " " : "").append(m[i]);
                firstMatrix = sb.toString();
                // ok = the 2x2 block is a rotation/flip (entries 0 / +-1) and the translation 0 / 1: no crop, no scale
                matrixOk = true;
                for (int k : new int[]{0, 1, 4, 5}) matrixOk &= Math.abs(m[k]) < 1e-6 || Math.abs(Math.abs(m[k]) - 1) < 1e-6;
                for (int k : new int[]{12, 13}) matrixOk &= Math.abs(m[k]) < 1e-6 || Math.abs(m[k] - 1) < 1e-6;
                matrixOk &= Math.abs(Math.abs(m[0] * m[5] - m[1] * m[4]) - 1) < 1e-6;
                if (!matrixOk && err == null) err = "SurfaceTexture matrix has crop/scale: " + firstMatrix;
                clockKnown = true;
                android.util.Log.i("arlat", "gl downscale first frame: clock " + clock + " matrix " + firstMatrix + " ok " + matrixOk);
            }
            GLES20.glViewport(0, 0, outW, outH);
            GLES20.glUseProgram(prog);
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
            GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex);
            GLES20.glUniformMatrix4fv(uTexM, 1, false, FLIP, 0);
            GLES20.glEnableVertexAttribArray(aPos);
            GLES20.glVertexAttribPointer(aPos, 2, GLES20.GL_FLOAT, false, 8, quad);
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
            boolean ok = EGLExt.eglPresentationTimeANDROID(dpy, surf, ts - tsShiftNs) & EGL14.eglSwapBuffers(dpy, surf);
            if (ok) nDrawn++;
            else if (nSwapFail++ == 0 && err == null) err = "eglSwapBuffers/eglPresentationTime failed: 0x" + Integer.toHexString(EGL14.eglGetError());
        } catch (Throwable e) {
            if (err == null) { err = e.toString(); android.util.Log.e("arlat", "gl downscale", e); }
        }
    }

    /** Stop feeding the encoder; returns after the GL thread has passed its current frame (then signalEndOfInputStream is safe). */
    void stopFeeding() {
        stopped = true;
        if (h == null) return;
        final CountDownLatch done = new CountDownLatch(1);
        h.post(done::countDown);
        try { done.await(500, TimeUnit.MILLISECONDS); } catch (InterruptedException ignored) { }
    }

    /** Frees EGL + SurfaceTexture on the GL thread and WAITS for it (max 1 s) so the encoder is released after its EGL surface. */
    void release() {
        stopped = true;
        if (h == null) return;
        final CountDownLatch done = new CountDownLatch(1);
        h.post(() -> {
            try {
                if (camSurface != null) camSurface.release();
                if (st != null) st.release();
                if (dpy != EGL14.EGL_NO_DISPLAY) {
                    EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
                    if (surf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, surf);
                    if (ctx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(dpy, ctx);
                    EGL14.eglReleaseThread();
                    EGL14.eglTerminate(dpy);
                }
            } catch (Throwable ignored) { }
            dpy = EGL14.EGL_NO_DISPLAY;
            done.countDown();
        });
        try { done.await(1, TimeUnit.SECONDS); } catch (InterruptedException ignored) { }
        th.quitSafely();
    }

    JSONObject json() {
        JSONObject j = new JSONObject();
        try {
            j.put("method", "camera " + inW + "x" + inH + " -> SurfaceTexture -> GLES2 bilinear at output pixel centres (= 2x2 box) -> encoder "
                    + outW + "x" + outH).put("camera_size", inW + "x" + inH).put("output_size", outW + "x" + outH)
                    .put("coord_map", "x_out = (x_in + 0.5) * " + ((double) outW / inW) + " - 0.5 (pixel centres at integers), same for y")
                    .put("texture_matrix_first", firstMatrix).put("matrix_ok", matrixOk)
                    .put("sampling_matrix", "fixed flip-only s=x t=1-y (SurfaceTexture matrix logged, not used)")
                    .put("buffer_clock", clock).put("pts_shift_ns", tsShiftNs)
                    .put("frames_drawn", nDrawn).put("swap_failures", nSwapFail).put("frames_after_stop", nSkipped).put("error", err == null ? "none" : err);
        } catch (Exception ignored) { }
        return j;
    }
}
