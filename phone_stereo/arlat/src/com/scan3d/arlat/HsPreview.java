package com.scan3d.arlat;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.SurfaceTexture;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.view.Gravity;
import android.view.Surface;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.TextView;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Locale;

/**
 * On-phone view + board check for the 240 fps high-speed recording. High-speed sessions only feed preview/encoder
 * surfaces (no CPU frames), so the camera also draws into a SurfaceTexture; a GL thread latches every frame and, about
 * 30 times a second, renders it to a small 360x640 (portrait) gray image (glReadPixels) that is shown on screen and searched for
 * the board markers (MarkerCheck, OpenCV ArUco in libarloop). Recording itself is untouched (encoder surface).
 */
class HsPreview {
    // the camera already rotates preview buffers upright (SurfaceTexture transform = sensor orientation), so the
    // small gray image is portrait 360x640 and is shown without further rotation (checked against the mp4 2026-10-19)
    static final int PW = 360, PH = 640;
    final FrameLayout root;
    private final View overlay;
    private final int camW, camH;
    private HandlerThread glT, detT;
    private Handler gl, det;
    private SurfaceTexture st;
    volatile Surface surface;
    private EGLDisplay dpy;
    private EGLSurface eglSurf;
    private EGLContext ctx;
    private int tex, prog, fbo, fboTex;
    private final float[] texM = new float[16];
    private long lastProcMs;
    private volatile boolean busy;
    private final Bitmap[] bmp = new Bitmap[2];
    private volatile int front;
    private volatile float[] quads = new float[1];
    private volatile long lastOkMs;
    private final java.util.ArrayDeque<long[]> hist = new java.util.ArrayDeque<>();   // {check time ms, board ok 1/0}
    private volatile double camFps, chkFps;
    private long camT0, chkT0; private int camN, chkN;

    HsPreview(Activity act, TextView status, int camW, int camH) {
        this.camW = camW; this.camH = camH;
        for (int i = 0; i < 2; i++) bmp[i] = Bitmap.createBitmap(PW, PH, Bitmap.Config.ALPHA_8);
        root = new FrameLayout(act);
        root.setBackgroundColor(Color.BLACK);
        root.setFitsSystemWindows(true);
        int sw = act.getResources().getDisplayMetrics().widthPixels;
        int vh = (int) ((long) sw * PH / PW);                // portrait: width = screen, height = sw * 16 / 9
        overlay = new Overlay(act);
        root.addView(overlay, new FrameLayout.LayoutParams(sw, vh, Gravity.TOP));
        if (status.getParent() != null) ((android.view.ViewGroup) status.getParent()).removeView(status);
        status.setTextSize(13); status.setTextColor(Color.WHITE); status.setPadding(24, 16, 24, 16);
        root.addView(status, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.BOTTOM));
        still = new TextView(act);
        still.setTextSize(64); still.setGravity(Gravity.CENTER); still.setTextColor(Color.YELLOW);
        still.setShadowLayer(12f, 0, 0, Color.BLACK); still.setVisibility(View.GONE);
        root.addView(still, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER));
    }

    // big on-screen countdown for the still start (the PC page is not visible while the monitor shows the board)
    static final int STILL_S = 10;
    private TextView still;
    private long recT0Ms;

    // 2:30 guided recording (still start, two set-downs, still end), then auto stop so no hand enters the view to stop it.
    // {end second, label, colour}; still phases yellow, motion green, set-down orange
    static final Object[][] GUIDE = {
        {10, "정지", Color.YELLOW},
        {55, "들고 움직이기 (보드 보면서)", Color.GREEN},
        {70, "세워서 내려놓고 정지", 0xFFFFA500},
        {120, "들고 움직이기 (보드 보면서)", Color.GREEN},
        {150, "세워서 내려놓고 정지", 0xFFFFA500},
    };
    static final int GUIDE_END_S = 150;

    /** Call when recording starts: big phase text + seconds left in the phase + next step; vibrates at each phase change;
     *  at GUIDE_END_S calls onEnd (auto stop). */
    void startGuide(final Activity act, final Runnable onEnd) {
        recT0Ms = SystemClock.elapsedRealtime();
        final int[] lastPhase = {-1};
        still.post(new Runnable() {
            @Override public void run() {
                long t = SystemClock.elapsedRealtime() - recT0Ms;
                int s = (int) (t / 1000);
                if (s >= GUIDE_END_S) {
                    still.setTextSize(48); still.setTextColor(Color.WHITE); still.setText("끝 — 자동으로 멈춤");
                    buzz(act, 600);
                    onEnd.run();
                    return;
                }
                int ph = 0;
                while ((Integer) GUIDE[ph][0] <= s) ph++;
                if (ph != lastPhase[0]) { if (lastPhase[0] >= 0) buzz(act, 250); lastPhase[0] = ph; }
                int left = (Integer) GUIDE[ph][0] - s;
                String next = ph + 1 < GUIDE.length ? "다음: " + GUIDE[ph + 1][1] : "다음: 자동 멈춤";
                still.setVisibility(View.VISIBLE);
                still.setTextSize(40); still.setTextColor((Integer) GUIDE[ph][2]);
                still.setText(GUIDE[ph][1] + "\n" + left + "\n" + next
                        + String.format(Locale.US, "\n녹화 %d:%02d / 2:30", s / 60, s % 60));
                still.postDelayed(this, 200);
            }
        });
    }

    static void buzz(Activity act, long ms) {
        try {
            android.os.Vibrator v = (android.os.Vibrator) act.getSystemService(android.content.Context.VIBRATOR_SERVICE);
            if (v != null) v.vibrate(android.os.VibrationEffect.createOneShot(ms, android.os.VibrationEffect.DEFAULT_AMPLITUDE));
        } catch (Throwable ignored) { }
    }

    /** Create the GL side; returns when the preview Surface exists (call before opening the camera session). */
    void start() throws InterruptedException {
        glT = new HandlerThread("hs-gl"); glT.start(); gl = new Handler(glT.getLooper());
        detT = new HandlerThread("hs-check"); detT.start(); det = new Handler(detT.getLooper());
        final Object lock = new Object();
        gl.post(() -> { initGl(); synchronized (lock) { lock.notifyAll(); } });
        synchronized (lock) { while (surface == null) lock.wait(2000); }
    }

    void stop() {
        if (gl != null) gl.post(() -> {
            try { if (st != null) st.release(); } catch (Exception ignored) { }
            EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
            EGL14.eglDestroyContext(dpy, ctx);
            glT.quitSafely();
        });
        if (detT != null) detT.quitSafely();
    }

    private void initGl() {
        dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
        int[] v = new int[2];
        EGL14.eglInitialize(dpy, v, 0, v, 1);
        int[] attr = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8, EGL14.EGL_NONE};
        EGLConfig[] cfg = new EGLConfig[1];
        int[] n = new int[1];
        EGL14.eglChooseConfig(dpy, attr, 0, cfg, 0, 1, n, 0);
        ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT, new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
        eglSurf = EGL14.eglCreatePbufferSurface(dpy, cfg[0], new int[]{EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE}, 0);
        EGL14.eglMakeCurrent(dpy, eglSurf, eglSurf, ctx);
        int[] t = new int[2];
        GLES20.glGenTextures(2, t, 0);
        tex = t[0]; fboTex = t[1];
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, fboTex);
        GLES20.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, GLES20.GL_RGBA, PW, PH, 0, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, null);
        int[] f = new int[1];
        GLES20.glGenFramebuffers(1, f, 0);
        fbo = f[0];
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, fbo);
        GLES20.glFramebufferTexture2D(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_TEXTURE_2D, fboTex, 0);
        prog = program(
                "attribute vec2 p; uniform mat4 M; varying vec2 uv; void main(){ gl_Position=vec4(p,0.,1.);"
                        + " uv=(M*vec4((p+1.)*0.5,0.,1.)).xy; }",
                "#extension GL_OES_EGL_image_external : require\nprecision mediump float; uniform samplerExternalOES s;"
                        + " varying vec2 uv; void main(){ vec3 c=texture2D(s,uv).rgb; float y=dot(c,vec3(.299,.587,.114));"
                        + " gl_FragColor=vec4(y,y,y,1.); }");
        st = new SurfaceTexture(tex);
        st.setDefaultBufferSize(camW, camH);
        st.setOnFrameAvailableListener(s -> onFrame(), gl);
        surface = new Surface(st);
    }

    private static int program(String vs, String fs) {
        int v = GLES20.glCreateShader(GLES20.GL_VERTEX_SHADER); GLES20.glShaderSource(v, vs); GLES20.glCompileShader(v);
        int f = GLES20.glCreateShader(GLES20.GL_FRAGMENT_SHADER); GLES20.glShaderSource(f, fs); GLES20.glCompileShader(f);
        int p = GLES20.glCreateProgram(); GLES20.glAttachShader(p, v); GLES20.glAttachShader(p, f); GLES20.glLinkProgram(p);
        return p;
    }

    private final FloatBuffer quad = ByteBuffer.allocateDirect(32).order(ByteOrder.nativeOrder()).asFloatBuffer()
            .put(new float[]{-1, -1, 1, -1, -1, 1, 1, 1});
    private final ByteBuffer px = ByteBuffer.allocateDirect(PW * PH * 4).order(ByteOrder.nativeOrder());

    /** GL thread: every camera frame (240/s). Latch it; ~30 times a second turn it into a small gray image. */
    private void onFrame() {
        try { st.updateTexImage(); } catch (Exception e) { return; }
        long now = SystemClock.elapsedRealtime();
        if (camT0 == 0) camT0 = now;
        if (++camN >= 30) { camFps = camN * 1000.0 / Math.max(1, now - camT0); camN = 0; camT0 = now; }
        // SurfaceTexture coalesces frames (~30 callbacks/s at 240 fps), so use each callback, at most ~30/s
        if (now - lastProcMs < 30) return;
        lastProcMs = now;
        st.getTransformMatrix(texM);
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, fbo);
        GLES20.glViewport(0, 0, PW, PH);
        GLES20.glUseProgram(prog);
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex);
        GLES20.glUniformMatrix4fv(GLES20.glGetUniformLocation(prog, "M"), 1, false, texM, 0);
        int pa = GLES20.glGetAttribLocation(prog, "p");
        quad.position(0);
        GLES20.glVertexAttribPointer(pa, 2, GLES20.GL_FLOAT, false, 0, quad);
        GLES20.glEnableVertexAttribArray(pa);
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
        px.position(0);
        GLES20.glReadPixels(0, 0, PW, PH, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, px);
        final byte[] g = new byte[PW * PH];
        for (int y = 0; y < PH; y++) {                   // GL rows are bottom-up -> flip; R channel = gray
            int src = (PH - 1 - y) * PW * 4, dst = y * PW;
            for (int x = 0; x < PW; x++) g[dst + x] = px.get(src + 4 * x);
        }
        int back = 1 - front;
        bmp[back].copyPixelsFromBuffer(ByteBuffer.wrap(g));
        front = back;
        overlay.postInvalidate();
        if (busy) return;
        busy = true;
        det.post(() -> {
            try {
                float[] r = MarkerCheck.nDetect(g, PW, PH);
                quads = r;
                long t = SystemClock.elapsedRealtime();
                if (r[0] >= 4) lastOkMs = t;
                synchronized (hist) { hist.addLast(new long[]{t, r[0] >= 4 ? 1 : 0}); while (hist.size() > 120) hist.removeFirst(); }
                if (chkT0 == 0) chkT0 = t;
                if (++chkN >= 15) { chkFps = chkN * 1000.0 / Math.max(1, t - chkT0); chkN = 0; chkT0 = t; }
            } catch (Throwable ignored) {
            } finally { busy = false; }
        });
    }

    private class Overlay extends View {
        final Paint img = new Paint(Paint.FILTER_BITMAP_FLAG), line = new Paint(Paint.ANTI_ALIAS_FLAG),
                big = new Paint(Paint.ANTI_ALIAS_FLAG), small = new Paint(Paint.ANTI_ALIAS_FLAG), box = new Paint();
        final Matrix M = new Matrix();
        final float[] pts = new float[8];
        Overlay(Activity a) {
            super(a);
            img.setColor(Color.WHITE);
            line.setStyle(Paint.Style.STROKE); line.setStrokeWidth(6); line.setColor(Color.rgb(34, 197, 94));
            big.setTextSize(64); big.setFakeBoldText(true);
            small.setTextSize(36); small.setColor(Color.WHITE);
            box.setColor(Color.argb(150, 0, 0, 0));
        }
        @Override protected void onDraw(Canvas c) {
            float vw = getWidth(), vh = getHeight();
            M.setPolyToPoly(new float[]{0, 0, PW, 0, 0, PH}, 0, new float[]{0, 0, vw, 0, 0, vh}, 0, 3);
            c.drawColor(Color.BLACK);
            c.drawBitmap(bmp[front], M, img);
            float[] q = quads;
            int n = (int) q[0];
            Path p = new Path();
            for (int i = 0; i < n && 1 + 8 * i + 7 < q.length; i++) {
                System.arraycopy(q, 1 + 8 * i, pts, 0, 8);
                M.mapPoints(pts);
                p.reset(); p.moveTo(pts[0], pts[1]); p.lineTo(pts[2], pts[3]); p.lineTo(pts[4], pts[5]); p.lineTo(pts[6], pts[7]); p.close();
                c.drawPath(p, line);
            }
            long now = SystemClock.elapsedRealtime();
            boolean ok = n >= 4;
            String head = ok ? "보정판 인식 ✓ " + n
                    : (lastOkMs > 0 ? String.format(Locale.US, "안 보임 %.1f s", (now - lastOkMs) / 1000.0) : "안 보임");
            big.setColor(ok ? Color.rgb(34, 197, 94) : Color.rgb(239, 68, 68));
            int tot = 0, miss = 0;                          // board checks in the last second and how many missed it
            synchronized (hist) { for (long[] e : hist) if (now - e[0] <= 1000) { tot++; if (e[1] == 0) miss++; } }
            c.drawRect(0, 0, vw, 215, box);
            c.drawText(head, 24, 90, big);
            small.setColor(miss == 0 ? Color.WHITE : Color.rgb(251, 191, 36));
            c.drawText(String.format(Locale.US, "최근 1초 놓침 %d / %d 번 확인", miss, tot), 24, 145, small);
            small.setColor(Color.WHITE);
            c.drawText(String.format(Locale.US, "화면 %.0f fps · 인식 확인 %.0f fps (녹화는 240 그대로)", camFps, chkFps), 24, 195, small);
        }
    }
}
