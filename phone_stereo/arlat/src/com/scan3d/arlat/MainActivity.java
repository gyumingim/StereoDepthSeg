package com.scan3d.arlat;

import android.app.Activity;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.os.Bundle;
import android.os.SystemClock;
import android.widget.TextView;

import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Locale;

/**
 * ARCore latency probe. A GL thread loops Session.update() and logs per new frame:
 * frame timestamp (camera), android camera timestamp, elapsedRealtimeNanos before/after update(), tracking state, pose
 * -> files/arlat_<ms>.csv. Latency = t_return - t_frame, both on the phone clock.
 * Run length: intent extra "secs" (default 30).
 */
public class MainActivity extends Activity {
    private Session session;
    private volatile boolean running;
    private Thread worker;
    private TextView status;
    private Cam2Probe cam2;

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        status = new TextView(this);
        status.setTextSize(18);
        status.setPadding(40, 120, 40, 40);
        setContentView(status);
    }

    @Override
    protected void onResume() {
        super.onResume();
        final int secs = getIntent().getIntExtra("secs", 30);
        String mode = getIntent().getStringExtra("mode");
        if ("cam2".equals(mode)) {          // camera2 delivery latency of the VIO lens instead of ARCore
            if (cam2 == null) {
                cam2 = new Cam2Probe(this, this::show);
                String phys = getIntent().getStringExtra("phys");
                try { cam2.start(secs, phys == null ? "2" : phys); }
                catch (Exception e) { status.setText("cam2 error: " + e); }
            }
            return;
        }
        try {
            if (session == null) {
                session = new Session(this);
                Config c = new Config(session);
                c.setUpdateMode(Config.UpdateMode.BLOCKING);
                c.setFocusMode(Config.FocusMode.FIXED);
                session.configure(c);
            }
            session.resume();
        } catch (Exception e) {
            status.setText("ARCore error: " + e);
            android.util.Log.e("arlat", "start", e);
            return;
        }
        running = true;
        worker = new Thread(() -> loop(secs), "arlat");
        worker.start();
    }

    @Override
    protected void onPause() {
        running = false;
        if (cam2 != null) { cam2.stop(); cam2 = null; }
        if (worker != null) {
            try { worker.join(2000); } catch (InterruptedException ignored) { }
        }
        if (session != null) session.pause();
        super.onPause();
    }

    private void show(final String s) {
        runOnUiThread(() -> status.setText(s));
    }

    private void loop(int secs) {
        EGLDisplay dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
        int[] ver = new int[2];
        EGL14.eglInitialize(dpy, ver, 0, ver, 1);
        int[] attr = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_NONE};
        EGLConfig[] cfg = new EGLConfig[1];
        int[] n = new int[1];
        EGL14.eglChooseConfig(dpy, attr, 0, cfg, 0, 1, n, 0);
        EGLContext ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT,
                new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
        EGLSurface surf = EGL14.eglCreatePbufferSurface(dpy, cfg[0],
                new int[]{EGL14.EGL_WIDTH, 16, EGL14.EGL_HEIGHT, 16, EGL14.EGL_NONE}, 0);
        EGL14.eglMakeCurrent(dpy, surf, surf, ctx);
        int[] tex = new int[1];
        GLES20.glGenTextures(1, tex, 0);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex[0]);
        session.setCameraTextureName(tex[0]);

        File out = new File(getExternalFilesDir(null), "arlat_" + System.currentTimeMillis() + ".csv");
        long tEnd = SystemClock.elapsedRealtime() + secs * 1000L;
        int frames = 0, tracking = 0;
        long lastTs = 0;
        try (PrintWriter w = new PrintWriter(new FileWriter(out))) {
            w.println("t_frame_ns,t_cam_android_ns,t_call_ns,t_return_ns,state,tx,ty,tz,qx,qy,qz,qw");
            while (running && SystemClock.elapsedRealtime() < tEnd) {
                long t0 = SystemClock.elapsedRealtimeNanos();
                Frame f = session.update();
                long t1 = SystemClock.elapsedRealtimeNanos();
                long ts = f.getTimestamp();
                if (ts == 0 || ts == lastTs) continue;
                lastTs = ts;
                TrackingState st = f.getCamera().getTrackingState();
                Pose p = f.getCamera().getPose();
                frames++;
                if (st == TrackingState.TRACKING) tracking++;
                w.println(String.format(Locale.US, "%d,%d,%d,%d,%s,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f", ts,
                        f.getAndroidCameraTimestamp(), t0, t1, st, p.tx(), p.ty(), p.tz(),
                        p.qx(), p.qy(), p.qz(), p.qw()));
                if (frames % 15 == 0) {
                    show(String.format(Locale.US, "frames %d tracking %d\nlatency %.1f ms\n%s", frames, tracking,
                            (t1 - ts) / 1e6, st));
                }
            }
        } catch (Exception e) {
            show("error: " + e);
            android.util.Log.e("arlat", "loop", e);
            return;
        }
        show("DONE " + frames + " frames -> " + out.getName());
        android.util.Log.i("arlat", "DONE " + out);
    }
}
