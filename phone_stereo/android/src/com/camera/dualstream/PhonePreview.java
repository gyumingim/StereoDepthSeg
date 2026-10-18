package com.camera.dualstream;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.view.Gravity;
import android.view.Surface;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.TextView;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Objdetect;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 1.14-preview: on-phone view of the recorded lens (gray, every frame = camera rate) + live ChArUco marker check
 * (OpenCV ArucoDetector, DICT_6X6_250 = scan3d/config/charuco/board_user_A4_6x6.json) drawn on top, so the person
 * recording ground truth for the IMU CNN sees right away whether the board is detected. One lens only (mono sessions).
 * The picture is the analysed Y plane itself (half size), not a camera preview stream: Samsung's preview stream is
 * cropped differently from the YUV stream, so outlines drawn from YUV detections landed beside the markers.
 * Detection runs on its own thread on the newest frame; frames arriving while it is busy are skipped.
 * Portrait screen, back camera sensor rotated 90 deg clockwise.
 */
class PhonePreview {
    final FrameLayout root;
    final Surface surface = null;              // no extra camera output (kept for MainActivity's hooks)
    private final View overlay;
    private final int w, h, hw, hh;
    private final Handler det;
    private volatile boolean busy;
    private final ArucoDetector detector;
    private final Bitmap[] bmp = new Bitmap[2];
    private volatile int front;
    private volatile float[][] quads = new float[0][];
    private volatile int nMarkers;
    private volatile long lastOkMs;
    private volatile double camFps, detFps, detMs;
    private long camT0, detT0; private int camN, detN;

    PhonePreview(Activity act, TextView status, int width, int height) {
        w = width; h = height; hw = w / 2; hh = h / 2;
        System.loadLibrary("opencv_java4");
        detector = new ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_6X6_250), new DetectorParameters());
        for (int i = 0; i < 2; i++) bmp[i] = Bitmap.createBitmap(hw, hh, Bitmap.Config.ALPHA_8);
        root = new FrameLayout(act);
        root.setBackgroundColor(Color.BLACK);
        root.setFitsSystemWindows(true);                     // keep the text below the status bar
        int sw = act.getResources().getDisplayMetrics().widthPixels;
        int vh = (int) ((long) sw * w / h);                 // portrait: width = screen, height = sw * W / H
        overlay = new Overlay(act);
        root.addView(overlay, new FrameLayout.LayoutParams(sw, vh, Gravity.TOP));
        status.setTextSize(13); status.setTextColor(Color.WHITE); status.setPadding(24, 16, 24, 16);
        root.addView(status, new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.BOTTOM));
        HandlerThread t = new HandlerThread("marker-check"); t.start();
        det = new Handler(t.getLooper());
    }

    boolean whenReady(Runnable r) { return true; }

    /** camera thread, every lens-0 frame (NV21, w x h). */
    void frame(byte[] nv21) {
        if (nv21 == null) return;
        long now = SystemClock.elapsedRealtime();
        if (camT0 == 0) camT0 = now;
        if (++camN >= 30) { camFps = camN * 1000.0 / Math.max(1, now - camT0); camN = 0; camT0 = now; }
        byte[] small = new byte[hw * hh];
        for (int y = 0; y < hh; y++) {
            int src = 2 * y * w, dst = y * hw;
            for (int x = 0; x < hw; x++) small[dst + x] = nv21[src + 2 * x];
        }
        int back = 1 - front;
        bmp[back].copyPixelsFromBuffer(ByteBuffer.wrap(small));
        front = back;
        overlay.postInvalidate();
        if (busy) return;
        busy = true;
        det.post(() -> detect(small));
    }

    private void detect(byte[] img) {
        long t0 = SystemClock.elapsedRealtime();
        try {
            Mat m = new Mat(hh, hw, CvType.CV_8UC1);
            m.put(0, 0, img);
            List<Mat> corners = new ArrayList<>();
            Mat ids = new Mat();
            detector.detectMarkers(m, corners, ids);
            float[][] q = new float[corners.size()][];
            for (int i = 0; i < corners.size(); i++) {
                float[] c = new float[8];
                corners.get(i).get(0, 0, c);
                q[i] = c;                                    // half-size image coordinates
                corners.get(i).release();
            }
            m.release(); ids.release();
            quads = q; nMarkers = q.length;
            long now = SystemClock.elapsedRealtime();
            if (q.length >= 4) lastOkMs = now;
            detMs = now - t0;
            if (detT0 == 0) detT0 = now;
            if (++detN >= 15) { detFps = detN * 1000.0 / Math.max(1, now - detT0); detN = 0; detT0 = now; }
        } catch (Exception ignored) {
        } finally {
            busy = false;
        }
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
            // half image (hw x hh, landscape) -> portrait view: (0,0)->(vw,0), (hw,0)->(vw,vh), (0,hh)->(0,0)
            M.setPolyToPoly(new float[]{0, 0, hw, 0, 0, hh}, 0, new float[]{vw, 0, vw, vh, 0, 0}, 0, 3);
            c.drawColor(Color.BLACK);
            c.drawBitmap(bmp[front], M, img);
            Path p = new Path();
            for (float[] q : quads) {
                System.arraycopy(q, 0, pts, 0, 8);
                M.mapPoints(pts);
                p.reset(); p.moveTo(pts[0], pts[1]); p.lineTo(pts[2], pts[3]); p.lineTo(pts[4], pts[5]); p.lineTo(pts[6], pts[7]); p.close();
                c.drawPath(p, line);
            }
            long now = SystemClock.elapsedRealtime();
            boolean ok = nMarkers >= 4;
            String head = ok ? "보정판 인식 ✓ " + nMarkers
                    : (lastOkMs > 0 ? String.format(Locale.US, "안 보임 %.1f s", (now - lastOkMs) / 1000.0) : "안 보임");
            big.setColor(ok ? Color.rgb(34, 197, 94) : Color.rgb(239, 68, 68));
            c.drawRect(0, 0, vw, 160, box);
            c.drawText(head, 24, 90, big);
            c.drawText(String.format(Locale.US, "카메라 %.0f fps · 인식 %.0f fps (%.0f ms)", camFps, detFps, detMs), 24, 145, small);
        }
    }
}
