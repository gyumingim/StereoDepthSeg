package com.scan3d.arlat;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.view.View;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * 0.6.25 calcap: the phone's OWN live camera picture (decimated Y of the calibration stream, every frame) + the PC's guide
 * overlay ("OVL <json>", all coordinates in 1920x1080 sensor pixels), rotated by the sensor orientation to the portrait screen.
 * Overlay json: tgt = target board outline (projected at the slot pose, 4 pts), lbl = target label, det = detected board hull,
 * pts = detected corners, col = "green" | "orange" | "blue" | "none", n / N = hold progress, edge = edge margin px.
 */
class CalCapView extends View {
    private Bitmap bm;
    private int[] px;
    private int bw, bh;
    private int sensorOrientation = 90;
    private volatile JSONObject ovl;
    private final Matrix m = new Matrix(), mi = new Matrix();
    private final Paint pImg = new Paint(Paint.FILTER_BITMAP_FLAG);
    private final Paint pFill = new Paint(Paint.ANTI_ALIAS_FLAG), pLine = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pText = new Paint(Paint.ANTI_ALIAS_FLAG), pBg = new Paint();
    private final Path path = new Path();
    private long frames;

    CalCapView(Context c) {
        super(c);
        setBackgroundColor(0xFF000000);
        pLine.setStyle(Paint.Style.STROKE);
        pLine.setStrokeJoin(Paint.Join.ROUND);
        pText.setColor(Color.WHITE);
        pText.setFakeBoldText(true);
        pBg.setColor(0xB0000000);
    }

    void setSensorOrientation(int so) { sensorOrientation = so; postInvalidate(); }

    /** Called on the camera thread with a decimated gray frame (w x h, values 0..255). */
    void frame(byte[] y, int w, int h) {
        synchronized (this) {
            if (bm == null || bw != w || bh != h) { bm = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888); px = new int[w * h]; bw = w; bh = h; }
            for (int i = 0; i < w * h; i++) { int v = y[i] & 0xFF; px[i] = 0xFF000000 | (v << 16) | (v << 8) | v; }
            bm.setPixels(px, 0, w, 0, 0, w, h);
            frames++;
        }
        postInvalidate();
    }

    void overlay(JSONObject j) { ovl = j; postInvalidate(); }

    /** sensor (1920x1080) -> view coordinates: rotate by the sensor orientation, fit, center. */
    private void layoutMatrix() {
        m.reset();
        m.setRotate(sensorOrientation);
        RectF r = new RectF(0, 0, CalCapRunner.W, CalCapRunner.H);
        m.mapRect(r);
        m.postTranslate(-r.left, -r.top);
        float s = Math.min(getWidth() / r.width(), getHeight() / r.height());
        m.postScale(s, s);
        m.postTranslate((getWidth() - r.width() * s) / 2f, (getHeight() - r.height() * s) / 2f);
    }

    private boolean poly(JSONArray a) {
        path.reset();
        if (a == null || a.length() < 2) return false;
        float[] q = new float[2];
        for (int i = 0; i < a.length(); i++) {
            JSONArray p = a.optJSONArray(i);
            if (p == null) return false;
            q[0] = (float) p.optDouble(0); q[1] = (float) p.optDouble(1);
            m.mapPoints(q);
            if (i == 0) path.moveTo(q[0], q[1]); else path.lineTo(q[0], q[1]);
        }
        path.close();
        return true;
    }

    @Override protected void onDraw(Canvas c) {
        super.onDraw(c);
        layoutMatrix();
        float u = Math.max(1f, Math.min(getWidth(), getHeight()) / 400f);     // ~1 px of a 400 px wide screen
        synchronized (this) {
            if (bm != null) {
                mi.set(m);
                mi.preScale(CalCapRunner.W / (float) bw, CalCapRunner.H / (float) bh);
                c.drawBitmap(bm, mi, pImg);
            }
        }
        JSONObject o = ovl;
        if (o == null) return;
        String col = o.optString("col", "none");
        int cDet = "green".equals(col) ? 0xFF22C55E : "blue".equals(col) ? 0xFF3B82F6 : 0xFFF97316;
        // edge margin (thin gray)
        int e = o.optInt("edge", 0);
        if (e > 0) {
            JSONArray r = new JSONArray();
            try {
                r.put(new JSONArray().put(e).put(e)).put(new JSONArray().put(CalCapRunner.W - e).put(e))
                 .put(new JSONArray().put(CalCapRunner.W - e).put(CalCapRunner.H - e)).put(new JSONArray().put(e).put(CalCapRunner.H - e));
            } catch (Exception ignored) { }
            if (poly(r)) { pLine.setColor(0x80A0A0A0); pLine.setStrokeWidth(u); c.drawPath(path, pLine); }
        }
        // target: translucent yellow quad + solid outline (where the board must appear, with the slot's perspective)
        if (poly(o.optJSONArray("tgt"))) {
            pFill.setStyle(Paint.Style.FILL); pFill.setColor(0x60FFD400);
            c.drawPath(path, pFill);
            pLine.setColor(0xFFFFD400); pLine.setStrokeWidth(3 * u);
            c.drawPath(path, pLine);
        }
        // detected board on top
        if (poly(o.optJSONArray("det"))) {
            pLine.setColor(cDet); pLine.setStrokeWidth(3 * u);
            c.drawPath(path, pLine);
        }
        JSONArray pts = o.optJSONArray("pts");
        if (pts != null) {
            pFill.setColor(cDet);
            float[] q = new float[2];
            for (int i = 0; i < pts.length(); i++) {
                JSONArray p = pts.optJSONArray(i);
                if (p == null) continue;
                q[0] = (float) p.optDouble(0); q[1] = (float) p.optDouble(1);
                m.mapPoints(q);
                c.drawCircle(q[0], q[1], 2.5f * u, pFill);
            }
        }
        // frame border + hold progress (green) / capturing (blue)
        if ("green".equals(col) || "blue".equals(col)) {
            RectF img = new RectF(0, 0, CalCapRunner.W, CalCapRunner.H);
            m.mapRect(img);
            pLine.setColor(cDet); pLine.setStrokeWidth(5 * u);
            c.drawRect(img, pLine);
            int n = o.optInt("n", 0), nn = Math.max(1, o.optInt("N", 5));
            float f = "blue".equals(col) ? 1f : Math.min(1f, n / (float) nn);
            pFill.setColor(cDet);
            c.drawRect(img.left, img.bottom - 8 * u, img.left + img.width() * f, img.bottom, pFill);
        }
        // target label (top of the picture, unrotated)
        String lbl = o.optString("lbl", "");
        if (!lbl.isEmpty()) {
            pText.setTextSize(15 * u);
            float tw = pText.measureText(lbl);
            RectF img = new RectF(0, 0, CalCapRunner.W, CalCapRunner.H);
            m.mapRect(img);
            float x = img.left + 6 * u, y = img.top + 6 * u;
            c.drawRect(x, y, x + tw + 10 * u, y + 22 * u, pBg);
            pText.setColor(0xFFFFD400);
            c.drawText(lbl, x + 5 * u, y + 16.5f * u, pText);
        }
    }
}
