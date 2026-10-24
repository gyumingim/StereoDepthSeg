package com.scan3d.arlat;

import android.app.Activity;
import android.graphics.Color;
import android.os.SystemClock;
import android.view.Gravity;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.Locale;

/**
 * 0.6.12: full-screen step guide for live OpenVINS (mode ov), readable from 1-2 m.
 * Intent extras as in hs/std (HsPreview.setGuide): guide = "label:secs;label:secs" (repeated until guide_secs), guide_secs.
 * Shows "밝기 맞추는 중…" until the first frame is fed to the bridge (after the auto-ISO phase), then counts down from that
 * moment: step label, seconds left in the step (huge), total left, next step; full-screen background colour by step order
 * (stepBg; black while waiting). Small live status underneath (MainActivity.show). Vibrates at each step change; at the end calls onEnd.
 */
class OvGuide {
    final LinearLayout root;
    private final TextView step, left, total, next;
    private final ArrayList<Object[]> steps = new ArrayList<>();   // {end second (Integer), label}
    final int totalS;
    final String spec;
    private long t0Ms = -1;

    /** Step colours by step order (user 2026-10-24), cycling: dark gray, blue, green, orange, purple; white text on all.
     *  Shared with the hs/std guide (HsPreview). Before the countdown (AE phase) / at the end: black. */
    static final int[] STEP_BG = {0xFF3A3A3A, 0xFF1D4ED8, 0xFF15803D, 0xFFC2410C, 0xFF7E22CE};
    static final int BG_WAIT = 0xFF000000;
    static int stepBg(int i) { return STEP_BG[i % STEP_BG.length]; }

    /** null when the spec is empty / invalid. */
    static OvGuide make(Activity act, String spec, int totalS) {
        if (spec == null || spec.trim().isEmpty() || totalS <= 0) return null;
        try { return new OvGuide(act, spec, totalS); } catch (Exception e) { android.util.Log.w("arlat", "guide " + spec, e); return null; }
    }

    private OvGuide(Activity act, String spec, int totalS) {
        this.spec = spec; this.totalS = totalS;
        String[] parts = spec.split(";");
        int t = 0, i = 0;
        while (t < totalS) {
            String p = parts[i % parts.length];
            int c = p.lastIndexOf(':');
            int d = Math.max(1, Integer.parseInt(p.substring(c + 1).trim()));
            t = Math.min(totalS, t + d);
            steps.add(new Object[]{t, p.substring(0, c).trim()});
            i++;
        }
        root = new LinearLayout(act);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setGravity(Gravity.CENTER_HORIZONTAL);
        root.setPadding(40, 140, 40, 40);
        root.setBackgroundColor(BG_WAIT);
        step = tv(act, 46, true); left = tv(act, 160, true); total = tv(act, 34, false); next = tv(act, 30, false);
        for (TextView v : new TextView[]{step, left, total, next}) root.addView(v);   // + MainActivity's small status view
        step.setText(act.getString(android.R.string.untitled).isEmpty() ? "" : "");
        waiting(true);
    }

    private static TextView tv(Activity act, int sp, boolean bold) {
        TextView v = new TextView(act);
        v.setTextSize(sp);
        v.setTextColor(Color.WHITE);
        v.setGravity(Gravity.CENTER_HORIZONTAL);
        if (bold) v.setTypeface(android.graphics.Typeface.DEFAULT_BOLD);
        return v;
    }

    /** Before the first fed frame: aeMetering = auto-ISO phase ("밝기 맞추는 중…"), else the camera is still opening. */
    void waiting(boolean aeMetering) {
        step.setText(aeMetering ? "밝기 맞추는 중…" : "카메라 여는 중…");
        left.setText("");
        total.setText(String.format(Locale.US, "곧 시작 (총 %d:%02d)", totalS / 60, totalS % 60));
        next.setText("첫 단계: " + steps.get(0)[1]);
    }

    boolean started() { return t0Ms >= 0; }

    /** Call on the UI thread when the first frame reaches the bridge: countdown starts now. */
    void start(final Activity act, final Runnable onEnd) {
        if (t0Ms >= 0) return;
        t0Ms = SystemClock.elapsedRealtime();
        final int[] lastPhase = {-1};
        root.post(new Runnable() {
            @Override public void run() {
                if (act.isFinishing()) return;
                long t = SystemClock.elapsedRealtime() - t0Ms;
                int s = (int) (t / 1000);
                if (s >= totalS) {
                    root.setBackgroundColor(BG_WAIT);
                    step.setText("끝 — 자동으로 멈춤"); left.setText(""); total.setText(""); next.setText("");
                    HsPreview.buzz(act, 600);
                    onEnd.run();
                    return;
                }
                int ph = 0;
                while ((Integer) steps.get(ph)[0] <= s) ph++;
                if (ph != lastPhase[0]) { if (lastPhase[0] >= 0) HsPreview.buzz(act, 250); lastPhase[0] = ph; }
                String label = (String) steps.get(ph)[1];
                root.setBackgroundColor(stepBg(ph));
                step.setText(label);
                left.setText(String.valueOf((Integer) steps.get(ph)[0] - s));
                int rem = totalS - s;
                total.setText(String.format(Locale.US, "전체 남은 %d:%02d", rem / 60, rem % 60));
                next.setText(ph + 1 < steps.size() ? "다음: " + steps.get(ph + 1)[1] : "다음: 자동 멈춤");
                root.postDelayed(this, 200);
            }
        });
    }
}
