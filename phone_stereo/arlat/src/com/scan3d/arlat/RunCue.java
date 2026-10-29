package com.scan3d.arlat;

import android.app.Activity;
import android.os.SystemClock;
import android.view.Gravity;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * On-screen Korean cue for a timed mode-ov run (no guide). Same in npu 0 and npu 1 runs; no sound / vibration; one
 * TextView pair updated at 2 Hz on the UI thread (reads volatile fields of OvRunner only).
 * 0.6.34 (user 10-28 after R1: VIO never initialised because the user stood still as told):
 *   first frame + 3 s: "준비 중 — 가만히"
 *   then ALWAYS "출발 — 폰을 위아래로 살짝 흔든 뒤 천천히 걷기" (OpenVINS static init starts on the first motion)
 *     not tracking 20 s after that: "폰을 조금 더 크게 흔들기"
 *   VIO tracking: green "VIO 시작됨" (3 s), then "천천히 한 바퀴 + 남은 초" - the run length (runSecs) counts FROM VIO INIT;
 *     last 15 s "출발 표시로 돌아와 멈추기", last 3 s "끝 — 가만히 3초", then onEnd (the activity finishes: "저장 중" / "완료").
 *   Hard cap: the intent secs (MainActivity timer from the start) still ends the run if VIO never initialises.
 * Small line: VIO 추적 중 / 초기화 전 / 잃음, NPU 꺼짐 / 여는 중 / 열림 / 거부 / 무효.
 */
final class RunCue {
    final LinearLayout root;
    private final TextView big, small;
    private final OvRunner ov;
    private final long runMs;
    private final Runnable onEnd;
    private long fedMs = -1, goMs = -1, initMs = -1;
    private volatile boolean done;
    private final Runnable tick = new Runnable() { @Override public void run() { update(); if (!done) h.postDelayed(this, 500); } };
    private final android.os.Handler h = new android.os.Handler(android.os.Looper.getMainLooper());   // 0.6.38b: not a View

    /** @param runSecs run length counted from VIO init; onEnd is called once when it is over */
    RunCue(Activity act, OvRunner ov, int runSecs, Runnable onEnd) {
        this.ov = ov;
        this.onEnd = onEnd;
        runMs = runSecs * 1000L;
        root = new LinearLayout(act);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setGravity(Gravity.CENTER);
        root.setBackgroundColor(0xFF3A3A3A);
        big = new TextView(act);
        big.setTextSize(40);
        big.setTextColor(0xFFFFFFFF);
        big.setGravity(Gravity.CENTER);
        small = new TextView(act);
        small.setTextSize(20);
        small.setTextColor(0xFFDDDDDD);
        small.setGravity(Gravity.CENTER);
        small.setPadding(0, 60, 0, 0);
        root.addView(big);
        root.addView(small);
        big.setText("준비 중 — 가만히");
        h.post(tick);
    }

    void saving() { done = true; big.setText("저장 중"); root.setBackgroundColor(0xFF3A3A3A); }
    void finished() { done = true; big.setText("완료"); root.setBackgroundColor(0xFF000000); }

    private void update() {
        if (done) return;
        final long now = SystemClock.elapsedRealtime();
        if (fedMs < 0 && ov.fedElapsedMs() > 0) fedMs = now;
        final String vs = ov.vioStatus();
        final boolean tracking = "tracking".equals(vs);
        if (initMs < 0 && tracking) initMs = now;
        String b;
        int bg;
        if (initMs < 0) {
            if (fedMs < 0 || now - fedMs < 3000) { b = "준비 중 — 가만히"; bg = 0xFF3A3A3A; }
            else {
                if (goMs < 0) goMs = now;
                if (now - goMs > 20000) { b = "폰을 조금 더\n크게 흔들기"; bg = 0xFFB91C1C; }
                else { b = "출발 —\n폰을 위아래로 살짝 흔든 뒤\n천천히 걷기"; bg = 0xFF1D4ED8; }
            }
        } else {
            final long left = runMs - (now - initMs);
            final long ls = Math.max(0, (left + 999) / 1000);
            if (left <= 0) {
                done = true;
                big.setText("저장 중");
                root.setBackgroundColor(0xFF3A3A3A);
                if (onEnd != null) onEnd.run();
                return;
            }
            if (now - initMs < 3000) { b = "VIO 시작됨\n천천히 한 바퀴\n" + ls + " 초"; bg = 0xFF15803D; }
            else if (left <= 3000) { b = "끝 — 가만히 3초"; bg = 0xFF7E22CE; }
            else if (left <= 15000) { b = "출발 표시로 돌아와 멈추기\n" + ls + " 초"; bg = 0xFFC2410C; }
            else { b = "천천히 한 바퀴\n" + ls + " 초"; bg = 0xFF1D4ED8; }
        }
        big.setText(b);
        root.setBackgroundColor(bg);
        String v = tracking ? "추적 중" : "lost".equals(vs) ? "잃음" : (initMs < 0 ? "초기화 전" : vs);
        small.setText("VIO " + v + "   |   NPU " + ov.npuCue());
    }
}
