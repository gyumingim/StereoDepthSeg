package com.scan3d.arlat;

import android.app.Activity;
import android.os.SystemClock;
import android.view.Gravity;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * 0.6.33 (user 10-28, error test): big Korean on-screen cue for a timed mode-ov run (intent secs, no guide). Same in npu 0
 * and npu 1 runs; no sound / vibration; one TextView pair updated at 2 Hz on the UI thread (reads two volatile fields of
 * OvRunner only). Phases (OpenVINS static init = still, then the first motion starts it):
 *   before the first frame is fed (auto ISO) and 3 s after it: "준비 중 — 가만히"
 *   then "출발 — 천천히 한 바퀴" + seconds left (if VIO is still not tracking 6 s after "출발": "초기화 안 됨 — 멈췄다 다시 출발")
 *   last 15 s: "출발 표시로 돌아와 멈추기", last 3 s: "끝 — 가만히 3초", at the stop: "저장 중", after the runner stopped: "완료".
 * Small line: VIO 상태 (추적 중 / 초기화 전 / 잃음) + NPU 상태 (꺼짐 / 열림 / 거부 / 무효).
 */
final class RunCue {
    final LinearLayout root;
    private final TextView big, small;
    private final OvRunner ov;
    private final long t0, secsMs;
    private long fedMs = -1, goMs = -1;
    private volatile boolean done;
    private final Runnable tick = new Runnable() { @Override public void run() { update(); if (!done) big.postDelayed(this, 500); } };

    RunCue(Activity act, OvRunner ov, int secs) {
        this.ov = ov;
        t0 = SystemClock.elapsedRealtime();
        secsMs = secs * 1000L;
        root = new LinearLayout(act);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setGravity(Gravity.CENTER);
        root.setBackgroundColor(0xFF000000);
        big = new TextView(act);
        big.setTextSize(44);
        big.setTextColor(0xFFFFFFFF);
        big.setGravity(Gravity.CENTER);
        small = new TextView(act);
        small.setTextSize(20);
        small.setTextColor(0xFFBBBBBB);
        small.setGravity(Gravity.CENTER);
        small.setPadding(0, 60, 0, 0);
        root.addView(big);
        root.addView(small);
        big.setText("준비 중 — 가만히");
        big.post(tick);
    }

    void saving() { done = true; big.setText("저장 중"); root.setBackgroundColor(0xFF3A3A3A); }
    void finished() { done = true; big.setText("완료"); root.setBackgroundColor(0xFF000000); }

    private void update() {
        final long now = SystemClock.elapsedRealtime();
        final long left = secsMs - (now - t0);
        if (fedMs < 0 && ov.fedElapsedMs() > 0) fedMs = now;
        final String vs = ov.vioStatus();
        final boolean tracking = "tracking".equals(vs);
        String b;
        int bg;
        if (fedMs < 0 || now - fedMs < 3000) { b = "준비 중 — 가만히"; bg = 0xFF3A3A3A; }
        else if (left <= 3000) { b = "끝 — 가만히 3초"; bg = 0xFF7E22CE; }
        else if (left <= 15000) { b = "출발 표시로 돌아와 멈추기\n" + (left / 1000) + " 초"; bg = 0xFFC2410C; }
        else {
            if (goMs < 0) goMs = now;
            if (!tracking && now - goMs > 6000) { b = "초기화 안 됨 —\n멈췄다 다시 출발"; bg = 0xFFB91C1C; }
            else { b = "출발 — 천천히 한 바퀴\n" + (left / 1000) + " 초"; bg = 0xFF1D4ED8; }
        }
        big.setText(b);
        root.setBackgroundColor(bg);
        String v = tracking ? "추적 중" : "lost".equals(vs) ? "잃음" : "초기화 전";
        small.setText("VIO " + v + "   |   NPU " + ov.npuCue());
    }
}
