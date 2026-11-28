package com.scan3d.arlat;

import android.app.Activity;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.os.BatteryManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.view.Gravity;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.Locale;

/**
 * 0.6.42 (2026-11-28, user): launcher screen with ONE start/stop button for a 3D-map recording made by hand.
 * Same recorder and same files as the PC intent path (mode hs, session std): HsRecorder with
 * main camera 0 / physical 5, 1920x1080, 30 fps, fixed focus 0.5 dpt, exposure 2000 us + auto ISO once at start,
 * 160 Mbps, IMU + temps.csv, overheat auto stop (battery 46.0 C / thermal CRITICAL) -> files/hs_<time>_std30/.
 * No camera preview (encoder output only: no extra GL / marker-check load).
 * Stop = HsRecorder.stop() (the same as MainActivity.onPause): meta.json stop_reason "user (button)".
 * Leaving the screen (home, screen off, another app) also stops and saves (stop_reason "user (left app)").
 * Own task affinity: the icon never re-delivers an old PC intent of MainActivity (mode hs/ov from history).
 */
public class RecActivity extends Activity {
    static final int FPS = 30;
    static final float FOCUS_DPT = 0.5f;
    static final int EXP_US = 2000;
    static final int KBPS = 160_000;

    private enum St { IDLE, STARTING, RECORDING, STOPPING }
    private St st = St.IDLE;
    private HsRecorder hs;
    private Button btn;
    private TextView info, status;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private HandlerThread tempT;
    private Handler tempH;
    private volatile String skin = "?";
    private long t0Ms, lastTapMs;
    private int lastFrames; private long lastFrameMs; private double fps;
    private String lastMsg = "";

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        LinearLayout ll = new LinearLayout(this);
        ll.setOrientation(LinearLayout.VERTICAL);
        ll.setBackgroundColor(Color.BLACK);
        ll.setPadding(48, 120, 48, 48);
        ll.setGravity(Gravity.CENTER_HORIZONTAL);
        TextView title = new TextView(this);
        title.setText("3D 지도 녹화 (30 fps)");
        title.setTextColor(Color.WHITE); title.setTextSize(22); title.setGravity(Gravity.CENTER);
        ll.addView(title, new LinearLayout.LayoutParams(-1, -2));
        btn = new Button(this);
        btn.setTextSize(40);
        btn.setOnClickListener(v -> tap());
        LinearLayout.LayoutParams bp = new LinearLayout.LayoutParams(-1, 520);
        bp.topMargin = 60; bp.bottomMargin = 40;
        ll.addView(btn, bp);
        info = new TextView(this);
        info.setTextColor(Color.WHITE); info.setTextSize(22); info.setGravity(Gravity.CENTER);
        ll.addView(info, new LinearLayout.LayoutParams(-1, -2));
        TextView tip = new TextView(this);
        tip.setText("시작·끝 2초 가만히 / 폰은 진행 방향에서 30–45° 옆 / 천천히 회전");
        tip.setTextColor(Color.YELLOW); tip.setTextSize(18); tip.setGravity(Gravity.CENTER); tip.setPadding(0, 40, 0, 40);
        ll.addView(tip, new LinearLayout.LayoutParams(-1, -2));
        status = new TextView(this);
        status.setTextColor(0xFFAAAAAA); status.setTextSize(14); status.setGravity(Gravity.CENTER);
        ll.addView(status, new LinearLayout.LayoutParams(-1, -2));
        setContentView(ll);
        render();
    }

    @Override
    protected void onResume() {
        super.onResume();
        tempT = new HandlerThread("rec-temp"); tempT.start(); tempH = new Handler(tempT.getLooper());
        tempH.post(tempTick);
        ui.post(tick);
        render();
    }

    @Override
    protected void onPause() {
        // leaving the screen = stop and save (same as the PC path: onPause -> HsRecorder.stop())
        if (hs != null && (st == St.RECORDING || st == St.STARTING)) stopRec("user (left app)");
        ui.removeCallbacks(tick);
        if (tempT != null) { tempT.quitSafely(); tempT = null; tempH = null; }
        super.onPause();
    }

    private void tap() {
        long now = SystemClock.elapsedRealtime();
        if (now - lastTapMs < 1000) return;          // double tap guard
        lastTapMs = now;
        if (st == St.IDLE) startRec();
        else if (st == St.RECORDING) stopRec("user (button)");
        // STARTING / STOPPING: ignored
    }

    // a recorder still closing its files from an earlier activity instance (camera not free yet)
    private static volatile HsRecorder closing;

    private void startRec() {
        HsRecorder c = closing;
        if (c != null) { lastMsg = "앞 녹화 저장 중 — 잠시 후 다시"; render(); return; }
        HsRecorder r = new HsRecorder(this, this::onStatus);
        r.session = "std";
        r.wantFps = FPS;
        r.autoIso = true;
        r.isoScale = 1f;
        r.expUs = EXP_US;
        r.iso = 0;
        r.evComp = 0;
        r.wantKbps = KBPS;
        r.focusDpt = FOCUS_DPT;
        r.maxBatteryTenthC = 460;
        r.recUi = "button";
        r.previewSurface = null;
        r.onFinished = () -> { if (closing == r) closing = null; ui.post(() -> finishedRec(r)); };
        st = St.STARTING; hs = r;
        t0Ms = SystemClock.elapsedRealtime(); lastFrames = 0; lastFrameMs = t0Ms; fps = 0;
        lastMsg = "카메라 준비 중 (밝기 측정 약 1.5 초)";
        render();
        try {
            r.start();
            st = St.RECORDING;
            android.util.Log.i("arlat", "rec button: start " + r.folder());
        } catch (Exception e) {
            android.util.Log.e("arlat", "rec button start", e);
            try { r.stop(); } catch (Throwable ignored) { }
            hs = null; st = St.IDLE;
            lastMsg = "시작 실패: " + e;
        }
        render();
    }

    private void stopRec(String reason) {
        final HsRecorder r = hs;
        if (r == null) return;
        r.stopReason = reason;
        closing = r;
        ui.postDelayed(() -> { if (closing == r) closing = null; }, 8000);   // never block the next start for good
        st = St.STOPPING;
        lastMsg = "저장 중...";
        render();
        r.stop();                                    // encoder EOS -> finish(): files closed, meta.json, onFinished
        // safety: if finish() never comes (camera never opened), free the button after 6 s
        ui.postDelayed(() -> { if (hs == r && st == St.STOPPING) { android.util.Log.w("arlat", "rec button: no finish in 6 s");
            hs = null; st = St.IDLE; lastMsg = "저장 확인 안 됨 (" + r.folder() + ")"; render(); } }, 6000);
    }

    private void finishedRec(HsRecorder r) {
        if (hs != r) return;
        String why = r.stopReason;
        int n = r.frames();
        double secs = (SystemClock.elapsedRealtime() - t0Ms) / 1000.0;
        lastMsg = (why.startsWith("overheat") ? "과열로 자동 멈춤 — 지금까지 저장됨\n" : "저장됨\n")
                + r.folder() + String.format(Locale.US, "\n%d 프레임, %.0f 초", n, secs);
        android.util.Log.i("arlat", "rec button: done " + r.folder() + " frames " + n + " reason " + why);
        hs = null; st = St.IDLE;
        render();
    }

    private void onStatus(String s) {
        android.util.Log.i("arlat", "rec status: " + s.replace('\n', ' '));
    }

    private final Runnable tick = new Runnable() {
        @Override public void run() {
            HsRecorder r = hs;
            if (r != null && st == St.RECORDING) {
                long now = SystemClock.elapsedRealtime();
                int n = r.frames();
                if (now - lastFrameMs >= 900) {
                    fps = (n - lastFrames) * 1000.0 / (now - lastFrameMs);
                    lastFrames = n; lastFrameMs = now;
                }
                if (!r.isRecording()) lastMsg = "멈추는 중 (" + r.stopReason + ")";
                else if (n > 0) lastMsg = "녹화 중 → " + r.folder();
            }
            render();
            ui.postDelayed(this, 1000);
        }
    };

    private void render() {
        if (btn == null) return;
        switch (st) {
            case IDLE: btn.setText("녹화 시작"); btn.setBackgroundColor(0xFF16A34A); btn.setEnabled(true); break;
            case STARTING: btn.setText("준비 중..."); btn.setBackgroundColor(0xFF6B7280); btn.setEnabled(false); break;
            case RECORDING: btn.setText("녹화 끝"); btn.setBackgroundColor(0xFFDC2626); btn.setEnabled(true); break;
            case STOPPING: btn.setText("저장 중..."); btn.setBackgroundColor(0xFF6B7280); btn.setEnabled(false); break;
        }
        btn.setTextColor(Color.WHITE);
        StringBuilder s = new StringBuilder();
        if (st == St.RECORDING || st == St.STOPPING) {
            long e = (SystemClock.elapsedRealtime() - t0Ms) / 1000;
            HsRecorder r = hs;
            s.append(String.format(Locale.US, "%d:%02d   프레임 %d   %.1f fps\n", e / 60, e % 60, r == null ? 0 : r.frames(), fps));
        }
        s.append(String.format(Locale.US, "배터리 %.1f °C   SKIN %s °C", batteryC(), skin));
        info.setText(s);
        status.setText(lastMsg + "\n" + HsRecorder.VERSION);
    }

    private double batteryC() {
        Intent b = registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
        return b == null ? 0 : b.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) / 10.0;
    }

    // SKIN from the thermalservice dump (needs adb 'pm grant com.scan3d.arlat android.permission.DUMP', as temps.csv)
    private final Runnable tempTick = new Runnable() {
        @Override public void run() {
            skin = readSkin();
            Handler h = tempH;
            if (h != null) h.postDelayed(this, 3000);
        }
    };

    private static final java.util.regex.Pattern HAL_T = java.util.regex.Pattern.compile("mValue=([-0-9.eE]+).*?mName=([^,}]+)");

    static String readSkin() {
        android.os.ParcelFileDescriptor[] p = null;
        try {
            android.os.IBinder b = (android.os.IBinder) Class.forName("android.os.ServiceManager")
                    .getMethod("getService", String.class).invoke(null, "thermalservice");
            if (b == null) return "?";
            p = android.os.ParcelFileDescriptor.createPipe();
            b.dumpAsync(p[1].getFileDescriptor(), new String[0]);
            p[1].close(); p[1] = null;
            boolean in = false;
            try (java.io.BufferedReader r = new java.io.BufferedReader(new java.io.InputStreamReader(
                    new android.os.ParcelFileDescriptor.AutoCloseInputStream(p[0])))) {
                p[0] = null;
                for (String line; (line = r.readLine()) != null; ) {
                    if (line.contains("Current temperatures from HAL")) { in = true; continue; }
                    if (!in) continue;
                    if (!line.isEmpty() && !Character.isWhitespace(line.charAt(0))) break;
                    java.util.regex.Matcher m = HAL_T.matcher(line);
                    if (m.find() && "SKIN".equals(m.group(2).trim()))
                        return String.format(Locale.US, "%.1f", Float.parseFloat(m.group(1)));
                }
            }
            return "?";
        } catch (Throwable e) { return "?"; }
        finally {
            if (p != null) for (android.os.ParcelFileDescriptor f : p) if (f != null) try { f.close(); } catch (Exception ignored) { }
        }
    }
}
