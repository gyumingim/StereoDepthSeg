package com.camera.dualstream;

import org.json.*;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Locale;

/** Lens phase alignment (scan3d/SPEC.md 15). The two physical sensors of the logical camera free-run
 *  (LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE = APPROXIMATE on the S24): their exposure start times differ by a random
 *  phase and slowly slip. This class measures phase = target lens - reference lens mid-exposure time, wrapped to
 *  [-T/2, T/2), and pushes it inside a limit. Mid-exposure time (since 1.7-mid, SPEC 1.2 / 15.1) = physical
 *  SENSOR_TIMESTAMP + SENSOR_EXPOSURE_TIME/2 + SENSOR_ROLLING_SHUTTER_SKEW/2 of that lens (physical result values; the
 *  activity fills in the logical / last known value when a lens result has none). The start-of-exposure phase
 *  (physical SENSOR_TIMESTAMP difference, the 1.6-sync definition) is still measured and reported (phase_start_ms):
 *   - method C (auto, since 1.6-sync): close the CameraDevice and open it again (ImageReaders, encoders and the socket
 *     stay). Measured on the S24 (main session 2026-10-16): the sensors keep running while the device is open and the
 *     phase is decided when the device is opened (app restart = device close/open gave +26.2 / -1.16 / -10.26 ms),
 *     while a repeating-request restart or a new session did not move it. New phase is measured on 3..5 frames;
 *     |phase| < 2.5 ms -> locked. Max 25 tries per round. If the phase after an open is uniform in [-T/2, T/2) the
 *     chance per try is 5 ms / T: 15 fps 7.5 % (expected ~13 tries, 25 tries succeed with ~86 %), 30 fps 15 %
 *     (expected ~7 tries, 25 tries ~98 %). After 25 failed tries: rest 30 s, then again (no "best phase" stop: the
 *     next open gives a new phase anyway). Locked and |phase| > 4 ms -> again C (not while frames go into an MP4:
 *     state "drift", warning only).
 *   - method A (sync=a only): per-lens SENSOR_FRAME_DURATION on the target lens. S24 does not follow it (MEASURED_A).
 *   - method B (sync=b only): stop/start the repeating request, then recreate the session. S24: no phase change
 *     (MEASURED_B).
 *  All calls on the camera handler thread except json()/suffix() (any thread); everything is synchronized.
 *  Actions.changed() must not take other locks synchronously (MainActivity posts it, REVIEW8 L1). */
final class PhaseSync {
    interface Actions {
        /** Method A: target lens frame duration override (ns) with manual exposure/iso for that lens (expNs/iso <= 0 =
         *  unknown, then only the frame duration is set). durNs <= 0 = back to the normal request. false = failed. */
        boolean applyDuration(long durNs, long expNs, int iso);
        boolean restartRepeating();
        boolean recreateSession();
        /** Method C: close the CameraDevice and open it again, asynchronously (posted to the camera thread). The
         *  activity calls reopenClosed() right after the close, or cancelReopen() when it may not close. false = not
         *  posted. */
        boolean reopenCamera();
        /** false while frames go into an MP4 (recording gate open): no camera restart then (MP4 gap). */
        boolean mayRestart();
        /** state or shown phase changed -> status line suffix */
        void changed();
    }

    static final String S_OFF = "off", S_MEASURING = "measuring", S_ALIGNING = "aligning", S_LOCKED = "locked",
            S_FAILED = "failed", S_DRIFT = "drift";
    static final long MS = 1_000_000L;
    static final long A_LOCK_NS = 1 * MS, A_REALIGN_NS = 2 * MS;          // method A thresholds
    static final long B_LOCK_NS = 2_500_000L, B_REALIGN_NS = 4 * MS;      // method B thresholds
    static final long C_LOCK_NS = 2_500_000L, C_REALIGN_NS = 4 * MS;      // method C thresholds
    static final int C_MAX_TRIES = 25;                                    // per round (see class comment)
    static final long C_REST_NS = 30_000_000_000L;                        // rest after a failed round
    static final int C_SKIP = 2, C_EARLY = 3, C_SAMPLES = 5;              // frames skipped / early decision / full
    static final long C_EARLY_FAR_NS = 6 * MS;                            // 3 frames already this far -> next try
    static final long DRIFT_MIN_GAP_NS = 20_000_000_000L;                 // min time locked before a drift re-align
                                                                          // (30 fps slips ~4 ms/s; 15 fps needs > 20 s anyway)
    static final double A_MAX_STEP = 0.02;                                // max frame duration change (fraction of T)
    static final double A_GAIN = 0.15;                                    // P gain per frame (request pipeline ~4 frames)
    static final int B_MAX_TRIES = 10;
    static final long A_TEST_NS = 3_000_000_000L;                         // "request ignored?" test window
    static final long A_TIMEOUT_NS = 20_000_000_000L;
    static final long FAILED_COOLDOWN_NS = 60_000_000_000L;
    /** Main session measurements on the S24 (app 1.5-sync, 640x480 15 fps, 25 s, 2026-10-16) -> caps.json / metrics. */
    static final String MEASURED_A = "S24 2026-10-16 app 1.5-sync 15 fps: SENSOR_FRAME_DURATION and AE keys are physical "
            + "request keys but the phase did not follow (requested -58.7ms, moved 0.2ms in 3 s)";
    static final String MEASURED_B = "S24 2026-10-16 app 1.5-sync 15 fps: 1 repeating restart + 9 session re-creations, "
            + "phase 29.2 -> 30.2 ms (natural slip only); app restart (device close/open) changes the phase randomly";

    final int ref, tgt;                 // indexes into ids
    final String refId, tgtId;
    final long nominalNs;
    final String mode;                  // auto | c | a | b | off
    final boolean aPossible, aeKey;
    final String aReason;
    final long minDurNs, maxDurNs;      // target lens frame duration limits (0 = unknown)
    private final Actions act;

    private String state;
    private String method = null;       // "C" | "A" | "B" | "none" (already in phase at start)
    private boolean aFailed;
    private String aFailReason;
    private long adjustments, restarts, episodes, tries;
    private boolean useSession;         // stop/start repeating did not move the phase -> recreate the session instead
    private final ArrayDeque<long[]> samples = new ArrayDeque<>();   // {t, wrapped, unwrapped}
    private final ArrayDeque<long[]> free = new ArrayDeque<>();      // {t, unwrapped} without override -> slip
    private final ArrayDeque<Long> periods = new ArrayDeque<>();
    private long lastRefTs = -1, lastWrapped, lastUnwrapped;
    private boolean haveLast;
    private long settleUntil, cooldownUntil, startedAt, lockedAt;
    private int bSamples;
    private long phaseBefore;
    private double slopeNsPerNs;        // slip (phase change per ns of time)
    private boolean slopeKnown;
    private boolean havePhase; private long lastPhase;   // last median(9), kept while the camera re-opens
    private final ArrayDeque<Long> startSamples = new ArrayDeque<>();   // wrapped start-of-exposure phase (report only)
    private boolean haveStart; private long lastStart;   // last median of startSamples
    // method A
    private boolean overrideOn, restorePending;
    private long lastDurReq, nudgeT0, nudgeU0, nudgeErr0, alignT0;
    private double pSum;                // sum of the proportional part of the requested change (ns)
    private int sinceApply, good;
    private long tgtExp = -1; private int tgtIso = -1; private long tgtDur = -1;
    // method C
    private static final int C_IDLE = 0, C_REQUESTED = 1, C_CLOSED = 2, C_MEASURE = 3;
    private int cStage = C_IDLE, cSkip, cTries, cRounds, cLocks;
    private long reopens, reopenFails, cReqAt;
    private long cTryLast = -1, cTryMax, cTrySum, cTryN;     // request -> decision (ns)
    private long cOpenLast = -1, cOpenMax, cOpenSum, cOpenN; // request -> first result of the new device (ns)
    private long cLastAlign = -1;                            // align start -> lock of the last C lock (ns)
    private String holdReason;
    // shown values
    private long shownPhase = Long.MIN_VALUE; private long shownAt;
    private final StringBuilder events = new StringBuilder();
    private int nEvents;

    PhaseSync(String[] ids, int fps, String mode, Caps.MethodA a, long minDurNs, long maxDurNs, Actions act) {
        int t = 1;
        for (int i = 0; i < ids.length; i++) if (ids[i].equals("5")) t = i;
        tgt = t; ref = t == 0 ? 1 : 0;
        refId = ids[ref]; tgtId = ids[tgt];
        nominalNs = Math.round(1e9 / Math.max(1, fps));
        this.mode = mode;
        this.aPossible = a != null && a.possible; this.aeKey = a != null && a.aeKey;
        this.aReason = a == null ? "not checked" : a.reason;
        this.minDurNs = minDurNs; this.maxDurNs = maxDurNs;
        this.act = act;
        state = "off".equals(mode) ? S_OFF : S_MEASURING;
    }

    synchronized boolean enabled() { return !S_OFF.equals(state); }
    synchronized boolean locked() { return S_LOCKED.equals(state); }
    synchronized String state() { return state; }

    private long period() {
        if (periods.size() < 5) return nominalNs;
        long[] a = new long[periods.size()]; int i = 0;
        for (long p : periods) a[i++] = p;
        Arrays.sort(a);
        return a[a.length / 2];
    }

    private static long wrap(long d, long T) { return d - T * Math.round((double) d / T); }

    private long median(int n) {
        int k = Math.min(n, samples.size());
        if (k == 0) return 0;
        long[] a = new long[k]; int i = 0;
        java.util.Iterator<long[]> it = samples.descendingIterator();
        while (i < k && it.hasNext()) a[i++] = it.next()[1];
        Arrays.sort(a);
        return a[k / 2];
    }

    private void resetSamples() {
        samples.clear(); free.clear(); haveLast = false; bSamples = 0; startSamples.clear();
    }

    private long startMedian() {
        if (startSamples.isEmpty()) return lastStart;
        long[] a = new long[startSamples.size()]; int i = 0;
        for (long v : startSamples) a[i++] = v;
        Arrays.sort(a);
        return a[a.length / 2];
    }

    private void event(long now, String what) {
        if (nEvents >= 40) return;                     // first 40 only (meta.json / metrics stay small)
        nEvents++;
        if (events.length() > 0) events.append("; ");
        events.append(String.format(Locale.US, "%.1fs %s", (now - startedAt) / 1e9, what));
    }

    /** Methods that restart the camera (C, B) must not run while an MP4 is being written. */
    private boolean restartsCamera() { return !"a".equals(mode); }

    /** Camera thread, every logical capture result with both physical timestamps. tRef/tTgt: physical SENSOR_TIMESTAMP
     *  (start of exposure, first row); mRef/mTgt: mid-exposure time of the same frames (the phase uses these).
     *  exp/iso/dur: target lens physical result values (null = not reported). */
    synchronized void onResult(long tRef, long tTgt, long mRef, long mTgt, long now, Long exp, Integer iso, Long dur) {
        if (S_OFF.equals(state)) return;
        if (startedAt == 0) startedAt = now;
        if (S_ALIGNING.equals(state) && "C".equals(method)) {
            if (cStage == C_REQUESTED) return;          // old device, close not done yet
            if (cStage == C_CLOSED) {                   // first result of the re-opened device
                cStage = C_MEASURE;
                cOpenLast = now - cReqAt; cOpenMax = Math.max(cOpenMax, cOpenLast); cOpenSum += cOpenLast; cOpenN++;
                cSkip = C_SKIP; resetSamples(); periods.clear(); lastRefTs = -1;
            }
            if (cSkip > 0) { cSkip--; lastRefTs = tRef; return; }
        }
        if (restorePending && overrideOn) {             // REVIEW8 M5: retry going back to the normal request
            if (act.applyDuration(0, 0, 0)) { overrideOn = false; restorePending = false; event(now, "A restore ok"); }
        }
        if (lastRefTs > 0) {
            long d = tRef - lastRefTs;
            if (d > nominalNs / 2 && d < nominalNs * 3 / 2) { periods.addLast(d); if (periods.size() > 30) periods.pollFirst(); }
        }
        lastRefTs = tRef;
        if (!overrideOn) {                               // AE values of the target lens, used when A turns AE off
            if (exp != null) tgtExp = exp;
            if (iso != null) tgtIso = iso;
        }
        if (dur != null) tgtDur = dur;
        if (now < settleUntil) return;
        long T = period();
        startSamples.addLast(wrap(tTgt - tRef, T));
        if (startSamples.size() > 9) startSamples.pollFirst();
        haveStart = true; lastStart = startMedian();
        long w = wrap(mTgt - mRef, T);
        long u = haveLast ? lastUnwrapped + wrap(w - lastWrapped, T) : w;
        lastWrapped = w; lastUnwrapped = u; haveLast = true;
        samples.addLast(new long[]{now, w, u});
        if (samples.size() > 60) samples.pollFirst();
        if (!overrideOn) {
            free.addLast(new long[]{now, u});
            while (free.size() > 2 && now - free.peekFirst()[0] > 4_000_000_000L) free.pollFirst();
            updateSlope();
        }
        havePhase = true; lastPhase = median(9);
        switch (state) {
            case S_MEASURING:
                if (samples.size() >= 10) decide(now);
                break;
            case S_LOCKED: {
                long m = median(9);
                long thr = "A".equals(method) || "a".equals(mode) ? A_REALIGN_NS : "B".equals(method) ? B_REALIGN_NS : C_REALIGN_NS;
                if (samples.size() >= 5 && Math.abs(m) > thr) {
                    if (restartsCamera() && !act.mayRestart()) hold(now, String.format(Locale.US, "drift %.1fms while recording", m / 1e6));
                    else if (!restartsCamera() || now - lockedAt >= DRIFT_MIN_GAP_NS) {
                        event(now, String.format(Locale.US, "drift %.1fms", m / 1e6)); startAlign(now);
                    }
                }
                break;
            }
            case S_ALIGNING:
                if ("A".equals(method)) stepA(now, T); else if ("B".equals(method)) stepB(now); else stepC(now);
                break;
            case S_FAILED:
                if (now > cooldownUntil) { state = S_MEASURING; resetSamples(); act.changed(); }
                break;
            case S_DRIFT:                                // recording ended -> may restart the camera again
                if (act.mayRestart()) { event(now, "recording ended, re-align"); holdReason = null; startAlign(now); }
                break;
            default:
        }
        long m = median(9);
        if (S_ALIGNING.equals(state) && Math.abs(m - shownPhase) >= MS / 2 && now - shownAt > 1_000_000_000L) {
            shownPhase = m; shownAt = now; act.changed();
        }
    }

    private void updateSlope() {
        if (free.size() < 10) return;
        long t0 = free.peekFirst()[0], span = free.peekLast()[0] - t0;
        if (span < 1_500_000_000L) return;
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        long u0 = free.peekFirst()[1];
        for (long[] s : free) {
            double x = (s[0] - t0) / 1e9, y = (s[1] - u0);
            n++; sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        double den = n * sxx - sx * sx;
        if (den <= 0) return;
        slopeNsPerNs = (n * sxy - sx * sy) / den / 1e9;   // ns of phase per ns of time
        slopeKnown = true;
    }

    private void decide(long now) {
        long m = median(9);
        long thr = "a".equals(mode) ? A_LOCK_NS : C_LOCK_NS;
        if (Math.abs(m) < thr) {
            state = S_LOCKED; method = "none"; lockedAt = now;
            event(now, String.format(Locale.US, "in phase at start %.1fms", m / 1e6));
            act.changed();
        } else startAlign(now);
    }

    private void startAlign(long now) {
        if (restartsCamera() && !act.mayRestart()) { hold(now, "recording: no camera restart"); return; }
        episodes++;
        state = S_ALIGNING; shownPhase = median(9); shownAt = now; alignT0 = now;
        if ("a".equals(mode)) {                          // REVIEW8 M4: sync=a never falls back to B
            if (!aPossible) {
                method = "A"; state = S_FAILED; cooldownUntil = Long.MAX_VALUE;
                event(now, "A not possible: " + aReason); act.changed(); return;
            }
            method = "A"; beginNudge(now);
        } else if ("b".equals(mode)) { method = "B"; tries = 0; beginRestart(now); }
        else { method = "C"; cTries = 0; beginReopen(now); }
        act.changed();
    }

    private void hold(long now, String why) {
        state = S_DRIFT; cStage = C_IDLE; holdReason = why;
        event(now, "hold: " + why);
        act.changed();
    }

    // ---------------- method C ----------------
    private void beginReopen(long now) {
        if (!act.mayRestart()) { hold(now, "recording: no camera reopen"); return; }
        cTries++; reopens++;
        phaseBefore = median(9);
        cReqAt = now; cStage = C_REQUESTED;
        resetSamples();
        boolean ok = act.reopenCamera();
        event(now, String.format(Locale.US, "C reopen %d from %.1fms", cTries, phaseBefore / 1e6) + (ok ? "" : " (not posted)"));
        if (!ok) { cStage = C_IDLE; reopenFails++; state = S_FAILED; cooldownUntil = now + C_REST_NS; }
        act.changed();
    }

    /** Camera thread: the activity closed the device (the next result comes from the new one). */
    synchronized void reopenClosed() { if (cStage == C_REQUESTED) cStage = C_CLOSED; }

    /** Camera thread: the activity did not close the device (recording started, camera stopped). */
    synchronized void cancelReopen(long now, String why) {
        if (cStage != C_REQUESTED) return;
        cStage = C_IDLE;
        hold(now, why);
    }

    /** Camera thread: opening the device again failed for good (the activity reports ERROR). */
    synchronized void reopenFailed(long now, String why) {
        reopenFails++; cStage = C_IDLE; state = S_FAILED; cooldownUntil = now + C_REST_NS;
        event(now, "C open failed: " + why);
        act.changed();
    }

    private void stepC(long now) {
        if (cStage != C_MEASURE) return;
        int n = samples.size();
        if (n >= C_EARLY && n < C_SAMPLES && Math.abs(median(n)) > C_EARLY_FAR_NS) { endTry(now, median(n), false); return; }
        if (n < C_SAMPLES) return;
        long m = median(C_SAMPLES);
        endTry(now, m, Math.abs(m) < C_LOCK_NS);
    }

    private void endTry(long now, long m, boolean ok) {
        cTryLast = now - cReqAt; cTryMax = Math.max(cTryMax, cTryLast); cTrySum += cTryLast; cTryN++;
        cStage = C_IDLE;
        if (ok) {
            state = S_LOCKED; lockedAt = now; cLocks++; cLastAlign = now - alignT0;
            event(now, String.format(Locale.US, "C locked %.1fms after %d tries %.1fs", m / 1e6, cTries, cLastAlign / 1e9));
            resetSamples(); act.changed(); return;
        }
        if (cTries >= C_MAX_TRIES) {
            cRounds++;
            state = S_FAILED; cooldownUntil = now + C_REST_NS;
            event(now, String.format(Locale.US, "C round %d: no lock in %d tries (last %.1fms), rest 30 s", cRounds, cTries, m / 1e6));
            act.changed(); return;
        }
        beginReopen(now);
    }

    // ---------------- method A (sync=a) ----------------
    private void beginNudge(long now) {
        nudgeT0 = now; nudgeU0 = lastUnwrapped; nudgeErr0 = median(5); pSum = 0; sinceApply = 1000; good = 0;
        event(now, String.format(Locale.US, "A start %.1fms", nudgeErr0 / 1e6));
    }

    /** Back to the normal request; on failure keep overrideOn and retry every frame (REVIEW8 M5). */
    private void endNudge() {
        if (overrideOn) {
            if (act.applyDuration(0, 0, 0)) { overrideOn = false; restorePending = false; }
            else restorePending = true;
        }
        lastDurReq = 0;
    }

    private void stepA(long now, long T) {
        long err = median(5);
        good = Math.abs(err) < A_LOCK_NS ? good + 1 : 0;
        if (good >= 3) {
            endNudge(); state = S_LOCKED; lockedAt = now; resetSamples();
            event(now, String.format(Locale.US, "A locked %.1fms", err / 1e6));
            act.changed(); return;
        }
        if (now - nudgeT0 > A_TEST_NS) {
            long moved = lastUnwrapped - nudgeU0;
            double natural = slopeKnown ? slopeNsPerNs * (now - nudgeT0) : 0;
            boolean noProgress = Math.abs(err) >= 0.7 * Math.abs(nudgeErr0);
            if (Math.abs(pSum) >= 3 * MS && Math.abs(moved - natural) < 0.3 * Math.abs(pSum) && noProgress) {
                failA(now, String.format(Locale.US, "phase did not follow SENSOR_FRAME_DURATION (requested %.1fms, moved %.1fms in 3 s)",
                        pSum / 1e6, moved / 1e6), true);
                return;
            }
            if (now - alignT0 > A_TIMEOUT_NS) { failA(now, "not locked within 20 s", false); return; }
            if (Math.abs(pSum) >= 3 * MS) { nudgeT0 = now; nudgeU0 = lastUnwrapped; nudgeErr0 = err; pSum = 0; }
        }
        double slipPerFrame = slopeKnown ? slopeNsPerNs * T : 0;
        double maxStep = A_MAX_STEP * T;
        double pPart = -A_GAIN * err;
        double rate = Math.max(-maxStep, Math.min(maxStep, pPart - slipPerFrame));
        long dur = Math.round(T + rate);
        if (minDurNs > 0) dur = Math.max(dur, minDurNs);
        if (maxDurNs > 0) dur = Math.min(dur, maxDurNs);
        if (overrideOn) pSum += (dur - T) + slipPerFrame;     // proportional part actually requested this frame
        if (++sinceApply >= 3 && (!overrideOn || Math.abs(dur - lastDurReq) > 20_000L)) {
            if (act.applyDuration(dur, tgtExp, tgtIso)) { overrideOn = true; restorePending = false; lastDurReq = dur; adjustments++; }
            else { failA(now, "request with physical SENSOR_FRAME_DURATION failed", true); return; }
            sinceApply = 0;
        }
    }

    /** sync=a only (A is not used by auto). permanent = the phone does not do it (not just a timeout, REVIEW8 H2.3). */
    private void failA(long now, String why, boolean permanent) {
        endNudge();
        if (permanent) aFailed = true;
        aFailReason = why;
        event(now, "A failed: " + why);
        state = S_FAILED; cooldownUntil = now + FAILED_COOLDOWN_NS;
        act.changed();
    }

    // ---------------- method B (sync=b) ----------------
    private void beginRestart(long now) {
        if (!act.mayRestart()) { hold(now, "recording: no camera restart"); return; }
        if (tries >= B_MAX_TRIES) {
            state = S_FAILED; cooldownUntil = now + FAILED_COOLDOWN_NS;
            event(now, String.format(Locale.US, "B gave up after %d tries %.1fms", tries, median(9) / 1e6));
            act.changed(); return;
        }
        tries++; restarts++;
        phaseBefore = median(9);
        boolean ok = useSession ? act.recreateSession() : act.restartRepeating();
        event(now, (useSession ? "B session " : "B repeat ") + tries + String.format(Locale.US, " from %.1fms", phaseBefore / 1e6) + (ok ? "" : " (failed)"));
        settleUntil = now + (useSession ? 1_500_000_000L : 700_000_000L);
        resetSamples(); periods.clear(); lastRefTs = -1;
    }

    private void stepB(long now) {
        if (++bSamples < 10) return;
        long m = median(9);
        if (Math.abs(m) < B_LOCK_NS) {
            state = S_LOCKED; lockedAt = now;
            event(now, String.format(Locale.US, "B locked %.1fms after %d", m / 1e6, tries));
            act.changed(); return;
        }
        long T = period();
        if (!useSession && Math.abs(wrap(m - phaseBefore, T)) < MS) useSession = true;
        beginRestart(now);
    }

    /** Called when the app stops: drop the override. */
    synchronized void stop() { overrideOn = false; restorePending = false; }

    // ---------------- outputs ----------------
    private String phaseStr() {
        long p = samples.isEmpty() ? lastPhase : median(9);
        return havePhase ? String.format(Locale.US, " phase=%+.1fms", p / 1e6) : "";
    }

    /** " sync=aligning phase=+3.2ms method=C try=4" | " sync=locked phase=+0.4ms method=C" | "" (off). Any thread.
     *  Keys after sync= are only phase/method/try (MainActivity strips exactly those). */
    synchronized String suffix() {
        if (S_OFF.equals(state)) return "";
        StringBuilder s = new StringBuilder(" sync=").append(state).append(phaseStr());
        if (method != null && !S_MEASURING.equals(state)) s.append(" method=").append(method);
        if (S_ALIGNING.equals(state) && "C".equals(method)) s.append(" try=").append(cTries);
        else if (S_ALIGNING.equals(state) && "B".equals(method)) s.append(" try=").append(tries);
        return s.toString();
    }

    private static Object ms(long ns) { return ns < 0 ? JSONObject.NULL : (Object) Math.round(ns / 1e6); }

    /** metrics.json / meta.json "sync". Any thread. */
    synchronized JSONObject json() throws JSONException {
        JSONObject j = new JSONObject();
        j.put("state", state);
        j.put("method", method == null ? JSONObject.NULL : method);
        long p = samples.isEmpty() ? lastPhase : median(9);
        // phase_ms = phase_mid_ms (the controlled value since 1.7-mid); phase_start_ms = physical SENSOR_TIMESTAMP difference
        j.put("phase_ms", havePhase ? (Object) Caps.round(p / 1e6) : JSONObject.NULL);
        j.put("phase_mid_ms", havePhase ? (Object) Caps.round(p / 1e6) : JSONObject.NULL);
        j.put("phase_start_ms", haveStart ? (Object) Caps.round(startMedian() / 1e6) : JSONObject.NULL);
        j.put("phase_ref", "mid_exposure");
        // slope is ns of phase per ns of time (= ms/ms) -> ms/s = x 1e3 (REVIEW8 M1)
        j.put("slip_ms_s", slopeKnown ? Caps.num(Math.round(slopeNsPerNs * 1e3 * 1000) / 1000.0) : JSONObject.NULL);
        j.put("adjustments", adjustments);
        j.put("restarts", restarts);
        j.put("episodes", episodes);
        j.put("tries", "C".equals(method) ? cTries : tries);
        j.put("period_ms", Caps.round(period() / 1e6));
        j.put("lens_ref", refId); j.put("lens_target", tgtId);
        j.put("mode", mode);
        j.put("auto_method", "C");
        // method C (camera re-open)
        j.put("reopens", reopens);
        j.put("reopen_fails", reopenFails);
        j.put("c_try", cTries); j.put("c_max_tries", C_MAX_TRIES); j.put("c_rounds_failed", cRounds); j.put("c_locks", cLocks);
        j.put("c_lock_ms", C_LOCK_NS / 1e6); j.put("c_realign_ms", C_REALIGN_NS / 1e6);
        j.put("c_expected_tries", Caps.round(period() / (double) (2 * C_LOCK_NS)));   // 1 / (5 ms / T)
        j.put("c_try_ms_last", ms(cTryLast)); j.put("c_try_ms_max", cTryN > 0 ? ms(cTryMax) : JSONObject.NULL);
        j.put("c_try_ms_avg", cTryN > 0 ? ms(cTrySum / cTryN) : JSONObject.NULL);
        j.put("c_open_ms_last", ms(cOpenLast)); j.put("c_open_ms_max", cOpenN > 0 ? ms(cOpenMax) : JSONObject.NULL);
        j.put("c_open_ms_avg", cOpenN > 0 ? ms(cOpenSum / cOpenN) : JSONObject.NULL);
        j.put("c_last_align_s", cLastAlign < 0 ? JSONObject.NULL : (Object) Caps.round(cLastAlign / 1e9));
        if (S_ALIGNING.equals(state)) j.put("align_s", Caps.round((lastNow() - alignT0) / 1e9));
        if (holdReason != null) j.put("hold_reason", holdReason);
        if (S_FAILED.equals(state) && cooldownUntil != Long.MAX_VALUE) j.put("retry_in_s", Math.max(0, Math.round((cooldownUntil - lastNow()) / 1e9)));
        // method A / B (sync=a|b only; measured results on the S24)
        j.put("method_a_possible", aPossible); j.put("method_a_ae_key", aeKey);
        j.put("method_a_reason", aReason == null ? JSONObject.NULL : aReason);
        j.put("method_a_failed", aFailed);
        if (aFailReason != null) j.put("method_a_fail_reason", aFailReason);
        j.put("method_a_measured", MEASURED_A);
        j.put("method_b_measured", MEASURED_B);
        j.put("override_on", overrideOn);
        if (overrideOn) j.put("target_dur_req_ms", Caps.round(lastDurReq / 1e6));
        if (tgtDur > 0) j.put("target_dur_ms", Caps.round(tgtDur / 1e6));
        j.put("session_recreate", useSession);
        j.put("events", events.toString());
        return j;
    }

    /** Same clock as onResult's now (camera timestamps / elapsedRealtimeNanos). */
    private static long lastNow() { return android.os.SystemClock.elapsedRealtimeNanos(); }
}
