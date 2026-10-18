package com.camera.dualstream;

import android.media.Image;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

/** Capture fast, save 1 of N (app 1.8-subsample, scan3d/SPEC.md 1.2.3): the lenses run at capture_fps (e.g. 60) and only
 *  save_fps (e.g. 15) frames per lens are copied, encoded, recorded and streamed.
 *  - reference lens (physical "2", else index 0): the frame whose mid-exposure time is nearest to the save grid
 *    (one every 1/save_fps s), decided from its capture result alone;
 *  - every other lens: the one frame whose mid-exposure time is nearest to a kept reference frame (|dt| <= 1 capture
 *    period), decided only from timestamps once frames of that lens past the reference time + 0.6 period are known.
 *  Decisions use only result timestamps (physical SENSOR_TIMESTAMP + exposure/2 + skew/2 of the last known values), so a
 *  dropped frame is closed at once and never copied. Images that arrive before their decision are held (ImageReader
 *  maxImages must be > HOLD_MAX). Camera handler thread only, except json()/dtMs*, which take the stats lock. */
final class Subsample {
    interface Sink {
        /** Keep: copy + encode this image. pairDtNs = this lens mid - reference mid (Long.MIN_VALUE for the reference). */
        void keep(int lens, long physicalTs, Image img, long pairDtNs);
    }
    static final int HOLD_MAX = 5;                         // held images per lens; readers get HOLD_MAX + 3
    private static final int SLOT_MAX = 48;                // ~0.8 s of 60 fps results
    private static final byte UNKNOWN = 0, PENDING = 1, KEEP = 2, DROP = 3;

    final int lenses, ref, captureFps, saveFps;
    private final long periodNs, saveNs;
    private final Sink sink;

    private final class Slot {
        final long logical;
        long[] phys, mid;                                  // null until the capture result arrived
        final byte[] state = new byte[lenses];
        final Image[] img = new Image[lenses];
        final long[] pairDt = new long[lenses];
        final boolean[] got = new boolean[lenses];        // image of that lens arrived (slot stays until all did)
        Slot(long l) { logical = l; Arrays.fill(pairDt, Long.MIN_VALUE); }
    }
    /** A kept reference frame waiting for the nearest frame of each other lens. */
    private static final class Ref {
        final long tr; final boolean[] done;
        Ref(long tr, int n) { this.tr = tr; done = new boolean[n]; }
    }
    private final LinkedHashMap<Long, Slot> slots = new LinkedHashMap<>();
    private final ArrayDeque<Ref> refs = new ArrayDeque<>();
    private final ArrayList<ArrayDeque<Slot>> cands = new ArrayList<>();
    private final long[] lastMid;
    private final int[] held;
    private long nextTarget = Long.MIN_VALUE;

    // ---- counters (stats lock) ----
    private final Object stats = new Object();
    private final long[] kept, seen, forced;
    private long pairs, unmatched;
    private final ArrayList<Integer> dtUs = new ArrayList<>();        // |pair dt| of every pair, us (whole recording)
    private final ArrayDeque<Integer> dtRecent = new ArrayDeque<>();  // last 30 pairs (metrics.json)

    Subsample(int lenses, int ref, int captureFps, int saveFps, Sink sink) {
        this.lenses = lenses; this.ref = ref; this.captureFps = captureFps; this.saveFps = saveFps; this.sink = sink;
        periodNs = Math.round(1e9 / captureFps); saveNs = Math.round(1e9 / saveFps);
        lastMid = new long[lenses]; Arrays.fill(lastMid, Long.MIN_VALUE);
        held = new int[lenses];
        kept = new long[lenses]; seen = new long[lenses]; forced = new long[lenses];
        for (int i = 0; i < lenses; i++) cands.add(new ArrayDeque<Slot>());
    }

    private Slot slot(long logical) {
        Slot s = slots.get(logical);
        if (s != null) return s;
        if (slots.size() >= SLOT_MAX) {                    // oldest slot: a result or image that never got its partner
            Iterator<Map.Entry<Long, Slot>> it = slots.entrySet().iterator();
            Slot old = it.next().getValue(); it.remove();
            for (int i = 0; i < lenses; i++) {
                if (old.state[i] != KEEP && old.state[i] != DROP) old.state[i] = DROP;
                closeHeld(old, i);
            }
            for (ArrayDeque<Slot> c : cands) c.remove(old);
        }
        s = new Slot(logical); slots.put(logical, s);
        return s;
    }

    /** Every capture result, in order. phys[i] null = no physical timestamp for that lens (frame cannot be used). */
    void result(long logical, Long[] phys, long[] midOff) {
        Slot s = slot(logical);
        if (s.phys != null) return;
        s.phys = new long[lenses]; s.mid = new long[lenses];
        for (int i = 0; i < lenses; i++) {
            if (phys[i] == null) { s.state[i] = DROP; continue; }
            s.phys[i] = phys[i]; s.mid[i] = phys[i] + midOff[i];
            if (i != ref) { s.state[i] = PENDING; cands.get(i).addLast(s); lastMid[i] = Math.max(lastMid[i], s.mid[i]); }
        }
        if (s.state[ref] != DROP) {
            long tr = s.mid[ref];
            lastMid[ref] = Math.max(lastMid[ref], tr);
            if (nextTarget == Long.MIN_VALUE || tr >= nextTarget - periodNs / 2) {
                s.state[ref] = KEEP;
                // stay on the save grid (missing capture frames only delay one pick); restart it after a long gap
                nextTarget = nextTarget != Long.MIN_VALUE && tr <= nextTarget + saveNs / 2 ? nextTarget + saveNs : tr + saveNs;
                refs.addLast(new Ref(tr, lenses));
            } else s.state[ref] = DROP;
        }
        for (int i = 0; i < lenses; i++) if (i != ref) resolve(i);
        Iterator<Ref> it = refs.iterator();
        while (it.hasNext()) {
            Ref r = it.next();
            boolean all = true;
            for (int i = 0; i < lenses; i++) if (i != ref && !r.done[i]) all = false;
            if (!all) break;
            it.remove();
        }
        flush();
    }

    /** Lens i: close every open reference whose window is fully known, then drop candidates no reference can use. */
    private void resolve(int i) {
        ArrayDeque<Slot> c = cands.get(i);
        for (Ref r : refs) {
            if (r.done[i]) continue;
            // all frames of lens i within 0.6 period after the reference are known (or lens i stopped for 3 periods)
            if (lastMid[i] < r.tr + periodNs * 6 / 10 && lastMid[ref] < r.tr + 3 * periodNs) break;
            Slot best = null; long bd = Long.MAX_VALUE;
            for (Slot s : c) {
                if (s.state[i] != PENDING) continue;
                long d = Math.abs(s.mid[i] - r.tr);
                if (d <= periodNs && d < bd) { bd = d; best = s; }
            }
            r.done[i] = true;
            if (best != null) {
                best.state[i] = KEEP;
                long dt = best.mid[i] - r.tr;
                best.pairDt[i] = dt;
                synchronized (stats) {
                    pairs++;
                    int us = (int) Math.min(Integer.MAX_VALUE, Math.abs(dt) / 1000);
                    dtUs.add(us); dtRecent.addLast(us); while (dtRecent.size() > 30) dtRecent.pollFirst();
                }
            } else synchronized (stats) { unmatched++; }
        }
        long bound = Long.MAX_VALUE;
        for (Ref r : refs) if (!r.done[i]) { bound = r.tr; break; }
        if (bound == Long.MAX_VALUE) bound = nextTarget == Long.MIN_VALUE ? Long.MIN_VALUE : nextTarget - periodNs / 2;
        for (Iterator<Slot> it = c.iterator(); it.hasNext(); ) {
            Slot s = it.next();
            if (s.state[i] == KEEP || s.state[i] == DROP) { it.remove(); continue; }
            if (bound != Long.MIN_VALUE && s.mid[i] < bound - periodNs) { s.state[i] = DROP; it.remove(); }
        }
    }

    /** Every acquired image (logical timestamp = Image.getTimestamp()). Never blocks; the image is closed here or later. */
    void image(int lens, long logical, Image img) {
        synchronized (stats) { seen[lens]++; }
        Slot s = slot(logical);
        if (s.got[lens]) { img.close(); return; }          // duplicate (should not happen)
        s.got[lens] = true; s.img[lens] = img; held[lens]++;
        apply(s, lens);
        if (held[lens] > HOLD_MAX) {                       // ImageReader would run out: drop the oldest held image
            for (Slot o : slots.values()) if (o.img[lens] != null && o != s) {
                o.state[lens] = DROP; closeHeld(o, lens);
                synchronized (stats) { forced[lens]++; }
                break;
            }
        }
    }

    private void flush() {
        for (Iterator<Map.Entry<Long, Slot>> it = slots.entrySet().iterator(); it.hasNext(); ) {
            Slot s = it.next().getValue();
            boolean open = s.phys == null;
            for (int i = 0; i < lenses; i++) {
                apply(s, i);
                if (s.state[i] == PENDING || s.state[i] == UNKNOWN || !s.got[i]) open = true;   // decision or image to come
            }
            if (!open) it.remove();
        }
    }

    private void apply(Slot s, int i) {
        Image img = s.img[i];
        if (img == null) return;
        if (s.state[i] == KEEP) {
            s.img[i] = null; held[i]--;
            try { sink.keep(i, s.phys[i], img, s.pairDt[i]); }
            finally { img.close(); }
            synchronized (stats) { kept[i]++; }
        } else if (s.state[i] == DROP) closeHeld(s, i);
    }

    private void closeHeld(Slot s, int i) {
        Image img = s.img[i];
        if (img == null) return;
        s.img[i] = null; held[i]--;
        try { img.close(); } catch (Exception ignored) {}
    }

    /** Close every held image (stop / camera re-open). */
    void clear() {
        for (Slot s : slots.values()) for (int i = 0; i < lenses; i++) closeHeld(s, i);
        slots.clear(); refs.clear();
        for (ArrayDeque<Slot> c : cands) c.clear();
        Arrays.fill(lastMid, Long.MIN_VALUE); Arrays.fill(held, 0);
        nextTarget = Long.MIN_VALUE;
    }

    private static double pct(ArrayList<Integer> sorted, double q) {
        if (sorted.isEmpty()) return Double.NaN;
        return sorted.get((int) Math.min(sorted.size() - 1, Math.round(q * (sorted.size() - 1)))) / 1000.0;
    }

    /** metrics.json / meta.json "subsample". recent=true: last 30 pairs only (metrics). Any thread. */
    JSONObject json(String[] ids, boolean recent) throws JSONException {
        JSONObject o = new JSONObject();
        o.put("capture_fps", captureFps); o.put("save_fps", saveFps); o.put("ref", ids[ref]);
        o.put("rule", "ref lens: nearest to the 1/save_fps grid; other lenses: nearest mid-exposure to the kept ref frame (|dt| <= 1 capture period)");
        ArrayList<Integer> d;
        synchronized (stats) {
            JSONObject k = new JSONObject(), sn = new JSONObject(), f = new JSONObject();
            for (int i = 0; i < lenses; i++) { k.put(ids[i], kept[i]); sn.put(ids[i], seen[i]); f.put(ids[i], forced[i]); }
            o.put("kept", k); o.put("seen", sn); o.put("forced_drops", f);
            o.put("pairs", pairs); o.put("unmatched", unmatched);
            d = new ArrayList<>(recent ? dtRecent : dtUs);
        }
        java.util.Collections.sort(d);
        o.put(recent ? "recent_n" : "n", d.size());
        o.put("dt_ms_median", Caps.num(Caps.round(pct(d, 0.5))));
        o.put("dt_ms_p95", Caps.num(Caps.round(pct(d, 0.95))));
        o.put("dt_ms_max", Caps.num(Caps.round(pct(d, 1.0))));
        return o;
    }
}
