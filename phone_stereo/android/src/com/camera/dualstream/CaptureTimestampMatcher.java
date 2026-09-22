package com.camera.dualstream;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

/** Exact Image timestamp -> logical result join. Confined to the camera Handler.
 * Payloads are copies, never open Images. Expiry uses callback arrival time, not sensor time.
 */
final class CaptureTimestampMatcher {
    interface Sink { void accept(int index, long physicalTimestamp, byte[] data); }
    private static final int LIMIT = 24;
    private static final long TTL_NS = 1_000_000_000L;
    private final int count;
    private final Sink sink;
    private final LinkedHashMap<Long, Entry> entries = new LinkedHashMap<>();
    long matched, dropped;
    private final class Entry {
        final long arrival;
        final byte[][] images = new byte[count][];
        final boolean[] consumed = new boolean[count];
        Long[] physical;
        Entry(long now) { arrival = now; }
    }
    CaptureTimestampMatcher(int count, Sink sink) { this.count = count; this.sink = sink; }
    private Entry entry(long timestamp, long now) {
        Iterator<Map.Entry<Long, Entry>> it = entries.entrySet().iterator();
        while (it.hasNext()) {
            Entry e = it.next().getValue();
            if (now - e.arrival >= TTL_NS) { discard(e); it.remove(); }
        }
        Entry e = entries.get(timestamp);
        if (e == null) {
            if (entries.size() >= LIMIT) {
                it = entries.entrySet().iterator();
                discard(it.next().getValue()); it.remove();
            }
            e = new Entry(now); entries.put(timestamp, e);
        }
        return e;
    }
    void result(long logical, Long[] physical, long now) {
        if (physical.length != count) throw new IllegalArgumentException("physical count");
        Entry e = entry(logical, now);
        if (e.physical != null) return;
        e.physical = physical.clone();
        for (int i = 0; i < count; i++) emit(e, i);
    }
    void image(int index, long logical, byte[] data, long now) {
        Entry e = entry(logical, now);
        if (e.consumed[index] || e.images[index] != null) return;
        e.images[index] = data;
        emit(e, index);
    }
    private void emit(Entry e, int i) {
        if (e.physical == null || e.images[i] == null) return;
        byte[] data = e.images[i]; e.images[i] = null; e.consumed[i] = true;
        if (e.physical[i] == null) { dropped++; return; }
        matched++;
        sink.accept(i, e.physical[i], data);
    }
    private void discard(Entry e) {
        for (byte[] data : e.images) if (data != null) dropped++;
    }
    void clear() { for (Entry e : entries.values()) discard(e); entries.clear(); }
    int size() { return entries.size(); }
}
