package org.json;

// PC-test stand-in for org.json.JSONObject (android.jar has only stubs that throw). Only what ViopRelayCore uses.
public class JSONObject {
    private final java.util.LinkedHashMap<String, Object> m = new java.util.LinkedHashMap<>();
    public JSONObject put(String k, Object v) { m.put(k, v); return this; }
    public JSONObject put(String k, long v) { m.put(k, v); return this; }
    public JSONObject put(String k, int v) { m.put(k, v); return this; }
    public JSONObject put(String k, double v) { m.put(k, v); return this; }
    public JSONObject put(String k, boolean v) { m.put(k, v); return this; }
    public Object opt(String k) { return m.get(k); }
    public long optLong(String k) { Object o = m.get(k); return o instanceof Number ? ((Number) o).longValue() : 0; }
    public double optDouble(String k) { Object o = m.get(k); return o instanceof Number ? ((Number) o).doubleValue() : Double.NaN; }
    public String optString(String k) { Object o = m.get(k); return o == null ? "" : o.toString(); }
    @Override public String toString() {
        StringBuilder sb = new StringBuilder("{");
        for (java.util.Map.Entry<String, Object> e : m.entrySet()) {
            if (sb.length() > 1) sb.append(", ");
            Object v = e.getValue();
            sb.append('"').append(e.getKey()).append("\": ");
            if (v instanceof String) sb.append('"').append(((String) v).replace("\n", "\\n")).append('"'); else sb.append(v);
        }
        return sb.append('}').toString();
    }
    public String toString(int indent) { return toString(); }
}
