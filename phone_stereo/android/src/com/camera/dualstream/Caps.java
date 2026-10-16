package com.camera.dualstream;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.hardware.camera2.*;
import android.hardware.camera2.params.*;
import android.media.MediaCodecInfo;
import android.media.MediaCodecList;
import android.media.MediaFormat;
import android.os.Build;
import android.util.Range;
import android.util.Size;
import org.json.*;
import java.util.*;

/** Read-only capability query -> caps.json (format scan3d/SPEC.md 14). Opens no camera, creates no codec.
 *  Sources (developer.android.com/reference):
 *   - android/hardware/camera2/params/StreamConfigurationMap#getOutputMinFrameDuration (per-size max fps)
 *   - android/media/MediaCodecInfo.VideoCapabilities#areSizeAndRateSupported, #getAchievableFrameRatesFor
 *   - android/hardware/camera2/CameraDevice.CameraDeviceSetup#isSessionConfigurationSupported (API 35, no open)
 *  The same checks (checkCamera / findAvcEncoder) gate the start: an unsupported width/height/fps is an ERROR,
 *  never silently replaced by the nearest supported size. */
final class Caps {
    static final int[] FPS_CANDIDATES = {10, 15, 20, 24, 30, 60};
    /** Recording default: 4 Mbps per lens at 640x480@15, scaled by pixels x fps (scan3d/SPEC.md 14.4). */
    static final double REC_KBPS_BASE = 4000, REC_BASE_PIXEL_RATE = 640.0 * 480 * 15;
    private static final int COLOR_FLEX = MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible;

    static boolean is43(int w, int h) { return w > 0 && h > 0 && w * 3 == h * 4; }

    /** YUV_420_888 min frame duration (ns): 0 = size not offered, -1 = offered but duration unknown. */
    static long minFrameNs(CameraCharacteristics c, int w, int h) {
        StreamConfigurationMap m = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        if (m == null) return 0;
        Size[] sizes = m.getOutputSizes(ImageFormat.YUV_420_888);
        if (sizes == null) return 0;
        for (Size s : sizes) if (s.getWidth() == w && s.getHeight() == h) {
            long d = m.getOutputMinFrameDuration(ImageFormat.YUV_420_888, s);
            return d > 0 ? d : -1;
        }
        return 0;
    }

    static int[] fixedFps(CameraCharacteristics logical) {
        Range<Integer>[] r = logical.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        TreeSet<Integer> s = new TreeSet<>();
        if (r != null) for (Range<Integer> x : r) if (x.getLower().equals(x.getUpper())) s.add(x.getUpper());
        int[] a = new int[s.size()]; int i = 0;
        for (int v : s) a[i++] = v;
        return a;
    }

    /** Max fps of one request that fills every lens at w x h: 1e9 / max(min frame duration). +Inf when unknown, 0 when a lens lacks the size. */
    static double pairMaxFps(CameraManager mgr, String[] ids, int w, int h) throws CameraAccessException {
        double max = Double.POSITIVE_INFINITY;
        for (String id : ids) {
            long d = minFrameNs(mgr.getCameraCharacteristics(id), w, h);
            if (d == 0) return 0;
            if (d > 0) max = Math.min(max, 1e9 / d);
        }
        return max;
    }

    /** null = camera side can stream w x h @ fps on every lens; else a short reason (goes into the ERROR status). */
    static String checkCamera(CameraManager mgr, String logical, String[] ids, int w, int h, int fps) throws CameraAccessException {
        if (w <= 0 || h <= 0 || (w & 1) != 0 || (h & 1) != 0) return "size must be positive and even";
        if (fps <= 0) return "fps must be > 0";
        for (String id : ids)
            if (minFrameNs(mgr.getCameraCharacteristics(id), w, h) == 0) return "lens " + id + " has no YUV " + w + "x" + h;
        double max = pairMaxFps(mgr, ids, w, h);
        if (fps > max + 0.01) return "lens max " + round(max) + " fps at " + w + "x" + h;
        int[] fixed = fixedFps(mgr.getCameraCharacteristics(logical));
        for (int f : fixed) if (f == fps) return null;
        return "no fixed AE range [" + fps + "," + fps + "] (fixed: " + Arrays.toString(fixed) + ")";
    }

    /** H.264 encoders, hardware first (isHardwareAccelerated, API 29+), then list order. */
    private static List<MediaCodecInfo> avcCache;
    static synchronized List<MediaCodecInfo> avcEncoders() {
        if (avcCache != null) return avcCache;
        List<MediaCodecInfo> hw = new ArrayList<>(), sw = new ArrayList<>();
        for (MediaCodecInfo ci : new MediaCodecList(MediaCodecList.REGULAR_CODECS).getCodecInfos()) {
            if (!ci.isEncoder()) continue;
            boolean avc = false;
            for (String t : ci.getSupportedTypes()) if (t.equalsIgnoreCase(MediaFormat.MIMETYPE_VIDEO_AVC)) avc = true;
            if (!avc) continue;
            if (Build.VERSION.SDK_INT >= 29 && ci.isHardwareAccelerated()) hw.add(ci); else sw.add(ci);
        }
        hw.addAll(sw);
        avcCache = Collections.unmodifiableList(hw);
        return avcCache;
    }

    private static boolean flexible(MediaCodecInfo.CodecCapabilities cc) {
        if (cc.colorFormats != null) for (int c : cc.colorFormats) if (c == COLOR_FLEX) return true;
        return false;
    }

    /** First (hardware-first) H.264 encoder supporting w x h @ fps, YUV420Flexible input and >= instances parallel
     *  instances (one per lens). null = none. The recorder uses exactly this codec (createByCodecName). */
    static String findAvcEncoder(int w, int h, int fps, int instances) {
        for (MediaCodecInfo ci : avcEncoders()) {
            try {
                MediaCodecInfo.CodecCapabilities cc = ci.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC);
                MediaCodecInfo.VideoCapabilities v = cc.getVideoCapabilities();
                if (v != null && v.areSizeAndRateSupported(w, h, fps) && flexible(cc) && cc.getMaxSupportedInstances() >= instances)
                    return ci.getName();
            } catch (Exception ignored) {}
        }
        return null;
    }

    /** Default per-lens bitrate (kbps): 4000 x (w*h*fps)/(640*480*15), min 500, clamped to the encoder bitrate range. */
    static int autoKbps(int w, int h, int fps, String encoderName) {
        int kbps = (int) Math.max(500, Math.round(REC_KBPS_BASE * ((double) w * h * fps) / REC_BASE_PIXEL_RATE));
        if (encoderName != null) for (MediaCodecInfo ci : avcEncoders()) if (ci.getName().equals(encoderName)) {
            try {
                Range<Integer> r = ci.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC).getVideoCapabilities().getBitrateRange();
                kbps = Math.max(r.getLower() / 1000 + 1, Math.min(r.getUpper() / 1000, kbps));
            } catch (Exception ignored) {}
        }
        return kbps;
    }

    // ---------------------------------------------------------------------------------------------------------
    /** Lens phase alignment method A (scan3d/SPEC.md 15): can the app change ONE lens' frame duration?
     *  Needs SENSOR_FRAME_DURATION in the logical camera's getAvailablePhysicalCameraRequestKeys() (API 28) and
     *  MANUAL_SENSOR on that lens or the logical camera (frame duration is a MANUAL_SENSOR control). With AE on, the
     *  HAL normally ignores SENSOR_FRAME_DURATION, so aeKey = CONTROL_AE_MODE + SENSOR_EXPOSURE_TIME +
     *  SENSOR_SENSITIVITY are physical request keys too (AE off on that lens only while nudging). Without aeKey the app
     *  still tries (frame duration only) and falls back to method B when the phase does not move within 3 s. */
    static final class MethodA {
        boolean possible, aeKey;
        String reason;
        JSONObject keys = new JSONObject();
    }

    static final CaptureRequest.Key<?>[] PHASE_KEYS = {CaptureRequest.SENSOR_FRAME_DURATION, CaptureRequest.SENSOR_EXPOSURE_TIME,
            CaptureRequest.SENSOR_SENSITIVITY, CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
            CaptureRequest.CONTROL_MODE};

    static boolean hasCap(CameraCharacteristics c, int cap) {
        int[] caps = c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
        if (caps != null) for (int x : caps) if (x == cap) return true;
        return false;
    }

    static MethodA methodA(CameraManager mgr, String logical, String target) {
        MethodA m = new MethodA();
        try {
            CameraCharacteristics lc = mgr.getCameraCharacteristics(logical);
            List<CaptureRequest.Key<?>> pk = lc.getAvailablePhysicalCameraRequestKeys();
            for (CaptureRequest.Key<?> k : PHASE_KEYS) m.keys.put(k.getName(), pk != null && pk.contains(k));
            if (pk == null || pk.isEmpty()) { m.reason = "no physical request keys (getAvailablePhysicalCameraRequestKeys empty)"; return m; }
            if (!pk.contains(CaptureRequest.SENSOR_FRAME_DURATION)) { m.reason = "SENSOR_FRAME_DURATION is not a physical request key"; return m; }
            boolean manual = hasCap(lc, CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR)
                    || hasCap(mgr.getCameraCharacteristics(target), CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR);
            if (!manual) { m.reason = "no MANUAL_SENSOR capability (logical or lens " + target + ")"; return m; }
            m.possible = true;
            m.aeKey = pk.contains(CaptureRequest.CONTROL_AE_MODE) && pk.contains(CaptureRequest.SENSOR_EXPOSURE_TIME)
                    && pk.contains(CaptureRequest.SENSOR_SENSITIVITY);
            m.reason = m.aeKey ? "ok: frame duration + AE off on lens " + target + " only while nudging"
                    : "frame duration only (CONTROL_AE_MODE/EXPOSURE/SENSITIVITY not physical keys; AE on may ignore it -> B after 3 s)";
        } catch (Exception e) { m.reason = "error " + e; m.possible = false; }
        return m;
    }

    static JSONArray keyNames(List<?> keys) {
        JSONArray a = new JSONArray();
        if (keys != null) for (Object k : keys) {
            if (k instanceof CaptureRequest.Key) a.put(((CaptureRequest.Key<?>) k).getName());
            else if (k instanceof CaptureResult.Key) a.put(((CaptureResult.Key<?>) k).getName());
            else a.put(String.valueOf(k));
        }
        return a;
    }

    /** Full caps.json document. Slow parts (session queries) included; call off the main thread. */
    static JSONObject build(Context ctx, CameraManager mgr, String logical, String[] ids) throws Exception {
        JSONObject j = new JSONObject();
        j.put("format", "scan3d-phone-caps/1");
        j.put("written_wall_ms", System.currentTimeMillis());
        j.put("written_wall", new java.text.SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.US).format(new Date()));
        try {
            android.content.pm.PackageInfo pi = ctx.getPackageManager().getPackageInfo(ctx.getPackageName(), 0);
            j.put("app_version", pi.versionName); j.put("app_version_code", pi.versionCode);
        } catch (Exception e) { j.put("app_version", "unknown"); }
        // frame header / frames CSV timestamps = mid-exposure (1.7-mid, SPEC 1.2); same clock as the IMU
        j.put("timestamp_ref", MainActivity.TS_REF);
        j.put("timestamp_ref_formula", "physical SENSOR_TIMESTAMP + SENSOR_EXPOSURE_TIME/2 + SENSOR_ROLLING_SHUTTER_SKEW/2 (per lens)");
        j.put("timestamp_clock", "SystemClock.elapsedRealtimeNanos");
        JSONObject dev = new JSONObject();
        dev.put("manufacturer", Build.MANUFACTURER); dev.put("model", Build.MODEL); dev.put("device", Build.DEVICE);
        dev.put("hardware", Build.HARDWARE); dev.put("android", Build.VERSION.RELEASE); dev.put("sdk", Build.VERSION.SDK_INT);
        if (Build.VERSION.SDK_INT >= 31) { dev.put("soc_manufacturer", Build.SOC_MANUFACTURER); dev.put("soc_model", Build.SOC_MODEL); }
        dev.put("cpu_cores", Runtime.getRuntime().availableProcessors());
        j.put("device", dev);

        CameraCharacteristics lc = mgr.getCameraCharacteristics(logical);
        JSONObject lj = new JSONObject();
        lj.put("id", logical);
        lj.put("physical_ids", new JSONArray(new ArrayList<>(lc.getPhysicalCameraIds())));
        int[] caps = lc.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
        boolean multi = false;
        if (caps != null) for (int c : caps) if (c == CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA) multi = true;
        lj.put("logical_multi_camera", multi);
        lj.put("sync_type", lc.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE));
        lj.put("hardware_level", lc.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL));
        JSONArray ranges = new JSONArray();
        Range<Integer>[] r = lc.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        if (r != null) for (Range<Integer> x : r) ranges.put(new JSONArray().put(x.getLower()).put(x.getUpper()));
        lj.put("ae_fps_ranges", ranges);
        JSONArray fixed = new JSONArray();
        for (int f : fixedFps(lc)) fixed.put(f);
        lj.put("fixed_fps", fixed);
        // Lens phase alignment inputs (SPEC 15): which per-lens (physical) request keys this logical camera accepts.
        List<CaptureRequest.Key<?>> pk = lc.getAvailablePhysicalCameraRequestKeys();
        lj.put("physical_request_keys", pk == null ? JSONObject.NULL : keyNames(pk));
        List<CaptureRequest.Key<?>> rk = lc.getAvailableCaptureRequestKeys();
        JSONObject rkHas = new JSONObject();
        for (CaptureRequest.Key<?> k : PHASE_KEYS) rkHas.put(k.getName(), rk != null && rk.contains(k));
        lj.put("request_keys_has", rkHas);
        lj.put("session_keys", keyNames(lc.getAvailableSessionKeys()));
        lj.put("manual_sensor", hasCap(lc, CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR));
        j.put("logical", lj);
        String target = ids.length > 1 ? ids[1] : ids[0];
        for (String id : ids) if (id.equals("5")) target = id;
        MethodA ma = methodA(mgr, logical, target);
        JSONObject ps = new JSONObject();
        ps.put("target_lens", target);
        ps.put("sync_type", lc.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE));   // 0 APPROXIMATE, 1 CALIBRATED
        ps.put("physical_request_keys_has", ma.keys);
        ps.put("method_a_possible", ma.possible);
        ps.put("method_a_ae_key", ma.aeKey);
        ps.put("method_a_reason", ma.reason);
        ps.put("auto_method", "C");                         // 1.6-sync: auto = camera re-open (SPEC 15.3)
        ps.put("method_a_measured", PhaseSync.MEASURED_A);
        ps.put("method_b_measured", PhaseSync.MEASURED_B);
        j.put("phase_sync", ps);

        // Per lens: every YUV size with min frame duration -> max fps.
        JSONObject lenses = new JSONObject();
        List<Set<String>> sizeSets = new ArrayList<>();
        for (String id : ids) {
            CameraCharacteristics c = mgr.getCameraCharacteristics(id);
            JSONObject o = new JSONObject();
            Rect aa = c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
            Size pa = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE);
            if (aa != null) o.put("active_array", new JSONArray().put(aa.left).put(aa.top).put(aa.right).put(aa.bottom));
            if (pa != null) o.put("pixel_array", new JSONArray().put(pa.getWidth()).put(pa.getHeight()));
            boolean aa43 = aa != null && is43(aa.width(), aa.height());
            o.put("active_array_4_3", aa43);
            o.put("sync_type", c.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE));
            o.put("manual_sensor", hasCap(c, CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR));
            Range<Long> er = c.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
            Range<Integer> sr = c.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
            Long mfd = c.get(CameraCharacteristics.SENSOR_INFO_MAX_FRAME_DURATION);
            o.put("exposure_ns_range", er == null ? JSONObject.NULL : range(er));
            o.put("sensitivity_range", sr == null ? JSONObject.NULL : range(sr));
            o.put("max_frame_duration_ns", mfd == null ? JSONObject.NULL : (Object) mfd);
            JSONArray yuv = new JSONArray();
            Set<String> set = new LinkedHashSet<>();
            StreamConfigurationMap m = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] sizes = m == null ? null : m.getOutputSizes(ImageFormat.YUV_420_888);
            if (sizes != null) for (Size s : sizes) {
                JSONObject y = new JSONObject();
                long d = m.getOutputMinFrameDuration(ImageFormat.YUV_420_888, s);
                y.put("w", s.getWidth()); y.put("h", s.getHeight());
                y.put("aspect_4_3", is43(s.getWidth(), s.getHeight()));
                // Default crop region = full active array; a 4:3 output of a 4:3 array is the whole sensor area (no crop),
                // so intrinsics scale by size ratio (scan3d/SPEC.md 14.5).
                y.put("full_active_array", aa43 && is43(s.getWidth(), s.getHeight()));
                y.put("min_frame_ns", d);
                y.put("max_fps", d > 0 ? num(round(1e9 / d)) : JSONObject.NULL);
                y.put("stall_ns", m.getOutputStallDuration(ImageFormat.YUV_420_888, s));
                yuv.put(y);
                set.add(s.getWidth() + "x" + s.getHeight());
            }
            o.put("yuv", yuv);
            lenses.put(id, o);
            sizeSets.add(set);
        }
        j.put("lenses", lenses);

        // Sizes every selected lens offers = sizes one request can use. Sorted by pixel count, large first.
        List<int[]> common = new ArrayList<>();
        if (!sizeSets.isEmpty()) for (String s : sizeSets.get(0)) {
            boolean all = true;
            for (Set<String> o : sizeSets) all &= o.contains(s);
            if (all) { String[] p = s.split("x"); common.add(new int[]{Integer.parseInt(p[0]), Integer.parseInt(p[1])}); }
        }
        Collections.sort(common, (a, b) -> Long.compare((long) b[0] * b[1], (long) a[0] * a[1]));

        List<MediaCodecInfo> encs = avcEncoders();
        JSONArray pairs = new JSONArray();
        for (int[] s : common) {
            JSONObject p = new JSONObject();
            p.put("w", s[0]); p.put("h", s[1]); p.put("aspect_4_3", is43(s[0], s[1]));
            double max = pairMaxFps(mgr, ids, s[0], s[1]);
            p.put("max_fps", num(round(max)));
            JSONArray camFps = new JSONArray();
            JSONObject enc = new JSONObject(), kbps = new JSONObject();
            for (int f : FPS_CANDIDATES) {
                if (checkCamera(mgr, logical, ids, s[0], s[1], f) == null) camFps.put(f);
                String e = findAvcEncoder(s[0], s[1], f, ids.length);
                enc.put(String.valueOf(f), e == null ? JSONObject.NULL : e);
                kbps.put(String.valueOf(f), autoKbps(s[0], s[1], f, e));
            }
            p.put("camera_fps", camFps);            // fps values the app accepts for live (checkCamera == null)
            p.put("record_encoder", enc);           // fps -> encoder name for record/both, null = rejected
            p.put("rec_kbps_default", kbps);
            // Live H.264 (stream_fmt=h264, default) uses the same pick and bitrate; mode=both shares the recorder's
            // encoder, so live + record still needs only one encoder instance per lens.
            p.put("live_encoder", enc);
            p.put("live_kbps_default", kbps);
            pairs.put(p);
        }
        j.put("pairs", pairs);
        // Live H.264 summary (field names agreed with scan3d/load_model.py). Profile / mode / low latency = what
        // H264.configure would try first on the encoder picked for 640x480@15 (the applied values: metrics h264.settings).
        JSONObject hs = new JSONObject();
        hs.put("default_stream_fmt", "h264"); hs.put("magic", "STEH");
        hs.put("input", "buffer");                       // YUV copy into the encoder input Image (LiveEncoder.java why)
        hs.put("shared_with_record", true);              // mode=both: one encoder per lens feeds MP4 + live stream
        hs.put("b_frames", 0); hs.put("i_interval_s", 1); hs.put("priority", 0);
        hs.put("encoders_per_lens", 1); hs.put("instances_needed", ids.length);
        hs.put("max_b_frames_key_api", Build.VERSION.SDK_INT >= 29);
        String le = findAvcEncoder(640, 480, 15, ids.length);
        hs.put("encoder_640x480_15", le == null ? JSONObject.NULL : le);
        for (MediaCodecInfo ci : encs) if (ci.getName().equals(le)) {
            try {
                MediaCodecInfo.CodecCapabilities cc = ci.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC);
                hs.put("hardware", Build.VERSION.SDK_INT >= 29 && ci.isHardwareAccelerated());
                hs.put("profile", H264.high(cc) ? "high" : "baseline");
                hs.put("bitrate_mode", H264.modeName(H264.bitrateMode(cc)));
                hs.put("low_latency", H264.lowLatencyFeature(cc));
            } catch (Exception ex) { hs.put("error", ex.toString()); }
        }
        j.put("live_h264", hs);
        j.put("pair_lenses", new JSONArray(Arrays.asList(ids)));

        JSONArray ej = new JSONArray();
        for (MediaCodecInfo ci : encs) {
            JSONObject e = new JSONObject();
            e.put("name", ci.getName());
            if (Build.VERSION.SDK_INT >= 29) {
                e.put("hardware", ci.isHardwareAccelerated()); e.put("software_only", ci.isSoftwareOnly()); e.put("vendor", ci.isVendor());
            }
            try {
                MediaCodecInfo.CodecCapabilities cc = ci.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC);
                MediaCodecInfo.VideoCapabilities v = cc.getVideoCapabilities();
                e.put("max_instances", cc.getMaxSupportedInstances());
                e.put("yuv420_flexible", flexible(cc));
                // Live H.264 settings support (scan3d/SPEC.md 1.2.2). KEY_MAX_B_FRAMES cannot be queried per codec:
                // b_frames_possible = Main/High listed (Baseline has no B-frames); the app always sets 0 (API 29+).
                e.put("low_latency_feature", H264.lowLatencyFeature(cc));
                e.put("profile_high", H264.high(cc)); e.put("profile_baseline", H264.baseline(cc));
                e.put("b_frames_possible", H264.high(cc) || H264.main(cc));
                e.put("max_b_frames_key", Build.VERSION.SDK_INT >= 29);
                JSONObject bm = new JSONObject();
                bm.put("vbr", H264.modeSupported(cc, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR));
                bm.put("cbr", H264.modeSupported(cc, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR));
                bm.put("cq", H264.modeSupported(cc, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CQ));
                e.put("bitrate_modes", bm);
                e.put("live_bitrate_mode", H264.modeName(H264.bitrateMode(cc)));
                if (v != null) {
                    e.put("bitrate_range", range(v.getBitrateRange()));
                    e.put("widths", range(v.getSupportedWidths())); e.put("heights", range(v.getSupportedHeights()));
                    e.put("frame_rates", range(v.getSupportedFrameRates()));
                    e.put("width_align", v.getWidthAlignment()); e.put("height_align", v.getHeightAlignment());
                    JSONArray pp = new JSONArray();
                    if (Build.VERSION.SDK_INT >= 29 && v.getSupportedPerformancePoints() != null)
                        for (MediaCodecInfo.VideoCapabilities.PerformancePoint x : v.getSupportedPerformancePoints()) pp.put(x.toString());
                    e.put("performance_points", pp);
                    JSONArray sz = new JSONArray();
                    for (int[] s : common) {
                        JSONObject q = new JSONObject();
                        q.put("w", s[0]); q.put("h", s[1]);
                        JSONObject ok = new JSONObject();
                        for (int f : FPS_CANDIDATES) ok.put(String.valueOf(f), v.areSizeAndRateSupported(s[0], s[1], f));
                        q.put("size_rate_supported", ok);
                        Range<Double> ach = null;
                        try { ach = v.getAchievableFrameRatesFor(s[0], s[1]); } catch (Exception ignored) {}
                        q.put("achievable_fps", ach == null ? JSONObject.NULL
                                : new JSONArray().put(num(round(ach.getLower()))).put(num(round(ach.getUpper()))));
                        sz.put(q);
                    }
                    e.put("sizes", sz);
                }
            } catch (Exception ex) { e.put("error", ex.toString()); }
            ej.put(e);
        }
        j.put("avc_encoders", ej);
        j.put("session_support", sessionSupport(mgr, logical, ids, common));
        return j;
    }

    /** API 35 CameraDeviceSetup: asks the HAL, without opening the camera, whether one regular session with one YUV
     *  output per lens at that size is supported; again with the AE target range set to the highest accepted fps. */
    static JSONObject sessionSupport(CameraManager mgr, String logical, String[] ids, List<int[]> sizes) throws JSONException {
        JSONObject j = new JSONObject();
        j.put("api", "CameraDevice.CameraDeviceSetup.isSessionConfigurationSupported (API 35)");
        if (Build.VERSION.SDK_INT < 35) { j.put("available", false); j.put("why", "sdk " + Build.VERSION.SDK_INT); return j; }
        try {
            if (!mgr.isCameraDeviceSetupSupported(logical)) { j.put("available", false); j.put("why", "isCameraDeviceSetupSupported=false"); return j; }
            CameraDevice.CameraDeviceSetup setup = mgr.getCameraDeviceSetup(logical);
            JSONArray res = new JSONArray();
            for (int[] s : sizes) {
                JSONObject q = new JSONObject();
                q.put("w", s[0]); q.put("h", s[1]);
                try {
                    q.put("supported", setup.isSessionConfigurationSupported(session(ids, s, null)));
                    int best = 0;
                    for (int f : FPS_CANDIDATES) if (checkCamera(mgr, logical, ids, s[0], s[1], f) == null) best = f;
                    if (best > 0) {
                        CaptureRequest.Builder b = setup.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                        b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(best, best));
                        q.put("max_fps_checked", best);
                        q.put("supported_at_max_fps", setup.isSessionConfigurationSupported(session(ids, s, b.build())));
                    }
                } catch (Throwable e) { q.put("supported", JSONObject.NULL); q.put("error", e.toString()); }
                res.put(q);
            }
            j.put("available", true);
            j.put("results", res);
        } catch (Throwable e) { j.put("available", false); j.put("why", e.toString()); }
        return j;
    }

    private static SessionConfiguration session(String[] ids, int[] s, CaptureRequest params) {
        List<OutputConfiguration> outs = new ArrayList<>();
        for (String id : ids) {
            OutputConfiguration o = new OutputConfiguration(ImageFormat.YUV_420_888, new Size(s[0], s[1]));
            o.setPhysicalCameraId(id);
            outs.add(o);
        }
        SessionConfiguration sc = new SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outs);
        if (params != null) sc.setSessionParameters(params);
        return sc;
    }

    private static JSONArray range(Range<? extends Number> r) throws JSONException {
        return r == null ? null : new JSONArray().put(r.getLower()).put(r.getUpper());
    }

    static double round(double v) { return Math.round(v * 100.0) / 100.0; }

    /** org.json rejects NaN/Infinity. */
    static Object num(double v) { return Double.isNaN(v) || Double.isInfinite(v) ? JSONObject.NULL : (Object) v; }
}
