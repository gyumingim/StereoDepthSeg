package com.camera.dualstream;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.*;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.*;
import android.hardware.camera2.params.*;
import android.media.Image;
import android.media.ImageReader;
import android.util.*;
import android.view.*;
import android.widget.TextView;
import org.json.*;
import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.util.*;
import java.util.concurrent.*;

/** Public Camera2 only. Sources: developer.android.com/media/camera/camera2/multi-camera.
 * One logical device, explicit physical outputs. Live stream H.264 "STEH" (default since 1.4-h264), JPEG "STEJ" or
 * NV21 "STER" + physical sensor timestamp over ADB TCP (scan3d/SPEC.md 1.2). The PC may send one-line commands back on
 * the same socket ("keyframe [lens]").
 */
public class MainActivity extends Activity {
    private static final String TAG = "PhoneStereo";
    private final List<ImageReader> readers = new ArrayList<>();
    /** Frame queue: 6 for JPEG/NV21 (big packets), 12 for H.264 (small packets; an eviction costs a keyframe wait). */
    private volatile BlockingQueue<Packet> pending = new ArrayBlockingQueue<>(6);
    // IMU/environment samples share the socket with a different magic; own queue so frame backpressure
    // never drops motion samples and 100 Hz sensors never evict frames.
    // 400 Hz accel+gyro x2 + 100 Hz x5 + 30 Hz frame_timing ~ 1.4 k/s -> ~3 s of headroom.
    private final BlockingQueue<Packet> imuPending = new ArrayBlockingQueue<>(4096);
    private SensorManager sensors;
    private HandlerThread imuThread;    // sensor callbacks off the camera thread (NV21 copy loop blocks it)
    private int imuPeriodUs = 2500;     // intent extra imu_us: accel/gyro period, 0 = SENSOR_DELAY_FASTEST
    private HandlerThread cameraThread;
    private Handler handler;
    private CameraDevice camera;
    private CameraCaptureSession session;
    private volatile boolean running;
    private volatile Socket client;
    private ServerSocket server;
    private TextView text;
    private CameraManager manager;
    private String[] ids;
    private long[] counts;
    private int width, height;
    private int fps = 15;               // intent extra fps = CAPTURE fps; must have a fixed AE range [fps,fps] (Caps.checkCamera)
    /** intent extra save_fps (1.8-subsample, SPEC 1.2.3): frames kept per lens per second for recording AND live stream.
     *  0 / absent = fps (old behaviour, every frame). fps must be a multiple (60/15, 30/15). Encoders run at saveFps. */
    private int saveFps = 15;
    /** saveFps < fps: picks 1 of fps/saveFps frames per lens by timestamp before any copy (camera thread). null = off. */
    private Subsample subsample;
    /** Load counters -> metrics.json every 1 s (scan3d/SPEC.md 14.3). */
    private volatile Metrics metrics;
    private HandlerThread metricsThread;
    private volatile String lastStatus = "";
    private String recEncoder;          // H.264 codec picked by Caps.findAvcEncoder for this size/fps
    private int recKbps;
    private boolean rawTimestamps;      // intent extra raw_ts=true: send logical timestamps (old behaviour)
    private static final int MAGIC_FRAME = 0x53544552, MAGIC_SENSOR = 0x53454E53;   // "STER", "SENS"
    /** JPEG frame packet "STEJ": same header as STER, payload = one baseline JPEG of the NV21 frame (scan3d/SPEC.md 1.2). */
    private static final int MAGIC_JPEG = 0x5354454A;
    // intent extras stream_fmt=nv21|jpeg (default jpeg), jpeg_q (default 90). Raw NV21 at 640x480 x2 lenses x30 fps is
    // ~28 MB/s and dropped this phone off USB; JPEG cuts it ~10x. Only the live stream is encoded, recording stays H.264.
    private boolean streamJpeg = false;
    private int jpegQuality = 90;
    /** stream_fmt=h264|jpeg|nv21 (default h264, user decision 2026-10-16; jpeg/nv21 = advanced settings). */
    private String streamFmt = "h264";
    private boolean streamH264 = true;
    /** "STEH": 28-byte common header + payload [u32 flags][u32 seq][Annex-B NAL bytes] (scan3d/SPEC.md 1.2.2). */
    private static final int MAGIC_H264 = 0x53544548;
    private static final int KIND_NV21 = 0, KIND_JPEG = 1, KIND_H264 = 2;
    private volatile LiveEncoder liveEncoder;    // live-only H.264: one encoder per lens
    private volatile boolean liveShared;                  // mode=both + h264: live stream = recorder encoder output (no 2nd encoder)
    private String liveEncoderName;
    private int liveKbps;
    /** Per lens: 1 = skip H.264 P-frames until the next keyframe (new client, or a frame was evicted from the queue). */
    private java.util.concurrent.atomic.AtomicIntegerArray waitKey;
    private final java.util.concurrent.atomic.AtomicLong keyRequests = new java.util.concurrent.atomic.AtomicLong();
    /** Live H.264 source generation: bumped whenever a new encoder (Recorder or LiveEncoder) becomes the live source.
     *  Output of an older source (e.g. a recorder still finishing after a stop timeout) is ignored (REVIEW7 H3). */
    private volatile int liveGen;
    private volatile String liveError;                 // last live encoder error (metrics.json h264.error, REVIEW7 H2)
    private volatile int liveErrorGen;                 // liveGen of the source that reported liveError
    /** offerFrame eviction and the serve thread's waitKey check run under this lock (REVIEW7 H1). */
    private final Object pendingLock = new Object();
    /** Per lens "loss epoch" (guarded by pendingLock): bumped on every queue eviction / lost encoder output, stamped on
     *  each H.264 packet in offerFrame. Only a keyframe stamped with the CURRENT epoch (= produced after the last loss)
     *  may end waitKey; a keyframe already queued (or already polled by serve) before the loss cannot (REVIEW7 확인). */
    private int[] lossEpoch;
    /** Keyframe request limit per lens (REVIEW7 확인): queue evictions, lost outputs and PC requests all pass it.
     *  Inside the limit only keyReqDeferred is set; the next encoder output after the limit sends one request. */
    private static final long KEY_REQ_MIN_NS = 400_000_000L;
    private final Object keyReqLock = new Object();
    private long[] keyReqAtNs;                         // guarded by keyReqLock; Long.MIN_VALUE = never sent
    private boolean[] keyReqDeferred;                  // guarded by keyReqLock
    private final java.util.concurrent.atomic.AtomicLong keyRequestsLimited = new java.util.concurrent.atomic.AtomicLong();
    /** One single-thread encoder per lens: keeps per-lens order (the PC drops frames whose timestamp goes back)
     *  and never blocks the camera thread. jpegBusy caps the backlog per lens; extra frames are dropped and counted. */
    private ExecutorService[] jpegEncoders;
    private java.util.concurrent.atomic.AtomicInteger[] jpegBusy;
    private volatile long jpegDropped = 0;
    private static final int JPEG_MAX_INFLIGHT = 2;
    public static final String ACTION_STOP = "com.camera.dualstream.STOP";

    // Recording mode (intent extra mode=live|record|both). Format: scan3d/RECORDING_FORMAT.md
    private String mode = "live";
    private boolean streamEnabled = true, recordEnabled = false;
    private volatile Recorder recorder;
    private JSONObject recMeta;
    /** Folder name of the current/last recording: kept in every later status line (rec=...) so the PC never loses it. */
    private volatile String lastRecName;
    /** Saved across recreate() (screen off/on): the recording already ended in onStop, do not open a new folder. */
    private static final String STATE_REC_DONE = "rec_done_name";
    private BroadcastReceiver stopReceiver;
    /** Per lens: physical (or logical when raw_ts) SENSOR_TIMESTAMP -> {logical ts, exposure ns, rs skew ns,
     *  mid offset ns, source}. Filled for every result (the mid-exposure frame time needs it, not only the recorder).
     *  Written and read on the camera handler thread only. */
    private List<LinkedHashMap<Long,long[]>> frameMeta;
    /** Frame time since 1.7-mid (scan3d/SPEC.md 1.2): mid-exposure = physical SENSOR_TIMESTAMP + SENSOR_EXPOSURE_TIME/2
     *  + SENSOR_ROLLING_SHUTTER_SKEW/2 of that lens. Clock unchanged (elapsedRealtimeNanos, same as the IMU). */
    static final String TS_REF = "mid_exposure";
    /** Sensor packet type 101 "stream_info" (SPEC 1.4): values [ts_ref code] (1 = mid exposure). Sent at connect and
     *  every 30 results, so the PC knows the frame headers are already mid-exposure times. */
    private static final int SENSOR_STREAM_INFO = 101;
    private static final float TS_REF_CODE_MID = 1f;
    /** mid offset value source per frame: 0 physical result, 1 logical result (lens result had no value),
     *  2 last known value of that lens (this result had none), 3 none known yet (mid = start). Camera thread. */
    static final int MID_PHYS = 0, MID_LOGICAL = 1, MID_LAST = 2, MID_NONE = 3;
    private long[] lastExpNs, lastSkewNs, lastMidOffNs;  // per lens, -1 = never known (camera thread)
    /** Frames sent/recorded per mid offset source + frames with no result entry (raw_ts image before its result or
     *  expired entry -> last offset of that lens). metrics.json / meta.json "mid_ts". */
    private final java.util.concurrent.atomic.AtomicLongArray midSrc = new java.util.concurrent.atomic.AtomicLongArray(4);
    private final java.util.concurrent.atomic.AtomicLong midNoResult = new java.util.concurrent.atomic.AtomicLong();

    /** Sensor event timestamps and camera timestamps (SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME on this device) are both
     *  SystemClock.elapsedRealtimeNanos(), so the PC can look up the IMU state at each frame's exposure time. */
    private final SensorEventListener imu = new SensorEventListener() {
        @Override public void onSensorChanged(SensorEvent e) {
            Recorder rec=recorder;
            if(rec!=null) rec.sensor(e.sensor.getType(),e.timestamp,e.values.clone());
            if(client==null) return;
            ByteBuffer b=ByteBuffer.allocate(4*e.values.length);          // big endian, like the frame header
            for(float v:e.values) b.putFloat(v);
            Packet p=new Packet(e.sensor.getType(),e.timestamp,b.array());
            if(!imuPending.offer(p)) { imuPending.poll(); imuPending.offer(p); }
        }
        @Override public void onAccuracyChanged(Sensor s,int accuracy) {}
    };

    /** Per-physical SENSOR_TIMESTAMP from the logical result: the HAL-reported per-sensor exposure timestamps
     *  associated with this capture (sync type APPROXIMATE). Sent as sensor packet type 100 = "frame_timing":
     *  [phys[0] offset ms, phys[1] offset ms, (phys[2] offset ms), exposure ms, frame duration ms, rolling shutter skew ms]
     *  offsets are relative to the logical SENSOR_TIMESTAMP that the ImageReader frames carry. Also logged as RESULT lines. */
    private long timingCount=0;
    private CaptureTimestampMatcher timestampMatcher;
    /** Lens phase alignment (scan3d/SPEC.md 15). null = off (intent sync=off). */
    private volatile PhaseSync phaseSync;
    private String syncMode="auto";                    // intent extra sync=auto|c|a|b|off (auto = c, SPEC 15.3)
    /** intent extra rec_wait_sync_s (default 40 = about 25 camera re-opens at 15 fps): mode=record|both wait for the lock. */
    private int recWaitSyncS=40;
    /** mode=record|both: frames go to the recorder only after the lenses are in phase (or rec_wait_sync_s passed).
     *  Written on the camera thread, read by PhaseSync.mayRestart / withSync on other threads. */
    private volatile boolean recGate;
    private long recGateUntilNs, recGateT0Ns;
    private String lastSyncSuffix="";                  // guarded by statusLock
    /** status() / syncChanged() / h264Error rewrite run under this lock; lastBase = last status line without the sync
     *  suffix, so a sync rewrite never brings back an older line (REVIEW8 L1). Lock order: statusLock -> PhaseSync. */
    private final Object statusLock=new Object();
    private String lastBase="";
    private float focusDiopters=1.0f;
    private CaptureRequest currentRequest;             // what the repeating request runs now (camera thread)
    private Range<Integer> sensRange;                  // target lens SENSOR_INFO_SENSITIVITY_RANGE (method A iso clamp)
    /** 1.12-shortexp: less motion blur. exp_cap_us > 0 = after AE meters (~15 frames) switch to manual exposure
     *  min(AE exposure, cap) and raise ISO to keep the same brightness (exposure x ISO); if ISO hits the lens max the
     *  exposure stays as short as that ISO allows. Only without phase sync (1 lens or sync=off). exp_gain < 1 = darker. */
    private long expCapNs; private double expGain=1.0;
    private long fixExpNs; private int fixIso; private int meterFrames;
    private Range<Integer> logicalSensRange;
    private List<CaptureRequest.Key<?>> physKeys;      // logical getAvailablePhysicalCameraRequestKeys (method A, REVIEW8 H1)
    private String logicalId;
    /** Method C camera re-open (camera thread only): reopening = waiting for onOpened of the new device. */
    private boolean reopening;
    private int reopenSeq, reopenOpenFails;
    private CameraDevice.StateCallback deviceCb;
    private final CameraCaptureSession.CaptureCallback timing=new CameraCaptureSession.CaptureCallback() {
        @Override public void onCaptureCompleted(CameraCaptureSession s,CaptureRequest r,TotalCaptureResult result) {
            try {
                if(!running) return;
                Long ts=result.get(CaptureResult.SENSOR_TIMESTAMP); if(ts==null) return;
                if(expCapNs>0 && fixExpNs==0 && phaseSync==null) meterShortExposure(result);
                Map<String,CaptureResult> phys=result.getPhysicalCameraResults();
                float[] v=new float[ids.length+3];
                Long[] physicalTimestamps=new Long[ids.length];
                StringBuilder sb=new StringBuilder("RESULT ts="+ts);
                for(int i=0;i<ids.length;i++) {
                    CaptureResult p=phys.get(ids[i]); Long pts=p==null?null:p.get(CaptureResult.SENSOR_TIMESTAMP);
                    v[i]=pts==null?Float.NaN:(pts-ts)/1e6f;
                    physicalTimestamps[i]=pts;
                    sb.append(" phys").append(ids[i]).append("=").append(pts==null?"null":String.valueOf(pts-ts));
                }
                Long exp=result.get(CaptureResult.SENSOR_EXPOSURE_TIME), dur=result.get(CaptureResult.SENSOR_FRAME_DURATION), skew=result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                v[ids.length]=exp==null?Float.NaN:exp/1e6f; v[ids.length+1]=dur==null?Float.NaN:dur/1e6f; v[ids.length+2]=skew==null?Float.NaN:skew/1e6f;
                sb.append(" exp=").append(exp).append(" dur=").append(dur).append(" skew=").append(skew);
                // Mid-exposure offset per lens (before the matcher emits, so enqueueFrame finds it): the lens's own
                // physical exposure / skew, else the logical value, else the last known value of that lens.
                long[] midOff=new long[ids.length];
                for(int i=0;i<ids.length;i++) {
                    CaptureResult p=phys.get(ids[i]);
                    Long pe=p==null?null:p.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                    Long pk=p==null?null:p.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                    int src=MID_PHYS;
                    if(pe==null || pk==null) src=MID_LOGICAL;
                    if(pe==null) pe=exp;
                    if(pk==null) pk=skew;
                    if(pe!=null && pe>=0) lastExpNs[i]=pe; else src=MID_LAST;
                    if(pk!=null && pk>=0) lastSkewNs[i]=pk; else src=MID_LAST;
                    long e=lastExpNs[i], k=lastSkewNs[i];
                    if(src==MID_LAST && e<0 && k<0) src=MID_NONE;
                    midOff[i]=(Math.max(0,e)+Math.max(0,k))/2;
                    lastMidOffNs[i]=midOff[i];
                    long[] m={ts,e,k,midOff[i],src};
                    Long key=rawTimestamps?ts:physicalTimestamps[i];
                    if(key!=null) frameMeta.get(i).put(key,m);
                }
                Subsample sub=subsample;
                if(sub!=null) sub.result(ts,physicalTimestamps,midOff);
                else if(!rawTimestamps) timestampMatcher.result(ts,physicalTimestamps,SystemClock.elapsedRealtimeNanos());
                PhaseSync ps=phaseSync;
                if(ps!=null && physicalTimestamps[ps.ref]!=null && physicalTimestamps[ps.tgt]!=null) {
                    CaptureResult tr=phys.get(ids[ps.tgt]);
                    long tR=physicalTimestamps[ps.ref], tT=physicalTimestamps[ps.tgt];
                    ps.onResult(tR,tT,tR+midOff[ps.ref],tT+midOff[ps.tgt],SystemClock.elapsedRealtimeNanos(),
                            tr==null?null:tr.get(CaptureResult.SENSOR_EXPOSURE_TIME),tr==null?null:tr.get(CaptureResult.SENSOR_SENSITIVITY),
                            tr==null?null:tr.get(CaptureResult.SENSOR_FRAME_DURATION));
                }
                sb.append(" matched=").append(timestampMatcher.matched).append(" unmatched_dropped=").append(timestampMatcher.dropped);
                if(timingCount++%30==0) Log.i(TAG,sb.toString());
                Recorder rec=recorder;
                if(rec!=null) rec.sensor(100,ts,v.clone());
                if(client!=null) {
                    ByteBuffer b=ByteBuffer.allocate(4*v.length); for(float f:v) b.putFloat(f);
                    Packet pk=new Packet(100,ts,b.array());
                    if(!imuPending.offer(pk)) { imuPending.poll(); imuPending.offer(pk); }
                    if(timingCount%30==1) {             // stream_info again now and then (a lost first one is not fatal)
                        Packet si=streamInfo();
                        if(!imuPending.offer(si)) { imuPending.poll(); imuPending.offer(si); }
                    }
                }
            } catch(Exception e) { Log.e(TAG,"timing",e); }
        }
        @Override public void onCaptureFailed(CameraCaptureSession s,CaptureRequest r,CaptureFailure f) {
            Metrics mt=metrics; if(mt!=null) mt.captureFailed.incrementAndGet();
        }
        @Override public void onCaptureBufferLost(CameraCaptureSession s,CaptureRequest r,Surface target,long frame) {
            Metrics mt=metrics; if(mt==null) return;
            for(int i=0;i<readers.size();i++) if(target.equals(readers.get(i).getSurface())) mt.bufferLost(i);
        }
    };

    /** Sensor packet type 101 "stream_info" (SPEC 1.4): [1.0] = frame header timestamps are mid-exposure times. */
    private static Packet streamInfo() {
        return new Packet(SENSOR_STREAM_INFO,SystemClock.elapsedRealtimeNanos(),ByteBuffer.allocate(4).putFloat(TS_REF_CODE_MID).array());
    }

    private void startSensors() {
        sensors=(SensorManager)getSystemService(SENSOR_SERVICE);
        imuThread=new HandlerThread("imu",android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY); imuThread.start();
        Handler imuHandler=new Handler(imuThread.getLooper());
        // Raw accel (specific force incl. gravity) + gyro for VIO at imu_us (default 2500 us = 400 Hz; above 200 Hz
        // needs HIGH_SAMPLING_RATE_SENSORS in the manifest). Fused/environment sensors stay at 10 ms (100 Hz).
        int[] fast={Sensor.TYPE_ACCELEROMETER,Sensor.TYPE_GYROSCOPE};
        int[] slow={Sensor.TYPE_GRAVITY,Sensor.TYPE_LINEAR_ACCELERATION,
                    Sensor.TYPE_GAME_ROTATION_VECTOR,Sensor.TYPE_ROTATION_VECTOR,Sensor.TYPE_PRESSURE};
        int fastUs=imuPeriodUs<=0?SensorManager.SENSOR_DELAY_FASTEST:imuPeriodUs;
        List<String> got=new ArrayList<>();
        JSONArray info=new JSONArray();
        for(int pass=0;pass<2;pass++) for(int t:pass==0?fast:slow) {
            Sensor s=sensors.getDefaultSensor(t);
            if(s==null) continue;
            int us=pass==0?fastUs:10000;
            boolean ok=sensors.registerListener(imu,s,us,imuHandler);
            if(ok) got.add(t+":"+s.getName());
            try {
                JSONObject j=new JSONObject();
                j.put("type",t); j.put("name",s.getName()); j.put("vendor",s.getVendor());
                j.put("requested_us",us); j.put("min_delay_us",s.getMinDelay()); j.put("max_delay_us",s.getMaxDelay());
                j.put("fifo_max",s.getFifoMaxEventCount()); j.put("resolution",s.getResolution());
                j.put("max_range",s.getMaximumRange()); j.put("registered",ok);
                info.put(j);
            } catch(JSONException e) { Log.e(TAG,"sensor info",e); }
        }
        try (FileOutputStream f=openFileOutput("sensors.json",MODE_PRIVATE)) {
            f.write(info.toString(2).getBytes("UTF-8"));
            Recorder rec=recorder;
            if(rec!=null) writeFile(new File(rec.dir,"sensors.json"),info.toString(2));
        } catch(Exception e) { Log.e(TAG,"sensors.json",e); }
        Log.i(TAG,"sensors imu_us="+imuPeriodUs+" "+got);
    }

    private static class Packet {
        final int index;
        final long timestamp;
        final byte[] data;
        final int kind, flags, seq;
        int epoch;                                     // H.264 only: lossEpoch[index] when queued (offerFrame, pendingLock)
        Packet(int i, long t, byte[] d) { this(i,t,d,KIND_NV21,0,0); }
        Packet(int i, long t, byte[] d, int kind, int flags, int seq) { index=i; timestamp=t; data=d; this.kind=kind; this.flags=flags; this.seq=seq; }
    }

    /** Encoded H.264 frames from LiveEncoder or (mode=both) the recorder's encoder. Encoder threads; never blocks.
     *  gen = source generation; call as liveSink(++liveGen) when creating a new live source (REVIEW7 H3). */
    private H264.Sink liveSink(final int gen) {
        return new H264.Sink() {
            @Override public void h264(int lens,long sensorTs,byte[] annexB,int flags,int seq,long encodeNs) {
                if(gen!=liveGen) return;                       // old source still draining -> never mix into the stream
                // A newer source is producing normally -> the error of an older source no longer applies (REVIEW7 확인 H2-2)
                if(liveError!=null && liveErrorGen!=gen) liveError=null;
                boolean key=(flags&H264.FLAG_KEY)!=0;
                Metrics mt=metrics;
                if(mt!=null) mt.h264(lens,encodeNs,annexB.length,key,System.nanoTime());
                keyframeTick(lens,key);
                if(!running || client==null || !streamEnabled) return;
                offerFrame(new Packet(lens,sensorTs,annexB,KIND_H264,flags,seq));
            }
            @Override public void h264Drop(int lens) { if(gen!=liveGen) return; Metrics mt=metrics; if(mt!=null) mt.h264Drop(lens); }
            @Override public void h264Lost(int lens) {
                if(gen!=liveGen) return;
                Metrics mt=metrics; if(mt!=null) mt.h264Lost(lens);
                java.util.concurrent.atomic.AtomicIntegerArray wk=waitKey;
                if(wk==null || lens<0 || lens>=wk.length()) return;
                // Same lock + epoch as a queue eviction: a keyframe of this lens still in the queue (from before this lost
                // output) can no longer end the wait, so the P-frames after the lost one never reach the PC (REVIEW7 확인).
                synchronized(pendingLock) { wk.set(lens,1); if(lossEpoch!=null) lossEpoch[lens]++; }
                requestKeyframe(lens);
            }
            @Override public void h264Error(int lens,String error) {
                if(gen!=liveGen) return;
                liveError=error; liveErrorGen=gen;
                Recorder rec=recorder;
                if(rec!=null || liveShared) {
                    // RECORDING / RECORDING+STREAMING: keep the first word (the PC's is_recording / stop wait read it; an
                    // ERROR first word would end the stop wait before the other lenses' MP4s are finished). Only a warn=
                    // suffix here; the error text goes to metrics.json h264.error (REVIEW7 확인 H2-1).
                    if(rec==null) return;                      // recording is being stopped: metrics only, status untouched
                                                               // (else this line could overwrite RECORD_DONE; REVIEW7 확인2)
                    synchronized(statusLock) {                 // read-modify-write under the status lock (REVIEW8 L1)
                        String base=lastBase==null?"":lastBase;
                        if(!base.startsWith("RECORDING")) base=runningState();
                        int w=base.indexOf(" warn=");
                        if(w>=0) base=base.substring(0,w);
                        status(base+" warn=LIVE_ENCODER lens="+lens);
                    }
                } else status("ERROR LIVE_ENCODER lens="+lens+" "+error+" | try stream_fmt=jpeg");
            }
        };
    }

    /** Keyframe on one lens (or all when lens < 0): PARAMETER_KEY_REQUEST_SYNC_FRAME on the live encoder, at most once per
     *  KEY_REQ_MIN_NS per lens. Inside the limit the request is remembered and sent by keyframeTick once the limit ends.
     *  waitKey is NOT touched here (callers set it); a stuck USB link therefore cannot turn every frame into an IDR. */
    private void requestKeyframe(int lens) {
        long[] at=keyReqAtNs; boolean[] def=keyReqDeferred;
        if(at==null || def==null) { sendKeyframe(lens); return; }
        long now=System.nanoTime();
        boolean all=true, any=false;
        boolean[] send=new boolean[at.length];
        synchronized(keyReqLock) {
            for(int i=0;i<at.length;i++) {
                if(lens>=0 && lens!=i) continue;
                if(at[i]==Long.MIN_VALUE || now-at[i]>=KEY_REQ_MIN_NS) { at[i]=now; def[i]=false; send[i]=true; any=true; }
                else { def[i]=true; all=false; keyRequestsLimited.incrementAndGet(); }
            }
        }
        if(lens<0 && all) { sendKeyframe(-1); return; }
        if(any) for(int i=0;i<send.length;i++) if(send[i]) sendKeyframe(i);
    }

    /** Every encoder output of the current source (encoder thread): a keyframe answers any remembered request; otherwise
     *  a remembered request whose limit has ended goes out now (one request, REVIEW7 확인). */
    private void keyframeTick(int lens,boolean key) {
        long[] at=keyReqAtNs; boolean[] def=keyReqDeferred;
        if(at==null || def==null || lens<0 || lens>=at.length) return;
        boolean fire=false;
        synchronized(keyReqLock) {
            if(!def[lens]) return;
            if(key) def[lens]=false;
            else if(System.nanoTime()-at[lens]>=KEY_REQ_MIN_NS) fire=true;
        }
        if(fire) requestKeyframe(lens);
    }

    private void sendKeyframe(int lens) {
        keyRequests.incrementAndGet();
        if(liveShared) { Recorder rec=recorder; if(rec!=null) rec.requestKey(lens); return; }
        LiveEncoder le=liveEncoder; if(le!=null) le.requestKey(lens);
    }

    /** One-line command from the PC on the stream socket (scan3d/SPEC.md 1.2.2): "keyframe" or "keyframe <lens index>". */
    private void command(String line) {
        if(line.isEmpty()) return;
        String[] w=line.split("\\s+");
        if(w[0].equalsIgnoreCase("keyframe")) {
            int lens=-1;
            if(w.length>1) try { lens=Integer.parseInt(w[1]); } catch(NumberFormatException e) { Log.w(TAG,"bad keyframe lens: "+line); return; }
            if(streamH264) requestKeyframe(lens);
            Log.i(TAG,"command keyframe lens="+lens);
        } else Log.w(TAG,"unknown command: "+line);
    }

    private static String stripSync(String s) {
        return s==null?null:s.replaceAll("\\s+ts_ref=\\S+","").replaceAll("\\s+sync=\\S+(\\s+(?:phase|method|try|rec_wait)=\\S+)*","");
    }

    /** mode=record|both and the recorder still waits for the lock: " rec_wait=1" (PC shows "맞춘 뒤 녹화 시작"). */
    private String gateTag() { return recGate && recorder!=null ? " rec_wait=1" : ""; }

    /** Running states (STREAMING / RECORDING / RECORDING+STREAMING) get the lens phase suffix
     *  " sync=aligning phase=+3.2ms method=C try=4 [rec_wait=1]" or " sync=locked phase=+0.4ms method=C" (SPEC 15.4).
     *  The first word never changes, so phone_link / win_relay parse the line as before; an older suffix is replaced. */
    private String withSync(String s) {
        if(s==null) return s;
        String t=s.trim();
        String first=t.isEmpty()?"":t.split("\\s+",2)[0];
        boolean run=false;
        for(String part:first.split("\\+")) if(part.equals("STREAMING")||part.equals("RECORDING")) run=true;
        if(!run || first.contains("ERROR")) return s;
        String base=stripSync(s);
        PhaseSync ps=phaseSync;
        // " ts_ref=mid": frame timestamps are mid-exposure times (1.7-mid, SPEC 1.2); old apps have no ts_ref
        String suf=" ts_ref=mid"+(ps==null?"":ps.suffix()+gateTag());
        int w=base.indexOf(" warn=");                     // keep warn=... last (h264Error strips from there)
        return w>=0?base.substring(0,w)+suf+base.substring(w):base+suf;
    }

    /** PhaseSync state/phase changed (posted to the camera thread, never called with the PhaseSync lock held):
     *  rewrite the current running status line from lastBase with the new suffix (REVIEW8 L1). */
    private void syncChanged() {
        PhaseSync ps=phaseSync; if(ps==null) return;
        synchronized(statusLock) {
            String suf=ps.suffix()+gateTag();
            if(suf.equals(lastSyncSuffix)) return;
            lastSyncSuffix=suf;
            Log.i(TAG,"sync"+suf);
            String b=lastBase;
            if(b!=null && !b.isEmpty()) { String n=withSync(b); if(!n.equals(lastStatus)) status(b); }
        }
    }

    private void status(String s) {
        synchronized(statusLock) {
            lastBase = stripSync(s);
            s = withSync(lastBase);
            Log.i(TAG, s);
            lastStatus = s;
            final String shown = s;
            runOnUiThread(() -> text.setText("Phone Stereo\n\n" + shown + "\n\nUSB → PC\n화면을 켜둔 상태에서 사용합니다."));
            try (FileOutputStream f = openFileOutput("status.txt", MODE_PRIVATE)) {
                f.write(s.getBytes("UTF-8"));
            } catch (IOException e) { Log.e(TAG,"status write",e); }
            // Copy readable without run-as: /sdcard/Android/data/com.camera.dualstream/files/status.txt
            File ext=getExternalFilesDir(null);
            if(ext!=null) try { writeFile(new File(ext,"status.txt"),s); } catch(IOException e) { Log.w(TAG,"ext status",e); }
        }
    }

    private static void writeFile(File f,String s) throws IOException {
        try (FileOutputStream o=new FileOutputStream(f)) { o.write(s.getBytes("UTF-8")); }
    }

    /** "STREAMING" (live, prefix unchanged for win_relay), "RECORDING" (record), "RECORDING+STREAMING" (both). */
    private String runningState() {
        Recorder rec=recorder;
        String recPart=rec!=null?" rec="+rec.dir.getName():"";
        if(rec!=null && streamEnabled) return "RECORDING+STREAMING"+recPart;
        if(rec!=null) return "RECORDING"+recPart;
        return "STREAMING";
    }

    private void startRecording(String logical,String physical,int fps,float focus) throws Exception {
        File base=getExternalFilesDir(null);
        if(base==null) throw new IOException("external files dir unavailable");
        String name="rec_"+new java.text.SimpleDateFormat("yyyyMMdd_HHmmss",Locale.US).format(new Date());
        File dir=new File(base,name);
        // rec_kbps extra overrides; default scales with pixels x fps (4 Mbps per lens at 640x480@15, Caps.autoKbps).
        int kbpsExtra=getIntent().getIntExtra("rec_kbps",0);
        int kbps=kbpsExtra>0?kbpsExtra:Caps.autoKbps(width,height,saveFps,recEncoder);
        recKbps=kbps;
        // 1.10-liveupload: also write the append-only .h264 (+ .h264.csv index) that UploadService sends while recording
        boolean liveUp=UploadService.liveWanted(this);
        boolean raw=getIntent().getBooleanExtra("rec_raw",false) || liveUp;
        // encoder / MP4 run at the SAVED rate; frames CSV gets pair_dt_ns when subsampling
        Recorder rec=new Recorder(dir,ids,width,height,saveFps,kbps,raw,recEncoder,liveShared?liveSink(++liveGen):null,subsample!=null);
        recMeta=new JSONObject();
        recMeta.put("format","scan3d-phone-rec/1");
        recMeta.put("mode",mode); recMeta.put("folder",name);
        recMeta.put("logical",logical); recMeta.put("physical",new JSONArray(Arrays.asList(ids)));
        // fps = frames per second IN the files (saved rate, what the PC pairs); capture_fps = camera rate (1.8-subsample)
        recMeta.put("width",width); recMeta.put("height",height); recMeta.put("fps",saveFps);
        recMeta.put("capture_fps",fps); recMeta.put("save_fps",saveFps);
        recMeta.put("imu_us",imuPeriodUs); recMeta.put("focus_diopters",focus);
        recMeta.put("raw_ts",rawTimestamps); recMeta.put("rec_kbps",kbps); recMeta.put("rec_raw_h264",raw);
        recMeta.put("live_upload",liveUp);
        recMeta.put("rec_kbps_source",kbpsExtra>0?"extra":"auto"); recMeta.put("encoder",recEncoder==null?"default":recEncoder);
        recMeta.put("i_frame_interval_s",1); recMeta.put("max_b_frames",0); recMeta.put("codec","video/avc");
        recMeta.put("live_shares_encoder",liveShared);
        recMeta.put("sync_mode",phaseSync==null?"off":syncMode);
        // mode=record|both: frames reach the MP4s after the lens phase lock (max rec_wait_sync_s). Per-frame phase is also
        // in the frame_timing (type 100) sensor rows: physical offsets of both lenses (SPEC 15.5)
        recMeta.put("rec_wait_sync_s",phaseSync!=null?recWaitSyncS:0);
        recMeta.put("timestamp_clock","SystemClock.elapsedRealtimeNanos (camera SENSOR_TIMESTAMP and SensorEvent.timestamp)");
        // frames CSV t_mid_ns (frame time) = physical_ts_ns + exposure_ns/2 + rolling_shutter_skew_ns/2 (1.7-mid)
        recMeta.put("timestamp_ref",TS_REF);
        recMeta.put("start_wall_ms",System.currentTimeMillis());
        recMeta.put("start_wall",new java.text.SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ",Locale.US).format(new Date()));
        recMeta.put("start_elapsed_ns",SystemClock.elapsedRealtimeNanos());
        try {
            android.content.pm.PackageInfo pi=getPackageManager().getPackageInfo(getPackageName(),0);
            recMeta.put("app_version",pi.versionName); recMeta.put("app_version_code",pi.versionCode);
        } catch(Exception e) { recMeta.put("app_version","unknown"); }
        recMeta.put("complete",false);
        writeFile(new File(dir,"meta.json"),recMeta.toString(2));
        try { copyFile(new File(getFilesDir(),"cameras.json"),new File(dir,"cameras.json")); }
        catch(IOException e) { Log.w(TAG,"cameras.json copy",e); }
        recorder=rec;
        lastRecName=name;
        UploadService.recordingNow=name;          // only the append-only files go up while it is written (1.10)
        UploadService.poke();
        Log.i(TAG,"recording to "+dir);
    }

    private static void copyFile(File a,File b) throws IOException {
        try (InputStream i=new FileInputStream(a); OutputStream o=new FileOutputStream(b)) {
            byte[] buf=new byte[8192]; int n; while((n=i.read(buf))>0) o.write(buf,0,n);
        }
    }

    /** Finish encoders/muxers and write the final meta.json. Safe to call more than once. */
    private synchronized void stopRecording(String reason) {
        Recorder rec=recorder;
        if(rec==null) return;
        recorder=null;
        boolean done=rec.stop(3000);
        final JSONObject meta=recMeta;
        try {
            meta.put("stop_reason",reason);
            meta.put("end_wall_ms",System.currentTimeMillis());
            meta.put("end_wall",new java.text.SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ",Locale.US).format(new Date()));
            meta.put("end_elapsed_ns",SystemClock.elapsedRealtimeNanos());
            meta.put("matcher_unmatched_dropped",timestampMatcher==null?0:timestampMatcher.dropped);
            meta.put("mid_ts",midJson());
            Subsample sub=subsample;
            if(sub!=null) meta.put("subsample",sub.json(ids,false));   // pairs, |dt| median/p95/max of the whole recording
            PhaseSync ps=phaseSync;
            synchronized(meta) { meta.put("sync",ps==null?JSONObject.NULL:ps.json()); }
            writeFinalMeta(rec,meta,done);
        } catch(Exception e) { Log.e(TAG,"meta.json",e); }
        if(!done) {
            // An encoder is still finishing after the timeout: meta says complete=false + finishing=true now and is
            // rewritten with the real muxer result when the last lens closes.
            rec.whenFinished(() -> {
                try { writeFinalMeta(rec,meta,true); Log.i(TAG,"meta.json rewritten after late finish "+rec.dir); }
                catch(Exception e) { Log.e(TAG,"meta.json (late)",e); }
            });
        }
        Log.i(TAG,"recording stopped ("+reason+") "+rec.dir+(done?"":" (encoder still finishing)"));
        if(liveShared && streamEnabled && running) {
            // mode=both: the live stream rode on the recorder encoder. Recording ended, streaming continues -> own encoder.
            liveShared=false; liveKbps=recKbps;
            for(int i=0;i<ids.length;i++) waitKey.set(i,1);
            // new generation first: a recorder that timed out (done=false) keeps draining into a sink that now ignores it
            liveEncoder=new LiveEncoder(ids,width,height,saveFps,liveKbps,liveEncoderName,liveSink(++liveGen));
            Log.i(TAG,"live H.264 moved from recorder to own encoder");
        }
        UploadService.recordingNow=null;          // meta "finishing" still keeps a late-closing folder out
        UploadService.poke();                     // live upload: send the tails + finish now, not after a 5 s idle wait
        status("RECORD_DONE rec="+rec.dir.getName()+" reason="+reason
                +(streamEnabled&&running?" | STREAMING continues":""));
    }

    /** frames/drops/lenses from the recorder; complete=true only when every lens closed (muxer result known). */
    private void writeFinalMeta(Recorder rec,JSONObject meta,boolean done) throws Exception {
        synchronized(meta) {
            JSONObject frames=new JSONObject(), drops=new JSONObject();
            for(int i=0;i<ids.length;i++) { frames.put(ids[i],rec.frames(i)); drops.put(ids[i],rec.drops(i)); }
            meta.put("frames",frames); meta.put("dropped",drops);
            meta.put("lenses",rec.lensStats());
            meta.put("imu_samples",rec.imuSamples()); meta.put("imu_dropped",rec.imuDrops());
            // max_b_frames=0 above is what was REQUESTED; this is what the encoder output really had (slice_type of every
            // output frame, REVIEW7 H5). true = PTS order != decode order in the MP4s (replay must not assume I+P only).
            meta.put("b_frames_seen",rec.bFramesSeen());
            meta.put("complete",done);
            if(done) meta.remove("finishing"); else meta.put("finishing",true);
            writeFile(new File(rec.dir,"meta.json"),meta.toString(2));
        }
    }

    @Override public void onSaveInstanceState(Bundle out) {
        super.onSaveInstanceState(out);
        if(lastRecName!=null && !"live".equals(mode)) out.putString(STATE_REC_DONE,lastRecName);
    }

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setShowWhenLocked(true);
        setTurnScreenOn(true);
        text = new TextView(this); text.setTextSize(18); text.setPadding(32,64,32,32);
        setContentView(text);
        UploadService.activityAlive=true;
        UploadService.kick(this);                 // 1.9-upload: send finished recordings (resumes interrupted ones)
        manager = (CameraManager)getSystemService(CAMERA_SERVICE);
        cameraThread = new HandlerThread("camera"); cameraThread.start();
        handler = new Handler(cameraThread.getLooper());
        startCapsQuery();
        try {
            JSONArray inventory = inventory();
            try (FileOutputStream f=openFileOutput("cameras.json",MODE_PRIVATE)) {
                f.write(inventory.toString(2).getBytes("UTF-8"));
            }
            String logical=getIntent().getStringExtra("logical");
            String physical=getIntent().getStringExtra("physical");
            if(logical==null || physical==null) { status("Inventory ready. Select cameras from PC."); return; }
            ids=physical.split(","); counts=new long[ids.length];
            timestampMatcher=new CaptureTimestampMatcher(ids.length,this::enqueueFrame);
            frameMeta=new ArrayList<>();
            for(int i=0;i<ids.length;i++) frameMeta.add(new LinkedHashMap<Long,long[]>(64,0.75f,false) {
                @Override protected boolean removeEldestEntry(Map.Entry<Long,long[]> e) { return size()>64; }
            });
            lastExpNs=new long[ids.length]; lastSkewNs=new long[ids.length]; lastMidOffNs=new long[ids.length];
            Arrays.fill(lastExpNs,-1); Arrays.fill(lastSkewNs,-1);
            // 1 lens (1.11-mono): single-camera recordings, e.g. lens 5 only for ChArUco ground truth (scan3d IMU_PLAN.md)
            if(ids.length<1 || ids.length>3 || new HashSet<>(Arrays.asList(ids)).size()!=ids.length)
                throw new IllegalArgumentException("Need 1, 2 or 3 distinct physical IDs");
            Set<String> available=manager.getCameraCharacteristics(logical).getPhysicalCameraIds();
            for(String id:ids) if(!available.contains(id)) throw new IllegalArgumentException("Not in logical group: "+id);
            width=getIntent().getIntExtra("width",640); height=getIntent().getIntExtra("height",480);
            fps=getIntent().getIntExtra("fps",15);
            int sf=getIntent().getIntExtra("save_fps",0);
            saveFps=sf>0?sf:fps;
            rawTimestamps=getIntent().getBooleanExtra("raw_ts",false);
            imuPeriodUs=getIntent().getIntExtra("imu_us",2500);
            expCapNs=Math.max(0,getIntent().getIntExtra("exp_cap_us",0))*1000L;
            expGain=Math.max(0.1,Math.min(1.0,getIntent().getFloatExtra("exp_gain",1.0f)));
            fixExpNs=0; fixIso=0; meterFrames=0; meterEv=0; meterExp=0; meterN=0;
            String fmt=getIntent().getStringExtra("stream_fmt");
            fmt=fmt==null?"h264":fmt.trim().toLowerCase(Locale.US);
            if(!fmt.equals("h264") && !fmt.equals("jpeg") && !fmt.equals("nv21"))
                throw new IllegalArgumentException("stream_fmt must be h264|jpeg|nv21: "+fmt);
            streamFmt=fmt; streamJpeg=fmt.equals("jpeg"); streamH264=fmt.equals("h264");
            waitKey=new java.util.concurrent.atomic.AtomicIntegerArray(ids.length);
            lossEpoch=new int[ids.length];
            long[] kat=new long[ids.length]; Arrays.fill(kat,Long.MIN_VALUE);
            keyReqDeferred=new boolean[ids.length]; keyReqAtNs=kat;
            jpegQuality=Math.max(1,Math.min(100,getIntent().getIntExtra("jpeg_q",90)));
            String m=getIntent().getStringExtra("mode");
            mode=m==null?"live":m.trim().toLowerCase(Locale.US);
            if(!mode.equals("live") && !mode.equals("record") && !mode.equals("both"))
                throw new IllegalArgumentException("mode must be live|record|both: "+m);
            streamEnabled=!mode.equals("record"); recordEnabled=!mode.equals("live");
            String recDone=state==null?null:state.getString(STATE_REC_DONE);
            if(recordEnabled && recDone!=null) {
                // recreate() after screen off/on: that recording was finished in onStop. Do not start a second folder.
                lastRecName=recDone; recordEnabled=false;
                if(!streamEnabled) {
                    status("RECORD_DONE rec="+recDone+" reason=activity_stop | screen was off; start a new recording from PC");
                    return;
                }
            }
            // Unsupported size/fps -> ERROR, never the nearest supported size (caps.json lists what works).
            String why=Caps.checkCamera(manager,logical,ids,width,height,fps);
            if(why==null && (saveFps>fps || fps%saveFps!=0)) why="save_fps "+saveFps+" must divide fps "+fps;
            if(why==null && saveFps<fps && rawTimestamps) why="save_fps needs raw_ts=false (picks frames by physical time)";
            if(why==null && recordEnabled) {
                recEncoder=Caps.findAvcEncoder(width,height,saveFps,ids.length);
                if(recEncoder==null) why="no H.264 encoder for "+ids.length+" x "+width+"x"+height+"@"+saveFps;
            }
            liveShared=streamEnabled && streamH264 && recordEnabled;
            if(why==null && streamEnabled && streamH264) {
                // Live-only: own encoder per lens. both: the recorder's encoder (same settings) feeds the stream too,
                // so live + record is still one encoder instance per lens (Caps max_instances >= lenses).
                liveEncoderName=liveShared?recEncoder:Caps.findAvcEncoder(width,height,saveFps,ids.length);
                if(liveEncoderName==null) why="no H.264 encoder for live "+ids.length+" x "+width+"x"+height+"@"+saveFps+" (try stream_fmt=jpeg)";
                int lk=getIntent().getIntExtra("live_kbps",0);
                liveKbps=liveShared?0:(lk>0?lk:Caps.autoKbps(width,height,saveFps,liveEncoderName));
            }
            if(why!=null) {
                status("ERROR UNSUPPORTED "+width+"x"+height+"@"+fps+" mode="+mode+" reason="+why+" | see caps.json");
                return;
            }
            String sm=getIntent().getStringExtra("sync");
            syncMode=sm==null?"auto":sm.trim().toLowerCase(Locale.US);
            if(syncMode.equals("c")) syncMode="auto";
            if(!syncMode.equals("auto") && !syncMode.equals("a") && !syncMode.equals("b") && !syncMode.equals("off"))
                throw new IllegalArgumentException("sync must be auto|c|a|b|off: "+sm);
            recWaitSyncS=Math.max(0,getIntent().getIntExtra("rec_wait_sync_s",40));
            focusDiopters=getIntent().getFloatExtra("focus_diopters",1.0f);
            logicalId=logical;
            if(!syncMode.equals("off") && ids.length>=2) phaseSync=createPhaseSync(logical);   // nothing to align with 1 lens
            // record and both: align (camera re-open) BEFORE the MP4s get frames; no re-open once they do (SPEC 15.5).
            // both: the live stream rides the recorder encoder, so it also starts after the lock.
            recGate=recordEnabled && phaseSync!=null && recWaitSyncS>0;
            if(saveFps<fps) {
                int r=0; for(int i=0;i<ids.length;i++) if(ids[i].equals("2")) r=i;   // reference lens = "2" (VIO lens)
                subsample=new Subsample(ids.length,r,fps,saveFps,this::keepFrame);
            }
            metrics=new Metrics(ids);
            running=true;
            UploadService.liveNow=streamEnabled;     // uploads wait while the live stream needs the uplink
            startMetrics();
            if(recordEnabled) {
                startRecording(logical,physical,fps,getIntent().getFloatExtra("focus_diopters",1.0f));
                stopReceiver=new BroadcastReceiver() {
                    @Override public void onReceive(Context c,Intent i) {
                        // Finishing encoders blocks briefly; keep it off the main thread.
                        final PendingResult pr=goAsync();
                        new Thread(() -> {
                            try {
                                stopRecording("broadcast");
                                // Keep rec=<name> so the PC (rec_status, pull guard) still knows which folder just finished.
                                if(!streamEnabled) runOnUiThread(() -> { stop();
                                    status("RECORD_DONE rec="+lastRecName+" reason=broadcast | capture stopped");
                                    // 1.9-upload: the service keeps the process; close the screen-on activity
                                    if(UploadService.config(MainActivity.this)!=null) finish(); });
                            } finally { pr.finish(); }
                        },"rec-stop").start();
                    }
                };
                IntentFilter filter=new IntentFilter(ACTION_STOP);
                if(Build.VERSION.SDK_INT>=33) registerReceiver(stopReceiver,filter,Context.RECEIVER_EXPORTED);
                else registerReceiver(stopReceiver,filter);
            }
            if(streamEnabled) {
                if(streamH264) {
                    pending=new ArrayBlockingQueue<>(12);
                    if(!liveShared) liveEncoder=new LiveEncoder(ids,width,height,saveFps,liveKbps,liveEncoderName,liveSink(++liveGen));
                }
                if(streamJpeg) {
                    jpegEncoders=new ExecutorService[ids.length]; jpegBusy=new java.util.concurrent.atomic.AtomicInteger[ids.length];
                    for(int i=0;i<ids.length;i++) {
                        final int lens=i;
                        jpegEncoders[i]=Executors.newSingleThreadExecutor(r -> {
                            Thread t=new Thread(r,"jpeg-"+lens); t.setPriority(Thread.NORM_PRIORITY); return t; });
                        jpegBusy[i]=new java.util.concurrent.atomic.AtomicInteger();
                    }
                }
                server=new ServerSocket(8765,1,InetAddress.getByName("127.0.0.1"));
                new Thread(this::serve,"usb-writer").start();
            }
            status("OPEN mode="+mode+" logical="+logical+" physical="+physical+" "+width+"x"+height+" fps="+fps
                    +(saveFps<fps?" save_fps="+saveFps:"")
                    +(streamEnabled?" stream_fmt="+(streamJpeg?"jpeg q="+jpegQuality:streamH264?"h264 enc="+liveEncoderName
                        +" kbps="+(liveShared?recKbps+" (shared with recording)":String.valueOf(liveKbps)):"nv21"):""));
            deviceCb=new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice d) {
                    if(!running) { d.close(); return; }
                    camera=d;
                    if(reopening) reopened(); else configure();
                }
                @Override public void onDisconnected(CameraDevice d) {
                    if(d!=camera && camera!=null) { d.close(); return; }          // an old device of a re-open
                    status("DISCONNECTED"); d.close();
                }
                @Override public void onError(CameraDevice d,int e) {
                    if(reopening && camera==null) { d.close(); reopenOpenFailed(reopenSeq,"CAMERA_ERROR "+e); return; }
                    status("CAMERA_ERROR "+e); d.close();
                }
            };
            manager.openCamera(logical,deviceCb,handler);
            startSensors();
        } catch(Exception e) { status("ERROR "+e); Log.e(TAG,"startup",e); }
    }

    /** Lens phase alignment for the pair (target = lens "5" when selected, else ids[1]; SPEC 15). */
    private PhaseSync createPhaseSync(String logical) throws Exception {
        int t=1; for(int i=0;i<ids.length;i++) if(ids[i].equals("5")) t=i;
        String target=ids[t];
        Caps.MethodA a=Caps.methodA(manager,logical,target);
        CameraCharacteristics tc=manager.getCameraCharacteristics(target);
        long minDur=Math.max(0,Caps.minFrameNs(tc,width,height));
        Long maxDur=tc.get(CameraCharacteristics.SENSOR_INFO_MAX_FRAME_DURATION);
        sensRange=tc.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
        try { physKeys=manager.getCameraCharacteristics(logical).getAvailablePhysicalCameraRequestKeys(); }
        catch(Exception e) { physKeys=null; }
        Log.i(TAG,"phase sync mode="+syncMode+" target="+target+" method_a="+a.possible+" ae_key="+a.aeKey+" ("+a.reason+")");
        return new PhaseSync(ids,fps,syncMode,a,minDur,maxDur==null?0:maxDur,new PhaseSync.Actions() {
            @Override public boolean applyDuration(long durNs,long expNs,int iso) { return applyLensDuration(durNs,expNs,iso); }
            @Override public boolean restartRepeating() { return restartRepeatingRequest(); }
            @Override public boolean recreateSession() { return recreateCaptureSession(); }
            @Override public boolean reopenCamera() { Handler h=handler; return h!=null && running && h.post(MainActivity.this::doReopen); }
            @Override public boolean mayRestart() { return mayRestartCamera(); }
            // posted: PhaseSync calls this with its own lock held; syncChanged takes statusLock (REVIEW8 L1 lock order)
            @Override public void changed() { Handler h=handler; if(h!=null) h.post(MainActivity.this::syncChanged); }
        });
    }

    /** false once frames go into an MP4 (no gate, or the gate opened): restarting the camera would cut the MP4. */
    private boolean mayRestartCamera() { return recorder==null || recGate; }

    /** Method C step (camera thread): close the CameraDevice, open it again. ImageReaders, encoders, the recorder and
     *  the socket stay; the new device gets a new session on the same surfaces (reopened()). */
    private void doReopen() {
        PhaseSync ps=phaseSync;
        long now=SystemClock.elapsedRealtimeNanos();
        if(ps==null) return;
        if(!running || camera==null || reopening) { ps.cancelReopen(now,"camera not running"); return; }
        if(!mayRestartCamera()) { ps.cancelReopen(now,"recording started"); return; }
        final int seq=++reopenSeq;
        reopening=true; reopenOpenFails=0;
        CameraCaptureSession s=session; session=null;
        CameraDevice d=camera; camera=null;
        try { if(s!=null) s.close(); } catch(Exception e) { Log.w(TAG,"reopen: session close",e); }
        try { d.close(); } catch(Exception e) { Log.w(TAG,"reopen: device close",e); }
        if(timestampMatcher!=null) timestampMatcher.clear();     // half pairs of the old device
        if(subsample!=null) subsample.clear();
        ps.reopenClosed();
        openForReopen(seq);
        handler.postDelayed(() -> reopenWatchdog(seq),5000);
    }

    private void openForReopen(int seq) {
        if(seq!=reopenSeq || !running || !reopening) return;
        try { manager.openCamera(logicalId,deviceCb,handler); }
        catch(Exception e) { Log.w(TAG,"reopen: openCamera",e); reopenOpenFailed(seq,e.toString()); }
    }

    /** Open of the new device failed: try again (max 3), then ERROR (the PC restarts the app). */
    private void reopenOpenFailed(int seq,String why) {
        if(seq!=reopenSeq || !running) return;
        if(++reopenOpenFails<=3) { handler.postDelayed(() -> openForReopen(seq),700); return; }
        reopening=false;
        PhaseSync ps=phaseSync; if(ps!=null) ps.reopenFailed(SystemClock.elapsedRealtimeNanos(),why);
        status("ERROR CAMERA_REOPEN_FAILED "+why);
    }

    /** 5 s after a re-open: no device -> open again; device but no session -> new session once more. */
    private void reopenWatchdog(int seq) {
        if(seq!=reopenSeq || !running) return;
        if(camera==null) { Log.w(TAG,"reopen: no onOpened in 5 s"); reopenOpenFailed(seq,"open timeout"); }
        else if(session==null) {
            Log.w(TAG,"reopen: no session in 5 s");
            try { createSession(false,1); } catch(Exception e) { status("ERROR CONFIGURE_FAILED (phase sync) "+e); }
        }
    }

    /** onOpened of a re-opened device (camera thread): new request for this device, session on the same readers. */
    private void reopened() {
        reopening=false; reopenOpenFails=0;
        try {
            currentRequest=buildRequest(0,0,0);
            createSession(false,0);
        } catch(Exception e) { status("ERROR CONFIGURE_FAILED (phase sync) "+e); Log.e(TAG,"reopen session",e); }
    }

    /** caps.json (scan3d/SPEC.md 14.2): query only, background thread so the camera start is not delayed.
     *  Lenses = intent physical (default 2,5) of logical (default 0). Written to files/ and external files/. */
    private void startCapsQuery() {
        final String logical=getIntent().getStringExtra("logical")!=null?getIntent().getStringExtra("logical"):"0";
        final String physical=getIntent().getStringExtra("physical")!=null?getIntent().getStringExtra("physical"):"2,5";
        Thread t=new Thread(() -> {
            try {
                String doc=Caps.build(this,manager,logical,physical.split(",")).toString(2);
                writeAtomic(getFilesDir(),"caps.json",doc);
                File ext=getExternalFilesDir(null);
                if(ext!=null) writeAtomic(ext,"caps.json",doc);
                Log.i(TAG,"caps.json written ("+doc.length()+" chars)");
            } catch(Throwable e) { Log.e(TAG,"caps.json",e); }
        },"caps");
        t.setPriority(Thread.MIN_PRIORITY);
        t.start();
    }

    /** Write name.tmp then rename, so a reader (adb shell cat) never sees half a file. */
    private static void writeAtomic(File dir,String name,String s) throws IOException {
        File tmp=new File(dir,name+".tmp"), dst=new File(dir,name);
        writeFile(tmp,s);
        if(!tmp.renameTo(dst)) { writeFile(dst,s); tmp.delete(); }
    }

    private long[] prevRecFrames;
    private long prevRecNs, prevCpuMs=-1, prevCpuNs, prevUsbBytes, prevUsbNs, thermalAt;
    private double thermalHeadroom=Double.NaN;

    private void startMetrics() {
        metricsThread=new HandlerThread("metrics"); metricsThread.start();
        final Handler h=new Handler(metricsThread.getLooper());
        h.post(new Runnable() { @Override public void run() {
            if(!running) return;
            writeMetrics();
            h.postDelayed(this,1000);
        }});
    }

    /** "mid_ts" counters (metrics.json / meta.json): frames by mid offset source (SPEC 1.2). Any thread. */
    private JSONObject midJson() throws JSONException {
        JSONObject o=new JSONObject();
        o.put("ref",TS_REF);
        o.put("from_physical",midSrc.get(MID_PHYS));
        o.put("from_logical",midSrc.get(MID_LOGICAL));
        o.put("last_known",midSrc.get(MID_LAST));      // this result had no exposure/skew -> last known value of that lens
        o.put("unknown",midSrc.get(MID_NONE));         // nothing known yet -> mid = start
        o.put("no_result",midNoResult.get());          // no result entry for the frame -> last offset of that lens
        return o;
    }

    /** metrics.json (scan3d/SPEC.md 14.3), metrics thread only. */
    private void writeMetrics() {
        Metrics mt=metrics; if(mt==null) return;
        try {
            long nowNs=System.nanoTime();
            JSONObject j=new JSONObject();
            j.put("format","scan3d-phone-metrics/1");
            j.put("t_wall_ms",System.currentTimeMillis()); j.put("elapsed_ns",SystemClock.elapsedRealtimeNanos());
            String st=lastStatus==null?"":lastStatus.trim();
            j.put("state",st.isEmpty()?"":st.split("[ |]",2)[0]);
            // fps = saved/streamed rate (frames the PC gets); capture_fps = camera rate (phone load, 1.8-subsample)
            j.put("mode",mode); j.put("width",width); j.put("height",height); j.put("fps",saveFps);
            j.put("capture_fps",fps); j.put("save_fps",saveFps);
            Subsample sub=subsample;
            if(sub!=null) j.put("subsample",sub.json(ids,true));
            j.put("physical",new JSONArray(Arrays.asList(ids)));
            j.put("stream_fmt",streamEnabled?streamFmt:"none"); j.put("jpeg_q",jpegQuality);
            if(streamEnabled && streamH264) {
                JSONObject h=new JSONObject();
                Recorder r0=recorder; LiveEncoder le=liveEncoder;
                JSONObject set=liveShared?(r0==null?null:r0.encoderInfo()):(le==null?null:le.info);
                h.put("encoder",liveEncoderName); h.put("shared_with_recorder",liveShared);
                h.put("kbps_target",liveShared?recKbps:liveKbps);
                j.put("live_kbps",liveShared?recKbps:liveKbps);   // target per lens (scan3d/load_model.py field)
                h.put("settings",set==null?JSONObject.NULL:set);
                h.put("key_requests",keyRequests.get());
                h.put("key_requests_limited",keyRequestsLimited.get());   // held back by the 0.4 s per-lens limit
                h.put("gen",liveGen);                                     // live source generation (PC clears old errors)
                String err=le==null?null:le.error();
                if(err==null) err=liveError;
                if(err!=null) h.put("error",err);
                j.put("h264",h);
            }
            j.put("window_s",Metrics.WINDOW_NS/1e9);
            Recorder rec=recorder;
            long[] rf=null, rq=null, rb=null; double[] rfps=null;
            if(rec!=null) {
                int n=ids.length; rf=new long[n]; rq=new long[n]; rb=new long[n]; rfps=new double[n];
                for(int i=0;i<n;i++) {
                    rf[i]=rec.frames(i); rq[i]=rec.drops(i); rb[i]=rec.busy(i);
                    rfps[i]=prevRecFrames!=null&&nowNs>prevRecNs?(rf[i]-prevRecFrames[i])*1e9/(nowNs-prevRecNs):Double.NaN;
                }
                prevRecFrames=rf; prevRecNs=nowNs;
                j.put("rec",rec.dir.getName()); j.put("rec_kbps",recKbps); j.put("rec_encoder",recEncoder);
            } else prevRecFrames=null;
            mt.fill(j,nowNs,rf,rfps,rq,rb);   // lenses[].cam_fps = capture rate, rec_fps / h264_fps = saved rate
            j.put("capture_failed",mt.captureFailed.get());
            CaptureTimestampMatcher tm=timestampMatcher;
            j.put("matcher_unmatched_dropped",tm==null?0:tm.dropped);
            j.put("timestamp_ref",TS_REF);
            j.put("mid_ts",midJson());
            PhaseSync ps=phaseSync;
            JSONObject sj=ps==null?new JSONObject().put("state","off").put("mode",syncMode):ps.json();
            sj.put("rec_wait",recGate && recorder!=null);           // recorder still waits for the lock
            sj.put("rec_wait_sync_s",recordEnabled?recWaitSyncS:0);
            j.put("sync",sj);
            JSONObject usb=new JSONObject();
            long ub=mt.usbBytes.get();
            usb.put("client",client!=null);
            usb.put("bytes_total",ub);
            usb.put("MBps",prevUsbNs>0&&nowNs>prevUsbNs?Caps.round((ub-prevUsbBytes)/1e6/((nowNs-prevUsbNs)/1e9)):0);
            prevUsbBytes=ub; prevUsbNs=nowNs;
            usb.put("frame_queue_evicted",mt.queueEvicted.get());
            j.put("usb",usb);
            JSONObject ph=new JSONObject();
            long cpu=android.os.Process.getElapsedCpuTime();                 // ms of CPU used by this process
            ph.put("proc_cpu_pct_one_core",prevCpuMs>=0&&nowNs>prevCpuNs?Caps.num(Caps.round((cpu-prevCpuMs)*1e6*100/(nowNs-prevCpuNs))):JSONObject.NULL);
            prevCpuMs=cpu; prevCpuNs=nowNs;
            ph.put("cpu_cores",Runtime.getRuntime().availableProcessors());
            try {
                Intent b=registerReceiver(null,new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
                int t=b==null?Integer.MIN_VALUE:b.getIntExtra(BatteryManager.EXTRA_TEMPERATURE,Integer.MIN_VALUE);
                ph.put("battery_temp_c",t==Integer.MIN_VALUE?JSONObject.NULL:(Object)(t/10.0));
            } catch(Exception e) { ph.put("battery_temp_c",JSONObject.NULL); }
            PowerManager pm=(PowerManager)getSystemService(POWER_SERVICE);
            if(pm!=null && Build.VERSION.SDK_INT>=29) ph.put("thermal_status",pm.getCurrentThermalStatus());
            if(pm!=null && Build.VERSION.SDK_INT>=30 && nowNs-thermalAt>10_000_000_000L) {   // headroom: at most every 10 s
                thermalHeadroom=pm.getThermalHeadroom(10); thermalAt=nowNs;
            }
            ph.put("thermal_headroom_10s",Caps.num(Caps.round(thermalHeadroom)));
            j.put("phone",ph);
            j.put("upload",UploadService.state);
            String doc=j.toString(1);
            writeAtomic(getFilesDir(),"metrics.json",doc);
            File ext=getExternalFilesDir(null);
            if(ext!=null) writeAtomic(ext,"metrics.json",doc);
        } catch(Exception e) { Log.w(TAG,"metrics",e); }
    }

    private JSONObject characteristics(String id) throws Exception {
        CameraCharacteristics c=manager.getCameraCharacteristics(id);
        JSONObject j=new JSONObject();
        j.put("id",id); j.put("facing",c.get(CameraCharacteristics.LENS_FACING));
        j.put("focal_mm",array(c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)));
        j.put("sensor_mm",String.valueOf(c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)));
        j.put("active_array",String.valueOf(c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)));
        j.put("orientation",c.get(CameraCharacteristics.SENSOR_ORIENTATION));
        j.put("timestamp_source",c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE));
        j.put("sync_type",c.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE));
        j.put("intrinsics",array(c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)));
        j.put("distortion",array(c.get(CameraCharacteristics.LENS_DISTORTION)));
        j.put("pose_translation",array(c.get(CameraCharacteristics.LENS_POSE_TRANSLATION)));
        j.put("pose_rotation",array(c.get(CameraCharacteristics.LENS_POSE_ROTATION)));
        j.put("pose_reference",c.get(CameraCharacteristics.LENS_POSE_REFERENCE));
        // 0 = fixed focus. Stereo needs a fixed focal length: autofocus breathing shifts fx/fy by ~0.5%.
        j.put("min_focus_diopters",c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE));
        j.put("focus_calibration",c.get(CameraCharacteristics.LENS_INFO_FOCUS_DISTANCE_CALIBRATION));
        JSONArray sizes=new JSONArray();
        StreamConfigurationMap m=c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        if(m!=null && m.getOutputSizes(ImageFormat.YUV_420_888)!=null)
            for(Size s:m.getOutputSizes(ImageFormat.YUV_420_888)) sizes.put(s.toString());
        j.put("yuv_sizes",sizes);
        j.put("fps_ranges",String.valueOf(Arrays.toString(c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES))));
        return j;
    }
    private JSONArray array(float[] values) throws JSONException {
        JSONArray a=new JSONArray(); if(values!=null) for(float v:values) a.put(v); return a;
    }
    private JSONArray inventory() throws Exception {
        JSONArray all=new JSONArray();
        for(String id:manager.getCameraIdList()) {
            JSONObject j=characteristics(id); JSONArray phys=new JSONArray();
            for(String p:manager.getCameraCharacteristics(id).getPhysicalCameraIds()) {
                try { phys.put(characteristics(p)); }
                catch(Exception e) { phys.put(new JSONObject().put("id",p).put("error",e.toString())); }
            }
            j.put("physical",phys); all.put(j);
        }
        return all;
    }

    /** Base repeating request; durNs > 0 = method A override on the target lens: SENSOR_FRAME_DURATION (+ AE off with
     *  the last AE exposure/iso of that lens when those are physical keys). Camera thread. */
    private CaptureRequest buildRequest(long durNs,long expNs,int iso) throws CameraAccessException {
        PhaseSync ps=phaseSync;
        boolean override=durNs>0 && ps!=null;
        // setPhysicalCameraKey needs a builder created with the physical id set (API 28); the normal request stays
        // the plain template request exactly as before 1.5.
        CaptureRequest.Builder request=override
                ?camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW,new HashSet<>(Arrays.asList(ids)))
                :camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        // A builder with physical ids gets per-lens settings filled from the template: copy every logical value that is a
        // physical request key to each lens too, or AF/OIS/fps range of BOTH lenses go back to defaults (REVIEW8 H1).
        setKey(request,override,CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
        setKey(request,override,CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
        // Fixed manual focus (diopters = 1/m). Autofocus changes the wide lens focal length between frames
        // (focus breathing), which moved rectified dy by 1-2 px between captures. 0 keeps autofocus.
        if(focusDiopters>0f) {
            setKey(request,override,CaptureRequest.CONTROL_AF_MODE,CaptureRequest.CONTROL_AF_MODE_OFF);
            setKey(request,override,CaptureRequest.LENS_FOCUS_DISTANCE,focusDiopters);
        }
        // Fixed AE fps range [fps,fps]: AE never changes the frame duration by itself (the phase control relies on it).
        Range<Integer>[] ranges=manager.getCameraCharacteristics(camera.getId()).get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        if(ranges!=null) for(Range<Integer> r:ranges)
            if(r.getLower()==fps && r.getUpper()==fps) { setKey(request,override,CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,r); break; }
        if(fixExpNs>0 && fixIso>0 && !override) {          // short manual exposure (meterShortExposure)
            setKey(request,false,CaptureRequest.CONTROL_AE_MODE,CaptureRequest.CONTROL_AE_MODE_OFF);
            setKey(request,false,CaptureRequest.SENSOR_EXPOSURE_TIME,fixExpNs);
            setKey(request,false,CaptureRequest.SENSOR_SENSITIVITY,fixIso);
            setKey(request,false,CaptureRequest.SENSOR_FRAME_DURATION,1_000_000_000L/Math.max(1,fps));
        }
        for(ImageReader reader:readers) request.addTarget(reader.getSurface());
        if(override) {
            String id=ids[ps.tgt];
            request.setPhysicalCameraKey(CaptureRequest.SENSOR_FRAME_DURATION,durNs,id);
            if(ps.aeKey && expNs>0 && iso>0) {
                // exposure must stay below the (shorter) frame duration or the HAL stretches the frame: clamp, keep
                // brightness with iso (within the lens sensitivity range)
                long exp=Math.min(expNs,durNs-500_000L);
                int is=(int)Math.round(iso*(double)expNs/Math.max(1,exp));
                if(sensRange!=null) is=Math.max(sensRange.getLower(),Math.min(sensRange.getUpper(),is));
                request.setPhysicalCameraKey(CaptureRequest.CONTROL_AE_MODE,CaptureRequest.CONTROL_AE_MODE_OFF,id);
                request.setPhysicalCameraKey(CaptureRequest.SENSOR_EXPOSURE_TIME,exp,id);
                request.setPhysicalCameraKey(CaptureRequest.SENSOR_SENSITIVITY,is,id);
            }
        }
        return request.build();
    }

    /** Logical value; with physical=true also the same value on every selected lens when it is a physical key. */
    private <T> void setKey(CaptureRequest.Builder b,boolean physical,CaptureRequest.Key<T> k,T v) {
        b.set(k,v);
        if(physical && physKeys!=null && physKeys.contains(k)) for(String id:ids) b.setPhysicalCameraKey(k,v,id);
    }

    /** Camera thread: average AE exposure x ISO over frames 5..15, then fix a short exposure (see expCapNs). */
    private double meterEv, meterExp; private int meterN;
    private void meterShortExposure(TotalCaptureResult r) {
        Long e=r.get(CaptureResult.SENSOR_EXPOSURE_TIME); Integer is=r.get(CaptureResult.SENSOR_SENSITIVITY);
        if(e==null || is==null || e<=0 || is<=0) return;
        meterFrames++;
        if(meterFrames<5) return;                          // AE still settling
        meterEv+=e*(double)is; meterExp+=e; meterN++;
        if(meterFrames<15) return;
        double ev=meterEv/meterN*expGain, aeExp=meterExp/meterN;   // ns x ISO, ns
        int isoMax=logicalSensRange!=null?logicalSensRange.getUpper():3200, isoMin=logicalSensRange!=null?logicalSensRange.getLower():50;
        long exp=(long)Math.min(expCapNs,aeExp);                  // never longer than AE chose
        if(ev/exp>isoMax) exp=(long)(ev/isoMax);                   // ISO at max -> shortest exposure that keeps brightness
        int iso=(int)Math.max(isoMin,Math.min(isoMax,Math.round(ev/Math.max(1,exp))));
        fixExpNs=exp; fixIso=iso;
        Log.i(TAG,"short exposure: AE ev "+(long)ev+" -> exp "+exp/1000+" us iso "+iso+" (cap "+expCapNs/1000+" us, iso max "+isoMax+")");
        status(runningState()+" exp_us="+exp/1000+" iso="+iso);
        try {
            CameraCaptureSession s=session;
            if(s!=null) { currentRequest=buildRequest(0,0,0); s.setRepeatingRequest(currentRequest,timing,handler); }
        } catch(Exception ex) { Log.w(TAG,"short exposure request failed",ex); fixExpNs=0; }
    }

    private void configure() {
        try {
            for(int i=0;i<ids.length;i++) {
                final int index=i;
                // subsample holds a few undecided images per lens (Subsample.HOLD_MAX) -> deeper reader
                ImageReader reader=ImageReader.newInstance(width,height,ImageFormat.YUV_420_888,subsample!=null?Subsample.HOLD_MAX+3:3);
                readers.add(reader);
                reader.setOnImageAvailableListener(r -> receive(r,index),handler);
            }
            try { logicalSensRange=manager.getCameraCharacteristics(ids[0]).get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE); }
            catch(Exception ignored) { }
            currentRequest=buildRequest(0,0,0);
            createSession(true,0);
        } catch(Exception e) { status("CONFIGURE_ERROR "+e); Log.e(TAG,"configure",e); }
    }

    /** One regular session, one physical YUV output per lens (the existing ImageReaders). first=false: method B
     *  re-creation (creating a session closes the previous one) or method C (new device). attempt: 0, retry 1.
     *  Camera thread. */
    private void createSession(final boolean first,final int attempt) throws CameraAccessException {
        List<OutputConfiguration> outputs=new ArrayList<>();
        for(int i=0;i<ids.length;i++) {
            OutputConfiguration out=new OutputConfiguration(readers.get(i).getSurface());
            out.setPhysicalCameraId(ids[i]); outputs.add(out);
        }
        SessionConfiguration config=new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,outputs,
            command -> handler.post(command),new CameraCaptureSession.StateCallback() {
                @Override public void onConfigured(CameraCaptureSession s) {
                    if(!running || camera==null) { s.close(); return; }
                    session=s;
                    try {
                        s.setRepeatingRequest(currentRequest,timing,handler);
                        if(first) status(runningState()+" logical="+camera.getId()+" physical="+Arrays.toString(ids)+" focus_diopters="+focusDiopters+" mode="+mode);
                        else Log.i(TAG,"session re-created (phase sync)");
                    } catch(Exception e) { status("REQUEST_FAILED "+e); }
                }
                @Override public void onConfigureFailed(CameraCaptureSession s) {
                    if(first) { status("CONFIGURE_FAILED"); return; }
                    // phase sync re-creation failed: one more try, then a clear ERROR (REVIEW8 L2; the PC restarts the app)
                    if(attempt<1 && running && camera!=null) {
                        Log.w(TAG,"session re-create failed, retry");
                        handler.postDelayed(() -> {
                            if(!running || camera==null || session!=null) return;
                            try { createSession(false,attempt+1); }
                            catch(Exception e) { status("ERROR CONFIGURE_FAILED (phase sync) "+e); }
                        },300);
                    } else status("ERROR CONFIGURE_FAILED (phase sync)");
                }
            });
        if(first) {
            try { Log.i(TAG,"session_supported="+camera.isSessionConfigurationSupported(config)); }
            catch(Exception e) { Log.i(TAG,"session query unavailable: "+e); }
        }
        camera.createCaptureSession(config);
    }

    /** PhaseSync method A (camera thread). durNs <= 0 = back to the normal request. */
    private boolean applyLensDuration(long durNs,long expNs,int iso) {
        CameraCaptureSession s=session;
        if(s==null || camera==null || !running) return false;
        try {
            CaptureRequest r=buildRequest(durNs,expNs,iso);
            s.setRepeatingRequest(r,timing,handler);
            currentRequest=r;
            return true;
        } catch(Exception e) {
            Log.w(TAG,"phase sync: duration request failed",e);
            if(durNs>0) try { currentRequest=buildRequest(0,0,0); s.setRepeatingRequest(currentRequest,timing,handler); } catch(Exception ignored) {}
            return false;
        }
    }

    /** PhaseSync method B step 1: stop and start the repeating request (camera thread). */
    private boolean restartRepeatingRequest() {
        CameraCaptureSession s=session;
        if(s==null || !running) return false;
        try { s.stopRepeating(); s.setRepeatingRequest(currentRequest,timing,handler); return true; }
        catch(Exception e) { Log.w(TAG,"phase sync: repeating restart failed",e); return false; }
    }

    /** PhaseSync method B step 2: new capture session on the same outputs (camera thread). */
    private boolean recreateCaptureSession() {
        if(camera==null || !running) return false;
        try {
            CameraCaptureSession old=session; session=null;
            if(old!=null) { try { old.stopRepeating(); } catch(Exception ignored) {} }
            createSession(false,0);
            return true;
        } catch(Exception e) { Log.w(TAG,"phase sync: session re-create failed",e); return false; }
    }

    private void receive(ImageReader reader,int index) {
        if(subsample!=null) { receiveSub(reader,index); return; }
        try (Image image=reader.acquireLatestImage()) {
            if(image==null || !running) return;
            counts[index]++;
            if(counts[index]==1 || counts[index]%150==0)
                Log.i(TAG,"FRAME physical="+ids[index]+" count="+counts[index]+" ts="+image.getTimestamp());
            Metrics mt=metrics;
            if(client==null && recorder==null) { if(mt!=null) mt.camFrame(index,image.getTimestamp(),0,fps); return; }
            long c0=System.nanoTime();
            byte[] nv21=toNv21(image);
            if(mt!=null) mt.camFrame(index,image.getTimestamp(),System.nanoTime()-c0,fps);
            if(rawTimestamps) enqueueFrame(index,image.getTimestamp(),nv21,Long.MIN_VALUE);
            else timestampMatcher.image(index,image.getTimestamp(),nv21,SystemClock.elapsedRealtimeNanos());
        } catch(Exception e) { Log.e(TAG,"frame",e); }
    }

    /** Subsample mode (camera thread): every image (acquireNextImage, none skipped) goes to Subsample, which closes it
     *  right away or holds it until its timestamp decision; kept ones come back in keepFrame. */
    private void receiveSub(ImageReader reader,int index) {
        Subsample sub=subsample;
        while(true) {
            Image image;
            try { image=reader.acquireNextImage(); }
            catch(IllegalStateException e) { Log.w(TAG,"subsample: reader full lens "+index); return; }
            if(image==null) return;
            if(!running) { image.close(); continue; }
            counts[index]++;
            if(counts[index]==1 || counts[index]%600==0)
                Log.i(TAG,"FRAME physical="+ids[index]+" count="+counts[index]+" ts="+image.getTimestamp());
            Metrics mt=metrics;
            if(mt!=null) mt.camFrame(index,image.getTimestamp(),0,fps);
            try { sub.image(index,image.getTimestamp(),image); }
            catch(Exception e) { Log.e(TAG,"subsample",e); try { image.close(); } catch(Exception ignored) {} }
        }
    }

    /** Subsample.Sink (camera thread): copy the kept image and send it on like a normal frame. The image is closed by
     *  Subsample after this returns. */
    private void keepFrame(int index,long physicalTs,Image image,long pairDtNs) {
        if(!running || (client==null && recorder==null)) return;
        long c0=System.nanoTime();
        byte[] nv21=toNv21(image);
        Metrics mt=metrics; if(mt!=null) mt.kept(index,System.nanoTime()-c0);
        enqueueFrame(index,physicalTs,nv21,pairDtNs);
    }

    /** YUV_420_888 -> NV21 copy. */
    private byte[] rowTmp;                             // toNv21 row scratch (camera thread only)
    private byte[] toNv21(Image image) {
        byte[] nv21=new byte[width*height*3/2];
        Image.Plane[] planes=image.getPlanes();
        // Plane buffers can have row padding and interleaved UV; never assume contiguous.
        for(int p=0;p<3;p++) {
            ByteBuffer b=planes[p].getBuffer(); int base=b.position();
            int w=p==0?width:width/2, h=p==0?height:height/2;
            int row=planes[p].getRowStride(), pixel=planes[p].getPixelStride();
            if(p==0 && pixel==1) {          // Y rows are contiguous: bulk copy (per-pixel get is slow at large sizes)
                ByteBuffer d=b.duplicate();
                for(int y=0;y<h;y++) { d.position(base+y*row); d.get(nv21,y*width,width); }
                continue;
            }
            // 1.8: one bulk get per row into a scratch array, then pick every pixel-th byte (same output as the old
            // per-pixel ByteBuffer.get loop, which took ~20 ms per 1440x1080 frame on the S24)
            int len=(w-1)*pixel+1;
            if(rowTmp==null || rowTmp.length<len) rowTmp=new byte[len];
            byte[] t=rowTmp;
            ByteBuffer d=b.duplicate();
            for(int y=0;y<h;y++) {
                d.position(base+y*row); d.get(t,0,len);
                if(p==0) { int o=y*width; for(int x=0;x<w;x++) nv21[o+x]=t[x*pixel]; }
                else { int o=width*height+y*width+(p==1?1:0); for(int x=0;x<w;x++) nv21[o+2*x]=t[x*pixel]; }
            }
        }
        return nv21;
    }

    /** Camera handler thread. timestamp = physical SENSOR_TIMESTAMP (logical when raw_ts). The frame goes on with its
     *  mid-exposure time (SPEC 1.2, 1.7-mid). The same read-only NV21 array goes to the stream queue and the recorder. */
    private void enqueueFrame(int index,long timestamp,byte[] data) { enqueueFrame(index,timestamp,data,Long.MIN_VALUE); }

    /** pairDtNs = this lens mid - kept reference mid (subsample mode), Long.MIN_VALUE = none (frames CSV pair_dt_ns). */
    private void enqueueFrame(int index,long timestamp,byte[] data,long pairDtNs) {
        if(!running) return;
        long[] m=frameMeta.get(index).remove(timestamp);
        long mid;
        if(m!=null) { mid=timestamp+m[3]; midSrc.incrementAndGet((int)m[4]); }
        else { mid=timestamp+lastMidOffNs[index]; midNoResult.incrementAndGet(); }   // raw_ts image before its result
        Recorder rec=recorder;
        if(rec!=null && recGate) {
            // mode=record: start the MP4s once the lenses are in phase (SPEC 15.5); frameMeta entries expire by themselves
            long now=SystemClock.elapsedRealtimeNanos();
            if(recGateT0Ns==0) { recGateT0Ns=now; recGateUntilNs=now+recWaitSyncS*1_000_000_000L; }
            PhaseSync ps=phaseSync;
            if(ps==null || ps.locked() || now>=recGateUntilNs) openRecGate(now);
            else rec=null;
        }
        if(rec!=null) {
            if(m==null) rec.frame(index,mid,timestamp,rawTimestamps?timestamp:-1,-1,-1,data,pairDtNs);
            else rec.frame(index,mid,timestamp,m[0],m[1],m[2],data,pairDtNs);
        }
        if(client==null) return;
        if(streamH264) { LiveEncoder le=liveEncoder; if(le!=null) le.frame(index,mid,data); return; }   // shared: done by rec.frame
        ExecutorService[] enc=jpegEncoders;                 // local copy: stop() may null the field
        if(streamJpeg) { if(enc!=null) encodeJpeg(enc[index],index,mid,data); return; }
        offerFrame(new Packet(index,mid,data));
    }

    private void openRecGate(long now) {
        recGate=false;
        JSONObject meta=recMeta;
        if(meta==null) return;
        try {
            synchronized(meta) {
                meta.put("sync_gate_wait_ms",(now-recGateT0Ns)/1_000_000L);
                PhaseSync ps=phaseSync;
                meta.put("sync_at_record_start",ps==null?JSONObject.NULL:ps.json());
            }
        } catch(JSONException e) { Log.w(TAG,"meta sync",e); }
        Log.i(TAG,"recording frames start after "+(now-recGateT0Ns)/1_000_000L+" ms (phase sync)");
        syncChanged();                                     // drop " rec_wait=1" from the status line
    }

    private void offerFrame(Packet packet) {
        BlockingQueue<Packet> q=pending;
        int evicted=-1;
        synchronized(pendingLock) {
            if(packet.kind==KIND_H264) packet.epoch=lossEpoch[packet.index];
            if(q.offer(packet)) return;
            // A lost H.264 frame breaks every later P-frame of that lens: set waitKey INSIDE the lock, before the serve
            // thread can check the next P of that lens (it checks waitKey under the same lock) -> no gap (REVIEW7 H1).
            Packet old=q.poll();
            if(old!=null && old.kind==KIND_H264) { waitKey.set(old.index,1); lossEpoch[old.index]++; evicted=old.index; }
            if(packet.kind==KIND_H264) packet.epoch=lossEpoch[packet.index];   // after a same-lens eviction: new epoch
            q.offer(packet);
        }
        Metrics mt=metrics; if(mt!=null) mt.queueEvicted.incrementAndGet();
        if(evicted>=0) requestKeyframe(evicted);
    }

    /** NV21 -> JPEG on the lens encoder thread. data is shared read-only with the recorder (YuvImage does not modify it). */
    private void encodeJpeg(ExecutorService encoder,final int index,final long timestamp,final byte[] data) {
        final java.util.concurrent.atomic.AtomicInteger busy=jpegBusy[index];
        final Metrics mt=metrics;
        if(busy.incrementAndGet()>JPEG_MAX_INFLIGHT) {
            busy.decrementAndGet(); jpegDropped++; if(mt!=null) mt.jpegDrop(index); return;
        }
        try {
            encoder.execute(() -> {
                try {
                    if(!running || client==null) return;
                    long t0=System.nanoTime();
                    android.graphics.YuvImage yuv=new android.graphics.YuvImage(data,ImageFormat.NV21,width,height,null);
                    ByteArrayOutputStream bos=new ByteArrayOutputStream(64*1024);
                    if(!yuv.compressToJpeg(new android.graphics.Rect(0,0,width,height),jpegQuality,bos)) {
                        jpegDropped++; if(mt!=null) mt.jpegDrop(index); return;
                    }
                    byte[] jpg=bos.toByteArray();
                    long t1=System.nanoTime();
                    if(mt!=null) mt.jpeg(index,t1-t0,jpg.length,t1);
                    offerFrame(new Packet(index,timestamp,jpg,KIND_JPEG,0,0));
                } catch(Exception e) { jpegDropped++; if(mt!=null) mt.jpegDrop(index); Log.e(TAG,"jpeg",e); }
                finally { busy.decrementAndGet(); }
            });
        } catch(RejectedExecutionException e) { busy.decrementAndGet(); }
    }

    private void serve() {
        while(running) {
            try (Socket s=server.accept()) {
                s.setTcpNoDelay(true); pending.clear();
                if(streamH264) {                      // new client: start every lens at a fresh keyframe (SPS/PPS first)
                    for(int i=0;i<ids.length;i++) waitKey.set(i,1);
                    requestKeyframe(-1);
                }
                client=s;
                final Socket sock=s;
                Thread cmd=new Thread(() -> {         // PC -> phone one-line commands (old PCs send nothing)
                    try {
                        BufferedReader r=new BufferedReader(new InputStreamReader(sock.getInputStream(),"UTF-8"));
                        String line; while((line=r.readLine())!=null) command(line.trim());
                    } catch(IOException ignored) {}
                },"usb-cmd");
                cmd.setDaemon(true); cmd.start();
                DataOutputStream out=new DataOutputStream(new BufferedOutputStream(s.getOutputStream(),1024*1024));
                imuPending.offer(streamInfo());       // early packet of the connection: ts_ref (SPEC 1.4 type 101)
                while(running) {
                    Packet p=pending.poll(5,TimeUnit.MILLISECONDS);
                    if(p!=null && p.kind==KIND_H264) {
                        boolean skip=false;
                        synchronized(pendingLock) {            // same lock as offerFrame's eviction (REVIEW7 H1)
                            if(waitKey.get(p.index)!=0) {
                                // only a keyframe produced after the last loss of this lens ends the wait (lossEpoch)
                                if((p.flags&H264.FLAG_KEY)!=0 && p.epoch==lossEpoch[p.index]) waitKey.set(p.index,0);
                                else skip=true;
                            }
                        }
                        if(skip) { Metrics mt=metrics; if(mt!=null) mt.h264WaitKey(p.index); p=null; }
                    }
                    if(p!=null && p.kind==KIND_H264) {
                        out.writeInt(MAGIC_H264); out.writeInt(p.index);
                        out.writeInt(width); out.writeInt(height); out.writeLong(p.timestamp);
                        out.writeInt(8+p.data.length); out.writeInt(p.flags); out.writeInt(p.seq); out.write(p.data);
                        Metrics mt=metrics; if(mt!=null) mt.usbBytes.addAndGet(36+p.data.length);
                    } else if(p!=null) {
                        out.writeInt(p.kind==KIND_JPEG?MAGIC_JPEG:MAGIC_FRAME); out.writeInt(p.index);
                        out.writeInt(width); out.writeInt(height); out.writeLong(p.timestamp);
                        out.writeInt(p.data.length); out.write(p.data);
                        Metrics mt=metrics; if(mt!=null) mt.usbBytes.addAndGet(28+p.data.length);
                    }
                    Packet q; int n=0;
                    while(n<1024 && (q=imuPending.poll())!=null) {
                        n++;
                        out.writeInt(MAGIC_SENSOR); out.writeInt(q.index);            // index = android Sensor type
                        out.writeInt(0); out.writeInt(0); out.writeLong(q.timestamp);
                        out.writeInt(q.data.length); out.write(q.data);
                        Metrics mt=metrics; if(mt!=null) mt.usbBytes.addAndGet(28+q.data.length);
                    }
                    if(p!=null || n>0) out.flush();
                }
            } catch(Exception e) { if(running) Log.i(TAG,"USB client: "+e); }
            finally { client=null; }
        }
    }

    // Transient focus changes can pause a visible activity during camera startup.
    // Release only when no longer visible; recreate a clean session when returning.
    @Override public void onStop() {
        super.onStop(); stop();
        status("STOPPED (activity not visible)"+(lastRecName!=null?" rec="+lastRecName:""));
    }
    @Override public void onRestart() { super.onRestart(); recreate(); }
    private void stop() {
        running=false;
        UploadService.liveNow=false;
        PhaseSync ps=phaseSync; if(ps!=null) ps.stop();
        if(handler!=null) handler.post(() -> { if(timestampMatcher!=null) timestampMatcher.clear(); if(subsample!=null) subsample.clear(); });
        if(sensors!=null) { sensors.unregisterListener(imu); sensors=null; }
        if(imuThread!=null) { imuThread.quitSafely(); imuThread=null; }
        if(metricsThread!=null) { metricsThread.quitSafely(); metricsThread=null; }
        if(session!=null) { session.close(); session=null; }
        if(camera!=null) { camera.close(); camera=null; }
        for(ImageReader r:readers) r.close(); readers.clear();
        if(jpegEncoders!=null) { for(ExecutorService e:jpegEncoders) e.shutdownNow(); jpegEncoders=null; }
        LiveEncoder le=liveEncoder; liveEncoder=null;
        if(le!=null) le.stop(500);
        if(jpegDropped>0) Log.i(TAG,"jpeg dropped="+jpegDropped);
        try { if(client!=null) client.close(); if(server!=null) server.close(); } catch(IOException ignored) {}
        if(cameraThread!=null) cameraThread.quitSafely();
        // Camera is closed, so no new frames: drain encoders and finalize MP4s (moov) so files stay playable.
        stopRecording("activity_stop");
    }
    @Override public void onDestroy() {
        stop();
        UploadService.activityAlive=false;
        if(stopReceiver!=null) { try { unregisterReceiver(stopReceiver); } catch(Exception ignored) {} stopReceiver=null; }
        super.onDestroy();
    }
}
