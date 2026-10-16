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
 * One logical device, explicit physical outputs. NV21 + sensor timestamp over ADB TCP.
 */
public class MainActivity extends Activity {
    private static final String TAG = "PhoneStereo";
    private final List<ImageReader> readers = new ArrayList<>();
    private final BlockingQueue<Packet> pending = new ArrayBlockingQueue<>(6);
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
    private boolean rawTimestamps;      // intent extra raw_ts=true: send logical timestamps (old behaviour)
    private static final int MAGIC_FRAME = 0x53544552, MAGIC_SENSOR = 0x53454E53;   // "STER", "SENS"
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
    /** Per lens: physical (or logical when raw_ts) SENSOR_TIMESTAMP -> {logical ts, exposure ns, rs skew ns}.
     *  Written and read on the camera handler thread only. */
    private List<LinkedHashMap<Long,long[]>> frameMeta;

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
    private final CameraCaptureSession.CaptureCallback timing=new CameraCaptureSession.CaptureCallback() {
        @Override public void onCaptureCompleted(CameraCaptureSession s,CaptureRequest r,TotalCaptureResult result) {
            try {
                if(!running) return;
                Long ts=result.get(CaptureResult.SENSOR_TIMESTAMP); if(ts==null) return;
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
                if(recorder!=null) {                    // before the matcher emits, so enqueueFrame finds it
                    long[] m={ts,exp==null?-1:exp,skew==null?-1:skew};
                    for(int i=0;i<ids.length;i++) {
                        Long key=rawTimestamps?ts:physicalTimestamps[i];
                        if(key!=null) frameMeta.get(i).put(key,m);
                    }
                }
                if(!rawTimestamps) timestampMatcher.result(ts,physicalTimestamps,SystemClock.elapsedRealtimeNanos());
                sb.append(" matched=").append(timestampMatcher.matched).append(" unmatched_dropped=").append(timestampMatcher.dropped);
                if(timingCount++%30==0) Log.i(TAG,sb.toString());
                Recorder rec=recorder;
                if(rec!=null) rec.sensor(100,ts,v.clone());
                if(client!=null) {
                    ByteBuffer b=ByteBuffer.allocate(4*v.length); for(float f:v) b.putFloat(f);
                    Packet pk=new Packet(100,ts,b.array());
                    if(!imuPending.offer(pk)) { imuPending.poll(); imuPending.offer(pk); }
                }
            } catch(Exception e) { Log.e(TAG,"timing",e); }
        }
    };

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
        Packet(int i, long t, byte[] d) { index=i; timestamp=t; data=d; }
    }

    private void status(String s) {
        Log.i(TAG, s);
        runOnUiThread(() -> text.setText("Phone Stereo\n\n" + s + "\n\nUSB → PC\n화면을 켜둔 상태에서 사용합니다."));
        try (FileOutputStream f = openFileOutput("status.txt", MODE_PRIVATE)) {
            f.write(s.getBytes("UTF-8"));
        } catch (IOException e) { Log.e(TAG,"status write",e); }
        // Copy readable without run-as: /sdcard/Android/data/com.camera.dualstream/files/status.txt
        File ext=getExternalFilesDir(null);
        if(ext!=null) try { writeFile(new File(ext,"status.txt"),s); } catch(IOException e) { Log.w(TAG,"ext status",e); }
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
        int kbps=getIntent().getIntExtra("rec_kbps",8000);
        boolean raw=getIntent().getBooleanExtra("rec_raw",false);
        frameMeta=new ArrayList<>();
        for(int i=0;i<ids.length;i++) frameMeta.add(new LinkedHashMap<Long,long[]>(64,0.75f,false) {
            @Override protected boolean removeEldestEntry(Map.Entry<Long,long[]> e) { return size()>64; }
        });
        Recorder rec=new Recorder(dir,ids,width,height,fps,kbps,raw);
        recMeta=new JSONObject();
        recMeta.put("format","scan3d-phone-rec/1");
        recMeta.put("mode",mode); recMeta.put("folder",name);
        recMeta.put("logical",logical); recMeta.put("physical",new JSONArray(Arrays.asList(ids)));
        recMeta.put("width",width); recMeta.put("height",height); recMeta.put("fps",fps);
        recMeta.put("imu_us",imuPeriodUs); recMeta.put("focus_diopters",focus);
        recMeta.put("raw_ts",rawTimestamps); recMeta.put("rec_kbps",kbps); recMeta.put("rec_raw_h264",raw);
        recMeta.put("i_frame_interval_s",1); recMeta.put("codec","video/avc");
        recMeta.put("timestamp_clock","SystemClock.elapsedRealtimeNanos (camera SENSOR_TIMESTAMP and SensorEvent.timestamp)");
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
        manager = (CameraManager)getSystemService(CAMERA_SERVICE);
        cameraThread = new HandlerThread("camera"); cameraThread.start();
        handler = new Handler(cameraThread.getLooper());
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
            if(ids.length<2 || ids.length>3 || new HashSet<>(Arrays.asList(ids)).size()!=ids.length)
                throw new IllegalArgumentException("Need 2 or 3 distinct physical IDs");
            Set<String> available=manager.getCameraCharacteristics(logical).getPhysicalCameraIds();
            for(String id:ids) if(!available.contains(id)) throw new IllegalArgumentException("Not in logical group: "+id);
            width=getIntent().getIntExtra("width",640); height=getIntent().getIntExtra("height",480);
            rawTimestamps=getIntent().getBooleanExtra("raw_ts",false);
            imuPeriodUs=getIntent().getIntExtra("imu_us",2500);
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
            running=true;
            if(recordEnabled) {
                startRecording(logical,physical,getIntent().getIntExtra("fps",15),getIntent().getFloatExtra("focus_diopters",1.0f));
                stopReceiver=new BroadcastReceiver() {
                    @Override public void onReceive(Context c,Intent i) {
                        // Finishing encoders blocks briefly; keep it off the main thread.
                        final PendingResult pr=goAsync();
                        new Thread(() -> {
                            try {
                                stopRecording("broadcast");
                                // Keep rec=<name> so the PC (rec_status, pull guard) still knows which folder just finished.
                                if(!streamEnabled) runOnUiThread(() -> { stop();
                                    status("RECORD_DONE rec="+lastRecName+" reason=broadcast | capture stopped"); });
                            } finally { pr.finish(); }
                        },"rec-stop").start();
                    }
                };
                IntentFilter filter=new IntentFilter(ACTION_STOP);
                if(Build.VERSION.SDK_INT>=33) registerReceiver(stopReceiver,filter,Context.RECEIVER_EXPORTED);
                else registerReceiver(stopReceiver,filter);
            }
            if(streamEnabled) {
                server=new ServerSocket(8765,1,InetAddress.getByName("127.0.0.1"));
                new Thread(this::serve,"usb-writer").start();
            }
            status("OPEN mode="+mode+" logical="+logical+" physical="+physical+" "+width+"x"+height);
            manager.openCamera(logical,new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice d) { camera=d; configure(); }
                @Override public void onDisconnected(CameraDevice d) { status("DISCONNECTED"); d.close(); }
                @Override public void onError(CameraDevice d,int e) { status("CAMERA_ERROR "+e); d.close(); }
            },handler);
            startSensors();
        } catch(Exception e) { status("ERROR "+e); Log.e(TAG,"startup",e); }
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

    private void configure() {
        try {
            List<OutputConfiguration> outputs=new ArrayList<>();
            CaptureRequest.Builder request=camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            request.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
            request.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
            // Fixed manual focus (diopters = 1/m). Autofocus changes the wide lens focal length between frames
            // (focus breathing), which moved rectified dy by 1-2 px between captures. 0 keeps autofocus.
            float focus=getIntent().getFloatExtra("focus_diopters",1.0f);
            if(focus>0f) {
                request.set(CaptureRequest.CONTROL_AF_MODE,CaptureRequest.CONTROL_AF_MODE_OFF);
                request.set(CaptureRequest.LENS_FOCUS_DISTANCE,focus);
            }
            int fps=getIntent().getIntExtra("fps",15);
            Range<Integer>[] ranges=manager.getCameraCharacteristics(camera.getId()).get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
            if(ranges!=null) for(Range<Integer> r:ranges)
                if(r.getLower()==fps && r.getUpper()==fps) { request.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,r); break; }
            for(int i=0;i<ids.length;i++) {
                final int index=i;
                ImageReader reader=ImageReader.newInstance(width,height,ImageFormat.YUV_420_888,3);
                readers.add(reader);
                reader.setOnImageAvailableListener(r -> receive(r,index),handler);
                OutputConfiguration out=new OutputConfiguration(reader.getSurface());
                out.setPhysicalCameraId(ids[i]); outputs.add(out); request.addTarget(reader.getSurface());
            }
            SessionConfiguration config=new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,outputs,
                command -> handler.post(command),new CameraCaptureSession.StateCallback() {
                    @Override public void onConfigured(CameraCaptureSession s) {
                        session=s;
                        try {
                            s.setRepeatingRequest(request.build(),timing,handler);
                            status(runningState()+" logical="+camera.getId()+" physical="+Arrays.toString(ids)+" focus_diopters="+focus+" mode="+mode);
                        } catch(Exception e) { status("REQUEST_FAILED "+e); }
                    }
                    @Override public void onConfigureFailed(CameraCaptureSession s) { status("CONFIGURE_FAILED"); }
                });
            try { Log.i(TAG,"session_supported="+camera.isSessionConfigurationSupported(config)); }
            catch(Exception e) { Log.i(TAG,"session query unavailable: "+e); }
            camera.createCaptureSession(config);
        } catch(Exception e) { status("CONFIGURE_ERROR "+e); Log.e(TAG,"configure",e); }
    }

    private void receive(ImageReader reader,int index) {
        try (Image image=reader.acquireLatestImage()) {
            if(image==null || !running) return;
            counts[index]++;
            if(counts[index]==1 || counts[index]%150==0)
                Log.i(TAG,"FRAME physical="+ids[index]+" count="+counts[index]+" ts="+image.getTimestamp());
            if(client==null && recorder==null) return;
            byte[] nv21=new byte[width*height*3/2];
            Image.Plane[] planes=image.getPlanes();
            // Plane buffers can have row padding and interleaved UV; never assume contiguous.
            for(int p=0;p<3;p++) {
                ByteBuffer b=planes[p].getBuffer(); int base=b.position();
                int w=p==0?width:width/2, h=p==0?height:height/2;
                int row=planes[p].getRowStride(), pixel=planes[p].getPixelStride();
                for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
                    int dst=p==0?y*width+x:width*height+y*width+2*x+(p==1?1:0);
                    nv21[dst]=b.get(base+y*row+x*pixel);
                }
            }
            if(rawTimestamps) enqueueFrame(index,image.getTimestamp(),nv21);
            else timestampMatcher.image(index,image.getTimestamp(),nv21,SystemClock.elapsedRealtimeNanos());
        } catch(Exception e) { Log.e(TAG,"frame",e); }
    }

    /** Camera handler thread. timestamp = physical SENSOR_TIMESTAMP (logical when raw_ts). The same read-only
     *  NV21 array goes to the stream queue and the recorder. */
    private void enqueueFrame(int index,long timestamp,byte[] data) {
        if(!running) return;
        Recorder rec=recorder;
        if(rec!=null) {
            long[] m=frameMeta.get(index).remove(timestamp);
            if(m==null) rec.frame(index,timestamp,rawTimestamps?timestamp:-1,-1,-1,data);
            else rec.frame(index,timestamp,m[0],m[1],m[2],data);
        }
        if(client==null) return;
        Packet packet=new Packet(index,timestamp,data);
        if(!pending.offer(packet)) { pending.poll(); pending.offer(packet); }
    }

    private void serve() {
        while(running) {
            try (Socket s=server.accept()) {
                s.setTcpNoDelay(true); client=s; pending.clear();
                DataOutputStream out=new DataOutputStream(new BufferedOutputStream(s.getOutputStream(),1024*1024));
                while(running) {
                    Packet p=pending.poll(5,TimeUnit.MILLISECONDS);
                    if(p!=null) {
                        out.writeInt(MAGIC_FRAME); out.writeInt(p.index);
                        out.writeInt(width); out.writeInt(height); out.writeLong(p.timestamp);
                        out.writeInt(p.data.length); out.write(p.data);
                    }
                    Packet q; int n=0;
                    while(n<1024 && (q=imuPending.poll())!=null) {
                        n++;
                        out.writeInt(MAGIC_SENSOR); out.writeInt(q.index);            // index = android Sensor type
                        out.writeInt(0); out.writeInt(0); out.writeLong(q.timestamp);
                        out.writeInt(q.data.length); out.write(q.data);
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
        if(handler!=null) handler.post(() -> { if(timestampMatcher!=null) timestampMatcher.clear(); });
        if(sensors!=null) { sensors.unregisterListener(imu); sensors=null; }
        if(imuThread!=null) { imuThread.quitSafely(); imuThread=null; }
        if(session!=null) { session.close(); session=null; }
        if(camera!=null) { camera.close(); camera=null; }
        for(ImageReader r:readers) r.close(); readers.clear();
        try { if(client!=null) client.close(); if(server!=null) server.close(); } catch(IOException ignored) {}
        if(cameraThread!=null) cameraThread.quitSafely();
        // Camera is closed, so no new frames: drain encoders and finalize MP4s (moov) so files stay playable.
        stopRecording("activity_stop");
    }
    @Override public void onDestroy() {
        stop();
        if(stopReceiver!=null) { try { unregisterReceiver(stopReceiver); } catch(Exception ignored) {} stopReceiver=null; }
        super.onDestroy();
    }
}
