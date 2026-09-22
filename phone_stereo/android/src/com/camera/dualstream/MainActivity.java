package com.camera.dualstream;

import android.app.Activity;
import android.os.*;
import android.graphics.ImageFormat;
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
            if(ids.length<2 || ids.length>3 || new HashSet<>(Arrays.asList(ids)).size()!=ids.length)
                throw new IllegalArgumentException("Need 2 or 3 distinct physical IDs");
            Set<String> available=manager.getCameraCharacteristics(logical).getPhysicalCameraIds();
            for(String id:ids) if(!available.contains(id)) throw new IllegalArgumentException("Not in logical group: "+id);
            width=getIntent().getIntExtra("width",640); height=getIntent().getIntExtra("height",480);
            running=true;
            server=new ServerSocket(8765,1,InetAddress.getByName("127.0.0.1"));
            new Thread(this::serve,"usb-writer").start();
            status("OPEN logical="+logical+" physical="+physical+" "+width+"x"+height);
            manager.openCamera(logical,new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice d) { camera=d; configure(); }
                @Override public void onDisconnected(CameraDevice d) { status("DISCONNECTED"); d.close(); }
                @Override public void onError(CameraDevice d,int e) { status("CAMERA_ERROR "+e); d.close(); }
            },handler);
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
                            s.setRepeatingRequest(request.build(),null,handler);
                            status("STREAMING logical="+camera.getId()+" physical="+Arrays.toString(ids));
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
            if(image==null) return;
            counts[index]++;
            if(counts[index]==1 || counts[index]%150==0)
                Log.i(TAG,"FRAME physical="+ids[index]+" count="+counts[index]+" ts="+image.getTimestamp());
            if(client==null) return;
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
            // Bound latency under USB/PC backpressure; discard oldest unsent packet.
            Packet packet=new Packet(index,image.getTimestamp(),nv21);
            if(!pending.offer(packet)) { pending.poll(); pending.offer(packet); }
        } catch(Exception e) { Log.e(TAG,"frame",e); }
    }

    private void serve() {
        while(running) {
            try (Socket s=server.accept()) {
                s.setTcpNoDelay(true); client=s; pending.clear();
                DataOutputStream out=new DataOutputStream(new BufferedOutputStream(s.getOutputStream(),1024*1024));
                while(running) {
                    Packet p=pending.poll(1,TimeUnit.SECONDS); if(p==null) continue;
                    out.writeInt(0x53544552); out.writeInt(p.index);
                    out.writeInt(width); out.writeInt(height); out.writeLong(p.timestamp);
                    out.writeInt(p.data.length); out.write(p.data); out.flush();
                }
            } catch(Exception e) { if(running) Log.i(TAG,"USB client: "+e); }
            finally { client=null; }
        }
    }

    // Transient focus changes can pause a visible activity during camera startup.
    // Release only when no longer visible; recreate a clean session when returning.
    @Override public void onStop() { super.onStop(); stop(); status("STOPPED (activity not visible)"); }
    @Override public void onRestart() { super.onRestart(); recreate(); }
    private void stop() {
        running=false;
        if(session!=null) { session.close(); session=null; }
        if(camera!=null) { camera.close(); camera=null; }
        for(ImageReader r:readers) r.close(); readers.clear();
        try { if(client!=null) client.close(); if(server!=null) server.close(); } catch(IOException ignored) {}
        if(cameraThread!=null) cameraThread.quitSafely();
    }
    @Override public void onDestroy() { stop(); super.onDestroy(); }
}
