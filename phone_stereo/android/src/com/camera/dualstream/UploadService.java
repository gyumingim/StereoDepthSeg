package com.camera.dualstream;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ServiceInfo;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;
import org.json.JSONObject;
import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.security.MessageDigest;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/** 1.9-upload: finished recordings -> PC over HTTPS (scan3d/upload_server.py behind a Cloudflare tunnel).
 *  1.10-liveupload: WHILE recording, the append-only files (camN.h264 + camN.h264.csv index, frames_camN.csv, imu.csv)
 *  go up as they grow; after [stop] only the tails + meta.json/cameras.json are left, so the PC has it within seconds.
 *  The PC rebuilds camN.mp4 from camN.h264 + index (scan3d/h264_mp4.py); the phone keeps its own camN.mp4 (backup,
 *  adb pull) and deletes its .h264 copy once the PC confirmed (sha256). Slower uplink: the backlog continues after stop.
 *  Config ext files/upload.json {url, token, enabled, live, wifi_only, delete_after, chunk_mb, parallel} (PC writes it over adb).
 *  Protocol: GET /api/up/status?rec=  ->  PUT /api/up/chunk?rec=&file=&off=&total=[&live=1] (resume from the PC's size,
 *  409 = use its "have")  ->  POST /api/up/finish?rec= {files:{name:{size,sha256}}}. Done names: ext files/upload_done.txt.
 *  Progress: ext files/upload_state.json (+ metrics.json "upload"). Foreground service (dataSync) so a finished activity
 *  or screen-off does not stop it; the PC no longer force-stops the app after a recording when uploads are on. */
public class UploadService extends Service {
    private static final String TAG = "PhoneStereoUp";
    private static final String CH = "upload";
    private static final int NOTE_ID = 7;
    /** Set by MainActivity: folder being recorded now (only append-only files go up) and whether the camera streams live. */
    static volatile String recordingNow;
    static volatile boolean liveNow;
    static volatile boolean activityAlive;
    static volatile JSONObject state = new JSONObject();
    private static volatile Thread worker;
    private static final Object NAP = new Object();
    private PowerManager.WakeLock wake;
    private WifiManager.WifiLock wifi;

    /** Running sha256 of the bytes already sent per file (dir/name) -> no full re-read at finish. */
    private static final class Dig { final MessageDigest md; final long n; Dig(MessageDigest m,long n) { md=m; this.n=n; } }
    private static final ConcurrentHashMap<String,Dig> digs=new ConcurrentHashMap<>();

    /** upload.json present and enabled. */
    static JSONObject config(Context c) {
        File ext=c.getExternalFilesDir(null);
        if(ext==null) return null;
        try {
            JSONObject j=new JSONObject(readText(new File(ext,"upload.json")));
            if(!j.optBoolean("enabled",true) || j.optString("url").isEmpty() || j.optString("token").isEmpty()) return null;
            return j;
        } catch(Exception e) { return null; }
    }

    /** Upload while recording (upload.json "live", default true): the recorder then also writes camN.h264 + index. */
    static boolean liveWanted(Context c) {
        JSONObject j=config(c);
        return j!=null && j.optBoolean("live",true);
    }

    /** Wake the uploader now (recording started / stopped). */
    static void poke() { synchronized(NAP) { NAP.notifyAll(); } }

    private static void nap(long ms) throws InterruptedException { synchronized(NAP) { NAP.wait(ms); } }

    /** From the activity (foreground: allowed to start a foreground service). */
    static void kick(Context c) {
        if(config(c)==null) return;
        try {
            Intent i=new Intent(c,UploadService.class);
            if(Build.VERSION.SDK_INT>=26) c.startForegroundService(i); else c.startService(i);
        } catch(Exception e) { Log.w(TAG,"start service",e); }
    }

    @Override public IBinder onBind(Intent i) { return null; }

    @Override public int onStartCommand(Intent intent,int flags,int startId) {
        NotificationManager nm=(NotificationManager)getSystemService(NOTIFICATION_SERVICE);
        if(Build.VERSION.SDK_INT>=26 && nm!=null)
            nm.createNotificationChannel(new NotificationChannel(CH,"녹화 업로드",NotificationManager.IMPORTANCE_LOW));
        Notification n=note("녹화 업로드 준비");
        if(Build.VERSION.SDK_INT>=29) startForeground(NOTE_ID,n,ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC);
        else startForeground(NOTE_ID,n);
        synchronized(UploadService.class) {
            if(worker==null || !worker.isAlive()) {
                worker=new Thread(this::loop,"uploader");
                worker.start();
            }
        }
        return START_STICKY;
    }

    private Notification note(String text) {
        Notification.Builder b=Build.VERSION.SDK_INT>=26?new Notification.Builder(this,CH):new Notification.Builder(this);
        return b.setSmallIcon(android.R.drawable.stat_sys_upload).setContentTitle("Phone Stereo").setContentText(text)
                .setOngoing(true).build();
    }

    private void notify(String text) {
        NotificationManager nm=(NotificationManager)getSystemService(NOTIFICATION_SERVICE);
        if(nm!=null) try { nm.notify(NOTE_ID,note(text)); } catch(Exception ignored) {}
    }

    // ---------------------------------------------------------------- main loop
    private void loop() {
        PowerManager pm=(PowerManager)getSystemService(POWER_SERVICE);
        WifiManager wm=(WifiManager)getApplicationContext().getSystemService(WIFI_SERVICE);
        try {
            if(pm!=null) { wake=pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK,"PhoneStereo:upload"); wake.acquire(6*3600_000L); }
            if(wm!=null) { wifi=wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF,"PhoneStereo:upload"); wifi.acquire(); }
            long backoff=5000; int idle=0;
            while(true) {
                JSONObject cfg=config(this);
                if(cfg==null) { setState("off",null,0,0,0,"no upload.json / disabled"); break; }
                String recNow=recordingNow;
                if(recNow!=null && cfg.optBoolean("live",true) && !liveNow && netOk(cfg)) {
                    idle=0;
                    try { liveUpload(cfg,recNow); backoff=5000; }
                    catch(Exception e) {
                        Log.w(TAG,"live upload "+recNow,e);
                        setState("error",recNow,0,0,0,String.valueOf(e));
                        nap(backoff); backoff=Math.min(backoff*2,60_000);
                    }
                    continue;
                }
                List<File> todo=candidates();
                if(todo.isEmpty()) {
                    // keep waiting while the activity lives (a recording may finish soon), else stop
                    setState("idle",null,0,0,0,null);
                    if((!activityAlive && recordingNow==null) || ++idle>720) break;
                    nap(5000); continue;
                }
                idle=0;
                if(liveNow) { setState("wait_live",null,0,0,0,"live stream running"); nap(5000); continue; }
                if(!netOk(cfg)) { setState("wait_net",null,0,0,0,cfg.optBoolean("wifi_only",true)?"no Wi-Fi":"no network"); nap(15000); continue; }
                boolean allOk=true;
                for(File d:todo) {
                    if(config(this)==null || liveNow) { allOk=false; break; }
                    try { uploadRec(cfg,d); }
                    catch(Exception e) {
                        allOk=false;
                        Log.w(TAG,"upload "+d.getName(),e);
                        setState("error",d.getName(),0,0,0,String.valueOf(e));
                        break;
                    }
                }
                if(allOk) backoff=5000;
                else { nap(backoff); backoff=Math.min(backoff*2,300_000); }
            }
        } catch(InterruptedException ignored) {
        } finally {
            try { if(wake!=null && wake.isHeld()) wake.release(); } catch(Exception ignored) {}
            try { if(wifi!=null && wifi.isHeld()) wifi.release(); } catch(Exception ignored) {}
            synchronized(UploadService.class) { worker=null; }
            stopForeground(true);
            stopSelf();
        }
    }

    private boolean netOk(JSONObject cfg) {
        ConnectivityManager cm=(ConnectivityManager)getSystemService(CONNECTIVITY_SERVICE);
        if(cm==null) return true;
        Network n=cm.getActiveNetwork();
        NetworkCapabilities c=n==null?null:cm.getNetworkCapabilities(n);
        if(c==null || !c.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)) return false;
        return !cfg.optBoolean("wifi_only",true) || c.hasCapability(NetworkCapabilities.NET_CAPABILITY_NOT_METERED);
    }

    /** rec_* folders with meta.json complete=true, not being recorded, not in upload_done.txt. Newest first. */
    private List<File> candidates() {
        List<File> out=new ArrayList<>();
        File ext=getExternalFilesDir(null);
        if(ext==null) return out;
        Set<String> done=doneSet();
        File[] ds=ext.listFiles();
        if(ds==null) return out;
        for(File d:ds) {
            String n=d.getName();
            if(!d.isDirectory() || !n.startsWith("rec_") || done.contains(n) || n.equals(recordingNow)) continue;
            try {
                JSONObject m=new JSONObject(readText(new File(d,"meta.json")));
                if(m.optBoolean("complete") && !m.optBoolean("finishing")) out.add(d);
            } catch(Exception ignored) {}
        }
        Collections.sort(out,(a,b) -> b.getName().compareTo(a.getName()));
        return out;
    }

    private Set<String> doneSet() {
        Set<String> s=new HashSet<>();
        File ext=getExternalFilesDir(null);
        try { for(String l:readText(new File(ext,"upload_done.txt")).split("\n")) if(!l.trim().isEmpty()) s.add(l.trim()); }
        catch(IOException ignored) {}
        return s;
    }

    private void markDone(String name) throws IOException {
        File ext=getExternalFilesDir(null);
        try(FileOutputStream o=new FileOutputStream(new File(ext,"upload_done.txt"),true)) { o.write((name+"\n").getBytes("UTF-8")); }
    }

    /** Files that only grow while recording (safe to send before the recording ends). */
    private static boolean appendOnly(String fn) {
        return fn.equals("imu.csv") || (fn.startsWith("frames_cam") && fn.endsWith(".csv"))
                || (fn.startsWith("cam") && (fn.endsWith(".h264") || fn.endsWith(".h264.csv")));
    }

    /** What goes to the PC: every file except camN.mp4 when camN.h264 + camN.h264.csv exist (the PC rebuilds the MP4). */
    private static List<File> filesFor(File dir) {
        List<File> out=new ArrayList<>();
        File[] fs=dir.listFiles();
        if(fs==null) return out;
        for(File f:fs) {
            String n=f.getName();
            if(!f.isFile() || n.startsWith(".")) continue;
            if(n.startsWith("cam") && n.endsWith(".mp4")) {
                String b=n.substring(0,n.length()-4);
                if(new File(dir,b+".h264").isFile() && new File(dir,b+".h264.csv").isFile()) continue;
            }
            out.add(f);
        }
        return out;
    }

    // ---------------------------------------------------------------- while recording
    /** Sends the growing append-only files of the folder being recorded until the recording stops (recordingNow changes),
     *  the live stream starts, the network goes or upload is disabled. Video: whenever 256 KB are new (or 1 s passed),
     *  CSVs every 64 KB / 3 s. At most one request per file in flight; up to parallel+1 files at once. */
    private void liveUpload(JSONObject cfg,String name) throws Exception {
        File ext=getExternalFilesDir(null);
        if(ext==null) { nap(1000); return; }
        final File dir=new File(ext,name);
        JSONObject st=new JSONObject(http(cfg,"GET","/api/up/status?rec="+enc(name),null,0));
        final JSONObject have=st.optJSONObject("have");
        final Map<String,Long> off=new ConcurrentHashMap<>();
        final Map<String,Long> lastPut=new HashMap<>();
        final Set<String> busy=ConcurrentHashMap.newKeySet();
        final AtomicLong sent=new AtomicLong();
        final AtomicInteger fails=new AtomicInteger();
        final AtomicReference<String> err=new AtomicReference<>();
        final int chunk=Math.max(1,Math.min(32,cfg.optInt("chunk_mb",16)))<<20;
        int par=Math.max(1,Math.min(4,cfg.optInt("parallel",2)))+1;
        ExecutorService ex=Executors.newFixedThreadPool(par);
        long t0=System.nanoTime();
        Log.i(TAG,"live upload "+name+" start");
        try {
            while(name.equals(recordingNow) && !liveNow && err.get()==null) {
                if(config(this)==null || !netOk(cfg)) break;
                File[] fs=dir.listFiles();
                long now=System.nanoTime(), tot=0, done=0;
                if(fs!=null) for(final File f:fs) {
                    final String fn=f.getName();
                    if(!f.isFile() || !appendOnly(fn)) continue;
                    long len=f.length();
                    Long o=off.get(fn);
                    if(o==null) {
                        o=have==null?0L:have.optLong(fn,0);
                        if(o>len) o=0L;      // PC has more than this file (old attempt) -> server answers 409 with its size
                        off.put(fn,o); lastPut.put(fn,now);
                    }
                    tot+=len; done+=Math.min(o,len);
                    if(busy.contains(fn) || len<=o) continue;
                    boolean video=fn.endsWith(".h264");
                    long minB=video?(512<<10):(64<<10), maxAge=video?1_000_000_000L:3_000_000_000L;
                    if(len-o<minB && now-lastPut.get(fn)<maxAge) continue;
                    final long from=o, n=Math.min(Math.min(chunk,4<<20),len-o), recTot=tot;   // <= 4 MB: quick hand-over at stop
                    lastPut.put(fn,now);
                    busy.add(fn);
                    ex.submit(() -> {
                        try {
                            long[] res=new long[1];
                            int code=putChunk(cfg,name,f,from,n,from+n,recTot,true,res);
                            if(code==200) { off.put(fn,res[0]); sent.addAndGet(n); fails.set(0); }
                            else if(code==409) off.put(fn,res[0]);
                            else if(fails.incrementAndGet()>8) err.set("live chunk http "+code);
                        } catch(Exception e) { err.set(String.valueOf(e)); }
                        finally { busy.remove(fn); }
                    });
                }
                double s=(System.nanoTime()-t0)/1e9;
                setState("live",name,done,tot,s>0?sent.get()/1048576.0/s:0,null);
                nap(200);
            }
        } finally {
            ex.shutdown();
            ex.awaitTermination(90,TimeUnit.SECONDS);
        }
        Log.i(TAG,String.format(Locale.US,"live upload %s stop: %.1f MB in %.1f s",name,sent.get()/1048576.0,(System.nanoTime()-t0)/1e9));
        if(err.get()!=null) throw new IOException(err.get());
    }

    // ---------------------------------------------------------------- one recording
    private void uploadRec(JSONObject cfg,File dir) throws Exception {
        String name=dir.getName();
        final List<File> files=filesFor(dir);
        long total=0;
        for(File f:files) total+=f.length();
        JSONObject st=new JSONObject(http(cfg,"GET","/api/up/status?rec="+enc(name),null,total));
        if(st.optBoolean("done") || sameSizes(st.optJSONObject("pc"),files)) {
            Log.i(TAG,name+": PC already has it");
            markDone(name);
            afterDone(cfg,dir);
            return;
        }
        JSONObject have=st.optJSONObject("have");
        final AtomicLong sent=new AtomicLong();
        long already=0;
        for(File f:files) already+=Math.min(f.length(),have==null?0:have.optLong(f.getName(),0));
        final long base=already, tot=total, t0=System.nanoTime();
        setState("uploading",name,base,tot,0,null);
        int par=Math.max(1,Math.min(4,cfg.optInt("parallel",2)));
        final int chunk=Math.max(1,Math.min(32,cfg.optInt("chunk_mb",16)))<<20;
        ExecutorService ex=Executors.newFixedThreadPool(par);
        List<Future<?>> fu=new ArrayList<>();
        // biggest remaining first so the two videos go in parallel
        final JSONObject haveF=have;
        Collections.sort(files,(a,b) -> Long.compare(b.length()-(haveF==null?0:haveF.optLong(b.getName(),0)),
                                                       a.length()-(haveF==null?0:haveF.optLong(a.getName(),0))));
        for(final File f:files) fu.add(ex.submit(() -> {
            long off=haveF==null?0:haveF.optLong(f.getName(),0);
            if(off>f.length()) off=0;
            int tries=0;
            while(off<f.length()) {
                long n=Math.min(chunk,f.length()-off);
                long[] res=new long[1];
                int code=putChunk(cfg,name,f,off,n,f.length(),tot,false,res);
                if(code==200) { sent.addAndGet(n); off=res[0]; tries=0; }
                else if(code==409) off=res[0];
                else if(++tries>4) throw new IOException("chunk http "+code);
                else Thread.sleep(2000L*tries);
                double s=(System.nanoTime()-t0)/1e9;
                setState("uploading",name,base+sent.get(),tot,s>0?sent.get()/1048576.0/s:0,null);
            }
            return null;
        }));
        ex.shutdown();
        try { for(Future<?> x:fu) x.get(); }
        catch(ExecutionException e) { ex.shutdownNow(); throw new IOException("upload "+name+": "+e.getCause(),e.getCause()); }
        setState("verify",name,tot,tot,0,null);
        long th=System.nanoTime();
        JSONObject want=new JSONObject();
        for(File f:files) want.put(f.getName(),new JSONObject().put("size",f.length()).put("sha256",sha256(f)));
        long hashMs=(System.nanoTime()-th)/1_000_000;
        String r=http(cfg,"POST","/api/up/finish?rec="+enc(name),new JSONObject().put("files",want).toString().getBytes("UTF-8"),tot);
        JSONObject fin=new JSONObject(r);
        if(!fin.optBoolean("ok")) throw new IOException("finish: "+r);
        double s=(System.nanoTime()-t0)/1e9;
        Log.i(TAG,String.format(Locale.US,"%s uploaded %.1f MB (rest after stop) in %.1f s, sha256 %d ms",name,sent.get()/1048576.0,s,hashMs));
        markDone(name);
        setState("done",name,tot,tot,s>0?sent.get()/1048576.0/s:0,null);
        afterDone(cfg,dir);
    }

    /** PC confirmed: delete_after -> whole folder; else drop the .h264 copies whose MP4 the phone finished (muxer_ok). */
    private void afterDone(JSONObject cfg,File dir) {
        String pre=dir.getAbsolutePath()+"/";
        for(String k:digs.keySet()) if(k.startsWith(pre)) digs.remove(k);
        if(cfg.optBoolean("delete_after",false)) { deleteRec(dir); return; }
        try {
            JSONObject lenses=new JSONObject(readText(new File(dir,"meta.json"))).optJSONObject("lenses");
            File[] fs=dir.listFiles();
            if(lenses==null || fs==null) return;
            for(File f:fs) {
                String n=f.getName();
                if(!n.startsWith("cam") || !n.endsWith(".h264")) continue;
                String id=n.substring(3,n.length()-5);
                JSONObject l=lenses.optJSONObject(id);
                File mp4=new File(dir,"cam"+id+".mp4");
                if(l!=null && l.optBoolean("muxer_ok") && mp4.length()>0) {
                    f.delete(); new File(dir,n+".csv").delete();
                }
            }
        } catch(Exception e) { Log.w(TAG,"after done "+dir.getName(),e); }
    }

    private static boolean sameSizes(JSONObject pc,List<File> files) {
        if(pc==null || files.isEmpty()) return false;
        for(File f:files) if(pc.optLong(f.getName(),-1)!=f.length()) return false;
        return true;
    }

    private void deleteRec(File dir) {
        File[] fs=dir.listFiles();
        if(fs!=null) for(File f:fs) f.delete();
        boolean ok=dir.delete();
        Log.i(TAG,"deleted on phone after PC confirm: "+dir.getName()+" "+ok);
    }

    /** returns HTTP code; res[0] = server size of the file after this call (200/409). fileTotal = file size known now
     *  (while recording = off+n). Keeps the running sha256 of the file when the chunk continues it. */
    private int putChunk(JSONObject cfg,String rec,File f,long off,long n,long fileTotal,long recTotal,boolean live,long[] res) throws Exception {
        String path="/api/up/chunk?rec="+enc(rec)+"&file="+enc(f.getName())+"&off="+off+"&total="+fileTotal+(live?"&live=1":"");
        String key=f.getAbsolutePath();
        Dig d=digs.get(key);
        MessageDigest md=null;
        try {
            if(d==null && off==0) md=MessageDigest.getInstance("SHA-256");
            else if(d!=null && d.n==off) md=(MessageDigest)d.md.clone();
        } catch(Exception e) { md=null; }
        HttpURLConnection c=open(cfg,"PUT",path,recTotal);
        try {
            c.setFixedLengthStreamingMode(n);
            c.setRequestProperty("Content-Type","application/octet-stream");
            try(RandomAccessFile in=new RandomAccessFile(f,"r"); OutputStream o=c.getOutputStream()) {
                in.seek(off);
                byte[] buf=new byte[256*1024];
                long left=n;
                while(left>0) {
                    int k=in.read(buf,0,(int)Math.min(buf.length,left));
                    if(k<0) throw new EOFException(f.getName());
                    o.write(buf,0,k); left-=k;
                    if(md!=null) md.update(buf,0,k);
                }
            }
            int code=c.getResponseCode();
            String body=readAll(code>=400?c.getErrorStream():c.getInputStream());
            try { res[0]=new JSONObject(body).optLong("have",off); } catch(Exception e) { res[0]=off; }
            if(code==200 && md!=null && res[0]==off+n) digs.put(key,new Dig(md,off+n));
            return code;
        } catch(IOException e) {
            Log.w(TAG,"chunk "+f.getName()+"@"+off+": "+e);
            c.disconnect();
            return -1;
        }
        // no disconnect() on success: on Android it closes the socket; with the body read the TLS connection is reused
        // (a new TLS handshake through Cloudflare per small live chunk cost more than the chunk itself).
    }

    private HttpURLConnection open(JSONObject cfg,String method,String path,long recTotal) throws IOException {
        HttpURLConnection c=(HttpURLConnection)new URL(cfg.optString("url")+path).openConnection();
        c.setRequestMethod(method);
        c.setConnectTimeout(20000); c.setReadTimeout(90000);
        c.setRequestProperty("Authorization","Bearer "+cfg.optString("token"));
        c.setRequestProperty("X-Rec-Total",String.valueOf(recTotal));
        c.setDoInput(true);
        if(!method.equals("GET")) c.setDoOutput(true);
        return c;
    }

    private String http(JSONObject cfg,String method,String path,byte[] body,long recTotal) throws IOException {
        HttpURLConnection c=open(cfg,method,path,recTotal);
        try {
            if(body!=null) {
                c.setRequestProperty("Content-Type","application/json");
                c.setFixedLengthStreamingMode(body.length);
                try(OutputStream o=c.getOutputStream()) { o.write(body); }
            }
            int code=c.getResponseCode();
            String r=readAll(code>=400?c.getErrorStream():c.getInputStream());
            if(code==401) throw new IOException("token rejected (401) - PC upload_token changed? run upload_server.py --push-now");
            if(code>=400 && code!=409) throw new IOException(method+" "+path+" -> "+code+" "+r);
            return r;
        } catch(IOException e) { c.disconnect(); throw e; }
    }

    // ---------------------------------------------------------------- state
    private long lastStateMs;
    private synchronized void setState(String what,String rec,long done,long total,double mbps,String err) {
        long now=System.currentTimeMillis();
        if((what.equals("uploading") || what.equals("live")) && now-lastStateMs<1000) return;
        lastStateMs=now;
        try {
            JSONObject j=new JSONObject();
            j.put("state",what); j.put("rec",rec==null?JSONObject.NULL:rec); j.put("done",done); j.put("total",total);
            j.put("mbps",Math.round(mbps*100)/100.0); j.put("error",err==null?JSONObject.NULL:err); j.put("t_wall_ms",now);
            if(what.equals("live")) j.put("lag_mb",Math.round((total-done)/104857.6)/10.0);   // not yet on the PC
            state=j;
            File ext=getExternalFilesDir(null);
            if(ext!=null) {
                File tmp=new File(ext,"upload_state.json.part");
                try(FileOutputStream o=new FileOutputStream(tmp)) { o.write(j.toString(1).getBytes("UTF-8")); }
                tmp.renameTo(new File(ext,"upload_state.json"));
            }
            if(what.equals("uploading") && total>0)
                notify(String.format(Locale.US,"%s 올리는 중 %d%% · %.1f MB/s",rec,done*100/total,mbps));
            else if(what.equals("live"))
                notify(String.format(Locale.US,"녹화하며 올리는 중 · %.1f MB/s · 밀림 %.1f MB",mbps,(total-done)/1048576.0));
            else if(what.equals("done")) notify(rec+" 업로드 끝");
            else if(what.equals("error")) notify("업로드 오류 (다시 시도): "+err);
            else if(what.startsWith("wait")) notify("업로드 대기: "+err);
        } catch(Exception e) { Log.w(TAG,"state",e); }
    }

    // ---------------------------------------------------------------- helpers
    private static String enc(String s) { try { return URLEncoder.encode(s,"UTF-8"); } catch(UnsupportedEncodingException e) { return s; } }

    static String readText(File f) throws IOException {
        try(InputStream i=new FileInputStream(f)) { return readAll(i); }
    }

    private static String readAll(InputStream i) throws IOException {
        if(i==null) return "";
        ByteArrayOutputStream b=new ByteArrayOutputStream();
        byte[] buf=new byte[8192]; int n;
        while((n=i.read(buf))>0) b.write(buf,0,n);
        i.close();
        return b.toString("UTF-8");
    }

    /** sha256 of the whole file: continues the running digest from the upload when there is one (only the tail is read). */
    private static String sha256(File f) throws Exception {
        Dig d=digs.get(f.getAbsolutePath());
        MessageDigest md=null;
        long from=0;
        if(d!=null && d.n<=f.length()) {
            try { md=(MessageDigest)d.md.clone(); from=d.n; } catch(CloneNotSupportedException e) { md=null; }
        }
        if(md==null) { md=MessageDigest.getInstance("SHA-256"); from=0; }
        try(RandomAccessFile in=new RandomAccessFile(f,"r")) {
            in.seek(from);
            byte[] buf=new byte[1<<20]; int n;
            while((n=in.read(buf))>0) md.update(buf,0,n);
        }
        StringBuilder sb=new StringBuilder();
        for(byte x:md.digest()) sb.append(String.format("%02x",x));
        return sb.toString();
    }
}
