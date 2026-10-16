package com.camera.dualstream;

import android.media.Image;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.util.Log;
import org.json.JSONObject;
import java.io.*;
import java.nio.ByteBuffer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

/** On-phone recording: one H.264 MP4 + timestamp CSV per lens, one IMU CSV. Format: scan3d/RECORDING_FORMAT.md.
 *  Each lens has its own encoder thread fed by a small bounded queue; a full queue drops the incoming frame
 *  (counted) so the camera thread and the TCP stream never wait on the encoder. */
final class Recorder {
    private static final String TAG = "PhoneStereoRec";
    private static final int QUEUE = 4;
    final File dir;
    private final int width, height, fps, bitrate;
    /** Codec chosen by Caps.findAvcEncoder (supports this size/fps); null = createEncoderByType (old behaviour). */
    private final String encoderName;
    private final Lens[] lenses;
    private final ImuWriter imu;
    /** mode=both + stream_fmt=h264: the live stream shares this encoder's output (one encoder per lens, not two).
     *  null = recording only. Settings come from H264.configure (B=0, I every 1 s, realtime). */
    private final H264.Sink live;
    private volatile boolean stopping;
    /** First frame timestamp (any lens): common PTS origin so both MP4s share one timebase. */
    private long t0 = Long.MIN_VALUE;

    private static final class Frame {
        final long physical, logical, exposure, skew; final byte[] nv21;
        Frame(long p, long l, long e, long s, byte[] d) { physical=p; logical=l; exposure=e; skew=s; nv21=d; }
    }
    private static final Frame EOS = new Frame(0,0,0,0,null);

    Recorder(File dir, String[] ids, int width, int height, int fps, int kbps, boolean rawH264, String encoderName, H264.Sink live) throws IOException {
        this.live=live;
        this.dir=dir; this.width=width; this.height=height; this.fps=fps; this.bitrate=kbps*1000; this.encoderName=encoderName;
        if(!dir.isDirectory() && !dir.mkdirs()) throw new IOException("mkdir "+dir);
        lenses=new Lens[ids.length];
        for(int i=0;i<ids.length;i++) lenses[i]=new Lens(i,ids[i],rawH264);
        imu=new ImuWriter(new File(dir,"imu.csv"));
        for(Lens l:lenses) l.start();
        imu.start();
    }

    /** Camera thread. Never blocks. */
    void frame(int index, long physical, long logical, long exposure, long skew, byte[] nv21) {
        if(stopping || index<0 || index>=lenses.length) return;
        synchronized(this) { if(t0==Long.MIN_VALUE) t0=physical; }
        Lens l=lenses[index];
        if(!l.queue.offer(new Frame(physical,logical,exposure,skew,nv21))) { l.dropped++; if(live!=null) live.h264Drop(index); }
    }

    /** Sensor thread. Never blocks. */
    void sensor(int type, long t, float[] values) {
        if(stopping) return;
        if(!imu.queue.offer(new ImuWriter.Sample(type,t,values))) imu.dropped++;
    }

    long frames(int i) { return lenses[i].encoded; }
    long drops(int i) { return lenses[i].dropped; }
    /** Frames the encoder had no input buffer for (hardware encoder too slow). */
    long busy(int i) { return lenses[i].busy; }
    long imuSamples() { return imu.written; }
    /** Encoder settings applied (lens 0), null until the encoder is configured. */
    JSONObject encoderInfo() { return lenses.length>0?lenses[0].settings:null; }
    /** Keyframe as soon as possible on lens (or all when lens < 0); used for the shared live stream. */
    void requestKey(int lens) { for(int i=0;i<lenses.length;i++) if(lens<0||lens==i) lenses[i].keyReq=true; }
    long imuDrops() { return imu.dropped; }
    /** Any encoder output frame was a B slice (slice_type check of every output, REVIEW7 H5). No Baseline re-configure:
     *  a recording already being muxed cannot switch profile mid-file, and the PC warns (b_frames_seen) instead. */
    boolean bFramesSeen() { for(Lens l:lenses) if(l.bFrames>0) return true; return false; }

    /** Drain queues, finish encoders (EOS), close muxers and CSVs. Blocks up to ~timeoutMs. Idempotent.
     *  Returns true when every lens has closed (muxer stopped); false = some encoder still finishing (see whenFinished). */
    boolean stop(long timeoutMs) {
        if(stopping) return finished();
        stopping=true;
        for(Lens l:lenses) { while(!l.queue.offer(EOS)) { if(l.queue.poll()!=null) l.dropped++; } }
        while(!imu.queue.offer(ImuWriter.Sample.EOS)) imu.queue.poll();
        long end=System.currentTimeMillis()+timeoutMs;
        try {
            for(Lens l:lenses) l.join(Math.max(1,end-System.currentTimeMillis()));
            imu.join(Math.max(1,end-System.currentTimeMillis()));
        } catch(InterruptedException ignored) { Thread.currentThread().interrupt(); }
        for(Lens l:lenses) if(l.isAlive()) Log.w(TAG,"encoder "+l.id+" still finishing after timeout");
        return finished();
    }

    private int closedLenses;
    private Runnable onFinished;

    /** True once every lens thread has run close() (muxer stopped or failed). */
    synchronized boolean finished() { return closedLenses>=lenses.length; }

    /** Run r (on the last lens thread, or right now) once every lens has closed. Used to rewrite meta.json after a stop timeout. */
    void whenFinished(Runnable r) {
        boolean now;
        synchronized(this) { now=closedLenses>=lenses.length; if(!now) onFinished=r; }
        if(now) r.run();
    }

    private void lensClosed() {
        Runnable r=null;
        synchronized(this) { closedLenses++; if(closedLenses>=lenses.length) { r=onFinished; onFinished=null; } }
        if(r!=null) try { r.run(); } catch(Exception e) { Log.e(TAG,"onFinished",e); }
    }

    JSONObject lensStats() throws Exception {
        JSONObject j=new JSONObject();
        for(Lens l:lenses) {
            JSONObject s=new JSONObject();
            s.put("video","cam"+l.id+".mp4"); s.put("frames_csv","frames_cam"+l.id+".csv");
            s.put("frames",l.encoded); s.put("dropped_queue",l.dropped); s.put("dropped_encoder_busy",l.busy);
            s.put("samples_written",l.samples); s.put("muxer_ok",l.muxerOk); s.put("encoder",l.codecName);
            s.put("b_frames_seen",l.bFrames>0); s.put("b_frames",l.bFrames);
            if(l.settings!=null) s.put("encoder_settings",l.settings);
            if(l.error!=null) s.put("error",l.error);
            j.put(l.id,s);
        }
        return j;
    }

    // ------------------------------------------------------------------------------------------------
    private final class Lens extends Thread {
        final String id;
        final BlockingQueue<Frame> queue=new ArrayBlockingQueue<>(QUEUE);
        volatile long encoded, dropped, busy, samples, bFrames;
        volatile boolean muxerOk;
        volatile String codecName, error;
        volatile JSONObject settings;
        volatile boolean keyReq;
        private byte[] csd;
        private int seq;
        /** pts -> {physical ts, System.nanoTime at queue}: sensor time + encode latency for the shared live stream. */
        private final LinkedHashMap<Long,long[]> ptsMeta=new LinkedHashMap<Long,long[]>(32,0.75f,false) {
            @Override protected boolean removeEldestEntry(Map.Entry<Long,long[]> e) { return size()>32; }
        };
        private final File mp4, csv, raw;
        private MediaCodec codec;
        private MediaMuxer muxer;
        private int track=-1;
        private boolean muxing;
        private long lastPts=-1;
        private BufferedWriter rows;
        private OutputStream rawOut;
        private final MediaCodec.BufferInfo info=new MediaCodec.BufferInfo();

        final int index;
        Lens(int index, String id, boolean rawH264) {
            super("rec-cam"+id);
            this.index=index; this.id=id;
            mp4=new File(dir,"cam"+id+".mp4"); csv=new File(dir,"frames_cam"+id+".csv");
            raw=rawH264?new File(dir,"cam"+id+".h264"):null;
        }

        @Override public void run() {
            try {
                rows=new BufferedWriter(new FileWriter(csv),1<<16);
                rows.write("frame_idx,physical_ts_ns,logical_ts_ns,exposure_ns,rolling_shutter_skew_ns,pts_us\n");
                if(raw!=null) rawOut=new BufferedOutputStream(new FileOutputStream(raw),1<<20);
                codec=encoderName!=null?MediaCodec.createByCodecName(encoderName)
                        :MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
                codecName=codec.getName();
                // I+P only (B=0), keyframe 1 s, realtime priority; latency keys only when the live stream shares it.
                settings=H264.configure(codec,width,height,fps,bitrate/1000,live!=null);
                codec.start();
                muxer=new MediaMuxer(mp4.getAbsolutePath(),MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
                long lastFlush=System.nanoTime();
                while(true) {
                    Frame fr=queue.poll(10,TimeUnit.MILLISECONDS);
                    if(fr==EOS) { endOfStream(); break; }
                    if(keyReq) { keyReq=false; try { H264.requestSync(codec); } catch(Exception e) { Log.w(TAG,"sync",e); } }
                    if(fr!=null) feed(fr);
                    drain(0);
                    if(System.nanoTime()-lastFlush>1_000_000_000L) { rows.flush(); if(rawOut!=null) rawOut.flush(); lastFlush=System.nanoTime(); }
                }
            } catch(Throwable e) {
                error=e.toString(); Log.e(TAG,"lens "+id,e);
                if(live!=null) try { live.h264Error(index,id+": "+error); } catch(Throwable ignored) {}
            } finally {
                close();
            }
        }

        private void feed(Frame fr) throws IOException {
            // Free output buffers first: a codec whose output side is full will not release input buffers.
            drain(0);
            int in=-1;
            for(int tries=0;tries<3 && in<0;tries++) {
                in=codec.dequeueInputBuffer(10_000);
                if(in<0) drain(0);
            }
            if(in<0) { busy++; if(live!=null) live.h264Drop(index); return; }
            long t0v; synchronized(Recorder.this) { t0v=t0; }
            long pts=(fr.physical-t0v)/1000;
            if(pts<=lastPts) pts=lastPts+1;   // MP4 needs strictly increasing PTS per track
            lastPts=pts;
            Image img=codec.getInputImage(in);
            int size;
            if(img!=null) { H264.fillImage(img,fr.nv21,width,height); size=width*height*3/2; }
            else size=fillBuffer(codec.getInputBuffer(in),fr.nv21);
            if(live!=null) ptsMeta.put(pts,new long[]{fr.physical,System.nanoTime()});
            codec.queueInputBuffer(in,0,size,pts,0);
            rows.write(encoded+","+fr.physical+","+fr.logical+","+fr.exposure+","+fr.skew+","+pts+"\n");
            encoded++;
        }

        private int inStride=-1, inSlice, inColor;

        /** Fallback when getInputImage is null: layout from codec.getInputFormat() (stride, slice-height, color-format).
         *  Supports YUV420SemiPlanar (NV12) and YUV420Planar (I420); anything else throws (recorded as lens error). */
        private int fillBuffer(ByteBuffer b, byte[] nv21) throws IOException {
            if(b==null) throw new IOException("no input buffer");
            if(inStride<0) {
                MediaFormat in=codec.getInputFormat();
                inStride=in.containsKey(MediaFormat.KEY_STRIDE)?in.getInteger(MediaFormat.KEY_STRIDE):width;
                inSlice=in.containsKey(MediaFormat.KEY_SLICE_HEIGHT)?in.getInteger(MediaFormat.KEY_SLICE_HEIGHT):height;
                inColor=in.containsKey(MediaFormat.KEY_COLOR_FORMAT)?in.getInteger(MediaFormat.KEY_COLOR_FORMAT)
                        :MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar;
                if(inStride<width) inStride=width;
                if(inSlice<height) inSlice=height;
                Log.i(TAG,"lens "+id+" buffer input: stride="+inStride+" slice="+inSlice+" color="+inColor);
            }
            boolean flex=inColor==MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible;
            if(flex && error==null) Log.w(TAG,"lens "+id+" flexible input without Image: assuming NV12 (unverified)");
            boolean semi=flex || inColor==MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar;
            boolean planar=inColor==MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Planar;
            if(!semi && !planar) throw new IOException("unsupported encoder input color-format "+inColor+" (no getInputImage)");
            int cw=width/2, ch=height/2, ys=width*height;
            int uvBase=inStride*inSlice;
            int need=semi? uvBase+inStride*(ch-1)+width : uvBase+(inStride/2)*(inSlice/2)+(inStride/2)*(ch-1)+cw;
            if(need>b.capacity()) throw new IOException("input buffer "+b.capacity()+" < "+need+" (stride "+inStride+" slice "+inSlice+")");
            b.clear();
            for(int r=0;r<height;r++) { b.position(r*inStride); b.put(nv21,r*width,width); }
            if(semi) {                                    // NV12: U,V interleaved
                for(int r=0;r<ch;r++) {
                    int o=uvBase+r*inStride, s=ys+r*width;
                    for(int c=0;c<cw;c++) { b.put(o+2*c,nv21[s+2*c+1]); b.put(o+2*c+1,nv21[s+2*c]); }
                }
            } else {                                      // I420: U plane, then V plane (half stride)
                int cs=inStride/2, vBase=uvBase+cs*(inSlice/2);
                for(int r=0;r<ch;r++) {
                    int s=ys+r*width;
                    for(int c=0;c<cw;c++) { b.put(uvBase+r*cs+c,nv21[s+2*c+1]); b.put(vBase+r*cs+c,nv21[s+2*c]); }
                }
            }
            int size=Math.min(b.capacity(),uvBase*3/2);
            b.position(0); b.limit(b.capacity());
            return Math.max(size,need);
        }

        private void endOfStream() throws IOException {
            for(int tries=0;tries<50;tries++) {
                int in=codec.dequeueInputBuffer(20_000);
                if(in>=0) { codec.queueInputBuffer(in,0,0,Math.max(0,lastPts+1),MediaCodec.BUFFER_FLAG_END_OF_STREAM); break; }
                drain(0);
            }
            long until=System.nanoTime()+1_500_000_000L;
            while(System.nanoTime()<until) if(drain(10_000)) break;
        }

        /** Returns true once the EOS buffer has come out. */
        private boolean drain(long timeoutUs) throws IOException {
            while(true) {
                int o=codec.dequeueOutputBuffer(info,timeoutUs);
                if(o==MediaCodec.INFO_TRY_AGAIN_LATER) return false;
                if(o==MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    if(!muxing) { track=muxer.addTrack(codec.getOutputFormat()); muxer.start(); muxing=true; }
                    continue;
                }
                if(o<0) continue;
                ByteBuffer ob=codec.getOutputBuffer(o);
                boolean config=(info.flags&MediaCodec.BUFFER_FLAG_CODEC_CONFIG)!=0;
                if(ob!=null && info.size>0) {
                    ob.position(info.offset); ob.limit(info.offset+info.size);
                    if(rawOut!=null) {                   // Annex-B elementary stream (SPS/PPS included)
                        byte[] tmp=new byte[info.size]; ob.duplicate().get(tmp); rawOut.write(tmp);
                    }
                    if(!config && H264.isBSlice(ob,info.offset,info.offset+info.size)) bFrames++;   // REVIEW7 H5
                    if(config) csd=H264.copy(ob,info);
                    else if(live!=null) {               // shared live stream: copy before the muxer reads the buffer
                        long[] m=ptsMeta.remove(info.presentationTimeUs);
                        if(m!=null) {
                            boolean key=(info.flags&MediaCodec.BUFFER_FLAG_KEY_FRAME)!=0;
                            byte[] data=H264.frameBytes(ob,info,csd,key);
                            int flags=(key?H264.FLAG_KEY:0)|(key&&H264.hasSps(data)?H264.FLAG_CONFIG:0);
                            try { live.h264(index,m[0],data,flags,seq++,System.nanoTime()-m[1]); } catch(Exception e) { Log.w(TAG,"live sink",e); }
                        } else {
                            seq++;                       // dropped output: PC sees the seq gap, lens waits for a keyframe (REVIEW7 H1)
                            try { live.h264Lost(index); } catch(Exception e) { Log.w(TAG,"live sink",e); }
                        }
                    }
                    if(!config && muxing) { muxer.writeSampleData(track,ob,info); samples++; }
                }
                codec.releaseOutputBuffer(o,false);
                if((info.flags&MediaCodec.BUFFER_FLAG_END_OF_STREAM)!=0) return true;
            }
        }

        private void close() {
            try { if(codec!=null) codec.stop(); } catch(Exception e) { Log.w(TAG,"codec stop",e); }
            try { if(codec!=null) codec.release(); } catch(Exception e) { Log.w(TAG,"codec release",e); }
            try { if(muxer!=null && muxing) { muxer.stop(); muxerOk=true; } } catch(Exception e) { error="muxer stop: "+e; Log.e(TAG,"muxer stop",e); }
            try { if(muxer!=null) muxer.release(); } catch(Exception e) { Log.w(TAG,"muxer release",e); }
            try { if(rows!=null) rows.close(); } catch(IOException ignored) {}
            try { if(rawOut!=null) rawOut.close(); } catch(IOException ignored) {}
            lensClosed();
        }
    }

    // ------------------------------------------------------------------------------------------------
    private static final class ImuWriter extends Thread {
        static final class Sample {
            static final Sample EOS=new Sample(-1,0,null);
            final int type; final long t; final float[] v;
            Sample(int type,long t,float[] v) { this.type=type; this.t=t; this.v=v; }
        }
        // 400 Hz accel+gyro + 5x100 Hz + frame_timing ~ 1.4 k/s -> ~12 s of headroom
        final BlockingQueue<Sample> queue=new ArrayBlockingQueue<>(16384);
        volatile long written, dropped;
        private final File file;
        ImuWriter(File f) { super("rec-imu"); file=f; }
        @Override public void run() {
            try(BufferedWriter w=new BufferedWriter(new FileWriter(file),1<<16)) {
                w.write("type,t_ns,v0,v1,v2,v3,v4,v5\n");
                StringBuilder sb=new StringBuilder(128);
                long lastFlush=System.nanoTime();
                while(true) {
                    Sample s=queue.poll(50,TimeUnit.MILLISECONDS);
                    if(s==Sample.EOS) break;
                    if(s!=null) {
                        sb.setLength(0); sb.append(s.type).append(',').append(s.t);
                        for(float x:s.v) sb.append(',').append(x);
                        sb.append('\n'); w.write(sb.toString()); written++;
                    }
                    if(System.nanoTime()-lastFlush>1_000_000_000L) { w.flush(); lastFlush=System.nanoTime(); }
                }
            } catch(Exception e) { Log.e(TAG,"imu writer",e); }
        }
    }
}
