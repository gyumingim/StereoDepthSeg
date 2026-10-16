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
    private final Lens[] lenses;
    private final ImuWriter imu;
    private volatile boolean stopping;
    /** First frame timestamp (any lens): common PTS origin so both MP4s share one timebase. */
    private long t0 = Long.MIN_VALUE;

    private static final class Frame {
        final long physical, logical, exposure, skew; final byte[] nv21;
        Frame(long p, long l, long e, long s, byte[] d) { physical=p; logical=l; exposure=e; skew=s; nv21=d; }
    }
    private static final Frame EOS = new Frame(0,0,0,0,null);

    Recorder(File dir, String[] ids, int width, int height, int fps, int kbps, boolean rawH264) throws IOException {
        this.dir=dir; this.width=width; this.height=height; this.fps=fps; this.bitrate=kbps*1000;
        if(!dir.isDirectory() && !dir.mkdirs()) throw new IOException("mkdir "+dir);
        lenses=new Lens[ids.length];
        for(int i=0;i<ids.length;i++) lenses[i]=new Lens(ids[i],rawH264);
        imu=new ImuWriter(new File(dir,"imu.csv"));
        for(Lens l:lenses) l.start();
        imu.start();
    }

    /** Camera thread. Never blocks. */
    void frame(int index, long physical, long logical, long exposure, long skew, byte[] nv21) {
        if(stopping || index<0 || index>=lenses.length) return;
        synchronized(this) { if(t0==Long.MIN_VALUE) t0=physical; }
        Lens l=lenses[index];
        if(!l.queue.offer(new Frame(physical,logical,exposure,skew,nv21))) l.dropped++;
    }

    /** Sensor thread. Never blocks. */
    void sensor(int type, long t, float[] values) {
        if(stopping) return;
        if(!imu.queue.offer(new ImuWriter.Sample(type,t,values))) imu.dropped++;
    }

    long frames(int i) { return lenses[i].encoded; }
    long drops(int i) { return lenses[i].dropped; }
    long imuSamples() { return imu.written; }
    long imuDrops() { return imu.dropped; }

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
            if(l.error!=null) s.put("error",l.error);
            j.put(l.id,s);
        }
        return j;
    }

    // ------------------------------------------------------------------------------------------------
    private final class Lens extends Thread {
        final String id;
        final BlockingQueue<Frame> queue=new ArrayBlockingQueue<>(QUEUE);
        volatile long encoded, dropped, busy, samples;
        volatile boolean muxerOk;
        volatile String codecName, error;
        private final File mp4, csv, raw;
        private MediaCodec codec;
        private MediaMuxer muxer;
        private int track=-1;
        private boolean muxing;
        private long lastPts=-1;
        private BufferedWriter rows;
        private OutputStream rawOut;
        private final MediaCodec.BufferInfo info=new MediaCodec.BufferInfo();

        Lens(String id, boolean rawH264) {
            super("rec-cam"+id);
            this.id=id;
            mp4=new File(dir,"cam"+id+".mp4"); csv=new File(dir,"frames_cam"+id+".csv");
            raw=rawH264?new File(dir,"cam"+id+".h264"):null;
        }

        @Override public void run() {
            try {
                rows=new BufferedWriter(new FileWriter(csv),1<<16);
                rows.write("frame_idx,physical_ts_ns,logical_ts_ns,exposure_ns,rolling_shutter_skew_ns,pts_us\n");
                if(raw!=null) rawOut=new BufferedOutputStream(new FileOutputStream(raw),1<<20);
                MediaFormat f=MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC,width,height);
                f.setInteger(MediaFormat.KEY_COLOR_FORMAT,MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible);
                f.setInteger(MediaFormat.KEY_BIT_RATE,bitrate);
                f.setInteger(MediaFormat.KEY_FRAME_RATE,fps);
                f.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL,1);
                codec=MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
                codecName=codec.getName();
                codec.configure(f,null,null,MediaCodec.CONFIGURE_FLAG_ENCODE);
                codec.start();
                muxer=new MediaMuxer(mp4.getAbsolutePath(),MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
                long lastFlush=System.nanoTime();
                while(true) {
                    Frame fr=queue.poll(10,TimeUnit.MILLISECONDS);
                    if(fr==EOS) { endOfStream(); break; }
                    if(fr!=null) feed(fr);
                    drain(0);
                    if(System.nanoTime()-lastFlush>1_000_000_000L) { rows.flush(); if(rawOut!=null) rawOut.flush(); lastFlush=System.nanoTime(); }
                }
            } catch(Throwable e) {
                error=e.toString(); Log.e(TAG,"lens "+id,e);
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
            if(in<0) { busy++; return; }
            long t0v; synchronized(Recorder.this) { t0v=t0; }
            long pts=(fr.physical-t0v)/1000;
            if(pts<=lastPts) pts=lastPts+1;   // MP4 needs strictly increasing PTS per track
            lastPts=pts;
            Image img=codec.getInputImage(in);
            int size;
            if(img!=null) { fill(img,fr.nv21); size=width*height*3/2; }
            else size=fillBuffer(codec.getInputBuffer(in),fr.nv21);
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

        /** NV21 (Y, then VU interleaved) -> encoder Image with arbitrary row/pixel strides. */
        private void fill(Image img, byte[] nv21) {
            Image.Plane[] p=img.getPlanes();
            ByteBuffer y=p[0].getBuffer(); int yr=p[0].getRowStride(), yp=p[0].getPixelStride();
            for(int r=0;r<height;r++) {
                if(yp==1) { y.position(r*yr); y.put(nv21,r*width,width); }
                else for(int c=0;c<width;c++) y.put(r*yr+c*yp,nv21[r*width+c]);
            }
            ByteBuffer u=p[1].getBuffer(), v=p[2].getBuffer();
            int ur=p[1].getRowStride(), up=p[1].getPixelStride(), vr=p[2].getRowStride(), vp=p[2].getPixelStride();
            int base=width*height, cw=width/2, ch=height/2;
            for(int r=0;r<ch;r++) for(int c=0;c<cw;c++) {
                int s=base+r*width+2*c;           // nv21[s]=V, nv21[s+1]=U
                v.put(r*vr+c*vp,nv21[s]); u.put(r*ur+c*up,nv21[s+1]);
            }
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
