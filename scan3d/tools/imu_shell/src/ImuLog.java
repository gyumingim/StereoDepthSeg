import android.content.Context;
import android.content.ContextWrapper;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.lang.reflect.Constructor;

/** IMU-only logger run as the adb shell user (no app install, no camera):
 *  adb shell CLASSPATH=/data/local/tmp/imulog.dex app_process / ImuLog OUT.csv SECS
 *  Registers accel(1), gyro(4), gyro_uncal(16), accel_uncal(35) at period 0 (fastest).
 *  Optional 3rd arg: comma list of sensor types (then 50 Hz), e.g. 2,14,11 for magnetometer + rotation vector.
 *  CSV: type,t_ns,v0..v5 (same as recordings imu.csv). Flushes every 1 s. Exits after SECS (or when killed:
 *  the sensor connection dies with the process). */
public class ImuLog {
    public static void main(String[] a) throws Exception {
        final String out = a.length > 0 ? a[0] : "/data/local/tmp/imu_static.csv";
        final long secs = a.length > 1 ? Long.parseLong(a[1]) : 1200;
        Looper.prepareMainLooper();
        Context sys;
        Class<?> at = Class.forName("android.app.ActivityThread");
        try {
            Object th = at.getMethod("systemMain").invoke(null);
            sys = (Context) at.getMethod("getSystemContext").invoke(th);
        } catch (Throwable t) {
            Constructor<?> c = at.getDeclaredConstructor(); c.setAccessible(true);
            Object th = c.newInstance();
            java.lang.reflect.Field f = at.getDeclaredField("sCurrentActivityThread"); f.setAccessible(true); f.set(null, th);
            sys = (Context) at.getMethod("getSystemContext").invoke(th);
        }
        Context ctx = new ContextWrapper(sys) {
            @Override public String getOpPackageName() { return "com.android.shell"; }
            @Override public String getPackageName() { return "com.android.shell"; }
            @Override public Context getApplicationContext() { return this; }
        };
        Class<?> ssm = Class.forName("android.hardware.SystemSensorManager");
        final SensorManager sm = (SensorManager) ssm.getConstructor(Context.class, Looper.class).newInstance(ctx, Looper.getMainLooper());
        final BufferedWriter w = new BufferedWriter(new FileWriter(out), 1 << 20);
        w.write("type,t_ns,v0,v1,v2,v3,v4,v5\n");
        HandlerThread ht = new HandlerThread("imu"); ht.start();
        Handler h = new Handler(ht.getLooper());
        final long[] n = new long[64];
        SensorEventListener l = new SensorEventListener() {
            public void onSensorChanged(SensorEvent e) {
                int t = e.sensor.getType();
                StringBuilder sb = new StringBuilder(96);
                sb.append(t).append(',').append(e.timestamp);
                for (float v : e.values) sb.append(',').append(v);
                sb.append('\n');
                synchronized (w) { try { w.write(sb.toString()); } catch (Exception ex) { } }
                if (t < 64) n[t]++;
            }
            public void onAccuracyChanged(Sensor s, int acc) { }
        };
        int[] types = {Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_GYROSCOPE, Sensor.TYPE_GYROSCOPE_UNCALIBRATED, Sensor.TYPE_ACCELEROMETER_UNCALIBRATED};
        int period = 0;
        if (a.length > 2) { String[] q = a[2].split(","); types = new int[q.length]; for (int i = 0; i < q.length; i++) types[i] = Integer.parseInt(q[i]); period = 20000; }
        for (int ty : types) {
            Sensor s = sm.getDefaultSensor(ty);
            boolean ok = s != null && sm.registerListener(l, s, period, h);
            System.out.println("register type " + ty + " " + (s == null ? "none" : s.getName() + " minDelay " + s.getMinDelay() + "us") + " ok=" + ok);
        }
        long t0 = SystemClock.elapsedRealtime();
        System.out.println("START elapsedRealtimeNanos " + SystemClock.elapsedRealtimeNanos() + " wall_ms " + System.currentTimeMillis());
        while (SystemClock.elapsedRealtime() - t0 < secs * 1000) {
            Thread.sleep(1000);
            synchronized (w) { w.flush(); }
            long el = (SystemClock.elapsedRealtime() - t0) / 1000;
            if (el % 10 == 0) System.out.println("t " + el + " s  n1 " + n[1] + " n4 " + n[4] + " n16 " + n[16] + " n35 " + n[35]);
        }
        sm.unregisterListener(l);
        synchronized (w) { w.flush(); w.close(); }
        System.out.println("DONE");
        System.exit(0);
    }
}
