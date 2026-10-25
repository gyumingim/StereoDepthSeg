// PC test of ViopImuSender.build (integ-20261025): prints one hex frame per line for fixed + random inputs
// (same inputs as imu_frame_c.c, which builds them with viop_proto.h). Run: imu_tx_test.sh
package com.scan3d.arlat;

public class ImuFrameTest {
    public static void main(String[] a) {
        java.util.Random r = new java.util.Random(12345);
        byte[] f = new byte[36];
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 2000; i++) {
            long tUs = i == 0 ? 0L : (i == 1 ? 0xFFFFFFFFFFFFL : 1_000_000_000L + i * 2003L);
            int flags = i % 3 == 0 ? ViopImuSender.IMUF_UNCAL : 0;
            float[] g = new float[3], ac = new float[3];
            for (int k = 0; k < 3; k++) {
                // includes saturation cases (|g| > 32.767 rad/s, |a| > 163.8 m/s^2) and rounding edges
                g[k] = (float) ((r.nextDouble() - 0.5) * (i % 50 == 0 ? 100 : 10));
                ac[k] = (float) ((r.nextDouble() - 0.5) * (i % 70 == 0 ? 500 : 40));
            }
            if (i == 2) { g[0] = 0.0005f; g[1] = -0.0005f; g[2] = 0.0015f; ac[0] = 0.0025f; ac[1] = -0.0025f; ac[2] = 9.81f; }
            ViopImuSender.build(f, i * 7 & 0xffff, tUs, flags, (i * 3 + 65530) & 0xffff, g, ac);
            sb.setLength(0);
            sb.append(tUs).append(' ').append(flags).append(' ').append(i * 7 & 0xffff).append(' ').append((i * 3 + 65530) & 0xffff);
            for (int k = 0; k < 3; k++) sb.append(' ').append(Float.floatToRawIntBits(g[k]));
            for (int k = 0; k < 3; k++) sb.append(' ').append(Float.floatToRawIntBits(ac[k]));
            sb.append(' ');
            for (byte b : f) sb.append(String.format("%02x", b & 0xff));
            System.out.println(sb);
        }
    }
}
