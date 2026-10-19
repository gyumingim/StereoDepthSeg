package com.scan3d.arlat;

/** ChArUco board marker check (DICT_6X6_250) in libarloop: returns [n, x0,y0..x3,y3 per marker]. */
final class MarkerCheck {
    static { System.loadLibrary("arloop"); }
    static native float[] nDetect(byte[] gray, int w, int h);
    private MarkerCheck() { }
}
