package com.scan3d.npuprobe;

import android.app.Instrumentation;
import android.os.Bundle;
import android.util.Log;

/** npu 2026-10-27: headless probe (no UI). adb shell am instrument -w -e args "..." com.scan3d.npuprobe/.Probe */
public class Probe extends Instrumentation {
    static { System.loadLibrary("npuprobe"); }
    static native String run(String args, String filesDir);
    private String args = "";

    @Override
    public void onCreate(Bundle b) {
        super.onCreate(b);
        if (b != null && b.getString("args") != null) args = b.getString("args");
        start();
    }

    @Override
    public void onStart() {
        String dir = getTargetContext().getFilesDir().getAbsolutePath();
        String r = run(args, dir);
        Log.i("npuprobe", "DONE");
        Bundle out = new Bundle();
        out.putString("stream", r);
        finish(0, out);
    }
}
