package com.scan3d.nncprobe;

import android.app.Instrumentation;
import android.os.Bundle;
import android.util.Log;

/** nncrun 2026-10-27: headless .nnc runner (no Activity, no UI, no permissions).
 *  adb shell am instrument -w -e args "model=x.nnc in=in.bin out=out.bin" com.scan3d.nncprobe/.Probe */
public class Probe extends Instrumentation {
    static { System.loadLibrary("nncrun"); }
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
        Log.i("nncrun", "DONE");
        Bundle out = new Bundle();
        out.putString("stream", r);
        finish(0, out);
    }
}
