# ovbaro 2026-10-26: app hooks (arlat) for the barometer; every default = unchanged (pressure sensor not registered)
import os
D = '/home/system/scanner/StereoDepthSeg-ovbaro/phone_stereo/arlat/src/com/scan3d/arlat/'


def edit(fn, pairs):
    p = D + fn
    s = open(p, encoding='utf-8').read()
    for a, b in pairs:
        assert s.count(a) == 1, (fn, a, s.count(a))
        s = s.replace(a, b)
    open(p, 'w', encoding='utf-8', newline='\n').write(s)
    print('edited', fn)


edit('OvRunner.java', [
    ('    boolean imuLog = false;                     // 0.6.19 intent imu_log: IMU delivery logs (class doc)\n',
     '    boolean imuLog = false;                     // 0.6.19 intent imu_log: IMU delivery logs (class doc)\n'
     '    // ovbaro (2026-10-26, default off = pressure sensor not registered, bridge command unchanged): intent baro 1 = register\n'
     '    // TYPE_PRESSURE at its fastest rate (S24 LPS22DF: 80 ms), send each sample as bridge record type 7 (<qd t_ns hPa) and run\n'
     '    // ov_bridge --baro 1 --baro-log <run>/baro_log.csv; every sample also in <run>/baro.csv (t_ns,hpa). meta "baro".\n'
     '    boolean baro = false;\n'
     '    private BufferedWriter baroW;\n'
     '    private long nBaro;\n'),
    ('        if (recoverFrames > 0) Collections.addAll(cmd, "--recover", Integer.toString(recoverFrames));                    // ovrobust\n',
     '        if (recoverFrames > 0) Collections.addAll(cmd, "--recover", Integer.toString(recoverFrames));                    // ovrobust\n'
     '        if (baro) Collections.addAll(cmd, "--baro", "1", "--baro-log", new File(runDir, "baro_log.csv").getAbsolutePath());   // ovbaro\n'),
    ('                .put("viop_hb_src", viopHbSrc);                                                                      // esp32fix\n',
     '                .put("viop_hb_src", viopHbSrc)                                                                       // esp32fix\n'
     '                .put("baro", baro ? 1 : 0);                                                                           // ovbaro\n'),
    ('        sm.registerListener(this, gyr, SensorManager.SENSOR_DELAY_FASTEST, ih);\n',
     '        sm.registerListener(this, gyr, SensorManager.SENSOR_DELAY_FASTEST, ih);\n'
     '        if (baro) {                                                   // ovbaro: barometer on the same IMU thread\n'
     '            Sensor ps = sm.getDefaultSensor(Sensor.TYPE_PRESSURE);\n'
     '            if (ps == null) status.show("no barometer");\n'
     '            else {\n'
     '                baroW = new BufferedWriter(new FileWriter(new File(runDir, "baro.csv")), 1 << 14);\n'
     '                baroW.write("t_ns,hpa\\n");\n'
     '                sm.registerListener(this, ps, SensorManager.SENSOR_DELAY_FASTEST, ih);\n'
     '                meta.put("baro_sensor", ps.getName()).put("baro_min_delay_us", ps.getMinDelay());\n'
     '            }\n'
     '        }\n'),
    ('''        if (e.sensor.getType() == accType) {
            if (!accT.isEmpty() && e.timestamp <= accT.get(accT.size() - 1)) return;''',
     '''        if (e.sensor.getType() == Sensor.TYPE_PRESSURE) { onBaro(e); return; }   // ovbaro (registered only with intent baro 1)
        if (e.sensor.getType() == accType) {
            if (!accT.isEmpty() && e.timestamp <= accT.get(accT.size() - 1)) return;'''),
    ('''    private void flushImu() {''',
     '''    /** ovbaro: one pressure sample -> bridge record type 7 (<qd: t_ns, hPa) on the IMU path (same queue / ordering rules). */
    private void onBaro(SensorEvent e) {
        ByteBuffer rec = ByteBuffer.allocate(12 + 16).order(ByteOrder.LITTLE_ENDIAN);
        rec.putInt(MAGIC).putInt(7).putInt(16).putLong(e.timestamp).putDouble(e.values[0]);
        if (imuPrio > 0) { imuQ.offer(rec.array()); wake.release(); }
        else toBridge.offer(rec.array());
        nBaro++;
        BufferedWriter w = baroW;
        if (w != null) try { w.write(e.timestamp + "," + e.values[0] + "\\n"); } catch (Exception ignored) { }
    }

    private void flushImu() {'''),
    ('''        closeQuiet(tempsW);
        if (imuLog && runDir != null) {''',
     '''        closeQuiet(tempsW);
        if (baroW != null) { try { if (imuThread != null) imuThread.join(300); } catch (InterruptedException ignored) { } closeQuiet(baroW); baroW = null; }   // ovbaro
        if (imuLog && runDir != null) {'''),
    ('.put("chunked_images", nChunkImg)', '.put("chunked_images", nChunkImg).put("baro_samples", nBaro)'),
])

edit('HsRecorder.java', [
    ('    int evComp = 0;\n',
     '    int evComp = 0;\n'
     '    boolean baro = false;                        // ovbaro 2026-10-26 intent hs_baro 1: TYPE_PRESSURE rows (type 6, hPa) in imu.csv\n'),
    ('        if (au != null) sm.registerListener(this, au, SensorManager.SENSOR_DELAY_FASTEST, imuH);\n',
     '        if (au != null) sm.registerListener(this, au, SensorManager.SENSOR_DELAY_FASTEST, imuH);\n'
     '        if (baro) {                                // ovbaro (default off = not registered): LPS22DF fastest = 80 ms\n'
     '            Sensor ps = sm.getDefaultSensor(Sensor.TYPE_PRESSURE);\n'
     '            if (ps != null) sm.registerListener(this, ps, SensorManager.SENSOR_DELAY_FASTEST, imuH);\n'
     '        }\n'),
    ('''        String row = (t == Sensor.TYPE_GYROSCOPE_UNCALIBRATED || t == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)''',
     '''        String row = t == Sensor.TYPE_PRESSURE ? t + "," + e.timestamp + "," + e.values[0] + "\\n"   // ovbaro: one value (hPa)
                : (t == Sensor.TYPE_GYROSCOPE_UNCALIBRATED || t == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)'''),
    ('.put("app_version", VERSION).put("temp_rows", nTemps)', '.put("app_version", VERSION).put("hs_baro", baro ? 1 : 0).put("temp_rows", nTemps)'),
])

edit('MainActivity.java', [
    ('                hs.focusDpt = getIntent().getFloatExtra("focus_dpt", 0.5f);\n',
     '                hs.focusDpt = getIntent().getFloatExtra("focus_dpt", 0.5f);\n'
     '                hs.baro = getIntent().getIntExtra("hs_baro", 0) != 0;                  // ovbaro: pressure rows in imu.csv\n'),
    ('                ov.recoverFrames = Math.max(0, Math.min(1000, getIntent().getIntExtra("recover", 0)));   // ovrobust: --recover N\n',
     '                ov.recoverFrames = Math.max(0, Math.min(1000, getIntent().getIntExtra("recover", 0)));   // ovrobust: --recover N\n'
     '                ov.baro = getIntent().getIntExtra("baro", 0) != 0;                      // ovbaro: barometer -> ov_bridge --baro 1\n'),
])
