def patch(p, pairs):
    s=open(p,encoding='utf-8').read()
    for old,new in pairs:
        assert s.count(old)==1,(p,old)
        s=s.replace(old,new)
    open(p,'w',encoding='utf-8',newline='\n').write(s)
patch('MainActivity.java',[(
'''                ov.viopHbHz = Math.max(0, Math.min(200, getIntent().getIntExtra("viop_hb_hz", 20)));
''','''                ov.viopHbHz = Math.max(0, Math.min(200, getIntent().getIntExtra("viop_hb_hz", 20)));
                ov.viopHbSrc = getIntent().getIntExtra("viop_hb_src", 0) != 0 ? 1 : 0;  // esp32fix: 1 = steady heartbeat thread
''')])
patch('OvRunner.java',[(
''' * meta.json "profile" / "viop_udp" / "viop_hb_hz" / "viop_imu".
''',''' * meta.json "profile" / "viop_udp" / "viop_hb_hz" / "viop_imu".
 * esp32fix-20261026 (default off = unchanged command line): intent viop_hb_src 1 -> ov_bridge --viop-hb-src 1 (VIOP
 * heartbeat from a steady 20 Hz thread instead of the main loop); meta.json "viop_hb_src".
'''),(
'''    int viopHbHz = 20;                          // 0.6.20 intent viop_hb_hz -> --viop-hb-hz
''','''    int viopHbHz = 20;                          // 0.6.20 intent viop_hb_hz -> --viop-hb-hz
    int viopHbSrc = 0;                          // esp32fix intent viop_hb_src 1 -> --viop-hb-src 1 (0: flag not passed)
'''),(
'''                    "--viop-log", new File(runDir, "viop_log.csv").getAbsolutePath());
''','''                    "--viop-log", new File(runDir, "viop_log.csv").getAbsolutePath());
            if (viopHbSrc != 0) Collections.addAll(cmd, "--viop-hb-src", "1");                                         // esp32fix
'''),(
'''.put("viop_hb_hz", viopHbHz).put("viop_imu", viopImu);   // 0.6.20
''','''.put("viop_hb_hz", viopHbHz).put("viop_imu", viopImu)   // 0.6.20
                .put("viop_hb_src", viopHbSrc);                                                                      // esp32fix
''')])
print('ok')
