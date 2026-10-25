def patch(p, pairs):
    s=open(p,encoding='utf-8').read()
    for old,new in pairs:
        assert s.count(old)==1,(p,old)
        s=s.replace(old,new)
    open(p,'w',encoding='utf-8',newline='\n').write(s)
patch('components/fc_safety/fc_safety.c',[
("""  /* esp32fix 2026-10-26: 0.4 -> 0.6 m/s and 0.99 -> 0.95 with cut mode 1 (sim sweep h 0.2-3 m x record error +-3 %:
   * C sim test t_openloop_cut 0.3/1/2 m x +-3 %: worst impact 2.19 m/s, no cut in the air, <= 0.18 s on the ground
   * above 85 % hover thrust; Python model 0.2-3 m: worst impact 2.98 m/s). Legacy: mode 0, 0.4, 0.99 */
  c->openloop_vz_down = 0.6f;
  c->openloop_cruise_ratio = 0.95f;""",
"""  /* esp32fix 2026-10-26: defaults = esp32fc (mode 0, 0.4, 0.99); the user decides. Mode 1 is opt-in and was evaluated
   * with openloop_cut_mode = 1, openloop_vz_down = 0.6, openloop_cruise_ratio = 0.95 (test t_openloop_cut, README) */
  c->openloop_vz_down = 0.4f;
  c->openloop_cruise_ratio = 0.99f;"""),
("""  c->openloop_cut_mode = 1;""","""  c->openloop_cut_mode = 0;"""),
("""  c->openloop_vio_desc_vz = 0.3f;
""","""  c->openloop_vio_desc_vz = 0.3f;
  c->openloop_vio_extend_ms = 2000;
"""),
("""    if (elapsed(in->now_ms, fs->t_tier_ms) >= fs->ol_cut_ms) { go(fs, FS_DISARMED, FS_R_LAND_TIMEOUT); return; }""",
"""    /* mode 1: never cut on the timer while VIO is usable and still shows a descent (vz <= -touch_vz_max), for at most
     * openloop_vio_extend_ms past the cut time (descent direction only + cap: a tip-over on the ground cannot keep the
     * motors on). Verifier 2026-10-26: the timer cut still descending vehicles in the air with VIO alive. */
    if (elapsed(in->now_ms, fs->t_tier_ms) >= fs->ol_cut_ms) {
      const bool extend = c->openloop_cut_mode == 1 && vio_ok && in->vz_vio <= -c->touch_vz_max &&
                          elapsed(in->now_ms, fs->t_tier_ms) < fs->ol_cut_ms + c->openloop_vio_extend_ms;
      if (!extend) { go(fs, FS_DISARMED, FS_R_LAND_TIMEOUT); return; }
    }"""),
])
patch('components/fc_safety/include/fc_safety.h',[
("""  float openloop_vio_desc_vz;   /* VIO touchdown armed only after VIO vz < -this was seen in the open-loop tier */
""","""  float openloop_vio_desc_vz;   /* VIO touchdown armed only after VIO vz < -this was seen in the open-loop tier */
  uint32_t openloop_vio_extend_ms; /* mode 1: timer cut deferred while VIO usable and vz <= -touch_vz_max, at most this */
"""),
("""  uint8_t openloop_cut_mode;    /* 1 (default): t_fast/t_slow band + end ramp + VIO touchdown; 0: legacy esp32fc cut */""",
"""  uint8_t openloop_cut_mode;    /* 0 (default, esp32fc legacy cut); 1 = opt-in: t_fast/t_slow band + end ramp + VIO */"""),
(""" *                    openloop_cut_mode 1 (default, esp32fix 2026-10-26)""",""" *                    openloop_cut_mode 1 (opt-in, esp32fix 2026-10-26; default 0 until the user decides)"""),
(""" *                    openloop_cut_mode 0 (legacy, esp32fc)""",""" *                    openloop_cut_mode 0 (default, legacy esp32fc)"""),
])
print('ok')
