def patch(p, pairs):
    s=open(p,encoding='utf-8').read()
    for old,new in pairs:
        assert s.count(old)==1,(p,old)
        s=s.replace(old,new)
    open(p,'w',encoding='utf-8',newline='\n').write(s)
patch('components/fc_safety/fc_safety.c',[
("""   * worst impact 2.98 m/s, no cut in the air, <= 0.12 s on the ground above 85 % hover thrust). Legacy: mode 0, 0.4, 0.99 */""",
"""   * C sim test t_openloop_cut 0.3/1/2 m x +-3 %: worst impact 2.19 m/s, no cut in the air, <= 0.18 s on the ground
   * above 85 % hover thrust; Python model 0.2-3 m: worst impact 2.98 m/s). Legacy: mode 0, 0.4, 0.99 */"""),
("""  fs->ol_touch_timing = false;
  fs->ol_fast_ms = fs->ol_slow_ms = 0;
}""","""  fs->ol_touch_timing = false;
  fs->ol_desc_seen = false;
  fs->ol_fast_ms = fs->ol_slow_ms = 0;
}"""),
("""    fs->ol_touch_timing = false;
    fs->ol_fast_ms = fs->ol_slow_ms = 0;
    if (c->openloop_cut_mode == 1) {""","""    fs->ol_touch_timing = false;
    fs->ol_desc_seen = false;
    fs->ol_fast_ms = fs->ol_slow_ms = 0;
    if (c->openloop_cut_mode == 1) {"""),
("""    /* mode 1: VIO still usable (only the IMU frames stopped): on the ground near the takeoff height, not moving */
    if (c->openloop_cut_mode == 1 && c->openloop_vio_touch && vio_usable(fs, in) && in->height_valid &&
        in->height_m < c->openloop_vio_touch_h && fabsf(in->vz_vio) < c->touch_vz_max) {""",
"""    /* mode 1: VIO still usable (only the IMU frames stopped): on the ground near the takeoff height, not moving --
     * only after a descent was seen on VIO in this tier (verifier 2026-10-26: at the tier start the vehicle hovers with
     * vz ~ 0, so a hover just above the takeoff height, or a low VIO height, would otherwise cut in the air) */
    const bool vio_ok = vio_usable(fs, in);
    if (vio_ok && in->vz_vio < -c->openloop_vio_desc_vz) fs->ol_desc_seen = true;
    if (c->openloop_cut_mode == 1 && c->openloop_vio_touch && vio_ok && fs->ol_desc_seen && in->height_valid &&
        in->height_m < c->openloop_vio_touch_h && fabsf(in->vz_vio) < c->touch_vz_max) {"""),
("""  c->openloop_vio_touch_ms = 150;
""","""  c->openloop_vio_touch_ms = 150;
  c->openloop_vio_desc_vz = 0.3f;
"""),
])
patch('components/fc_safety/include/fc_safety.h',[
("""  uint32_t openloop_vio_touch_ms;
""","""  uint32_t openloop_vio_touch_ms;
  float openloop_vio_desc_vz;   /* VIO touchdown armed only after VIO vz < -this was seen in the open-loop tier */
"""),
("""  bool ol_touch_timing;            /* mode 1: VIO touchdown condition timing */
""","""  bool ol_touch_timing;            /* mode 1: VIO touchdown condition timing */
  bool ol_desc_seen;               /* mode 1: VIO descent seen in this tier (arms the VIO touchdown) */
"""),
(""" *                    height < openloop_vio_touch_h and |vz| < touch_vz_max for openloop_vio_touch_ms -> cut.""",
""" *                    after a VIO descent (vz < -openloop_vio_desc_vz) was seen in this tier,
 *                    height < openloop_vio_touch_h and |vz| < touch_vz_max for openloop_vio_touch_ms -> cut."""),
])
print('ok')
