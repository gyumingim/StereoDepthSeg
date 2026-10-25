p = 'test_safety.c'
s = open(p, encoding='utf-8').read()


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep("""  float h_ofs;                /* VIO height error (reported height = z + h_ofs) */""",
    """  float h_ofs;                /* VIO height error (reported height = z + h_ofs) */
  float vz_ofs;               /* VIO vertical speed error (reported vz = vz + vz_ofs) */""")
rep("""  in->vz_vio = s->vz;""", """  in->vz_vio = s->vz + s->vz_ofs;""")
rep("""  s.imu_ok = false; /* phone IMU stream lost (USB / phone): no gyro, no accel, and (USB) no pose / heartbeat either */
  s.acc_ok = false;
  s.pose_on = s.hb_on = false; /* esp32fix: whole phone link gone (with poses on, mode 1 would cut on VIO touchdown) */""",
    """  s.imu_ok = false; /* phone IMU stream lost (USB / phone): no gyro, no accel */
  s.acc_ok = false;""")
rep("""  /* mode 1 (default): cut at the slow (+3 % thrust) touchdown time from the last VIO height (~1 m) + 0.2 s */
  CHECK(s.fs.ol_ramp && s.fs.ol_fast_ms > 1000 && s.fs.ol_fast_ms < s.fs.ol_slow_ms);
  CHECK(s.fs.ol_cut_ms == s.fs.ol_slow_ms + 200 && s.fs.ol_cut_ms < 3000);""",
    """  /* cut time from the last VIO height (~1 m): (0.4 + (1 - 0.08) / 0.4) * 1.2 + 0.3 = 3.54 s */
  CHECK(s.fs.ol_cut_ms > 3300 && s.fs.ol_cut_ms < 3800);""")
rep("""  CHECK(ms - t_ground < 1000); /* esp32fix: legacy spun ~1670 ms on the ground at ~98 % hover thrust */
""", "")
rep("""  /* after vz_down / acc_down (0.6 s with the esp32fix defaults): cruise just under hover, before the end ramp (t_fast) */
  run_ms(&s, (uint32_t)(1000.0f * s.fs.cfg.openloop_vz_down / s.fs.cfg.openloop_acc_down) + 100);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.ol_ramp && elapsed_ms_lt(&s, s.fs.ol_fast_ms));
  if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(s.fs.cfg.openloop_cruise_ratio) * vc) < 0.002f);
  /* after the end ramp: openloop_end_ratio */
  while (s.fs.state == FS_EMERG_LAND && !elapsed_ms_ge(&s, s.fs.ol_fast_ms + 400 + 10)) sim_tick(&s);
  if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(s.fs.cfg.openloop_end_ratio) * vc) < 0.002f);""",
    """  run_ms(&s, 500); /* after vz_down / acc_down = 0.4 s: cruise just under hover */
  if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(0.99f) * vc) < 0.002f);""")
rep("""static bool elapsed_ms_lt(const sim_t *s, uint32_t ms) { return s->t - s->fs.t_tier_ms < ms; }
static bool elapsed_ms_ge(const sim_t *s, uint32_t ms) { return s->t - s->fs.t_tier_ms >= ms; }

""", "")
rep("""static ol_res_t ol_case(const fs_config_t *c, float z0, float m, bool keep_pose) {""",
    """static ol_res_t ol_case2(const fs_config_t *c, float z0, float m, bool keep_pose, float h_ofs, float vz_ofs);
static ol_res_t ol_case(const fs_config_t *c, float z0, float m, bool keep_pose) { return ol_case2(c, z0, m, keep_pose, 0, 0); }
static ol_res_t ol_case2(const fs_config_t *c, float z0, float m, bool keep_pose, float h_ofs, float vz_ofs) {""")
rep("""  /* sim thrust = (col / hover_true)^2: hover_true / sqrt(m) scales the thrust of every command by m */
  s.hover_true = 0.42f / sqrtf(m);""",
    """  /* sim thrust = (col / hover_true)^2 and the open-loop collective = mean of the recorded per-motor commands * frac:
   * hover_true = mean(record) / sqrt(m) makes the RECORD exactly m x weight (verifier: the record itself is 1-1.5 % low,
   * so 0.42 / sqrt(m) did not give the nominal error) */
  {
    float mr = 0;
    for (int i = 0; i < 4; i++) mr += 0.25f * s.fs.rec.m[i];
    s.hover_true = mr / sqrtf(m);
  }
  s.h_ofs = h_ofs;
  s.vz_ofs = vz_ofs;""")
rep("""static void t_openloop_cut(void) {""", """static void mode1_cfg(fs_config_t *c) {
  fs_config_default(c);
  c->openloop_cut_mode = 1;
  c->openloop_vz_down = 0.6f;
  c->openloop_cruise_ratio = 0.95f;
}

static void t_openloop_cut(void) {""")
rep("""    fs_config_t c;
    fs_config_default(&c);
    if (legacy) { c.openloop_cut_mode = 0; c.openloop_vz_down = 0.4f; c.openloop_cruise_ratio = 0.99f; }""",
    """    fs_config_t c;
    if (legacy) fs_config_default(&c); /* default = legacy esp32fc (mode 0, 0.4, 0.99) */
    else mode1_cfg(&c);""")
rep("""          CHECK(r.t_ground >= 0 && r.z_cut < 0.01f); /* never cut in the air within +-3 % record error */""",
    """          CHECK(r.t_ground >= 0 && r.z_cut < 0.01f); /* no cut in the air within +-3 % record error (SAME model) */""")
rep("""  g_test = "openloop_vio_touchdown";
  fs_config_t c;
  fs_config_default(&c);""", """  g_test = "openloop_vio_touchdown";
  fs_config_t c;
  mode1_cfg(&c);""")
rep(r"""  /* verifier 2026-10-26 regression: hovering 1 m up while VIO says 0.05 m (took off from a higher surface / VIO height
   * error): at the tier start vz ~ 0 -> the VIO touchdown must NOT fire before a descent was seen (was: cut at 150 ms) */
  g_test = "openloop_vio_touch_needs_descent";
  {
    sim_t s;
    fs_config_default(&c);
    sim_init(&s, &c);
    fly_to_hover(&s, 2000);
    s.h_ofs = -0.95f;
    s.nev = 0;
    s.imu_ok = s.acc_ok = false;
    run_ms(&s, 250);
    CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_OPENLOOP && s.z > 0.9f);
    CHECK(count_ev(&s, FS_EV_TRANSITION, FS_DISARMED) == 0);
  }
}""", r"""  /* verifier 2026-10-26 regression: hovering 1 m up while VIO says 0.05 m (took off from a higher surface / VIO height
   * error): at the tier start vz ~ 0 -> the VIO touchdown must NOT fire before a descent was seen (was: cut at 150 ms),
   * and (second verifier) the timer computed from the wrong height must not cut while VIO still shows the descent:
   * run to the end -> cut on the ground */
  g_test = "openloop_vio_touch_needs_descent";
  mode1_cfg(&c);
  r = ol_case2(&c, 1.0f, 1.0f, true, -0.95f, 0.0f);
  printf("  mode1 + VIO, VIO height 0.95 m low: ground %d ms, cut %d ms (%s), z at cut %.2f, impact %.2f m/s\n", r.t_ground,
         r.t_cut, (r.reason & FS_R_TOUCHDOWN) ? "touch" : "timer", (double)r.z_cut, (double)r.vimp);
  CHECK(r.t_cut > 250 && r.t_ground > 0 && r.z_cut < 0.01f && (r.reason & FS_R_TOUCHDOWN));
  fs_config_default(&c); /* legacy (default) for reference */
  r = ol_case2(&c, 1.0f, 1.0f, true, -0.95f, 0.0f);
  printf("  legacy + VIO, VIO height 0.95 m low: cut %d ms, z at cut %.2f\n", r.t_cut, (double)r.z_cut);

  /* timer deferred while VIO shows descent: VIO alive, record error -8..+8 %, 1 and 2 m -> no cut in the air */
  g_test = "openloop_timer_waits_for_vio_descent";
  {
    static const float mm[] = {0.92f, 0.97f, 1.0f, 1.03f, 1.05f, 1.08f};
    mode1_cfg(&c);
    c.openloop_vio_touch = false; /* timer only (+ deferral), so the deferral itself is what is tested */
    for (int a = 1; a <= 2; a++)
      for (int b = 0; b < 6; b++) {
        r = ol_case2(&c, (float)a, mm[b], true, 0.0f, 0.0f);
        printf("  mode1 timer+VIO deferral h %d m thrust x%.2f: ground %d ms, cut %d ms, z at cut %.2f, impact %.2f m/s\n", a,
               (double)mm[b], r.t_ground, r.t_cut, (double)r.z_cut, (double)r.vimp);
        CHECK(r.t_ground > 0 && r.z_cut < 0.01f);
      }
  }
  /* cap: VIO keeps reporting a descent on the ground (e.g. tipped over, vz bias) -> cut at ol_cut_ms + extend cap */
  g_test = "openloop_vio_deferral_cap";
  {
    sim_t s;
    mode1_cfg(&c);
    c.openloop_vio_touch = false;
    sim_init(&s, &c);
    fly_to_hover(&s, 2000);
    s.vz_ofs = -0.5f;
    s.imu_ok = s.acc_ok = false;
    sim_tick(&s);
    uint32_t cut = s.fs.ol_cut_ms, t0 = s.fs.t_tier_ms;
    int ms = run_until(&s, FS_DISARMED, 10000);
    printf("  mode1 deferral cap: cut %u ms after the tier start (timer %u + cap %u)\n", s.t - t0, cut, c.openloop_vio_extend_ms);
    CHECK(ms > 0 && s.t - t0 >= cut + c.openloop_vio_extend_ms && s.t - t0 <= cut + c.openloop_vio_extend_ms + 2);
  }
  /* mode 1 profile: cruise before t_fast, end ratio after the ramp */
  g_test = "openloop_mode1_profile";
  {
    sim_t s;
    mode1_cfg(&c);
    sim_init(&s, &c);
    fly_to_hover(&s, 2000);
    fs_hover_rec_t hr;
    fs_get_hover_record(&s.fs, &hr);
    s.imu_ok = s.acc_ok = false;
    s.pose_on = s.hb_on = false;
    sim_tick(&s);
    CHECK(s.fs.ol_ramp && s.fs.ol_fast_ms > 1000 && s.fs.ol_fast_ms < s.fs.ol_slow_ms && s.fs.ol_cut_ms == s.fs.ol_slow_ms + 200);
    uint32_t t0 = s.fs.t_tier_ms;
    while (s.fs.state == FS_EMERG_LAND && s.t - t0 < 700) sim_tick(&s);
    CHECK(s.fs.state == FS_EMERG_LAND && fabsf(s.out.motor[0] - hr.m[0] * sqrtf(0.95f)) < 0.002f);
    while (s.fs.state == FS_EMERG_LAND && s.t - t0 < s.fs.ol_fast_ms + 410) sim_tick(&s);
    if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - hr.m[0] * sqrtf(0.70f)) < 0.002f);
  }
}""")
open(p, 'w', encoding='utf-8', newline='\n').write(s)
print('ok')
