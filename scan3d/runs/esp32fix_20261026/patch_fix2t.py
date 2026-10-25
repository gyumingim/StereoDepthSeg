p='test_safety.c'
s=open(p,encoding='utf-8').read()
def rep(old,new):
    global s
    assert s.count(old)==1,old
    s=s.replace(old,new)
rep("""  float tilt_deg;             /* forced roll */""","""  float tilt_deg;             /* forced roll */
  float h_ofs;                /* VIO height error (reported height = z + h_ofs) */""")
rep("""  in->height_m = s->z;""","""  in->height_m = s->z + s->h_ofs;""")
rep("""  c.openloop_vio_touch = false;
  r = ol_case(&c, 1.0f, 1.0f, true);
  CHECK(r.t_ground > 0 && (r.reason & FS_R_LAND_TIMEOUT) && !(r.reason & FS_R_TOUCHDOWN));
""","""  c.openloop_vio_touch = false;
  r = ol_case(&c, 1.0f, 1.0f, true);
  CHECK(r.t_ground > 0 && (r.reason & FS_R_LAND_TIMEOUT) && !(r.reason & FS_R_TOUCHDOWN));

  /* verifier 2026-10-26 regression: hovering 1 m up while VIO says 0.05 m (took off from a higher surface / VIO height
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
""")
open(p,'w',encoding='utf-8',newline='\n').write(s)
print('ok')
