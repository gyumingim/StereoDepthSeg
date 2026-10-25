import sys
p='main.cpp'
s=open(p,encoding='utf-8').read()
def rep(old,new,cnt=1):
    global s
    assert s.count(old)==cnt,(old,s.count(old))
    s=s.replace(old,new)
rep("""//   heartbeats on a clean exit) + TIME_RESP replies. --viop-log FILE, --viop-replay-time 0|1 as for MAVLink.
""","""//   heartbeats on a clean exit) + TIME_RESP replies. --viop-log FILE, --viop-replay-time 0|1 as for MAVLink.
// --viop-hb-src 0|1 (esp32fix-20261026; default 0 = as above, unchanged): 1 = heartbeat from a dedicated thread on a
//   steady 1/HZ grid; a beat is skipped while no IMU record was received for 100 ms or the main loop handled no IMU
//   record for 200 ms (feed stopped / bridge hung -> heartbeats stop as before). See mav_out.h hb_run.
""")
rep("""  double viop_hb_hz = 20.0;   // --viop-hb-hz (VIOP heartbeat rate; 0 = none)
""","""  double viop_hb_hz = 20.0;   // --viop-hb-hz (VIOP heartbeat rate; 0 = none)
  int viop_hb_src = 0;        // --viop-hb-src 0|1 (esp32fix-20261026: 1 = steady heartbeat thread; 0 = main loop)
""")
rep(r'[--viop-hb-hz HZ]\n"', r'[--viop-hb-hz HZ] [--viop-hb-src 0|1]\n"')
rep("""    else if (s == "--viop-hb-hz") a.viop_hb_hz = std::stod(next());
""","""    else if (s == "--viop-hb-hz") a.viop_hb_hz = std::stod(next());
    else if (s == "--viop-hb-src") a.viop_hb_src = std::stoi(next());  // esp32fix-20261026
""")
rep("""    if (viop && (a.viop_hb_hz < 0.0 || a.viop_hb_hz > 200.0)) err = "--viop-hb-hz must be in [0, 200]";
""","""    if (viop && (a.viop_hb_hz < 0.0 || a.viop_hb_hz > 200.0)) err = "--viop-hb-hz must be in [0, 200]";
    if (a.viop_hb_src != 0 && a.viop_hb_src != 1) err = "--viop-hb-src must be 0 or 1";
    mo->hb_src = viop ? a.viop_hb_src : 0;  // esp32fix-20261026
""")
rep("""    g_mav = std::move(mo);
  }
""","""    g_mav = std::move(mo);
    g_mav->hb_thread_start();  // --viop-hb-src 1 only (beats start once IMU is received and handled)
    if (g_mav->hb_src) std::fprintf(stderr, "[ov_bridge] viop heartbeat: thread %.1f Hz (feed %d ms / main loop %d ms liveness)\\n",
                                    g_mav->hb_hz, g_mav->hb_feed_ms, g_mav->hb_main_ms);
  }
""")
rep("""        g_mav->hb_tick(sys_ && sys_->initialized(), lost_);  // VIOP heartbeat from the main loop (stops if the feed stops)
""","""        if (g_mav->hb_src) {  // esp32fix-20261026 --viop-hb-src 1: the heartbeat thread sends; the main loop only reports
          g_mav->note_main(sys_ && sys_->initialized(), lost_);
          if (!g_inq) g_mav->note_feed();  // no reader thread: the main loop is where IMU is received
        } else
          g_mav->hb_tick(sys_ && sys_->initialized(), lost_);  // VIOP heartbeat from the main loop (stops if the feed stops)
""")
rep("""      if (pred) pred->push_imu(r.buf.data());
    }
""","""      if (pred) pred->push_imu(r.buf.data());
      if (g_mav) g_mav->note_feed();  // esp32fix-20261026: --viop-hb-src 1 feed liveness (no-op otherwise)
    }
""")
rep("""      for (int i = 0; i < 3; i++) g_mav->hb_tick(false, false, true);  // VIOP: "stopping" heartbeats on a clean exit
""","""      g_mav->hb_thread_stop();  // --viop-hb-src 1 (no-op otherwise)
      for (int i = 0; i < 3; i++) g_mav->hb_tick(false, false, true);  // VIOP: "stopping" heartbeats on a clean exit
""")
open(p,'w',encoding='utf-8',newline='\n').write(s)
p='mav_out.h'
s=open(p,encoding='utf-8').read()
rep("#include <atomic>\n","#include <atomic>\n#include <chrono>\n")
rep("""  bool open(std::string &err) {""","""  ~Out() { hb_thread_stop(); }  // esp32fix: never destroy a joinable heartbeat thread (--viop-hb-src 1, e.g. --tcp exit)

  bool open(std::string &err) {""")
open(p,'w',encoding='utf-8',newline='\n').write(s)
print("ok")
