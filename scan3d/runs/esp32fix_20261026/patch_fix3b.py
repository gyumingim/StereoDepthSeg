p = 'main.cpp'
s = open(p, encoding='utf-8').read()


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep("""std::unique_ptr<mav::Out> g_mav;  // --mav-udp (nullptr = off)
""", """std::unique_ptr<mav::Out> g_mav;  // --mav-udp (nullptr = off)
// esp32fix-20261026: set only with --viop-hb-src 1; the input reader thread touches the link object only through this
// pointer (default: never), and that object is then never destroyed (released at the clean exit, see main) so a detached
// reader can never use a destroyed object.
std::atomic<mav::Out *> g_hb_feed_out{nullptr};
""")
rep("""      if (g_mav) g_mav->note_feed();  // esp32fix-20261026: --viop-hb-src 1 feed liveness (no-op otherwise)
""", """      if (mav::Out *ho = g_hb_feed_out.load(std::memory_order_acquire)) ho->note_feed();  // esp32fix: --viop-hb-src 1 only
""")
rep("""    g_mav->hb_thread_start();  // --viop-hb-src 1 only (beats start once IMU is received and handled)
""", """    g_mav->hb_thread_start();  // --viop-hb-src 1 only (beats start once IMU is received and handled)
    if (g_mav->hb_src) g_hb_feed_out.store(g_mav.get(), std::memory_order_release);
""")
rep("""      g_mav->close();
""", """      g_mav->close();
      if (g_hb_feed_out.load()) (void)g_mav.release();  // --viop-hb-src 1: keep the object alive for a detached reader
""")
open(p, 'w', encoding='utf-8', newline='\n').write(s)
print('ok')
