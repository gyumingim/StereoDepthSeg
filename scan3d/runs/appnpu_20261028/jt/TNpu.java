package com.scan3d.arlat;
// appnpu: JVM check of NpuLoop.pickCpus / bridgeCfg (android.jar stubs on the class path; no Android call is made)
public class TNpu {
    static int bad = 0;
    static void eq(String what, String got, String want) {
        boolean ok = got.equals(want); if (!ok) bad++;
        System.out.println((ok ? "ok   " : "FAIL ") + what + " -> " + got + (ok ? "" : " (want " + want + ")"));
    }
    public static void main(String[] a) {
        NpuLoop n = new NpuLoop(null);
        eq("auto, bridge not pinned", n.pickCpus(""), "0,1,2,3");
        eq("auto, bridge 7-9", n.pickCpus("7-9"), "0,1,2,3,4,5,6");
        eq("auto, bridge 4,5,7-9", n.pickCpus("4,5,7-9"), "0,1,2,3,6");
        n.cpus = "2-3,8"; eq("explicit 2-3,8", n.pickCpus("7-9"), "2,3,8");
        n.cpus = "none"; eq("none", n.pickCpus(""), "none");
        n.cpus = "x"; try { n.pickCpus(""); bad++; System.out.println("FAIL bad cpus accepted"); } catch (IllegalArgumentException e) { System.out.println("ok   bad cpus rejected"); }
        eq("bridge cfg", n.bridgeCfg(), "xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1,xf_timeout_ms=3000");
        n.cfgExtra = "max_cand=2"; eq("bridge cfg + npu_cfg", n.bridgeCfg(), "xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1,xf_timeout_ms=3000,max_cand=2");
        System.out.println(bad == 0 ? "RESULT PASS" : "RESULT FAIL " + bad);
        System.exit(bad == 0 ? 0 : 1);
    }
}
