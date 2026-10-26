import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

// lens05: PC test of OvRunner.checkRemapSize (body pasted from OvRunner.java by run_t.sh, isMain/W/H stubbed)
public class T {
    static boolean main_ = true;
    static int W = 1920, H = 1080;
    static boolean isMain() { return main_; }
    //BODY//
    static void t(String name, String remap, String yaml, boolean expectOk) {
        try { checkRemapSize(new File(remap), new File(yaml)); System.out.println((expectOk ? "PASS " : "FAIL ") + name + ": ok"); }
        catch (Exception e) { System.out.println((expectOk ? "FAIL " : "PASS ") + name + ": " + e.getMessage()); }
    }
    public static void main(String[] a) throws Exception {
        String n = a[0], top = a[1], old = a[2];
        t("new c4f60 remap + c4f60 yaml", n + "/c4f60/remap_1920x1080.bin", n + "/c4f60/kalibr_imucam_chain.yaml", true);
        t("new top 640x360 remap + base60 yaml", n + "/remap_1920x1080.bin", n + "/base60/kalibr_imucam_chain.yaml", true);
        t("new c7 1280x720 remap + c7 yaml", n + "/c7/remap_1920x1080.bin", n + "/c7/kalibr_imucam_chain.yaml", true);
        t("fallback 640x360 remap + c4f60 yaml (old silent case)", n + "/remap_1920x1080.bin", n + "/c4f60/kalibr_imucam_chain.yaml", false);
        t("old c4f60 remap + old yaml", old + "/c4f60/remap_1920x1080.bin", old + "/c4f60/kalibr_imucam_chain.yaml", true);
        W = 1280; H = 720;
        t("c7 1920 source map with 1280x720 camera", n + "/c7/remap_1920x1080.bin", n + "/c7/kalibr_imucam_chain.yaml", false);
        t("c7 remap_1280x720 with 1280x720 camera", n + "/c7/remap_1280x720.bin", n + "/c7/kalibr_imucam_chain.yaml", true);
        W = 640; H = 480; main_ = false;
        t("lens0u v1 remap + top-level lens2 yaml (non-main)", top + "/lens0u_remap.bin", top + "/kalibr_imucam_chain.yaml", true);
        t("missing yaml -> skipped", n + "/remap_1920x1080.bin", n + "/nonexist.yaml", true);
    }
}
