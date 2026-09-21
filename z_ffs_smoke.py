#!/usr/bin/env python3
"""
z_ffs_smoke.py — Fast-FoundationStereo 가 이 PC 에서 도는지 저장소 demo_data 로 확인

  ./venv_ffs/bin/python z_ffs_smoke.py

저장소가 함께 주는 정류된 예제 쌍(demo_data/left.png, right.png, K.txt)으로
lib_ffs.load/infer 를 돌려 시차 통계·속도·VRAM 을 찍는다. 우리 파이프라인 검증이 아니라
"모델 로딩 + forward 가 성공하는가"만 본다.
"""
import sys, time
from pathlib import Path
import numpy as np, cv2
import lib_ffs

HERE = Path(__file__).parent
DEMO = lib_ffs.FFS_DIR / "demo_data"


def main():
    cands = sorted(DEMO.glob("*left*")) + sorted(DEMO.glob("*0.png")) + sorted(DEMO.glob("left*"))
    if not cands:
        sys.exit(f"demo_data 에서 왼쪽 영상을 못 찾음: {list(DEMO.glob('*'))[:10]}")
    lp = cands[0]; rp = Path(str(lp).replace("left", "right").replace("0.png", "1.png"))
    if not rp.exists():
        sys.exit(f"오른쪽 영상 없음: {rp}")
    L = cv2.cvtColor(cv2.imread(str(lp)), cv2.COLOR_BGR2RGB); R = cv2.cvtColor(cv2.imread(str(rp)), cv2.COLOR_BGR2RGB)
    print(f"입력 {lp.name}/{rp.name} {L.shape[1]}x{L.shape[0]}")
    import torch
    t = time.perf_counter(); model = lib_ffs.load(); print(f"모델 로드 {time.perf_counter()-t:.1f}s")
    for i, (sc, it) in enumerate([(1.0, 8), (1.0, 8), (0.5, 4), (0.5, 4)]):
        torch.cuda.synchronize(); t = time.perf_counter()
        disp = lib_ffs.infer(model, L, R, valid_iters=it, scale=sc)
        torch.cuda.synchronize(); dt = (time.perf_counter() - t) * 1000
        v = disp > 0.5
        print(f"  scale {sc} iters {it}: {dt:7.0f} ms{' (첫 호출, 컴파일 포함)' if i == 0 else ''}  "
              f"시차 유효 {v.mean()*100:.1f}%  중앙값 {np.median(disp[v]):.1f}px  최대 {disp.max():.1f}px  "
              f"VRAM 피크 {torch.cuda.max_memory_allocated()/1e6:.0f} MB")
    kt = DEMO / "K.txt"
    if kt.exists():
        lines = kt.read_text().split("\n"); K = np.array(lines[0].split(), float).reshape(3, 3); b = float(lines[1])
        depth = K[0, 0] * b / np.where(disp > 0.5, disp, np.nan)
        print(f"  K.txt: fx {K[0,0]:.1f} baseline {b:.4f}m -> 깊이 중앙값 {np.nanmedian(depth):.3f} m")
    out = HERE / "out_dense"; out.mkdir(exist_ok=True)
    dn = (255 * disp / max(1e-6, np.percentile(disp[disp > 0.5], 99))).clip(0, 255).astype(np.uint8)
    cv2.imwrite(str(out / "smoke_disp_vis.png"), cv2.applyColorMap(dn, cv2.COLORMAP_TURBO))
    print(f"저장: {out/'smoke_disp_vis.png'}")


if __name__ == "__main__":
    main()
