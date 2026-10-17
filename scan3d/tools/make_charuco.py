"""인쇄용 ChArUco 보드 (PNG + PDF + 보드 json) 만들기.

  ./venv_ffs/bin/python scan3d/tools/make_charuco.py                    # A4 가로, DICT_4X4_50, 7x5 칸
  ./venv_ffs/bin/python scan3d/tools/make_charuco.py --paper A3         # A3 가로 (칸이 커서 먼 거리 2 m 까지 잘 보임)
  ./venv_ffs/bin/python scan3d/tools/make_charuco.py --dict DICT_5X5_100 --out scan3d/config/charuco

출력 (--out, 기본 scan3d/config/charuco/)
  charuco_<종이>.png   종이 전체 그림 (--dpi, 기본 300)
  charuco_<종이>.pdf   같은 그림, 종이 크기 그대로 (인쇄 대화상자에서 "실제 크기 100 %" 로 인쇄, "용지에 맞춤" 끄기)
  board_<종이>.json    보드 명세. 인쇄 후 자로 잰 보드 전체 가로·세로(mm)를 measured_width_mm / measured_height_mm 에 넣는다.
                       aruco_scale.load_board_spec() 이 이 파일을 읽는다.

보드 아래 여백에 안내 문구와 눈금 (칸 경계마다 짧은 선) 을 같이 찍는다. 행 수는 홀수 (OpenCV 4.6 에서 짝수 행 무늬가 바뀜).

자 없이 칸 크기 재기 (이미 인쇄한 보드, "용지에 맞춤" 으로 크기를 모를 때):
  ./venv_ffs/bin/python scan3d/tools/make_charuco.py --measure 사진.jpg --board scan3d/config/charuco/board_user_A4_6x6.json
  ./venv_ffs/bin/python scan3d/tools/make_charuco.py --measure 사진.jpg --ref card --board ...   # 카드를 종이 옆에
  ./venv_ffs/bin/python scan3d/tools/make_charuco.py --measure a.jpg b.jpg c.jpg --ref both --board ... --write
  사진 = 종이 전체가 어두운 바닥 위에 보이게, 바로 위에서 (40 cm 이상, 그림자·반사 없이). 여러 장이면 중앙값.
  --write 를 주면 board json 의 square_mm 에 결과를 넣고 measured_by 를 남긴다. 확인 그림 <사진>_measure.jpg 저장.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(SCAN3D))

import aruco_scale  # noqa: E402

PAPER_MM = {"A4": (297.0, 210.0), "A3": (420.0, 297.0)}     # 가로 방향
FONTS = ["/mnt/c/Windows/Fonts/malgun.ttf", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
         "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "C:/Windows/Fonts/malgun.ttf"]


def font(px):
    for p in FONTS:
        if Path(p).is_file():
            return ImageFont.truetype(p, px), True
    try:
        return ImageFont.truetype("DejaVuSans.ttf", px), False
    except OSError:
        return ImageFont.load_default(), False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--paper", default="A4", choices=sorted(PAPER_MM))
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--sx", type=int, default=7, help="가로 칸 수")
    ap.add_argument("--sy", type=int, default=5, help="세로 칸 수 (홀수 권장)")
    ap.add_argument("--square-mm", type=float, default=None, help="칸 크기 (기본: 종이에 들어가는 가장 큰 정수 mm)")
    ap.add_argument("--marker-ratio", type=float, default=0.75, help="칸 대비 마커 크기")
    ap.add_argument("--margin-mm", type=float, default=12.0, help="종이 가장자리 여백 (프린터 못 찍는 영역 포함)")
    ap.add_argument("--text-mm", type=float, default=22.0, help="아래 안내 문구 높이")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", default=str(SCAN3D / "config" / "charuco"))
    ap.add_argument("--measure", nargs="+", default=None, metavar="PHOTO", help="자 없이 칸 크기 재기 (사진 경로들)")
    ap.add_argument("--ref", default="a4", choices=["a4", "card", "both"], help="--measure 기준 물체")
    ap.add_argument("--board", default=None, help="--measure 보드 json")
    ap.add_argument("--write", action="store_true", help="--measure 결과를 board json square_mm 에 기록")
    a = ap.parse_args(argv)
    if a.measure:
        return measure(a)
    if a.sy % 2 == 0:
        print("경고: 세로 칸 수가 짝수 — OpenCV 4.5 와 4.7+ 의 무늬가 다름 (aruco_scale 은 legacy 무늬로 맞춤)")

    Wmm, Hmm = PAPER_MM[a.paper]
    avail_w = Wmm - 2 * a.margin_mm
    avail_h = Hmm - 2 * a.margin_mm - a.text_mm
    sq = a.square_mm or float(np.floor(min(avail_w / a.sx, avail_h / a.sy)))
    if sq * a.sx > avail_w + 1e-6 or sq * a.sy > avail_h + 1e-6:
        raise SystemExit(f"칸 {sq} mm 는 {a.paper} 에 안 들어감 (최대 {min(avail_w / a.sx, avail_h / a.sy):.1f} mm)")
    mm = a.dpi / 25.4
    PW, PH = int(round(Wmm * mm)), int(round(Hmm * mm))
    sq_px = int(round(sq * mm))
    bw, bh = sq_px * a.sx, sq_px * a.sy
    spec = aruco_scale.load_board_spec(None, dict=a.dict, squares_x=a.sx, squares_y=a.sy, square_mm_nominal=sq,
                                       marker_ratio=a.marker_ratio)
    board = aruco_scale.Board(spec)
    img = board.image(bw, bh, 0)
    page = Image.new("L", (PW, PH), 255)
    x0 = (PW - bw) // 2
    y0 = int(round(a.margin_mm * mm))
    page.paste(Image.fromarray(img), (x0, y0))
    dr = ImageDraw.Draw(page)
    # 눈금: 보드 위·왼쪽 바깥에 칸 경계마다 짧은 선 (전체 가로 = 0 ~ sx 눈금 사이, 세로 = 0 ~ sy)
    tick = int(round(3 * mm))
    lw = max(1, int(round(0.25 * mm)))
    for i in range(a.sx + 1):
        x = x0 + i * sq_px
        dr.line([(x, y0 - tick - 2), (x, y0 - 2)], fill=0, width=lw)
    for j in range(a.sy + 1):
        y = y0 + j * sq_px
        dr.line([(x0 - tick - 2, y), (x0 - 2, y)], fill=0, width=lw)
    # 안내 문구
    fpx = int(round(3.2 * mm))
    F, kor = font(fpx)
    ty = y0 + bh + int(round(3 * mm))
    W_mm, H_mm = sq * a.sx, sq * a.sy
    if kor:
        lines = [f"ChArUco {a.sx}x{a.sy}  {a.dict}  칸 {sq:g} mm (의도)  보드 {W_mm:g} x {H_mm:g} mm  — 인쇄: 실제 크기 100 %",
                 f"인쇄 후 자로 위쪽 눈금 0~{a.sx} 사이 (보드 전체 가로) 와 왼쪽 눈금 0~{a.sy} 사이 (세로) 를 0.5 mm 까지 재서",
                 f"board_{a.paper}.json 의 measured_width_mm / measured_height_mm 에 넣는다. 평평한 판에 붙여 벽에 (휘면 안 됨)."]
    else:
        lines = [f"ChArUco {a.sx}x{a.sy} {a.dict} square {sq:g} mm (nominal) board {W_mm:g} x {H_mm:g} mm - print at 100 %",
                 f"After printing, measure top ticks 0..{a.sx} (board width) and left ticks 0..{a.sy} (height) to 0.5 mm",
                 f"and enter them as measured_width_mm / measured_height_mm in board_{a.paper}.json. Mount flat."]
    for k, s in enumerate(lines):
        dr.text((x0, ty + k * int(fpx * 1.35)), s, fill=0, font=F)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    png = out / f"charuco_{a.paper}.png"
    pdf = out / f"charuco_{a.paper}.pdf"
    js = out / f"board_{a.paper}.json"
    page.save(png, dpi=(a.dpi, a.dpi))
    page.convert("RGB").save(pdf, "PDF", resolution=float(a.dpi))
    meta = dict(dict=a.dict, squares_x=a.sx, squares_y=a.sy, square_mm_nominal=sq, marker_ratio=a.marker_ratio,
                board_width_mm_nominal=W_mm, board_height_mm_nominal=H_mm, paper=a.paper,
                measured_width_mm=None, measured_height_mm=None, square_mm=None,
                note="인쇄 후 보드 전체 가로·세로를 자로 재서 measured_width_mm / measured_height_mm 에 넣을 것 "
                     "(또는 칸 1개 크기를 square_mm 에 직접). 비워 두면 의도 값을 쓰고 축척 확인은 의미 없음.")
    if js.exists():
        old = json.loads(js.read_text())
        same = all(old.get(k) == meta[k] for k in ("dict", "squares_x", "squares_y", "square_mm_nominal"))
        if same:
            for k in ("measured_width_mm", "measured_height_mm", "square_mm"):
                meta[k] = old.get(k)                   # 이미 잰 값은 지우지 않음
    js.write_text(json.dumps(meta, ensure_ascii=False, indent=1) + "\n")
    print(f"{png}\n{pdf}\n{js}")
    print(f"{a.paper} 가로 {PW}x{PH} px @ {a.dpi} dpi · 칸 {sq:g} mm · 마커 {sq * a.marker_ratio:g} mm · "
          f"모서리 {board.n_corners} 개 · 마커 id 0~{a.sx * a.sy // 2 - 1} · 글꼴 {'한글' if kor else '영문'}")


def measure(a):
    import cv2
    spec = aruco_scale.load_board_spec(a.board)
    board = aruco_scale.Board(spec)
    refs = ["a4", "card"] if a.ref == "both" else [a.ref]
    vals = []
    for ph in a.measure:
        img = cv2.imread(ph)
        if img is None:
            print(f"{ph}: 못 읽음")
            continue
        vis = img.copy()
        for ref in refs:
            r = aruco_scale.measure_square_mm(img, board, ref)
            if not r["ok"]:
                print(f"{ph} [{ref}] 실패: {r.get('why')}")
                continue
            vals.append((ref, r["square_mm"]))
            print(f"{ph} [{ref}] 칸 {r['square_mm']:.3f} mm (가로 {r['sx_mm']:.3f} / 세로 {r['sy_mm']:.3f}, "
                  f"잔차 {r['rms_mm']:.3f} mm, 모서리 {r['n']})" + ("  경고: " + "; ".join(r["warn"]) if r["warn"] else ""))
            q = np.array(r["quad"], np.int32)
            cv2.polylines(vis, [q], True, (0, 0, 255) if ref == "a4" else (255, 0, 0), max(2, img.shape[1] // 600))
        c, _, _ = board.detect(img)
        for x, y in c:
            cv2.circle(vis, (int(x), int(y)), max(3, img.shape[1] // 400), (0, 255, 0), -1)
        out = str(Path(ph).with_suffix("")) + "_measure.jpg"
        cv2.imwrite(out, vis)
        print(f"  확인 그림 {out}")
    if not vals:
        raise SystemExit("측정 실패")
    sq = float(np.median([v for _, v in vals]))
    spread = float(np.ptp([v for _, v in vals])) if len(vals) > 1 else 0.0
    print(f"\n칸 크기 = {sq:.3f} mm (측정 {len(vals)} 개, 최대-최소 {spread:.3f} mm) · 보드 "
          f"{sq * spec['squares_x']:.1f} x {sq * spec['squares_y']:.1f} mm")
    if a.write and a.board:
        j = json.loads(Path(a.board).read_text())
        j["square_mm"] = round(sq, 3)
        j["measured_by"] = dict(method="photo", refs=sorted({r for r, _ in vals}), n=len(vals),
                                spread_mm=round(spread, 3), photos=[str(p) for p in a.measure])
        Path(a.board).write_text(json.dumps(j, ensure_ascii=False, indent=1) + "\n")
        print(f"기록: {a.board} square_mm = {sq:.3f}")


if __name__ == "__main__":
    main()
