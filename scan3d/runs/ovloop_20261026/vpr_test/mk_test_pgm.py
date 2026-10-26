import cv2, os, sys
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
cap = cv2.VideoCapture(src); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS)
print("frames", n, "fps", fps, cap.get(3), cap.get(4))
want = [30, 31, n//3, n//2, n-30]
rows = []
for i in want:
    cap.set(cv2.CAP_PROP_POS_FRAMES, i); ok, f = cap.read()
    assert ok, i
    g = cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (960, 540), interpolation=cv2.INTER_AREA)
    t = int(round(i / fps * 1e9)) + 1_000_000_000
    cv2.imwrite(f"{out}/kf_{t}.pgm", g); rows.append(f"{t},{i}")
open(f"{out}/index.csv", "w").write("t_ns,frame\n" + "\n".join(rows) + "\n")
print(rows)
