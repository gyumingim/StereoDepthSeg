#!/bin/bash
# dry run 30 fps (2026-11-27): decimate 121801 (60 fps) to 30 fps = keep even frames 0,2,4,..; timestamps kept; IMU untouched (symlink).
# Video re-encoded LOSSLESS (libx264 -qp 0, yuv420p) so decoded pixels equal the original decoded frames (checked after).
set -euo pipefail
SRC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_121801_std60
O=/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127
R=$O/rec_hs_20261112_121801_std30
FF=/home/system/scanner/gsmap_env/venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v9.0.2
mkdir -p $R
ln -sf $SRC/imu.csv $R/imu.csv
ln -sf $SRC/temps.csv $R/temps.csv
date +"start %F %T"
nice -n 19 $FF -hide_banner -loglevel error -y -threads 8 -i $SRC/cam5.mp4 -vf "select=not(mod(n\,2)),setpts=N/30/TB" -fps_mode passthrough \
   -c:v libx264 -qp 0 -preset ultrafast -pix_fmt yuv420p -color_range tv -colorspace bt709 $R/cam5.mp4
date +"video done %F %T"
python3 - "$SRC" "$R" <<'EOF'
import csv, json, sys, time
src, r = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(f"{src}/frames_cam5.csv")))
keep = rows[0::2]
with open(f"{r}/frames_cam5.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader()
    for n, row in enumerate(keep):
        row = dict(row); row["frame_idx"] = str(n); w.writerow(row)
phys = {row["physical_ts_ns"] for row in keep}
res = list(csv.DictReader(open(f"{src}/results_cam5.csv")))
rk = [x for x in res if x["sensor_ts_ns"] in phys]
with open(f"{r}/results_cam5.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(res[0].keys())); w.writeheader(); w.writerows(rk)
m = json.load(open(f"{src}/meta.json"))
m.update(fps=30, save_fps=30, frames=len(keep), folder="hs_20261112_121801_std30",
         decimated_from=dict(src=src, rule="every other frame (60 fps frame 2k -> 30 fps frame k), t_mid/physical ts kept, IMU untouched",
                             video="libx264 -qp 0 lossless yuv420p re-encode of the decoded original", made=time.strftime("%F %T"),
                             src_frames=len(rows), results_rows_kept=len(rk)))
json.dump(m, open(f"{r}/meta.json", "w"), indent=1)
print("frames", len(keep), "results rows", len(rk), "of", len(res))
EOF
ls -la $R
date +"all done %F %T"
