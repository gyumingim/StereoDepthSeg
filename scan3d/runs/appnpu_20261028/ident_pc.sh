PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
R=~/scanner/StereoDepthSeg/scan3d/recordings
C=~/scanner/tmp_lens05/new_gen/c4f60
O=~/scanner/tmp_appnpu/ident
cp ~/scanner/tmp_appnpu/build_ref/ov_bridge $O/ov_bridge_ref; cp ~/scanner/tmp_appnpu/build_pc/ov_bridge $O/ov_bridge_new
sha256sum $O/ov_bridge_ref $O/ov_bridge_new
nice -n 5 $PY /home/system/scanner/StereoDepthSeg-appnpu/scan3d/runs/appnpu_20261028/appnpu_replay.py $R/hs_20261024_165506_std60 $C $O/ov_bridge_ref $O/ref > $O/ref.log 2>&1 &
nice -n 5 $PY /home/system/scanner/StereoDepthSeg-appnpu/scan3d/runs/appnpu_20261028/appnpu_replay.py $R/hs_20261024_165506_std60 $C $O/ov_bridge_new $O/new > $O/new.log 2>&1 &
wait
tail -1 $O/ref.log; tail -1 $O/new.log
