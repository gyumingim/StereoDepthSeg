#!/bin/bash
# npu: clean LighterGlue -> TFLite incl. int8 (calibration = 40 real XFeat pairs from our recordings).  conv_lg_int8.sh N [name]
N=$1; NM=${2:-lgc_$N}
D=/home/system/scanner/tmp_npu/data
A=()
for i in kpts0 kpts1 desc0 desc1; do A+=(-cind $i $D/lgcal${N}_$i.npy '[[[0.0]]]' '[[[1.0]]]'); done
bash /home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027/conv_tfl.sh $NM -n -kat kpts0 kpts1 desc0 desc1 -oiqt "${A[@]}"
