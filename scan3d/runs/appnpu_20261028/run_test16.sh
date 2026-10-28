set -u
cd ~/scanner/tmp_appnpu/pc
N=~/scanner/StereoDepthSeg-appnpu/phone_stereo/arlat/npu
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export PYTHONPATH=/home/system/scanner/tmp_ovxfeat/pylib
WR=/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/L6_a16mc/lg_wr.npy
./xfnpu_pc appnpu_test ${XFM:-pc:xfsoft} pc:lg512 $WR npu_test.jsonl "$PY $N/npu_emu.py" ${TOPK:-512} x512 ${PAD:-2} 2> srv.err &
SP=$!
for i in $(seq 1 120); do grep -q "ready\|failed" srv.err 2>/dev/null && break; sleep 1; done
cat srv.err
$PY $N/test_server.py appnpu_test test_server_k${TOPK:-512}_p${PAD:-2}.json > test.log 2>&1; echo "test rc $?"
kill $SP; wait $SP; tail -3 srv.err
tail -40 test.log
