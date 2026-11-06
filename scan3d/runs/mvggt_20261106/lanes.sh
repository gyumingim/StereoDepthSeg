#!/bin/bash
# mvggt 11-06 (coordinator): two lanes so the GPU keeps working.  usage: lanes.sh cpu | lanes.sh gpu
#  cpu lane (heavy.lock inside run_rec): 222747 BA copy when featloop's rebuilt ba_S exists, dumps, aligns.
#  gpu lane (gpu.lock only):  inference (VGGT, MASt3R), B0 meshes, V/M meshes as soon as their inputs exist.
# Each step is skipped when its output exists; a failed step leaves rec/<id>/FAIL_<step> and is not retried.
# Done when no step is left: writes lane_<lane>.done.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
Q=scan3d/runs/mvggt_20261106; L=$1
RECS="165506 20261022_134204 20261020_222747"
F222=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec/20261020_222747/ba_S
step() { id=$1; st=$2; exp=$3; [ -f $Q/rec/$id/FAIL_$st ] && return 1
         echo "-- $L $id $st $(date '+%F %T')"; $Q/run_rec.sh $id $st >> $Q/rec/$id.out 2>&1 || true
         [ -f $Q/rec/$id/$exp ] || { echo "   FAILED $id $st (no $exp)"; date > $Q/rec/$id/FAIL_$st; }
         return 0; }
# an output counts as present also when the step that makes it failed (so the other lane does not wait forever)
have() { [ -f $Q/rec/$1/$2 ]; }
gone() { have $1 $2 || have $1 FAIL_$3; }
mdone() { for v in "$@"; do [ -f $Q/rec/$id/mesh_$v/summary.json ] || return 1; done; }
while true; do
  did=0; pend=0
  for id in $RECS; do
    D=$Q/rec/$id
    if [ $id = 20261020_222747 ] && [ ! -f $D/poses_ba.jsonl.gz ]; then
      if [ "$L" = cpu ] && [ -f $F222/poses_ba.jsonl.gz ] && [ -f $F222/ba.json ]; then
        mkdir -p $D && cp $F222/poses_ba.jsonl.gz $F222/poses_vio_kf.jsonl.gz $F222/ba.json $D/ \
          && sha256sum $F222/poses_ba.jsonl.gz $D/poses_ba.jsonl.gz > $D/ba_sha.txt && echo "-- 222747 BA copied $(date '+%F %T')"
      else continue; fi
    fi
    if [ "$L" = cpu ]; then
      have $id frames/frames.npz || { pend=1; step $id dump frames/frames.npz && did=1; break; }
      if have $id raw_vggt.npz && ! have $id al/align_vggt.json; then step $id vggt_al al/align_vggt.json && did=1; break; fi
      if have $id raw_mast3r.npz && ! have $id al/align_mast3r.json; then step $id mast3r_al al/align_mast3r.json && did=1; break; fi
      { { gone $id al/align_vggt.json vggt_al || have $id FAIL_vggt_inf; } && { gone $id al/align_mast3r.json mast3r_al || have $id FAIL_mast3r_inf; }; } || pend=1
    else
      have $id frames/frames.npz || { pend=1; continue; }
      # priority inside a recording: VGGT inference, MASt3R inference, B0, V meshes, M meshes
      if ! gone $id raw_vggt.npz vggt_inf; then step $id vggt_inf raw_vggt.npz && did=1; break; fi
      if ! gone $id raw_mast3r.npz mast3r_inf; then step $id mast3r_inf raw_mast3r.npz && did=1; break; fi
      mdone B0 B0_fs || have $id FAIL_b0 || { step $id b0 mesh_B0_fs/summary.json && did=1; break; }
      if have $id al/align_vggt.json && ! mdone V1 V1_fs V2 V2_fs && ! have $id FAIL_v; then step $id v mesh_V2_fs/summary.json && did=1; break; fi
      if have $id al/align_mast3r.json && ! mdone M1 M1_fs M2 M2_fs && ! have $id FAIL_m; then step $id m mesh_M2_fs/summary.json && did=1; break; fi
      { { mdone V1 V1_fs V2 V2_fs || have $id FAIL_v || have $id FAIL_vggt_inf || have $id FAIL_vggt_al; } && { mdone M1 M1_fs M2 M2_fs || have $id FAIL_m || have $id FAIL_mast3r_inf || have $id FAIL_mast3r_al; }; } || pend=1
    fi
  done
  [ $did = 1 ] && continue
  # 222747 still pending while featloop has not rebuilt it and the cpu lane is busy elsewhere -> stop waiting once the
  # rest is done (README: skipped if missing)
  if [ $pend = 0 ]; then break; fi
  # nothing runnable now: inputs from the other lane are pending
  sleep 30
  [ -f $Q/lanes.stop ] && break
done
[ -f $Q/rec/20261020_222747/poses_ba.jsonl.gz ] || echo "222747 skipped $(date '+%F %T'): featloop ba_S missing" > $Q/rec/20261020_222747.skipped
echo "LANE $L DONE $(date '+%F %T')" | tee $Q/lane_$L.done
