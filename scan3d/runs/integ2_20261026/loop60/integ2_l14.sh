R=~/scanner/tmp_integ2l; MINE=~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge
C4=~/scanner/tmp_ovloop/cfg_c4/estimator_config.yaml
mkdir -p $R/novoc/sub $R/withvoc && cp ~/scanner/tmp_ovloop/cfg_c4/* $R/novoc/sub/ && cp ~/scanner/tmp_ovloop/cfg_c4/* $R/withvoc/ && cp ~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/loop/voc_k10L4.voc $R/withvoc/
t() { local n="$1"; shift; $MINE "$@" --stdio < /dev/null > /dev/null 2> $R/chk.err; echo "rc $? | $n | $(grep -E "vocabulary|dropped|cannot|loop|ERROR|error" $R/chk.err | head -3 | tr "
" " ")"; }
t "rec60, voc next to cfg" $R/withvoc/estimator_config.yaml --profile rec60
t "rec60, no voc near cfg -> source fallback" $R/novoc/sub/estimator_config.yaml --profile rec60
t "rec60 + --loop 0" $R/novoc/sub/estimator_config.yaml --profile rec60 --loop 0
t "rec20261025 (no loop expected)" $R/novoc/sub/estimator_config.yaml --profile rec20261025
t "rec60 + recover 30" $R/novoc/sub/estimator_config.yaml --profile rec60 --recover 30
t "explicit --loop 1 + recover 30" $R/novoc/sub/estimator_config.yaml --loop 1 --recover 30
t "rec60 + bad --loop-vocab" $R/novoc/sub/estimator_config.yaml --profile rec60 --loop-vocab /nonexist.voc
