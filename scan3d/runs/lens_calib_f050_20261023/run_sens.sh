#!/bin/bash
# Sensitivity: drop the overexposed near shot / vibrating short shots / partial corner shots (run from scan3d/)
cd "$(dirname "$0")/../.."
D=runs/lens_calib_f050_20261023
P="../venv_ffs/bin/python $D/calib.py --models k1k2p_k3 rational k1_prior"
$P --exclude hs_20261023_201830_std60 --tag ex201830 > $D/logs/ex201830.log 2>&1 &
$P --exclude hs_20261023_210124_std60 hs_20261023_210413_std60 hs_20261023_210805_std60 hs_20261023_211016_std60 --tag exvib > $D/logs/exvib.log 2>&1 &
$P --exclude hs_20261023_205456_std60 hs_20261023_205650_std60 --tag expartial > $D/logs/expartial.log 2>&1 &
$P --exclude hs_20261023_210928_std60 --tag ex210928 > $D/logs/ex210928.log 2>&1 &
wait
for f in ex201830 exvib expartial ex210928; do echo "== $f"; grep -E "^\[|strat" $D/logs/$f.log | cut -c1-420; done
