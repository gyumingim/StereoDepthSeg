#!/usr/bin/env python3
"""Static-init gyro bias from a prior instead of the window mean (2026-10-23, docs/initbg_plan.md).

Patches a COPY of ~/scanner/src/open_vins (never the original). New optional keys (ov_init options):
  init_bg_mode   int, 0 = off (stock: bg0 = mean gyro of the bias window), 1 = bg0 = 0, 2 = bg0 = init_bg_prior
  init_bg_prior  [x, y, z] rad/s (used only with mode 2)
Init timing, gravity direction, ba, v = 0 and the initial covariance (bg sigma 0.02 rad/s) are untouched.

  python3 tools/ov_initbg_patch.py ~/scanner/src/open_vins_initbg
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])


def sub(path, old, new):
    p = root / path
    s = p.read_text()
    if new in s:
        print("already", path)
        return
    assert s.count(old) == 1, (path, old[:60])
    p.write_text(s.replace(old, new))
    print("patched", path)


H = "ov_init/src/init/InertialInitializerOptions.h"
sub(H, "  double init_max_disparity = 1.0;\n",
    "  double init_max_disparity = 1.0;\n\n"
    "  /// (open_vins_initbg) static init gyro bias: 0 = window mean (stock), 1 = zero, 2 = init_bg_prior\n"
    "  int init_bg_mode = 0;\n\n"
    "  /// (open_vins_initbg) gyro bias prior for init_bg_mode 2 (rad/s)\n"
    "  Eigen::Vector3d init_bg_prior = Eigen::Vector3d::Zero();\n")
sub(H, '      parser->parse_config("init_max_disparity", init_max_disparity);\n',
    '      parser->parse_config("init_max_disparity", init_max_disparity);\n'
    '      parser->parse_config("init_bg_mode", init_bg_mode, false);\n'
    '      std::vector<double> bg_prior = {0, 0, 0};\n'
    '      parser->parse_config("init_bg_prior", bg_prior, false);\n'
    '      init_bg_prior << bg_prior.at(0), bg_prior.at(1), bg_prior.at(2);\n')
sub(H, '    PRINT_DEBUG("  - init_max_disparity: %.2f\\n", init_max_disparity);\n',
    '    PRINT_DEBUG("  - init_max_disparity: %.2f\\n", init_max_disparity);\n'
    '    PRINT_DEBUG("  - init_bg_mode: %d prior %.5f, %.5f, %.5f\\n", init_bg_mode, init_bg_prior(0), init_bg_prior(1), init_bg_prior(2));\n')

S = "ov_init/src/static/StaticInitializer.cpp"
sub(S, "  Eigen::Vector3d bg = w_avg_2to1;\n",
    "  Eigen::Vector3d bg = w_avg_2to1;\n"
    "  // (open_vins_initbg) bias from a prior instead of the window mean\n"
    "  if (params.init_bg_mode == 1) {\n"
    "    bg.setZero();\n"
    "  } else if (params.init_bg_mode == 2) {\n"
    "    bg = params.init_bg_prior;\n"
    "  }\n"
    "  if (params.init_bg_mode != 0) {\n"
    "    PRINT_INFO(GREEN \"[init-s]: bg mode %d -> %.5f, %.5f, %.5f (window mean %.5f, %.5f, %.5f)\\n\" RESET, params.init_bg_mode, bg(0), bg(1),\n"
    "               bg(2), w_avg_2to1(0), w_avg_2to1(1), w_avg_2to1(2));\n"
    "  }\n")
