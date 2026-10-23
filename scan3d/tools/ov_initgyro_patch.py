#!/usr/bin/env python3
"""Gyro stillness gate for OpenVINS static init (2026-10-23, docs/initgyro_plan.md).

Patches a COPY of ~/scanner/src/open_vins (never the original): ov_init options get `init_max_gyro` (rad/s, optional key,
default 0 = off = same code path as stock). StaticInitializer: after the stock accel-excitation checks, if init_max_gyro > 0,
split the bias window (older half, [newest - W, newest - W/2], the window bg/gravity/v=0 are taken from) into 0.1 s
sub-windows, take g = max over sub-windows of |mean(gyro)|; if g > init_max_gyro -> return false (wait for the next window).

  python3 tools/ov_initgyro_patch.py ~/scanner/src/open_vins_initgyro
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
    "  /// (open_vins_initgyro) static init: max over 0.1 s sub-windows of |mean gyro| in the bias window (rad/s). 0 = off\n"
    "  double init_max_gyro = 0.0;\n")
sub(H, '      parser->parse_config("init_max_disparity", init_max_disparity);\n',
    '      parser->parse_config("init_max_disparity", init_max_disparity);\n'
    '      parser->parse_config("init_max_gyro", init_max_gyro, false);\n')
sub(H, '    PRINT_DEBUG("  - init_max_disparity: %.2f\\n", init_max_disparity);\n',
    '    PRINT_DEBUG("  - init_max_disparity: %.2f\\n", init_max_disparity);\n'
    '    PRINT_DEBUG("  - init_max_gyro: %.4f\\n", init_max_gyro);\n')

S = "ov_init/src/static/StaticInitializer.cpp"
sub(S, "  // Get rotation with z axis aligned with -g (z_in_G=0,0,1)\n",
    "  // (open_vins_initgyro) gyro stillness of the bias window: max over 0.1 s sub-windows of |mean gyro|\n"
    "  if (params.init_max_gyro > 0.0) {\n"
    "    double t_lo = newesttime - 1.0 * params.init_window_time;\n"
    "    int nbin = std::max(1, (int)std::ceil(0.5 * params.init_window_time / 0.1 - 1e-9));\n"
    "    std::vector<Eigen::Vector3d> wsum(nbin, Eigen::Vector3d::Zero());\n"
    "    std::vector<int> wcnt(nbin, 0);\n"
    "    for (const ImuData &data : window_2to1) {\n"
    "      int b = std::min(nbin - 1, std::max(0, (int)std::floor((data.timestamp - t_lo) / 0.1)));\n"
    "      wsum.at(b) += data.wm;\n"
    "      wcnt.at(b)++;\n"
    "    }\n"
    "    double g_max = 0.0;\n"
    "    for (int b = 0; b < nbin; b++) {\n"
    "      if (wcnt.at(b) > 0)\n"
    "        g_max = std::max(g_max, (wsum.at(b) / wcnt.at(b)).norm());\n"
    "    }\n"
    "    if (g_max > params.init_max_gyro) {\n"
    "      PRINT_INFO(YELLOW \"[init-s]: gyro not still in bias window, %.4f > %.4f rad/s\\n\" RESET, g_max, params.init_max_gyro);\n"
    "      return false;\n"
    "    }\n"
    "    PRINT_INFO(GREEN \"[init-s]: gyro still in bias window, %.4f <= %.4f rad/s\\n\" RESET, g_max, params.init_max_gyro);\n"
    "  }\n\n"
    "  // Get rotation with z axis aligned with -g (z_in_G=0,0,1)\n")
