#!/usr/bin/env python3
"""npu: downstream check of quantised XFeat: on 13 eval pairs (frame, +30 frames) take top-512 keypoints from each XFeat
variant, match with the fp32 LighterGlue (lgc_512.onnx, ORT CPU = same as kornia), then RANSAC fundamental-matrix inliers
(cv2, 1 px, seed fixed).  Compare #matches and #inliers vs fp32 XFeat.  venv_ffs python (torch, cv2).
  python3 acc_downstream.py OUT.json"""
import os, sys, json, time
import cv2, numpy as np, onnxruntime as ort, torch, torch.nn.functional as F
import tensorflow as tf  # noqa (only if available)
