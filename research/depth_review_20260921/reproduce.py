"""CPU-only review reproductions; no camera access, weights, or production writes.
Run from project venv. Writes latest_results.json beside this script.
Results are observed failure cases, not a passing regression-test report.
"""
import contextlib
import hashlib
import io
import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import lib_rectify as lr
import lib_stereo as ls
import lib_detect as ld
import z_reconstruct as zr
import z_stereo_pose as zp
import z_object_depth as zd
from ultralytics.engine.results import Results

results = {}
K = np.array([[800., 0., 320.], [0., 800., 240.], [0., 0., 1.]])
cal = dict(K=K, dist=np.zeros(5), image_size=(640, 480))
img = np.zeros((480, 640, 3), np.uint8)
T = np.array([[-.1], [0.], [0.]])

def project(points, k):
    v = points @ k.T
    return v[:, :2] / v[:, 2:]

def synthetic_reconstruct(points, R, t, right_cal=cal):
    left = project(points, K)
    right = project(points @ R.T + t.reshape(1, 3), right_cal['K'])
    direct, ok = ls.triangulate(ls._to_normalized(left, K, cal['dist']),
                                ls._to_normalized(right, right_cal['K'], right_cal['dist']), R, t)
    with patch.object(ls, '_match_sift', return_value=(left, right, len(left))):
        out = ls.reconstruct(img, img, cal, R, t, calibR=right_cal)
    return out, float(np.max(np.abs(direct - points)))

rng = np.random.default_rng(20260921)
points = np.column_stack([rng.uniform(-.4, .4, 100), rng.uniform(-.3, .3, 100), rng.uniform(1.5, 3., 100)])
pitch = cv2.Rodrigues(np.array([np.deg2rad(3), 0., 0.]))[0]
out, error = synthetic_reconstruct(points, pitch, T)
results['rotated_epipolar'] = dict(input_matches=100, direct_max_error_m=error, diag=out['diag'])
right_cal = dict(cal, K=np.array([[520.,0.,320.],[0.,520.,240.],[0.,0.,1.]]))
out, error = synthetic_reconstruct(points, np.eye(3), T, right_cal)
results['different_intrinsics'] = dict(input_matches=100, direct_max_error_m=error, diag=out['diag'])

# Physical camera center stays at (+0.1, 0, 0); choose yaw to cancel image disparity near Z=2.
yaw = cv2.Rodrigues(np.array([0., np.arctan(.05), 0.]))[0]
t_yaw = -yaw @ np.array([[.1],[0.],[0.]])
near = np.column_stack([rng.uniform(-.01,.01,30), rng.uniform(-.01,.01,30), np.full(30,2.)])
out, error = synthetic_reconstruct(near, yaw, t_yaw)
results['unrectified_disparity_gate'] = dict(input_matches=30, direct_max_error_m=error, diag=out['diag'])

# Actual detector adapter with fabricated network Results; no model download/inference.
mask = np.zeros((100,100), np.uint8)
mask[10:90,10:90] = 1
mask[20:80,20:80] = 0
res = Results(orig_img=np.zeros((100,100,3),np.uint8), path='synthetic', names={0:'ring'},
              boxes=torch.tensor([[10.,10.,90.,90.,.9,0.]]), masks=torch.from_numpy(mask[None]))
net = SimpleNamespace(predict=lambda *a, **kw: [res])
with patch.dict(ld._MODEL_CACHE, {'review-stub':net}):
    detected = ld.detect_yolo(res.orig_img, model='review-stub')[0]['mask'] > 0
depth = np.where(mask, 1., 3.)
results['mask_topology'] = dict(input_pixels=int(mask.sum()), output_pixels=int(detected.sum()),
    false_added_pixels=int((detected & ~mask.astype(bool)).sum()),
    original_depth_m=float(np.median(depth[mask > 0])), converted_depth_m=float(np.median(depth[detected])))

for name, t in [('normal',T), ('reversed',-T), ('vertical',np.array([[0.],[-.1],[0.]]))]:
    rp = lr.rectify_maps(cal,cal,np.eye(3),t)
    d = np.full((480,640),40,np.float32)
    formula = lr.disparity_to_depth(d,rp)
    qxyz = cv2.reprojectImageTo3D(d,rp['Q'])
    results['rectify_'+name] = dict(baseline_m=rp['baseline_m'], formula_z_m=float(formula[240,320]),
                                   q_z_m=float(qxyz[240,320,2]))
rp = lr.rectify_maps(cal,cal,np.eye(3),T)
bad = np.array([[np.nan,np.inf,-np.inf,0.,-1.,40.]],np.float32)
results['invalid_disparity'] = dict(output=[str(x) for x in lr.disparity_to_depth(bad,rp)[0]])
small = lr.rectify_maps(cal,cal,np.eye(3),T,size=(320,240))
results['rectify_size'] = dict(requested_size=small['size'], resulting_fx=small['f'],
    full_fx=rp['f'], output_cx=small['cx'], sampled_source_x0=float(small['mapL'][0][0,0]),
    sampled_source_xlast=float(small['mapL'][0][0,-1]))

# Use real clustering on a dense target plus spatially separated background samples.
foreground = rng.normal([0.,0.,2.], [.005,.005,.001], size=(50,3))
background = rng.uniform([-.9,-.7,3.8],[.9,.7,4.2],size=(60,3))
out, error = synthetic_reconstruct(np.vstack([foreground,background]), np.eye(3), T)
results['postcluster_distance'] = dict(diag_z_m=out['diag'].get('z_m'), final_points=len(out['points_map']),
    kept_depth_m=float(np.median(out['points_map'][:,1])) if len(out['points_map']) else None,
    clusters=out['box']['clusters'] if out['box'] else None)

# Exercise result writing in isolation with temporary shots and stubbed expensive stages.
with tempfile.TemporaryDirectory() as td:
    root=Path(td); shots=root/'shots'; shots.mkdir()
    (shots/'meta.json').write_text('{"mode":"mono","baseline_m":0.1}')
    cv2.imwrite(str(shots/'L.png'),img); cv2.imwrite(str(shots/'R.png'),img)
    args=SimpleNamespace(roi=[(0,0,10,10)],yolo=False,seg=False,known_mm=None,z_range=(.1,50.),no_view=True)
    result_path=root/'results.json'; result_path.write_text('[{"old":true}]')
    no_points=dict(diag=dict(n_knn=0,n_ratio=0),box=None)
    with patch.object(zr,'HERE',root),patch.object(zr,'SHOT_DIR',shots),patch.object(zr,'load_pose',return_value=(np.eye(3),T,cal,cal)),patch.object(ls,'reconstruct',return_value=no_points),contextlib.redirect_stdout(io.StringIO()):
        zr.main(args)
    results['stale_results'] = json.loads(result_path.read_text())
    fail=dict(diag=dict(n_knn=10,n_ratio=10,n_final=4,dy_rms=0.,disp_px=(40.,40.,40.),z_m=(2.,2.,2.),reproj_px=5.),
              box=dict(center=np.array([0.,2.,0.]),dims_sorted=np.array([.2,.1,.01]),method='stub'))
    with patch.object(zr,'HERE',root),patch.object(zr,'SHOT_DIR',shots),patch.object(zr,'load_pose',return_value=(np.eye(3),T,cal,cal)),patch.object(ls,'reconstruct',return_value=fail),contextlib.redirect_stdout(io.StringIO()):
        zr.main(args)
    results['failed_gate_saved'] = json.loads(result_path.read_text())
    pose=root/'calib_stereo.json'; pose.write_text(json.dumps({'T':[-.1,0,0]}))
    result_path.write_text('[{"label":"test","distance_m":2.0}]')
    with patch.object(zp,'HERE',root),patch.object(zp,'OUT_JSON',pose),contextlib.redirect_stdout(io.StringIO()):
        zp.cmd_rescale('1=1.0')
        first=json.loads(pose.read_text())['baseline_m']
        zp.cmd_rescale('1=1.0')
        second=json.loads(pose.read_text())['baseline_m']
    results['repeat_rescale'] = dict(initial_baseline_m=.1, first_m=first, repeated_m=second)

# Algebra for the pinned upstream single-TRT demo, not a TRT engine execution.
s=.5; f=800.; b=.1; model_disparity=20.
results['upstream_trt_scaling'] = dict(correct_depth_m=(f*s)*b/model_disparity,
    script_depth_m=(f*s)*b/(model_disparity/s))

# New dense entry point, added to the workspace while this review was running.
rp = lr.rectify_maps(cal,cal,np.eye(3),T)
rp['roi2'] = (0, 100, 640, 200)
valid, depth = zd.valid_mask(np.full((480,640),40,np.float32),rp,(.1,20.))
results['right_roi_y_ignored'] = dict(right_roi=rp['roi2'], row0_col100_valid=bool(valid[0,100]),
    valid_outside_right_y=int(valid[:100].sum()+valid[300:].sum()))
all_points=np.vstack([foreground,background]).reshape(10,11,3)
agg,_=zd.aggregate(np.ones((10,11),np.uint8),np.ones((10,11),bool),all_points[:,:,2],all_points,'synthetic')
results['dense_aggregate'] = agg

with tempfile.TemporaryDirectory() as td:
    root=Path(td)
    for name in ['L.png','R.png']: cv2.imwrite(str(root/name),img)
    cp=root/'cal.json'; cp.write_text(json.dumps(dict(K=K.tolist(),dist=[0]*5,image_size=[640,480])))
    sp=root/'stereo.json'; sp.write_text(json.dumps(dict(R=np.eye(3).tolist(),T=T.ravel().tolist())))
    args=SimpleNamespace(calib_l=str(cp),calib_r=str(cp),calib_stereo=str(sp),left=str(root/'L.png'),right=str(root/'R.png'),
        out=str(root/'out'),ckpt='stub',iters=8,scale=1.,z_range=(.1,20.),mode='both',model=None,conf=.25,classes=None,device='cpu',known_m=None)
    with patch.object(lr,'epipolar_check',return_value=(float('nan'),float('nan'),0)),patch.object(zd.lib_ffs,'load',return_value=None),patch.object(zd.lib_ffs,'infer',return_value=np.full((480,640),40,np.float32)),patch.object(ld,'detect_yolo',return_value=[]),contextlib.redirect_stdout(io.StringIO()):
        try: zd.main(args)
        except ValueError as exc: results['dense_nan_export']=str(exc)
    zp._save(sp,cal,cal,np.eye(3),T,0.,0.,None,None)
    results['provisional_method'] = json.loads(sp.read_text())['method']

left_keys=[cv2.KeyPoint(100.,100.,1),cv2.KeyPoint(110.,100.,1)]
right_keys=[cv2.KeyPoint(90.,100.,1)]
sift=SimpleNamespace(detectAndCompute=__import__('unittest.mock',fromlist=['Mock']).Mock(side_effect=[
    (left_keys,np.zeros((2,128),np.float32)),(right_keys,np.zeros((1,128),np.float32))]))
with patch.object(cv2,'SIFT_create',return_value=sift):
    try: lr.epipolar_check(img,img)
    except ValueError as exc: results['epipolar_single_descriptor']=str(exc)
results['sources_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
    ['lib_stereo.py','lib_detect.py','lib_rectify.py','lib_ffs.py','z_object_depth.py','z_reconstruct.py','z_stereo_pose.py','z_capture.py','HARNESS.md']}
Path(__file__).with_name('latest_results.json').write_text(json.dumps(results,indent=2,allow_nan=False))
print(json.dumps(results,indent=2,allow_nan=False))
