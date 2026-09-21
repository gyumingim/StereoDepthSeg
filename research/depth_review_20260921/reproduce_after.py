"""reproduce.py 의 수정 후 판 — 같은 입력, 새 계약(예외·인덱스·상태값)에 맞춰 관측만 한다.
Run: ./venv/bin/python research/depth_review_20260921/reproduce_after.py"""
import contextlib, io, json, sys, tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2, numpy as np, torch

ROOT = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(ROOT))
import lib_rectify as lr, lib_stereo as ls, lib_detect as ld, z_reconstruct as zr, z_stereo_pose as zp, z_object_depth as zd
from ultralytics.engine.results import Results

R = {}
K = np.array([[800., 0., 320.], [0., 800., 240.], [0., 0., 1.]])
cal = dict(K=K, dist=np.zeros(5), image_size=(640, 480)); img = np.zeros((480, 640, 3), np.uint8)
T = np.array([[-.1], [0.], [0.]])
proj = lambda P, k: (P @ k.T)[:, :2] / (P @ k.T)[:, 2:]

def recon(points, Rm, t, rc=cal):
    left, right = proj(points, K), proj(points @ Rm.T + t.reshape(1, 3), rc['K'])
    with patch.object(ls, '_match_sift', return_value=(left, right, len(left))):
        out = ls.reconstruct(img, img, cal, Rm, t, calibR=rc)
    err = float(np.max(np.abs(out['points_cam'] - points[out['idx_final']]))) if len(out['idx_final']) else None
    d = out['diag']; return dict(n_epipolar=d['n_epipolar'], n_depth=d['n_depth'], n_final=d['n_final'], max_3d_err_m=err), out

rng = np.random.default_rng(20260921)
pts = np.column_stack([rng.uniform(-.4, .4, 100), rng.uniform(-.3, .3, 100), rng.uniform(1.5, 3., 100)])
R['rotated_epipolar'], _ = recon(pts, cv2.Rodrigues(np.array([np.deg2rad(3), 0., 0.]))[0], T)
rc = dict(cal, K=np.array([[520., 0., 320.], [0., 520., 240.], [0., 0., 1.]]))
R['different_intrinsics'], _ = recon(pts, np.eye(3), T, rc)
yaw = cv2.Rodrigues(np.array([0., np.arctan(.05), 0.]))[0]
near = np.column_stack([rng.uniform(-.01, .01, 30), rng.uniform(-.01, .01, 30), np.full(30, 2.)])
R['unrectified_disparity_gate'], _ = recon(near, yaw, -yaw @ np.array([[.1], [0.], [0.]]))

mask = np.zeros((100, 100), np.uint8); mask[10:90, 10:90] = 1; mask[20:80, 20:80] = 0
res = Results(orig_img=np.zeros((100, 100, 3), np.uint8), path='synthetic', names={0: 'ring'},
              boxes=torch.tensor([[10., 10., 90., 90., .9, 0.]]), masks=torch.from_numpy(mask[None]))
with patch.dict(ld._MODEL_CACHE, {'stub': SimpleNamespace(predict=lambda *a, **kw: [res])}):
    det = ld.detect_yolo(res.orig_img, model='stub')[0]['mask'] > 0
depth = np.where(mask, 1., 3.)
R['mask_topology'] = dict(input_pixels=int(mask.sum()), output_pixels=int(det.sum()),
                         false_added_pixels=int((det & ~mask.astype(bool)).sum()),
                         converted_depth_m=float(np.median(depth[det])))

for name, t in [('normal', T), ('reversed', -T), ('vertical', np.array([[0.], [-.1], [0.]]))]:
    try:
        rp = lr.rectify_maps(cal, cal, np.eye(3), t); d = np.full((480, 640), 40, np.float32)
        R['rectify_' + name] = dict(baseline_m=rp['baseline_m'], tx_m=rp['tx_m'],
                                    formula_z_m=float(lr.disparity_to_depth(d, rp)[240, 320]),
                                    q_z_m=float(cv2.reprojectImageTo3D(d, rp['Q'])[240, 320, 2]))
    except Exception as e: R['rectify_' + name] = f"{type(e).__name__}: {str(e)[:70]}"
rp = lr.rectify_maps(cal, cal, np.eye(3), T)
R['invalid_disparity'] = [str(x) for x in lr.disparity_to_depth(np.array([[np.nan, np.inf, -np.inf, 0., -1., 40.]], np.float32), rp)[0]]
try: lr.rectify_maps(cal, cal, np.eye(3), T, size=(320, 240)); R['rectify_size'] = 'size 인자 여전히 허용'
except TypeError as e: R['rectify_size'] = f"TypeError (제거됨): {str(e)[:60]}"

fg = rng.normal([0., 0., 2.], [.005, .005, .001], size=(50, 3)); bg = rng.uniform([-.9, -.7, 3.8], [.9, .7, 4.2], size=(60, 3))
summ, out = recon(np.vstack([fg, bg]), np.eye(3), T)
R['postcluster_distance'] = dict(diag_z_m=out['diag'].get('z_m'), diag_z_raw_m=out['diag'].get('z_raw_m'),
                                 center_cam_m=out['diag'].get('center_cam_m'), final_points=len(out['points_map']))

with tempfile.TemporaryDirectory() as td:
    root = Path(td); shots = root / 'shots'; shots.mkdir()
    (shots / 'meta.json').write_text('{"mode":"mono","baseline_m":0.1}')
    cv2.imwrite(str(shots / 'L.png'), img); cv2.imwrite(str(shots / 'R.png'), img)
    args = SimpleNamespace(roi=[(0, 0, 10, 10)], yolo=False, seg=False, known_mm=None, z_range=(.1, 50.), no_view=True)
    rp_ = root / 'results.json'; rp_.write_text('[{"old":true}]')
    info = dict(pose_source='mono', baseline_m=.1, scale_status='measured(ruler)')
    nop = dict(diag=dict(n_knn=0, n_ratio=0, n_epipolar=0, n_depth=0, n_final=0), box=None, points_cam=np.zeros((0, 3)), idx_final=np.zeros(0, int))
    P = [patch.object(zr, 'HERE', root), patch.object(zr, 'SHOT_DIR', shots), patch.object(zr, 'RESULTS_JSON', rp_),
         patch.object(zr, 'load_pose', return_value=(np.eye(3), T, cal, cal, info))]
    with P[0], P[1], P[2], P[3], patch.object(ls, 'reconstruct', return_value=nop), contextlib.redirect_stdout(io.StringIO()):
        zr.main(args)
    j = json.loads(rp_.read_text()); R['stale_results'] = dict(rewritten='meta' in j, n_objects=len(j.get('objects', [])), statuses=[o['status'] for o in j.get('objects', [])])
    fail = dict(diag=dict(n_knn=10, n_ratio=10, n_epipolar=10, n_depth=10, n_final=4, epi_px=0., tri_deg=(2., 2., 2.), z_m=(2., 2., 2.), z_raw_m=(2., 2., 2.),
                          reproj_px=5., center_cam_m=[0., 0., 2.]), box=dict(center=np.array([0., 2., 0.]), dims_sorted=np.array([.2, .1, .01]), method='stub'),
                points_cam=np.zeros((4, 3)), idx_final=np.arange(4))
    with P[0], P[1], P[2], P[3], patch.object(ls, 'reconstruct', return_value=fail), contextlib.redirect_stdout(io.StringIO()):
        zr.main(args)
    j = json.loads(rp_.read_text()); R['failed_gate_saved'] = dict(status=j['objects'][0]['status'], gates=j['objects'][0]['gates'], n_ok_meta=j['meta']['n_ok'])
    # rescale: 현재 baseline 과 같은 결과 -> 1회 허용, 2회째 거부
    pose = root / 'calib_stereo.json'; pose.write_text(json.dumps(dict(T=[-.1, 0, 0], baseline_m=.1, scale_status='provisional')))
    rp_.write_text(json.dumps(dict(meta=dict(pose_source='stereo', stereo_baseline_m=.1, time='t'), objects=[dict(label='test', status='ok', distance_m=2.0)])))
    log = []
    for i in range(2):
        try:
            with patch.object(zp, 'OUT_JSON', pose), patch.object(zp, 'RESULTS_JSON', rp_), contextlib.redirect_stdout(io.StringIO()):
                zp.cmd_rescale('1=1.0'); log.append(json.loads(pose.read_text())['baseline_m'])
        except SystemExit as e: log.append(f"거부: {str(e)[:60]}")
    R['repeat_rescale'] = dict(initial=.1, first=log[0], second=log[1], scale_status=json.loads(pose.read_text())['scale_status'])

rp = lr.rectify_maps(cal, cal, np.eye(3), T); rp['roi2'] = (0, 100, 640, 200)
valid, _ = zd.valid_mask(np.full((480, 640), 40, np.float32), rp, (.1, 20.))
R['right_roi_y_ignored'] = dict(valid_outside_right_y=int(valid[:100].sum() + valid[300:].sum()))
allp = np.vstack([fg, bg]).reshape(10, 11, 3)
det_ = dict(box=(0, 0, 11, 10), conf=1.0, label='synthetic')
agg = zd.aggregate(zd._empty('synthetic', 1, det_, 'bbox'), np.ones((10, 11), np.uint8), np.ones((10, 11), bool), allp[:, :, 2], allp)
R['dense_aggregate'] = {k: agg.get(k) for k in ('status', 'depth_raw_m', 'depth_rect_m', 'selected_fraction', 'selection_policy', 'minority_fraction')}

with tempfile.TemporaryDirectory() as td:
    root = Path(td)
    for n in ['L.png', 'R.png']: cv2.imwrite(str(root / n), img)
    cp = root / 'cal.json'; cp.write_text(json.dumps(dict(K=K.tolist(), dist=[0] * 5, image_size=[640, 480])))
    sp = root / 'stereo.json'; sp.write_text(json.dumps(dict(K1=K.tolist(), dist1=[0]*5, K2=K.tolist(), dist2=[0]*5, image_size=[640, 480], R=np.eye(3).tolist(), T=T.ravel().tolist(), baseline_m=.1, scale_status='measured')))
    args = SimpleNamespace(calib_l=str(cp), calib_r=str(cp), calib_stereo=str(sp), left=str(root / 'L.png'), right=str(root / 'R.png'), out=str(root / 'out'),
                           ckpt='stub', iters=8, scale=1., z_range=(.1, 20.), mode='both', model=None, conf=.25, classes=None, device='cpu', known_m=None, known_idx=1,
                           pose='stereo', lr_check=False, lr_thr_px=1.5)
    unavailable = dict(status='unavailable', n=0, dy_med=None, dy_rms=None, pos_dx_frac=None)
    try:
        with patch.object(lr, 'epipolar_check', return_value=unavailable), patch.object(zd.lib_ffs, 'load', return_value=None), \
             patch.object(zd.lib_ffs, 'infer', return_value=np.full((480, 640), 40, np.float32)), patch.object(ld, 'detect_yolo', return_value=[]), contextlib.redirect_stdout(io.StringIO()):
            zd.main(args)
        j = json.loads((root / 'out' / 'objects.json').read_text()); R['dense_nan_export'] = dict(saved=True, epipolar=j['meta']['checks']['epipolar'])
    except Exception as e: R['dense_nan_export'] = f"{type(e).__name__}: {e}"
    with patch.object(zp, 'OUT_JSON', sp), contextlib.redirect_stdout(io.StringIO()):
        zp._save(cal, cal, np.eye(3), T, 0., 0., 100, 50, 'essential+provisional', 'provisional')
    j = json.loads(sp.read_text()); R['provisional_method'] = dict(method=j['method'], scale_status=j['scale_status'])

sift = SimpleNamespace(detectAndCompute=__import__('unittest.mock', fromlist=['Mock']).Mock(side_effect=[
    ([cv2.KeyPoint(100., 100., 1), cv2.KeyPoint(110., 100., 1)], np.zeros((2, 128), np.float32)), ([cv2.KeyPoint(90., 100., 1)], np.zeros((1, 128), np.float32))]))
with patch.object(cv2, 'SIFT_create', return_value=sift):
    try: R['epipolar_single_descriptor'] = lr.epipolar_check(img, img)
    except Exception as e: R['epipolar_single_descriptor'] = f"{type(e).__name__}: {e}"
Path(__file__).with_name('after_results.json').write_text(json.dumps(R, indent=2, allow_nan=False, default=str))
print(json.dumps(R, indent=1, allow_nan=False, default=str))
