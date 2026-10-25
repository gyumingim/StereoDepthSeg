import gzip, csv
R='/home/system/scanner/StereoDepthSeg-ovdrone/scan3d/runs/ovdrone_20261025/phone/out_L_drone2/'
def q(x,f): x=sorted(x); return x[min(len(x)-1,int(f*len(x)))]
for run,fn in (('M_viop_r1','viop_log.csv.gz'),('M_mav_r1','mav_log.csv.gz')):
    rows=list(csv.DictReader(gzip.open(R+run+'/'+fn,'rt')))
    t=[int(r['t_pred_ns']) for r in rows]
    g=[(b-a)/1e6 for a,b in zip(t,t[1:])]
    dur=(t[-1]-t[0])/1e9
    print(run,'n',len(t),'dur %.1f s'%dur,'rate %.1f Hz'%(len(t)/dur),'gap p50 %.2f p95 %.2f max %.2f'%(q(g,.5),q(g,.95),max(g)),
      'gap>10ms %.1f%%'%(100*sum(x>10 for x in g)/len(g)),'gap>20ms %.2f%%'%(100*sum(x>20 for x in g)/len(g)),
      'age p50 %.2f p95 %.2f'%(q([float(r['age_ms']) for r in rows],.5),q([float(r['age_ms']) for r in rows],.95)))
