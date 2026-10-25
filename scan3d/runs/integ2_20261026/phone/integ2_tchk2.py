import csv,sys
O=sys.argv[1]
for t in sys.argv[2:]:
    T=list(csv.DictReader(open(f"{O}/{t}/tl.csv")))
    big=[(round(float(r['t']),3),round(float(r['feed_ms']),1),round(float(r['wait_ms']),1),r['init'],r['cam_queue']) for r in T if float(r['feed_ms'])>100]
    # lag: rx_boot - stream t
    print(t,"frames",len(T),"feed>100ms:",big[:8])
    lag=[(float(r['rx_boot_ns'])/1e6-float(r['t'])*1e3) for r in T if r['rx_boot_ns'] not in ('','0')]
    if lag:
        m0=min(lag); 
        L=[(round(float(r['t']),2), round(float(r['rx_boot_ns'])/1e6-float(r['t'])*1e3-m0,1)) for r in T if r['rx_boot_ns'] not in ('','0')]
        print("   rx lag rel (t, ms) every 150th:", L[::150][:16])
