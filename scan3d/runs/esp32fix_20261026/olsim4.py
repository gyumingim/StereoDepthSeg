from olsim3 import show
from olsim2 import mk
for kw in [dict(v1=0.4,r2=0.99,rend=0.7,tramp=0.4),dict(v1=0.6,r2=0.95,rend=0.7,tramp=0.4,emax=0.05),dict(v1=0.6,r2=0.95,rend=0.75,tramp=0.4)]:
    show(str(kw), lambda h0: (lambda r:(r[0],r[3]))(mk(h0,**kw)))
