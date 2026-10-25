# ruler check: this branch's PRK/B0K cells vs integ-20261025 PR/B0 cells (pose lines without timing / klt / h / late_dropped)
import gzip,json,glob,sys,os
EX={"ms","remap_ms","wait_ms","klt","h","late_dropped"}
mine,ref=sys.argv[1],sys.argv[2]
same=diff=miss=0
for f in sorted(glob.glob(f"/tmp/ovgyro/ate/{mine}/*.poses.jsonl.gz")):
    key=os.path.basename(f).split(f"_{mine}")[0]
    g=f"/tmp/integ/ate/{ref}/{key}_{ref}.poses.jsonl.gz"
    if not os.path.exists(g): miss+=1; continue
    L=lambda p:[{k:v for k,v in json.loads(x).items() if k not in EX} for x in gzip.open(p,"rt") if x.strip()]
    if L(f)==L(g): same+=1
    else: diff+=1; print("DIFF",key)
print(f"{mine} vs integ {ref}: identical {same} different {diff} missing {miss}")
