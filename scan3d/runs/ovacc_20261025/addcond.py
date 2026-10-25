"""atomically add condition lines to acc_run.CONDS (jobs import the file while it runs):
python3 addcond.py '"F30": L(cfg="fast_threshold=30"),' ..."""
import os, sys, py_compile
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "acc_run.py")
s = open(p).read()
anchor = '    "DS2": L(cfg="dt_slam_delay=2"),\n'
assert anchor in s
add = "".join(f"    {x}\n" for x in sys.argv[1:] if x.split(":")[0] not in s)
s = s.replace(anchor, anchor + add)
open(p + ".tmp", "w").write(s)
py_compile.compile(p + ".tmp", doraise=True)
os.replace(p + ".tmp", p)
print("added:\n" + add)
