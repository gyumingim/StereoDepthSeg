# integ2 calcap smoke (copy of runs/calcap_20261025/smoke.py WITHOUT CAP: no stills written): start calcap mode, PREV x15, STAT, STOP.
import socket, subprocess, time, json, os, sys
ADB=["C:/Users/a3162/platform-tools/adb.exe","-s","100.79.156.20:5555"]
def adb(*a): return subprocess.run(ADB+list(a),capture_output=True,text=True,timeout=60).stdout
t0=time.time()
print(time.strftime("%F %T"), "start")
adb("shell","am","start","-n","com.scan3d.arlat/.MainActivity","--es","mode","calcap","--ei","max_s","60","--ei","idle_s","45")
s=None
for i in range(20):
    try: s=socket.create_connection(("100.79.156.20",8811),timeout=3); break
    except OSError: time.sleep(0.7)
print("connect", s is not None, round(time.time()-t0,1))
if s is None:
    adb("shell","am","force-stop","com.scan3d.arlat"); sys.exit(1)
s.settimeout(8); f=s.makefile("rb")
def cmd(l): s.sendall((l+"\n").encode()); return f.readline().decode().strip()
time.sleep(3)
n=0; tp=time.time()
for i in range(15):
    s.sendall(b"PREV\n"); h=f.readline().decode().strip()
    if h.startswith("JPG"): data=f.read(int(h.split()[1])); n+=1
print("prev ok", n, "of 15 in", round(time.time()-tp,1), "s")
st=cmd("STAT"); print(st[:1500])
print(cmd("STOP")); s.close()
time.sleep(2)
adb("shell","am","force-stop","com.scan3d.arlat")
time.sleep(1)
print("elapsed", round(time.time()-t0,1))
cam=adb("shell","dumpsys media.camera | grep -A2 'Active Camera Clients'")
print("camera:", cam.strip().replace("\n"," | "))
print("app pid:", adb("shell","pidof com.scan3d.arlat").strip() or "none")
