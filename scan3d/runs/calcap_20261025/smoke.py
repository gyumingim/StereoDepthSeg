import socket, subprocess, time, json, os, sys
ADB=["C:/Users/a3162/platform-tools/adb.exe","-s","100.79.156.20:5555"]
def adb(*a): return subprocess.run(ADB+list(a),capture_output=True,text=True,timeout=60).stdout
t0=time.time()
adb("shell","am","start","-n","com.scan3d.arlat/.MainActivity","--es","mode","calcap","--ei","max_s","60","--ei","idle_s","45")
s=None
for i in range(20):
    try: s=socket.create_connection(("100.79.156.20",8811),timeout=3); break
    except OSError: time.sleep(0.7)
print("connect", s is not None, round(time.time()-t0,1)); s.settimeout(8); f=s.makefile("rb")
def cmd(l): s.sendall((l+"\n").encode()); return f.readline().decode().strip()
time.sleep(3)
n=0; tp=time.time()
for i in range(15):
    s.sendall(b"PREV\n"); h=f.readline().decode().strip()
    if h.startswith("JPG"): data=f.read(int(h.split()[1])); n+=1; open("smoke_prev.jpg","wb").write(data)
print("prev ok", n, "of 15 in", round(time.time()-tp,1), "s")
print(cmd("MSG 2563eb 스모크 시험"))
print(cmd("CAP 3 smoke {}")[:300])
st=cmd("STAT"); print(st[:1500])
print(cmd("STOP")); s.close()
time.sleep(2)
print("elapsed", round(time.time()-t0,1))
