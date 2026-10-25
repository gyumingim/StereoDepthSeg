# integ2 calcap smoke v2 = the calcap server's own start sequence (calcap_server.py start_app + session reconnect, 3 attempts),
# no CAP (no stills written), PREV x15, STAT, STOP, force-stop, camera check. Camera time <= ~15 s.
import socket, subprocess, time, sys
APP="com.scan3d.arlat"
ADB=["C:/Users/a3162/platform-tools/adb.exe","-s","100.79.156.20:5555"]
def adb(*a): return subprocess.run(ADB+list(a),capture_output=True,text=True,timeout=60).stdout
t0=time.time(); print(time.strftime("%F %T"), "start")
adb("shell","am","force-stop",APP)
adb("shell","input","keyevent","KEYCODE_WAKEUP")
adb("shell","pm","grant",APP,"android.permission.DUMP")
print(adb("shell","am","start","-n",APP+"/.MainActivity","--es","mode","calcap","--ei","fps","10","--ei","port","8811","--ei","idle_s","45","--ei","max_s","60").strip()[-120:])
ok=False
for attempt in range(3):
    time.sleep(3.0)
    s=None
    for i in range(20):
        try: s=socket.create_connection(("100.79.156.20",8811),timeout=3); break
        except OSError: time.sleep(1.0)
    print("attempt", attempt, "connect", s is not None, round(time.time()-t0,1))
    if s is None: break
    s.settimeout(8); f=s.makefile("rb")
    def cmd(l):
        s.sendall((l+"\n").encode()); r=f.readline()
        if not r: raise ConnectionError("phone closed the link")
        return r.decode().strip()
    try:
        n=0; tp=time.time()
        for i in range(15):
            s.sendall(b"PREV\n"); h=f.readline().decode().strip()
            if not h: raise ConnectionError("phone closed the link (PREV)")
            if h.startswith("JPG"): f.read(int(h.split()[1])); n+=1
        print("prev ok", n, "of 15 in", round(time.time()-tp,1), "s")
        print("STAT", cmd("STAT")[:1200])
        print("STOP", cmd("STOP")); s.close(); ok=True; break
    except (ConnectionError, OSError) as e:
        print("link closed:", e); s.close()
time.sleep(2)
adb("shell","am","force-stop",APP); time.sleep(1)
print("ok", ok, "elapsed", round(time.time()-t0,1))
print("camera:", adb("shell","dumpsys media.camera | grep -A1 'Active Camera Clients'").strip().replace("\n"," | "))
print("app pid:", adb("shell","pidof",APP).strip() or "none")
