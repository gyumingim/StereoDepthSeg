' 3D scanner idle web server (http://localhost:8795), started hidden at Windows logon.
' Installed copy: %APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\scan3d_server.vbs  (delete it to disable autostart)
' serve_idle.sh exits at once if the server is already running.
CreateObject("WScript.Shell").Run "wsl.exe -d Ubuntu-22.04 -e bash -lc ""bash /home/system/scanner/StereoDepthSeg/scan3d/serve_idle.sh""", 0, False
