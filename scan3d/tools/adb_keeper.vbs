' Phone wireless adb keeper (scan3d/tools/adb_keeper.py), started hidden at Windows logon.
' Install (copy to Startup):
'   copy "\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\tools\adb_keeper.vbs" "%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\"
' Remove: delete that copy. Log: scan3d\runs\adb_keeper.log. A second copy exits at once (lock port 8797).
CreateObject("WScript.Shell").Run "pythonw.exe ""\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\tools\adb_keeper.py""", 0, False
