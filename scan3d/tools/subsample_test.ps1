param(
  [string]$Name = "t",
  [int]$Fps = 15,              # camera (capture) fps
  [int]$SaveFps = 0,           # 0 = old mode (save every frame); e.g. -Fps 60 -SaveFps 15
  [int]$Seconds = 60,
  [string]$Mode = "both",      # both = recorder + shared live encoder (no PC client needed)
  [string]$Out = "",           # result folder (default: scan3d/runs/subsample_<Name>)
  [switch]$Keep                # keep the test recording on the phone (default: delete it after pulling the small files)
)
# Phone app 1.8-subsample measurement (SPEC.md 1.2.3). Records $Seconds s at 1440x1080 lenses 2,5 with the app started
# directly by adb, saves metrics.json every 5 s, then pulls meta.json + frames CSVs + imu.csv (small) and the mp4 sizes.
# The scanner server must be idle (no app use). Server auto-pull is paused during the test and switched back on after.
# Summary: python3 scan3d/tools/subsample_ana.py <folder>
$ErrorActionPreference = "Continue"
$A = "C:\Users\a3162\platform-tools\adb.exe"
$repo = "\\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg"
if (-not $Out) { $Out = Join-Path $repo "scan3d\runs\subsample_$Name" }
New-Item -ItemType Directory -Force $Out | Out-Null
$ext = "/sdcard/Android/data/com.camera.dualstream/files"
$st0 = try { (Invoke-RestMethod -Uri "http://localhost:8795/api/state" -TimeoutSec 10).session.mode } catch { "" }
if ($st0 -and $st0.Trim() -ne "idle") { Write-Output "server not idle ($st0) - stop"; exit 1 }
try { Invoke-RestMethod -Uri "http://localhost:8795/api/cmd" -Method Post -ContentType "application/json" -Body (@{cmd="rec_auto"; pull=$false} | ConvertTo-Json) -TimeoutSec 10 | Out-Null } catch { }
try {
  & $A shell am force-stop com.camera.dualstream | Out-Null
  Start-Sleep 1
  $extra = @()
  if ($SaveFps -gt 0) { $extra = @("--ei", "save_fps", "$SaveFps") }
  & $A shell am start -S -n com.camera.dualstream/.MainActivity --es logical 0 --es physical 2,5 --ei width 1440 --ei height 1080 --ei fps $Fps @extra --ef focus_diopters 1.0 --ez raw_ts false --ei imu_us 2500 --es mode $Mode --es stream_fmt h264 --es sync off --ei rec_kbps 14000 | Out-Null
  $t0 = Get-Date
  $i = 0
  while (((Get-Date) - $t0).TotalSeconds -lt $Seconds) {
    Start-Sleep 5
    $i++
    $m = & $A shell cat "$ext/metrics.json" | Out-String
    Set-Content -Path (Join-Path $Out ("metrics_{0:D3}.json" -f $i)) -Value $m -Encoding utf8
  }
  & $A shell am broadcast -a com.camera.dualstream.STOP | Out-Null
  $st = ""
  for ($k = 0; $k -lt 40; $k++) {
    Start-Sleep -Milliseconds 500
    $st = & $A shell cat "$ext/status.txt" | Out-String
    if ($st -match "^RECORD_DONE") { break }
  }
  Set-Content -Path (Join-Path $Out "status.txt") -Value $st -Encoding utf8
  $rec = ([regex]::Match($st, "rec=(rec_\S+)")).Groups[1].Value
  Start-Sleep 2
  & $A shell am force-stop com.camera.dualstream | Out-Null
  if ($rec) {
    $d = "$ext/$rec"
    & $A shell ls -l $d | Set-Content -Path (Join-Path $Out "ls.txt") -Encoding utf8
    foreach ($f in @("meta.json", "frames_cam2.csv", "frames_cam5.csv", "imu.csv")) { & $A pull "$d/$f" (Join-Path $Out $f) | Out-Null }
    if (-not $Keep -and $rec -match "^rec_\d{8}_\d{6}$") { & $A shell rm -r $d | Out-Null; "deleted test recording $rec on phone" }
  }
  "done $Name rec=$rec -> $Out"
} finally {
  try { Invoke-RestMethod -Uri "http://localhost:8795/api/cmd" -Method Post -ContentType "application/json" -Body (@{cmd="rec_auto"; pull=$true} | ConvertTo-Json) -TimeoutSec 10 | Out-Null } catch { }
}
