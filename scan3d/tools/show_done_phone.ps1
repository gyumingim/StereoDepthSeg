# 작업 끝: 폰 화면을 깨우고 '작업 완료' 전체 화면 페이지를 띄움 (+ 알림 한 줄). 소리 없음.
#   powershell -ExecutionPolicy Bypass -File show_done_phone.ps1 "한 줄 결과"
param([Parameter(Position = 0)][string]$Message = "", [string]$Adb = "C:\Users\a3162\platform-tools\adb.exe")
$dev = (& $Adb devices) -join "`n"
if ($dev -notmatch "\S+\s+device(\s|$)") { Write-Output "폰 연결 안 됨 (adb devices)"; exit 1 }
$esc = [System.Net.WebUtility]::HtmlEncode($Message)
$now = Get-Date -Format "HH:mm"
$html = @"
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>작업 완료</title></head>
<body style="margin:0;background:#0a7d3b;color:#fff;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100vh;text-align:center">
<div style="padding:24px"><div style="font-size:80px">&#10004;</div><div style="font-size:34px;font-weight:bold">작업 완료</div>
<div style="font-size:20px;margin-top:18px;line-height:1.5">$esc</div><div style="font-size:16px;margin-top:24px;opacity:.8">$now</div></div></body></html>
"@
$tmp = Join-Path $env:TEMP "claude_done.html"
[IO.File]::WriteAllText($tmp, $html, (New-Object System.Text.UTF8Encoding $false))
& $Adb push $tmp /sdcard/Download/claude_done.html 2>$null | Out-Null
& $Adb shell input keyevent KEYCODE_WAKEUP | Out-Null
& $Adb shell "am start -a android.intent.action.VIEW -d file:///sdcard/Download/claude_done.html -t text/html" 2>$null | Out-Null
if ($Message) {
  $m = "'" + ($Message -replace "'", "’") + "'"
  & $Adb shell cmd notification post -S bigtext -t "'작업 완료'" ("claude_" + (Get-Date -Format HHmmss)) $m 2>$null | Out-Null
}
Write-Output "폰 화면 띄움: $Message"
