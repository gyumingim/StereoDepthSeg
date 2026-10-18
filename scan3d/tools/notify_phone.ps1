# 폰에 알림 보내기 (adb 로 연결된 폰 — USB 또는 무선). 앱 설치 없이 안드로이드 기본 'cmd notification post' 사용.
#   powershell -ExecutionPolicy Bypass -File notify_phone.ps1 "작업 끝: regress PASS"
#   powershell -ExecutionPolicy Bypass -File notify_phone.ps1 "본문" -Title "제목"
param(
  [Parameter(Mandatory = $true, Position = 0)][string]$Message,
  [string]$Title = "Claude 작업 알림",
  [string]$Adb = "C:\Users\a3162\platform-tools\adb.exe"
)
$tag = "claude_" + (Get-Date -Format "HHmmss")
# adb shell 은 인자를 한 번 더 셸로 해석하므로 작은따옴표로 감싸고 안의 작은따옴표는 바꿈
$t = "'" + ($Title -replace "'", "’") + "'"
$m = "'" + ($Message -replace "'", "’") + "'"
& $Adb shell cmd notification post -S bigtext -t $t $tag $m | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Output "알림 실패 (폰 연결 확인: adb devices)"; exit 1 }
Write-Output "알림 보냄: $Message"
