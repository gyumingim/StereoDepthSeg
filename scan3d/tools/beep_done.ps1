# 작업 끝 알림: PC 와 폰에서 최대 소리로 1 초짜리 비프 2번.
#   powershell -ExecutionPolicy Bypass -File beep_done.ps1 [-NoPc] [-NoPhone]
# 폰 소리 파일: /sdcard/Download/beep1s.wav (1200 Hz, 1 s) — 없으면 PC 의 같은 파일을 push.
param([switch]$NoPc, [switch]$NoPhone, [string]$Adb = "C:\Users\a3162\platform-tools\adb.exe")

$phoneOk = $false
if (-not $NoPhone) {
  $dev = (& $Adb devices) -join "`n"
  if ($dev -match "\S+\s+device(\s|$)") {
    $phoneOk = $true
    & $Adb shell "cmd media_session volume --stream 3 --set 15" 2>$null | Out-Null
  } else { Write-Output "폰 연결 안 됨 (adb devices) - 폰 비프 생략" }
}
if (-not $NoPc) {
  $sh = New-Object -ComObject WScript.Shell
  1..50 | ForEach-Object { $sh.SendKeys([char]175) }      # Windows 마스터 볼륨 최대
  Start-Sleep -Milliseconds 300
}

foreach ($i in 1..2) {
  if ($phoneOk) {
    & $Adb shell "am start -a android.intent.action.VIEW -d file:///sdcard/Download/beep1s.wav -t audio/wav" 2>$null | Out-Null
  }
  if (-not $NoPc) {
    try { [console]::Beep(1200, 1000) } catch { [System.Media.SystemSounds]::Exclamation.Play(); Start-Sleep -Seconds 1 }
  } else { Start-Sleep -Seconds 1 }
  Start-Sleep -Milliseconds 500
}
Write-Output "비프 완료"
