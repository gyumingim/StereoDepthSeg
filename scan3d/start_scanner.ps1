# 3D 스캐너 켜기/끄기 (Windows). 평소에는 필요 없음 — 로그인 때 시작프로그램의 scan3d_server.vbs 가 대기 서버를 켜 두고,
# 브라우저에서 http://localhost:8795 → [스캔 시작] 하면 중계·폰 앱·VIO 를 웹 서버가 켠다.
# 이 스크립트는 대기 서버가 안 떠 있을 때 손으로 켜거나(start_scanner.bat), 전부 끌 때(stop_scanner.bat) 쓴다.
#   powershell -ExecutionPolicy Bypass -File start_scanner.ps1          # 서버 확인·시작 → 브라우저 열기
#   powershell -ExecutionPolicy Bypass -File start_scanner.ps1 -Stop    # 녹화 정상 정지 → 중계·스캐너 종료
param([switch]$Stop, [switch]$NoBrowser)

$ErrorActionPreference = 'Continue'
$Distro = 'Ubuntu-22.04'
$Repo = '/home/system/scanner/StereoDepthSeg'
$Url = 'http://localhost:8795'
$LogDir = Join-Path $env:LOCALAPPDATA 'scan3d'
New-Item -ItemType Directory -Force $LogDir | Out-Null
$Log = Join-Path $LogDir 'start_scanner.log'
function Say($m) { $line = (Get-Date -Format 'HH:mm:ss ') + $m; Write-Host $line; Add-Content -Path $Log -Value $line -Encoding UTF8 }

function Find-Adb {
  # 고정 위치를 맨 앞에 (Temp 폴더는 정리되면 사라짐, REVIEW4 B5)
  $c = @('C:\Users\a3162\platform-tools\adb.exe',
         $env:SCAN3D_ADB,
         (Join-Path $env:USERPROFILE 'platform-tools\adb.exe'),
         (Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe'))
  $w = Get-Command adb.exe -ErrorAction SilentlyContinue
  if ($w) { $c += $w.Source }
  foreach ($p in $c) { if ($p -and (Test-Path $p)) { return $p } }
  return $null
}

function Server-Up {
  try { Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 "$Url/api/state" | Out-Null; return $true } catch { return $false }
}

function Post-Cmd($body) {
  try { Invoke-RestMethod -Method Post -Uri "$Url/api/cmd" -ContentType 'application/json' -Body ($body | ConvertTo-Json -Compress) -TimeoutSec 30 }
  catch { $null }
}

if ($Stop) {
  Say '끄는 중…'
  if (Server-Up) {
    Post-Cmd @{ cmd = 'stop'; stop_app = $true } | Out-Null      # 녹화 중이면 STOP 브로드캐스트로 mp4 마무리
    for ($i = 0; $i -lt 20; $i++) {                              # 앱 정지(최대 약 10 s) 기다림
      try { $s = Invoke-RestMethod -TimeoutSec 2 "$Url/api/state" } catch { break }
      if ($s.session.mode -eq 'idle' -and ($null -eq $s.session.app -or $s.session.app.state -ne '정지 중')) { break }
      Start-Sleep -Milliseconds 500
    }
  }
  Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -ne $PID -and $_.Name -notmatch '^(powershell|pwsh)' -and $_.CommandLine -like '*win_relay.py*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue; Say "중계 종료 pid=$($_.ProcessId)" }
  # [s]·[o] 대괄호: 패턴이 이 bash 자신의 명령줄과는 안 맞게 (자기 부모 bash 를 죽이지 않게, REVIEW4 B8)
  wsl.exe -d $Distro -e bash -lc "pkill -TERM -f 'scan3d/[s]can3d.py' ; sleep 2 ; pkill -TERM -f 'ov_bridge/build/[o]v_bridge' ; true"
  $adb = Find-Adb
  if ($adb) { & $adb shell am force-stop com.camera.dualstream 2>$null | Out-Null }
  Say '꺼졌습니다. 다시 쓰려면 start_scanner.bat (또는 Windows 다시 로그인).'
  exit 0
}

# ---- 켜기 ----
$adb = Find-Adb
if (-not $adb) {
  Say 'adb 를 찾지 못했습니다. SCAN3D_ADB 환경변수에 adb.exe 경로를 넣거나 Android platform-tools 를 설치하세요.'
} else {
  Say "adb: $adb"
  $dev = (& $adb devices) | Select-Object -Skip 1 | Where-Object { $_ -match '\tdevice$' }
  if (-not $dev) { Say '폰을 USB 로 연결하고 폰 화면의 "USB 디버깅 허용" 을 눌러 주세요. (웹 화면에도 안내가 나옵니다)' }
}

if (Server-Up) {
  Say '스캐너 서버가 이미 켜져 있습니다.'
} else {
  Say '스캐너 서버 켜는 중… (WSL, 창 없음)'
  # PowerShell 5.1 Start-Process 는 배열 원소를 따옴표 없이 이어 붙인다 → 공백 없는 원소만 넘긴다 (REVIEW4 B1)
  Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d', $Distro, '-e', 'bash', '-l', "$Repo/scan3d/serve_idle.sh") -WindowStyle Hidden
  $ok = $false
  for ($i = 0; $i -lt 120; $i++) { if (Server-Up) { $ok = $true; break }; Start-Sleep -Milliseconds 500 }
  if (-not $ok) { Say "서버가 60 초 안에 안 떴습니다. 로그: \\wsl.localhost\$Distro$($Repo -replace '/', '\')\scan3d\runs\server_*.log"; exit 1 }
  Say '서버 켜짐.'
}
if (-not $NoBrowser) { Start-Process $Url }
