# scan3d 대기 서버 다시 켜기 (Windows PowerShell 5.1).
#   powershell -ExecutionPolicy Bypass -File \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\tools\restart_server.ps1
#   WSL 쪽에서는 bash scan3d/tools/restart_server.sh (이 파일을 부름)
# 하는 일: WSL 의 scan3d.py·ov_bridge 종료 (+ Windows 중계 win_relay.py) → 포트 8795 해제 기다림
#          → 시작프로그램의 scan3d_server.vbs 실행 → /api/state 가 응답할 때까지 최대 60 s → 결과 출력.
#          한 번에 안 뜨면 한 번 더 시도. 성공 0, 실패 1.
# 주의: 다른 사람이(에이전트가) 스캔·복원 시험 중이면 끊긴다. -Force 없이는 실시간·복원 중일 때 거절.
param(
  [string]$Vbs = '',            # 비우면 시작프로그램 → 저장소 순서로 찾음 (WSL 에서 부를 때는 .sh 가 넘김)
  [int]$TimeoutSec = 60,
  [int]$Tries = 2,
  [switch]$Force,               # 스캔·복원 중이어도 끔
  [switch]$KeepRelay            # Windows 중계(win_relay.py)는 그대로 둠
)
$ErrorActionPreference = 'Continue'
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch {}
$Distro = 'Ubuntu-22.04'
$Repo = '/home/system/scanner/StereoDepthSeg'
$Url = 'http://localhost:8795'
$Port = 8795
# WSL interop 로 불리면 환경 변수가 거의 없다 (SystemRoot·APPDATA 없음) → 고정 경로 먼저
$Wsl = 'C:\Windows\System32\wsl.exe'
if (-not (Test-Path $Wsl)) { $c = Get-Command wsl.exe -ErrorAction SilentlyContinue; if ($c) { $Wsl = $c.Source } }
$Wscript = 'C:\Windows\System32\wscript.exe'

function Say($m) { Write-Host ((Get-Date -Format 'HH:mm:ss ') + $m) }

function Get-State {
  try { return Invoke-RestMethod -UseBasicParsing -TimeoutSec 3 "$Url/api/state" } catch { return $null }
}

function Find-Vbs {
  if ($Vbs -and (Test-Path $Vbs)) { return $Vbs }
  $c = @()
  if ($env:APPDATA) { $c += (Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\Startup\scan3d_server.vbs') }
  $c += 'C:\Users\a3162\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Startup\scan3d_server.vbs'
  $c += "\\wsl.localhost\$Distro" + ($Repo -replace '/', '\') + '\scan3d\scan3d_server.vbs'
  foreach ($p in $c) { if ($p -and (Test-Path $p)) { return $p } }
  return $null
}

function Stop-Server {
  if (-not $KeepRelay) {
    # 자기 자신·PowerShell 은 빼고 (명령줄 패턴이 이 PowerShell 과도 맞을 수 있음)
    Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -ne $PID -and $_.Name -notmatch '^(powershell|pwsh)' -and $_.CommandLine -like '*win_relay.py*' } |
      ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue; Say "중계 종료 pid=$($_.ProcessId)" }
  }
  # [s]·[o] 대괄호: 패턴이 이 bash 자신의 명령줄과 맞지 않게. TERM → 10 s 기다림 → KILL. 그 뒤 포트 해제 최대 15 s.
  $sh = "pkill -TERM -f 'scan3d/[s]can3d.py'; pkill -TERM -f 'ov_bridge/build/[o]v_bridge'; " +
        "for i in `$(seq 20); do pgrep -f 'scan3d/[s]can3d.py' >/dev/null || break; sleep 0.5; done; " +
        "pkill -KILL -f 'scan3d/[s]can3d.py'; pkill -KILL -f 'ov_bridge/build/[o]v_bridge'; " +
        "for i in `$(seq 30); do ss -ltn 'sport = :$Port' | grep -q LISTEN || { echo PORT_FREE; exit 0; }; sleep 0.5; done; echo PORT_BUSY"
  $r = & $Wsl -d $Distro -e bash -lc $sh
  return ($r -join ' ')
}

function Wait-Up {
  $t0 = Get-Date
  while (((Get-Date) - $t0).TotalSeconds -lt $TimeoutSec) {
    $s = Get-State
    if ($s) { return @{ s = $s; sec = [int]((Get-Date) - $t0).TotalSeconds } }
    Start-Sleep -Seconds 1
  }
  return $null
}

$st = Get-State
if ($st -and -not $Force) {
  $mode = $st.session.mode
  if ($mode -and $mode -ne 'idle') {
    Say "거절: 서버가 지금 '$mode' 중 (스캔·복원). 그래도 끄려면 -Force"
    exit 1
  }
}
$vbsPath = Find-Vbs
if (-not $vbsPath) { Say '실패: scan3d_server.vbs 를 찾지 못함 (시작프로그램·저장소 scan3d/)'; exit 1 }

for ($try = 1; $try -le $Tries; $try++) {
  Say "[$try/$Tries] 서버 종료 중..."
  $r = Stop-Server
  if ($r -notmatch 'PORT_FREE') { Say "주의: 포트 $Port 가 아직 사용 중 ($r)" }
  Say "시작: $vbsPath"
  & $Wscript //nologo "$vbsPath"
  $up = Wait-Up
  if ($up) {
    $s = $up.s
    Say ("OK: 서버 응답 ({0} s)  mode={1} link={2}  adb={3}" -f $up.sec, $s.session.mode, $s.session.link, $s.session.adb.kind)
    Say "웹: $Url   로그: $Repo/scan3d/runs/server_$(Get-Date -Format yyyyMMdd).log"
    exit 0
  }
  Say "응답 없음 ($TimeoutSec s). 로그 끝부분:"
  & $Wsl -d $Distro -e bash -lc "tail -n 15 $Repo/scan3d/runs/server_`$(date +%Y%m%d).log 2>/dev/null"
}
Say '실패: 서버가 뜨지 않음. 위 로그를 보거나 손으로: wsl -d Ubuntu-22.04 -e bash -lc "bash ~/scanner/StereoDepthSeg/scan3d/serve_idle.sh"'
exit 1
