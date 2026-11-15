# 3D 스캔 시험 (맵 모드) 한 번에 켜기 — 2026-11-15
#   powershell -ExecutionPolicy Bypass -File C:\Users\a3162\scan3d_handoff\live_test\start_live.ps1            # 서버 + 노트북 확인 + 폰 녹화 시작 (30 s)
#   ... start_live.ps1 -Secs 45                # 녹화 길이 (10~120 s, 폰 화면 안내 후 자동 멈춤)
#   ... start_live.ps1 -Kbps 40000             # 영상 40 Mbps (무선 adb 로 가져오기 4배 빠름, 미검증)
#   ... start_live.ps1 -NoPhone                # 서버(페이지)만 켬. 폰 카메라는 안 켬
#   ... start_live.ps1 -Stop                   # 페이지의 STOP 버튼과 같음
# 페이지: http://localhost:8797  (창은 열지 않음 — 브라우저에서 직접 연다)
param([int]$Secs = 30, [int]$Kbps = 160000, [switch]$NoPhone, [switch]$Stop, [int]$Port = 8797)
$ErrorActionPreference = "Stop"
$Py = "C:\Users\a3162\AppData\Local\Programs\Python\Python312\python.exe"
$Server = "\\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg-livetest\scan3d\live_test\live_server.py"
$Base = "http://127.0.0.1:$Port"
$LogDir = "C:\Users\a3162\scan3d_live\logs"
New-Item -ItemType Directory -Force $LogDir | Out-Null

function Get-State { try { Invoke-RestMethod "$Base/api/state" -TimeoutSec 3 } catch { $null } }
function Post($path, $body) {
  $b = [System.Text.Encoding]::UTF8.GetBytes(($body | ConvertTo-Json -Compress))
  Invoke-RestMethod "$Base$path" -Method Post -Body $b -ContentType "application/json; charset=utf-8" -TimeoutSec 30
}

if ($Stop) { $r = Post "/api/stop" @{}; Write-Output "STOP: $($r.msg)"; exit 0 }

if (-not (Get-State)) {
  $ts = Get-Date -Format "yyyyMMdd_HHmmss"
  Start-Process -FilePath $Py -ArgumentList @("-u", "`"$Server`"", "--port", "$Port") -WindowStyle Hidden `
    -RedirectStandardOutput "$LogDir\stdout_$ts.txt" -RedirectStandardError "$LogDir\stderr_$ts.txt"
  for ($i = 0; $i -lt 30 -and -not (Get-State); $i++) { Start-Sleep -Milliseconds 500 }
  if (-not (Get-State)) { Write-Output "서버가 안 켜짐 — $LogDir\stderr_$ts.txt 확인"; exit 1 }
  Write-Output "서버 켜짐: http://localhost:$Port"
} else { Write-Output "서버 이미 켜져 있음: http://localhost:$Port" }

if ($NoPhone) { Write-Output "폰은 안 켬 (-NoPhone)"; exit 0 }

$r = Post "/api/start" @{ secs = $Secs; kbps = $Kbps }
Write-Output "시작: $($r.msg)  (노트북 확인 -> 폰 녹화 $Secs s, $([int]($Kbps/1000)) Mbps)"
# 준비 확인(노트북 연결·코드 보내기)이 끝나고 녹화가 시작될 때까지 따라가며 보여 줌 (최대 3 분)
for ($i = 0; $i -lt 180; $i++) {
  Start-Sleep -Seconds 1
  $s = Get-State
  if ($s -and $s.phase -ne "preflight" -and $s.phase -ne "starting") { break }
}
$s = Get-State
Write-Output "상태: $($s.phase) — $($s.msg)"
Write-Output "페이지: http://localhost:$Port  (STOP 버튼 = 폰 끄고 찍은 만큼 메시로)"
