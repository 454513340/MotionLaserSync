# Windows PowerShell: run from the compiled MotionLaserSync.exe directory:
#   powershell -ExecutionPolicy Bypass -File .\RunGuiSmokeTest.ps1 -ExePath 'D:\path\MotionLaserSync.exe'
# This exercises GUI construction and opens the multi-pose dialog once.
# It does not connect to the motion controller or start scans.
param(
    [Parameter(Mandatory=$true)]
    [string]$ExePath
)
$ErrorActionPreference = 'Stop'
$ExePath = (Resolve-Path $ExePath).Path
$trace = Join-Path $env:TEMP 'MotionLaserSync_gui_smoke.txt'
if (Test-Path $trace) { Remove-Item $trace -Force }
Write-Host "Starting GUI smoke test: $ExePath --smoke-ui"
$p = Start-Process -FilePath $ExePath -ArgumentList '--smoke-ui' -PassThru -Wait
if (Test-Path $trace) {
    Get-Content -Path $trace
} else {
    Write-Host "FAIL: smoke trace was never created at $trace"
    exit 9
}
if ($p.ExitCode -ne 0 -or -not (Select-String -Path $trace -Pattern 'PASS: GUI smoke test complete' -Quiet)) {
    Write-Host "FAIL: Windows GUI smoke test, exit code $($p.ExitCode)"
    exit 10
}
Write-Host 'PASS: GUI startup and multi-pose dialog opening verified.'
exit 0
