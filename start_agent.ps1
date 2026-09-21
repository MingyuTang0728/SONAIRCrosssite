# start_agent.ps1 — run the host agent that talks to the robot and the sensors.
#
#       .\start_agent.ps1
#       .\start_agent.ps1 -UrIp 192.168.1.50
#
# Leave this window open while you work. Everything the browser shows comes
# through this process.

param(
    [string]$UrIp = "192.168.0.20",
    [int]$FusionHubPort = 5005,
    [string]$RunDir = ""
)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

$vpy = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $vpy)) {
    Write-Host "No .venv found — run .\setup_windows.ps1 first." -ForegroundColor Red
    exit 1
}

$env:UR_IP = $UrIp
$env:BENCH_FUSIONHUB_PORT = "$FusionHubPort"
if ($RunDir) { $env:BENCH_RUN_DIR = $RunDir }

Write-Host "SONAIR host agent" -ForegroundColor Cyan
Write-Host "  UR controller : $UrIp"
Write-Host "  FusionHub UDP : $FusionHubPort"
Write-Host "  Runs saved to : $(if ($RunDir) { $RunDir } else { Join-Path $PSScriptRoot 'bench_runs' })"
Write-Host ""
Write-Host "  Keep this window open. Ctrl+C stops the agent." -ForegroundColor DarkGray
Write-Host ""

& $vpy multimodal_bridge.py
