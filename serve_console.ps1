# serve_console.ps1 — serve the console over HTTP and open it.
#
#       .\serve_console.ps1
#
# The console MUST be served over HTTP. Opening the .html file directly
# (file://) blocks the browser from fetching vendor/three/, so the 3D stage
# never initialises and importing a GLB silently does nothing.

param([int]$Port = 8000)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

$vpy = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $vpy)) {
    Write-Host "No .venv found — run .\setup_windows.ps1 first." -ForegroundColor Red
    exit 1
}

$url = "http://localhost:$Port/Remote_control_Benchmark.html"
Write-Host "Serving $PSScriptRoot on port $Port" -ForegroundColor Cyan
Write-Host "  Console: $url"
Write-Host "  Keep this window open. Ctrl+C stops the server." -ForegroundColor DarkGray
Write-Host ""

Start-Process $url
& $vpy -m http.server $Port
