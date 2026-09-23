# serve_console.ps1 — serve the console over HTTP and open it.
#
#       .\serve_console.ps1
#
# The console MUST be served over HTTP. Opened as a file (file://) the browser
# refuses the cross-origin request for ur5e.glb, so the cell view sits on
# "Loading the robot model" for ever. Measured, not assumed: the request is
# blocked with ERR_FAILED and the model loads in 68 ms over HTTP.

param([int]$Port = 8000)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

. "$PSScriptRoot\_pick_python.ps1"

$url = "http://localhost:$Port/SONAIR_Console.html"
Write-Host "Serving $PSScriptRoot on port $Port" -ForegroundColor Cyan
Write-Host "  Console: $url"
Write-Host "  Keep this window open. Ctrl+C stops the server." -ForegroundColor DarkGray
Write-Host ""

Start-Process $url
& $vpy -m http.server $Port
