# setup_windows.ps1 — one-time setup on the workstation wired to the UR5e.
#
#   Right-click > Run with PowerShell, or from a PowerShell prompt:
#       cd C:\SONAIR
#       .\setup_windows.ps1
#
# It finds a usable Python, creates a virtual environment in .venv, installs
# the dependencies, and runs the pre-flight check.
#
# If PowerShell refuses to run this file ("running scripts is disabled"):
#       Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
# That lasts only for the current window, which is the smallest change that works.

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

Write-Host "SONAIR setup" -ForegroundColor Cyan
Write-Host "Folder: $PSScriptRoot"
Write-Host ""

# --- warn about cloud-synced folders before doing any work -------------------
if ($PSScriptRoot -match "OneDrive|Dropbox|Google Drive") {
    Write-Host "WARNING: this folder is inside a cloud-synced drive." -ForegroundColor Yellow
    Write-Host "  Recording writes a JSON line 125 times a second. A sync client will"
    Write-Host "  compete for the file handle and can lock a run mid-capture."
    Write-Host "  Move the project to a plain local path such as C:\SONAIR first."
    Write-Host ""
    $go = Read-Host "Continue anyway? (y/N)"
    if ($go -ne "y") { Write-Host "Stopped."; exit 1 }
}

# --- find a Python ------------------------------------------------------------
# The `py` launcher is installed by python.org and is the reliable way to pick a
# version on Windows. 3.11 is preferred because pyrealsense2 publishes wheels
# for it; newer versions often have none, and the camera install then fails
# while everything else would have worked.
$pyExe  = $null
$pyArgs = @()
foreach ($cand in @("3.11", "3.10", "3.9", "3.12", "3.13")) {
    try {
        & py "-$cand" --version *> $null
        if ($LASTEXITCODE -eq 0) { $pyExe = "py"; $pyArgs = @("-$cand"); break }
    } catch { }
}
if (-not $pyExe) {
    try {
        $v = & python --version 2>&1
        # A bare `python` on Windows 11 with no Python installed is a Microsoft
        # Store stub: it prints a "not found" notice and exits 9009. Treat that
        # as no Python, or the venv step fails with a far more confusing error.
        if ($LASTEXITCODE -eq 0 -and "$v" -notmatch "was not found|Microsoft Store") {
            $pyExe = "python"; $pyArgs = @()
        }
    } catch { }
}

if (-not $pyExe) {
    Write-Host "No Python found." -ForegroundColor Red
    Write-Host ""
    Write-Host "  1. Download Python 3.11 from https://www.python.org/downloads/windows/"
    Write-Host "  2. In the installer, TICK 'Add python.exe to PATH' on the first screen."
    Write-Host "  3. Close this window, open a NEW PowerShell, and run this script again."
    Write-Host ""
    Write-Host "  Do not use the Microsoft Store stub: typing 'python' opens the Store"
    Write-Host "  instead of running anything, which is what 'pip is not recognized' means."
    exit 1
}
Write-Host ("Using: " + (@($pyExe) + $pyArgs -join " ")) -ForegroundColor Green
& $pyExe @pyArgs --version

# --- virtual environment ------------------------------------------------------
if (-not (Test-Path ".venv")) {
    Write-Host "`nCreating virtual environment in .venv ..."
    & $pyExe @pyArgs -m venv .venv
} else {
    Write-Host "`n.venv already exists — reusing it."
}
$vpy = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $vpy)) { Write-Host "venv creation failed." -ForegroundColor Red; exit 1 }

# --- dependencies -------------------------------------------------------------
Write-Host "`nInstalling dependencies ..."
& $vpy -m pip install --upgrade pip --quiet
& $vpy -m pip install websockets numpy opencv-python

# pyrealsense2 last and on its own: it is the one most likely to have no wheel
# for the installed Python, and a failure here must not stop the rest.
Write-Host "`nInstalling pyrealsense2 (camera) ..."
& $vpy -m pip install pyrealsense2
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "pyrealsense2 did not install." -ForegroundColor Yellow
    Write-Host "  Almost always this means there is no wheel for this Python version."
    Write-Host "  Everything except the camera still works. To get the camera, install"
    Write-Host "  Python 3.11, delete the .venv folder, and run this script again."
}

# --- pre-flight ---------------------------------------------------------------
Write-Host "`nRunning the pre-flight check ...`n"
& $vpy check_setup.py

Write-Host ""
Write-Host "Setup finished." -ForegroundColor Cyan
Write-Host "Next:  .\start_agent.ps1        (in one PowerShell window)"
Write-Host "       .\serve_console.ps1      (in a second window)"
