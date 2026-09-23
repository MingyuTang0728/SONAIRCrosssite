# _pick_python.ps1 — find the interpreter to run this project with.
#
# Dot-source it:   . "$PSScriptRoot\_pick_python.ps1"
# It sets $vpy, or exits with a message that says what to do.
#
# A virtual environment is preferred when one exists, because it isolates
# this project. It is NOT required: a system Python with the packages
# installed works exactly as well, and demanding a .venv turned a working
# machine into a blocked one with a message that named the wrong problem.

$vpy = $null
$venv = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (Test-Path $venv) {
    $vpy = $venv
    Write-Host "Using the project's virtual environment." -ForegroundColor DarkGray
} else {
    foreach ($candidate in @("python", "py")) {
        $found = Get-Command $candidate -ErrorAction SilentlyContinue
        if ($found) {
            # `py` is a launcher, not an interpreter: ask it for the real path
            # so every message that quotes it can be pasted and run.
            $vpy = if ($candidate -eq "py") {
                (& py -c "import sys; print(sys.executable)" 2>$null)
            } else { $found.Source }
            if ($vpy) { break }
        }
    }
    if ($vpy) {
        Write-Host "Using $vpy" -ForegroundColor DarkGray
    }
}

if (-not $vpy) {
    Write-Host "No Python found." -ForegroundColor Red
    Write-Host "  Install Python 3.11 or 3.12 from python.org and tick"
    Write-Host "  'Add python.exe to PATH', then run this again."
    exit 1
}

# The agent needs websockets before it can start at all. Say so here rather
# than letting it fail with an import traceback.
& $vpy -c "import websockets" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "This Python is missing the project's packages." -ForegroundColor Red
    Write-Host "  Run:  $vpy install_deps.py"
    exit 1
}
