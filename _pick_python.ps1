# _pick_python.ps1 — find the interpreter to run this project with.
#
# Dot-source it, then stop if nothing was found:
#     . "$PSScriptRoot\_pick_python.ps1"
#     if (-not $vpy) { exit 1 }
# It sets $vpy, or leaves it empty after a message that says what to do.
# (An `exit` in here would NOT stop the caller: a dot-sourced script's exit
# only ends itself, so the caller went on to "Serving ..." with no Python.)
#
# A virtual environment is preferred when one exists, because it isolates
# this project. It is NOT required: a system Python with the packages
# installed works exactly as well.
#
# Every Python on the machine is considered, and the first one that already
# has the project's packages wins. This matters on Windows: `python` on PATH
# is often the Microsoft Store alias in ...\WindowsApps\, which either opens
# the Store or is a second, empty Python. Picking it made the agent and the
# web server exit at once, and the browser then said "connection refused" --
# which looks like a firewall problem and is not one.

# Probing an interpreter that is missing a package writes to stderr; under
# the caller's "Stop" preference Windows PowerShell 5.1 would turn that into
# a terminating error. Probe leniently, restore the caller's setting after.
$_sonairEap = $ErrorActionPreference
$ErrorActionPreference = "Continue"

function _SonairRealPath([string]$exe, [string[]]$pre = @()) {
    try {
        # No `Select-Object -First` here: it stops the pipeline early, which
        # kills the interpreter and leaves a failed exit code behind.
        $out = @(& $exe @pre -c "import sys; print(sys.executable)" 2>$null)
        $p = if ($out.Count) { "$($out[-1])".Trim() } else { "" }
        if ($LASTEXITCODE -eq 0 -and $p -and (Test-Path -LiteralPath $p)) { return $p }
    } catch { }
    return $null
}

$candidates = New-Object System.Collections.Generic.List[string]
function _SonairAdd([string]$p) {
    if ($p -and -not $candidates.Contains($p)) { $candidates.Add($p) }
}

$venv = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (Test-Path -LiteralPath $venv) { _SonairAdd $venv }

# The py launcher, asked for the versions this project is tested on first.
if (Get-Command py -ErrorAction SilentlyContinue) {
    foreach ($v in @("-3.12", "-3.11", "-3")) { _SonairAdd (_SonairRealPath "py" @($v)) }
}
# The usual python.org install folders, whether or not they are on PATH.
foreach ($root in @("$env:LOCALAPPDATA\Programs\Python", "$env:ProgramFiles\Python",
                    "C:\Program Files\Python")) {
    if ($root -and (Test-Path -LiteralPath $root)) {
        Get-ChildItem -LiteralPath $root -Directory -Filter "Python3*" -ErrorAction SilentlyContinue |
            Sort-Object Name -Descending |
            ForEach-Object { _SonairAdd (Join-Path $_.FullName "python.exe") }
    }
}
# Whatever `python` is on PATH, last, and only if it is a real interpreter.
foreach ($c in (Get-Command python, python3 -All -ErrorAction SilentlyContinue)) {
    _SonairAdd (_SonairRealPath $c.Source)
}

$real = @($candidates | Where-Object { (Test-Path -LiteralPath $_) -and ($_ -notmatch '\\WindowsApps\\') })
$stub = @($candidates | Where-Object { $_ -match '\\WindowsApps\\' })

$vpy = $null
# A Store Python that really has the packages is still fine, as a last resort.
foreach ($p in @($real + $stub)) {
    try {
        & $p -c "import websockets" 2>$null
        if ($LASTEXITCODE -eq 0) { $vpy = $p; break }
    } catch { }
}

$ErrorActionPreference = $_sonairEap

if ($vpy) {
    Write-Host "Using $vpy" -ForegroundColor DarkGray
} elseif ($real.Count -gt 0) {
    Write-Host "Found Python, but none of them has the project's packages yet:" -ForegroundColor Red
    $real | ForEach-Object { Write-Host "    $_" }
    Write-Host "  Run this once, then start this script again:"
    Write-Host "    & `"$($real[0])`" install_deps.py" -ForegroundColor Yellow
} else {
    Write-Host "No usable Python found." -ForegroundColor Red
    if ($stub.Count -gt 0) {
        Write-Host "  'python' here is only the Microsoft Store alias ($($stub[0]))."
    }
    Write-Host "  Install Python 3.12 from python.org (tick 'Add python.exe to PATH'),"
    Write-Host "  then run:  py -3.12 install_deps.py"
}
