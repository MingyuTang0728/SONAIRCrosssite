# start_sim_cell.ps1 -- the simulated cell: URSim (UR's controller software)
# in Docker, MuJoCo as the arm, served to the console as robot 127.0.0.2.
#
#       .\start_sim_cell.ps1
#
# Then start the agent with that address (.\start_agent.ps1 -UrIp 127.0.0.2)
# and serve the console as usual. Everything recorded is labelled simulated
# and kept in bench_runs\simcell, apart from the real runs.
#
# Once only: run install_sim.py first, and in URSim's own screen
# (http://localhost:6080/vnc.html) confirm the safety configuration when it
# asks -- until then URSim refuses the pendant speed slider.

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot
. "$PSScriptRoot\_pick_python.ps1"

Write-Host "SONAIR simulated cell" -ForegroundColor Cyan
Write-Host "  controller : URSim 5.11.11 (Docker, ports on 127.0.0.1)"
Write-Host "  arm        : MuJoCo (the benchmark's S0 model)"
Write-Host "  console    : robot address 127.0.0.2"
Write-Host "  URSim view : http://localhost:6080/vnc.html"
Write-Host ""
& $vpy sim_cell.py @args
