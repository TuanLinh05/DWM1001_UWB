# Launch UWB Fast Studio (PyQt6 High-Speed 2D/3D Positioning & Calibration GUI)
param(
    [string]$Port = "",
    [int]$Baud = 460800,
    [switch]$Demo
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ArgsList = @("$ScriptDir\main_gui.py", "--baud", "$Baud")
if ($Port -ne "") {
    $ArgsList += @("--port", $Port)
}
if ($Demo) {
    $ArgsList += "--demo"
}

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3.12 @ArgsList
} else {
    & python @ArgsList
}
