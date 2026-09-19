$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path $PSScriptRoot).Path
$entryPoint = Join-Path $projectRoot 'uwb_uart_gui.py'

py -3.12 -c 'import serial, tkinter'
if ($LASTEXITCODE -ne 0) {
    throw "Missing dependency. Run: py -3.12 -m pip install -r '$projectRoot\requirements.txt'"
}

py -3.12 $entryPoint

