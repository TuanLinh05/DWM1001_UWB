# Thin wrapper kept for the VS Code tasks in ..\.vscode. The build logic for
# every node lives in Firmware\scripts\node.ps1; node paths, boards and flash
# methods are registered in Firmware\scripts\uwb_nodes.ps1.
$ErrorActionPreference = 'Stop'
$nodeDir = Split-Path $PSScriptRoot -Parent
$firmwareRoot = $nodeDir
while (-not (Test-Path -LiteralPath (Join-Path $firmwareRoot 'scripts\uwb_nodes.ps1'))) {
    $firmwareRoot = Split-Path $firmwareRoot -Parent
    if (-not $firmwareRoot) { throw 'Firmware\scripts\uwb_nodes.ps1 was not found above this node.' }
}
& (Join-Path $firmwareRoot 'scripts\node.ps1') -Project (Split-Path $nodeDir -Leaf) -Action Build
