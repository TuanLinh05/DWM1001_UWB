$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$firmwareRoot = (Split-Path $projectRoot -Parent)
$projectName = Split-Path $projectRoot -Leaf
$hexFile = Join-Path $projectRoot 'build\zephyr\zephyr.hex'
$openOcdConfig = Join-Path $projectRoot '.vscode\openocd-stlink.cfg'

if (-not (Test-Path -LiteralPath $hexFile)) {
    throw "$projectName has not been built. Run scripts\build.ps1 first."
}

. (Join-Path $firmwareRoot 'scripts\ncs_env.ps1')
$openOcd = Find-OpenOcdEnvironment

& $openOcd.Executable `
    -s $openOcd.Scripts `
    -f $openOcdConfig `
    -c "program {$hexFile} verify reset exit"
if ($LASTEXITCODE -ne 0) {
    throw "ST-LINK flash failed: $projectName"
}
