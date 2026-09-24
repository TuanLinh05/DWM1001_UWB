$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$firmwareRoot = (Split-Path $projectRoot -Parent)
$drive = 'U:'
$driveRoot = 'U:\'
$mappedBuild = $driveRoot + 'Sniffer_DevKit\build'
$createdMapping = $false

if (-not (Test-Path -LiteralPath (Join-Path $projectRoot 'build\zephyr\zephyr.hex'))) {
    throw 'DWM1001-DEV TAG has not been built. Run scripts\build.ps1 first.'
}
. (Join-Path $PSScriptRoot 'ncs_env.ps1')
$ncsEnvironment = Initialize-NcsEnvironment
$west = $ncsEnvironment.West
if ($null -eq $ncsEnvironment.JLinkRoot) {
    throw 'SEGGER J-Link was not found. Set JLINK_ROOT to the J-Link installation directory.'
}

$existingTarget = $null
foreach ($line in @(subst.exe)) {
    if ($line -match '^U:\\:\s*=>\s*(.+)$') {
        $existingTarget = $Matches[1].Trim()
        break
    }
}

if ($null -ne $existingTarget) {
    if (-not $existingTarget.Equals($firmwareRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Drive U: is already mapped to '$existingTarget'."
    }
} elseif (Test-Path -LiteralPath $driveRoot) {
    throw 'Drive U: is already in use.'
} else {
    subst.exe $drive $firmwareRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot map U: to '$firmwareRoot'."
    }
    $createdMapping = $true
}

try {
    Push-Location $driveRoot
    try {
        & $west -z $env:ZEPHYR_BASE flash -d $mappedBuild --runner jlink
        if ($LASTEXITCODE -ne 0) {
            throw 'DWM1001-DEV TAG flash failed.'
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    if ($createdMapping) {
        subst.exe $drive /d
    }
}
