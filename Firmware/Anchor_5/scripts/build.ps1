$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$firmwareRoot = (Split-Path $projectRoot -Parent)
$projectName = Split-Path $projectRoot -Leaf
$board = 'decawave_dwm1001_dev/nrf52832'
$drive = 'U:'
$driveRoot = 'U:\'
$mappedProject = $driveRoot + $projectName
$createdMapping = $false

. (Join-Path $firmwareRoot 'scripts\ncs_env.ps1')
$ncsEnvironment = Initialize-NcsEnvironment
$west = $ncsEnvironment.West

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
        & $west -z $env:ZEPHYR_BASE build --no-sysbuild --pristine=always `
            -b $board `
            -d (Join-Path $mappedProject 'build') `
            $mappedProject
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed: $projectName"
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
