$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$firmwareRoot = (Split-Path $projectRoot -Parent)
$drive = 'U:'
$driveRoot = 'U:\'
$mappedBuild = $driveRoot + 'Tag\build'
$createdMapping = $false

if (-not (Test-Path -LiteralPath (Join-Path $projectRoot 'build\zephyr\zephyr.hex'))) {
    throw 'TAG has not been built. Run scripts\build.ps1 first.'
}
if (-not (Get-Command west -ErrorAction SilentlyContinue)) {
    throw 'west was not found. Load ncs_env.ps1 before running this script.'
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
        west flash -d $mappedBuild
        if ($LASTEXITCODE -ne 0) {
            throw 'TAG flash failed.'
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
