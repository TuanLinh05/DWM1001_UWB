$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$firmwareRoot = (Split-Path $projectRoot -Parent)
# The drive mapping below hides .git from the build; pass the real root for the build ID.
$env:UWB_REPO_ROOT = Split-Path $firmwareRoot -Parent
$board = 'decawave_dwm1001_dev/nrf52832'
$drive = 'U:'
$driveRoot = 'U:\'
# Join-Path validates that U: already exists.  Build the string before subst
# instead, because the mapping is intentionally created later in this script.
$mappedProject = $driveRoot + 'Tag'
$createdMapping = $false

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
        west build --no-sysbuild --pristine=always `
            -b $board `
            -d (Join-Path $mappedProject 'build') `
            $mappedProject
        if ($LASTEXITCODE -ne 0) {
            throw 'TAG build failed.'
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
