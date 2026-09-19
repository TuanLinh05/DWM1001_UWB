$ErrorActionPreference = 'Stop'

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$board = 'decawave_dwm1001_dev/nrf52832'
$drive = 'U:'
$driveRoot = 'U:\'
$createdMapping = $false

. (Join-Path $PSScriptRoot 'ncs_env.ps1')
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
        $projects = @('Tag') + @(1..8 | ForEach-Object { "Anchor_$_" })
        foreach ($project in $projects) {
            $sourceDir = Join-Path $driveRoot $project
            $buildDir = Join-Path $sourceDir 'build'

            Write-Host "Building $project..." -ForegroundColor Cyan
            & $west -z $env:ZEPHYR_BASE build --no-sysbuild --pristine=always `
                -b $board `
                -d $buildDir `
                $sourceDir

            if ($LASTEXITCODE -ne 0) {
                throw "Build failed: $project"
            }
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
