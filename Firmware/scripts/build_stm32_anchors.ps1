<#
.SYNOPSIS
    Build the four STM32F103C8T6 DW1000 anchor images for ST-LINK Utility.

.PARAMETER Anchors
    Anchor numbers to build. The default is 5, 6, 7, and 8.
#>

[CmdletBinding()]
param([ValidateSet(5, 6, 7, 8)][int[]]$Anchors = @(5, 6, 7, 8))

$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$env:UWB_REPO_ROOT = Split-Path $firmwareRoot -Parent
$board = 'stm32_min_dev@blue/stm32f103xb'
$drive = 'U:'
$driveRoot = 'U:\'
$createdMapping = $false
$priorExtraModules = $env:ZEPHYR_EXTRA_MODULES

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
        throw "U: is already mapped to '$existingTarget'."
    }
} elseif (Test-Path -LiteralPath $driveRoot) {
    throw 'U: is already in use.'
} else {
    subst.exe $drive $firmwareRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot map U: to '$firmwareRoot'."
    }
    $createdMapping = $true
}

try {
    $rootPrefix = [IO.Path]::GetFullPath($firmwareRoot).TrimEnd('\') + '\'
    $sdkHal = Join-Path $ncsEnvironment.SdkRoot 'modules\hal\stm32'
    if (-not (Test-Path -LiteralPath (Join-Path $sdkHal 'zephyr\module.yml'))) {
        throw "Missing hal_stm32 at '$sdkHal' (Zephyr commit 39130f29ae37c1db34095478ca02b6419b70dcdc)."
    }
    # NCS's manifest omits STM32 support; explicitly add the installed module.
    $env:ZEPHYR_EXTRA_MODULES = $sdkHal
    $distRoot = [IO.Path]::GetFullPath((Join-Path $firmwareRoot 'stm32_anchor\dist'))
    if (-not $distRoot.StartsWith($rootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Distribution directory escapes Firmware: '$distRoot'."
    }
    New-Item -ItemType Directory -Path $distRoot -Force | Out-Null
    Push-Location $driveRoot
    try {
    foreach ($number in $Anchors) {
        $project = "STM32_Anchor_$number"
        $physicalProject = Join-Path $firmwareRoot $project
        $physicalBuild = [IO.Path]::GetFullPath((Join-Path $physicalProject 'build'))
        if (-not $physicalBuild.StartsWith($rootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Build directory escapes Firmware: '$physicalBuild'."
        }
        if (-not (Test-Path -LiteralPath (Join-Path $physicalProject 'CMakeLists.txt'))) {
            throw "Missing project: '$physicalProject'."
        }

        $sourceDir = Join-Path $driveRoot $project
        $buildDir = Join-Path $sourceDir 'build'
        Write-Host "Building A$number ($board)..." -ForegroundColor Cyan
        & $west -z $env:ZEPHYR_BASE build --no-sysbuild --pristine=always `
            -b $board -d $buildDir $sourceDir
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed: A$number"
        }

        $hex = Join-Path $physicalBuild 'zephyr\zephyr.hex'
        if (-not (Test-Path -LiteralPath $hex)) {
            throw "Build produced no HEX: '$hex'."
        }
        $distHex = Join-Path $distRoot "anchor_${number}_stm32f103.hex"
        Copy-Item -LiteralPath $hex -Destination $distHex -Force
        $item = Get-Item -LiteralPath $distHex
        $hash = (Get-FileHash -LiteralPath $distHex -Algorithm SHA256).Hash
        Write-Host ("A{0}: {1} bytes, SHA256 {2}" -f $number, $item.Length, $hash) -ForegroundColor Green
        Write-Host "    $distHex"
    }
    }
    finally {
        Pop-Location
    }
}
finally {
    $env:ZEPHYR_EXTRA_MODULES = $priorExtraModules
    if ($createdMapping) {
        subst.exe $drive /d
    }
}
