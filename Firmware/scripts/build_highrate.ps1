<#
.SYNOPSIS
    Build the >=200 Hz candidate package (222 Hz nominal), with strict DS-TWR.
.PARAMETER Projects
    Defaults to all nine nodes. A subset is kept in its staging directory;
    it never replaces the complete release package.
#>
[CmdletBinding()]
param([ValidateSet('Tag_DevKit','Anchor_1','Anchor_2','Anchor_3','Anchor_4',
                  'STM32_Anchor_5','STM32_Anchor_6','STM32_Anchor_7','STM32_Anchor_8')]
      [string[]]$Projects = @('Tag_DevKit','Anchor_1','Anchor_2','Anchor_3','Anchor_4',
                            'STM32_Anchor_5','STM32_Anchor_6','STM32_Anchor_7','STM32_Anchor_8'))

$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$env:UWB_REPO_ROOT = Split-Path $firmwareRoot -Parent
$rootPrefix = [IO.Path]::GetFullPath($firmwareRoot).TrimEnd('\') + '\'
$dist = Join-Path $firmwareRoot 'highrate_200_dist'
$stage = Join-Path $dist ('build_staging_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
$completePackage = ($Projects.Count -eq 9 -and @($Projects | Select-Object -Unique).Count -eq 9)
$manifestRows = @()
$driveRoot = 'U:\'
$createdMapping = $false
$priorModules = $env:ZEPHYR_EXTRA_MODULES
. (Join-Path $PSScriptRoot 'ncs_env.ps1')
$ncs = Initialize-NcsEnvironment

$existingTarget = $null
foreach ($line in @(subst.exe)) {
    if ($line -match '^U:\\:\s*=>\s*(.+)$') { $existingTarget = $Matches[1].Trim(); break }
}
if ($null -ne $existingTarget) {
    if (-not $existingTarget.Equals($firmwareRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw "U: is mapped to '$existingTarget'."
    }
} elseif (Test-Path -LiteralPath $driveRoot) { throw 'U: is already in use.' }
else {
    subst.exe U: $firmwareRoot
    if ($LASTEXITCODE -ne 0) { throw 'Cannot map U: to Firmware.' }
    $createdMapping = $true
}

try {
    New-Item -ItemType Directory -Path $dist -Force | Out-Null
    New-Item -ItemType Directory -Path $stage -Force | Out-Null
    Push-Location $driveRoot
    try {
        foreach ($project in $Projects) {
            $build = [IO.Path]::GetFullPath((Join-Path $firmwareRoot "$project\build_highrate"))
            if (-not $build.StartsWith($rootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
                throw "Build directory escapes Firmware: '$build'."
            }
            $options = @('-DUWB_HIGH_RATE=ON')
            if ($project.StartsWith('STM32_')) {
                $board = 'stm32_min_dev@blue/stm32f103xb'
                $env:ZEPHYR_EXTRA_MODULES = Join-Path $ncs.SdkRoot 'modules\hal\stm32'
                if (-not (Test-Path -LiteralPath (Join-Path $env:ZEPHYR_EXTRA_MODULES 'zephyr\module.yml'))) {
                    throw 'Installed hal_stm32 module is missing.'
                }
                $name = 'A' + ($project -replace '^STM32_Anchor_', '') + '_STM32F103C8.hex'
                $options += '-DEXTRA_DTC_OVERLAY_FILE=U:/profiles/highrate_stm32.overlay'
                $options += '-DEXTRA_CONF_FILE=U:/profiles/highrate_common.conf'
            } else {
                $board = 'decawave_dwm1001_dev/nrf52832'
                $env:ZEPHYR_EXTRA_MODULES = $priorModules
                $options += '-DEXTRA_DTC_OVERLAY_FILE=U:/profiles/highrate_nrf.overlay'
                $options += '-DEXTRA_CONF_FILE=U:/profiles/highrate_common.conf;U:/profiles/highrate_nrf.conf'
                $name = if ($project -eq 'Tag_DevKit') { 'TAG_DWM1001_DEV.hex' }
                        else { 'A' + ($project -replace '^Anchor_', '') + '_DWM1001C_nRF52832.hex' }
            }
            Write-Host "Building high-rate $project..." -ForegroundColor Cyan
            & $ncs.West -z $env:ZEPHYR_BASE build --no-sysbuild --pristine=always `
                -b $board -d ($driveRoot + "$project\build_highrate") `
                ($driveRoot + $project) -- @options
            if ($LASTEXITCODE -ne 0) { throw "High-rate build failed: $project" }
            Copy-Item -LiteralPath (Join-Path $build 'zephyr\zephyr.hex') `
                      -Destination (Join-Path $stage $name)
            $ninjaText = Get-Content -LiteralPath (Join-Path $build 'build.ninja') -Raw
            if ($ninjaText -notmatch 'UWB_BUILD_CONFIG_HASH=(0x[0-9a-fA-F]{8})') {
                throw "Build identity missing: $project"
            }
            $configId = $Matches[1]
            $image = Get-Item -LiteralPath (Join-Path $stage $name)
            $manifestRows += [pscustomobject]@{
                File = $name
                Bytes = $image.Length
                SHA256 = (Get-FileHash -LiteralPath $image.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
                ConfigID = $configId
                Board = $board
                SPIHz = if ($project.StartsWith('STM32_')) { 16000000 } else { 8000000 }
            }
        }
        $manifestRows | Sort-Object File | Export-Csv -LiteralPath (Join-Path $stage 'manifest.csv') `
            -NoTypeInformation -Encoding utf8
        if ($completePackage) {
            # Publish only after all nine builds succeed. A failed/subset build
            # cannot silently mix old/new radio backends in the release.
            foreach ($row in $manifestRows) {
                Copy-Item -LiteralPath (Join-Path $stage $row.File) -Destination (Join-Path $dist $row.File) -Force
                if ((Get-FileHash -LiteralPath (Join-Path $dist $row.File) -Algorithm SHA256).Hash `
                    -ne $row.SHA256) { throw "Package copy hash mismatch: $($row.File)" }
            }
            Copy-Item -LiteralPath (Join-Path $stage 'manifest.csv') `
                -Destination (Join-Path $dist 'manifest.csv') -Force
            Write-Host "Complete candidate package: $dist" -ForegroundColor Green
        } else {
            Write-Host "Subset build (release unchanged): $stage" -ForegroundColor Yellow
        }
    } finally { Pop-Location }
} finally {
    $env:ZEPHYR_EXTRA_MODULES = $priorModules
    if ($createdMapping) { subst.exe U: /d }
}
