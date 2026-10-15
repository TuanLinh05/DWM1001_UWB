<#
.SYNOPSIS
    Build the >=200 Hz candidate package (222 Hz nominal), with strict DS-TWR.
.DESCRIPTION
    Output: release\highrate_200\ (nine HEX + manifest.csv). Each node builds
    into <node>\build_highrate so the normal build\ folder stays untouched.
.PARAMETER Projects
    Defaults to all nine nodes. A subset is kept in its staging directory;
    it never replaces the complete release package.
#>
[CmdletBinding()]
param([ValidateSet('Tag_DevKit','Anchor_1','Anchor_2','Anchor_3','Anchor_4',
                  'STM32_Anchor_5','STM32_Anchor_6','STM32_Anchor_7','STM32_Anchor_8')]
      [string[]]$Projects = @('Tag_DevKit','Anchor_1','Anchor_2','Anchor_3','Anchor_4',
                            'STM32_Anchor_5','STM32_Anchor_6','STM32_Anchor_7','STM32_Anchor_8'),
      [ValidateSet(200, 300)]
      [int]$Rate = 200)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'uwb_nodes.ps1')
. (Join-Path $PSScriptRoot 'ncs_env.ps1')

$firmwareRoot = $script:UwbFirmwareRoot
$dist = Join-Path $firmwareRoot "release\highrate_$Rate"
$buildDirName = if ($Rate -eq 300) { 'build_highrate_300' } else { 'build_highrate' }
$stage = Join-Path $dist ('build_staging_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
$completePackage = ($Projects.Count -eq 9 -and @($Projects | Select-Object -Unique).Count -eq 9)
$manifestRows = @()
$ncs = Initialize-NcsEnvironment

New-Item -ItemType Directory -Path $dist -Force | Out-Null
New-Item -ItemType Directory -Path $stage -Force | Out-Null

Invoke-InUwbDrive {
    foreach ($project in $Projects) {
        $node = Get-UwbNode $project
        $options = @('-DUWB_HIGH_RATE=ON')
        if ($Rate -eq 300) {
            $options += '-DUWB_HIGH_RATE_300=ON'
        }
        if ($node.IsStm32) {
            $name = 'A' + ($project -replace '^STM32_Anchor_', '') + '_STM32F103C8.hex'
            $options += '-DEXTRA_DTC_OVERLAY_FILE=U:/profiles/highrate_stm32.overlay'
            $options += '-DEXTRA_CONF_FILE=U:/profiles/highrate_common.conf'
        } else {
            $options += '-DEXTRA_DTC_OVERLAY_FILE=U:/profiles/highrate_nrf.overlay'
            $options += '-DEXTRA_CONF_FILE=U:/profiles/highrate_common.conf;U:/profiles/highrate_nrf.conf'
            $name = if ($project -eq 'Tag_DevKit') { 'TAG_DWM1001_DEV.hex' }
                    else { 'A' + ($project -replace '^Anchor_', '') + '_DWM1001C_nRF52832.hex' }
        }
        $build = Invoke-UwbWestBuild -Name $project -Ncs $ncs -BuildDirName $buildDirName -CMakeArgs $options
        Copy-Item -LiteralPath (Join-Path $build 'zephyr\zephyr.hex') `
                  -Destination (Join-Path $stage $name)
        $ninjaText = Get-Content -LiteralPath (Join-Path $build 'build.ninja') -Raw
        if ($ninjaText -notmatch 'UWB_BUILD_CONFIG_HASH=(0x[0-9a-fA-F]{8})') {
            throw "Build identity missing: $project"
        }
        $configId = $Matches[1]
        $image = Get-Item -LiteralPath (Join-Path $stage $name)
        $script:manifestRows += [pscustomobject]@{
            File = $name
            Bytes = $image.Length
            SHA256 = (Get-FileHash -LiteralPath $image.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
            ConfigID = $configId
            Board = $node.Board
            SPIHz = if ($node.IsStm32) { 16000000 } else { 8000000 }
        }
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
