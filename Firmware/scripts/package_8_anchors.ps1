<#
.SYNOPSIS
    Collect the eight physical anchor images with a programmer/ID manifest.

.PARAMETER Build
    Rebuild A1-A4 (nRF52832) and A5-A8 (STM32F103) before packaging.
#>

[CmdletBinding()]
param([switch]$Build)

$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$dist = [IO.Path]::GetFullPath((Join-Path $firmwareRoot 'anchor_8_dist'))
$rootPrefix = [IO.Path]::GetFullPath($firmwareRoot).TrimEnd('\') + '\'
if (-not $dist.StartsWith($rootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Distribution directory escapes Firmware: '$dist'."
}

if ($Build) {
    & (Join-Path $PSScriptRoot 'build_all.ps1') -Projects Anchor_1,Anchor_2,Anchor_3,Anchor_4
    if ($LASTEXITCODE -ne 0) { throw 'Build failed for A1-A4.' }
    & (Join-Path $PSScriptRoot 'build_stm32_anchors.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Build failed for A5-A8.' }
}

$python = Get-Command python -ErrorAction Stop
& $python.Source (Join-Path $firmwareRoot 'tools\verify_stm32_images.py')
if ($LASTEXITCODE -ne 0) { throw 'STM32 image validation failed.' }

New-Item -ItemType Directory -Path $dist -Force | Out-Null
$manifest = @()
foreach ($number in 1..8) {
    if ($number -le 4) {
        $source = Join-Path $firmwareRoot "Anchor_$number\build\zephyr\zephyr.hex"
        $fileName = "A${number}_DWM1001C_nRF52832.hex"
        $mcu = 'nRF52832'
        $programmer = 'J-Link/OpenOCD'
    } else {
        $source = Join-Path $firmwareRoot "stm32_anchor\dist\anchor_${number}_stm32f103.hex"
        $fileName = "A${number}_STM32F103C8.hex"
        $mcu = 'STM32F103C8T6'
        $programmer = 'ST-LINK Utility'
    }
    if (-not (Test-Path -LiteralPath $source)) { throw "Missing A$number image: '$source'" }
    $destination = Join-Path $dist $fileName
    Copy-Item -LiteralPath $source -Destination $destination -Force
    $item = Get-Item -LiteralPath $destination
    $manifest += [pscustomobject]@{
        Anchor = "A$number"
        Address = ('0x{0:X4}' -f $number)
        MCU = $mcu
        Programmer = $programmer
        File = $fileName
        Bytes = $item.Length
        SHA256 = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
$manifest | Export-Csv -LiteralPath (Join-Path $dist 'manifest.csv') -NoTypeInformation -Encoding utf8
$manifest | Format-Table Anchor,MCU,Programmer,File,Bytes -AutoSize
Write-Host "Package: $dist" -ForegroundColor Green
