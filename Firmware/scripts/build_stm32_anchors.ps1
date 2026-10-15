<#
.SYNOPSIS
    Build the four STM32F103C8T6 DW1000 anchor images for ST-LINK Utility.

.DESCRIPTION
    Projects: nodes\anchors\stm32\STM32_Anchor_5..8.
    Output:   release\stm32_anchor\anchor_<N>_stm32f103.hex

.PARAMETER Anchors
    Anchor numbers to build. The default is 5, 6, 7, and 8.
#>

[CmdletBinding()]
param([ValidateSet(5, 6, 7, 8)][int[]]$Anchors = @(5, 6, 7, 8))

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'uwb_nodes.ps1')
. (Join-Path $PSScriptRoot 'ncs_env.ps1')

$firmwareRoot = $script:UwbFirmwareRoot
$distRoot = Join-Path $firmwareRoot 'release\stm32_anchor'
New-Item -ItemType Directory -Path $distRoot -Force | Out-Null

$ncs = Initialize-NcsEnvironment
Invoke-InUwbDrive {
    foreach ($number in $Anchors) {
        $buildDir = Invoke-UwbWestBuild -Name "STM32_Anchor_$number" -Ncs $ncs
        $hex = Join-Path $buildDir 'zephyr\zephyr.hex'
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
