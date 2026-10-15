<#
.SYNOPSIS
    Single registry of every firmware node + shared build/flash helpers.

.DESCRIPTION
    Dot-source this file from any script under Firmware/:
        . (Join-Path <Firmware>\scripts 'uwb_nodes.ps1')

    Every script that needs a node directory asks Get-UwbNode instead of
    hard-coding a path, so moving a node only means editing $UwbNodes below.

    Zephyr's Kconfig cannot handle the space in "Drone Project", so builds run
    from a drive letter (U:) mapped to Firmware/ with subst. Invoke-InUwbDrive
    owns that mapping for every script.
#>

$ErrorActionPreference = 'Stop'

$script:UwbFirmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$script:UwbRepoRoot = Split-Path $script:UwbFirmwareRoot -Parent
$script:UwbDrive = 'U:'
$script:UwbDriveRoot = 'U:\'

$script:UwbNrfBoard = 'decawave_dwm1001_dev/nrf52832'
$script:UwbStm32Board = 'stm32_min_dev@blue/stm32f103xb'

# Name -> Path (relative to Firmware/), Role, Board, Flash method.
#   Flash: 'jlink'   west flash --runner jlink (DWM1001-DEV on-board J-Link)
#          'west'    west flash with the board's default runner
#          'openocd' OpenOCD + ST-LINK using <node>\.vscode\openocd-stlink.cfg
#          'stlink-utility'  HEX is programmed by hand with ST-LINK Utility
$script:UwbNodes = [ordered]@{
    'Tag'            = @{ Path = 'nodes\tag\Tag';                Role = 'TAG';     Board = $script:UwbNrfBoard; Flash = 'west' }
    'Tag_DevKit'     = @{ Path = 'nodes\tag\Tag_DevKit';         Role = 'TAG';     Board = $script:UwbNrfBoard; Flash = 'jlink' }
    'Sniffer_DevKit' = @{ Path = 'nodes\sniffer\Sniffer_DevKit'; Role = 'SNIFFER'; Board = $script:UwbNrfBoard; Flash = 'jlink' }
}
foreach ($n in 1..8) {
    $script:UwbNodes["Anchor_$n"] = @{
        Path = "nodes\anchors\nrf\Anchor_$n"; Role = 'ANCHOR'; Board = $script:UwbNrfBoard; Flash = 'openocd' }
}
foreach ($n in 5..8) {
    $script:UwbNodes["STM32_Anchor_$n"] = @{
        Path = "nodes\anchors\stm32\STM32_Anchor_$n"; Role = 'ANCHOR'; Board = $script:UwbStm32Board; Flash = 'stlink-utility' }
}

function Get-UwbNodeNames {
    return @($script:UwbNodes.Keys)
}

function Get-UwbNode {
    <# Returns Name, Path (relative), Dir (absolute), MappedDir (on U:), Role, Board, Flash, IsStm32. #>
    param([Parameter(Mandatory = $true)][string]$Name)

    if (-not $script:UwbNodes.Contains($Name)) {
        throw "Unknown node '$Name'. Known: $((Get-UwbNodeNames) -join ', ')"
    }
    $entry = $script:UwbNodes[$Name]
    return [pscustomobject]@{
        Name = $Name
        Path = $entry.Path
        Dir = Join-Path $script:UwbFirmwareRoot $entry.Path
        # Plain string: Join-Path would fail before U: is mapped.
        MappedDir = $script:UwbDriveRoot + $entry.Path
        Role = $entry.Role
        Board = $entry.Board
        Flash = $entry.Flash
        IsStm32 = $entry.Board -eq $script:UwbStm32Board
    }
}

function Find-UwbNodeByDirectory {
    <# Maps a node directory (e.g. the parent of a node's scripts/) to its name. #>
    param([Parameter(Mandatory = $true)][string]$Directory)

    $full = [IO.Path]::GetFullPath($Directory).TrimEnd('\')
    foreach ($name in Get-UwbNodeNames) {
        $node = Get-UwbNode $name
        if ([IO.Path]::GetFullPath($node.Dir).TrimEnd('\') -ieq $full) { return $node }
    }
    throw "'$Directory' is not a registered node directory (see scripts\uwb_nodes.ps1)."
}

function Invoke-InUwbDrive {
    <# Maps U: to Firmware/ (unless already mapped there), runs $Body from U:\, unmaps. #>
    param([Parameter(Mandatory = $true)][scriptblock]$Body)

    # The drive mapping hides .git from the build; pass the real root for the build ID.
    $env:UWB_REPO_ROOT = $script:UwbRepoRoot

    $existingTarget = $null
    foreach ($line in @(subst.exe)) {
        if ($line -match '^U:\\:\s*=>\s*(.+)$') {
            $existingTarget = $Matches[1].Trim()
            break
        }
    }

    $createdMapping = $false
    if ($null -ne $existingTarget) {
        if (-not $existingTarget.Equals($script:UwbFirmwareRoot, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Drive U: is already mapped to '$existingTarget'."
        }
    } elseif (Test-Path -LiteralPath $script:UwbDriveRoot) {
        throw 'Drive U: is already in use.'
    } else {
        subst.exe $script:UwbDrive $script:UwbFirmwareRoot
        if ($LASTEXITCODE -ne 0) {
            throw "Cannot map U: to '$($script:UwbFirmwareRoot)'."
        }
        $createdMapping = $true
    }

    try {
        Push-Location $script:UwbDriveRoot
        try {
            & $Body
        }
        finally {
            Pop-Location
        }
    }
    finally {
        if ($createdMapping) {
            subst.exe $script:UwbDrive /d
        }
    }
}

function Invoke-UwbWestBuild {
    <#
    Builds one node with west. Must run inside Invoke-InUwbDrive.
    $Ncs is the object returned by Initialize-NcsEnvironment.
    #>
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)]$Ncs,
        [string]$BuildDirName = 'build',
        [string[]]$CMakeArgs = @()
    )

    $node = Get-UwbNode $Name
    if (-not (Test-Path -LiteralPath (Join-Path $node.Dir 'CMakeLists.txt'))) {
        throw "Missing project: '$($node.Dir)'."
    }

    $priorModules = $env:ZEPHYR_EXTRA_MODULES
    try {
        if ($node.IsStm32) {
            # NCS's manifest omits STM32 support; explicitly add the installed module.
            $sdkHal = Join-Path $Ncs.SdkRoot 'modules\hal\stm32'
            if (-not (Test-Path -LiteralPath (Join-Path $sdkHal 'zephyr\module.yml'))) {
                throw "Missing hal_stm32 at '$sdkHal' (Zephyr commit 39130f29ae37c1db34095478ca02b6419b70dcdc)."
            }
            $env:ZEPHYR_EXTRA_MODULES = $sdkHal
        }

        $westArgs = @('-z', $env:ZEPHYR_BASE, 'build', '--no-sysbuild', '--pristine=always',
                      '-b', $node.Board, '-d', ($node.MappedDir + '\' + $BuildDirName), $node.MappedDir)
        if ($CMakeArgs.Count -gt 0) { $westArgs += @('--') + $CMakeArgs }

        Write-Host "Building $Name ($($node.Path), $($node.Board))..." -ForegroundColor Cyan
        # Out-Host: west's output must not become part of the return value.
        & $Ncs.West @westArgs | Out-Host
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed: $Name"
        }
    }
    finally {
        $env:ZEPHYR_EXTRA_MODULES = $priorModules
    }

    return Join-Path $node.Dir $BuildDirName
}
