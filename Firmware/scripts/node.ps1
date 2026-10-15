<#
.SYNOPSIS
    Build or flash one firmware node by name.

.DESCRIPTION
    Every node's scripts\build.ps1 / scripts\flash.ps1 (used by the VS Code
    tasks in <node>\.vscode) forwards here, so the build/flash logic exists
    once. Node paths, boards and flash methods come from uwb_nodes.ps1.

.EXAMPLE
    .\scripts\node.ps1 -Project Tag_DevKit -Action Build
.EXAMPLE
    .\scripts\node.ps1 -Project Anchor_3 -Action Flash
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Project,
    [Parameter(Mandatory = $true)][ValidateSet('Build', 'Flash')][string]$Action
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'uwb_nodes.ps1')
. (Join-Path $PSScriptRoot 'ncs_env.ps1')

$node = Get-UwbNode $Project

if ($Action -eq 'Build') {
    $ncs = Initialize-NcsEnvironment
    Invoke-InUwbDrive { Invoke-UwbWestBuild -Name $node.Name -Ncs $ncs | Out-Null }
    Write-Host "Built $($node.Name): $(Join-Path $node.Dir 'build\zephyr\zephyr.hex')" -ForegroundColor Green
    return
}

# ---- Flash
$hexFile = Join-Path $node.Dir 'build\zephyr\zephyr.hex'
if (-not (Test-Path -LiteralPath $hexFile)) {
    throw "$($node.Name) has not been built. Run scripts\build.ps1 first."
}

switch ($node.Flash) {
    'openocd' {
        $openOcd = Find-OpenOcdEnvironment
        $openOcdConfig = Join-Path $node.Dir '.vscode\openocd-stlink.cfg'
        & $openOcd.Executable `
            -s $openOcd.Scripts `
            -f $openOcdConfig `
            -c "program {$hexFile} verify reset exit"
        if ($LASTEXITCODE -ne 0) {
            throw "ST-LINK flash failed: $($node.Name)"
        }
    }
    'jlink' {
        $ncs = Initialize-NcsEnvironment
        if ($null -eq $ncs.JLinkRoot) {
            throw 'SEGGER J-Link was not found. Set JLINK_ROOT to the J-Link installation directory.'
        }
        Invoke-InUwbDrive {
            & $ncs.West -z $env:ZEPHYR_BASE flash -d ($node.MappedDir + '\build') --runner jlink
            if ($LASTEXITCODE -ne 0) { throw "J-Link flash failed: $($node.Name)" }
        }
    }
    'west' {
        $ncs = Initialize-NcsEnvironment
        Invoke-InUwbDrive {
            & $ncs.West -z $env:ZEPHYR_BASE flash -d ($node.MappedDir + '\build')
            if ($LASTEXITCODE -ne 0) { throw "Flash failed: $($node.Name)" }
        }
    }
    'stlink-utility' {
        throw "$($node.Name) is programmed with ST-LINK Utility: open $hexFile (or the release HEX in release\)."
    }
}
