<#
.SYNOPSIS
    Build every DWM1001 node project with nRF Connect SDK v3.4.0.

.PARAMETER Projects
    Subset to build, e.g. -Projects Tag,Anchor_1. Default: all 11 projects.
#>

[CmdletBinding()]
param(
    [string[]]$Projects
)

$ErrorActionPreference = 'Stop'

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
# The drive mapping below hides .git from the build; pass the real root for the build ID.
$env:UWB_REPO_ROOT = Split-Path $firmwareRoot -Parent
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
        $allProjects = @('Tag', 'Tag_DevKit', 'Sniffer_DevKit') +
                       @(1..8 | ForEach-Object { "Anchor_$_" })
        if ($null -eq $Projects -or $Projects.Count -eq 0) {
            $projects = $allProjects
        } else {
            foreach ($requested in $Projects) {
                if ($allProjects -notcontains $requested) {
                    throw "Unknown project '$requested'. Known: $($allProjects -join ', ')"
                }
            }
            $projects = $Projects
        }
        $sizes = [ordered]@{}
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

            $binary = Join-Path $buildDir 'zephyr\zephyr.bin'
            if (Test-Path -LiteralPath $binary) {
                $sizes[$project] = (Get-Item -LiteralPath $binary).Length
            }
        }

        Write-Host ""
        Write-Host "built $($projects.Count) project(s):" -ForegroundColor Green
        foreach ($entry in $sizes.GetEnumerator()) {
            Write-Host ("  {0,-16} {1,8} B" -f $entry.Key, $entry.Value)
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
