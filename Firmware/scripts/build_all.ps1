<#
.SYNOPSIS
    Build DWM1001/STM32 node projects with nRF Connect SDK v3.4.0.

.PARAMETER Projects
    Subset to build, e.g. -Projects Tag,Anchor_1. Default: the 11 nRF52832
    projects (Tag, Tag_DevKit, Sniffer_DevKit, Anchor_1..8). STM32_Anchor_5..8
    can be named explicitly too (or use build_stm32_anchors.ps1).
    Node names/paths: scripts\uwb_nodes.ps1.
#>

[CmdletBinding()]
param(
    [string[]]$Projects
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'uwb_nodes.ps1')
. (Join-Path $PSScriptRoot 'ncs_env.ps1')

$defaultProjects = @('Tag', 'Tag_DevKit', 'Sniffer_DevKit') + @(1..8 | ForEach-Object { "Anchor_$_" })
if ($null -eq $Projects -or $Projects.Count -eq 0) {
    $Projects = $defaultProjects
}
# 'powershell -File build_all.ps1 -Projects A,B' passes "A,B" as one string.
$Projects = @($Projects | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
foreach ($requested in $Projects) {
    Get-UwbNode $requested | Out-Null   # throws on unknown names
}

$ncs = Initialize-NcsEnvironment
$sizes = [ordered]@{}
Invoke-InUwbDrive {
    foreach ($project in $Projects) {
        $buildDir = Invoke-UwbWestBuild -Name $project -Ncs $ncs
        $binary = Join-Path $buildDir 'zephyr\zephyr.bin'
        if (Test-Path -LiteralPath $binary) {
            $sizes[$project] = (Get-Item -LiteralPath $binary).Length
        }
    }
}

Write-Host ""
Write-Host "built $($Projects.Count) project(s):" -ForegroundColor Green
foreach ($entry in $sizes.GetEnumerator()) {
    Write-Host ("  {0,-16} {1,8} B  {2}" -f $entry.Key, $entry.Value, (Get-UwbNode $entry.Key).Path)
}
