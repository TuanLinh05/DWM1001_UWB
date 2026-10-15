<#
.SYNOPSIS
Flash a packaged highrate HEX without rebuilding or using the normal build folder.
.EXAMPLE
./flash_highrate.ps1 -Node TAG
.EXAMPLE
./flash_highrate.ps1 -Node A1 -DryRun
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('TAG', 'A1', 'A2', 'A3', 'A4')]
    [string]$Node,
    [switch]$DryRun,
    [string]$JLinkRoot,
    [ValidatePattern('^[0-9]+$')]
    [string]$JLinkSerial,
    [string]$OpenOcdPath,
    [string]$OpenOcdScripts
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'uwb_nodes.ps1')
$firmwareRoot = $script:UwbFirmwareRoot
$packageRoot = Join-Path $firmwareRoot 'release\highrate_200'
$Node = $Node.ToUpperInvariant()
$fileName = if ($Node -eq 'TAG') { 'TAG_DWM1001_DEV.hex' } else { "${Node}_DWM1001C_nRF52832.hex" }
$hexPath = (Resolve-Path -LiteralPath (Join-Path $packageRoot $fileName)).Path
$manifest = @(Import-Csv -LiteralPath (Join-Path $packageRoot 'manifest.csv') |
    Where-Object { $_.File -eq $fileName })
if ($manifest.Count -ne 1) { throw "Missing or duplicate manifest entry: $fileName" }
$hash = (Get-FileHash -LiteralPath $hexPath -Algorithm SHA256).Hash
if ($hash -ne $manifest[0].SHA256) { throw "SHA256 mismatch: $hexPath. Rebuild the package before flashing." }

Write-Host "Node: $Node"
Write-Host "HEX:  $hexPath"
Write-Host "SHA256: $hash (matches manifest)"
Write-Host 'Connect only the intended target to the selected programmer.'

if ($Node -eq 'TAG') {
    . (Join-Path $PSScriptRoot 'ncs_env.ps1')
    $resolvedJLinkRoot = Find-JLinkRoot -RequestedRoot $JLinkRoot
    if (-not $resolvedJLinkRoot) { throw 'JLink.exe not found. Specify -JLinkRoot or set JLINK_ROOT.' }
    $executable = Join-Path $resolvedJLinkRoot 'JLink.exe'
    # The application uses the Cortex-M DWT cycle counter for deadlines.
    # J-Link normally clears DEMCR.TRCENA on close, which stops that counter
    # after the application has started. Keep its debug clock enabled.
    $commands = @('ExitOnError 1', 'exec SetDbgPowerDownOnClose = 0',
        'r', 'h', ('loadfile "{0}"' -f $hexPath), 'r', 'g', 'q')
    $arguments = @('-Device', 'nRF52832_xxAA', '-If', 'SWD', '-Speed', '4000',
        '-AutoConnect', '1', '-ExitOnError', '1', '-NoGui', '1')
    if ($JLinkSerial) { $arguments += @('-USB', $JLinkSerial) }
    Write-Host "Programmer: $executable"
    if ($DryRun) {
        Write-Host ('Arguments: ' + ($arguments -join ' ') + ' -CommandFile <temporary command file>')
        $commands | ForEach-Object { Write-Host $_ }
        Write-Host 'DRY RUN: no programmer was started and no target was accessed.'
        return
    }
    $commandFile = [System.IO.Path]::GetTempFileName()
    try {
        Set-Content -LiteralPath $commandFile -Value $commands -Encoding ASCII
        & $executable @arguments -CommandFile $commandFile
        if ($LASTEXITCODE -ne 0) { throw "J-Link failed with exit code $LASTEXITCODE. Check the output above." }
    } finally {
        Remove-Item -LiteralPath $commandFile -ErrorAction SilentlyContinue
    }
} else {
    . (Join-Path $PSScriptRoot 'ncs_env.ps1')
    $openOcd = Find-OpenOcdEnvironment -RequestedExecutable $OpenOcdPath -RequestedScripts $OpenOcdScripts
    $anchorNumber = $Node.Substring(1)
    $config = (Resolve-Path -LiteralPath (Join-Path (Get-UwbNode "Anchor_${anchorNumber}").Dir '.vscode\openocd-stlink.cfg')).Path
    $tclHexPath = $hexPath.Replace('\', '/')
    if ($tclHexPath -match '[{}\r\n]') { throw 'HEX path contains unsupported Tcl characters.' }
    $programCommand = "program {$tclHexPath} verify reset exit"
    Write-Host "Programmer: $($openOcd.Executable)"
    Write-Host "Config: $config"
    Write-Host "Command: $programCommand"
    if ($DryRun) {
        Write-Host 'DRY RUN: no programmer was started and no target was accessed.'
        return
    }
    & $openOcd.Executable -s $openOcd.Scripts -f $config -c $programCommand
    if ($LASTEXITCODE -ne 0) { throw "OpenOCD failed with exit code $LASTEXITCODE. Check the output above." }
}
Write-Host "Flash programming completed for $Node; runtime operation has not been checked."
Write-Host 'Check LEDs, firmware identity and ranging in the next log.'
