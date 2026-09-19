$ErrorActionPreference = "Stop"

$appName = "DWM1001_UWB_Ground_Control"
$outputDirectory = Join-Path $PSScriptRoot "Output"
$buildDirectory = Join-Path $PSScriptRoot ".pyinstaller"
$entryPoint = Join-Path $PSScriptRoot "uwb_uart_gui.py"
$versionFile = Join-Path $PSScriptRoot "exe_version_info.txt"

New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
New-Item -ItemType Directory -Path $buildDirectory -Force | Out-Null

py -3.12 -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name $appName `
    --distpath $outputDirectory `
    --workpath (Join-Path $buildDirectory "work") `
    --specpath $buildDirectory `
    --version-file $versionFile `
    --hidden-import serial.tools.list_ports_windows `
    $entryPoint

if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller failed with exit code $LASTEXITCODE"
}

Copy-Item -LiteralPath (Join-Path $PSScriptRoot "HUONG_DAN_DEMO.txt") -Destination $outputDirectory -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot "CHAY_DEMO.cmd") -Destination $outputDirectory -Force

$executable = Join-Path $outputDirectory "$appName.exe"
Write-Host "Built: $executable"
Get-Item -LiteralPath $executable | Select-Object FullName, Length, LastWriteTime
