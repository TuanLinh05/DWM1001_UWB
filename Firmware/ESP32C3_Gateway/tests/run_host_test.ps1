$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$parserSource = Join-Path $projectRoot 'main\telemetry_protocol.c'
$parserInclude = Join-Path $projectRoot 'main'
$testSource = Join-Path $PSScriptRoot 'test_telemetry_protocol.c'
$testOutput = Join-Path ([IO.Path]::GetTempPath()) 'uwb-firmware-host-tests'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
$testBinary = Join-Path $testOutput 'test_telemetry_protocol.exe'

$gcc = Get-Command gcc -ErrorAction Stop
& $gcc.Source -std=c11 -Wall -Wextra -Werror `
    -I $parserInclude $parserSource $testSource -o $testBinary
if ($LASTEXITCODE -ne 0) {
    throw "Host parser test build failed ($LASTEXITCODE)."
}

& $testBinary
if ($LASTEXITCODE -ne 0) {
    throw "Host parser test failed ($LASTEXITCODE)."
}
