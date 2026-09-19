$ErrorActionPreference = 'Stop'
$firmwareRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path ([IO.Path]::GetTempPath()) 'uwb-firmware-host-tests'
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
$includeDirectory = Join-Path $firmwareRoot 'Tag\include'
foreach ($name in @('test_driver','test_tag_state')) {
    $sources = @((Join-Path $PSScriptRoot "$name.c"))
    if ($name -eq 'test_tag_state') {
        $sources += Join-Path $firmwareRoot 'Tag\src\drivers\dw1000.c'
        $sources += Join-Path $firmwareRoot 'Tag\src\filters\range_filter.c'
    }
    $binary = Join-Path $outputDirectory "$name.exe"
    & gcc -std=c11 -Wall -Wextra -Werror -I $includeDirectory @sources -o $binary
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
}

# Compile the production-only modules that are not linked into the two host
# executables above. This catches array-size/telemetry regressions without NCS.
& gcc -std=c11 -Wall -Wextra -Werror -I $includeDirectory `
    -c (Join-Path $firmwareRoot 'Tag\src\telemetry\telemetry.c') `
    -o (Join-Path $outputDirectory 'tag_telemetry.o')
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: Tag telemetry' }

$anchorInclude = Join-Path $firmwareRoot 'Anchor_1\include'
& gcc -std=c11 -Wall -Wextra -Werror -I $anchorInclude `
    -c (Join-Path $firmwareRoot 'Anchor_1\src\ranging\anchor_ranging.c') `
    -o (Join-Path $outputDirectory 'anchor_ranging.o')
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: Anchor responder' }

& (Join-Path $firmwareRoot 'ESP32C3_Gateway\tests\run_host_test.ps1')
if ($LASTEXITCODE -ne 0) { throw 'ESP32-C3 gateway parser validation failed.' }

$python = Get-Command py -ErrorAction SilentlyContinue
if ($null -ne $python) {
    & $python.Source -3 (Join-Path $PSScriptRoot 'test_anchor_projects.py')
} else {
    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $python) { throw 'Python was not found for anchor-project validation.' }
    & $python.Source (Join-Path $PSScriptRoot 'test_anchor_projects.py')
}
if ($LASTEXITCODE -ne 0) { throw 'Anchor-project validation failed.' }
