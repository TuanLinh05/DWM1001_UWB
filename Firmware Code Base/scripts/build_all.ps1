$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$board = 'decawave_dwm1001_dev/nrf52832'
$variants = @(
    @{ Name = 'tag';           Config = 'config/tag.conf' },
    @{ Name = 'tag-ascii';     Config = 'config/tag_ascii.conf' },
    @{ Name = 'anchor-1';      Config = 'config/anchor_1.conf' },
    @{ Name = 'anchor-2';      Config = 'config/anchor_2.conf' },
    @{ Name = 'anchor-3';      Config = 'config/anchor_3.conf' },
    @{ Name = 'anchor-4';      Config = 'config/anchor_4.conf' },
    @{ Name = 'hardware-test'; Config = 'config/hardware_test.conf' }
)

Push-Location $projectRoot
try {
    foreach ($variant in $variants) {
        Write-Host "Building $($variant.Name)..." -ForegroundColor Cyan
        west build --no-sysbuild --pristine=always `
            -b $board `
            -d (Join-Path 'build' $variant.Name) `
            --extra-conf $variant.Config `
            .
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed: $($variant.Name)"
        }
    }
}
finally {
    Pop-Location
}
