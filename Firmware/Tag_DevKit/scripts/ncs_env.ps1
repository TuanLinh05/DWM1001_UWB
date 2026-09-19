function Find-NcsSdkRoot {
    param([string]$RequestedRoot)

    $candidates = @(
        $RequestedRoot,
        $env:NCS_SDK_ROOT,
        'E:\software\nordic\ncs\v3.4.0',
        'C:\ncs\v3.4.0',
        (Join-Path $env:USERPROFILE 'ncs\v3.4.0')
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

    foreach ($candidate in $candidates) {
        $zephyrBase = Join-Path $candidate 'zephyr'
        if ((Test-Path -LiteralPath (Join-Path $zephyrBase 'scripts\west-commands.yml')) -and
            (Test-Path -LiteralPath (Join-Path $candidate '.west\config'))) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw 'nRF Connect SDK v3.4.0 was not found. Set NCS_SDK_ROOT to its installation directory.'
}

function Find-NcsToolchainRoot {
    param(
        [string]$SdkRoot,
        [string]$RequestedRoot
    )

    $candidates = @(@($RequestedRoot, $env:NCS_TOOLCHAIN_ROOT) |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) })

    $ncsInstallRoot = Split-Path $SdkRoot -Parent
    $toolchainsRoot = Join-Path $ncsInstallRoot 'toolchains'
    if (Test-Path -LiteralPath $toolchainsRoot) {
        $candidates += Get-ChildItem -LiteralPath $toolchainsRoot -Directory |
            Sort-Object LastWriteTime -Descending |
            ForEach-Object { $_.FullName }
    }

    foreach ($candidate in $candidates) {
        if ((Test-Path -LiteralPath (Join-Path $candidate 'environment.json')) -and
            (Test-Path -LiteralPath (Join-Path $candidate 'opt\bin\Scripts\west.exe'))) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw 'The nRF Connect SDK toolchain was not found. Set NCS_TOOLCHAIN_ROOT to the toolchain bundle directory.'
}

function Find-JLinkRoot {
    param([string]$RequestedRoot)

    $candidates = @(@($RequestedRoot, $env:JLINK_ROOT, 'E:\software\SEGGER\JLink_V924a') |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) })

    foreach ($base in @('E:\software\SEGGER', 'C:\Program Files\SEGGER', 'C:\Program Files (x86)\SEGGER')) {
        if (Test-Path -LiteralPath $base) {
            $candidates += Get-ChildItem -LiteralPath $base -Directory -Filter 'JLink*' |
                Sort-Object LastWriteTime -Descending |
                ForEach-Object { $_.FullName }
        }
    }

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath (Join-Path $candidate 'JLink.exe')) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    return $null
}

function Initialize-NcsEnvironment {
    param(
        [string]$SdkRoot,
        [string]$ToolchainRoot,
        [string]$JLinkRoot
    )

    $sdk = Find-NcsSdkRoot -RequestedRoot $SdkRoot
    $toolchain = Find-NcsToolchainRoot -SdkRoot $sdk -RequestedRoot $ToolchainRoot
    $jlink = Find-JLinkRoot -RequestedRoot $JLinkRoot

    $prependPaths = @(
        $toolchain,
        (Join-Path $toolchain 'mingw64\bin'),
        (Join-Path $toolchain 'bin'),
        (Join-Path $toolchain 'opt\bin'),
        (Join-Path $toolchain 'opt\bin\Scripts'),
        (Join-Path $toolchain 'opt\nanopb\generator-bin'),
        (Join-Path $toolchain 'nrfutil\bin'),
        (Join-Path $toolchain 'opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin'),
        (Join-Path $toolchain 'opt\zephyr-sdk\gnu\riscv64-zephyr-elf\bin')
    )
    if ($null -ne $jlink) {
        $prependPaths += $jlink
    }

    $env:PATH = (($prependPaths + @($env:PATH)) -join ';')
    $env:PYTHONPATH = @(
        (Join-Path $toolchain 'opt\bin'),
        (Join-Path $toolchain 'opt\bin\Lib'),
        (Join-Path $toolchain 'opt\bin\Lib\site-packages')
    ) -join ';'
    $env:NRFUTIL_HOME = Join-Path $toolchain 'nrfutil\home'
    $env:ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr/gnu'
    $env:ZEPHYR_SDK_INSTALL_DIR = Join-Path $toolchain 'opt\zephyr-sdk'
    $env:ZEPHYR_BASE = Join-Path $sdk 'zephyr'

    $west = Join-Path $toolchain 'opt\bin\Scripts\west.exe'
    & $west --version | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "The west executable in '$toolchain' could not start."
    }

    return [pscustomobject]@{
        SdkRoot = $sdk
        ToolchainRoot = $toolchain
        JLinkRoot = $jlink
        West = $west
    }
}
