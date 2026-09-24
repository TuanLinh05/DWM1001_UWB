<#
.SYNOPSIS
    Host test suite for the DWM1001 firmware: no hardware, no NCS, no Zephyr.

.DESCRIPTION
    Every C test compiles the production sources with gcc and drives them
    through the register-level DW1000 simulator (tests\dw1000_sim.c), so the
    real state machines, the DS-TWR math, the command executor and the
    telemetry encoders run on the PC. The Python tests keep the GUI decoder,
    the sniffer tool and the project layout in step with the firmware.

    Requires gcc (MinGW-w64) and Python 3 on PATH. Run before every flash:
        powershell -ExecutionPolicy Bypass -File Firmware\tests\run_host_tests.ps1

.PARAMETER Filter
    Wildcard over test names, e.g. -Filter 'test_tag*' to run one group.

.PARAMETER SkipGateway
    Skip the ESP32-C3 gateway parser test (it also only needs gcc).
#>

[CmdletBinding()]
param(
    [string]$Filter = '*',
    [switch]$SkipGateway
)

$ErrorActionPreference = 'Stop'

$firmwareRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $firmwareRoot
$commonSource = Join-Path $firmwareRoot 'common\src'
$guiTests = Join-Path $repoRoot 'Software\UWB_UART_GUI\tests'
$outputDirectory = Join-Path ([IO.Path]::GetTempPath()) 'uwb-firmware-host-tests'
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

$gcc = (Get-Command gcc -ErrorAction SilentlyContinue)
if ($null -eq $gcc) { throw 'gcc was not found on PATH (install MinGW-w64).' }

$pythonCommand = Get-Command py -ErrorAction SilentlyContinue
$pythonPrefix = @('-3')
if ($null -eq $pythonCommand) {
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    $pythonPrefix = @()
}
if ($null -eq $pythonCommand) { throw 'Python 3 was not found on PATH.' }

$script:passed = @()

function Invoke-Step {
    param([string]$Name, [scriptblock]$Body)

    if ($Name -notlike $Filter) { return }
    Write-Host "== $Name" -ForegroundColor Cyan
    & $Body
    $script:passed += $Name
}

function Invoke-Tool {
    param([string]$Path, [string[]]$Arguments, [string]$Message)

    # Out-Host: keep the tool's output on the console instead of letting it
    # leak into the return value of the caller (Build-CTest returns a path).
    & $Path @Arguments | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Message (exit $LASTEXITCODE)" }
}

function Invoke-Python {
    param([string[]]$Arguments, [string]$Message)

    Invoke-Tool -Path $pythonCommand.Source -Arguments ($pythonPrefix + $Arguments) -Message $Message
}

# Each C test compiles its own sources; the ranging modules are #included by
# the tests themselves, which is why they are not listed here. 'sim' is the
# DW1000 register simulator, 'Config' picks the node's uwb_app_config.h.
$cTests = @(
    @{ Name = 'test_driver'; Role = 'TAG'; Config = 'Tag'
       Sources = @('sim', 'drivers\dw1000.c') }
    @{ Name = 'test_uwb_frame'; Role = 'TAG'; Config = 'Tag'
       Sources = @('ranging\uwb_frame.c') }
    @{ Name = 'test_tag_state'; Role = 'TAG'; Config = 'Tag'
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c', 'filters\range_filter.c') }
    @{ Name = 'test_anchor_state'; Role = 'ANCHOR'; Config = 'Anchor_1'
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c') }
    # No role: the parser half of uwb_cmd.c must build without the executor.
    @{ Name = 'test_cmd_parser'; Role = ''; Config = 'Tag'
       Sources = @('app\uwb_cmd.c', 'telemetry\telemetry_frame.c') }
    @{ Name = 'test_cmd_executor'; Role = 'TAG'; Config = 'Tag'
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c', 'filters\range_filter.c',
                    'app\uwb_cmd.c', 'telemetry\telemetry.c', 'telemetry\telemetry_frame.c') }
    @{ Name = 'test_settings'; Role = 'TAG'; Config = 'Tag'
       Sources = @('app\uwb_settings.c') }
    # No role either: the encoders are role independent, and leaving the
    # executor out of uwb_cmd.c keeps the golden binary free of Zephyr stubs.
    @{ Name = 'test_telemetry_golden'; Role = ''; Config = 'Tag'
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c', 'filters\range_filter.c',
                   'app\uwb_cmd.c', 'telemetry\telemetry.c', 'telemetry\telemetry_frame.c')
       Arguments = @((Join-Path $outputDirectory 'golden.bin')) }
)

function Build-CTest {
    param([hashtable]$Test)

    $binary = Join-Path $outputDirectory "$($Test.Name).exe"
    $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-Wshadow', '-O1')
    if ($Test.Role) { $arguments += "-DUWB_ROLE_$($Test.Role)" }
    foreach ($include in @((Join-Path $firmwareRoot 'common\include'),
                           (Join-Path $firmwareRoot "$($Test.Config)\include"),
                           $PSScriptRoot)) {
        $arguments += @('-I', $include)
    }
    $arguments += (Join-Path $PSScriptRoot "$($Test.Name).c")
    foreach ($source in $Test.Sources) {
        if ($source -eq 'sim') {
            $arguments += (Join-Path $PSScriptRoot 'dw1000_sim.c')
        } else {
            $arguments += (Join-Path $commonSource $source)
        }
    }
    $arguments += @('-lm', '-o', $binary)
    Invoke-Tool -Path $gcc.Source -Arguments $arguments -Message "Compile failed: $($Test.Name)"
    return $binary
}

foreach ($test in $cTests) {
    $current = $test
    Invoke-Step -Name $current.Name -Body {
        $binary = Build-CTest -Test $current
        $runArguments = @()
        if ($current.ContainsKey('Arguments')) { $runArguments = $current.Arguments }
        Invoke-Tool -Path $binary -Arguments $runArguments -Message "Test failed: $($current.Name)"
    }
}

# The golden file the C encoders just produced must decode with the GUI's
# Python decoder, field by field: that is the C <-> host contract.
Invoke-Step -Name 'test_telemetry_golden.py' -Body {
    $golden = Join-Path $outputDirectory 'golden.bin'
    if (-not (Test-Path $golden)) {
        $binary = Build-CTest -Test ($cTests | Where-Object { $_.Name -eq 'test_telemetry_golden' })
        Invoke-Tool -Path $binary -Arguments @($golden) -Message 'Golden generator failed'
    }
    Invoke-Python -Arguments @((Join-Path $PSScriptRoot 'test_telemetry_golden.py'), $golden) `
                  -Message 'Golden telemetry cross-check failed'
}

Invoke-Step -Name 'test_node_projects.py' -Body {
    Invoke-Python -Arguments @((Join-Path $PSScriptRoot 'test_node_projects.py')) `
                  -Message 'Node project validation failed'
}

Invoke-Step -Name 'test_sniffer_tool.py' -Body {
    Invoke-Python -Arguments @((Join-Path $PSScriptRoot 'test_sniffer_tool.py')) `
                  -Message 'Sniffer tool test failed'
}

Invoke-Step -Name 'gui_python_tests' -Body {
    Invoke-Python -Arguments @('-m', 'unittest', 'discover', '-s', $guiTests, '-t', $guiTests) `
                  -Message 'GUI/protocol Python tests failed'
}

if (-not $SkipGateway) {
    Invoke-Step -Name 'gateway_parser' -Body {
        & (Join-Path $firmwareRoot 'ESP32C3_Gateway\tests\run_host_test.ps1')
        if ($LASTEXITCODE -ne 0) { throw 'ESP32-C3 gateway parser test failed.' }
    }
}

Write-Host ""
Write-Host "host tests passed: $($script:passed.Count)" -ForegroundColor Green
$script:passed | ForEach-Object { Write-Host "  - $_" }
