<#
Student launcher: downloads a private runtime on first use, then runs offline.
No global Python installation, shell activation, or administrator access required.
#>
[CmdletBinding()]
param(
    [ValidateSet(1, 2, 3, 4)][int]$Lesson = 1,
    [switch]$Headless,
    [double]$Duration = 16,
    [double]$Dt = 1.0 / 240.0,
    [string]$Csv,
    [switch]$SetupOnly,
    [switch]$Test
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$simulatorRoot = $PSScriptRoot
$toolsDirectory = Join-Path $simulatorRoot '.tools'
$runtimeDirectory = Join-Path $simulatorRoot '.conda'
$mambaExecutable = Join-Path $toolsDirectory 'Library/bin/micromamba.exe'
$env:MAMBA_ROOT_PREFIX = Join-Path $simulatorRoot '.mamba'
$env:PYTHONNOUSERSITE = '1'
$env:CONDA_PREFIX = $runtimeDirectory
$env:PATH = "$runtimeDirectory;$(Join-Path $runtimeDirectory 'Library/bin');$(Join-Path $runtimeDirectory 'Scripts');$env:PATH"
$pythonExecutable = Join-Path $runtimeDirectory 'python.exe'

try {
    if (-not (Test-Path -LiteralPath $mambaExecutable)) {
        Write-Host 'First-time setup: downloading the local environment manager.'
        Write-Host 'An internet connection is required for setup. Nothing is installed globally.'
        New-Item -ItemType Directory -Force -Path $toolsDirectory | Out-Null
        $archive = Join-Path $toolsDirectory 'micromamba.tar.bz2'
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -UseBasicParsing -Uri 'https://micro.mamba.pm/api/micromamba/win-64/latest' -OutFile $archive
        & tar.exe -xjf $archive -C $toolsDirectory Library/bin/micromamba.exe
        if ($LASTEXITCODE -ne 0) { throw 'Could not extract the runtime manager. Windows tar.exe is required.' }
    }

    $runtimeReady = $false
    if (Test-Path -LiteralPath $pythonExecutable) {
        & $pythonExecutable -c 'import sys, numpy, pybullet, tkinter; assert sys.version_info[:2] == (3, 12)'
        $runtimeReady = $LASTEXITCODE -eq 0
    }
    if (-not $runtimeReady) {
        Write-Host 'Setting up Python 3.12 and the precompiled physics engine. This may take a few minutes.'
        $environmentFile = Join-Path $simulatorRoot 'environment-win-64.lock'
        if (-not (Test-Path -LiteralPath $environmentFile)) {
            $environmentFile = Join-Path $simulatorRoot 'environment.yml'
        }
        & $mambaExecutable create --yes --no-rc --prefix $runtimeDirectory --file $environmentFile
        if ($LASTEXITCODE -ne 0) { throw 'Environment setup failed. Check your connection and rerun this launcher.' }
        & $mambaExecutable clean --all --yes --no-rc | Out-Null
    }
    if ($SetupOnly) {
        Write-Host 'Setup complete. Double-click start-simulator.cmd to open the laboratory.'
        exit 0
    }

    Push-Location -LiteralPath $simulatorRoot
    try {
        if ($Test) {
            & $pythonExecutable -c 'import importlib.util, sys; sys.exit(importlib.util.find_spec(\"pytest\") is None)'
            if ($LASTEXITCODE -ne 0) {
                & $mambaExecutable install --yes --no-rc --prefix $runtimeDirectory 'pytest>=8,<9'
                if ($LASTEXITCODE -ne 0) { throw 'Could not install pytest.' }
                & $mambaExecutable clean --all --yes --no-rc | Out-Null
            }
            & $pythonExecutable -m pytest tests
        } else {
            $culture = [Globalization.CultureInfo]::InvariantCulture
            $simulationArguments = @('-m', 'robotics_sim', '--dt', $Dt.ToString($culture))
            if ($PSBoundParameters.ContainsKey('Lesson')) { $simulationArguments += @('--lesson', "$Lesson") }
            if ($PSBoundParameters.ContainsKey('Duration')) { $simulationArguments += @('--duration', $Duration.ToString($culture)) }
            if ($Headless) { $simulationArguments += '--headless' }
            if ($Csv) { $simulationArguments += @('--csv', $Csv) }
            & $pythonExecutable @simulationArguments
        }
        $simulationExitCode = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    exit $simulationExitCode
} catch {
    Write-Host "Simulator error: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
