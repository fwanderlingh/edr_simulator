<#
Student launcher: downloads a private runtime on first use, then runs offline.
No global Python installation, shell activation, or administrator access required.
#>
[CmdletBinding()]
param(
    [ValidateSet(1, 2, 3, 4, 5, 8)][int]$Lesson = 1,
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
# 2.6+ uses a deep channel/platform cache hierarchy that can exceed Windows
# path limits. Keep this versioned so existing installs also receive the fix.
$mambaRelease = '2.5.0-2'
$mambaExecutable = Join-Path $toolsDirectory "Library/bin/micromamba-$mambaRelease.exe"
$env:MAMBA_ROOT_PREFIX = Join-Path $simulatorRoot '.mamba'
# Bypass incomplete hierarchical caches left by the previous launcher. Also
# isolate downloads/cleanup from any global Conda package cache configuration.
$env:CONDA_PKGS_DIRS = Join-Path $env:MAMBA_ROOT_PREFIX 'pkgs-flat'
$env:PYTHONNOUSERSITE = '1'
$env:CONDA_PREFIX = $runtimeDirectory
$env:PATH = "$runtimeDirectory;$(Join-Path $runtimeDirectory 'Library/bin');$(Join-Path $runtimeDirectory 'Scripts');$env:PATH"
$pythonExecutable = Join-Path $runtimeDirectory 'python.exe'

function Get-LocalSha256 {
    param([string]$Path)
    # Use .NET directly: Get-FileHash is unavailable in some student shells.
    $sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        $stream = [IO.File]::OpenRead($Path)
        try {
            return [BitConverter]::ToString($sha256.ComputeHash($stream)).Replace('-', '')
        } finally {
            $stream.Dispose()
        }
    } finally {
        $sha256.Dispose()
    }
}

function Initialize-WindowsLibraries {
    # The Windows micromamba build dynamically links the VC runtime. Deploy
    # private copies before its first invocation, including on existing installs.
    $bootstrapDirectory = Join-Path $simulatorRoot 'bootstrap/win-64'
    $libraryHashes = @{
        'msvcp140.dll' = '7C26614E1D733892C2DEAC7E245CE115504B1D80592DD0A01B08E3E5A55F89CA'
        'vcruntime140.dll' = 'D1F4225DF2CD877DBF130D5668A021DCE3F94118455FF5EC952061C30AFC9CE7'
        'vcruntime140_1.dll' = 'A7146C08F89FE5B04541AB507CDB59FF7B44534D4BA3C668A426C6450A03434E'
    }
    foreach ($name in $libraryHashes.Keys) {
        $source = Join-Path $bootstrapDirectory $name
        if (-not (Test-Path -LiteralPath $source)) {
            throw "Missing bootstrap library: $source. Copy the complete bootstrap folder from the updated simulator distribution."
        }
        if ((Get-LocalSha256 $source) -ne $libraryHashes[$name]) {
            throw "Bootstrap library failed its integrity check: $source. Replace the bootstrap folder from the simulator distribution."
        }
    }
    $managerDirectory = Split-Path -Parent $mambaExecutable
    New-Item -ItemType Directory -Force -Path $managerDirectory | Out-Null
    foreach ($name in $libraryHashes.Keys) {
        $destination = Join-Path $managerDirectory $name
        if ((Test-Path -LiteralPath $destination) -and
            (Get-LocalSha256 $destination) -eq $libraryHashes[$name]) { continue }
        Copy-Item -LiteralPath (Join-Path $bootstrapDirectory $name) -Destination $destination -Force
    }
}

function Initialize-EnvironmentManager {
    Initialize-WindowsLibraries
    if (-not (Test-Path -LiteralPath $mambaExecutable)) {
        Write-Host 'First-time setup: downloading the local environment manager.'
        Write-Host 'An internet connection is required for setup. Nothing is installed globally.'
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $mambaExecutable) | Out-Null
        $partialDownload = "$mambaExecutable.download"
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -UseBasicParsing -Uri "https://github.com/mamba-org/micromamba-releases/releases/download/$mambaRelease/micromamba-win-64.exe" -OutFile $partialDownload
        $expectedHash = 'BAF6D56A31A63A75F0C77BDCE27FEFEA57B1BDC8AA25B50A1BE45696C0E737E7'
        if ((Get-LocalSha256 $partialDownload) -ne $expectedHash) {
            throw 'The environment manager download failed its integrity check. Rerun this launcher to download it again.'
        }
        Move-Item -LiteralPath $partialDownload -Destination $mambaExecutable -Force
    }
}

function Test-Runtime {
    if (-not (Test-Path -LiteralPath $pythonExecutable)) { return $false }
    & $pythonExecutable -c 'import sys, numpy, pybullet, tkinter; from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg; assert sys.version_info[:2] == (3, 12)' | Out-Host
    return $LASTEXITCODE -eq 0
}

function Clear-PackageCache {
    & $mambaExecutable clean --all --yes --no-rc | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Warning 'Setup succeeded, but the downloaded package cache could not be cleaned. You can still run the simulator.'
    }
}

try {
    $runtimeReady = Test-Runtime
    if (-not $runtimeReady) {
        Initialize-EnvironmentManager
        Write-Host 'Setting up Python 3.12, the physics engine and plotting tools. This may take a few minutes.'
        $environmentFile = Join-Path $simulatorRoot 'environment-win-64.lock'
        if (-not (Test-Path -LiteralPath $environmentFile)) {
            $environmentFile = Join-Path $simulatorRoot 'environment.yml'
        }
        $environmentCommand = if (Test-Path -LiteralPath $pythonExecutable) { 'install' } else { 'create' }
        # A failed extraction can leave package records present but files missing.
        # Reinstall those packages when repairing an unusable existing runtime.
        $repairArguments = if ($environmentCommand -eq 'install') { @('--force-reinstall') } else { @() }
        & $mambaExecutable $environmentCommand --yes --no-rc --prefix $runtimeDirectory --file $environmentFile @repairArguments
        if ($LASTEXITCODE -ne 0) {
            throw 'Environment setup failed. Read the libmamba error above. For cache/path errors, close other launchers and retry from a short writable path (for example C:\robotica). For download errors, check your connection. See README.md: Windows setup errors.'
        }
        if (-not (Test-Runtime)) { throw 'The Python runtime is incomplete. Rerun this launcher to retry setup.' }
        Clear-PackageCache
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
                Initialize-EnvironmentManager
                & $mambaExecutable install --yes --no-rc --prefix $runtimeDirectory 'pytest>=8,<9'
                if ($LASTEXITCODE -ne 0) { throw 'Could not install pytest.' }
                Clear-PackageCache
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
